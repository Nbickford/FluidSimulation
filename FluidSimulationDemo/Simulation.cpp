//*********************************************************
// Simulation.cpp
// Top level of code for doing fluid simulation on the CPU.
//
// Authors:
//   Neil Bickford
//*********************************************************

#include "Simulation.h"
#include "odprintf.h"
#include <limits>
#include <queue> // For serial extrapolation - see ExtrapolateValues(4).

#include "debugroutines.h"

int frame = 0;

FluidSim::FluidSim(int xSize, int ySize, float CellsPerMeter)
	:mX(xSize), mY(ySize), m_CellsPerMeter(CellsPerMeter),
	m_particles() {
	// Set up MAC velocity grids and initialize velocities
	m_MU = new float[(xSize + 1)*ySize];
	m_MV = new float[xSize*(ySize + 1)];

	ResetSimulation();
}

FluidSim::~FluidSim() {
	m_particles.clear();
	delete[] m_MU;
	delete[] m_MV;
}

void FluidSim::ResetSimulation() {
	// TODO: the scale on this for vectorCurl is incorrect
	// MU
	for (int y = 0; y < mY; y++) {
		for (int x = 0; x < mX + 1; x++) {
			XMFLOAT2 vc = vectorCurl(x / m_CellsPerMeter, (y + 0.5f) / m_CellsPerMeter);
			U(x, y) = vc.x;
		}
	}

	// MV
	for (int y = 0; y < mY + 1; y++) {
		for (int x = 0; x < mX; x++) {
			XMFLOAT2 vc = vectorCurl((x + 0.5f) / m_CellsPerMeter, y / m_CellsPerMeter);
			V(x, y) = vc.y;
		}
	}

	// Create a uniform distribution of particles
	// and set their velocities
	m_particles.clear();
	for (int y = 1; y < mY-1; y++) {
		for (int x = 1; x < mX-1; x++) {
			float px = x - 0.25f;
			float py = y - 0.25f;
			float rX = px / m_CellsPerMeter; //real-world X
			float rY = py / m_CellsPerMeter;
			float d = 0.5f / m_CellsPerMeter;
			m_particles.emplace_back(rX, rY, InterpolateMACCell(mX*rX,mY*rY));
			m_particles.emplace_back(rX + d, rY, InterpolateMACCell(mX*(rX+d), mY*rY));
			m_particles.emplace_back(rX, rY + d, InterpolateMACCell(mX*rX, mY*(rY+d)));
			m_particles.emplace_back(rX + d, rY + d, InterpolateMACCell(mX*(rX+d), mY*(rY+d)));
		}
	}

	// Remove initial divergence
	Project(1.0);

	// Print divergence
	PrintDivergence();
}

void FluidSim::PrintDivergence() {
	bool pAr = false;
	float maxdiv = 0;
	int maxdivX = -1;
	int maxdivY = -1;
	if (pAr) odprintf("{");
	for (int y = 0; y < mY; y++) {
		if (pAr) odprintf("{");
		for (int x = 0; x < mX; x++) {
			float div = U(x + 1, y) + V(x, y + 1) - U(x, y) - V(x, y);
			if (abs(div) > abs(maxdiv)) {
				maxdiv = div;
				maxdivX = x;
				maxdivY = y;
			}
			if (pAr) odprintf("%f", div);
			if (x != mX - 1) {
				if (pAr) odprintf(",");
			}
		}
		if (pAr) odprintf("}");
		if (y != mY - 1) {
			if (pAr) odprintf(",\n");
		}
	}
	if (pAr) odprintf("}");
	odprintf("Maximum divergence of %f is at (x,y)=(%i, %i)", maxdiv, maxdivX, maxdivY);
}

void FluidSim::Simulate(float dt) {
	// Clamp maximum dt
	dt = MathHelper::Clamp(dt, 0.0f, 1.0f/15.0f);

	// Iterate frame counter
	frame++;
	
	// QQQ DEBUG: Set dt to 0.01 to match simple.cpp
	dt = 0.01f;

	Advect(m_particles, dt);

	// Moving complexity here for the moment, this should be refactored out
	// Store the old grid (...this actually does seem to be the best way)
	float* m_oldU = new float[(mX + 1)*mY];
	float* m_oldV = new float[mX*(mY + 1)];
	for (int i = 0; i < (mX + 1)*mY; i++)
		m_oldU[i] = m_MU[i];
	for (int i = 0; i < mX*(mY + 1); i++) {
		m_oldV[i] = m_MV[i];
	}

	// In this step, we also do a hybrid FLIP/PIC step to update the new particle velocities.
	// Letting alpha be 6*dt*m_nu*m_CellsPerMeter^2 (pg. 118),
	float alpha = MathHelper::Clamp(6 * dt*m_nu*m_CellsPerMeter*m_CellsPerMeter, 0.0f, 1.0f);
	//alpha = 0.3f; // can only go down to 0.3 on a 64x64 grid for now (due to lack of velocity interpolation code?)
	// Well, I implemented the velocity interpolation code, and it's still breaking.
	// Time to check our results against a working implementation!
	// Turns out that implementation also breaks on this, but in a different way.
	// It now looks like the problem's that the projection routine changes its results dramatically depending
	// on which cells are classified as air - or in this case, which cells have extrapolated results (which of course
	// will conflict with the boundary conditions).
	// There are three things that might help fix this:
	// 1. Implementing the case of cells containing air in the fluid solver. (If we have enough air, the projection
	// routine might just leave enough of the velocities alone.)
	// 2. Particle reseeding.
	// 3. (Experimental) Fractional volumes of air depending on level sets.
	// 4. Just clamp particle velocities/positions :/
	alpha = 0.14f; // On a 32x32 grid with a dt of 0.01, we can now go down to 0.14.
				 // Ex. For a 64x64 grid with a dt of 1/60, this is 0.0003645, since m_nu is so small (~10^-6)

	TransferParticlesToGrid(m_particles);

	// Add gravity
	// AddBodyForces(dt);

	Project(dt);

	PrintDivergence();

	//---------------------------------------
	// FINISH SETTING NEW PARTICLE VELOCITIES
	//---------------------------------------
	// Compute difference between new grid and old grid
	// At the end, we want to have
	// u^new = alpha*newgrid + (1-alpha)*u_old + (1-alpha)*(newgrid-oldgrid)
	//       = (1-alpha)*u_old + newgrid-(1-alpha)*oldgrid
	for (int i = 0; i < (mX + 1)*mY; i++)
		m_oldU[i] = m_MU[i]-(1.0f-alpha)*m_oldU[i];
	for (int i = 0; i < mX*(mY + 1); i++) {
		m_oldV[i] = m_MV[i]-(1.0f-alpha)*m_oldV[i];
	}
	// odprintf("diff1 %f", ComputeL2Norm(m_oldU, (mX+1), mY));
	// odprintf("diff2 %f", ComputeL2Norm(m_oldU, mX, (mY+1)));

	// Allow for interpolation via tricky pointer hacking
	std::swap(m_oldU, m_MU);
	std::swap(m_oldV, m_MV);

	int len = (int)m_particles.size();
	for (int i = 0; i < len; i++) {
		XMFLOAT2 interpDiff = InterpolateMACCell(mX*m_particles[i].X, mY*m_particles[i].Y);
		if (fabsf(interpDiff.x) > 1000 || fabsf(interpDiff.y) > 1000) {
			odprintf("Hey, what's going on here?");
		}
		m_particles[i].uX = (1.0f-alpha)*m_particles[i].uX + interpDiff.x;
		m_particles[i].uY = (1.0f-alpha)*m_particles[i].uY + interpDiff.y;
	}
	// ...and swap back
	std::swap(m_oldU, m_MU);
	std::swap(m_oldV, m_MV);

	// ...and clean up :(
	delete[] m_oldU;
	delete[] m_oldV;
}

void FluidSim::Advect(std::vector<Particle> &particles, float dt) {
	// Tracing particle paths and velocities using an RK3 method
	// [Bridson, pg. 109-110], based on [Ralston, 1962].
	// For the velocity calculation, suppose a particle moves with velocity (ux, uy) m/s.
	// Then the particle moves (ux,uy)*dt meters in dt seconds, which we can invert.

	// Set to false to use interpolate and the computed MAC grids
	int len = (int)particles.size();
	if (false) {
		for (int i = 0; i < len; i++) {
			XMFLOAT2 k1 = vectorCurl(particles[i].X, particles[i].Y);
			XMFLOAT2 k2 = vectorCurl(particles[i].X + 0.5f*dt*k1.x, particles[i].Y + 0.5f*dt*k1.y);
			XMFLOAT2 k3 = vectorCurl(particles[i].X + 0.75f*dt*k2.x, particles[i].Y + 0.75f*dt*k2.y);

			particles[i].uX = (2.0f / 9.0f)*k1.x + (3.0f / 9.0f)*k2.x + (4.0f / 9.0f)*k3.x;
			particles[i].uY = (2.0f / 9.0f)*k1.y + (3.0f / 9.0f)*k2.y + (4.0f / 9.0f)*k3.y;
			particles[i].X += dt*particles[i].uX;
			particles[i].Y += dt*particles[i].uY;
		}

	} else {
		for (int i = 0; i < len; i++) {
			// This was surprisingly difficult to find, but according to an article in GPU Gems 5
			// (source: https://books.google.com/books?id=uIDSBQAAQBAJ&pg=PA60&lpg=PA60&dq=using+flip+with+runge-kutta&source=bl&ots=XMJL03mmLe&sig=ZVlwK4HpjKePBfpq9ew1T4zeuUI&hl=en&sa=X&ved=0ahUKEwiWtcjpgdfZAhVIYK0KHVz1BVsQ6AEIeTAI#v=onepage&q=using%20flip%20with%20runge-kutta&f=false)
			// (search key: "using FLIP with runge-kutta")
			// we should actually update the particle's position entirely using interpolation on the grid,
			// and only update the particle's velocity using the grid-update step in FLIP.
			XMFLOAT2 k1 = InterpolateMACCell(mX*particles[i].X, mY*particles[i].Y); // should actually be cellsPerMeter?
			XMFLOAT2 k2 = InterpolateMACCell(mX*(particles[i].X + 0.5f*dt*k1.x), mY*(particles[i].Y + 0.5f*dt*k1.y));
			XMFLOAT2 k3 = InterpolateMACCell(mX*(particles[i].X + 0.75f*dt*k2.x), mY*(particles[i].Y + 0.75f*dt*k2.y));

			float vX = (2.0f / 9.0f)*k1.x + (3.0f / 9.0f)*k2.x + (4.0f / 9.0f)*k3.x;
			float vY = (2.0f / 9.0f)*k1.y + (3.0f / 9.0f)*k2.y + (4.0f / 9.0f)*k3.y;
			particles[i].X += dt*vX;
			particles[i].Y += dt*vY;
		}
	}
}

void FluidSim::TransferParticlesToGrid(std::vector<Particle> &particles) {
	// VELOCITY UPDATES
	// Implements Bridson's equation 7.2 with a trilinear hat kernel (pg. 112) for transferring particle velocities
	// to the grid, along with a hybrid FLIP/PIC update.
	//-------------------------------------
	// TRANSFER PARTICLE VELOCITIES TO GRID
	//-------------------------------------
	int len = (int)particles.size();
	// Accumulates weights
	float* uAmts = new float[(mX + 1)*mY](); // sets to 0, hopefully
	float* vAmts = new float[mX*(mY + 1)]();

	// Clear u and v velocity arrays
	for (int y = 0; y < mY; y++) {
		for (int x = 0; x < mX + 1; x++) {
			U(x, y) = 0.0f;
		}
	}
	for (int y = 0; y < mY + 1; y++) {
		for (int x = 0; x < mX; x++) {
			V(x, y) = 0.0f;
		}
	}

	for (int i = 0; i < len; i++) {
		// We have to assume that particles can be outside the grid for the moment :(
		float px = particles[i].X*m_CellsPerMeter;
		float py = particles[i].Y*m_CellsPerMeter;

		if (px<-0.5 || px>(mX + 0.5) || py<-0.5 || py>(mY + 0.5)) {
			continue; // out of bounds, sorry
		}
		// ensured: -0.5<px<mX+0.5 and -0.5<py<mY+0.5

		// Remember, the centers of squares are actually integers, not half-integers.
		// U
		// affects values: y=floor(py) and floor(py)+1
		//                 x=floor(px+1/2)-1/2 and floor(px+1/2)+1/2 ...though for indices we add 1/2 to each of those
		int iy = (int)floorf(py);
		int ix = (int)floorf(px + 0.5f);
		// Computing alpha for y and x
		float alphay = py - (float)iy;
		float alphax = (px + 0.5f) - (float)ix;
		// Assigning values and weights
		for (int y = 0; y <= 1; y++) {
			for (int x = 0; x <= 1; x++) {
				if ((ix + x) <= mX && 0 <= (iy + y) && (iy + y) < mY) {
					float w = (x > 0 ? alphax : 1.f - alphax)*(y > 0 ? alphay : 1.f - alphay);
					U(ix + x, iy + y) += w*particles[i].uX;
					uAmts[(ix + x) + (mX + 1)*(iy + y)] += w;
				}
			}
		}

		// V
		// affects values: x=floor(px) and floor(px)+1
		//                 y=floor(py+1/2)-1/2 and floor(py+1/2)+1/2, with same 1/2 index bias as above.
		ix = (int)floorf(px);
		iy = (int)floorf(py + 0.5f);
		// Computing alpha for x and y
		alphax = px - (float)ix;
		alphay = (py + 0.5f) - (float)iy;
		// Assigning values and weights
		for (int y = 0; y <= 1; y++) {
			for (int x = 0; x <= 1; x++) {
				if ((iy + y) <= mY && 0 <= (ix + x) && (ix + x) < mX) {
					float w = (x > 0 ? alphax : 1.f - alphax)*(y > 0 ? alphay : 1.f - alphay);
					V(ix + x, iy + y) += w*particles[i].uY;
					vAmts[(ix + x) + mX*(iy + y)] += w;
				}
			}
		}
	}

	float minAmt = 100.0f;

	// Divide U and V by uAmts and vAmts
	float div_eps = std::numeric_limits<float>::denorm_min();
	//div_eps = 1.3f;
	for (int y = 0; y < mY; y++) {
		for (int x = 0; x < mX + 1; x++) {
			U(x, y) = (U(x, y) / (div_eps + uAmts[x + (mX + 1)*y]));
			if (uAmts[x + (mX + 1)*y] < minAmt)
				minAmt = uAmts[x + (mX + 1)*y];
		}
	}
	for (int y = 0; y < mY + 1; y++) {
		for (int x = 0; x < mX; x++) {
			V(x, y) = (V(x, y) / (div_eps + vAmts[x + mX*y]));
			if (vAmts[x + mX*y] < minAmt)
				minAmt = vAmts[x + mX*y];
		}
	}

	// EXTRAPOLATE UNKNOWN VELOCITIES
	// Fixed: We know the velocities of U and V at the edges as well.
	float zero_thresh = 0.01f;
	bool* uValid = new bool[(mX + 1)*mY];
	bool* vValid = new bool[mX*(mY + 1)];
	for (int i = 0; i < (mX + 1)*mY; i++)
		uValid[i] = (uAmts[i] > zero_thresh);
	for (int i = 0; i < mX*(mY + 1); i++)
		vValid[i] = (vAmts[i] > zero_thresh);

	// Edges
	for (int y = 0; y < mY; y++) {
		uValid[0 + y*(mX + 1)] = true; U(0, y) = 0;
		uValid[mX + y*(mX + 1)] = true; U(mX, y) = 0;
	}
	for (int x = 0; x < mX; x++) {
		vValid[x + 0 * mX] = true; V(x, 0) = 0;
		vValid[x + mY*mX] = true; V(x, mY) = 0;
	}

	ExtrapolateValues(m_MU, uValid, mX + 1, mY);
	ExtrapolateValues(m_MV, vValid, mX, mY + 1);

	delete[] uValid;
	delete[] vValid;

	delete[] uAmts;
	delete[] vAmts;
}

void FluidSim::ExtrapolateValues(float* srcAr, bool* validAr, int xSize, int ySize) {
	// Simple breadth-first-search-based extrapolation routine based off of Bridson, end of Chapter 4.
	// Since this is BFS, it's not immediately amenable to parallel processing, other than in the usual way
	// which requires lots of synchronization.
	// However, when we're parallelizing this routine, we could potentially try the following approach:
	// 1. Do a fast scan over the grid to construct the Manhattan distance from each grid point to the closest
	//      valid grid point.
	// 2. Do some sort of hierarchical bucket sort to partition the cells into sets based off of their
	//      distances from step (1).
	// 3. Extrapolate the values for the grid by extrapolating the values at distance i in parallel
	
	// Note, however, that this won't produce the same results as the following serial algorithm, so we'll have
	// to modify this later on. The priority here is getting the code to work.

	// Precondition: At least one value of validAr is true.

	// Adjacency directions
	//const int offsets[8] = { 1,0, -1,0, 0,1, 0,-1 };
	//const int numDirs = 4;
	const int offsets[16] = { 1,1, 1,0, 1,-1, 0,1, 0,-1, -1,1, -1,0, -1,-1 };
	const int numDirs = 8;
	
	// BFS initialization
	std::queue<int> q;
	for (int y = 0; y < ySize; y++) {
		for (int x = 0; x < xSize; x++) {
			if (!validAr[x + xSize*y]) {
				// are any neighbors valid?
				for (int d = 0; d < numDirs; d++) {
					int nx = x+offsets[2 * d];
					int ny = y+offsets[2 * d + 1];
					if (0 <= nx && nx < xSize && 0 <= ny && ny < ySize && validAr[nx + xSize*ny]) {
						q.push(x);
						q.push(y);
						break;
					}
				}
			}
		}
	}

	// BFS iteration
	while (q.size() > 0) {
		int x = q.front();  q.pop();
		int y = q.front();  q.pop();
		// already processed?
		if (validAr[x + xSize*y]) {
			break;
		}
		// Interpolate value and add additional values
		float numNeighbors = 0.0f;
		float sumNeighbors = 0.0f;
		for (int d = 0; d < numDirs; d++) {
			int nx = x + offsets[2 * d];
			int ny = y + offsets[2 * d + 1];
			if (0 <= nx && nx < xSize && 0 <= ny && ny < ySize) {
				int i = nx + xSize*ny;
				if (validAr[i]) {
					sumNeighbors += srcAr[i]; // Interpolate value
					numNeighbors += 1.0f;
				} else {
					q.push(nx); // Add additional values
					q.push(ny);
				}
			}
		}
		assert(numNeighbors > 0.0f);
		srcAr[x + xSize*y] = sumNeighbors / numNeighbors;
		validAr[x + xSize*y] = true;
	}
	// and that's it!
}

void FluidSim::AddBodyForces(float dt) {
	// Just adds dt*g to the velocity field
	// We use Cartesian coordinates for our grids, hooray!
	int count = mX*(mY+1);
	float gdT = m_gY*dt; // in m/s
	for (int i = 0; i < count; ++i) {
		m_MV[i] += gdT;
	}
}

void odPrintArray(double* ar, int xSize, int ySize) {
	odprintf("{");
	for (int y = 0; y < ySize; y++) {
		odprintf("{");
		for (int x = 0; x < xSize; x++) {
			odprintf("%lf", ar[x+xSize*y]);
			if (x != xSize - 1) {
				odprintf(",");
			}
		}
		odprintf("}");
		if (y != ySize - 1) {
			odprintf(",\n");
		}
	}
	odprintf("}\n");
}

void FluidSim::Project(float dt) {
	//... this is basically Chapter 5 of Bridson.

	// At the heart of this projection routine, we need to solve a set of equations, of the form
	// 4 p_{i,j} - p_{i+1,j} - p_{i-1,j} - p_{i,j+1} - p_{i,j-1} = -rhs[i,j]
	// where rhs[i,j] is the negative discrete divergence at (i,j), times dx^2*rho/dt:
	// rhs[i,j]=-dx*rho/dt (u{i+1/2,j}-u{i-1/2,j}+v{i,j+1/2}-v{i,j-1/2}).

	// This equation is modified at the following way at the boundaries:
	// (see Bridson's lecture notes or the 2nd ed. of the book, pg. 76 for a derivation:)
	
	// - For each bordering solid cell, reduce the coefficient 4 by 1 and remove the reference
	// to that p. Also, use u_solid inside the solid matierial (which in this case we take to be 0)
	//
	// - For each bordering air cell, just remove the reference to that p.

	// We now solve this linear system with the Gauss-Seidel method, using a checkerboard update.
	// Our operations are:
	// x |-> Dx: divide by number of non-solid neighbors
	// x |-> Rx: replace x_{i,j} with -(sum(water neighbors' pressures)).

	// For now, this is simple enough that we can just write it inside a loop.

	// For Gauss-Seidel, we just do Jacobi's method, but we just only use one array of pressures.

	// I. Compute the right-hand side b.
	int MN = mX*mY;
	double* b = new double[MN];
	double* p = new double[MN](); // should zero everything for us

	// INDEXING: b[x,y] = b[x+mX*y].

	double solidVel = 0;
	double dx = 1.0 / m_CellsPerMeter;
	double scale = -dx*m_rho / dt;

	for (int y = 0; y < mY; y++) {
		for (int x = 0; x < mX; x++) {
			double velR = (x == mX - 1 ? solidVel : U(x + 1, y));
			double velL = (x == 0      ? solidVel : U(x    , y));
			double velU = (y == mY - 1 ? solidVel : V(x, y + 1));
			double velD = (y == 0      ? solidVel : V(x, y    ));
			b[x + mX*y] = scale*(velR + velU - velL - velD);
		}
	}

	// II. Gauss-Seidel iteration.
	// For writing simplicity, let's start by writing everything inline.

	// Based off of measurements from this code, we generally want to take
	// omega to be almost exactly 2-3.22133/mX, based on iterating
	// on 16x16, 32x32, and 64x64 grids for 30, 30, and 60 iterations, respectively.
	// (Note, interestingly, that in the last two cases this isn't even enough iterations
	// for boundary conditions to reach the edges of the grid, yet it still manages an
	// eventual convergence rate of 1/0.85!)

	double omega = 2 - 3.22133 / mX;

	int numIterations = 200;

	for (int iter = 0; iter < numIterations; iter++) {
		for (int stage = 0; stage <= 1; stage++) {
			for (int y = 0; y < mY; y++) {
				for (int x = 0; x < mX; x++) {
					// Checkerboard iteration
					if (((x + y) % 2) != stage) continue;

					// Gauss-Seidel update p[x,y].
					double numNeighbors = 0;
					double neighborMinusSum = 0;

					if (x != 0) {
						numNeighbors++;
						neighborMinusSum -= p[(x - 1) + mX*(y)];
					}
					if (x != mX - 1) {
						numNeighbors++;
						neighborMinusSum -= p[(x + 1) + mX*(y)];
					}
					if (y != 0) {
						numNeighbors++;
						neighborMinusSum -= p[x + mX*(y - 1)];
					}
					if (y != mY - 1) {
						numNeighbors++;
						neighborMinusSum -= p[x + mX*(y + 1)];
					}

					// Get ready for it... here it comes!
					// p[x + mX*y] = (b[x + mX*y] - neighborMinusSum) / numNeighbors;

					// Woah, it's successive over-relaxation!
					p[x + mX*y] = (1 - omega)*p[x + mX*y] + omega*(b[x + mX*y] - neighborMinusSum) / numNeighbors;
				}
			}
		}
	}
	
	// Compute and print L2 norm
	// Measure norm
	double l2Sq = 0;
	for (int y = 0; y < mY; y++) {
		for (int x = 0; x < mX; x++) {
			double numNeighbors = 0;
			double neighborMinusSum = 0;

			if (x != 0) {
				numNeighbors++;
				neighborMinusSum -= p[(x - 1) + mX*(y)];
			}
			if (x != mX - 1) {
				numNeighbors++;
				neighborMinusSum -= p[(x + 1) + mX*(y)];
			}
			if (y != 0) {
				numNeighbors++;
				neighborMinusSum -= p[x + mX*(y - 1)];
			}
			if (y != mY - 1) {
				numNeighbors++;
				neighborMinusSum -= p[x + mX*(y + 1)];
			}

			l2Sq += pow(numNeighbors*p[x + mX*y] + neighborMinusSum - b[x + mX*y], 2.0);
		}
	}

	odprintf("After %i iterations, the L2 norm was %lf.", numIterations, l2Sq);

	if (isnan(l2Sq)) {
		odprintf("And that's a problem!");
	}

	// Remove pressure from velocities
	// Pressure gradient update:
	// u_{i+1/2,j,k}^{n+1} = u_{i+1/2,j,k}-dt(p_{i+1,j,k}-p_{i,j,k})/(rho dx)

	// Edges
	for (int x = 0; x < mX; x++) {
		V(x, 0) = 0;
		V(x, mY) = 0;
	}

	for (int y = 0; y < mY; y++) {
		U(0, y) = 0;
		U(mX, y) = 0;
	}

	// Interior
	scale = dt / (m_rho*dx);
	// U
	for (int y = 0; y < mY; y++) {
		for (int x = 0; x < mX-1; x++){
			U(x + 1, y) = U(x + 1, y) - static_cast<float>(scale*(p[(x + 1) + mX*y] - p[x + mX*y]));
		}
	}
	// V
	for (int y = 0; y < mY - 1; y++) {
		for (int x = 0; x < mX; x++) {
			V(x, y + 1) = V(x, y + 1) - static_cast<float>(scale*(p[x + mX*(y + 1)] - p[x + mX*y]));
		}
	}
	// end (modifies internal state).

	delete[] b;
	delete[] p;
}

float peaks(float x, float y) {
	// See An Introduction to Optimization, 4th Edition, pg. 290
	return 3.0f*(1.0f - x)*(1.0f - x)*expf(-x*x - (y + 1.f)*(y + 1.f))
		- 10.f*(0.2f*x - x*x*x + y*y*y*y*y)*expf(-x*x - y*y)
		- expf(-(x + 1.0f)*(x + 1.0f) - y*y) / 3.0f;
	// return sinf(x) + sinf(y); // Simpler peaks function
}

// Computes the 2D vector field N at the point (x,y)
XMFLOAT2 vectorFunction(float x, float y) {
	// Here, our function consists of vectors pointing to the maxima of peaks (above)
	// Note that we don't normalize this, since we'll have potential problems with numerical
	// instability.
	// We just use really cheap finite differences here.
	float eps = 1e-5f; // Must be greater than 2^-23 ~ 1e-6.92
	float p0 = peaks(x, y);
	float dx = (peaks(x + eps, y) - p0) / eps;
	float dy = (peaks(x, y + eps) - p0) / eps;
	// Scale by a function to go to 0 at +-3
	float scale = 1.0;// max(0.0f, 1.0 - powf(max(abs(x), abs(y)) / 3.0f, 16.0f));
	return XMFLOAT2(dx*scale, dy*scale);
}

// Computes the vector potential psi in [Bridson, 2nd ed., p. 171]
// using N (vectorFunction) above.
XMFLOAT2 vectorPotential(float x, float y) {
	// We just change the envelope of the function so that it goes to 0 at +-3:
	return vectorFunction(x, y);
}

// Computes the curl of the vector potential at a given point (x,y).
// ...of course, we actually decided to just make this really ad-hoc just now, so ignore all the above.
XMFLOAT2 vectorCurl(float x, float y) {
	XMFLOAT2 vf = vectorFunction(6.0f*x-3.0f, 6.0f*y-3.0f);
	return XMFLOAT2(0.1f*vf.y, -0.1f*vf.x); // should hypothetically circle around peaks
}