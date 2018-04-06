//*********************************************************
// Simulation.cpp
// Top level of code for doing fluid simulation on the CPU.
//
// Authors:
//   Neil Bickford
//*********************************************************

#include "Simulation2D.h"
#include "odprintf.h"
#include <limits>
#include <queue> // For serial extrapolation - see ExtrapolateValues(4).
#include <random>
#include <algorithm> // for std::min

#include "debugroutines.h"

FluidSim::FluidSim(int xSize, int ySize, float CellsPerMeter)
	:mX(xSize), mY(ySize), m_CellsPerMeter(CellsPerMeter),
	m_particles() {
	// Set up MAC velocity grids and initialize velocities
	m_MU = new float[(xSize + 1)*ySize];
	m_MV = new float[xSize*(ySize + 1)];
	
	// Level sets and auxiliary fields
	m_Phi = new float[xSize*ySize];

	ResetSimulation();
}

FluidSim::~FluidSim() {
	m_particles.clear();
	delete[] m_MU;
	delete[] m_MV;
	delete[] m_Phi;
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

	std::default_random_engine generator(0);
	std::uniform_real_distribution<float> distribution(-0.25f, 0.25f);

	// Create a uniform distribution of particles
	// and set their velocities
	m_particles.clear();
	for (int y = 1; y < mY-1; y++) {
		for (int x = mX/2; x < mX-1; x++) {
			float px = x - 0.25f;
			float py = y - 0.25f;
			float rX = px / m_CellsPerMeter; //real-world X
			float rY = py / m_CellsPerMeter;
			float d = 0.5f / m_CellsPerMeter;
			for (int u = 0; u <= 1; u++) {
				for (int v = 0; v <= 1; v++) {
					float m1 = rX + u*d + distribution(generator)/m_CellsPerMeter;
					float m2 = rY + v*d + distribution(generator)/m_CellsPerMeter;
					m_particles.emplace_back(m1, m2, InterpolateMACCell(mX*m1, mY*m2));
				}
			}
		}
	}
}

void FluidSim::Simulate(float dt) {
	// Clamp maximum dt
	dt = MathHelper::Clamp(dt, 0.0f, 1.0f/15.0f);
	dt = 0.01f; // QQQ DEBUG Limit dt for debugging purposes

	// Iterate frame counter
	static int frame = 0;
	frame++;

	Advect(m_particles, dt);

	// In this step, we also do a hybrid FLIP/PIC step to update the new particle velocities.
	// Letting alpha be 6*dt*m_nu*m_CellsPerMeter^2 (pg. 118),
	float alpha = MathHelper::Clamp(6 * dt*m_nu*m_CellsPerMeter*m_CellsPerMeter, 0.0f, 1.0f);
				 // Ex. For a 64x64 grid with a dt of 1/60, this is 0.0003645, since m_nu is so small (~10^-6)
	ComputeLevelSet(m_particles);

	TransferParticlesToGrid(m_particles);

	// Moving complexity here for the moment, this should be refactored out
	// Store the old grid (...this actually does seem to be the best way)
	float* m_oldU = new float[(mX + 1)*mY];
	float* m_oldV = new float[mX*(mY + 1)];
	for (int i = 0; i < (mX + 1)*mY; i++)
		m_oldU[i] = m_MU[i];
	for (int i = 0; i < mX*(mY + 1); i++) {
		m_oldV[i] = m_MV[i];
	}

	// Add gravity
	AddBodyForces(dt);

	Project(dt);

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
		if (m_particles[i].uX > 100000) {
			m_particles[i].uX = 1000000;
			assert(false);
			odprintf("hey");
		}
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

		// Clamp to box, slightly inwards
		float eps = 0.1f;
		particles[i].X = MathHelper::Clamp(particles[i].X, (-0.5f + eps)/mX, 1.0f + (-0.5f - eps)/mX);
		particles[i].Y = MathHelper::Clamp(particles[i].Y, (-0.5f + eps)/mY, 1.0f + (-0.5f - eps) / mY);
	}
}


void FluidSim::clsInner(int dx, int dy, int x, int y,
	const std::vector<Particle> &particles,
	int* closestParticles,
	float* mPhi,
	float mPRadius, int mX, int mY) {

	int otherPt = closestParticles[(x + dx) + (y + dy)*mX];
	if (otherPt > 0) {
		float px = particles[otherPt].X*mX;
		float py = particles[otherPt].Y*mY;
		float dist = sqrtf((px - x)*(px - x) + (py - y)*(py - y)) - mPRadius;
		if (closestParticles[x + mX*y] < 0 || dist < mPhi[x + mX*y]) {
			closestParticles[x + mX*y] = otherPt;
			mPhi[x + mX*y] = dist;
		}
	}
}

void FluidSim::ComputeLevelSet(const std::vector<Particle> &particles) {
	// Uses fast sweeping to compute the level set Phi of the given grid,
	// using the kernel
	// sqrt((x-x0)^2+(y-y0)^2)-m_pRadius.

	// Bridson p. 126 says AKPG07 has a practical fast /marching/ method.
	// However, chapter 4.3 (pg 49-51) has pseudocode for the method we'll be using,
	// which is based on algorithm 4 in [Tsa02],
	// "Rapid and Accurate Computation of the Distance Function using Grids"
	// See also [JBS06].
	// In this code, we'll use the 2D method from Zhao's "A Fast Sweeping Method for Eikonal Equations",
	// which I found from Gomez' https://github.com/jvgomez/fast_methods.


	// 1. Compute distances for cells containing particles.
	// Holds the index of the closest particle to each cell.
	int* closestParticles = new int[mX*mY];
	// Initialize everything to -1, indicating unknown.
	// Because of this array, we don't need to initialize m_Phi!
	int gridSize = mX*mY;
	for (int i = 0; i < gridSize; i++) {
		closestParticles[i] = -1;
	}

	int len = (int)particles.size();
	for (int i = 0; i < len; i++) {
		// Compute nearest grid index
		float px = particles[i].X*mX;
		float py = particles[i].Y*mY;
		int cellX = (int)roundf(px);
		int cellY = (int)roundf(py);
		if (cellX<0 || cellX >= mX || cellY<0 || cellY >= mY) continue;
		// Compute kernel
		float k = sqrtf((px - (float)cellX)*(px - (float)cellX)
			+ (py - (float)cellY)*(py - (float)cellY)) - m_pRadius;

		if ((closestParticles[cellX + mX*cellY] < 0) || (m_Phi[cellX + mX*cellY] > k)) {
			closestParticles[cellX + mX*cellY] = i;
			m_Phi[cellX + mX*cellY] = k;
		}
	}

	// 2. Sweep over the grid in all 4 grid orders.
	// This section uses the clsInner fragment, defined above.
	// This isn't great; I hope there's a better way to do it.

	// Q: Does this work?
	// A: Only if you like Manhattan distance. Otherwise, use Zhao's method, below.
	// That said, running this twice might work.
	// looks in x- x+ y- y+
	// Although it's not mentioned in the above articles (?), this method seems to work:
	// If not, use the method in the commented block of text below.
	/*for (int y = 0; y < mY; y++) {
		for (int x = 1; x < mX; x++) {
			clsInner(-1, 0, x, y, particles, closestParticles, m_Phi, m_pRadius, mX, mY);
		}
	}

	for (int y = 0; y < mY; y++) {
		for (int x = mX-2; x >= 0; x--) {
			clsInner(1, 0, x, y, particles, closestParticles, m_Phi, m_pRadius, mX, mY);
		}
	}

	for (int y = 1; y < mY; y++) {
		for (int x = 0; x < mX; x++) {
			clsInner(0, -1, x, y, particles, closestParticles, m_Phi, m_pRadius, mX, mY);
		}
	}

	for (int y = mY-2; y >=0 ; y--) {
		for (int x = 0; x < mX; x++) {
			clsInner(0, 1, x, y, particles, closestParticles, m_Phi, m_pRadius, mX, mY);
		}
	}*/

	// Now, the actual for loops.
	for (int y = 0; y < mY; y++) {
		for (int x = 0; x < mX; x++) {
			if (x != 0)
				clsInner(-1, 0, x, y, particles, closestParticles, m_Phi, m_pRadius, mX, mY);
			if (y != 0) 
				clsInner(0, -1, x, y, particles, closestParticles, m_Phi, m_pRadius, mX, mY);
		}
	}

	for (int x = mX - 1; x >= 0; x--) {
		for (int y = 0; y < mY; y++) {
			if (x != mX - 1)
				clsInner(1, 0, x, y, particles, closestParticles, m_Phi, m_pRadius, mX, mY);
			if (y != 0)
				clsInner(0, -1, x, y, particles, closestParticles, m_Phi, m_pRadius, mX, mY);
		}
	}

	for (int x = mX - 1; x >= 0; x--) {
		for (int y = mY - 1; y >= 0; y--) {
			if (x != mX - 1)
				clsInner(1, 0, x, y, particles, closestParticles, m_Phi, m_pRadius, mX, mY);
			if (y != mY - 1)
				clsInner(0, 1, x, y, particles, closestParticles, m_Phi, m_pRadius, mX, mY);
		}
	}

	for (int x = 0; x < mX; x++) {
		for (int y = mY - 1; y >= 0; y--) {
			if (x != 0)
				clsInner(-1, 0, x, y, particles, closestParticles, m_Phi, m_pRadius, mX, mY);
			if (y != mY - 1)
				clsInner(0, 1, x, y, particles, closestParticles, m_Phi, m_pRadius, mX, mY);
		}
	}
	// and that's it! Clear temporary array.
	delete[] closestParticles;
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
	
	// 1. Does a fast scan over the grid to classify cells by their Manhattan distance to valid cells.
	int* cd = new int[xSize*ySize];

	// Same fast scan method as in ComputeLevelSet, just written out
	int inf = 1000000000;
	for (int y = 0; y < ySize; y++) {
		for (int x = 0; x < xSize; x++) {
			if (validAr[x + xSize*y]) {
				cd[x + xSize*y] = 0;
			} else {
				cd[x + xSize*y] = inf;
			}
		}
	}

	// y+ x+
	for (int y = 0; y < ySize; y++) {
		for (int x = 0; x < xSize; x++) {
			cd[x + xSize*y] = std::min({cd[x + xSize*y],
				(x == 0 ? inf : cd[(x - 1) + xSize*y]+1),
				(y == 0 ? inf : cd[x + (xSize*(y - 1))]+1)
			});
		}
	}

	// y+ x-
	for (int y = 0; y < ySize; y++) {
		for (int x = xSize-1; x >= 0; x--) {
			cd[x + xSize*y] = std::min({ cd[x + xSize*y],
				(x == xSize-1 ? inf : cd[(x + 1) + xSize*y]+1),
				(y == 0 ? inf : cd[x + (xSize*(y - 1))]+1)
			});
		}
	}

	// y- x-
	for (int y = ySize-1; y >=0; y--) {
		for (int x = xSize - 1; x >= 0; x--) {
			cd[x + xSize*y] = std::min({ cd[x + xSize*y],
				(x == xSize - 1 ? inf : cd[(x + 1) + xSize*y]+1),
				(y == ySize - 1 ? inf : cd[x + (xSize*(y + 1))]+1)
			});
		}
	}

	// y- x+
	for (int y = ySize - 1; y >= 0; y--) {
		for (int x = 0; x < xSize; x++) {
			cd[x + xSize*y] = std::min({ cd[x + xSize*y],
				(x == 0 ? inf : cd[(x - 1) + xSize*y]+1),
				(y == ySize - 1 ? inf : cd[x + (xSize*(y + 1))]+1)
			});
		}
	}

	// 2. Partition cells into bins
	// Counts (at most xSize+ySize, but let's count it)
	int numBuckets = 0;
	for (int y = 0; y < ySize; y++) {
		for (int x = 0; x < xSize; x++) {
			if (cd[x + xSize*y] > numBuckets) {
				numBuckets = cd[x + xSize*y];
			}
		}
	}
	numBuckets++;

	int* bucketCounts = new int[numBuckets];
	for (int i = 0; i < numBuckets; i++) {
		bucketCounts[i] = 0;
	}
	for (int y = 0; y < ySize; y++) {
		for (int x = 0; x < xSize; x++) {
			bucketCounts[cd[x + xSize*y]]++;
		}
	}

	// Prefix sum
	int* sums = new int[numBuckets];
	sums[0] = 0;
	for (int i = 1; i < numBuckets; i++) {
		sums[i] = sums[i - 1] + bucketCounts[i - 1];
	}
	
	assert(sums[numBuckets - 1] + bucketCounts[numBuckets - 1] == xSize*ySize);

	int* indices = new int[xSize*ySize * 2];
	for (int y = 0; y < ySize; y++) {
		for (int x = 0; x < xSize; x++) {
			int c = cd[x + xSize*y];
			int index = sums[c];
			indices[2 * index] = x;
			indices[2 * index + 1] = y;
			sums[c]++;
		}
	}

	// 3. Extrapolate values from the inside out.
	int directions[8] = { 1,0,0,1,-1,0,0,-1 };
	int numDirs = 4;
	for (int i = 0; i < xSize*ySize; i++) {
		int x = indices[2 * i];
		int y = indices[2 * i + 1];
		int myc = cd[x + xSize*y];
		if (myc == 0) continue; // don't modify known values
		float numNeighbors = 0.0f;
		float neighborSum = 0.0f;

		for (int d = 0; d < numDirs; d++) {
			int nx = x + directions[2 * d];
			int ny = y + directions[2 * d + 1];
			if (0 <= nx && nx < xSize && 0 <= ny && ny < ySize && (cd[nx + xSize*ny] < myc)) {
				numNeighbors += 1.0f;
				neighborSum += srcAr[nx + xSize*ny];
			}
		}
		assert(numNeighbors > 0.0f);
		srcAr[x + xSize*y] = neighborSum / numNeighbors;
	}

	// Clean up
	delete[] cd;
	delete[] bucketCounts;
	delete[] sums;
	delete[] indices;
	
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

void FluidSim::Project(float dt) {
	//... this is basically Chapter 5 of Bridson.

	// At the heart of this projection routine, we need to solve a set of equations, of the form
	// 4 p_{i,j} - p_{i+1,j} - p_{i-1,j} - p_{i,j+1} - p_{i,j-1} = -rhs[i,j]
	// where rhs[i,j] is the negative discrete divergence at (i,j), times dx^2*rho/dt:
	// rhs[i,j]=-dx*rho/dt (u{i+1/2,j}-u{i-1/2,j}+v{i,j+1/2}-v{i,j-1/2}).

	// This equation is modified at the following way at the boundaries:
	// (see Bridson's lecture notes or the 2nd ed. of the book, pg. 76 for a derivation:)
	
	// - For each bordering solid cell, reduce the coefficient 4 by 1 and remove the reference
	// to that p. Also, use u_solid inside the solid material (which in this case we take to be 0)
	//
	// - For each bordering air cell, just remove the reference to that p.

	// We now solve this linear system with the Gauss-Seidel method, using a checkerboard update.
	// Our operations are:
	// x |-> Dx: divide by number of non-solid neighbors
	// x |-> Rx: replace x_{i,j} with -(sum(water neighbors' pressures)).

	// Now, we'll use the level set calculations, using the method on pg. 128 of Bridson, 2nd ed.
	// Bridson never writes the equation out, but the entire effect of this really is to increase
	// the coefficient on the diagonal from 4 (in the all-fluid case) to 4-phi_{i+1,j}/phi_{i,j}.
	// (This is an increase, because phi_{i+1,j} and phi_i,j} have different signs.)
	// The right hand side is unchanged, and remains the discrete divergence.

	// For now, this is simple enough that we can just write it inside a loop.

	// For Gauss-Seidel, we just do Jacobi's method, but we just only use one array of pressures.

	double maxLSRatio = 1000.0; // What should we clamp -phi_{i+1,j}/phi{i,j} to?

	// I. Compute the right-hand side b.
	int MN = mX*mY;
	double* b = new double[MN];
	double* p = new double[MN](); // should zero everything for us
	double* diagCoeffs = new double[MN];

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

	// Compute diagonal coefficients.
	for (int y = 0; y < mY; y++) {
		for (int x = 0; x < mX; x++) {
			// We still don't solve for pressures in air.
			if (Phi(x, y) >= 0.0) continue;

			// = # of non-solid neighbors plus level set things
			double numNeighbors = 0;
			if (x != 0) {
				numNeighbors += 1;
				if (Phi(x - 1, y) > 0.0) {
					// Plus fluid fractions thing
					numNeighbors += MathHelper::Clamp(-(double)Phi(x - 1, y) / Phi(x, y), 0.0, maxLSRatio);
				}
			}

			if (x != mX - 1) {
				numNeighbors += 1;
				if (Phi(x + 1, y) > 0.0) {
					numNeighbors += MathHelper::Clamp(-(double)Phi(x + 1, y) / Phi(x, y),0.0,maxLSRatio);
				}
			}

			if (y != 0) {
				numNeighbors += 1;
				if (Phi(x, y - 1) > 0.0) {
					numNeighbors += MathHelper::Clamp(-(double)Phi(x, y - 1) / Phi(x, y),0.0,maxLSRatio);
				}
			}

			if (y != mY - 1) {
				numNeighbors += 1;
				if (Phi(x, y + 1) > 0.0) {
					numNeighbors += MathHelper::Clamp(-(double)Phi(x, y + 1) / Phi(x, y),0.0,maxLSRatio);
				}
			}

			diagCoeffs[x + mX*y] = numNeighbors;
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

	int numIterations = 120;

	for (int iter = 0; iter < numIterations; iter++) {
		for (int stage = 0; stage <= 1; stage++) {
			for (int y = 0; y < mY; y++) {
				for (int x = 0; x < mX; x++) {
					// Checkerboard iteration
					if (((x + y) % 2) != stage) continue;

					// Gauss-Seidel update p[x,y].
					double numNeighbors = diagCoeffs[x + mX*y];
					double neighborMinusSum = 0;

					// If this cell is air, then there's no equation for this cell - 
					if (Phi(x, y) >= 0.0f) continue;

					if (x != 0) {
						if (Phi(x - 1, y) < 0.0f) {
							neighborMinusSum -= p[(x - 1) + mX*(y)];
						}
					}
					if (x != mX - 1) {
						if (Phi(x + 1, y) < 0.0f) {
							neighborMinusSum -= p[(x + 1) + mX*(y)];
						}
					}
					if (y != 0) {
						if (Phi(x, y - 1) < 0.0f){
							neighborMinusSum -= p[x + mX*(y - 1)];
						}
					}
					if (y != mY - 1) {
						if (Phi(x, y + 1) < 0.0f) {
							neighborMinusSum -= p[x + mX*(y + 1)];
						}
					}

					// Get ready for it... here it comes!
					// p[x + mX*y] = (b[x + mX*y] - neighborMinusSum) / numNeighbors;

					// Woah, it's successive over-relaxation!
					p[x + mX*y] = (1 - omega)*p[x + mX*y] + omega*(b[x + mX*y] - neighborMinusSum) / numNeighbors;
				}
			}
		}
	}

	// Remove pressure from velocities
	// Pressure gradient update in the usual case:
	// u_{i+1/2,j,k}^{n+1} = u_{i+1/2,j,k}-dt(p_{i+1,j,k}-p_{i,j,k})/(rho dx)
	// In the case where {i,j} is water and {i+1,j} is air (and similarly for other cases):
	// u_{i+1/2,j,k}^{n+1} = u_{i+1/2,j,k}+dt(1+clamp(-phi_{i+1,j,k}/phi_{i,j,k}))*p_{i,j,k}/(rho dx)
	// Note that this does indeed have the right limiting behavior.

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
			double phiL = Phi(x, y);
			double phiR = Phi(x + 1, y);
			// Four cases:
			if (phiL < 0.0 && phiR < 0.0) {
				U(x + 1, y) = (float)(U(x + 1, y) - scale*(p[(x + 1) + mX*y] - p[x + mX*y]));
			} else if (phiL < 0.0 && phiR >= 0.0) {
				U(x + 1, y) = (float)(U(x + 1, y) + scale*(1 + MathHelper::Clamp(-phiR / phiL, 0.0, maxLSRatio))*p[x + mX*y]);
			} else if (phiL >= 0.0 && phiR < 0.0) {
				// I think this is right (...it seems to be?)
				U(x + 1, y) = (float)(U(x + 1, y) + scale*(1 + MathHelper::Clamp(-phiL / phiR, 0.0, maxLSRatio))*p[(x + 1) + mX*y]);
			} else {
				// In air
				U(x + 1, y) = 0;
			}
		}
	}
	// V
	for (int y = 0; y < mY - 1; y++) {
		for (int x = 0; x < mX; x++) {
			double phiD = Phi(x, y);
			double phiU = Phi(x, y+1);
			if (phiD < 0.0 && phiU < 0.0) {
				V(x, y + 1) = (float)(V(x, y + 1) - scale*(p[x + mX*(y+1)] - p[x + mX*y]));
			} else if (phiD < 0.0 && phiU >= 0.0) {
				V(x, y + 1) = (float)(V(x, y + 1) + scale*(1 + MathHelper::Clamp(-phiU / phiD, 0.0, maxLSRatio))*p[x + mX*y]);
			} else if (phiD >= 0.0 && phiU < 0.0) {
				V(x, y + 1) = (float)(V(x, y + 1) + scale*(1 + MathHelper::Clamp(-phiD / phiU, 0.0, maxLSRatio))*p[x + mX*(y+1)]);
			} else {
				V(x, y + 1) = 0;
			}
		}
	}
	// end (modifies internal state).

	delete[] b;
	delete[] p;
	delete[] diagCoeffs;
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