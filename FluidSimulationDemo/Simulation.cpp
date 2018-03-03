//*********************************************************
// Simulation.cpp
// Top level of code for doing fluid simulation on the CPU.
//
// Authors:
//   Neil Bickford
//*********************************************************

#include "Simulation.h"
#include "odprintf.h"

FluidSim::FluidSim(int xSize, int ySize, float CellsPerMeter)
	:mX(xSize), mY(ySize), m_CellsPerMeter(CellsPerMeter),
	m_particles() {
	// Set up MAC velocity grids and initialize velocities
	m_MU = new float[(xSize + 1)*ySize];
	m_MV = new float[xSize*(ySize + 1)];

	// TODO: the scale on this for vectorCurl is incorrect
	// MU
	for (int y = 0; y < ySize; y++) {
		for (int x = 0; x < xSize + 1; x++){
			XMFLOAT2 vc = vectorCurl(x/CellsPerMeter, (y+0.5f)/CellsPerMeter);
			U(x, y) = vc.x;
		}
	}

	// MV
	for (int y = 0; y < ySize + 1; y++) {
		for (int x = 0; x < xSize; x++) {
			XMFLOAT2 vc = vectorCurl((x+0.5f)/CellsPerMeter, y/CellsPerMeter);
			V(x, y) = vc.y;
		}
	}

	// Create a uniform distribution of particles
	// and set their velocities
	for (int y = 0; y < ySize; y++) {
		for (int x = 0; x < xSize; x++) {
			float px = x + 0.25f;
			float py = y + 0.25f;
			float rX = px / m_CellsPerMeter; //real-world X
			float rY = py / m_CellsPerMeter;
			float d = 0.5f / m_CellsPerMeter;
			m_particles.emplace_back(rX, rY, vectorCurl(rX, rY));
			m_particles.emplace_back(rX+d, rY, vectorCurl((rX+d), rY));
			m_particles.emplace_back(rX, rY+d, vectorCurl(rX, rY+d));
			m_particles.emplace_back(rX+d, rY+d, vectorCurl((rX+d), (rY + d)));
		}
	}

	// TEST
	// This doesn't do anything yet
	Project(1.0);

	float maxdiv = 0;
	int maxdivX = -1;
	int maxdivY = -1;
	odprintf("{");
	for (int y = 0; y < ySize; y++) {
		odprintf("{");
		for (int x = 0; x < xSize; x++) {
			float div = U(x + 1, y) + V(x, y + 1) - U(x, y) - V(x, y);
			if (abs(div) > abs(maxdiv)) {
				maxdiv = div;
				maxdivX = x;
				maxdivY = y;
			}
			odprintf("%f", div);
			if (x != xSize - 1) {
				odprintf(",");
			}
		}
		odprintf("}");
		if (y != ySize - 1) {
			odprintf(",\n");
		}
	}
	odprintf("}");
	odprintf("Maximum divergence of %f is at (x,y)=(%i, %i)", maxdiv, maxdivX, maxdivY);
}

FluidSim::~FluidSim() {
	m_particles.clear();
	delete[] m_MU;
	delete[] m_MV;
}

void FluidSim::Simulate(float dt) {
	// Clamp maximum dt
	dt = MathHelper::Clamp(dt, 0.0f, 0.5f);

	Advect(m_particles, dt);

	//AddBodyForces(dt);

	Project(dt);
}

void FluidSim::Advect(std::vector<Particle> &particles, float dt) {
	// Tracing particle paths and velocities using an RK3 method
	// [Bridson, pg. 109-110], based on [Ralston, 1962].
	// For the velocity calculation, suppose a particle moves with velocity (ux, uy) m/s.
	// Then the particle moves (ux,uy)*dt meters in dt seconds, which we can invert.

	// Set to false to use interpolate and the computed MAC grids
	if (false) {
		int len = particles.size();
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
		int len = particles.size();
		for (int i = 0; i < len; i++) {
			XMFLOAT2 k1 = InterpolateMACCell(mX*particles[i].X, mY*particles[i].Y);
			XMFLOAT2 k2 = InterpolateMACCell(mX*(particles[i].X + 0.5f*dt*k1.x), mY*(particles[i].Y + 0.5f*dt*k1.y));
			XMFLOAT2 k3 = InterpolateMACCell(mX*(particles[i].X + 0.75f*dt*k2.x), mY*(particles[i].Y + 0.75f*dt*k2.y));

			particles[i].uX = (2.0f / 9.0f)*k1.x + (3.0f / 9.0f)*k2.x + (4.0f / 9.0f)*k3.x;
			particles[i].uY = (2.0f / 9.0f)*k1.y + (3.0f / 9.0f)*k2.y + (4.0f / 9.0f)*k3.y;
			particles[i].X += dt*particles[i].uX;
			particles[i].Y += dt*particles[i].uY;
		}
	}
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

/*void odPrintDivergence(double* ar, int xSize, int ySize) {
	odprintf("{");
	for (int y = 0; y < ySize; y++) {
		odprintf("{");
		for (int x = 0; x < xSize; x++) {
			float div = U(x + 1, y) + V(x, y + 1) - U(x, y) - V(x, y);
			if (abs(div) > abs(maxdiv)) {
				maxdiv = div;
				maxdivX = x;
				maxdivY = y;
			}
			odprintf("%f", div);
			if (x != xSize - 1) {
				odprintf(",");
			}
		}
		odprintf("}");
		if (y != ySize - 1) {
			odprintf(",\n");
		}
	}
	odprintf("}");
	odprintf("Maximum divergence of %f is at (x,y)=(%i, %i)", maxdiv, maxdivX, maxdivY);
}*/

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

	double omega = 0.0;
	double lastL2 = 0.0;

	// Based off of measurements from this code, we generally want to take
	// omega to be almost exactly 2-3.22133/mX, based on iterating
	// on 16x16, 32x32, and 64x64 grids for 30, 30, and 60 iterations, respectively.
	// (Note, interestingly, that in the last two cases this isn't even enough iterations
	// for boundary conditions to reach the edges of the grid, yet it still manages an
	// eventual convergence rate of 1/0.85!)

	int numIterations = 60;
	while (omega < 1.99999) {
		for (int y = 0; y < mY; y++) {
			for (int x = 0; x < mX; x++) {
				p[x + mX*y] = 0;
			}
		}

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

			//odPrintArray(p,mX,mY);
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

			/*if (omega == 1.00) {
				odPrintArray(p, mX, mY);
			}*/

			if (iter == numIterations - 1)
				odprintf("{%lf, %lf, %lf},", omega, sqrt(l2Sq / lastL2), sqrt(l2Sq));
			lastL2 = l2Sq;
		}
		omega += 0.001;
	}
	//odPrintArray(b, mX, mY);
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