//************************************************************
// Simulation3D.cpp
// Top level of code for doing 3D fluid simulation on the CPU.
//
// Authors:
//   Neil Bickford
//************************************************************

#include "Simulation3D.h"
#include "odprintf.h"
#include <limits>
#include <queue> // For serial extrapolation - see ExtrapolateValues(4).
#include <random>
#include <algorithm> // for std::min
#include <numeric> // for debug std::accumulate

//#include "debugroutines.h" TODO: Implement stb-like single-header library properly

FluidSim3::FluidSim3(int xSize, int ySize, int zSize, float CellsPerMeter)
	:mX(xSize), mY(ySize), mZ(zSize), m_CellsPerMeter(CellsPerMeter),
	m_particles() {
	// Set up MAC velocity grids and initialize velocities
	m_MU = new float[(xSize + 1)*ySize*zSize];
	m_MV = new float[xSize*(ySize + 1)*zSize];
	m_MW = new float[xSize*ySize*(zSize + 1)];

	// Level sets and auxiliary fields
	m_Phi = new float[xSize*ySize*zSize];

	ResetSimulation();
}

FluidSim3::~FluidSim3() {
	m_particles.clear();
	delete[] m_MU;
	delete[] m_MV;
	delete[] m_MW;
	delete[] m_Phi;
}

void FluidSim3::ResetSimulation() {
	// TODO: the scale on this for vectorCurl is incorrect
	std::default_random_engine generator(0);
	std::uniform_real_distribution<float> distribution(-0.25f, 0.25f);
	// MU
	for (int z = 0; z < mZ; z++) {
		for (int y = 0; y < mY; y++) {
			for (int x = 0; x < mX + 1; x++) {
				//U(x, y, z) = 0.0f;
				U(x, y, z) = distribution(generator);
			}
		}
	}

	// MV
	for (int z = 0; z < mZ; z++) {
		for (int y = 0; y < mY + 1; y++) {
			for (int x = 0; x < mX; x++) {
				V(x, y, z) = distribution(generator); //0.0f;
			}
		}
	}

	// MW
	for (int z = 0; z < mZ + 1; z++) {
		for (int y = 0; y < mY; y++) {
			for (int x = 0; x < mX; x++) {
				W(x, y, z) = distribution(generator); //0.0f;
			}
		}
	}

	// Create a uniform distribution of particles
	// and set their velocities
	m_particles.clear();
	for (int z = 1; z < mZ - 1; z++) {
		for (int y = 1; y < mY - 1; y++) {
			for (int x = mX / 2; x < mX - 1; x++) {
				float px = x - 0.25f;
				float py = y - 0.25f;
				float pz = z - 0.25f;
				float rX = px / m_CellsPerMeter; //real-world X
				float rY = py / m_CellsPerMeter;
				float rZ = pz / m_CellsPerMeter;
				float d = 0.5f / m_CellsPerMeter;
				for (int u = 0; u <= 1; u++) {
					for (int v = 0; v <= 1; v++) {
						for (int w = 0; w <= 1; w++) {
							float m1 = rX + u*d + distribution(generator) / m_CellsPerMeter;
							float m2 = rY + v*d + distribution(generator) / m_CellsPerMeter;
							float m3 = rZ + w*d + distribution(generator) / m_CellsPerMeter;
							m_particles.emplace_back(m1, m2, m3, InterpolateMACCell(mX*m1, mY*m2, mZ*m3));
						}
					}
				}
			}
		}
	}
}

void FluidSim3::Simulate(float dt) {
	// Clamp maximum dt
	dt = MathHelper::Clamp(dt, 0.0f, 1.0f / 15.0f);
	dt = 0.01f; // QQQ DEBUG Limit dt for debugging purposes
	
				// Iterate frame counter
	static int frame = 0;
	frame++;

	Advect(m_particles, dt);

	// In this step, we also do a hybrid FLIP/PIC step to update the new particle velocities.
	// Letting alpha be 6*dt*m_nu*m_CellsPerMeter^2 (pg. 118),
	float alpha = MathHelper::Clamp(6 * dt*m_nu*m_CellsPerMeter*m_CellsPerMeter, 0.0f, 1.0f);
	//alpha = 1.00f;
	// Ex. For a 64x64 grid with a dt of 1/60, this is 0.0003645, since m_nu is so small (~10^-6)
	ComputeLevelSet(m_particles);

	TransferParticlesToGrid(m_particles);

	// Moving complexity here for the moment, this should be refactored out
	// Store the old grid (...this actually does seem to be the best way)
	float* m_oldU = new float[(mX + 1)*mY*mZ];
	float* m_oldV = new float[mX*(mY + 1)*mZ];
	float* m_oldW = new float[mX*mY*(mZ + 1)];
	for (int i = 0; i < (mX + 1)*mY*mZ; i++) {
		m_oldU[i] = m_MU[i];
	}
	for (int i = 0; i < mX*(mY + 1)*mZ; i++) {
		m_oldV[i] = m_MV[i];
	}
	for (int i = 0; i < mX*mY*(mZ + 1); i++) {
		m_oldW[i] = m_MW[i];
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
	for (int i = 0; i < (mX + 1)*mY*mZ; i++) {
		m_oldU[i] = m_MU[i] - (1.0f - alpha)*m_oldU[i];
	}
	for (int i = 0; i < mX*(mY + 1)*mZ; i++) {
		m_oldV[i] = m_MV[i] - (1.0f - alpha)*m_oldV[i];
	}
	for (int i = 0; i < mX*mY*(mZ + 1); i++) {
		m_oldW[i] = m_MW[i] - (1.0f - alpha)*m_oldW[i];
	}

	// Allow for interpolation via tricky pointer hacking
	std::swap(m_oldU, m_MU);
	std::swap(m_oldV, m_MV);
	std::swap(m_oldW, m_MW);

	int len = (int)m_particles.size();
	for (int i = 0; i < len; i++) {
		XMFLOAT3 interpDiff = InterpolateMACCell(mX*m_particles[i].X, mY*m_particles[i].Y, mZ*m_particles[i].Z);
		if (fabsf(interpDiff.x) > 1000 || fabsf(interpDiff.y) > 1000 || fabsf(interpDiff.z)>1000) {
			odprintf("Hey, what's going on here?");
		}
		m_particles[i].uX = (1.0f - alpha)*m_particles[i].uX + interpDiff.x;
		m_particles[i].uY = (1.0f - alpha)*m_particles[i].uY + interpDiff.y;
		m_particles[i].uZ = (1.0f - alpha)*m_particles[i].uZ + interpDiff.z;
		if (m_particles[i].uX > 100000) {
			m_particles[i].uX = 1000000;
			assert(false && "Velocity was too high!");
		}
	}
	// ...and swap back
	std::swap(m_oldU, m_MU);
	std::swap(m_oldV, m_MV);
	std::swap(m_oldW, m_MW);

	// ...and clean up :(
	delete[] m_oldU;
	delete[] m_oldV;
	delete[] m_oldW;

	//odprintf("Ran a step");
}

void FluidSim3::Advect(std::vector<Particle3> &particles, float dt) {
	// Tracing particle paths and velocities using an RK3 method
	// [Bridson, pg. 109-110], based on [Ralston, 1962].
	// For the velocity calculation, suppose a particle moves with velocity (ux, uy) m/s.
	// Then the particle moves (ux,uy)*dt meters in dt seconds, which we can invert.

	// Set to false to use interpolate and the computed MAC grids
	int len = (int)particles.size();
	int di = 272;
	if (particles[di].Y > 15.0f / 16.0f) {
		odprintf("{%f, %f, %f}", particles[di].uX, particles[di].uY, particles[di].uZ);
	}
	for (int i = 0; i < len; i++) {
		if (i == di && particles[di].Y > 15.0f / 16.0f) {
			odprintf("hey");
		}
		// This was surprisingly difficult to find, but according to an article in GPU Gems 5
		// (source: https://books.google.com/books?id=uIDSBQAAQBAJ&pg=PA60&lpg=PA60&dq=using+flip+with+runge-kutta&source=bl&ots=XMJL03mmLe&sig=ZVlwK4HpjKePBfpq9ew1T4zeuUI&hl=en&sa=X&ved=0ahUKEwiWtcjpgdfZAhVIYK0KHVz1BVsQ6AEIeTAI#v=onepage&q=using%20flip%20with%20runge-kutta&f=false)
		// (search key: "using FLIP with runge-kutta")
		// we should actually update the particle's position entirely using interpolation on the grid,
		// and only update the particle's velocity using the grid-update step in FLIP.
		XMFLOAT3 k1 = InterpolateMACCell(mX*particles[i].X, mY*particles[i].Y, mZ*particles[i].Z); // should actually be cellsPerMeter?
		XMFLOAT3 k2 = InterpolateMACCell(mX*(particles[i].X + 0.5f*dt*k1.x), 
										 mY*(particles[i].Y + 0.5f*dt*k1.y),
										 mZ*(particles[i].Z+0.5f*dt*k1.z));
		XMFLOAT3 k3 = InterpolateMACCell(mX*(particles[i].X + 0.75f*dt*k2.x), 
										 mY*(particles[i].Y + 0.75f*dt*k2.y),
										 mZ*(particles[i].Z + 0.75f*dt*k2.z));

		float vX = (2.0f / 9.0f)*k1.x + (3.0f / 9.0f)*k2.x + (4.0f / 9.0f)*k3.x;
		float vY = (2.0f / 9.0f)*k1.y + (3.0f / 9.0f)*k2.y + (4.0f / 9.0f)*k3.y;
		float vZ = (2.0f / 9.0f)*k1.z + (3.0f / 9.0f)*k2.z + (4.0f / 9.0f)*k3.z;
		particles[i].X += dt*vX;
		particles[i].Y += dt*vY;
		particles[i].Z += dt*vZ;

		// Clamp to box, slightly inwards
		float eps = 0.1f;
		particles[i].X = MathHelper::Clamp(particles[i].X, (-0.5f + eps) / mX, 1.0f + (-0.5f - eps) / mX);
		particles[i].Y = MathHelper::Clamp(particles[i].Y, (-0.5f + eps) / mY, 1.0f + (-0.5f - eps) / mY);
		particles[i].Z = MathHelper::Clamp(particles[i].Z, (-0.5f + eps) / mZ, 1.0f + (-0.5f - eps) / mZ);
	}
}

float FluidSim3::ptDistance(float x0, float y0, float z0, float x1, float y1, float z1) {
	return sqrtf((x0 - x1)*(x0 - x1) + (y0 - y1)*(y0 - y1) + (z0 - z1)*(z0 - z1)) - m_pRadius;
}

void FluidSim3::clsInner(int dx, int dy, int dz, int x, int y, int z,
	int* closestParticles) {

	int otherPt = closestParticles[(x + dx) + mX*((y + dy) + mY*(z + dz))];
	if (otherPt > 0) {
		float px = m_particles[otherPt].X*mX;
		float py = m_particles[otherPt].Y*mY;
		float pz = m_particles[otherPt].Z*mZ;
		float dist = ptDistance((float)x, (float)y, (float)z, px, py, pz);
		int idx = x + mX*(y + mY*z);
		if (closestParticles[idx] < 0 || dist < m_Phi[idx]) {
			closestParticles[idx] = otherPt;
			m_Phi[idx] = dist;
		}
	}
}

void FluidSim3::ComputeLevelSet(const std::vector<Particle3> &particles) {
	// Uses fast sweeping to compute the level set Phi of the given grid,
	// using the kernel
	// sqrt((x-x0)^2+(y-y0)^2)-m_pRadius.

	// Woah, fast sweeping has a Wikipedia article! But it's a stub.
	// Bridson p. 126 says AKPG07 has a practical fast /marching/ method
	// However, chapter 4.3 (pg 49-51) has pseudocode for the method we'll be using,
	// which is based on algorithm 4 in [Tsa02],
	// "Rapid and Accurate Computation of the Distance Function using Grids".
	// See also [JBS06], "3D Distance Fields: A Survey of Techniques and Applications".
	// In this code, we'll use Table 1 of "Fast Occlusion Sweeping" by Singh, Yuksel,
	// and House, which requires 24 sweeps. (Question: Can we do better? 2 passes of a
	// Manhattan distance method give 12 sweeps, but is that enough?)
	

	// 1. Compute distances for cells containing particles.
	// Holds the index of the closest particle to each cell.
	int* closestParticles = new int[mX*mY*mZ];
	// Initialize everything to -1, indicating unknown.
	// Because of this array, we don't need to initialize m_Phi!
	int gridSize = mX*mY*mZ;
	for (int i = 0; i < gridSize; i++) {
		closestParticles[i] = -1;
	}

	int len = (int)particles.size();
	for (int i = 0; i < len; i++) {
		// Compute nearest grid index
		float px = particles[i].X*mX;
		float py = particles[i].Y*mY;
		float pz = particles[i].Z*mZ;
		int cellX = (int)roundf(px);
		int cellY = (int)roundf(py);
		int cellZ = (int)roundf(pz);
		if (cellX<0 || cellX >= mX 
			|| cellY<0 || cellY >= mY
			|| cellZ<0 || cellZ >= mZ) continue;
		// Compute kernel
		float k = ptDistance(px, py, pz, (float)cellX, (float)cellY, (float)cellZ);
		int ci = cellX + mX*(cellY + mY*cellZ);

		if ((closestParticles[ci] < 0) || (m_Phi[ci] > k)) {
			closestParticles[ci] = i;
			m_Phi[ci] = k;
		}
	}

	// 2. Sweep over the grid in all 4 grid orders.
	// This section uses the clsInner fragment, defined above.
	// This isn't great; I hope there's a better way to do it.

	// x+ y+ z+
	for (int z = 0; z < mZ; z++) {
		for (int y = 0; y < mY; y++) {
			for (int x = 0; x < mX; x++) {
				if (x != 0)
					clsInner(-1, 0, 0, x, y, z, closestParticles);
				if (y != 0)
					clsInner(0, -1, 0, x, y, z, closestParticles);
				if (z != 0)
					clsInner(0, 0, -1, x, y, z, closestParticles);
			}
		}
	}

	// x- y+ z+
	for (int z = 0; z < mZ; z++) {
		for (int y = 0; y < mY; y++) {
			for (int x = mX-1; x >=0; x--) {
				if (x != mX-1)
					clsInner(1, 0, 0, x, y, z, closestParticles);
				if (y != 0)
					clsInner(0, -1, 0, x, y, z, closestParticles);
				if (z != 0)
					clsInner(0, 0, -1, x, y, z, closestParticles);
			}
		}
	}

	// x+ y- z+
	for (int z = 0; z < mZ; z++) {
		for (int y = mY-1; y >= 0; y--) {
			for (int x = 0; x < mX; x++) {
				if (x != 0)
					clsInner(-1, 0, 0, x, y, z, closestParticles);
				if (y != mY-1)
					clsInner(0, 1, 0, x, y, z, closestParticles);
				if (z != 0)
					clsInner(0, 0, -1, x, y, z, closestParticles);
			}
		}
	}

	// x- y- z+
	for (int z = 0; z < mZ; z++) {
		for (int y = mY - 1; y >= 0; y--) {
			for (int x = mX-1; x >= 0; x--) {
				if (x != mX - 1)
					clsInner(1, 0, 0, x, y, z, closestParticles);
				if (y != mY - 1)
					clsInner(0, 1, 0, x, y, z, closestParticles);
				if (z != 0)
					clsInner(0, 0, -1, x, y, z, closestParticles);
			}
		}
	}

	// x+ y+ z-
	for (int z = mZ-1; z >= 0; z--) {
		for (int y = 0; y < mY; y++) {
			for (int x = 0; x < mX; x++) {
				if (x != 0)
					clsInner(-1, 0, 0, x, y, z, closestParticles);
				if (y != 0)
					clsInner(0, -1, 0, x, y, z, closestParticles);
				if (z != mZ-1)
					clsInner(0, 0, 1, x, y, z, closestParticles);
			}
		}
	}

	// x- y+ z-
	for (int z = mZ-1; z >= 0; z--) {
		for (int y = 0; y < mY; y++) {
			for (int x = mX - 1; x >= 0; x--) {
				if (x != mX - 1)
					clsInner(1, 0, 0, x, y, z, closestParticles);
				if (y != 0)
					clsInner(0, -1, 0, x, y, z, closestParticles);
				if (z != mZ-1)
					clsInner(0, 0, 1, x, y, z, closestParticles);
			}
		}
	}

	// x+ y- z-
	for (int z = mZ-1; z >= 0; z--) {
		for (int y = mY - 1; y >= 0; y--) {
			for (int x = 0; x < mX; x++) {
				if (x != 0)
					clsInner(-1, 0, 0, x, y, z, closestParticles);
				if (y != mY - 1)
					clsInner(0, 1, 0, x, y, z, closestParticles);
				if (z != mZ - 1)
					clsInner(0, 0, 1, x, y, z, closestParticles);
			}
		}
	}

	// x- y- z-
	for (int z = mZ-1; z >= 0; z--) {
		for (int y = mY - 1; y >= 0; y--) {
			for (int x = mX - 1; x >= 0; x--) {
				if (x != mX - 1)
					clsInner(1, 0, 0, x, y, z, closestParticles);
				if (y != mY - 1)
					clsInner(0, 1, 0, x, y, z, closestParticles);
				if (z != mZ - 1)
					clsInner(0, 0, 1, x, y, z, closestParticles);
			}
		}
	}

	// and that's it! Clear temporary array.
	delete[] closestParticles;
}

void FluidSim3::TransferParticlesToGrid(std::vector<Particle3> &particles) {
	// VELOCITY UPDATES
	// Implements Bridson's equation 7.2 with a trilinear hat kernel (pg. 112) for transferring particle velocities
	// to the grid, along with a hybrid FLIP/PIC update.
	//-------------------------------------
	// TRANSFER PARTICLE VELOCITIES TO GRID
	//-------------------------------------
	int len = (int)particles.size();
	// Accumulates weights
	float* uAmts = new float[(mX + 1)*mY*mZ](); // sets to 0, hopefully
	float* vAmts = new float[mX*(mY + 1)*mZ]();
	float* wAmts = new float[mX*mY*(mZ + 1)]();

	// Clear velocity arrays
	memset(m_MU, 0, sizeof(float)*(mX + 1)*mY*mZ);
	memset(m_MV, 0, sizeof(float)*mX*(mY + 1)*mZ);
	memset(m_MW, 0, sizeof(float)*mX*mY*(mZ + 1));

	for (int i = 0; i < len; i++) {
		// We have to assume that particles can be outside the grid for the moment :(
		float px = particles[i].X*m_CellsPerMeter;
		float py = particles[i].Y*m_CellsPerMeter;
		float pz = particles[i].Z*m_CellsPerMeter;

		if (px<-0.5 || px>(mX + 0.5)
			|| py<-0.5 || py>(mY + 0.5)
			|| pz<-0.5 || pz>(mZ + 0.5)) {
			continue; // out of bounds, sorry
		}
		// ensured: -0.5<px<mX+0.5, -0.5<py<mY+0.5, and -0.5<pz<mZ+0.5

		// Remember, the centers of squares are actually integers, not half-integers.
		// U
		// affects values: z=floor(pz) and floor(pz)+1
		//                 y=floor(py) and floor(py)+1
		//                 x=floor(px+1/2)-1/2 and floor(px+1/2)+1/2 ...though for indices we add 1/2 to each of those
		int iz = (int)floorf(pz);
		int iy = (int)floorf(py);
		int ix = (int)floorf(px + 0.5f);
		// Computing alpha for z, y, and x
		float alphaz = pz - (float)iz;
		float alphay = py - (float)iy;
		float alphax = (px + 0.5f) - (float)ix;
		// Assigning values and weights
		for (int z = 0; z <= 1; z++) {
			for (int y = 0; y <= 1; y++) {
				for (int x = 0; x <= 1; x++) {
					if ((ix + x) <= mX 
						&& 0 <= (iy + y) && (iy + y) < mY
						&& 0 <= (iz + z) && (iz + z) < mZ) {
						float w = (x > 0 ? alphax : 1.f - alphax)
							     *(y > 0 ? alphay : 1.f - alphay)
						         *(z > 0 ? alphaz : 1.f - alphaz);
						U(ix + x, iy + y, iz+z) += w*particles[i].uX;
						uAmts[(ix + x) + (mX + 1)*((iy + y) + mZ*(iz + z))] += w;
					}
				}
			}
		}

		// V
		// affects values: x=floor(px) and floor(px)+1
		//                 z=floor(pz) and floor(pz)+1
		//                 y=floor(py+1/2)-1/2 and floor(py+1/2)+1/2, with same 1/2 index bias as above.
		ix = (int)floorf(px);
		iy = (int)floorf(py + 0.5f);
		iz = (int)floorf(pz);
		// Computing alpha for x and y
		alphax = px - (float)ix;
		alphay = (py + 0.5f) - (float)iy;
		alphaz = pz - (float)iz;
		// Assigning values and weights
		for (int z = 0; z <= 1; z++) {
			for (int y = 0; y <= 1; y++) {
				for (int x = 0; x <= 1; x++) {
					if ((iy + y) <= mY
						&& 0 <= (ix + x) && (ix + x) < mX
						&& 0 <= (iz + z) && (iz + z) < mZ) {
						float w = (x > 0 ? alphax : 1.f - alphax)
							     *(y > 0 ? alphay : 1.f - alphay)
							     *(z > 0 ? alphaz : 1.f - alphaz);
						V(ix + x, iy + y, iz + z) += w*particles[i].uY;
						vAmts[(ix + x) + mX*((iy + y) + (mY + 1)*(iz + z))] += w;
					}
				}
			}
		}

		// W
		// affects values: x=floor(px) and floor(px)+1
		//                 y=floor(py) and floor(py)+1
		//                 z=floor(pz+1/2)-1/2 and floor(pz+1/2)+1/2, with same 1/2 index bias as above.
		ix = (int)floorf(px);
		iy = (int)floorf(py);
		iz = (int)floorf(pz + 0.5f);
		// Computing alpha for x and y
		alphax = px - (float)ix;
		alphay = py - (float)iy;
		alphaz = (pz + 0.5f) - (float)iz;
		// Assigning values and weights
		for (int z = 0; z <= 1; z++) {
			for (int y = 0; y <= 1; y++) {
				for (int x = 0; x <= 1; x++) {
					if ((iz + z) <= mZ
						&& 0 <= (ix + x) && (ix + x) < mX
						&& 0 <= (iy + y) && (iy + y) < mY) {
						float w = (x > 0 ? alphax : 1.f - alphax)
							*(y > 0 ? alphay : 1.f - alphay)
							*(z > 0 ? alphaz : 1.f - alphaz);
						W(ix + x, iy + y, iz + z) += w*particles[i].uZ;
						wAmts[(ix + x) + mX*((iy + y) + mY*(iz + z))] += w;
					}
				}
			}
		}
	}

	// Divide U, V, and W by uAmts, vAmts, and wAmts
	float div_eps = std::numeric_limits<float>::denorm_min();
	for (int z = 0; z < mZ; z++) {
		for (int y = 0; y < mY; y++) {
			for (int x = 0; x < mX + 1; x++) {
				U(x, y, z) = (U(x, y, z) / (div_eps + uAmts[x + (mX + 1)*(y + mY*z)]));
			}
		}
	}
	for (int z = 0; z < mZ; z++) {
		for (int y = 0; y < mY + 1; y++) {
			for (int x = 0; x < mX; x++) {
				V(x, y, z) = (V(x, y, z) / (div_eps + vAmts[x + mX*(y + (mY + 1)*z)]));
			}
		}
	}
	for (int z = 0; z < mZ + 1; z++) {
		for (int y = 0; y < mY; y++) {
			for (int x = 0; x < mX; x++) {
				W(x, y, z) = (W(x, y, z) / (div_eps + wAmts[x + mX*(y + mY*z)]));
			}
		}
	}

	// EXTRAPOLATE UNKNOWN VELOCITIES
	// Fixed: We know the velocities of U and V at the edges as well.
	float zero_thresh = 0.01f;
	bool* uValid = new bool[(mX + 1)*mY*mZ];
	bool* vValid = new bool[mX*(mY + 1)*mZ];
	bool* wValid = new bool[mX*mY*(mZ + 1)];
	for (int i = 0; i < (mX + 1)*mY*mZ; i++)
		uValid[i] = (uAmts[i] > zero_thresh);
	for (int i = 0; i < mX*(mY + 1)*mZ; i++)
		vValid[i] = (vAmts[i] > zero_thresh);
	for (int i = 0; i < mX*mY*(mZ + 1); i++) {
		wValid[i] = (wAmts[i] > zero_thresh);
	}

	// Edges
	SetEdgeVelocitiesToZero();
	// U
	for (int z = 0; z < mZ; z++) {
		for (int y = 0; y < mY; y++) {
			uValid[0 + (mX + 1)*(y + mY*z)] = true;
			uValid[mX + (mX + 1)*(y + mY*z)] = true;
		}
	}
	// V
	for (int z = 0; z < mZ; z++) {
		for (int x = 0; x < mX; x++) {
			vValid[x + mX*(0+(mY+1)*z)] = true;
			vValid[x + mX*(mY+(mY+1)*z)] = true;
		}
	}
	// W
	for (int y = 0; y < mY; y++) {
		for (int x = 0; x < mX; x++) {
			wValid[x + mX*(y + mY*0)] = true;
			wValid[x + mX*(y + mY*mZ)] = true;
		}
	}

	ExtrapolateValues(m_MU, uValid, mX + 1, mY, mZ);
	ExtrapolateValues(m_MV, vValid, mX, mY + 1, mZ);
	ExtrapolateValues(m_MW, wValid, mX, mY, mZ + 1);

	delete[] uValid;
	delete[] vValid;
	delete[] wValid;

	delete[] uAmts;
	delete[] vAmts;
	delete[] wAmts;
}

void FluidSim3::ExtrapolateValues(float* srcAr, bool* validAr, int xSize, int ySize, int zSize) {
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
	int* cd = new int[xSize*ySize*zSize];

	// Since we're just computing Manhattan distance, I think we can do this in just 6 scans:
	// x- x+ y- y+ z- z+

	// Initialize
	int inf = 1000000000;
	for (int i = 0; i < xSize*ySize*zSize; i++) {
		if (validAr[i]) {
			cd[i] = 0;
		} else {
			cd[i] = inf;
		}
	}

	// x-
	for (int x = 1; x < xSize; x++) {
		for (int z = 0; z < zSize; z++) {
			for (int y = 0; y < ySize; y++) {
				int idx = x + xSize*(y + ySize*z);
				cd[idx] = std::min(cd[idx], cd[idx - 1] + 1);
			}
		}
	}

	// x+
	for (int x = xSize-2; x >= 0; x--) {
		for (int z = 0; z < zSize; z++) {
			for (int y = 0; y < ySize; y++) {
				int idx = x + xSize*(y + ySize*z);
				cd[idx] = std::min(cd[idx], cd[idx + 1] + 1);
			}
		}
	}

	// y-
	for (int y = 1; y < ySize; y++) {
		for (int z = 0; z < zSize; z++) {
			for (int x = 0; x < xSize; x++) {
				int idx = x + xSize*(y + ySize*z);
				cd[idx] = std::min(cd[idx], cd[idx - xSize] + 1);
			}
		}
	}

	// y+
	for (int y = ySize-2; y >= 0; y--) {
		for (int z = 0; z < zSize; z++) {
			for (int x = 0; x < xSize; x++) {
				int idx = x + xSize*(y + ySize*z);
				cd[idx] = std::min(cd[idx], cd[idx + xSize] + 1);
			}
		}
	}

	// z-
	for (int z = 1; z < zSize; z++) {
		for (int y = 0; y < ySize; y++) {
			for (int x = 0; x < xSize; x++) {
				int idx = x + xSize*(y + ySize*z);
				cd[idx] = std::min(cd[idx], cd[idx - xSize*ySize] + 1);
			}
		}
	}

	// z+
	for (int z = zSize-2; z >= 0; z--) {
		for (int y = 0; y < ySize; y++) {
			for (int x = 0; x < xSize; x++) {
				int idx = x + xSize*(y + ySize*z);
				cd[idx] = std::min(cd[idx], cd[idx + xSize*ySize] + 1);
			}
		}
	}

	// 2. Partition cells into bins
	// Counts (at most xSize+ySize, but let's count it)
	int numBuckets = 0;
	for (int i = 0; i < xSize*ySize*zSize; i++) {
		if (cd[i] > numBuckets) {
			numBuckets = cd[i];
		}
	}
	numBuckets++;

	int* bucketCounts = new int[numBuckets];
	for (int i = 0; i < numBuckets; i++) {
		bucketCounts[i] = 0;
	}
	for (int i = 0; i < xSize*ySize*zSize; i++) {
		bucketCounts[cd[i]]++;
	}

	// Prefix sum
	int* sums = new int[numBuckets];
	sums[0] = 0;
	for (int i = 1; i < numBuckets; i++) {
		sums[i] = sums[i - 1] + bucketCounts[i - 1];
	}

	assert(sums[numBuckets - 1] + bucketCounts[numBuckets - 1] == xSize*ySize*zSize);

	int* indices = new int[xSize*ySize*zSize * 3];
	for (int z = 0; z < zSize; z++) {
		for (int y = 0; y < ySize; y++) {
			for (int x = 0; x < xSize; x++) {
				int c = cd[x + xSize*(y + ySize*z)];
				int index = sums[c];
				indices[3 * index] = x;
				indices[3 * index + 1] = y;
				indices[3 * index + 2] = z;
				sums[c]++;
			}
		}
	}

	// 3. Extrapolate values from the inside out.
	int directions[18] = {1,0,0, -1,0,0, 0,1,0, 0,-1,0, 0,0,1, 0,0,-1};
	int numDirs = 6;
	for (int i = 0; i < xSize*ySize*zSize; i++) {
		int x = indices[3 * i];
		int y = indices[3 * i + 1];
		int z = indices[3 * i + 2];
		int myc = cd[x + xSize*(y+ySize*z)];
		if (myc == 0) continue; // don't modify known values
		float numNeighbors = 0.0f;
		float neighborSum = 0.0f;

		for (int d = 0; d < numDirs; d++) {
			int nx = x + directions[3 * d];
			int ny = y + directions[3 * d + 1];
			int nz = z + directions[3 * d + 2];
			int idx = nx + xSize*(ny + ySize*nz);
			if (0 <= nx && nx < xSize 
				&& 0 <= ny && ny < ySize 
				&& 0 <= nz && nz < zSize
				&& (cd[idx] < myc)) {
				numNeighbors += 1.0f;
				neighborSum += srcAr[idx];
			}
		}
		assert(numNeighbors > 0.0f);
		srcAr[x + xSize*(y+ySize*z)] = neighborSum / numNeighbors;
	}

	// Clean up
	delete[] cd;
	delete[] bucketCounts;
	delete[] sums;
	delete[] indices;

	// and that's it!
}

void FluidSim3::AddBodyForces(float dt) {
	// Just adds dt*g to the velocity field
	// We use Cartesian coordinates for our grids, hooray!
	int count = mX*(mY + 1)*mZ;
	float gdT = m_gY*dt; // in m/s
	for (int i = 0; i < count; ++i) {
		m_MV[i] += gdT;
	}
}

void FluidSim3::Project(float dt) {
	//... this is basically Chapter 5 of Bridson.

	// At the heart of this projection routine, we need to solve a set of equations, of the form
	// 6 p_{i,j} - p_{i+1,j,k} - p_{i-1,j,k} 
	//           - p_{i,j+1,k} - p_{i,j-1,k}
	//           - p_{i,j,k+1} - p_{i,j,k-1} = -rhs[i,j]
	// where rhs[i,j] is the negative discrete divergence at (i,j), times dx^2*rho/dt:
	// rhs[i,j]=-dx*rho/dt (u{i+1/2,j,k}-u{i-1/2,j,k}+v{i,j+1/2,k}-v{i,j-1/2,k}+w{i,j,k+1/2}-w{i,j,k-1/2}).

	// This equation is modified at the following way at the boundaries:
	// (see Bridson's lecture notes or the 2nd ed. of the book, pg. 76 for a derivation:)

	// - For each bordering solid cell, reduce the coefficient 6 by 1 and remove the reference
	// to that p. Also, use u_solid inside the solid material (which in this case we take to be 0)
	//
	// - For each bordering air cell, just remove the reference to that p.

	// We now solve this linear system with the Gauss-Seidel method, using a checkerboard update.
	// Our operations are:
	// x |-> Dx: divide by number of non-solid neighbors
	// x |-> Rx: replace x_{i,j} with -(sum(water neighbors' pressures)).

	// Now, we'll use the level set calculations, using the method on pg. 128 of Bridson, 2nd ed.
	// Bridson never writes the equation out, but the entire effect of this really is to increase
	// the coefficient on the diagonal from 6 (in the all-fluid case) to 6-phi_{i+1,j}/phi_{i,j}.
	// (This is an increase, because phi_{i+1,j} and phi_i,j} have different signs.)
	// The right hand side is unchanged, and remains the discrete divergence.

	// For now, this is simple enough that we can just write it inside a loop.

	// For Gauss-Seidel, we just do Jacobi's method, but we just only use one array of pressures.

	double maxLSRatio = 1000.0; // What should we clamp -phi_{i+1,j}/phi{i,j} to?

								// I. Compute the right-hand side b.
	int MN = mX*mY*mZ;
	double* b = new double[MN];
	double* p = new double[MN](); // should zero everything for us
	double* diagCoeffs = new double[MN];

	// INDEXING: b[x,y,z] = b[x+mX*(y+mY*z)].

	double solidVel = 0;
	double dx = 1.0 / m_CellsPerMeter;
	double scale = -dx*m_rho / dt;

	for (int z = 0; z < mZ; z++) {
		for (int y = 0; y < mY; y++) {
			for (int x = 0; x < mX; x++) {
				double velXp = (x == mX - 1 ? solidVel : U(x + 1, y, z));
				double velXm = (x == 0 ? solidVel : U(x, y, z));
				double velYp = (y == mY - 1 ? solidVel : V(x, y + 1, z));
				double velYm = (y == 0 ? solidVel : V(x, y, z));
				double velZp = (z == mZ - 1 ? solidVel : W(x, y, z + 1));
				double velZm = (z == 0 ? solidVel : W(x, y, z));
				b[x + mX*(y+mY*z)] = scale*(velXp - velXm + velYp - velYm + velZp - velZm);
			}
		}
	}

	// TEMP DEBUG
	/*for (int z = 0; z < mZ; z++) {
		for (int y = 0; y < mY; y++) {
			for (int x = 0; x < mX; x++) {
				Phi(x, y, z) = -1.0f; // everything is water
			}
		}
	}*/

	// Compute diagonal coefficients.
	for (int z = 0; z < mZ; z++) {
		for (int y = 0; y < mY; y++) {
			for (int x = 0; x < mX; x++) {
				// We still don't solve for pressures in air.
				if (Phi(x, y, z) >= 0.0) continue;

				// = # of non-solid neighbors plus level set things
				double numNeighbors = 0;
				if (x != 0) {
					numNeighbors += 1;
					if (Phi(x - 1, y, z) > 0.0) {
						// Plus fluid fractions thing
						numNeighbors += MathHelper::Clamp(-(double)Phi(x - 1, y, z) / Phi(x, y, z), 0.0, maxLSRatio);
					}
				}

				if (x != mX - 1) {
					numNeighbors += 1;
					if (Phi(x + 1, y, z) > 0.0) {
						numNeighbors += MathHelper::Clamp(-(double)Phi(x + 1, y, z) / Phi(x, y, z), 0.0, maxLSRatio);
					}
				}

				if (y != 0) {
					numNeighbors += 1;
					if (Phi(x, y - 1, z) > 0.0) {
						numNeighbors += MathHelper::Clamp(-(double)Phi(x, y - 1, z) / Phi(x, y, z), 0.0, maxLSRatio);
					}
				}

				if (y != mY - 1) {
					numNeighbors += 1;
					if (Phi(x, y + 1, z) > 0.0) {
						numNeighbors += MathHelper::Clamp(-(double)Phi(x, y + 1, z) / Phi(x, y, z), 0.0, maxLSRatio);
					}
				}

				if (z != 0) {
					numNeighbors += 1;
					if (Phi(x, y, z - 1) > 0.0) {
						numNeighbors += MathHelper::Clamp(-(double)Phi(x, y, z - 1) / Phi(x, y, z), 0.0, maxLSRatio);
					}
				}

				if (z != mZ - 1) {
					numNeighbors += 1;
					if (Phi(x, y, z + 1) > 0.0) {
						numNeighbors += MathHelper::Clamp(-(double)Phi(x, y, z + 1) / Phi(x, y, z), 0.0, maxLSRatio);
					}
				}

				assert(numNeighbors > 0.0);

				diagCoeffs[x + mX*(y + mY*z)] = numNeighbors;
			}
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

	// For a 3D grid with a dambreak scenario on the first frame and 100 iterations, 
	// we wind up getting
	// size    min omega    max divergence  L2 norm
	// 16      1.808+-0.023 6.817e-07       0.000000
	// 32      1.895        1.239e-05       0.000725
	// 64      1.951        6.652e-03       0.979345
	// Quadratically fitting this to a formula of the form 2-c/x, we get
	// c = (2*3-1.808-1.895-1.951)/(1/16+1/32+1/64) = 3.16343
	// check:               max divergence  L2 norm
	// 2-3.16343/16 = 1.802 1.583e-08       0.000000
	// 2-3.16343/32 = 1.901 3.807e-05       0.003555
	// 2-3.16343/64 = 1.951 6.652e-03       0.979345
	// This is really close to the 3.22133 we get for the 2D case
	// in a different scenario!

	double omega = 2 - 3.16343 / mX;

	int numIterations = 100;

	for (int iter = 0; iter < numIterations; iter++) {
		for (int stage = 0; stage <= 1; stage++) {
			for (int z = 0; z < mZ; z++) {
				for (int y = 0; y < mY; y++) {
					for (int x = 0; x < mX; x++) {
						// Checkerboard iteration
						if (((x + y + z) % 2) != stage) continue;

						// Gauss-Seidel update p[x,y].
						int idx = x + mX * (y + mY * z);
						double numNeighbors = diagCoeffs[idx];
						double neighborMinusSum = 0;

						// If this cell is air, then there's no equation for this cell - 
						if (Phi(x, y, z) >= 0.0f) continue;

						if (x != 0) {
							if (Phi(x - 1, y, z) < 0.0f) {
								neighborMinusSum -= p[idx - 1];
							}
						}
						if (x != mX - 1) {
							if (Phi(x + 1, y, z) < 0.0f) {
								neighborMinusSum -= p[idx + 1];
							}
						}
						if (y != 0) {
							if (Phi(x, y - 1, z) < 0.0f) {
								neighborMinusSum -= p[idx - mX];
							}
						}
						if (y != mY - 1) {
							if (Phi(x, y + 1, z) < 0.0f) {
								neighborMinusSum -= p[idx + mX];
							}
						}
						if (z != 0) {
							if (Phi(x, y, z - 1) < 0.0f) {
								neighborMinusSum -= p[idx - mX * mY];
							}
						}
						if (z != mZ - 1) {
							if (Phi(x, y, z + 1) < 0.0f) {
								neighborMinusSum -= p[idx + mX * mY];
							}
						}

						// Successive over-relaxation
						p[idx] = (1 - omega)*p[idx] + omega * (b[idx] - neighborMinusSum) / numNeighbors;
					}
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
	SetEdgeVelocitiesToZero();

	// Interior
	scale = dt / (m_rho*dx);
	// U
	for (int z = 0; z < mZ; z++) {
		for (int y = 0; y < mY; y++) {
			for (int x = 0; x < mX - 1; x++) {
				int idx = x + mX * (y + mY * z);
				double phiL = Phi(x, y, z);
				double phiR = Phi(x + 1, y, z);
				// Four cases:
				if (phiL < 0.0 && phiR < 0.0) {
					U(x + 1, y, z) = (float)(U(x + 1, y, z) - scale * (p[idx + 1] - p[idx]));
				}
				else if (phiL < 0.0 && phiR >= 0.0) {
					U(x + 1, y, z) = (float)(U(x + 1, y, z) + scale * (1 + MathHelper::Clamp(-phiR / phiL, 0.0, maxLSRatio))*p[idx]);
				}
				else if (phiL >= 0.0 && phiR < 0.0) {
					// I think this is right (...it seems to be?) (It was not.)
					U(x + 1, y, z) = (float)(U(x + 1, y, z) - scale * (1 + MathHelper::Clamp(-phiL / phiR, 0.0, maxLSRatio))*p[idx + 1]);
				}
				else {
					// In air
					U(x + 1, y, z) = 0;
				}
			}
		}
	}
	// V
	for (int z = 0; z < mZ; z++) {
		for (int y = 0; y < mY - 1; y++) {
			for (int x = 0; x < mX; x++) {
				int idx = x + mX * (y + mY * z);
				double phiD = Phi(x, y, z);
				double phiU = Phi(x, y + 1, z);
				if (phiD < 0.0 && phiU < 0.0) {
					V(x, y + 1, z) = (float)(V(x, y + 1, z) - scale * (p[idx + mX] - p[idx]));
				}
				else if (phiD < 0.0 && phiU >= 0.0) {
					V(x, y + 1, z) = (float)(V(x, y + 1, z) + scale * (1 + MathHelper::Clamp(-phiU / phiD, 0.0, maxLSRatio))*p[idx]);
				}
				else if (phiD >= 0.0 && phiU < 0.0) {
					V(x, y + 1, z) = (float)(V(x, y + 1, z) - scale * (1 + MathHelper::Clamp(-phiD / phiU, 0.0, maxLSRatio))*p[idx + mX]);
				}
				else {
					V(x, y + 1, z) = 0;
				}
			}
		}
	}
	// W
	for (int z = 0; z < mZ - 1; z++) {
		for (int y = 0; y < mY; y++) {
			for (int x = 0; x < mX; x++) {
				int idx = x + mX * (y + mY * z);
				double phiD = Phi(x, y, z);
				double phiU = Phi(x, y, z + 1);
				if (phiD < 0.0 && phiU < 0.0) {
					W(x, y, z + 1) = (float)(W(x, y, z + 1) - scale * (p[idx + mX * mY] - p[idx]));
				}
				else if (phiD < 0.0 && phiU >= 0.0) {
					W(x, y, z + 1) = (float)(W(x, y, z + 1) + scale * (1 + MathHelper::Clamp(-phiU / phiD, 0.0, maxLSRatio))*p[idx]);
				}
				else if (phiD >= 0.0 && phiU < 0.0) {
					W(x, y, z + 1) = (float)(W(x, y, z + 1) - scale * (1 + MathHelper::Clamp(-phiD / phiU, 0.0, maxLSRatio))*p[idx + mX * mY]);
				}
				else {
					W(x, y, z + 1) = 0;
				}
			}
		}
	}
	// end (modifies internal state).

	// odprintf("%f", omega);
	// PrintDivergence(); //<- this may only give accurate results if Phi<0 for all x, y, and z

	delete[] b;
	delete[] p;
	delete[] diagCoeffs;
}

void FluidSim3::PrintDivergence() {
	// DEBUG CODE
	// Measure the divergence of the resulting vector field.
	// After enough iterations, this should be 0.
	// In practice...
	double l2Sum = 0.0;
	double maxDivergence = 0.0;
	int mdX, mdY, mdZ;
	float* divs = new float[mX*mY*mZ];

	for (int z = 0; z < mZ; z++) {
		for (int y = 0; y < mY; y++) {
			for (int x = 0; x < mX; x++) {
				if (Phi(x, y, z) >= 0.0) {
					divs[x + mX * (y + mY * z)] = 0;
				}
				else {
					// don't forget - it's a y-up coordinate system!
					float velUp = V(x, y + 1, z);
					float velDown = V(x, y, z);
					float velLeft = U(x, y, z);
					float velRight = U(x + 1, y, z);
					float velFwds = W(x, y, z + 1);
					float velBkwds = W(x, y, z);
					float div = velUp - velDown + velRight - velLeft + velFwds - velBkwds;
					divs[x + mX * (y + mY * z)] = div;
					if (div > maxDivergence) {
						mdX = x;
						mdY = y;
						mdZ = z;
						maxDivergence = div;
					}
					l2Sum += div * div;
				}
			}
		}
	}

	odprintf("L2 norm of divergence was %lf.", sqrt(l2Sum));
	odprintf("Maximum divergence was    %.3e", maxDivergence);
	odprintf("which was at {%i, %i, %i}.", mdX, mdY, mdZ);

	delete[] divs;
}

void FluidSim3::SetEdgeVelocitiesToZero() {
	// U
	for (int z = 0; z < mZ; z++) {
		for (int y = 0; y < mY; y++) {
			U(0, y, z) = 0;
			U(mX, y, z) = 0;
		}
	}
	// V
	for (int z = 0; z < mZ; z++) {
		for (int x = 0; x < mX; x++) {
			V(x, 0, z) = 0;
			V(x, mY, z) = 0;
		}
	}
	// W
	for (int y = 0; y < mY; y++) {
		for (int x = 0; x < mX; x++) {
			W(x, y, 0) = 0;
			W(x, y, mZ) = 0;
		}
	}
}