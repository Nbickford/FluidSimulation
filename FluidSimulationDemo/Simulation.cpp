//*********************************************************
// Simulation.cpp
// Top level of code for doing fluid simulation on the CPU.
//
// Authors:
//   Neil Bickford
//*********************************************************

#include "Simulation.h"

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
			XMFLOAT2 vc = vectorCurl((x-0.5f)/CellsPerMeter, y/CellsPerMeter);
			U(x, y) = vc.x;
		}
	}

	// MV
	for (int y = 0; y < ySize + 1; y++) {
		for (int x = 0; x < xSize; x++) {
			XMFLOAT2 vc = vectorCurl(x/CellsPerMeter, (y - 0.5f)/CellsPerMeter);
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
}

FluidSim::~FluidSim() {
	m_particles.clear();
	delete[] m_MU;
	delete[] m_MV;
}

void FluidSim::Simulate(float dt) {
	// For the moment, we just have a particle trace a straight line.
	// Suppose a particle moves with velocity (ux, uy) m/s.
	// Then the particle moves (ux,uy)*dt meters in dt seconds.
	int len = m_particles.size();
	for (int i = 0; i < len; i++) {
		XMFLOAT2 nv = vectorCurl(m_particles[i].X, m_particles[i].Y);
		m_particles[i].uX = nv.x;
		m_particles[i].uY = nv.y;
		m_particles[i].X += dt*m_particles[i].uX;
		m_particles[i].Y += dt*m_particles[i].uY;
	}
}

float peaks(float x, float y) {
	// See An Introduction to Optimization, 4th Edition, pg. 290
	return 3.0f*(1.0f - x)*(1.0f - x)*expf(-x*x - (y + 1.f)*(y + 1.f))
		- 10.f*(0.2f*x - x*x*x + y*y*y*y*y)*expf(-x*x - y*y)
		- expf(-(x + 1.0f)*(x + 1.0f) - y*y) / 3.0f;
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
	float scale = powf(max(0.0f,1.0f - max(abs(x), abs(y)) / 3.0f), 8.0f);
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