//*********************************************************
// Simulation.h
// Top level of code for doing fluid simulation on the CPU.
//
// Authors:
//   Neil Bickford
//*********************************************************

#ifndef FSD_SIMULATION_H
#define FSD_SIMULATION_H

#include <vector>
#include "MathHelper.h"
using namespace DirectX;

// Represents a fluid particle.
struct Particle {
	// Positions in real-world units (meters)
	float X;
	float Y;
	// Velocity in real-world units (m/s)
	float uX;
	float uY;

	Particle()
		:X(0.f), Y(0.f), uX(0.f), uY(0.f) {
	}

	Particle(float px, float py, XMFLOAT2 vel)
	:X(px), Y(py), uX(vel.x), uY(vel.y){
	}
};

// Represents the current state of a fluid simulation.
class FluidSim {
public:
	FluidSim(int xSize, int ySize, float CellsPerMeter);
	~FluidSim();
	//FluidSim(int xSize, int ySize, int zSize, float CellsPerMeter);

	// Returns a NEW character array representing a visualization of a texture
	// in ARGB format with dimensions nX*nY.
	// Wait, why am I even doing this in 3D already???
	//char* CreateTextureVisualization() const;
	void Simulate(float dt);
private:
	//******************************************************
	// 2D METHODS
	//******************************************************
	// Looks up points in their respective MAC grids.
	// See Bridson, 2nd Ed., p.25.
	// u(i,j,k) = u_{i-1/2,j} (Returns a reference)
	float& U(int i, int j) { return m_MU[i + (mX + 1)*j]; }

	// v(i,j,k) = u_{i,j-1/2,k} (Returns a reference)
	float& V(int i, int j) { return m_MV[i + mX*j]; }

	XMFLOAT2 InterpolateMACCell(float i, float j) {
		// Interpolating between velocities on a MAC grid...gets a bit tricky.
		// It's easiest to follow this if you draw a MAC grid in 2D and follow it out.

		// U
		// Compute indices and fractional parts
		// Truncated values (relative to non-I, J, or K grid)
		float tI = MathHelper::Clamp(i - 0.5f, 0.0f, mX - 1.0f);
		float tJ = MathHelper::Clamp(j - 0.5f, 0.0f, mY - 1.0f);

		// Compute normal and truncated integer parts and normal and truncated fractional parts
		int iI = (int)std::floor(i); if (iI == mX) iI--;
		int iJ = (int)std::floor(j); if (iJ == mY) iJ--;
		int iTI = (int)std::floor(tI); if (iTI == mX - 1) iTI--;
		int iTJ = (int)std::floor(tJ); if (iTJ == mY - 1) iTJ--;

		float fI = i - iI;
		float fJ = j - iJ;
		float fTI = tI - iTI;
		float fTJ = tJ - iTJ;

		// Trilinear interpolation for u
		// We basically just trilinearly interpolate (i, tJ) on the MAC U grid.
		// Interpolate along i
		float t00 = MathHelper::Lerp(U(iI, iTJ), U(iI + 1, iTJ), fI);
		float t10 = MathHelper::Lerp(U(iI, iTJ + 1), U(iI + 1, iTJ + 1), fI);

		// Interpolate along j
		float uFinal = MathHelper::Lerp(t00, t10, fTJ);

		// OK! Now that you've got the hang of it, here's trilinear interpolation for v.
		// Realistically, we could probably shorten this by a factor of 2 by just using the fact
		// that U and V are just staggered grids, and writing a single function for that.
		// This is also a really good reason for implementing this on the GPU - we get bilinear
		// interpolation, at least, for "free".
		// Interpolate (tI, j) on the MAC V grid.
		t00 = MathHelper::Lerp(V(iTI, iJ), V(iTI + 1, iJ), fTI);
		t10 = MathHelper::Lerp(V(iTI, iJ + 1), V(iTI + 1, iJ + 1), fTI);

		float vFinal = MathHelper::Lerp(t00, t10, fJ);

		return XMFLOAT2(uFinal, vFinal);
	}

	/*	// Looks up points in their respective MAC grids.
	// See Bridson, 2nd Ed., p.25.
	// u(i,j,k) = u_{i-1/2,j,k} (Returns a reference)
	float& U(int i, int j, int k) { return m_MU[i + (mX + 1)*(j + mY*k)]; }

	// v(i,j,k) = u_{i,j-1/2,k} (Returns a reference)
	float& V(int i, int j, int k) { return m_MV[i + mX*(j + (mY + 1)*k)]; }

	// w(i,j,k) = u_{i,j,k-1/2} (Returns a reference)
	float& W(int i, int j, int k) { return m_MW[i + mX*(j + mY*k)]; }*/

	// Interpolates between points on the MAC grid to get the
	// velocity at (i,j,k) in cell units.
	// Uses simple linear interpolation at the moment.
	/*XMFLOAT3 InterpolateMACCell(float i, float j, float k) {
		// Interpolating between velocities on a MAC grid...gets a bit tricky.
		// It's easiest to follow this if you draw a MAC grid in 2D and follow it out.

		// U
		// Compute indices and fractional parts
		// Truncated values (relative to non-I, J, or K grid)
		float tI = MathHelper::Clamp(i - 0.5f, 0.0f, mX - 1.0f);
		float tJ = MathHelper::Clamp(j - 0.5f, 0.0f, mY - 1.0f);
		float tK = MathHelper::Clamp(k - 0.5f, 0.0f, mZ - 1.0f);

		// Compute normal and truncated integer parts and normal and truncated fractional parts
		int iI = std::floor(i); if (iI == mX) iI--;
		int iJ = std::floor(j); if (iJ == mY) iJ--;
		int iK = std::floor(k); if (iK == mZ) iK--;
		int iTI = std::floor(tI); if (iTI == mX - 1) iTI--;
		int iTJ = std::floor(tJ); if (iTJ == mY - 1) iTJ--;
		int iTK = std::floor(tK); if (iTK == mZ - 1) iTK--;

		float fI = i - iI;
		float fJ = j - iJ;
		float fK = k - iK;
		float fTI = tI - iTI;
		float fTJ = tJ - iTJ;
		float fTK = tK - iTK;

		// Trilinear interpolation for u
		// We basically just trilinearly interpolate (i, tJ, tK) on the MAC U grid.
		// Interpolate along i
		float t00 = MathHelper::Lerp(U(iI, iTJ    , iTK    ), U(iI + 1, iTJ    , iTK    ), fI);
		float t01 = MathHelper::Lerp(U(iI, iTJ    , iTK + 1), U(iI + 1, iTJ    , iTK + 1), fI);
		float t10 = MathHelper::Lerp(U(iI, iTJ + 1, iTK    ), U(iI + 1, iTJ + 1, iTK    ), fI);
		float t11 = MathHelper::Lerp(U(iI, iTJ + 1, iTK + 1), U(iI + 1, iTJ + 1, iTK + 1), fI);

		// Interpolate along j
		float t0 = MathHelper::Lerp(t00, t10, fTJ);
		float t1 = MathHelper::Lerp(t01, t11, fTJ);

		// Interpolate along k
		float uFinal = MathHelper::Lerp(t0, t1, fTK);

		// OK! Now that you've got the hang of it, here's trilinear interpolation for v and w.
		// Realistically, we could probably shorten this by a factor of 3 by just using the fact
		// that U, V, and W are just staggered grids, and writing a single function for that.
		// This is also a really good reason for implementing this on the GPU - we get bilinear
		// interpolation, at least, for "free".
		// Interpolate (tI, j, tK) on the MAC V grid.
		t00 = MathHelper::Lerp(V(iTI, iJ    , iTK    ), V(iTI + 1, iJ    , iTK    ), fTI);
		t01 = MathHelper::Lerp(V(iTI, iJ    , iTK + 1), V(iTI + 1, iJ    , iTK + 1), fTI);
		t10 = MathHelper::Lerp(V(iTI, iJ + 1, iTK    ), V(iTI + 1, iJ + 1, iTK    ), fTI);
		t11 = MathHelper::Lerp(V(iTI, iJ + 1, iTK + 1), V(iTI + 1, iJ + 1, iTK + 1), fTI);

		t0 = MathHelper::Lerp(t00, t10, fJ);
		t1 = MathHelper::Lerp(t01, t11, tJ);

		float vFinal = MathHelper::Lerp(t0, t1, fTK);

		// Interpolate (tI, tJ, k) on the MAC W grid.
		t00 = MathHelper::Lerp(W(iTI, iTJ    , iK    ), W(iTI + 1, iTJ    , iK    ), fTI);
		t01 = MathHelper::Lerp(W(iTI, iTJ    , iK + 1), W(iTI + 1, iTJ    , iK + 1), fTI);
		t10 = MathHelper::Lerp(W(iTI, iTJ + 1, iK    ), W(iTI + 1, iTJ + 1, iK    ), fTI);
		t11 = MathHelper::Lerp(W(iTI, iTJ + 1, iK + 1), W(iTI + 1, iTJ + 1, iK + 1), fTI);

		t0 = MathHelper::Lerp(t00, t10, fTJ);
		t1 = MathHelper::Lerp(t01, t11, fTJ);

		float wFinal = MathHelper::Lerp(t0, t1, fK);

		return XMFLOAT3(uFinal, vFinal, wFinal);
	}*/
private:
	// Extents of the fluid simulation in cells
	int mX;
	int mY;
	// int mZ;

	// Cells per meter
	float m_CellsPerMeter;

	// Density of water (in kg/m^3)
	float m_rho = 1000.0f;

	// Acceleration due to gravity (in m/s^2 upwards)
	float m_gY = -9.81f;

	// Kinematic viscosity of water (in m^2/s)
	float m_nu = 8.90f*1e-4f / 1000.0f; //(=dynamic viscosity/density, both of which are constant) 
	
	// MAC velocity grids.
	// These are all stored in row-major order, so that X is the least significant part.
	// X component of velocity on an (mX+1)*mY*mZ grid.
	float* m_MU;
	// Y component of velocity on an mX*(mY+1)*mZ grid.
	float* m_MV;
	// Z component of velocity on an mX*mY*(mZ+1) grid.
	// float* m_MW;
	
public: // For visualization, for the moment - we could of course execute rendering commands from here though
	// List of particles
	std::vector<Particle> m_particles;
};


// Functions for generating peaks (just as a test to create a sample velocity field for advection)

// Matlab's "peaks" function
float peaks(float x, float y);

// Computes the 2D vector field N at the point (x,y)
XMFLOAT2 vectorFunction(float x, float y);

// Computes the vector potential psi in [Bridson, 2nd ed., p. 171]
// using N (vectorFunction) above.
XMFLOAT2 vectorPotential(float x, float y);

// Computes the curl of the vector potential at a given point (x,y).
// ...of course, we actually decided to just make this really ad-hoc just now, so ignore all the above.
XMFLOAT2 vectorCurl(float x, float y);

#endif