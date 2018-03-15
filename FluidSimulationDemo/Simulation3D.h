//************************************************************
// Simulation3D.h
// Top level of code for doing 3D fluid simulation on the CPU.
//
// Authors:
//   Neil Bickford
//************************************************************

#ifndef FSD_SIMULATION3D_H
#define FSD_SIMULATION3D_H

#include <vector>
#include "MathHelper.h"
using namespace DirectX;

// Represents a fluid particle.
struct Particle3 {
	// Positions in real-world units (meters)
	float X;
	float Y;
	float Z;
	// Velocity in real-world units (m/s)
	float uX;
	float uY;
	float uZ;

	Particle3()
		:X(0.f), Y(0.f), Z(0.f), uX(0.f), uY(0.f), uZ(0.f) {
	}

	Particle3(float px, float py, float pz, XMFLOAT3 vel)
		:X(px), Y(py), Z(pz), uX(vel.x), uY(vel.y), uZ(vel.z) {
	}

	Particle3(float px, float py, float pz, float ux, float uy, float uz)
		:X(px), Y(py), Z(pz), uX(ux), uY(uy), uZ(uz) {
	}
};

// Represents the current state of a fluid simulation.
class FluidSim3 {
public:
	FluidSim3(int xSize, int ySize, int zSize, float CellsPerMeter);
	~FluidSim3();
	void ResetSimulation();

	void Simulate(float dt);
private:
	void Advect(std::vector<Particle3> &particles, float dt);
	float ptDistance(float x0, float y0, float z0, float x1, float y1, float z1);
	void clsInner(int dx, int dy, int dz, int x, int y, int z, int* closestParticles);
	void ComputeLevelSet(const std::vector<Particle3> &particles);
	void TransferParticlesToGrid(std::vector<Particle3> &particles);
	void ExtrapolateValues(float* srcAr, bool* validAr, int xSize, int ySize, int zSize);
	void AddBodyForces(float dt);
	void Project(float dt);
	void SetEdgeVelocitiesToZero();
private:
	// Looks up points in their respective MAC grids.
	// See Bridson, 2nd Ed., p.25.
	// u(i,j,k) = u_{i-1/2,j} (Returns a reference)
	float& U(int i, int j, int k) { return m_MU[i + (mX + 1)*(j + mY*k)]; }

	// v(i,j,k) = v_{i,j-1/2,k} (Returns a reference)
	float& V(int i, int j, int k) { return m_MV[i + mX*(j + (mY + 1)*k)]; }

	// w(i,j,k) = u_{i,j,k-1/2} (Returns a reference)
	float& W(int i, int j, int k) { return m_MW[i + mX*(j + mY*k)]; }



	// phi(i,j,k) = phi_{i,j,k} (Returns a reference)
	float& Phi(int i, int j, int k) { return m_Phi[i + mX*(j+mY*k)]; };

	// NOTE: i, j, AND K ARE ARRAY INDICES - NOT POSITIONS!
	XMFLOAT3 InterpolateMACCell(float i, float j, float k) {
		// Interpolating between velocities on a MAC grid...gets a bit tricky.
		// It's easiest to follow this if you draw a MAC grid in 2D and follow it out.

		// Compute indices and fractional parts
		// Normal values (relative to non-I, J, or K grid)
		float nI = MathHelper::Clamp(i, 0.0f, mX - 1.0f);
		float nJ = MathHelper::Clamp(j, 0.0f, mY - 1.0f);
		float nK = MathHelper::Clamp(k, 0.0f, mZ - 1.0f);
		// Extended values (relative to grid in that direction)
		float eI = MathHelper::Clamp(i + 0.5f, 0.0f, static_cast<float>(mX));
		float eJ = MathHelper::Clamp(j + 0.5f, 0.0f, static_cast<float>(mY));
		float eK = MathHelper::Clamp(k + 0.5f, 0.0f, static_cast<float>(mZ));

		// Compute normal and truncated integer parts and normal and truncated fractional parts
		int iI = (int)std::floor(nI); if (iI == mX - 1) iI--;
		int iJ = (int)std::floor(nJ); if (iJ == mY - 1) iJ--;
		int iK = (int)std::floor(nK); if (iK == mZ - 1) iK--;
		int iEI = (int)std::floor(eI); if (iEI == mX) iEI--;
		int iEJ = (int)std::floor(eJ); if (iEJ == mY) iEJ--;
		int iEK = (int)std::floor(eK); if (iEK == mZ) iEK--;

		float fI = nI - iI;
		float fJ = nJ - iJ;
		float fK = nK - iK;
		float fEI = eI - iEI;
		float fEJ = eJ - iEJ;
		float fEK = eK - iEK;

		// Trilinear interpolation for u
		// We basically just trilinearly interpolate (eI, J) on the MAC U grid.
		// Interpolate along i
		float t00 = MathHelper::Lerp(U(iEI, iJ, iK), U(iEI + 1, iJ, iK), fEI);
		float t10 = MathHelper::Lerp(U(iEI, iJ + 1, iK), U(iEI + 1, iJ + 1, iK), fEI);
		float t01 = MathHelper::Lerp(U(iEI, iJ, iK + 1), U(iEI + 1, iJ, iK + 1), fEI);
		float t11 = MathHelper::Lerp(U(iEI, iJ + 1, iK + 1), U(iEI + 1, iJ + 1, iK + 1), fEI);

		// Interpolate along j
		float tx0 = MathHelper::Lerp(t00, t10, fJ);
		float tx1 = MathHelper::Lerp(t01, t11, fJ);

		// Interpolate along k
		float uFinal = MathHelper::Lerp(tx0, tx1, fK);

		// OK! Now that you've got the hang of it, here's trilinear interpolation for v.
		// Interpolate (I, eJ) on the MAC V grid.
		t00 = MathHelper::Lerp(V(iI, iEJ, iK), V(iI + 1, iEJ, iK), fI);
		t10 = MathHelper::Lerp(V(iI, iEJ + 1, iK), V(iI + 1, iEJ + 1, iK), fI);
		t01 = MathHelper::Lerp(V(iI, iEJ, iK + 1), V(iI, iEJ, iK + 1), fI);
		t11 = MathHelper::Lerp(V(iI, iEJ + 1, iK + 1), V(iI, iEJ + 1, iK + 1), fI);

		tx0 = MathHelper::Lerp(t00, t10, fEJ);
		tx1 = MathHelper::Lerp(t01, t11, fEJ);

		float vFinal = MathHelper::Lerp(tx0, tx1, fJ);

		// Interpolate (I, J, eK) on the MAC W grid.
		t00 = MathHelper::Lerp(W(iI, iJ, iEK), V(iI + 1, iJ, iEK), fI);
		t10 = MathHelper::Lerp(W(iI, iJ + 1, iEK), V(iI + 1, iJ + 1, iEK), fI);
		t01 = MathHelper::Lerp(W(iI, iJ, iEK + 1), W(iI + 1, iJ, iEK + 1), fI);
		t11 = MathHelper::Lerp(W(iI, iJ + 1, iEK + 1), W(iI + 1, iJ + 1, iEK + 1), fI);

		tx0 = MathHelper::Lerp(t00, t10, fJ);
		tx1 = MathHelper::Lerp(t01, t11, fJ);

		float wFinal = MathHelper::Lerp(tx0, tx1, fEJ);

		return XMFLOAT3(uFinal, vFinal, wFinal);
	}

private:
	// Extents of the fluid simulation in cells
	int mX;
	int mY;
	int mZ;

	// Cells per meter
	float m_CellsPerMeter;

	// Density of water (in kg/m^3)
	float m_rho = 1000.0f;

	// Acceleration due to gravity (in m/s^2 upwards)
	float m_gY = -9.81f;

	// Kinematic viscosity of water (in m^2/s)
	float m_nu = 8.90f*1e-4f / 1000.0f; //(=dynamic viscosity/density, both of which are constant) 

										// Particle radius
	float m_pRadius = 2 / sqrtf(2.0f); // Needs to be at least sqrt(2)/2 to avoid particles
									   // getting stuck between cells classified as air

	// MAC velocity grids.
	// These are all stored in row-major order, so that X is the least significant part.
	// X component of velocity on an (mX+1)*mY*mZ grid.
	float* m_MU;
	// Y component of velocity on an mX*(mY+1)*mZ grid.
	float* m_MV;
	// Z component of velocity on an mX*mY*(mZ+1) grid.
	float* m_MW;

	// Level set (sampled at point centers, size mX*mY)
	// Note: = DISTANCE IN GRID CELLS
	float* m_Phi;

public: // For visualization, for the moment - we could of course execute rendering commands from here though
		// List of particles
	std::vector<Particle3> m_particles;
};

#endif
