//************************************************************
// Simulation3D.h
// Top level of code for doing GPU-based 3D fluid simulation.
//
// Authors:
//   Neil Bickford
//************************************************************

#ifndef FSD_SIMULATION_H
#define FSD_SIMULATION_H

#include <vector>
#include "MathHelper.h"
#include "d3dUtil.h"
#include "ParticleDefs.h"
#include "FX11\d3dx11effect.h"
using namespace DirectX;

// Represents the current state of a fluid simulation.
class GPFluidSim {
public:
	GPFluidSim(int xSize, int ySize, int zSize, float CellsPerMeter);
	void Initialize(ID3D11Device* device, ID3D11DeviceContext* immediateContext);
	~GPFluidSim();
	void ResetSimulation();

	void Simulate(float dt);
private:
	// GPU resource acquisition and destruction
	void AcquireResources();
public:
	void ReleaseResources();
private:
	void CreateTexture3D(ID3D11Texture3D** texPtr, int width, int height, int depth, bool staging=false);
	void CreateTexture3D(ID3D11Texture3D** texPtr, int width, int height, int depth, bool staging, DXGI_FORMAT format);
	void CreateStructuredBuffer(ID3D11Buffer** bfrPtr, int stride, int numElements, bool staging=false);
	void Create3DSRV(ID3D11Texture3D* texPtr, ID3D11ShaderResourceView** srvPtr);
	void Create3DSRV(ID3D11Texture3D* texPtr, ID3D11ShaderResourceView** srvPtr, DXGI_FORMAT format);
	void CreateStructuredBufferSRV(ID3D11Buffer* bfrPtr, ID3D11ShaderResourceView** srvPtr, int numElements);
	void Create3DUAV(ID3D11Texture3D* texPtr, ID3D11UnorderedAccessView** uavPtr, int wSize);
	void Create3DUAV(ID3D11Texture3D* texPtr, ID3D11UnorderedAccessView** uavPtr, int wSize, DXGI_FORMAT format);
	void CreateStructuredBufferUAV(ID3D11Buffer* bfrPtr, ID3D11UnorderedAccessView** uavPtr, int numElements);
	void CreateConstantBuffer(ID3D11Buffer** bfrPtr, int byteLength);
	//void CompileAndCreateEffect(const std::wstring& filename, ID3DX11Effect** mFX);
	void CompileAndCreateCS(const std::wstring& filename, ID3D11ComputeShader** mFX);

	void UploadU();
	void UploadV();
	void UploadW();
	template<typename T> void UploadTex(ID3D11Texture3D* destTex, T* srcBuf, int sizeX, int sizeY);
	template<typename T> T* RetrieveTex(ID3D11Texture3D* srcTex, ID3D11Texture3D* stagingTex, int sizeX, int sizeY, int sizeZ);
	template<typename T> T* RetrieveBuffer(ID3D11Buffer* srcBuf, ID3D11Buffer* stagingBuf, int len); // Retrieves a raw typed buffer from the GPU.
	void UploadParticles(ID3D11Buffer* bfrPtr);

	// Utility
	void SetParametersConstantBuffer(float dt, float alpha, int slot);
private:
	void Advect(std::vector<Particle3> &particles, float dt);
	void AdvectGPU(float dt);
	float ptDistance(float x0, float y0, float z0, float x1, float y1, float z1);
	void clsInner(int dx, int dy, int dz, int x, int y, int z, int* closestParticles);
	void ComputeLevelSet(const std::vector<Particle3> &particles);
	void TransferParticlesToGridGPU();
	void TransferParticlesToGrid(std::vector<Particle3> &particles);
	void ExtrapolateValues(float* srcAr, bool* validAr, int xSize, int ySize, int zSize);
	void AddBodyForces(float dt);
	void AddBodyForcesGPU(float dt);
	void Project(float dt);
	void ProjectGPU(float dt);
	void SetEdgeVelocitiesToZero();

	void PrintDivergence();
private:
	// Looks up points in their respective MAC grids.
	// See Bridson, 2nd Ed., p.25.
	// u(i,j,k) = u_{i-1/2,j} (Returns a reference)
	float& U(int i, int j, int k) { return m_MU[i + (mX + 1)*(j + mY * k)]; }

	// v(i,j,k) = v_{i,j-1/2,k} (Returns a reference)
	float& V(int i, int j, int k) { return m_MV[i + mX * (j + (mY + 1)*k)]; }

	// w(i,j,k) = u_{i,j,k-1/2} (Returns a reference)
	float& W(int i, int j, int k) { return m_MW[i + mX * (j + mY * k)]; }



	// phi(i,j,k) = phi_{i,j,k} (Returns a reference)
	float& Phi(int i, int j, int k) { return m_Phi[i + mX * (j + mY * k)]; };

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
		t01 = MathHelper::Lerp(V(iI, iEJ, iK + 1), V(iI + 1, iEJ, iK + 1), fI);
		t11 = MathHelper::Lerp(V(iI, iEJ + 1, iK + 1), V(iI + 1, iEJ + 1, iK + 1), fI);

		tx0 = MathHelper::Lerp(t00, t10, fEJ);
		tx1 = MathHelper::Lerp(t01, t11, fEJ);

		float vFinal = MathHelper::Lerp(tx0, tx1, fK);

		// Interpolate (I, J, eK) on the MAC W grid.
		t00 = MathHelper::Lerp(W(iI, iJ, iEK), W(iI + 1, iJ, iEK), fI);
		t10 = MathHelper::Lerp(W(iI, iJ + 1, iEK), W(iI + 1, iJ + 1, iEK), fI);
		t01 = MathHelper::Lerp(W(iI, iJ, iEK + 1), W(iI + 1, iJ, iEK + 1), fI);
		t11 = MathHelper::Lerp(W(iI, iJ + 1, iEK + 1), W(iI + 1, iJ + 1, iEK + 1), fI);

		tx0 = MathHelper::Lerp(t00, t10, fJ);
		tx1 = MathHelper::Lerp(t01, t11, fJ);

		float wFinal = MathHelper::Lerp(tx0, tx1, fEK);

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
	float m_pRadius = 1.0f; // Needs to be at least sqrt(3)/2 to avoid particles
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

private:
	// GPU resources and devices
	ID3D11Device* md3dDevice;
	ID3D11DeviceContext* md3dImmediateContext;

	// MAC-size 3D resources
	ID3D11Texture3D* m_gpU;
	ID3D11Texture3D* m_gpV;
	ID3D11Texture3D* m_gpW;
	ID3D11Texture3D* m_gpOldU;
	ID3D11Texture3D* m_gpOldV;
	ID3D11Texture3D* m_gpOldW;
	ID3D11ShaderResourceView* m_gpUSRV;
	ID3D11ShaderResourceView* m_gpVSRV;
	ID3D11ShaderResourceView* m_gpWSRV;
	ID3D11ShaderResourceView* m_gpOldUSRV;
	ID3D11ShaderResourceView* m_gpOldVSRV;
	ID3D11ShaderResourceView* m_gpOldWSRV;
	ID3D11UnorderedAccessView* m_gpUUAV;
	ID3D11UnorderedAccessView* m_gpVUAV;
	ID3D11UnorderedAccessView* m_gpWUAV;
	ID3D11UnorderedAccessView* m_gpOldUUAV;
	ID3D11UnorderedAccessView* m_gpOldVUAV;
	ID3D11UnorderedAccessView* m_gpOldWUAV;
	
	// Normal-size 3D resources
	ID3D11Texture3D* m_gpCounts;
	ID3D11Texture3D* m_gpClosestParticles;
	ID3D11Texture3D* m_gpPhi;
	ID3D11Texture3D* m_gpProjectRHS;
	ID3D11Texture3D* m_gpDiagCoeffs;
	ID3D11Texture3D* m_gpProjectP;
	ID3D11ShaderResourceView* m_gpCountsSRV;
	ID3D11ShaderResourceView* m_gpClosestParticlesSRV;
	ID3D11ShaderResourceView* m_gpPhiSRV;
	ID3D11ShaderResourceView* m_gpProjectRHSSRV;
	ID3D11ShaderResourceView* m_gpDiagCoeffsSRV;
	ID3D11ShaderResourceView* m_gpProjectPSRV;
	ID3D11UnorderedAccessView* m_gpCountsUAV;
	ID3D11UnorderedAccessView* m_gpClosestParticlesUAV;
	ID3D11UnorderedAccessView* m_gpPhiUAV;
	ID3D11UnorderedAccessView* m_gpProjectRHSUAV;
	ID3D11UnorderedAccessView* m_gpDiagCoeffsUAV;
	ID3D11UnorderedAccessView* m_gpProjectPUAV;

	public:
	// Structured buffer; layout consists of consecutive Particle3s.
	ID3D11Buffer* m_gpParticles;
	ID3D11ShaderResourceView* m_gpParticlesSRV;
	ID3D11UnorderedAccessView* m_gpParticlesUAV;
	private:
	// Flattened buffer taking (x,y,z) index and returning a flattened array of particles.
	ID3D11Buffer* m_gpBinnedParticles;
	ID3D11ShaderResourceView* m_gpBinnedParticlesSRV;
	ID3D11UnorderedAccessView* m_gpBinnedParticlesUAV;

	// EFFECTS AND EFFECT-SPECIFIC CONSTANT BUFFERS
	ID3D11ComputeShader* m_gpAdvectFX;
	ID3D11ComputeShader* m_gpCountParticlesFX;
	ID3D11ComputeShader* m_gpClearIntArrayFX;
	ID3D11ComputeShader* m_gpBinParticlesFX;
	ID3D11ComputeShader* m_gpClearFloatArrayFX;
	ID3D11ComputeShader* m_gpComputeClosestParticleNeighborsFX;
	ID3D11ComputeShader* m_gpClosestParticlesSweepXmFX;
	ID3D11ComputeShader* m_gpClosestParticlesSweepXpFX;
	ID3D11ComputeShader* m_gpClosestParticlesSweepYmFX;
	ID3D11ComputeShader* m_gpClosestParticlesSweepYpFX;
	ID3D11ComputeShader* m_gpClosestParticlesSweepZmFX;
	ID3D11ComputeShader* m_gpClosestParticlesSweepZpFX;
	ID3D11ComputeShader* m_gpTransferParticleVelocitiesUFX;
	ID3D11ComputeShader* m_gpTransferParticleVelocitiesVFX;
	ID3D11ComputeShader* m_gpTransferParticleVelocitiesWFX;
	ID3D11ComputeShader* m_gpAddBodyForcesFX;
	ID3D11ComputeShader* m_gpProjectComputeRHSFX;
	ID3D11ComputeShader* m_gpProjectComputeDiagCoeffsFX;
	ID3D11ComputeShader* m_gpProjectIteration1FX;
	ID3D11ComputeShader* m_gpProjectIteration2FX;
	ID3D11ComputeShader* m_gpProjectToVelFX;
	ID3D11ComputeShader* m_gpUpdateParticleVelocitiesFX;
	ID3D11ComputeShader* m_gpExtrapolateParticleVelocitiesFX;

	// Parameters constant buffer
	ID3D11Buffer* m_gpParametersCB;

	// STAGING BUFFERS
	// UINT grid-size resource.
	ID3D11Texture3D* m_gpIntGridStage;
	// FLOAT grid-size resource
	ID3D11Texture3D* m_gpFloatGridStage;
	ID3D11Texture3D* m_gpFloatUStage; // U-size ((mX+1)*mY*mZ) resource
	ID3D11Texture3D* m_gpFloatVStage; // V-size (mX*(mY+1)*mZ) resource
	ID3D11Texture3D* m_gpFloatWStage; // W-size (mX*mY*(mZ+1)) resource
	// Tempporary: Use this for debugging GPU shaders.
	ID3D11Buffer* m_gpTemp;

	// SAMPLER STATES
	ID3D11SamplerState* m_gpLinearSampler;

public: // For visualization, for the moment - we could of course execute rendering commands from here though
		// List of particles
	std::vector<Particle3> m_particles;
};

#endif
