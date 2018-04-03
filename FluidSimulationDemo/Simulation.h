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

	template<typename T> void UploadTex(ID3D11Texture3D* destTex, T* srcBuf, int sizeX, int sizeY);
	template<typename T> T* RetrieveTex(ID3D11Texture3D* srcTex, ID3D11Texture3D* stagingTex, int sizeX, int sizeY, int sizeZ);
	template<typename T> T* RetrieveBuffer(ID3D11Buffer* srcBuf, ID3D11Buffer* stagingBuf, int len); // Retrieves a raw typed buffer from the GPU.
	void UploadParticles(ID3D11Buffer* bfrPtr, std::vector<Particle3> srcParticles);

	// Utility
	void SetParametersConstantBuffer(float dt, float alpha, int slot);
private:
	void AdvectGPU(float dt);
	void TransferParticlesToGridGPU();
	void AddBodyForcesGPU(float dt);
	void ProjectGPU(float dt);
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
	ID3D11Buffer * m_gpParticles = nullptr;
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
public:
	UINT numParticles;
};

#endif
