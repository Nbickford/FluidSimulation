//************************************************************
// Simulation3D.cpp
// Top level of code for doing 3D fluid simulation on the CPU.
//
// Authors:
//   Neil Bickford
//************************************************************

#include "Simulation.h"
#include "GPUProfiler.h"
#include "odprintf.h"
#include <limits>
#include <queue> // For serial extrapolation - see ExtrapolateValues(4).
#include <random>
#include <algorithm> // for std::min
#include <numeric> // for debug std::accumulate

//#include "debugroutines.h" TODO: Implement stb-like single-header library properly

// Don't forget to call Initialize after this!
GPFluidSim::GPFluidSim(int xSize, int ySize, int zSize, float CellsPerMeter)
	:mX(xSize), mY(ySize), mZ(zSize), m_CellsPerMeter(CellsPerMeter){
}

GPFluidSim::~GPFluidSim() {
	ReleaseResources();
}

void GPFluidSim::Initialize(ID3D11Device* device,
	ID3D11DeviceContext* immediateContext) {
	md3dDevice = device;
	md3dImmediateContext = immediateContext;

	AcquireResources();

	ResetSimulation();
}

void GPFluidSim::ResetSimulation() {
	std::minstd_rand generator(0); // Just an LCG; it doesn't need to be /that/ good...
	std::uniform_real_distribution<float> distribution(-0.25f, 0.25f);

	// Create a uniform distribution of particles
	// and set their velocities
	std::vector<float> newParticles;
	newParticles.reserve(4 * mX*mY*mZ);
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
							float m1 = rX + u * d + distribution(generator) / m_CellsPerMeter;
							float m2 = rY + v * d + distribution(generator) / m_CellsPerMeter;
							float m3 = rZ + w * d + distribution(generator) / m_CellsPerMeter;
							newParticles.push_back(m1);
							newParticles.push_back(m2);
							newParticles.push_back(m3);
							newParticles.push_back(0.0f);
							newParticles.push_back(0.0f);
							newParticles.push_back(0.0f);
						}
					}
				}
			}
		}
	}

	// Create array of particles if none exists, and set the particles.
	if (m_gpParticles == nullptr) {
		numParticles = static_cast<UINT>(newParticles.size()/6);
		CreateStructuredBuffer(&m_gpParticles, sizeof(Particle3), numParticles);
		CreateStructuredBufferSRV(m_gpParticles, &m_gpParticlesSRV, numParticles);
		CreateStructuredBufferUAV(m_gpParticles, &m_gpParticlesUAV, numParticles);
		CreateStructuredBuffer(&m_gpBinnedParticles, sizeof(Particle3), numParticles);
		CreateStructuredBufferSRV(m_gpBinnedParticles, &m_gpBinnedParticlesSRV, numParticles);
		CreateStructuredBufferUAV(m_gpBinnedParticles, &m_gpBinnedParticlesUAV, numParticles);
		// Staging buffers
		CreateStructuredBuffer(&m_gpTemp, sizeof(Particle3), numParticles, true);
	}

	UploadParticles(m_gpParticles, newParticles);
}

/// <summary>Uploads the (flattened) particles in srcParticles to the buffer pointed
/// to by bfrPtr.</summary>
void GPFluidSim::UploadParticles(ID3D11Buffer* bfrPtr, std::vector<float> srcParticles) {
	// Make sure that the particles can fit in the buffer.
	UINT len = static_cast<UINT>(srcParticles.size());
	D3D11_BUFFER_DESC bfrDesc;
	bfrPtr->GetDesc(&bfrDesc);
	UINT buffSize = bfrDesc.ByteWidth / sizeof(UINT);
	if (buffSize < len) {
		assert(0 && "GPFluidSim::UploadParticles: Length of particle vector was longer than buffer to copy to!");
		return;
	}
	
	float* tmpParticles = new float[buffSize];
	for (UINT i = 0; i < len; i++) {
		tmpParticles[i] = srcParticles[i];
	}
	md3dImmediateContext->UpdateSubresource(m_gpParticles, 0, 0,
		reinterpret_cast<const void*>(tmpParticles),
		sizeof(float)*buffSize, sizeof(float)*buffSize);
	delete[] tmpParticles;
}

// Creates the velocity fields and Texture3Ds needed to run the simulation on the GPU.
void GPFluidSim::AcquireResources() {
	

	// Velocity arrays
	CreateTexture3D(&m_gpU, mX + 1, mY, mZ);
	CreateTexture3D(&m_gpV, mX, mY + 1, mZ);
	CreateTexture3D(&m_gpW, mX, mY, mZ + 1);
	CreateTexture3D(&m_gpOldU, mX + 1, mY, mZ);
	CreateTexture3D(&m_gpOldV, mX, mY + 1, mZ);
	CreateTexture3D(&m_gpOldW, mX, mY, mZ + 1);
	Create3DSRV(m_gpU, &m_gpUSRV);
	Create3DSRV(m_gpV, &m_gpVSRV);
	Create3DSRV(m_gpW, &m_gpWSRV);
	Create3DSRV(m_gpOldU, &m_gpOldUSRV);
	Create3DSRV(m_gpOldV, &m_gpOldVSRV);
	Create3DSRV(m_gpOldW, &m_gpOldWSRV);
	Create3DUAV(m_gpU, &m_gpUUAV, mZ);
	Create3DUAV(m_gpV, &m_gpVUAV, mZ);
	Create3DUAV(m_gpW, &m_gpWUAV, mZ+1); // yes indeed
	Create3DUAV(m_gpOldU, &m_gpOldUUAV, mZ);
	Create3DUAV(m_gpOldV, &m_gpOldVUAV, mZ);
	Create3DUAV(m_gpOldW, &m_gpOldWUAV, mZ + 1);

	// Normal-size integer 3D arrays
	CreateTexture3D(&m_gpCounts, mX, mY, mZ, false, DXGI_FORMAT_R32_UINT);
	CreateTexture3D(&m_gpClosestParticles, mX, mY, mZ, false, DXGI_FORMAT_R32_UINT);
	Create3DSRV(m_gpCounts, &m_gpCountsSRV, DXGI_FORMAT_R32_UINT);
	Create3DSRV(m_gpClosestParticles, &m_gpClosestParticlesSRV, DXGI_FORMAT_R32_UINT);
	Create3DUAV(m_gpCounts, &m_gpCountsUAV, mZ, DXGI_FORMAT_R32_UINT);
	Create3DUAV(m_gpClosestParticles, &m_gpClosestParticlesUAV, mZ, DXGI_FORMAT_R32_UINT);

	// Normal-size floating-point 3D arrays
	CreateTexture3D(&m_gpPhi, mX, mY, mZ);
	CreateTexture3D(&m_gpProjectRHS, mX, mY, mZ);
	CreateTexture3D(&m_gpDiagCoeffs, mX, mY, mZ);
	CreateTexture3D(&m_gpProjectP, mX, mY, mZ);
	Create3DSRV(m_gpPhi, &m_gpPhiSRV);
	Create3DSRV(m_gpProjectRHS, &m_gpProjectRHSSRV);
	Create3DSRV(m_gpDiagCoeffs, &m_gpDiagCoeffsSRV);
	Create3DSRV(m_gpProjectP, &m_gpProjectPSRV);
	Create3DUAV(m_gpPhi, &m_gpPhiUAV, mZ);
	Create3DUAV(m_gpProjectRHS, &m_gpProjectRHSUAV, mZ);
	Create3DUAV(m_gpDiagCoeffs, &m_gpDiagCoeffsUAV, mZ);
	Create3DUAV(m_gpProjectP, &m_gpProjectPUAV, mZ);
	// Staging buffer for counts
	CreateTexture3D(&m_gpIntGridStage, mX, mY, mZ, true, DXGI_FORMAT_R32_UINT);
	CreateTexture3D(&m_gpFloatGridStage, mX, mY, mZ, true, DXGI_FORMAT_R32_FLOAT);
	CreateTexture3D(&m_gpFloatUStage, mX + 1, mY, mZ, true, DXGI_FORMAT_R32_FLOAT);
	CreateTexture3D(&m_gpFloatVStage, mX, mY + 1, mZ, true, DXGI_FORMAT_R32_FLOAT);
	CreateTexture3D(&m_gpFloatWStage, mX, mY, mZ + 1, true, DXGI_FORMAT_R32_FLOAT);


	// Particles
	// We need to know the number of particles we'll have ahead of time.
	// At the moment, we assume that the number of particles is constant.
	// Therefore, if we ever implement particle reseeding, we'll need to change this.
	// (ResetSimulation has the relevant logic)
	ResetSimulation();

	// Sampler states
	D3D11_SAMPLER_DESC samplerDesc;
	samplerDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
	samplerDesc.AddressU = D3D11_TEXTURE_ADDRESS_CLAMP; // Q: Should this be border 0?
	samplerDesc.AddressV = D3D11_TEXTURE_ADDRESS_CLAMP;
	samplerDesc.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;
	samplerDesc.MipLODBias = 0.0f;
	samplerDesc.MaxAnisotropy = 1;
	samplerDesc.ComparisonFunc = D3D11_COMPARISON_NEVER;
	samplerDesc.MinLOD = -FLT_MAX;
	samplerDesc.MaxLOD = FLT_MAX;
	md3dDevice->CreateSamplerState(&samplerDesc, &m_gpLinearSampler);

	//---------------------------------
	// KERNELS
	//---------------------------------
	CompileAndCreateCS(L"FX\\gpAdvect.hlsl", &m_gpAdvectFX);
	CompileAndCreateCS(L"FX\\gpCountParticles.hlsl", &m_gpCountParticlesFX);
	CompileAndCreateCS(L"FX\\gpClearIntArray.hlsl", &m_gpClearIntArrayFX);
	CompileAndCreateCS(L"FX\\gpBinParticles.hlsl", &m_gpBinParticlesFX);
	CompileAndCreateCS(L"FX\\gpClearFloatArray.hlsl", &m_gpClearFloatArrayFX);
	CompileAndCreateCS(L"FX\\gpComputeClosestParticleNeighbors.hlsl",
		&m_gpComputeClosestParticleNeighborsFX);
	CompileAndCreateCS(L"FX\\gpClosestParticlesSweepXm.hlsl", &m_gpClosestParticlesSweepXmFX);
	CompileAndCreateCS(L"FX\\gpClosestParticlesSweepXp.hlsl", &m_gpClosestParticlesSweepXpFX);
	CompileAndCreateCS(L"FX\\gpClosestParticlesSweepYm.hlsl", &m_gpClosestParticlesSweepYmFX);
	CompileAndCreateCS(L"FX\\gpClosestParticlesSweepYp.hlsl", &m_gpClosestParticlesSweepYpFX);
	CompileAndCreateCS(L"FX\\gpClosestParticlesSweepZm.hlsl", &m_gpClosestParticlesSweepZmFX);
	CompileAndCreateCS(L"FX\\gpClosestParticlesSweepZp.hlsl", &m_gpClosestParticlesSweepZpFX);
	CompileAndCreateCS(L"FX\\gpTransferParticleVelocitiesU.hlsl", &m_gpTransferParticleVelocitiesUFX);
	CompileAndCreateCS(L"FX\\gpTransferParticleVelocitiesV.hlsl", &m_gpTransferParticleVelocitiesVFX);
	CompileAndCreateCS(L"FX\\gpTransferParticleVelocitiesW.hlsl", &m_gpTransferParticleVelocitiesWFX);
	CompileAndCreateCS(L"FX\\gpAddBodyForces.hlsl", &m_gpAddBodyForcesFX);
	CompileAndCreateCS(L"FX\\gpProjectComputeRHS.hlsl", &m_gpProjectComputeRHSFX);
	CompileAndCreateCS(L"FX\\gpProjectComputeDiagCoeffs.hlsl", &m_gpProjectComputeDiagCoeffsFX);
	CompileAndCreateCS(L"FX\\gpProjectIteration1.hlsl", &m_gpProjectIteration1FX);
	CompileAndCreateCS(L"FX\\gpProjectIteration2.hlsl", &m_gpProjectIteration2FX);
	CompileAndCreateCS(L"FX\\gpProjectToVel.hlsl", &m_gpProjectToVelFX);
	CompileAndCreateCS(L"FX\\gpUpdateParticleVelocities.hlsl", &m_gpUpdateParticleVelocitiesFX);
	CompileAndCreateCS(L"FX\\gpExtrapolateParticleVelocities.hlsl", &m_gpExtrapolateParticleVelocitiesFX);
	CompileAndCreateCS(L"FX\\gpBlur.hlsl", &m_gpBlurFX);

	CreateConstantBuffer(&m_gpParametersCB, 12 * sizeof(float));
}

void GPFluidSim::ReleaseResources() {
	ReleaseCOM(m_gpLinearSampler);

	ReleaseCOM(m_gpTemp);
	ReleaseCOM(m_gpFloatWStage);
	ReleaseCOM(m_gpFloatVStage);
	ReleaseCOM(m_gpFloatUStage);
	ReleaseCOM(m_gpFloatGridStage);
	ReleaseCOM(m_gpIntGridStage);

	ReleaseCOM(m_gpParametersCB);

	ReleaseCOM(m_gpBlurFX);
	ReleaseCOM(m_gpExtrapolateParticleVelocitiesFX);
	ReleaseCOM(m_gpUpdateParticleVelocitiesFX);
	ReleaseCOM(m_gpProjectToVelFX);
	ReleaseCOM(m_gpProjectIteration2FX);
	ReleaseCOM(m_gpProjectIteration1FX);
	ReleaseCOM(m_gpProjectComputeDiagCoeffsFX);
	ReleaseCOM(m_gpProjectComputeRHSFX);
	ReleaseCOM(m_gpAddBodyForcesFX);
	ReleaseCOM(m_gpTransferParticleVelocitiesWFX);
	ReleaseCOM(m_gpTransferParticleVelocitiesVFX);
	ReleaseCOM(m_gpTransferParticleVelocitiesUFX);
	ReleaseCOM(m_gpClosestParticlesSweepZpFX);
	ReleaseCOM(m_gpClosestParticlesSweepZmFX);
	ReleaseCOM(m_gpClosestParticlesSweepYpFX);
	ReleaseCOM(m_gpClosestParticlesSweepYmFX);
	ReleaseCOM(m_gpClosestParticlesSweepXpFX);
	ReleaseCOM(m_gpClosestParticlesSweepXmFX);
	ReleaseCOM(m_gpComputeClosestParticleNeighborsFX);
	ReleaseCOM(m_gpClearFloatArrayFX);
	ReleaseCOM(m_gpBinParticlesFX);
	ReleaseCOM(m_gpClearIntArrayFX);
	ReleaseCOM(m_gpAdvectFX);
	ReleaseCOM(m_gpCountParticlesFX);

	ReleaseCOM(m_gpBinnedParticlesUAV);
	ReleaseCOM(m_gpBinnedParticlesSRV);
	ReleaseCOM(m_gpBinnedParticles);
	ReleaseCOM(m_gpParticlesUAV);
	ReleaseCOM(m_gpParticlesSRV);
	ReleaseCOM(m_gpParticles);

	ReleaseCOM(m_gpPhiUAV);
	ReleaseCOM(m_gpPhiSRV);
	ReleaseCOM(m_gpPhi);

	ReleaseCOM(m_gpClosestParticlesUAV);
	ReleaseCOM(m_gpClosestParticlesSRV);
	ReleaseCOM(m_gpClosestParticles);
	ReleaseCOM(m_gpCountsUAV);
	ReleaseCOM(m_gpCountsSRV);
	ReleaseCOM(m_gpCounts);

	ReleaseCOM(m_gpOldWUAV);
	ReleaseCOM(m_gpOldVUAV);
	ReleaseCOM(m_gpOldUUAV);
	ReleaseCOM(m_gpWUAV);
	ReleaseCOM(m_gpVUAV);
	ReleaseCOM(m_gpUUAV);
	ReleaseCOM(m_gpOldWSRV);
	ReleaseCOM(m_gpOldVSRV);
	ReleaseCOM(m_gpOldUSRV);
	ReleaseCOM(m_gpWSRV);
	ReleaseCOM(m_gpVSRV);
	ReleaseCOM(m_gpUSRV);
	ReleaseCOM(m_gpOldW);
	ReleaseCOM(m_gpOldV);
	ReleaseCOM(m_gpOldU);
	ReleaseCOM(m_gpW);
	ReleaseCOM(m_gpV);
	ReleaseCOM(m_gpU);
}

/// Speeds up the timestep used in the simulation by a factor of two.
void GPFluidSim::IncreaseSpeed() {
	m_simulationRate *= 2.0f;
	if (m_simulationRate >= 1.0f) { // Clamp to at most as fast as reality
		m_simulationRate = 1.0f;
	}
}

/// Slows down the timestep used in the simulation by a factor of two.
void GPFluidSim::DecreaseSpeed() {
	m_simulationRate /= 2.0f;
	// We can go as slow as we like - as long as we don't risk floating-point underflow:
	// (we would have to press - around 2^7+23 times:)
	float minRate = std::numeric_limits<float>::denorm_min();
	if (m_simulationRate <= minRate) {
		m_simulationRate = minRate;
	}
}

/// <summary>Creates a new empty float-based (RW) Texture3D resource on the GPU.</summary>
/// <param name="width">X dimension of the texture to create</param>
/// <param name="height">Y dimension of the texture to create</param>
/// <param name="depth">Z dimension of the texture to create</param>
void GPFluidSim::CreateTexture3D(ID3D11Texture3D** texPtr, int width, int height, int depth, bool staging) {
	CreateTexture3D(texPtr, width, height, depth, staging, DXGI_FORMAT_R32_FLOAT);
}

void GPFluidSim::CreateTexture3D(ID3D11Texture3D** texPtr, int width, int height, int depth, bool staging, DXGI_FORMAT format) {
	// Reference page: https://msdn.microsoft.com/en-us/library/windows/desktop/ff476254(v=vs.85).aspx

	D3D11_TEXTURE3D_DESC texDesc;
	texDesc.Width = width;
	texDesc.Height = height;
	texDesc.Depth = depth;
	texDesc.MipLevels = 1; // no mipmapping
	texDesc.Format = format;
	// default: gpu rw; can technically write from cpu using UpdateSubresource
	texDesc.Usage = (staging ? D3D11_USAGE_STAGING : D3D11_USAGE_DEFAULT);
	texDesc.BindFlags = (staging ? 0 : D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS);
	texDesc.CPUAccessFlags = (staging ? D3D11_CPU_ACCESS_READ : 0); // default: we'll use UpdateSubresource
	texDesc.MiscFlags = 0;

	HR(md3dDevice->CreateTexture3D(&texDesc, 0, texPtr));
}

/// <summary>Creates a (RW) structured buffer with a given stride and number of elements.</summary>
/// <param name="stride">The size of each data element.</param>
/// <param name="numElements">The number of elements in the structured buffer.</param>
void GPFluidSim::CreateStructuredBuffer(ID3D11Buffer** bfrPtr, int stride, int numElements, bool staging) {
	// Reference page: https://msdn.microsoft.com/en-us/library/windows/desktop/ff476092(v=vs.85).aspx

	D3D11_BUFFER_DESC bfrDesc;
	bfrDesc.ByteWidth = stride * numElements; // total size of the buffer
	bfrDesc.Usage = (staging?D3D11_USAGE_STAGING:D3D11_USAGE_DEFAULT); // gpu rw, cpu UpdateSubresource
	bfrDesc.BindFlags = (staging?0:D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS);
	bfrDesc.CPUAccessFlags = (staging ? D3D11_CPU_ACCESS_READ : 0);
	bfrDesc.StructureByteStride = stride;
	bfrDesc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;

	HR(md3dDevice->CreateBuffer(&bfrDesc, 0, bfrPtr));
}

// Creates a shader resource view for a 3D float-based texture.
void GPFluidSim::Create3DSRV(ID3D11Texture3D* texPtr, ID3D11ShaderResourceView** srvPtr) {
	Create3DSRV(texPtr, srvPtr, DXGI_FORMAT_R32_FLOAT);
}

void GPFluidSim::Create3DSRV(ID3D11Texture3D* texPtr, ID3D11ShaderResourceView** srvPtr, DXGI_FORMAT format) {
	// Reference page: https://msdn.microsoft.com/en-us/library/windows/desktop/ff476211(v=vs.85).aspx

	D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc;
	srvDesc.Format = format;
	srvDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE3D;
	srvDesc.Texture3D.MipLevels = 1;
	srvDesc.Texture3D.MostDetailedMip = 0;

	HR(md3dDevice->CreateShaderResourceView(texPtr, &srvDesc, srvPtr));
}

// Creates a shader resource view for a structured buffer.
// numElements: the number of data elements in the buffer.
void GPFluidSim::CreateStructuredBufferSRV(ID3D11Buffer* bfrPtr, ID3D11ShaderResourceView** srvPtr, int numElements) {
	D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc;
	srvDesc.Format = DXGI_FORMAT_UNKNOWN; // Since it's a structured buffer
	srvDesc.ViewDimension = D3D11_SRV_DIMENSION_BUFFEREX;
	srvDesc.BufferEx.FirstElement = 0;
	srvDesc.BufferEx.NumElements = numElements;
	srvDesc.BufferEx.Flags = 0;

	HR(md3dDevice->CreateShaderResourceView(bfrPtr, &srvDesc, srvPtr));
}

// Creates an unordered access view for a 3D float-based texture.
// wSize: the depth (z dimension size) of the texture.
void GPFluidSim::Create3DUAV(ID3D11Texture3D* texPtr, ID3D11UnorderedAccessView** uavPtr, int wSize) {
	// For more information on why we need to specify the depth of the texture,
	// see Zink, Pettineo, and Hoxley's "Practical Rendering and Computation with Direct3D 11", page 101.

	Create3DUAV(texPtr, uavPtr, wSize, DXGI_FORMAT_R32_FLOAT);
}

void GPFluidSim::Create3DUAV(ID3D11Texture3D* texPtr, ID3D11UnorderedAccessView** uavPtr, int wSize, DXGI_FORMAT format) {
	D3D11_UNORDERED_ACCESS_VIEW_DESC uavDesc;
	uavDesc.Format = format;
	uavDesc.ViewDimension = D3D11_UAV_DIMENSION_TEXTURE3D;
	uavDesc.Texture3D.MipSlice = 0;
	uavDesc.Texture3D.FirstWSlice = 0;
	uavDesc.Texture3D.WSize = wSize;

	HR(md3dDevice->CreateUnorderedAccessView(texPtr, &uavDesc, uavPtr));
}

// Creates an unordered access view for a structured buffer.
// numElements: the number of data elements in the buffer.
void GPFluidSim::CreateStructuredBufferUAV(ID3D11Buffer* bfrPtr, ID3D11UnorderedAccessView** uavPtr, int numElements) {
	
	D3D11_UNORDERED_ACCESS_VIEW_DESC uavDesc;
	uavDesc.Format = DXGI_FORMAT_UNKNOWN; // because it's a structured buffer
	uavDesc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
	uavDesc.Buffer.FirstElement = 0;
	uavDesc.Buffer.NumElements = numElements;
	uavDesc.Buffer.Flags = 0;

	HR(md3dDevice->CreateUnorderedAccessView(bfrPtr, &uavDesc, uavPtr));
}

template<typename T>
void GPFluidSim::UploadTex(ID3D11Texture3D* destTex, T* srcBuf, int sizeX, int sizeY) {
	md3dImmediateContext->UpdateSubresource(destTex, 0, 0,
		reinterpret_cast<const T*>(srcBuf),
		sizeof(T)*sizeX, sizeof(T)*sizeX*sizeY);
}

// Make sure to delete[] the returned array afterwards!
template<typename T> T* GPFluidSim::RetrieveTex(ID3D11Texture3D* srcTex, ID3D11Texture3D* stagingTex, int sizeX, int sizeY, int sizeZ) {
	D3D11_MAPPED_SUBRESOURCE mapped;
	md3dImmediateContext->CopyResource(stagingTex, srcTex);
	md3dImmediateContext->Map(stagingTex, 0, D3D11_MAP_READ, 0, &mapped);
	int cStride = mapped.RowPitch / sizeof(T); 
	int cPitch = mapped.DepthPitch / sizeof(T);
	T* retArray = new T[sizeX*sizeY*sizeZ];
	const T* srcArray = reinterpret_cast<const T*>(mapped.pData);
	for (int z = 0; z < sizeZ; z++) {
		for (int y = 0; y < sizeY; y++) {
			for (int x = 0; x < sizeX; x++) {
				retArray[x + sizeX * (y + sizeY * z)] = srcArray[x + cStride * y + cPitch * z];
			}
		}
	}
	md3dImmediateContext->Unmap(stagingTex, 0);
	return retArray;
}

template<typename T> T* GPFluidSim::RetrieveBuffer(ID3D11Buffer* srcBuf, ID3D11Buffer* stagingBuf, int len) {
	D3D11_MAPPED_SUBRESOURCE mapped;
	md3dImmediateContext->CopyResource(stagingBuf, srcBuf);
	md3dImmediateContext->Map(stagingBuf, 0, D3D11_MAP_READ, 0, &mapped);
	T* retArray = new T[len];
	const T* srcArray = reinterpret_cast<const T*>(mapped.pData);
	for (int i = 0; i < len; i++) {
		retArray[i] = srcArray[i];
	}
	md3dImmediateContext->Unmap(stagingBuf, 0);
	return retArray;
}

void GPFluidSim::CompileAndCreateCS(const std::wstring& filename, ID3D11ComputeShader** mFX) {
	ID3DBlob* compiledShader = d3dUtil::CompileShader(filename, nullptr, "main", "cs_5_0");

	HR(md3dDevice->CreateComputeShader(compiledShader->GetBufferPointer(),
		compiledShader->GetBufferSize(), NULL, mFX));

	// Done with the compiled shader.
	ReleaseCOM(compiledShader);
}

void GPFluidSim::CreateConstantBuffer(ID3D11Buffer** bfrPtr, int byteWidth) {
	// Reference: https://msdn.microsoft.com/en-us/library/windows/desktop/ff476896(v=vs.85).aspx
	D3D11_BUFFER_DESC cbBufDesc;
	cbBufDesc.ByteWidth = byteWidth;
	cbBufDesc.Usage = D3D11_USAGE_DYNAMIC;
	cbBufDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	cbBufDesc.MiscFlags = 0;
	cbBufDesc.StructureByteStride = 0;
	cbBufDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;

	HR(md3dDevice->CreateBuffer(&cbBufDesc, NULL, bfrPtr));
}

// Sets and binds the parameters constant buffer to have the given dt and to fit
// in the given compute shader slot.
void GPFluidSim::SetParametersConstantBuffer(float dt, float alpha, int slot) {
	// TODO, if it's worth it: Make it so that we don't read the CB back and forth (just bind it)
	// if our value of dt is the same.

	// Constant buffers
	// API-side constant buffer reference:
	// https://developer.nvidia.com/content/constant-buffers-without-constant-pain-0
	D3D11_MAPPED_SUBRESOURCE mappedData;
	struct AdvectCBStruct {
		float mmx;
		float mmy;
		float mmz;
		float mdt;
		float alpha;
		float particleRadius;
		float pad2;
		float pad3;
		UINT numParticles;
	};
	AdvectCBStruct acbs = { (float)mX, (float)mY, (float)mZ, dt,
	alpha, m_pRadius, 0.0f, 0.0f, numParticles};
	md3dImmediateContext->Map(m_gpParametersCB, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedData);
	memcpy(mappedData.pData, reinterpret_cast<void*>(&acbs), sizeof(AdvectCBStruct));
	md3dImmediateContext->Unmap(m_gpParametersCB, 0);
	md3dImmediateContext->CSSetConstantBuffers(0, 1, &m_gpParametersCB);
}

void GPFluidSim::Simulate(float dt, const GPUProfiler& profiler) {
	// Clamp maximum dt
	dt = MathHelper::Clamp(dt*m_simulationRate, 0.0f, 1.0f / 15.0f);

	// Iterate frame counter
	static int frame = 0;
	frame++;

	AdvectGPU(dt, profiler); // This is the normal code

	// Now checked; error of 0.000028 relative to ~0.2.
	TransferParticlesToGridGPU(profiler);

	// For the GPU side, we'll just do this by copying the new grids to the old grids. The standard
	// pointer-swapping trick doesn't seem to be applicable here, but maybe there's something we
	// can do if this becomes a problem?
	md3dImmediateContext->CopyResource(m_gpOldU, m_gpU);
	md3dImmediateContext->CopyResource(m_gpOldV, m_gpV);
	md3dImmediateContext->CopyResource(m_gpOldW, m_gpW);
    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_FLIP_COPYVELOCITIES);

	// Add gravity
	AddBodyForcesGPU(dt, profiler);

	ProjectGPU(dt, profiler);

	// In this step, we also do a hybrid FLIP/PIC step to update the new particle velocities.
	// Letting alpha be 6*dt*m_nu*m_CellsPerMeter^2 (pg. 118),
	float alpha = MathHelper::Clamp(6 * dt*m_nu*m_CellsPerMeter*m_CellsPerMeter, 0.0f, 1.0f);
	// Ex. For a 64x64 grid with a dt of 1/60, this is 0.0003645, since m_nu is so small (~10^-6)

	// u^new = (1-alpha)*u_old + newgrid - (1-alpha)*oldgrid
	SetParametersConstantBuffer(dt, alpha, 0);
	md3dImmediateContext->CSSetShader(m_gpUpdateParticleVelocitiesFX, NULL, 0);
	ID3D11ShaderResourceView* csSRVs6[6] = { m_gpUSRV, m_gpVSRV, m_gpWSRV, m_gpOldUSRV, m_gpOldVSRV, m_gpOldWSRV };
	md3dImmediateContext->CSSetShaderResources(0, 6, csSRVs6);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpParticlesUAV, NULL);
	md3dImmediateContext->CSSetSamplers(0, 1, &m_gpLinearSampler);
	md3dImmediateContext->Dispatch((numParticles + 63) / 64, 1, 1);

	// Clean up
	ID3D11ShaderResourceView* nullSRVs6[6] = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };
	ID3D11UnorderedAccessView* nullUAVs[1] = { nullptr };
	md3dImmediateContext->CSSetShaderResources(0, 6, nullSRVs6);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, nullUAVs, NULL);
    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_FLIP_APPLY);

	// Finally, blur Phi for rendering:
	md3dImmediateContext->CSSetShader(m_gpBlurFX, NULL, 0);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpPhiUAV, NULL);
	md3dImmediateContext->Dispatch((mX + 3) / 4, (mY + 3) / 4, (mZ + 3) / 4);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, nullUAVs, NULL);
    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_BLURLEVELSET);
}

void GPFluidSim::AdvectGPU(float dt, const GPUProfiler& profiler) {
	// This code matches the code on the GPU for an advection size of dt=0.1
	// down to a maximum difference of about 10^(-3), which is OK (given how large the velocities are)
	// This might be due to something like what's discussed in
	// https://devtalk.nvidia.com/default/topic/528016/cuda-programming-and-performance/accuracy-of-1d-linear-interpolation-by-cuda-texture-interpolation/1
	// which references http://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html#linear-filtering?
	// See also http://iquilezles.org/www/articles/hwinterpolation/hwinterpolation.htm.
	// If so, then we would expect a first-order quantization error (before RK3) of at most
	// 2^(-9) times the maximum velocity?

	// Set inputs for compute shader
	md3dImmediateContext->CSSetShader(m_gpAdvectFX, NULL, 0);
	ID3D11ShaderResourceView* csSRVs3[3] = { m_gpUSRV, m_gpVSRV, m_gpWSRV };
	md3dImmediateContext->CSSetShaderResources(0, 3, csSRVs3);
	// Targets
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpParticlesUAV, NULL);
	// Samplers
	md3dImmediateContext->CSSetSamplers(0, 1, &m_gpLinearSampler);
	// Constant buffers
	SetParametersConstantBuffer(dt, 0, 0);
	// Run shader
	md3dImmediateContext->Dispatch((numParticles + 63) / 64, 1, 1);

	// Clean up
	ID3D11SamplerState* nullSamplers[1] = { nullptr };
	md3dImmediateContext->CSSetSamplers(0, 1, nullSamplers);
	ID3D11UnorderedAccessView* nullUAVs[1] = { nullptr };
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, nullUAVs, NULL);
	ID3D11ShaderResourceView* nullSRVs[4] = { nullptr, nullptr, nullptr};
	md3dImmediateContext->CSSetShaderResources(0, 4, nullSRVs);
	md3dImmediateContext->CSSetShader(NULL, NULL, 0);

    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_ADVECT);
}

void GPFluidSim::TransferParticlesToGridGPU(const GPUProfiler& profiler) {
	// This method encapsulates the GPU version of both the ComputeLevelSet
	// and TransferParticlesToGridMethod.

	// For now, we just count how many particles fit in each cell.
	// We'll then use that structure to sort the particles into cells,
	// which we can then use for rendering and enumerating through the particles
	// in each cell.

	// First, clear m_gpCountsUAV.
	md3dImmediateContext->CSSetShader(m_gpClearIntArrayFX, NULL, 0);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpCountsUAV, NULL);
	md3dImmediateContext->Dispatch((mX + 3) / 4, (mY + 3) / 4, (mZ + 3) / 4);
	// (We don't need to clean up for this special case)
    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_TRANSFERPTG_CLEARCOUNTS);

	// Count how many particles fit in each cell
	// ASSUMES that particles have already been uploaded
	// Set inputs and targets for compute shader
	md3dImmediateContext->CSSetShader(m_gpCountParticlesFX, NULL, 0);
	md3dImmediateContext->CSSetShaderResources(0, 1, &m_gpParticlesSRV);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpCountsUAV, NULL);
	// Constant buffers
	SetParametersConstantBuffer(0, 0, 0);
	// Run shader
	md3dImmediateContext->Dispatch((numParticles + 63) / 64, 1, 1);

	// Clean up
	ID3D11UnorderedAccessView* nullUAVs[1] = { nullptr };
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, nullUAVs, NULL);
	ID3D11ShaderResourceView* nullSRVs[1] = { nullptr};
	md3dImmediateContext->CSSetShaderResources(0, 1, nullSRVs);
	md3dImmediateContext->CSSetShader(NULL, NULL, 0);
    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_TRANSFERPTG_COUNTPARTICLES);

	// Next: Do a prefix sum (or just take the maximum, which requires passing another
	// cbuffer to the method/extending current Parameters, but is simpler) to figure out
	// how to compute the list of indices into each cell and sort particles into cells.
	// OK, it looks like we're doing a prefix sum!
	// We'll use the OpenGL Scan algorithm from https://developer.nvidia.com/gpugems/GPUGems3/gpugems3_ch39.html,
	// which is slow (the article talks about the improvements their method has over it), but luckily is
	// simple.
	// Unfortunately, there don't seem to be dispatch-wide memory barriers available, so we'll have
	// to treat this algorithm like a multipass shader.
	// Also, unless we have 10^5 = 46.416 elements on a side,
	// the graph in Figure 39-8 seems to imply
	// that it's faster to just run it on the CPU.
	// So, I guess, let's just do that.
	// (Though, of course, there's always the question of whether this additional staging and copying
	// we're doing slows the entire process down.)
	// This is potentially evil, but I think it's legal?
	// I figure I'll modify this section to run on the GPU if it turns out to be a significant
	// performance factor.

	// Update: According to RenderDoc, the Map/[prefix sum]/Unmap step here now takes up about 52ms/214ms for a 64^3 grid,
	// which is significant.

	md3dImmediateContext->CopyResource(m_gpIntGridStage, m_gpCounts);
	D3D11_MAPPED_SUBRESOURCE mapped;
	md3dImmediateContext->Map(m_gpIntGridStage, 0, D3D11_MAP_READ, 0, &mapped); // modified to be shifted - check rest of code
    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_TRANSFERPTG_PREFIXSUM_COPYMAP);
	int* sums = new int[mZ*mapped.DepthPitch/sizeof(float)](); // not sure if this is secure
	int* mappedData = reinterpret_cast<int*>(mapped.pData);
	int prevSum = 0;
	int prevData = 0;
	for (int z = 0; z < mZ; z++) {
		for (int y = 0; y < mY; y++) {
			for (int x = 0; x < mX; x++) {
				int i = x + (mapped.RowPitch/sizeof(float))*y + (mapped.DepthPitch/sizeof(float))*z;
				sums[i] = prevSum + prevData;
				prevSum = sums[i];
				prevData = mappedData[i];
			}
		}
	}
    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_TRANSFERPTG_PREFIXSUM_WAIT);
	md3dImmediateContext->Unmap(m_gpIntGridStage, 0);
	// Put the results into Counts (we'll recover the results in the next shader)
	assert(sizeof(int) == 4);
	md3dImmediateContext->UpdateSubresource(m_gpCounts, 0, NULL, reinterpret_cast<void*>(sums),
		mapped.RowPitch, mapped.DepthPitch);

	delete[] sums;
    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_TRANSFERPTG_PREFIXSUM_UNMAPUPDATE);

	// Finally, put particles into cells.
	// This modifies gpCounts to become a shifted prefix sum!
	md3dImmediateContext->CSSetShader(m_gpBinParticlesFX, NULL, 0);
	md3dImmediateContext->CSSetShaderResources(0, 1, &m_gpParticlesSRV);
	ID3D11UnorderedAccessView* csUAVs2[2] = { m_gpCountsUAV, m_gpBinnedParticlesUAV };
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 2, csUAVs2, NULL);
	SetParametersConstantBuffer(0, 0, 0);
	md3dImmediateContext->Dispatch((numParticles + 63) / 64, 1, 1);
	// Clean up
	ID3D11UnorderedAccessView* nullUAVs2[2] = { nullptr, nullptr };
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 2, nullUAVs2, NULL);
	md3dImmediateContext->CSSetShader(NULL, NULL, 0);
    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_TRANSFERPTG_BIN);

	// OK! So, to get the index of particles at [x,y,z], we access
	// the previous cell; to get the number of particles, we subtract
	// that from the value of the cell at [x,y,z].

	// NEXT:
	// - compute closest particles for each known cell and each neighboring cell,
	// in a way that works (which neighbors do we have to look at?) (A: all 26-
	// no smaller number works!)
	// - extrapolate that to the whole grid using fast sweeping to get Phi.
	//---
	// - transfer particle velocities to grid using a trilinear hat kernel
	// (each sample point looks at particles in 18 cells, but only considers 8 cubic cells of particles)
	// - extrapolate velocities to rest of grid by just using closest-particle info
	// - Interesting question: What conditions do the particle velocities in the
	// air have to satisfy?

	// Clear m_gpPhi with infinity
	SetParametersConstantBuffer(0, INFINITY, 0);
	md3dImmediateContext->CSSetShader(m_gpClearFloatArrayFX, NULL, 0);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpPhiUAV, NULL);
	md3dImmediateContext->Dispatch((mX + 3) / 4, (mY + 3) / 4, (mZ + 3) / 4);
    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_TRANSFERPTG_LEVELSET_CLEAR);

	// Compute closest particles for each known cell and neighboring cell
	md3dImmediateContext->CSSetShader(m_gpComputeClosestParticleNeighborsFX, NULL, 0);
	ID3D11ShaderResourceView* csSRVs2[2] = { m_gpCountsSRV, m_gpBinnedParticlesSRV };
	ID3D11UnorderedAccessView* csCPUAVs2[2] = { m_gpClosestParticlesUAV, m_gpPhiUAV };
	md3dImmediateContext->CSSetShaderResources(0, 2, csSRVs2);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 2, csCPUAVs2, NULL);
	SetParametersConstantBuffer(0, 0, 0);
	md3dImmediateContext->Dispatch((mX + 7) / 8, (mY + 7) / 8, (mZ + 7) / 8);
    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_TRANSFERPTG_LEVELSET_ZERO);

	// Next: extrapolate that to the rest of the grid
	// This is a series of 24 single-direction sweeps, as listed in the original
	// ComputeLevelSet code, based off of Table 1 of "Fast Occlusion Sweeping" by 
	// Singh, Yuksel, and House, which requires 24 sweeps. (Question: Can we do better?
	// 2 passes of a Manhattan distance method give 12 sweeps, but is that enough?)

	// Directions: x- y- z-  x+ y- z-  x- y+ z-  x+ y+ z-  x- y- z+  x+ y- z+  x- y+ z+  x+ y+ z+
	// 0 1 2 3 4 5: x- x+ y- y+ z- z+
	int numSweepDirections = 24;
	int sweepDirections[24] = {
		0, 2, 4,
		1, 2, 4,
		0, 3, 4,
		1, 3, 4,
		0, 2, 5,
		1, 2, 5,
		0, 3, 5,
		1, 3, 5,
	};

	// Luckily, since we use the same parameters as gpComputeClosestParticleNeighbors, we don't
	// need to issue any calls other than setting shaders and calling dispatches!
	for (int i = 0; i < numSweepDirections; i++) {
		int dir = sweepDirections[i];
		switch (dir) {
		case 0:
			md3dImmediateContext->CSSetShader(m_gpClosestParticlesSweepXmFX, NULL, 0);
			break;
		case 1:
			md3dImmediateContext->CSSetShader(m_gpClosestParticlesSweepXpFX, NULL, 0);
			break;
		case 2:
			md3dImmediateContext->CSSetShader(m_gpClosestParticlesSweepYmFX, NULL, 0);
			break;
		case 3:
			md3dImmediateContext->CSSetShader(m_gpClosestParticlesSweepYpFX, NULL, 0);
			break;
		case 4:
			md3dImmediateContext->CSSetShader(m_gpClosestParticlesSweepZmFX, NULL, 0);
			break;
		case 5:
			md3dImmediateContext->CSSetShader(m_gpClosestParticlesSweepZpFX, NULL, 0);
			break;
		}

		switch (dir) {
		case 0:
		case 1:
			md3dImmediateContext->Dispatch(1,            (mY + 7) / 8, (mZ + 7) / 8);
			break;
		case 2:
		case 3:
			md3dImmediateContext->Dispatch((mX + 7) / 8,            1, (mY + 7) / 8);
			break;
		case 4:
		case 5:
			md3dImmediateContext->Dispatch((mX + 7) / 8, (mY + 7) / 8,            1);
			break;
		}
	}

	// Unbind UAVs
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 2, nullUAVs2, NULL);
    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_TRANSFERPTG_LEVELSET_SWEEP);


	// Next: Transfer particle velocities to grids and extrapolate velocities.
	// Each velocity point looks at the particles in its 18 immediate neighbors: [-1 0]
	// in its direction, and [-1 0 1] in the other two directions.
	// See shader code for more information.
	// Update: According to RenderDoc, these three draw calls take up 95.9ms/214.5ms on a 64^3 grid.
	// That might just be due to the incredible number of particles each cell needs to go through (~144)?
	md3dImmediateContext->CSSetShader(m_gpTransferParticleVelocitiesUFX, NULL, 0);
	ID3D11ShaderResourceView* csSRVs3[3] = { m_gpCountsSRV, m_gpBinnedParticlesSRV, m_gpClosestParticlesSRV };
	md3dImmediateContext->CSSetShaderResources(0, 3, csSRVs3);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpUUAV, NULL);
	md3dImmediateContext->Dispatch((mX + 1 + 3) / 4, (mY + 3) / 4, (mZ + 3) / 4);
	// V
	md3dImmediateContext->CSSetShader(m_gpTransferParticleVelocitiesVFX, NULL, 0);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpVUAV, NULL);
	md3dImmediateContext->Dispatch((mX + 3) / 4, (mY + 1 + 3) / 4, (mZ + 3) / 4);
	// W
	md3dImmediateContext->CSSetShader(m_gpTransferParticleVelocitiesWFX, NULL, 0);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpWUAV, NULL);
	md3dImmediateContext->Dispatch((mX + 3) / 4, (mY + 3) / 4, (mZ + 1 + 3) / 4);

	// Clean up
	ID3D11ShaderResourceView* nullSRVs3[3] = { nullptr, nullptr, nullptr };
	md3dImmediateContext->CSSetShaderResources(0, 3, nullSRVs3);
    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_TRANSFERPTG_VELOCITY);

	// Finally, it turns out we actually need to extrapolate the particle velocities properly
	// by a single grid cell. We can do this kind of cleverly by implicitly marking invalid
	// values (above) and running the following shader on each of the arrays:
	// ...or, we might have had a good method before!
	md3dImmediateContext->CSSetShader(m_gpExtrapolateParticleVelocitiesFX, NULL, 0);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpUUAV, NULL);
	md3dImmediateContext->Dispatch((mX + 1 + 3) / 4, (mY + 3) / 4, (mZ + 3) / 4);

	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpVUAV, NULL);
	md3dImmediateContext->Dispatch((mX + 3) / 4, (mY + 3 + 1) / 4, (mZ + 3) / 4);

	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpWUAV, NULL);
	md3dImmediateContext->Dispatch((mX + 3) / 4, (mY + 3) / 4, (mZ + 3 + 1) / 4);

	// and with that, we're done!
	// Clean up
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, nullUAVs, NULL);
	md3dImmediateContext->CSSetShader(NULL, NULL, 0);
    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_TRANSFERPTG_VELOCITY_EXTRAPOLATE);
}

void GPFluidSim::AddBodyForcesGPU(float dt, const GPUProfiler& profiler) {
	// Just adds dt*g to the velocity field
	SetParametersConstantBuffer(dt, 0, 0);
	md3dImmediateContext->CSSetShader(m_gpAddBodyForcesFX, NULL, 0);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpVUAV, NULL);
	md3dImmediateContext->Dispatch((mX + 3) / 4, (mY + 1 + 3) / 4, (mZ + 3) / 4);
	// Clean up
	ID3D11UnorderedAccessView* nullUAVs[1] = { nullptr };
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, nullUAVs, NULL);

    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_BODYFORCES);
}

void GPFluidSim::ProjectGPU(float dt, const GPUProfiler& profiler) {
	// GPU-based version of the Project method (see Simulation2D.cpp or Simulation3D.cpp for more information).
	// Steps:
	// 1. Compute right-hand-side (vel, dt => b)
	// 2. Compute diagonal coefficients (Phi => diagCoeffs)
	// 3. [After setting p to 0,] solve the linear system (diagCoeffs, Phi, b, omega, p => new p)
	//    The previous step is run with a 2-color checkerboard iteration numIterations (i.e. many) times.
	// 4. Compute new velocity fields (p, Phi => U, V, W) - can potentially be done in one pass!

	// 1. Compute right-hand-side (vel, dt => b)
	SetParametersConstantBuffer(dt, 0, 0);
	md3dImmediateContext->CSSetShader(m_gpProjectComputeRHSFX, NULL, 0);
	ID3D11ShaderResourceView* csSRVs3[3] = { m_gpUSRV, m_gpVSRV, m_gpWSRV };
	md3dImmediateContext->CSSetShaderResources(0, 3, csSRVs3);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpProjectRHSUAV, NULL);
	md3dImmediateContext->Dispatch((mX + 3) / 4, (mY + 3) / 4, (mZ + 3) / 4);
	// Clean up 1
	ID3D11ShaderResourceView* nullSRVs3[3] = { nullptr, nullptr, nullptr };
	md3dImmediateContext->CSSetShaderResources(0, 3, nullSRVs3);
    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_PROJECT_RHS);

	// 2. Compute diagonal coefficients (Phi => diagCoeffs)
	md3dImmediateContext->CSSetShader(m_gpProjectComputeDiagCoeffsFX, NULL, 0);
	md3dImmediateContext->CSSetShaderResources(0, 1, &m_gpPhiSRV);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpDiagCoeffsUAV, NULL);
	md3dImmediateContext->Dispatch((mX + 3) / 4, (mY + 3) / 4, (mZ + 3) / 4);
	// Clean up 2
	ID3D11ShaderResourceView* nullSRVs1[1] = { nullptr };
	md3dImmediateContext->CSSetShaderResources(0, 1, nullSRVs1);
    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_PROJECT_DIAGCOEFFS);

	// 3a. Set p to 0. (We pass 0 in through the alpha parameter!)
	md3dImmediateContext->CSSetShader(m_gpClearFloatArrayFX, NULL, 0);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpProjectPUAV, NULL);
	md3dImmediateContext->Dispatch((mX + 3) / 4, (mY + 3) / 4, (mZ + 3) / 4);
    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_PROJECT_PCLEAR);

	// 3b. Solve the linear system using a 2-color checkerboard Gauss-Seidel iteration.
	//     (diagCoeffs, Phi, omega, p => new p)
	// This step has a maximum relative error of ~0.000000 at the 2nd iteration, and an
	// absolute error of ~0.00250 = -640.958429 - (-640.958679) at the 100th iteration.
	//
	// For the moment, we'll implement this as a single file for a single 2-color iteration,
	// but this doesn't do Gauss-Seidel perfectly because of inter-dispatch synchronization.
	// I think it should still converge to the proper value - the question is, what value
	// do we gain from this approach?
	// Another question: Is it important to use double precision here?
	//
	// We pass in our SOR value of omega through the alpha parameter, since we have space for it.
	float omega = 2 - 3.16343f / mX;
	int numIterations = 100;

	SetParametersConstantBuffer(dt, omega, 0);
	
	ID3D11ShaderResourceView* csProjectSRVs3[3] = { m_gpDiagCoeffsSRV, m_gpPhiSRV, m_gpProjectRHSSRV };
	md3dImmediateContext->CSSetShaderResources(0, 3, csProjectSRVs3);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpProjectPUAV, NULL);
	for (int i = 0; i < numIterations; i++) {
		md3dImmediateContext->CSSetShader(m_gpProjectIteration1FX, NULL, 0);
		md3dImmediateContext->Dispatch((mX + 7) / 8, (mY + 3) / 4, (mZ + 3) / 4); // This is because of our 2x1x1 thread assignment
		md3dImmediateContext->CSSetShader(m_gpProjectIteration2FX, NULL, 0);
		md3dImmediateContext->Dispatch((mX + 7) / 8, (mY + 3) / 4, (mZ + 3) / 4);
	}

	// Clean up 3b
	ID3D11UnorderedAccessView* nullUAVs1[1] = { nullptr };
	md3dImmediateContext->CSSetShaderResources(0, 3, nullSRVs3);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, nullUAVs1, NULL);
    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_PROJECT_SOR);

	// 4. Compute new velocity fields (p, Phi => U, V, W)
	md3dImmediateContext->CSSetShader(m_gpProjectToVelFX, NULL, 0);
	ID3D11ShaderResourceView* csSRVs2[2] = { m_gpProjectPSRV, m_gpPhiSRV };
	ID3D11UnorderedAccessView* csUAVs3[3] = { m_gpUUAV, m_gpVUAV, m_gpWUAV };
	md3dImmediateContext->CSSetShaderResources(0, 2, csSRVs2);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 3, csUAVs3, NULL);
	md3dImmediateContext->Dispatch((mX + 3) / 4, (mY + 3) / 4, (mZ + 3) / 4);
	// Clean up 4
	ID3D11ShaderResourceView* nullSRVs2[2] = { nullptr, nullptr};
	ID3D11UnorderedAccessView* nullUAVs3[3] = { nullptr, nullptr, nullptr };
	md3dImmediateContext->CSSetShaderResources(0, 2, nullSRVs2);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 3, nullUAVs3, NULL);
    profiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_PROJECT_TOVELOCITY);
}