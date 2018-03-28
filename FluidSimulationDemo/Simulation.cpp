//************************************************************
// Simulation3D.cpp
// Top level of code for doing 3D fluid simulation on the CPU.
//
// Authors:
//   Neil Bickford
//************************************************************

#include "Simulation.h"
#include "odprintf.h"
#include <limits>
#include <queue> // For serial extrapolation - see ExtrapolateValues(4).
#include <random>
#include <algorithm> // for std::min
#include <numeric> // for debug std::accumulate

//#include "debugroutines.h" TODO: Implement stb-like single-header library properly

// Don't forget to call Initialize after this!
GPFluidSim::GPFluidSim(int xSize, int ySize, int zSize, float CellsPerMeter)
	:mX(xSize), mY(ySize), mZ(zSize), m_CellsPerMeter(CellsPerMeter),
	m_particles() {

	// Set up MAC velocity grids and initialize velocities
	m_MU = new float[(xSize + 1)*ySize*zSize];
	m_MV = new float[xSize*(ySize + 1)*zSize];
	m_MW = new float[xSize*ySize*(zSize + 1)];

	// Level sets and auxiliary fields
	m_Phi = new float[xSize*ySize*zSize];
}

GPFluidSim::~GPFluidSim() {
	ReleaseResources();
	m_particles.clear();
	delete[] m_MU;
	delete[] m_MV;
	delete[] m_MW;
	delete[] m_Phi;
}

void GPFluidSim::Initialize(ID3D11Device* device,
	ID3D11DeviceContext* immediateContext) {
	md3dDevice = device;
	md3dImmediateContext = immediateContext;

	AcquireResources();

	ResetSimulation();
}

void GPFluidSim::AcquireResources() {
	// Velocity fields

	// Texture3D acquisition

	// Velocity arrays
	CreateTexture3D(&m_gpU, mX + 1, mY, mZ);
	CreateTexture3D(&m_gpV, mX, mY + 1, mZ);
	CreateTexture3D(&m_gpW, mX, mY, mZ + 1);
	Create3DSRV(m_gpU, &m_gpUSRV);
	Create3DSRV(m_gpV, &m_gpVSRV);
	Create3DSRV(m_gpW, &m_gpWSRV);
	Create3DUAV(m_gpU, &m_gpUUAV, mZ);
	Create3DUAV(m_gpV, &m_gpVUAV, mZ);
	Create3DUAV(m_gpW, &m_gpWUAV, mZ+1); // yes indeed
	// Normal-size integer 3D arrays
	CreateTexture3D(&m_gpCounts, mX, mY, mZ, false, DXGI_FORMAT_R32_UINT);
	CreateTexture3D(&m_gpClosestParticles, mX, mY, mZ, false, DXGI_FORMAT_R32_UINT);
	Create3DSRV(m_gpCounts, &m_gpCountsSRV, DXGI_FORMAT_R32_UINT);
	Create3DSRV(m_gpClosestParticles, &m_gpClosestParticlesSRV, DXGI_FORMAT_R32_UINT);
	Create3DUAV(m_gpCounts, &m_gpCountsUAV, mZ, DXGI_FORMAT_R32_UINT);
	Create3DUAV(m_gpClosestParticles, &m_gpClosestParticlesUAV, mZ, DXGI_FORMAT_R32_UINT);
	// Normal-size floating-point 3D arrays
	CreateTexture3D(&m_gpPhi, mX, mY, mZ);
	Create3DSRV(m_gpPhi, &m_gpPhiSRV);
	Create3DUAV(m_gpPhi, &m_gpPhiUAV, mZ);
	// Staging buffer for counts
	CreateTexture3D(&m_gpIntGridStage, mX, mY, mZ, true, DXGI_FORMAT_R32_UINT);


	// Particles
	// Problem: To fit this, we'll ideally need to know the number of particles
	// we'll be using ahead of time. For the moment, we'll do this by using the
	// theoretical maximum. In theory, this could be fixed by more closely tying
	// together resource acquisition and initialization.
	int maxParticles = 8 * mX*mY*mZ;
	CreateStructuredBuffer(&m_gpParticles, sizeof(Particle3), maxParticles);
	CreateStructuredBufferSRV(m_gpParticles, &m_gpParticlesSRV, maxParticles);
	CreateStructuredBufferUAV(m_gpParticles, &m_gpParticlesUAV, maxParticles);
	CreateStructuredBuffer(&m_gpBinnedParticles, sizeof(Particle3), maxParticles);
	CreateStructuredBufferSRV(m_gpBinnedParticles, &m_gpBinnedParticlesSRV, maxParticles);
	CreateStructuredBufferUAV(m_gpBinnedParticles, &m_gpBinnedParticlesUAV, maxParticles);

	// Backbuffers
	CreateStructuredBuffer(&m_gpParticlesTarget, sizeof(Particle3), maxParticles);
	CreateStructuredBufferSRV(m_gpParticlesTarget, &m_gpParticlesTargetSRV, maxParticles);
	CreateStructuredBufferUAV(m_gpParticlesTarget, &m_gpParticlesTargetUAV, maxParticles);

	// Staging buffers
	CreateStructuredBuffer(&m_gpTemp, sizeof(Particle3), maxParticles, true);

	// Sampler states
	D3D11_SAMPLER_DESC samplerDesc;
	samplerDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
	samplerDesc.AddressU = D3D11_TEXTURE_ADDRESS_CLAMP;
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

	CreateConstantBuffer(&m_gpParametersCB, 12 * sizeof(float));
}

void GPFluidSim::ReleaseResources() {
	ReleaseCOM(m_gpLinearSampler);

	ReleaseCOM(m_gpTemp);
	ReleaseCOM(m_gpIntGridStage);

	ReleaseCOM(m_gpParametersCB);

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
	ReleaseCOM(m_gpParticlesTargetUAV);
	ReleaseCOM(m_gpParticlesTargetSRV);
	ReleaseCOM(m_gpParticlesTarget);
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

	ReleaseCOM(m_gpWUAV);
	ReleaseCOM(m_gpVUAV);
	ReleaseCOM(m_gpUUAV);
	ReleaseCOM(m_gpWSRV);
	ReleaseCOM(m_gpVSRV);
	ReleaseCOM(m_gpUSRV);
	ReleaseCOM(m_gpW);
	ReleaseCOM(m_gpV);
	ReleaseCOM(m_gpU);
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

/// <summary>Uploads the particles in m_particles to the buffer pointed
/// to by bfrPtr (which must be 6*8*mX*mY*mZ floats in size).</summary>
void GPFluidSim::UploadParticles(ID3D11Buffer* bfrPtr) {
	int len = static_cast<int>(m_particles.size());
	int buffSize = 6 * 8 * mX*mY*mZ;
	assert(6 * len < buffSize);
	float* tmpParticles = new float[buffSize];
	for (int i = 0; i < len; i++) {
		tmpParticles[6 * i + 0] = m_particles[i].X;
		tmpParticles[6 * i + 1] = m_particles[i].Y;
		tmpParticles[6 * i + 2] = m_particles[i].Z;
		tmpParticles[6 * i + 3] = m_particles[i].uX;
		tmpParticles[6 * i + 4] = m_particles[i].uY;
		tmpParticles[6 * i + 5] = m_particles[i].uZ;
	}
	md3dImmediateContext->UpdateSubresource(m_gpParticles, 0, 0,
		reinterpret_cast<const void*>(tmpParticles),
		sizeof(float)*buffSize, sizeof(float)*buffSize);
	delete[] tmpParticles;
}

void GPFluidSim::UploadU() {
	md3dImmediateContext->UpdateSubresource(m_gpU, 0, 0,
		reinterpret_cast<const void*>(m_MU),
		sizeof(float)*(mX + 1), sizeof(float)*(mX + 1)*mY);
}

void GPFluidSim::UploadV() {
	md3dImmediateContext->UpdateSubresource(m_gpV, 0, 0,
		reinterpret_cast<const void*>(m_MV),
		sizeof(float)*mX, sizeof(float)*mX*(mY + 1));
}

void GPFluidSim::UploadW() {
	md3dImmediateContext->UpdateSubresource(m_gpW, 0, 0,
		reinterpret_cast<const void*>(m_MW),
		sizeof(float)*mX, sizeof(float)*mX*mY);
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
	alpha, m_pRadius, 0.0f, 0.0f,
	static_cast<UINT>(m_particles.size()) };
	md3dImmediateContext->Map(m_gpParametersCB, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedData);
	memcpy(mappedData.pData, reinterpret_cast<void*>(&acbs), sizeof(AdvectCBStruct));
	md3dImmediateContext->Unmap(m_gpParametersCB, 0);
	md3dImmediateContext->CSSetConstantBuffers(0, 1, &m_gpParametersCB);
}

void GPFluidSim::ResetSimulation() {
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
							float m1 = rX + u * d + distribution(generator) / m_CellsPerMeter;
							float m2 = rY + v * d + distribution(generator) / m_CellsPerMeter;
							float m3 = rZ + w * d + distribution(generator) / m_CellsPerMeter;
							m_particles.emplace_back(m1, m2, m3, InterpolateMACCell(mX*m1, mY*m2, mZ*m3));
						}
					}
				}
			}
		}
	}

	// Update GPU resources
	UploadU();
	UploadV();
	UploadW();

	UploadParticles(m_gpParticles);
}

void GPFluidSim::Simulate(float dt) {
	// Clamp maximum dt
	dt = MathHelper::Clamp(dt, 0.0f, 1.0f / 15.0f);
	dt = 0.01f; // QQQ DEBUG Limit dt for debugging purposes

				// Iterate frame counter
	static int frame = 0;
	frame++;

	AdvectGPU(dt);

	// In this step, we also do a hybrid FLIP/PIC step to update the new particle velocities.
	// Letting alpha be 6*dt*m_nu*m_CellsPerMeter^2 (pg. 118),
	float alpha = MathHelper::Clamp(6 * dt*m_nu*m_CellsPerMeter*m_CellsPerMeter, 0.0f, 1.0f);
	//alpha = 1.00f;
	// Ex. For a 64x64 grid with a dt of 1/60, this is 0.0003645, since m_nu is so small (~10^-6)
	ComputeLevelSet(m_particles);

	TransferParticlesToGrid(m_particles);

	// Debugging
	TransferParticlesToGridGPU();

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
	AddBodyForcesGPU(dt);

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

void GPFluidSim::Advect(std::vector<Particle3> &particles, float dt) {
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
		XMFLOAT3 k1 = InterpolateMACCell(mX*particles[i].X, mY*particles[i].Y, mZ*particles[i].Z); // should actually be cellsPerMeter?
		XMFLOAT3 k2 = InterpolateMACCell(mX*(particles[i].X + 0.5f*dt*k1.x),
			mY*(particles[i].Y + 0.5f*dt*k1.y),
			mZ*(particles[i].Z + 0.5f*dt*k1.z));
		XMFLOAT3 k3 = InterpolateMACCell(mX*(particles[i].X + 0.75f*dt*k2.x),
			mY*(particles[i].Y + 0.75f*dt*k2.y),
			mZ*(particles[i].Z + 0.75f*dt*k2.z));

		float vX = (2.0f / 9.0f)*k1.x + (3.0f / 9.0f)*k2.x + (4.0f / 9.0f)*k3.x;
		float vY = (2.0f / 9.0f)*k1.y + (3.0f / 9.0f)*k2.y + (4.0f / 9.0f)*k3.y;
		float vZ = (2.0f / 9.0f)*k1.z + (3.0f / 9.0f)*k2.z + (4.0f / 9.0f)*k3.z;
		particles[i].X += dt * vX;
		particles[i].Y += dt * vY;
		particles[i].Z += dt * vZ;

		// Clamp to box, slightly inwards
		float eps = 0.1f;
		particles[i].X = MathHelper::Clamp(particles[i].X, (-0.5f + eps) / mX, 1.0f + (-0.5f - eps) / mX);
		particles[i].Y = MathHelper::Clamp(particles[i].Y, (-0.5f + eps) / mY, 1.0f + (-0.5f - eps) / mY);
		particles[i].Z = MathHelper::Clamp(particles[i].Z, (-0.5f + eps) / mZ, 1.0f + (-0.5f - eps) / mZ);
	}
}

void GPFluidSim::AdvectGPU(float dt) {
	// This code matches the code on the GPU for an advection size of dt=0.1
	// down to a maximum difference of about 10^(-3), which is OK (given how large the velocities are)
	// This might be due to something like what's discussed in
	// https://devtalk.nvidia.com/default/topic/528016/cuda-programming-and-performance/accuracy-of-1d-linear-interpolation-by-cuda-texture-interpolation/1
	// which references http://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html#linear-filtering?
	// If so, then we would expect a first-order quantization error (before RK3) of at most
	// 2^(-9) times the maximum velocity?

	UploadU();
	UploadV();
	UploadW();
	UploadParticles(m_gpParticles);
	// Set inputs and targets for compute shader
	md3dImmediateContext->CSSetShader(m_gpAdvectFX, NULL, 0);
	md3dImmediateContext->CSSetShaderResources(0, 1, &m_gpUSRV);
	md3dImmediateContext->CSSetShaderResources(1, 1, &m_gpVSRV);
	md3dImmediateContext->CSSetShaderResources(2, 1, &m_gpWSRV);
	md3dImmediateContext->CSSetShaderResources(3, 1, &m_gpParticlesSRV);
	// Targets
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpParticlesTargetUAV, NULL);
	// Samplers
	md3dImmediateContext->CSSetSamplers(0, 1, &m_gpLinearSampler);
	// Constant buffers
	SetParametersConstantBuffer(dt, 0, 0);
	// Run shader
	md3dImmediateContext->Dispatch(((UINT)m_particles.size() + 63) / 64, 1, 1);
	// Run CPU version
	Advect(m_particles, dt);

	// Compare results? Reference: Luna, v. 11, pg. 443-444
#if 0 // set this to 1 to compare results
	md3dImmediateContext->CopyResource(m_gpTemp, m_gpParticlesTarget);
	md3dImmediateContext->Map(m_gpTemp, 0, D3D11_MAP_READ, 0, &mappedData);
	Particle3* dataView = reinterpret_cast<Particle3*>(mappedData.pData);

	float maxDiff=0.0f;
	int maxInd=-1;
	for (int i = 0; i < m_particles.size(); i++) {
		float m2 =        fabsf(m_particles[i].X - dataView[i].X);
		m2 = std::max(m2, fabsf(m_particles[i].Y - dataView[i].Y));
		m2 = std::max(m2, fabsf(m_particles[i].Z - dataView[i].Z));
		m2 = std::max(m2, fabsf(m_particles[i].uX - dataView[i].uX));
		m2 = std::max(m2, fabsf(m_particles[i].uY - dataView[i].uY));
		m2 = std::max(m2, fabsf(m_particles[i].uZ - dataView[i].uZ));
		if (m2 > maxDiff) {
			maxDiff = m2;
			maxInd = i;
		}
	}

	odprintf("Max diff: %.3e at %i", maxDiff, maxInd);

	md3dImmediateContext->Unmap(m_gpTemp, 0);

#endif

	// Clean up
	ID3D11SamplerState* nullSamplers[1] = { nullptr };
	md3dImmediateContext->CSSetSamplers(0, 1, nullSamplers);
	ID3D11UnorderedAccessView* nullUAVs[1] = { nullptr };
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, nullUAVs, NULL);
	ID3D11ShaderResourceView* nullSRVs[3] = { nullptr, nullptr, nullptr };
	md3dImmediateContext->CSSetShaderResources(0, 3, nullSRVs);
	md3dImmediateContext->CSSetShader(NULL, NULL, 0);
}

float GPFluidSim::ptDistance(float x0, float y0, float z0, float x1, float y1, float z1) {
	return sqrtf((x0 - x1)*(x0 - x1) + (y0 - y1)*(y0 - y1) + (z0 - z1)*(z0 - z1)) - m_pRadius;
}

void GPFluidSim::clsInner(int dx, int dy, int dz, int x, int y, int z,
	int* closestParticles) {

	int otherPt = closestParticles[(x + dx) + mX * ((y + dy) + mY * (z + dz))];
	if (otherPt > 0) {
		float px = m_particles[otherPt].X*mX;
		float py = m_particles[otherPt].Y*mY;
		float pz = m_particles[otherPt].Z*mZ;
		float dist = ptDistance((float)x, (float)y, (float)z, px, py, pz);
		int idx = x + mX * (y + mY * z);
		if (closestParticles[idx] < 0 || dist < m_Phi[idx]) {
			closestParticles[idx] = otherPt;
			m_Phi[idx] = dist;
		}
	}
}

void GPFluidSim::ComputeLevelSet(const std::vector<Particle3> &particles) {
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
	int gridSize = mX * mY*mZ;
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
		int ci = cellX + mX * (cellY + mY * cellZ);

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
			for (int x = mX - 1; x >= 0; x--) {
				if (x != mX - 1)
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
		for (int y = mY - 1; y >= 0; y--) {
			for (int x = 0; x < mX; x++) {
				if (x != 0)
					clsInner(-1, 0, 0, x, y, z, closestParticles);
				if (y != mY - 1)
					clsInner(0, 1, 0, x, y, z, closestParticles);
				if (z != 0)
					clsInner(0, 0, -1, x, y, z, closestParticles);
			}
		}
	}

	// x- y- z+
	for (int z = 0; z < mZ; z++) {
		for (int y = mY - 1; y >= 0; y--) {
			for (int x = mX - 1; x >= 0; x--) {
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
	for (int z = mZ - 1; z >= 0; z--) {
		for (int y = 0; y < mY; y++) {
			for (int x = 0; x < mX; x++) {
				if (x != 0)
					clsInner(-1, 0, 0, x, y, z, closestParticles);
				if (y != 0)
					clsInner(0, -1, 0, x, y, z, closestParticles);
				if (z != mZ - 1)
					clsInner(0, 0, 1, x, y, z, closestParticles);
			}
		}
	}

	// x- y+ z-
	for (int z = mZ - 1; z >= 0; z--) {
		for (int y = 0; y < mY; y++) {
			for (int x = mX - 1; x >= 0; x--) {
				if (x != mX - 1)
					clsInner(1, 0, 0, x, y, z, closestParticles);
				if (y != 0)
					clsInner(0, -1, 0, x, y, z, closestParticles);
				if (z != mZ - 1)
					clsInner(0, 0, 1, x, y, z, closestParticles);
			}
		}
	}

	// x+ y- z-
	for (int z = mZ - 1; z >= 0; z--) {
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
	for (int z = mZ - 1; z >= 0; z--) {
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

void GPFluidSim::TransferParticlesToGridGPU() {
	// This method should eventually encapsulate both the ComputeLevelSet
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

	// Count how many particles fit in each cell
	// ASSUMES that particles have already been uploaded
	// Set inputs and targets for compute shader
	md3dImmediateContext->CSSetShader(m_gpCountParticlesFX, NULL, 0);
	md3dImmediateContext->CSSetShaderResources(0, 1, &m_gpParticlesSRV);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpCountsUAV, NULL);
	// Constant buffers
	SetParametersConstantBuffer(0, 0, 0);
	// Run shader
	int len = static_cast<int>(m_particles.size());
	md3dImmediateContext->Dispatch((len + 63) / 64, 1, 1);

	// Clean up
	ID3D11UnorderedAccessView* nullUAVs[1] = { nullptr };
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, nullUAVs, NULL);
	ID3D11ShaderResourceView* nullSRVs[1] = { nullptr};
	md3dImmediateContext->CSSetShaderResources(0, 1, nullSRVs);
	md3dImmediateContext->CSSetShader(NULL, NULL, 0);

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

	md3dImmediateContext->CopyResource(m_gpIntGridStage, m_gpCounts);
	D3D11_MAPPED_SUBRESOURCE mapped;
	int* sums = new int[mX*mY*mZ];
	md3dImmediateContext->Map(m_gpIntGridStage, 0, D3D11_MAP_READ, 0, &mapped); // modified to be shifted - check rest of code
	int* mappedData = reinterpret_cast<int*>(mapped.pData);
	sums[0] = mappedData[0];
	for (int i = 1; i < mX*mY*mZ; i++) {
		sums[i] = mappedData[i - 1] + sums[i - 1]; //hooray
	}
	md3dImmediateContext->Unmap(m_gpIntGridStage, 0);
	// Put the results into Counts (we'll recover the results in the next shader)
	assert(sizeof(int) == 4);
	md3dImmediateContext->UpdateSubresource(m_gpCounts, 0, NULL, reinterpret_cast<void*>(sums),
		4 * mX, 4 * mX*mY);

	// Finally, put particles into cells.
	// This modifies gpCounts to become a shifted prefix sum!
	md3dImmediateContext->CSSetShader(m_gpBinParticlesFX, NULL, 0);
	md3dImmediateContext->CSSetShaderResources(0, 1, &m_gpParticlesSRV);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpCountsUAV, NULL);
	md3dImmediateContext->CSSetUnorderedAccessViews(1, 1, &m_gpBinnedParticlesUAV, NULL);
	SetParametersConstantBuffer(0, 0, 0);
	md3dImmediateContext->Dispatch((len + 63) / 64, 1, 1);
	// Clean up
	ID3D11UnorderedAccessView* nullUAVs2[2] = { nullptr, nullptr };
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 2, nullUAVs2, NULL);
	md3dImmediateContext->CSSetShader(NULL, NULL, 0);

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
	// (each sample point looks at particles in 18 cells, but only considers 4 cubic cells
	// of particles, I think - need to define radius of hat kernel)
	// - extrapolate velocities to rest of grid by just using closest-particle info
	// - Interesting question: What conditions do the particle velocities in the
	// air have to satisfy?

	// Clear m_gpPhi
	md3dImmediateContext->CSSetShader(m_gpClearFloatArrayFX, NULL, 0);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpPhiUAV, NULL);
	md3dImmediateContext->Dispatch((mX + 3) / 4, (mY + 3) / 4, (mZ + 3) / 4);

	// Compute closest particles for each known cell and neighboring cell
	md3dImmediateContext->CSSetShader(m_gpComputeClosestParticleNeighborsFX, NULL, 0);
	md3dImmediateContext->CSSetShaderResources(0, 1, &m_gpCountsSRV);
	md3dImmediateContext->CSSetShaderResources(1, 1, &m_gpBinnedParticlesSRV);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpClosestParticlesUAV, NULL);
	md3dImmediateContext->CSSetUnorderedAccessViews(1, 1, &m_gpPhiUAV, NULL);
	SetParametersConstantBuffer(0, 0, 0);
	md3dImmediateContext->Dispatch((mX + 7) / 8, (mY + 7) / 8, (mZ + 7) / 8);

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
	// Next: Transfer particle velocities to grids and extrapolate velocities.
	// Each velocity point looks at the particles in its 18 immediate neighbors: [-1 0]
	// in its direction, and [-1 0 1] in the other two directions.
	// See shader code for more information.
	md3dImmediateContext->CSSetShader(m_gpTransferParticleVelocitiesUFX, NULL, 0);
	md3dImmediateContext->CSSetShaderResources(0, 1, &m_gpCountsSRV);
	md3dImmediateContext->CSSetShaderResources(1, 1, &m_gpBinnedParticlesSRV);
	md3dImmediateContext->CSSetShaderResources(2, 1, &m_gpClosestParticlesSRV);
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

	// and with that, we're done!
	// Clean up
	ID3D11ShaderResourceView* nullSRVs3[3] = { nullptr, nullptr, nullptr };
	md3dImmediateContext->CSSetShaderResources(0, 3, nullSRVs3);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, nullUAVs, NULL);
	md3dImmediateContext->CSSetShader(NULL, NULL, 0);
}

void GPFluidSim::TransferParticlesToGrid(std::vector<Particle3> &particles) {
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
						U(ix + x, iy + y, iz + z) += w * particles[i].uX;
						uAmts[(ix + x) + (mX + 1)*((iy + y) + mZ * (iz + z))] += w;
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
						V(ix + x, iy + y, iz + z) += w * particles[i].uY;
						vAmts[(ix + x) + mX * ((iy + y) + (mY + 1)*(iz + z))] += w;
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
						W(ix + x, iy + y, iz + z) += w * particles[i].uZ;
						wAmts[(ix + x) + mX * ((iy + y) + mY * (iz + z))] += w;
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
				U(x, y, z) = (U(x, y, z) / (div_eps + uAmts[x + (mX + 1)*(y + mY * z)]));
			}
		}
	}
	for (int z = 0; z < mZ; z++) {
		for (int y = 0; y < mY + 1; y++) {
			for (int x = 0; x < mX; x++) {
				V(x, y, z) = (V(x, y, z) / (div_eps + vAmts[x + mX * (y + (mY + 1)*z)]));
			}
		}
	}
	for (int z = 0; z < mZ + 1; z++) {
		for (int y = 0; y < mY; y++) {
			for (int x = 0; x < mX; x++) {
				W(x, y, z) = (W(x, y, z) / (div_eps + wAmts[x + mX * (y + mY * z)]));
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
			uValid[0 + (mX + 1)*(y + mY * z)] = true;
			uValid[mX + (mX + 1)*(y + mY * z)] = true;
		}
	}
	// V
	for (int z = 0; z < mZ; z++) {
		for (int x = 0; x < mX; x++) {
			vValid[x + mX * (0 + (mY + 1)*z)] = true;
			vValid[x + mX * (mY + (mY + 1)*z)] = true;
		}
	}
	// W
	for (int y = 0; y < mY; y++) {
		for (int x = 0; x < mX; x++) {
			wValid[x + mX * (y + mY * 0)] = true;
			wValid[x + mX * (y + mY * mZ)] = true;
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

void GPFluidSim::ExtrapolateValues(float* srcAr, bool* validAr, int xSize, int ySize, int zSize) {
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
		}
		else {
			cd[i] = inf;
		}
	}

	// x-
	for (int x = 1; x < xSize; x++) {
		for (int z = 0; z < zSize; z++) {
			for (int y = 0; y < ySize; y++) {
				int idx = x + xSize * (y + ySize * z);
				cd[idx] = std::min(cd[idx], cd[idx - 1] + 1);
			}
		}
	}

	// x+
	for (int x = xSize - 2; x >= 0; x--) {
		for (int z = 0; z < zSize; z++) {
			for (int y = 0; y < ySize; y++) {
				int idx = x + xSize * (y + ySize * z);
				cd[idx] = std::min(cd[idx], cd[idx + 1] + 1);
			}
		}
	}

	// y-
	for (int y = 1; y < ySize; y++) {
		for (int z = 0; z < zSize; z++) {
			for (int x = 0; x < xSize; x++) {
				int idx = x + xSize * (y + ySize * z);
				cd[idx] = std::min(cd[idx], cd[idx - xSize] + 1);
			}
		}
	}

	// y+
	for (int y = ySize - 2; y >= 0; y--) {
		for (int z = 0; z < zSize; z++) {
			for (int x = 0; x < xSize; x++) {
				int idx = x + xSize * (y + ySize * z);
				cd[idx] = std::min(cd[idx], cd[idx + xSize] + 1);
			}
		}
	}

	// z-
	for (int z = 1; z < zSize; z++) {
		for (int y = 0; y < ySize; y++) {
			for (int x = 0; x < xSize; x++) {
				int idx = x + xSize * (y + ySize * z);
				cd[idx] = std::min(cd[idx], cd[idx - xSize * ySize] + 1);
			}
		}
	}

	// z+
	for (int z = zSize - 2; z >= 0; z--) {
		for (int y = 0; y < ySize; y++) {
			for (int x = 0; x < xSize; x++) {
				int idx = x + xSize * (y + ySize * z);
				cd[idx] = std::min(cd[idx], cd[idx + xSize * ySize] + 1);
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

	assert(sums[numBuckets - 1] + bucketCounts[numBuckets - 1] == xSize * ySize*zSize);

	int* indices = new int[xSize*ySize*zSize * 3];
	for (int z = 0; z < zSize; z++) {
		for (int y = 0; y < ySize; y++) {
			for (int x = 0; x < xSize; x++) {
				int c = cd[x + xSize * (y + ySize * z)];
				int index = sums[c];
				indices[3 * index] = x;
				indices[3 * index + 1] = y;
				indices[3 * index + 2] = z;
				sums[c]++;
			}
		}
	}

	// 3. Extrapolate values from the inside out.
	int directions[18] = { 1,0,0, -1,0,0, 0,1,0, 0,-1,0, 0,0,1, 0,0,-1 };
	int numDirs = 6;
	for (int i = 0; i < xSize*ySize*zSize; i++) {
		int x = indices[3 * i];
		int y = indices[3 * i + 1];
		int z = indices[3 * i + 2];
		int myc = cd[x + xSize * (y + ySize * z)];
		if (myc == 0) continue; // don't modify known values
		float numNeighbors = 0.0f;
		float neighborSum = 0.0f;

		for (int d = 0; d < numDirs; d++) {
			int nx = x + directions[3 * d];
			int ny = y + directions[3 * d + 1];
			int nz = z + directions[3 * d + 2];
			int idx = nx + xSize * (ny + ySize * nz);
			if (0 <= nx && nx < xSize
				&& 0 <= ny && ny < ySize
				&& 0 <= nz && nz < zSize
				&& (cd[idx] < myc)) {
				numNeighbors += 1.0f;
				neighborSum += srcAr[idx];
			}
		}
		assert(numNeighbors > 0.0f);
		srcAr[x + xSize * (y + ySize * z)] = neighborSum / numNeighbors;
	}

	// Clean up
	delete[] cd;
	delete[] bucketCounts;
	delete[] sums;
	delete[] indices;

	// and that's it!
}

void GPFluidSim::AddBodyForces(float dt) {
	// Just adds dt*g to the velocity field
	// We use Cartesian coordinates for our grids, hooray!
	int count = mX * (mY + 1)*mZ;
	float gdT = m_gY * dt; // in m/s
	for (int i = 0; i < count; ++i) {
		m_MV[i] += gdT;
	}
}

void GPFluidSim::AddBodyForcesGPU(float dt) {
	// Just adds dt*g to the velocity field
	SetParametersConstantBuffer(dt, 0, 0);
	md3dImmediateContext->CSSetShader(m_gpAddBodyForcesFX, NULL, 0);
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, &m_gpVUAV, NULL);
	md3dImmediateContext->Dispatch((mX + 3) / 4, (mY + 3) / 4, (mZ + 3) / 4);
	// Clean up
	ID3D11UnorderedAccessView* nullUAVs[1] = { nullptr };
	md3dImmediateContext->CSSetUnorderedAccessViews(0, 1, nullUAVs, NULL);

}

void GPFluidSim::Project(float dt) {
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
	int MN = mX * mY*mZ;
	double* b = new double[MN];
	double* p = new double[MN](); // should zero everything for us
	double* diagCoeffs = new double[MN];

	// INDEXING: b[x,y,z] = b[x+mX*(y+mY*z)].

	double solidVel = 0;
	double dx = 1.0 / m_CellsPerMeter;
	double scale = -dx * m_rho / dt;

	for (int z = 0; z < mZ; z++) {
		for (int y = 0; y < mY; y++) {
			for (int x = 0; x < mX; x++) {
				double velXp = (x == mX - 1 ? solidVel : U(x + 1, y, z));
				double velXm = (x == 0 ? solidVel : U(x, y, z));
				double velYp = (y == mY - 1 ? solidVel : V(x, y + 1, z));
				double velYm = (y == 0 ? solidVel : V(x, y, z));
				double velZp = (z == mZ - 1 ? solidVel : W(x, y, z + 1));
				double velZm = (z == 0 ? solidVel : W(x, y, z));
				b[x + mX * (y + mY * z)] = scale * (velXp - velXm + velYp - velYm + velZp - velZm);
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

				diagCoeffs[x + mX * (y + mY * z)] = numNeighbors;
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

void GPFluidSim::PrintDivergence() {
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

void GPFluidSim::SetEdgeVelocitiesToZero() {
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