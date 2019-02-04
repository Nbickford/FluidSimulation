//*****************************************************************
// FluidSimDemo.cpp
// Renders the fluid simulation from Simulation.cpp and handles
// application logic.
// Based in part on BoxDemo.cpp from Frank Luna (C) 2011.
// 
// Controls:
//   Hold the left mouse button to rotate the camera.
//   Hold the right mouse button to zoom in and out.
//   Press + to speed up the simulation by a factor of 2.
//   Press - to slow down the simulation by a factor of 2.
//   Press 0 to reset the view.
//   Press r to restart the simulation.
//
// Authors:
//   Neil Bickford
//*****************************************************************

#define FLUID_SIM_DEMO
#ifdef FLUID_SIM_DEMO

#include "d3dApp.h"
#include "FX11\d3dx11effect.h"
#include "GPUProfiler.h"
#include "MathHelper.h"
#include "odprintf.h"
#include "Simulation.h"
#include <vector>

class FluidSimDemo :public D3DApp {
public:
	FluidSimDemo(HINSTANCE hInstance);
	~FluidSimDemo();

	bool Init();
	void OnResize();
	void UpdateView();
	void UpdateScene(float dt);
	void DrawScene();

	void OnMouseDown(WPARAM btnState, int x, int y);
	void OnMouseUp(WPARAM btnState, int x, int y);
	void OnMouseMove(WPARAM btnState, int x, int y);
	void OnCharacterKey(char keyCode);

private:
	void BuildGeometryBuffers();
	void BuildFX();
	void BuildVertexLayout();
	void BuildResources();
private:
	// ALGORITHM PARAMETERS
	const int mTexWidth = 64;
	const int mTexHeight = 64;
	const int mTexDepth = 64;

	ID3DX11Effect* mPSRenderFX;
	ID3DX11EffectTechnique* mPSRenderTech;
	ID3DX11EffectMatrixVariable* mPSRenderView;
	ID3DX11EffectShaderResourceVariable* mPSRenderPhi;

	XMFLOAT4X4 mView;

	float mCamPhi; // from azimuth
	float mCamTheta;
	float mCamFOV = MathHelper::Pi/3.0f; // vertical field of view (in radians) - initially 60 degrees

	POINT mLastMousePos;

	// Local game state
	float totalTime = 0.0f; // NOTE: This is a problem, since it'll start to have problems after ~ 77 hours.
	GPFluidSim fluidSim;
    GPUProfiler mProfiler;
};

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE prevInstance,
	PSTR cmdLine, int showCmd)
{
	// Don't enable run-time memory check for debug builds.

	FluidSimDemo theApp(hInstance);
	if (!theApp.Init()) {
		return 0;
	}

	return theApp.Run();
}

// Constructor
FluidSimDemo::FluidSimDemo(HINSTANCE hInstance)
	:D3DApp(hInstance),
	mCamPhi(3.1415926535f / 2.0f), mCamTheta(0.0f),
	fluidSim(mTexWidth, mTexHeight, mTexDepth, (float)mTexWidth)
{
	mMainWndCaption = L"Fluid Simulation Demo";

	mLastMousePos.x = 0;
	mLastMousePos.y = 0;

	XMMATRIX I = XMMatrixIdentity();
	XMStoreFloat4x4(&mView, I);
}

// Destructor
FluidSimDemo::~FluidSimDemo() {
	fluidSim.ReleaseResources();
    mProfiler.ReleaseResources();

	// Release effects
	// TODO (neil): Should we release effect variables and techniques as well?
	// The original Luna code doesn't release them, but this should at least
	// reduce the number of possibilities we need to work through when debugging
	// cleaning up references at the end of the application.
	ReleaseCOM(mPSRenderPhi);
	ReleaseCOM(mPSRenderView);
	ReleaseCOM(mPSRenderTech);
	ReleaseCOM(mPSRenderFX);
}

// Initialization and loading
bool FluidSimDemo::Init() {
	if (!D3DApp::Init()) {
		return false;
	}

	fluidSim.Initialize(md3dDevice, md3dImmediateContext);
    mProfiler.Initialize(md3dDevice);

	BuildGeometryBuffers();
	BuildFX();
	BuildVertexLayout();
	BuildResources();

	return true;
}

void FluidSimDemo::OnResize() {
	// Resize back buffer, depth/stencil buffers
	D3DApp::OnResize();

	UpdateView();
}

void FluidSimDemo::UpdateView() {
	// Build the view matrix
	float rad = 1.5f; // Radius of the camera (...a compile-time constant)
	XMVECTOR pos = XMVectorSet(
		rad*sinf(mCamTheta)*sinf(mCamPhi),
		rad*cosf(mCamPhi),
		-rad * cosf(mCamTheta)*sinf(mCamPhi), 1.0f);
	XMVECTOR target = XMVectorSet(0.0f, 0.0f, 0.0f, 0.0f);
	XMVECTOR up = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);

	XMMATRIX V = XMMatrixLookAtLH(pos, target, up);
	V = XMMatrixInverse(nullptr, V); // Transform back to the matrix W (DX11, pg. 164)
	// V now stores 90* FOVs in the horizontal and vertical axes; we want to change this
	// to be a more reasonable FOV in the vertical axis and match the screen size in the horizontal axis.
	// tan(vFOV*/2) = new length of vertical axis
	float vScale = tanf(0.5f*mCamFOV);
	float hScale = (vScale*mClientWidth) / mClientHeight;
	V = XMMatrixMultiply(XMMatrixScaling(hScale, vScale, 1.0f), V);
	XMStoreFloat4x4(&mView, V);
}

void FluidSimDemo::UpdateScene(float dt) {
    mProfiler.BeginFrame(md3dImmediateContext);

	totalTime += dt;

	UpdateView();

	fluidSim.Simulate(dt);
    
    mProfiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_UPDATE);
}

void FluidSimDemo::DrawScene() {
	// Clear render targets
	md3dImmediateContext->ClearRenderTargetView(mRenderTargetView,
		reinterpret_cast<const float*>(&Colors::LightSteelBlue));
	md3dImmediateContext->ClearDepthStencilView(mDepthStencilView,
		D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

	// See https://msdn.microsoft.com/en-us/library/windows/desktop/bb232912(v=vs.85).aspx
	md3dImmediateContext->IASetInputLayout(NULL);
	md3dImmediateContext->IASetPrimitiveTopology(
		D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	// Set render parameters here
	D3DX11_TECHNIQUE_DESC techDesc;
	mPSRenderView->SetMatrix(reinterpret_cast<float*>(&mView));
	mPSRenderPhi->SetResource(fluidSim.m_gpPhiSRV);

	// We don't need to set any index buffers - all three triangle vertices are drawn from the GPU!
	md3dImmediateContext->IASetIndexBuffer(NULL, DXGI_FORMAT_UNKNOWN, 0);

	mPSRenderTech->GetDesc(&techDesc);

	for (UINT p = 0; p < techDesc.Passes; ++p) {
		mPSRenderTech->GetPassByIndex(p)->Apply(0, md3dImmediateContext);
		// Draw the triangle
		md3dImmediateContext->Draw(3, 0);
	}

    mProfiler.TimestampComplete(md3dImmediateContext, GPU_PROFILER_MARK_DRAW);

	// Unbind SRVs here
	mPSRenderPhi->SetResource(NULL);
	mPSRenderTech->GetPassByIndex(0)->Apply(0, md3dImmediateContext);
	HR(mSwapChain->Present(0, 0));

    mProfiler.EndFrame(md3dImmediateContext);
    odprintf("GPU time:\t%.2fms\t%.2fms\t%.2fms",
        1000.0 * mProfiler.DT(GPU_PROFILER_MARK_UPDATE),
        1000.0 * mProfiler.DT(GPU_PROFILER_MARK_DRAW),
        1000.0 * mProfiler.DT(GPU_PROFILER_MARK_END_FRAME));
}

// Updating
void FluidSimDemo::OnMouseDown(WPARAM btnState, int x, int y) {
	mLastMousePos.x = x;
	mLastMousePos.y = y;

	SetCapture(mhMainWnd); // hmm, interesting
}

void FluidSimDemo::OnMouseUp(WPARAM btnState, int x, int y) {
	ReleaseCapture();
}

void FluidSimDemo::OnMouseMove(WPARAM btnState, int x, int y) {
	// Rotational (3D) version
	if ((btnState & MK_LBUTTON) != 0) {
		// Make each pixel correspond to a quarter of a degree.
		float dx = XMConvertToRadians(
			0.25f*static_cast<float>(x - mLastMousePos.x));
		float dy = XMConvertToRadians(
			0.25f*static_cast<float>(y - mLastMousePos.y));

		// Update angles based on input to orbit camera around box.
		mCamTheta -= dx;
		mCamPhi -= dy;

		// Restrict the angle mPhi.
		mCamPhi = MathHelper::Clamp(mCamPhi, 0.1f, MathHelper::Pi - 0.1f);
	}
	if ((btnState & MK_RBUTTON) != 0) {
		// Zoom in or out of the scene.
		// Imagine our camera has its viewscreen right in front of it.
		// When we zoom in, we want to decrease the height of this viewscreen by some
		// exponential amount. Similarly, when we zoom out, we want to increase
		// the height of this viewscreen by some exponential amount.

		// Compute dy.
		float dy = static_cast<float>(y - mLastMousePos.y);

		// Compute the current (unitless) height of this virtual viewscreen.
		float vHeight = 2.0f*tanf(mCamFOV / 2.0f);

		// Compute the scaling factor, a^(-dy).
		// We choose a so that a^(mClientHeight) = 16, say:
		float a = powf(16.0f, 1.0f / mClientHeight);

		vHeight *= powf(a, -dy);

		// Transfer this back to the camera's field of view.
		mCamFOV = 2.0f*atanf(vHeight / 2.0f);
		
	}

	mLastMousePos.x = x;
	mLastMousePos.y = y;
}

void FluidSimDemo::OnCharacterKey(char keyCode) {
	switch (keyCode) {
	case '+':
	case '=': // b/c user doesn't think they need to hold down shift
		// Increase the timestep used in the fluid simulation
		fluidSim.IncreaseSpeed();
		break;
	case '-':
		// Decrease the timestep used in the fluid simulation
		fluidSim.DecreaseSpeed();
		break;
	case '0':
		// Reset view
		mCamPhi = 3.14159f / 2.0f;
		mCamTheta = 0.0f;
		mCamFOV = MathHelper::Pi / 3.0f;
		break;
	case 'r':
		// Reset simulation
		fluidSim.ResetSimulation();
		break;
	}
}

void FluidSimDemo::BuildGeometryBuffers() {
	// Normally, we'd create all of the vertex and index buffers
	// we'd need here.
	// However, since we switched to a Shadertoy-style renderer,
	// we don't actually need any geometry anymore!
	// In case we ever decide to add in dust particles, we should
	// check at the previous versions of this code in the Git
	// repository or in the Old versions of FluidSimDemo.cpp
	// to see how to define a vertex buffer, input layout, and
	// relevant shaders.
}

void FluidSimDemo::BuildResources() {
	// Normally, we'd create our textures for rendering here.
	// However, since we got rid of the textured quad for debugging, we no longer
	// need to have any textures!
	// Still, this is where we'd load and create our resources (if the application needed any for rendering
	// beyond the textures in d3dApp.cpp)
}



// Build FX
void FluidSimDemo::BuildFX() {
	DWORD shaderFlags = 0;
#if defined(DEBUG) || defined(_DEBUG)
	shaderFlags |= D3D10_SHADER_DEBUG;
	//shaderFlags |= D3D10_SHADER_SKIP_OPTIMIZATION;
#endif

	//--------------------------------
	// FULL RENDER FX
	//--------------------------------
	ID3DBlob* compiledShader = d3dUtil::CompileShader(L"FX\\Render.fx", nullptr, "", "fx_5_0");

	HR(D3DX11CreateEffectFromMemory(
		compiledShader->GetBufferPointer(),
		compiledShader->GetBufferSize(),
		0, md3dDevice, &mPSRenderFX));

	ReleaseCOM(compiledShader);

	mPSRenderTech = mPSRenderFX->GetTechniqueByName("ColorTech");
	// Get references to variables
	mPSRenderView = mPSRenderFX->GetVariableByName("mView")->AsMatrix();
	mPSRenderPhi = mPSRenderFX->GetVariableByName("gPhi")->AsShaderResource();
}

void FluidSimDemo::BuildVertexLayout() {
	// Since we don't have vertices, we don't have vertex layouts!
}
#endif