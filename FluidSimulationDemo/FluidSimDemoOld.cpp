//*****************************************************************
// BoxDemo.cpp by Frank Luna (C) 2011 All Rights Reserved.
// Reconstructed for DX11 without D3DX in 2017.
//
// Demonstrates rendering a colored box.
// 
// Controls:
//   Hold the right mouse button down to zoom in and out.
//   Press + to zoom in by a factor of 2.
//   Press - to zoom out by a factor of 2.
//   Press 0 to return to a centered pixel-for-pixel zoom level.
//   Hold the left mouse button down and move the mouse to pan.
//
//*****************************************************************

#if 0
#define BOX_DEMO
#ifdef BOX_DEMO

#include "d3dApp.h"
#include "FX11\d3dx11effect.h"
#include "MathHelper.h"
#include "odprintf.h"
#include "Simulation3D.h"
#include <vector>

struct Vertex {
	XMFLOAT3 Pos;
	XMFLOAT2 UV;
};

struct Point {
	XMFLOAT3 Pos;
};

class BoxApp :public D3DApp {
public:
	BoxApp(HINSTANCE hInstance);
	~BoxApp();

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
	const int mTexWidth = 16;
	const int mTexHeight = 16;
	const int mTexDepth = 16;

	// INPUT LAYOUTS
	// Layout: Position {x32, y32, z32}
	//         Texture  {u32, v32}
	// Matches definition of struct Vertex above.
	ID3D11InputLayout* mInputLayout;
	// Layout: Position {x32, y32, z32}
	// Matches definition of struct Point above.
	ID3D11InputLayout* mPointsInputLayout;

	// MESHES
	ID3D11Buffer* mQuadVB; // Vertex buffer for the textured quad
	ID3D11Buffer* mQuadIB; // Index buffer for the textured quad

	ID3D11Buffer* mPointVB; // Vertex buffer for debug visualization points.
	ID3D11Buffer* mPointIB; // Index buffer for debug visualization points
	UINT mPointCount = 0;

	// TEXTURES
	ID3D11Texture2D* mDiffuseMap; // We keep this so that we can modify it dynamically
	ID3D11ShaderResourceView* mDiffuseMapSRV;

	// EFFECTS
	ID3DX11Effect* mFX;
	ID3DX11EffectTechnique* mTech;
	// Effect variables
	ID3DX11EffectMatrixVariable* mfxWorldViewProj;
	ID3DX11EffectShaderResourceVariable* mfxDiffuseMap;

	ID3DX11Effect* mDebugPointsFX;
	ID3DX11EffectTechnique* mDebugPointsTech;
	ID3DX11EffectMatrixVariable* mDebugPointsFXWorld;
	ID3DX11EffectMatrixVariable* mDebugPointsFXViewProj;
	ID3DX11EffectScalarVariable* mDebugPointsFXPtSize;

	XMFLOAT4X4 mWorld;
	XMFLOAT4X4 mView;
	XMFLOAT4X4 mProj;

	float mZoomFactor;
	float mCameraPositionX;
	float mCameraPositionY;

	POINT mLastMousePos;

	// Local game state
	float totalTime = 0.0f; // NOTE: This is a problem, since it'll start to have problems after ~ 77 hours.
	FluidSim fluidSim;
};

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE prevInstance,
	PSTR cmdLine, int showCmd)
{
	// Don't enable run-time memory check for debug builds.

	BoxApp theApp(hInstance);
	if (!theApp.Init()) {
		return 0;
	}

	return theApp.Run();
}

// Constructor
BoxApp::BoxApp(HINSTANCE hInstance)
	:D3DApp(hInstance), mQuadVB(0), mQuadIB(0), mFX(0), mTech(0), mInputLayout(0),
	mZoomFactor(8.0f), mCameraPositionX(0.0f), mCameraPositionY(0.0f),
	mDiffuseMap(0), mDiffuseMapSRV(0),
	fluidSim(mTexWidth, mTexHeight, (float)mTexWidth)
{
	mMainWndCaption = L"Fluid Simulation Demo";

	mLastMousePos.x = 0;
	mLastMousePos.y = 0;

	XMMATRIX I = XMMatrixIdentity();
	XMStoreFloat4x4(&mWorld, I);
	XMStoreFloat4x4(&mView, I);
	XMStoreFloat4x4(&mProj, I);
}

// Destructor
BoxApp::~BoxApp() {
	ReleaseCOM(mQuadVB);
	ReleaseCOM(mQuadIB);
	ReleaseCOM(mPointVB);
	ReleaseCOM(mPointIB);

	// Resources
	ReleaseCOM(mDiffuseMapSRV);
	ReleaseCOM(mDiffuseMap);

	// Release effects
	ReleaseCOM(mFX);
	ReleaseCOM(mDebugPointsFX);

	// Release input layouts
	ReleaseCOM(mInputLayout);
	// TODO(neil): Why not release mTech?
}

// Initialization and loading
bool BoxApp::Init() {
	if (!D3DApp::Init()) {
		return false;
	}

	BuildGeometryBuffers();
	BuildFX();
	BuildVertexLayout();
	BuildResources();

	return true;
}

void BoxApp::OnResize() {
	// Resize back buffer, depth/stencil buffers
	D3DApp::OnResize();

	// The window resized, so update the aspect ratio and recompute
	// the projection matrix.
	// XMMATRIX P = XMMatrixPerspectiveFovLH(0.25f*MathHelper::Pi,
	// 	AspectRatio(), 1.0f, 1000.0f);
	
	UpdateView();
}

void BoxApp::UpdateView() {
	// Build the view matrix
	XMVECTOR pos = XMVectorSet(mCameraPositionX, mCameraPositionY, -1.0f, 1.0f);
	XMVECTOR target = XMVectorSet(mCameraPositionX, mCameraPositionY, 0.0f, 0.0f);
	XMVECTOR up = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);

	XMMATRIX V = XMMatrixLookAtLH(pos, target, up);
	XMStoreFloat4x4(&mView, V);

	// We want the quad in the center, which measures [-1,1]x[-1,1] in world space, to cover
	// exactly (times zoomFactor) mTexWidthxmTexHeight pixels.
	// so 2/[world screen width] = mZoomFactor*mTexWidth/[pixel screen width].
	// TODO: BUG: If we zoom in too far, the view height becomes too small, which
	// causes an assertion failure in the XMMath library.
	// This also fails when the client width or height is equal to 0, which occurs
	// whenever we minimize the window.
	XMMATRIX P = XMMatrixOrthographicLH((2.0f*mClientWidth) / (mZoomFactor*mTexWidth),
		(2.0f*mClientHeight) / (mZoomFactor*mTexHeight),
		0.5f,
		2.0f);
	XMStoreFloat4x4(&mProj, P);

	// Interesting problem: at zoom factor 1, this fails when mClientWidth is odd.
	// I think this is because the centers of the texels of the quad no longer match
	// with the centers of the pixels of the screen.
}

void BoxApp::UpdateScene(float dt) {
	totalTime += dt;

	UpdateView();

	fluidSim.Simulate(dt);

	// Update the texture on the quad - in this case, using a nice sine wave!
	// This is obviously not the best way to do this, but this is just to get to the
	// state where this is possible.
	/*const int bpp = 4;
	char* newImg = new char[bpp*mTexWidth*mTexHeight];
	for (int y = 0; y < mTexHeight; y++) {
		for (int x = 0; x < mTexWidth; x++) {
			// Since our texture is RGBA, we store the values in ABGR format.
			float uvX = static_cast<float>(x)/mTexWidth;
			float uvY = 1.0f - static_cast<float>(y)/mTexHeight;
			float r = 0.5f - 0.5f*cosf(XM_PI*uvX);
			float g = 0.5f - 0.5f*cosf(XM_PI*uvY);
			float b = 0.5f + 0.5f*sinf(totalTime);
			newImg[bpp*(x + mTexWidth*y) + 3] = (char)255; // A
			newImg[bpp*(x + mTexWidth*y) + 2] = (char)(255 * b); // B
			newImg[bpp*(x + mTexWidth*y) + 1] = (char)(255 * g); // G
			newImg[bpp*(x + mTexWidth*y) + 0] = (char)(255 * r); // R
		}
	}
	// Update subresource
	md3dImmediateContext->UpdateSubresource(mDiffuseMap, 0, NULL, (const void*)newImg, bpp*mTexWidth, 0);
	// Generate mipmaps
	md3dImmediateContext->GenerateMips(mDiffuseMapSRV);
	// Clean up
	delete[] newImg; // hopefully this is ok*/

	// Update the points from the fluid simulation
	mPointCount = (UINT)fluidSim.m_particles.size();
	Point* newPoints = new Point[mPointCount];
	for (UINT i = 0; i < mPointCount; i++) {
		newPoints[i].Pos = XMFLOAT3(2.0f*(fluidSim.m_particles[i].X+0.5f/mTexWidth)-1.0f,
			2.0f*(fluidSim.m_particles[i].Y+0.5f/mTexHeight)-1.0f,
			-0.1f);
	}

	D3D11_MAPPED_SUBRESOURCE mappedResource;
	ZeroMemory(&mappedResource, sizeof(D3D11_MAPPED_SUBRESOURCE));
	md3dImmediateContext->Map(mPointVB, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
	memcpy(mappedResource.pData, newPoints, sizeof(Point)*mPointCount);
	md3dImmediateContext->Unmap(mPointVB, 0);

	delete[] newPoints;
}

void BoxApp::DrawScene() {
	// Clear render targets
	md3dImmediateContext->ClearRenderTargetView(mRenderTargetView,
		reinterpret_cast<const float*>(&Colors::Black));
	md3dImmediateContext->ClearDepthStencilView(mDepthStencilView,
		D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

	// Set input layout and bind vertex buffers
	md3dImmediateContext->IASetInputLayout(mInputLayout);
	md3dImmediateContext->IASetPrimitiveTopology(
		D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	UINT stride = sizeof(Vertex);
	UINT offset = 0;
	md3dImmediateContext->IASetVertexBuffers(0, 1, &mQuadVB,
		&stride, &offset);
	md3dImmediateContext->IASetIndexBuffer(mQuadIB,
		DXGI_FORMAT_R32_UINT, 0);

	// Set constants
	XMMATRIX world = XMLoadFloat4x4(&mWorld);
	XMMATRIX view = XMLoadFloat4x4(&mView);
	XMMATRIX proj = XMLoadFloat4x4(&mProj);
	XMMATRIX viewProj = view*proj;
	XMMATRIX worldViewProj = world*view*proj;

	mfxWorldViewProj->SetMatrix(reinterpret_cast<float*>(&worldViewProj));
	mfxDiffuseMap->SetResource(mDiffuseMapSRV);

	D3DX11_TECHNIQUE_DESC techDesc;
	mTech->GetDesc(&techDesc);
	for (UINT p = 0; p < techDesc.Passes; ++p) {
		mTech->GetPassByIndex(p)->Apply(0, md3dImmediateContext);

		// 6 indices for the quad
		md3dImmediateContext->DrawIndexed(6, 0, 0);
	}

	// Draw debug points
	// in this case, as triangles!
	stride = sizeof(Point);
	offset = 0;
	
	md3dImmediateContext->IASetInputLayout(mPointsInputLayout);
	md3dImmediateContext->IASetPrimitiveTopology(
		D3D11_PRIMITIVE_TOPOLOGY_POINTLIST);

	// Each point should be about 1/4 of a grid cell size, so:
	// (total width)/(4*(number of cells))
	// However, from the projection calculation above, we should also have that since [-1,1] in x in world-space covers
	// zoomFactor*mTexWidth pixels (magnification of (zoomFactor*mTexWidth)/2), the point size should also be at least 
	// 2/(zoomFactor*mTexWidth).
	mDebugPointsFXPtSize->SetFloat(max(2.0f/(4.0f*mTexWidth),
		2.0f/(mZoomFactor*mTexWidth))); // NOTE: Assumes mTexWidth==mTexHeight!
	mDebugPointsFXWorld->SetMatrix(reinterpret_cast<float*>(&world));
	mDebugPointsFXViewProj->SetMatrix(reinterpret_cast<float*>(&viewProj));

	md3dImmediateContext->IASetVertexBuffers(0, 1, &mPointVB,
		&stride, &offset);
	md3dImmediateContext->IASetIndexBuffer(mPointIB,
		DXGI_FORMAT_R32_UINT, 0);

	mDebugPointsTech->GetDesc(&techDesc);

	for (UINT p = 0; p < techDesc.Passes; ++p) {
		mDebugPointsTech->GetPassByIndex(p)->Apply(0, md3dImmediateContext);

		UINT numIndices = mPointCount;
		md3dImmediateContext->DrawIndexed(numIndices, 0, 0);
	}

	HR(mSwapChain->Present(0, 0));
}

// Updating
void BoxApp::OnMouseDown(WPARAM btnState, int x, int y) {
	mLastMousePos.x = x;
	mLastMousePos.y = y;

	SetCapture(mhMainWnd); // hmm, interesting
}

void BoxApp::OnMouseUp(WPARAM btnState, int x, int y) {
	ReleaseCapture();
}

void BoxApp::OnMouseMove(WPARAM btnState, int x, int y) {
	// Orthographic (2D) version
	if ((btnState & MK_LBUTTON) != 0) {
		// Each pixel corresponds to one screen pixel.
		// We know that the projection matrix satisfies the property
		// [0,yWorldMax,0,0]P = [0,1,...,...]
		// so moving mClientHeight/2 pixels up corresponds to moving up yWorldMax world units.
		// Since P is of the form
		// [m11,   0,  0, 0]
		// [0  , m22,  0, 0]
		// [0  ,   0,...,..]
		// [0  ,   0,...,..]
		// we have yWorldMax = 1/m22, and we can compute the appropriate pixel->world scaling factor.
		float p2WorldSF = 2.0f / (mProj._22*mClientHeight);
		float dx = p2WorldSF*static_cast<float>(x - mLastMousePos.x);
		float dy = p2WorldSF*static_cast<float>(y - mLastMousePos.y);
		mCameraPositionX -= dx;
		mCameraPositionY += dy;
	}
	if ((btnState & MK_RBUTTON) != 0) {
		// We just make each pixel correspond to some small factor.
		// Say, a^(mClientWidth pixels) = 512, so
		float dx = static_cast<float>(x - mLastMousePos.x);
		float dy = static_cast<float>(y - mLastMousePos.y);
		float fac = powf(512.0f, (dx-dy) / mClientWidth);
		mZoomFactor *= fac;
	}
	/*if ((btnState & MK_LBUTTON) != 0) {
		// Make each pixel correspond to a quarter of a degree.
		float dx = XMConvertToRadians(
			0.25f*static_cast<float>(x - mLastMousePos.x));
		float dy = XMConvertToRadians(
			0.25f*static_cast<float>(y - mLastMousePos.y));

		// Update angles based on input to orbit camera around box.
		mTheta += dx;
		mPhi += dy;

		// Restrict the angle mPhi.
		mPhi = MathHelper::Clamp(mPhi, 0.1f, MathHelper::Pi - 0.1f);
	}
	else if ((btnState & MK_RBUTTON) != 0) {
		// Make each pixel correspond to 0.005 unit in the scene.
		float dx = 0.005f*static_cast<float>(x - mLastMousePos.x);
		float dy = 0.005f*static_cast<float>(y - mLastMousePos.y);

		// Update the camera radius based on input.
		mRadius += dx - dy;

		// Restrict the radius.
		mRadius = MathHelper::Clamp(mRadius, 3.0f, 15.0f);
	}*/

	mLastMousePos.x = x;
	mLastMousePos.y = y;
}

void BoxApp::OnCharacterKey(char keyCode) {
	switch (keyCode) {
	case '+':
	case '=': // b/c user doesn't think they need to hold down shift
		mZoomFactor *= 2.0f;
		break;
	case '-':
		mZoomFactor *= 0.5f;
		break;
	case '0':
		mZoomFactor = 1.0f;
		mCameraPositionX = 0.0f; // TODO: fix this for the case of non-even windows
		mCameraPositionY = 0.0f;
		break;
	case 'r':
		fluidSim.ResetSimulation();
		break;
	//default:
	//	odprintf("Unknown character: %c\n", keyCode);
	//	break;
	}
}

void BoxApp::BuildGeometryBuffers() {
	// Create vertex buffer and send it to the GPU
	Vertex vertices[] =
	{
		{ XMFLOAT3(-1.0f, -1.0f,  0.0f), XMFLOAT2(0.0f, 1.0f) },
		{ XMFLOAT3(-1.0f, +1.0f,  0.0f), XMFLOAT2(0.0f, 0.0f) },
		{ XMFLOAT3(+1.0f, +1.0f,  0.0f), XMFLOAT2(1.0f, 0.0f) },
		{ XMFLOAT3(+1.0f, -1.0f,  0.0f), XMFLOAT2(1.0f, 1.0f) }
	};

	// Create the buffer on the device; get a pointer to the ID3D11Buffer, which is mQuadVB.
	D3D11_BUFFER_DESC vbd;
	vbd.Usage = D3D11_USAGE_IMMUTABLE;
	vbd.ByteWidth = sizeof(Vertex) * 4; // Size of the buffer in bytes
	vbd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
	vbd.CPUAccessFlags = 0;
	vbd.MiscFlags = 0;
	vbd.StructureByteStride = 0;
	D3D11_SUBRESOURCE_DATA vinitData; // Declare initial memory to fill the buffer with
	vinitData.pSysMem = vertices;
	HR(md3dDevice->CreateBuffer(&vbd, &vinitData, &mQuadVB));

	// Create the index buffer...
	UINT indices[] = {
		// front face
		0, 1, 2,
		0, 2, 3,
	};

	// ...and send it to the GPU.
	D3D11_BUFFER_DESC ibd; // Create a buffer
	ibd.Usage = D3D11_USAGE_IMMUTABLE;
	ibd.ByteWidth = sizeof(UINT) * 36; // Size of the buffer in bytes
	ibd.BindFlags = D3D11_BIND_INDEX_BUFFER; // Can be bound as an index buffer
	ibd.CPUAccessFlags = 0;
	ibd.MiscFlags = 0;
	ibd.StructureByteStride = 0;
	D3D11_SUBRESOURCE_DATA iinitData; // Describe what data to initially fill it with
	iinitData.pSysMem = indices;
	HR(md3dDevice->CreateBuffer(&ibd, &iinitData, &mQuadIB));


	//********************************************************
	// POINTS
	//********************************************************
	// We'll create an empty vertex buffer, which we'll then fill in.
	UINT count = 4 * mTexWidth*mTexHeight;
	D3D11_BUFFER_DESC vbd2;
	vbd2.ByteWidth = sizeof(Point) * count; // HARDCODED :(
	vbd2.Usage = D3D11_USAGE_DYNAMIC; // That's right
	vbd2.BindFlags = D3D11_BIND_VERTEX_BUFFER;
	vbd2.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
	vbd2.MiscFlags = 0;
	vbd2.StructureByteStride = 0;
	HR(md3dDevice->CreateBuffer(&vbd2, NULL, &mPointVB));

	// Create the index buffer...
	UINT* indices2 = new UINT[count];
	for (UINT i = 0; i < count; i++) {
		indices2[i] = i;
	}
	D3D11_BUFFER_DESC ibd2;
	ibd2.ByteWidth = sizeof(UINT)*count;
	ibd2.Usage = D3D11_USAGE_IMMUTABLE;
	ibd2.BindFlags = D3D11_BIND_INDEX_BUFFER;
	ibd2.CPUAccessFlags = 0;
	ibd2.MiscFlags = 0;
	ibd2.StructureByteStride = 0;
	D3D11_SUBRESOURCE_DATA iinitData2; // Describe what data to initially fill it with
	iinitData2.pSysMem = indices2;
	HR(md3dDevice->CreateBuffer(&ibd2, &iinitData2, &mPointIB));

	delete[] indices2;
}

void BoxApp::BuildResources() {
	// Create the ID3D11 resource, which in this case will be a mTexHeight*mTexWidth checkerboard texture.
	D3D11_TEXTURE2D_DESC t2dDesc;
	t2dDesc.Width = mTexWidth;
	t2dDesc.Height = mTexHeight;
	t2dDesc.MipLevels = 0; // generate full set of subresources
	t2dDesc.ArraySize = 1;
	t2dDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	t2dDesc.SampleDesc.Count = 1;
	t2dDesc.SampleDesc.Quality = 0;
	t2dDesc.Usage = D3D11_USAGE_DEFAULT; // Since we want to read and write from the GPU, and update texture subresources.
	t2dDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_RENDER_TARGET;
	t2dDesc.CPUAccessFlags = 0;
	t2dDesc.MiscFlags = D3D11_RESOURCE_MISC_GENERATE_MIPS;

	//...and create the texture.
	HR(md3dDevice->CreateTexture2D(&t2dDesc, NULL, &mDiffuseMap));

	// Construct the memory with which to fill the first level of the texture
	// Since we're using R8G8B8A8_UNORM, the bytes are in ABGR format.
	// See https://msdn.microsoft.com/en-us/library/windows/desktop/bb173059(v=vs.85).aspx for more info.
	// Update: Or are they?!?!?!?!?
	const int bpp = 4; // bytes per pixel

	char* colorData = new char[bpp*mTexWidth*mTexHeight];
	for (int y = 0; y < mTexHeight; y++) {
		for (int x = 0; x < mTexWidth; x++) {
			if ((x + y) % 2 == 0) {
				colorData[bpp*(y*mTexWidth + x) + 0] = (char)32; // R
				colorData[bpp*(y*mTexWidth + x) + 1] = (char)32; // G
				colorData[bpp*(y*mTexWidth + x) + 2] = (char)32; // B
				colorData[bpp*(y*mTexWidth + x) + 3] = (char)255; // A
			} else {
				colorData[bpp*(y*mTexWidth + x) + 0] = (char)16; // R
				colorData[bpp*(y*mTexWidth + x) + 1] = (char)16; // G
				colorData[bpp*(y*mTexWidth + x) + 2] = (char)16; // B
				colorData[bpp*(y*mTexWidth + x) + 3] = (char)255; // A
			}
		}
	}

	// Replace the first level of the subresource
	md3dImmediateContext->UpdateSubresource(mDiffuseMap, 0, NULL, (const void*)colorData, bpp*mTexWidth, 0);

	// Finally, create the shader resource view.
	D3D11_SHADER_RESOURCE_VIEW_DESC srvDesc;
	srvDesc.Format = t2dDesc.Format;
	srvDesc.ViewDimension = D3D_SRV_DIMENSION_TEXTURE2D;
	srvDesc.Texture2D.MostDetailedMip = 0;
	srvDesc.Texture2D.MipLevels = -1;

	HR(md3dDevice->CreateShaderResourceView(mDiffuseMap, &srvDesc, &mDiffuseMapSRV));

	//...and generate mipmaps for the diffuse map.
	md3dImmediateContext->GenerateMips(mDiffuseMapSRV);

	delete[] colorData;
}



// Build FX
void BoxApp::BuildFX() {
	DWORD shaderFlags = 0;
#if defined(DEBUG) || defined(_DEBUG)
	shaderFlags |= D3D10_SHADER_DEBUG;
	//shaderFlags |= D3D10_SHADER_SKIP_OPTIMIZATION;
#endif

	//--------------------------------
	// BASIC FX
	//--------------------------------
	ID3DBlob* compiledShader = d3dUtil::CompileShader(L"FX\\Basic.fx", nullptr, "", "fx_5_0");

	HR(D3DX11CreateEffectFromMemory(
		compiledShader->GetBufferPointer(),
		compiledShader->GetBufferSize(),
		0, md3dDevice, &mFX));

	// Done with the compiled shader.
	ReleaseCOM(compiledShader);

	mTech = mFX->GetTechniqueByName("ColorTech");
	mfxWorldViewProj = mFX->GetVariableByName(
		"gWorldViewProj")->AsMatrix();
	mfxDiffuseMap = mFX->GetVariableByName(
		"gDiffuseMap")->AsShaderResource();

	//--------------------------------
	// DEBUG POINTS FX
	//--------------------------------
	compiledShader = d3dUtil::CompileShader(L"FX\\DebugPointsQuads.fx", nullptr, "", "fx_5_0");

	HR(D3DX11CreateEffectFromMemory(
		compiledShader->GetBufferPointer(),
		compiledShader->GetBufferSize(),
		0, md3dDevice, &mDebugPointsFX));

	ReleaseCOM(compiledShader);

	mDebugPointsTech = mDebugPointsFX->GetTechniqueByName("ColorTech");
	mDebugPointsFXPtSize = mDebugPointsFX->GetVariableByName("gPtSize")->AsScalar();
	mDebugPointsFXWorld = mDebugPointsFX->GetVariableByName("gWorld")->AsMatrix();
	mDebugPointsFXViewProj = mDebugPointsFX->GetVariableByName("gViewProj")->AsMatrix();
}

void BoxApp::BuildVertexLayout() {
	// Create the vertex input layout.
	D3D11_INPUT_ELEMENT_DESC vertexDesc[] =
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0,
		D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 12,
		D3D11_INPUT_PER_VERTEX_DATA, 0 }
	};

	// Create the input layout
	D3DX11_PASS_DESC passDesc;
	mTech->GetPassByIndex(0)->GetDesc(&passDesc);
	HR(md3dDevice->CreateInputLayout(vertexDesc, 2,
		passDesc.pIAInputSignature,
		passDesc.IAInputSignatureSize, &mInputLayout));

	//----------------------------------
	// POINTS
	//----------------------------------
	D3D11_INPUT_ELEMENT_DESC pointsDesc[]=
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0,
		D3D11_INPUT_PER_VERTEX_DATA, 0}
	};

	// Create the input layout
	mDebugPointsTech->GetPassByIndex(0)->GetDesc(&passDesc);
	HR(md3dDevice->CreateInputLayout(pointsDesc, 1,
		passDesc.pIAInputSignature,
		passDesc.IAInputSignatureSize, &mPointsInputLayout));
}
#endif
#endif