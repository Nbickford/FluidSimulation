// d3dApp.cpp. Originally from Frank Luna, reconstructed here.

#define NOMINMAX // thanks Windows
#include "d3dApp.h"
#include <windowsx.h> // contains lots of useful macros for Windows programming.
#include <sstream>
#define DX11_ODPRINTF_IMPLEMENTATION
#include "odprintf.h"

#pragma comment(lib,"d3d11.lib")
#pragma comment(lib, "dxgi.lib")

// This is a hack to add a callback. Would be nice if there were a better way of doing this.
// (notably, restricts things to one d3dapp per application, which makes sense.)
// From GitHub.
namespace
{
	// This is just used to forward Windows messages from a global window
	// procedure to our member function window procedure because we cannot
	// assign a member function to WNDCLASS::lpfnWndProc.
	D3DApp* gd3dApp = 0;
}

LRESULT CALLBACK
MainWndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	// Forward hwnd on because we can get messages (e.g., WM_CREATE)
	// before CreateWindow returns, and thus before mhMainWnd is valid.
	return gd3dApp->MsgProc(hwnd, msg, wParam, lParam);
}

D3DApp::D3DApp(HINSTANCE hInstance)
	: mhAppInst(hInstance),
	mhMainWnd(0),
	mAppPaused(false),
	mMinimized(false),
	mMaximized(false),
	mResizing(false),
	m4xMsaaQuality(0),

	md3dDevice(0),
	md3dImmediateContext(0),
	mSwapChain(0),
	mDepthStencilBuffer(0),
	mRenderTargetView(0),
	mDepthStencilView(0),

	mMainWndCaption(L"D3D11 Application"),
	md3dDriverType(D3D_DRIVER_TYPE_HARDWARE),
	mClientWidth(800),
	mClientHeight(600),
	mEnable4xMsaa(false) {

	gd3dApp = this;
}

D3DApp::~D3DApp() {
	// Release in roughly reverse order, by hierarchy.
	// This probably doesn't matter?
	// Views
	ReleaseCOM(mRenderTargetView);
	ReleaseCOM(mDepthStencilView);

	// Swap chain, then resources
	ReleaseCOM(mSwapChain);
	ReleaseCOM(mDepthStencilBuffer);

	// Restore all default settings
	if (md3dImmediateContext) {
		md3dImmediateContext->ClearState();
		// Maybe flushing the device helps with all these messages?
		md3dImmediateContext->Flush();
	}

	ReleaseCOM(md3dImmediateContext);

	ID3D11Debug *pDebug = nullptr;
	HR(md3dDevice->QueryInterface(IID_PPV_ARGS(&pDebug)));

	if (pDebug != nullptr) {

		pDebug->ReportLiveDeviceObjects(D3D11_RLDO_DETAIL);
		pDebug->Release();
	}

	ReleaseCOM(md3dDevice);
}

bool D3DApp::Init()
{
	if (!InitMainWindow())
		return false;

	if (!InitDirect3D())
		return false;

	return true;
}

HINSTANCE D3DApp::AppInst() const {
	return mhAppInst;
}

HWND D3DApp::MainWnd() const {
	return mhMainWnd;
}

float D3DApp::AspectRatio() const {
	// Q: why not just (float)?
	return static_cast<float>(mClientWidth) / mClientHeight;
}

int D3DApp::Run() {
	MSG msg = { 0 };

	mTimer.Reset();

	while (msg.message != WM_QUIT) {
		// If there are Window messages, then process them.
		if (PeekMessage(&msg, 0, 0, 0, PM_REMOVE)) {
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		else {
			// Otherwise, perform animation/game logic.
			mTimer.Tick();
			if (!mAppPaused) {
				CalculateFrameStats();
				UpdateScene(mTimer.DeltaTime());
				DrawScene();
			}
			else {
				Sleep(100);
			}
		}

	}

	return (int)msg.wParam;
}

bool D3DApp::InitMainWindow() {
	// Erm, basic Win32 window initialization here.
	// From page 799 onwards, roughly.

	// The first task to creating a window is to describe some of its
	// characteristics by filling out a WNDCLASS structure.
	WNDCLASS wc;

	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.lpfnWndProc = MainWndProc;
	wc.cbClsExtra = 0;
	wc.cbWndExtra = 0;
	wc.hInstance = mhAppInst;
	wc.hIcon = LoadIcon(0, IDI_APPLICATION);
	wc.hCursor = LoadCursor(0, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);
	wc.lpszMenuName = 0;
	wc.lpszClassName = L"BasicWndClass";

	// Next, we register this WNDCLASS instance with Windows so
	// that we can create a window based on it.
	if (!RegisterClass(&wc)) {
		MessageBox(0, L"RegisterClass FAILED", 0, 0);
		return false;
	}

	// With our WNDCLASS instance registered, we can create a
	// window with the CreateWindow function.

	// See more complicated window dimension sizing in GitHub code.
	// Here's the more complicated window dimension sizing code:
	// Compute window rectangle dimensions based on requested client area dimensions.
	RECT R = { 0, 0, mClientWidth, mClientHeight };
	AdjustWindowRect(&R, WS_OVERLAPPEDWINDOW, false);
	int width = R.right - R.left;
	int height = R.bottom - R.top;
	// Yep - turns out this is actually required to have a final rendering size of
	// 800 x 600!

	mhMainWnd = CreateWindow(
		L"BasicWndClass", // Registered WNDCLASS instance to use.
		L"Win32Basic", // window title
		WS_OVERLAPPEDWINDOW, // style flags
		CW_USEDEFAULT, // x-coordinate
		CW_USEDEFAULT, // y-coordinate
		width, // width
		height, // height
		0, // parent window
		0, // menu handle
		mhAppInst, // app instance
		0); // extra creation parameters

	if (mhMainWnd == 0) {
		MessageBox(0, L"CreateWindow Failed!", 0, 0);
		return false;
	}

	// Show and update the window.
	ShowWindow(mhMainWnd, SW_SHOW); // Value here from GitHub; I don't know what it does.
	UpdateWindow(mhMainWnd);

	return true;
}

bool D3DApp::InitDirect3D() {
	// From Section 4.2.

	// 1. Create the ID3D11Device and ID3D11DeviceContext interfaces
	// using the D3D11CreateDevice function.

	UINT createDeviceFlags = 0;

#if defined(DEBUG) || defined(_DEBUG)
	createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif
	// We can also add the singlethreaded flag here.

	// Exercise 2: Enumerate adapters.
	// Code from https://msdn.microsoft.com/en-us/library/windows/desktop/bb174538(v=vs.85).aspx.
	/*
	{
	// Maybe not the most sustainable solution - would make sense to maintain a factory as an object.
	std::vector <IDXGIAdapter*> vAdapters;
	IDXGIFactory* pFactory;
	CreateDXGIFactory(__uuidof(IDXGIFactory), (void**)(&pFactory));
	UINT i = 0;
	IDXGIAdapter * pAdapter;
	while (pFactory->EnumAdapters(i, &pAdapter) != DXGI_ERROR_NOT_FOUND)
	{
	vAdapters.push_back(pAdapter);
	++i;
	}

	odprintf("*** NUM ADAPTERS = %i", vAdapters.size());
	// Exercise 3.
	for (int i = 0; i < vAdapters.size(); i++) {
	LARGE_INTEGER pUMDVersion;
	// Via MSDN, this no longer functions in Direct3D 11. (Instead, they recommend setting up the interface
	// and seeing if you hit an error :| )
	if (vAdapters[i]->CheckInterfaceSupport(__uuidof(ID3D10Device), &pUMDVersion) == DXGI_ERROR_UNSUPPORTED) {
	odprintf("*** D3D10 NOT SUPPORTED FOR ADAPTER %i", i);
	}
	else {
	odprintf("*** D3D10 SUPPORTED FOR ADAPTER %i", i);
	}

	// Exercise 4.
	std::vector <IDXGIOutput*> vOutputs;
	IDXGIOutput* pOutput;
	for (UINT j = 0; vAdapters[i]->EnumOutputs(j, &pOutput) != DXGI_ERROR_NOT_FOUND; j++) {
	vOutputs.push_back(pOutput);
	}

	odprintf("*** NUM OUTPUTS FOR ADAPTER %i = %i", i, vOutputs.size());
	// Clean up outputs
	for (int j = 0; j < vOutputs.size(); j++) {
	// Exercise 5: ...and get the list of display modes for adapter i, output j.

	// Note to self: MSDN now recommends using all of the DXGI 1.1 routines.
	UINT num = 0;
	vOutputs[j]->GetDisplayModeList(DXGI_FORMAT_R8G8B8A8_UNORM, 0, &num, 0);

	DXGI_MODE_DESC* pDescs = new DXGI_MODE_DESC[num];
	vOutputs[j]->GetDisplayModeList(DXGI_FORMAT_R8G8B8A8_UNORM, 0, &num, pDescs);

	// Print the list of display modes for adapter i, output j.
	for (UINT k = 0; k < num; k++) {
	odprintf("***   WIDTH = %i HEIGHT = %i REFRESH = %f",
	pDescs[k].Width,
	pDescs[k].Height,
	(float)pDescs[k].RefreshRate.Numerator/pDescs[k].RefreshRate.Denominator);
	}

	// Clean up this output and array of descriptions
	delete[] pDescs;
	ReleaseCOM(vOutputs[j]);
	}

	// Clean up this adapter
	ReleaseCOM(vAdapters[i]);
	}

	// Clean up this factory
	ReleaseCOM(pFactory);
	}
	*/ // Exercises 2-5

	//D3D11CreateDeviceAndSwapChain(adapter, driver, 0, flags, featurelevel, featurelevels, sdkversion,...)
	D3D_FEATURE_LEVEL featureLevel;
	HRESULT hr = D3D11CreateDevice(
		0, // default adapter
		D3D_DRIVER_TYPE_HARDWARE,
		0, // no software device
		createDeviceFlags,
		0, 0, // default feature level array
		D3D11_SDK_VERSION,
		&md3dDevice,
		&featureLevel,
		&md3dImmediateContext);

	if (FAILED(hr)) {
		MessageBox(0, L"D3D11Create Device Failed.", 0, 0);
		return false;
	}

#if defined(DEBUG) | defined(_DEBUG)
	ID3D11Debug *d3dDebug = nullptr;
	if (SUCCEEDED(md3dDevice->QueryInterface(__uuidof(ID3D11Debug), (void**)&d3dDebug)))
	{
		ID3D11InfoQueue *d3dInfoQueue = nullptr;
		if (SUCCEEDED(d3dDebug->QueryInterface(__uuidof(ID3D11InfoQueue), (void**)&d3dInfoQueue)))
		{
			//d3dInfoQueue->SetBreakOnSeverity(D3D11_MESSAGE_SEVERITY_WARNING, true);
			d3dInfoQueue->SetBreakOnSeverity(D3D11_MESSAGE_SEVERITY_CORRUPTION, true);
			d3dInfoQueue->SetBreakOnSeverity(D3D11_MESSAGE_SEVERITY_ERROR, true);
		}
		d3dDebug->Release();
	}
#endif

	//if (featureLevel < D3D_FEATURE_LEVEL_11_0) {
	//	MessageBox(0, L"Direct3D Feature Level was under 11.0!", 0, 0);
	//}

	// 2. Check for 4X MSAA quality level support using the
	// ID3D11Device::CheckMultisampleQualityLevels method.

	HR(md3dDevice->CheckMultisampleQualityLevels(
		DXGI_FORMAT_R8G8B8A8_UNORM, 4, &m4xMsaaQuality));
	assert(m4xMsaaQuality > 0);

	// 3. Describe the characteristics of the swap chain
	// we are going to create by filling out an instance
	// of the DXGI_SWAP_CHAIN_DESC structure.

	DXGI_SWAP_CHAIN_DESC sd;
	sd.BufferDesc.Width = mClientWidth;
	sd.BufferDesc.Height = mClientHeight;
	sd.BufferDesc.RefreshRate.Numerator = 60;
	sd.BufferDesc.RefreshRate.Denominator = 1;
	sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	sd.BufferDesc.ScanlineOrdering = DXGI_MODE_SCANLINE_ORDER_UNSPECIFIED;
	sd.BufferDesc.Scaling = DXGI_MODE_SCALING_UNSPECIFIED;

	// Use 4X MSAA?
	if (mEnable4xMsaa) {
		sd.SampleDesc.Count = 4;

		// m4xMsaaQuality is returned via CheckMultisampleQualityLevels().
		sd.SampleDesc.Quality = m4xMsaaQuality - 1;
	}
	else {
		// No MSAA
		sd.SampleDesc.Count = 1;
		sd.SampleDesc.Quality = 0;
	}

	sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	sd.BufferCount = 1;
	sd.OutputWindow = mhMainWnd;
	sd.Windowed = true;
	sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
	sd.Flags = 0;

	// 4. Query the IDXGIFactory instance that was used to create
	// the device, and create an IDXGISwapChain instance.
	IDXGIDevice* dxgiDevice = 0;
	HR(md3dDevice->QueryInterface(__uuidof(IDXGIDevice),
		(void**)&dxgiDevice));

	IDXGIAdapter* dxgiAdapter = 0;
	HR(dxgiDevice->GetParent(__uuidof(IDXGIAdapter),
		(void**)&dxgiAdapter));

	// Finally, get the IDXGIFactory interface.
	IDXGIFactory* dxgiFactory = 0;
	HR(dxgiAdapter->GetParent(__uuidof(IDXGIFactory),
		(void**)&dxgiFactory));

	// Now, create the swap chain.
	HR(dxgiFactory->CreateSwapChain(md3dDevice, &sd, &mSwapChain));

	// Exercise 1: Disable the ALT-ENTER functionality to enter full screen.
	// dxgiFactory->MakeWindowAssociation(mhMainWnd, DXGI_MWA_NO_WINDOW_CHANGES);
	// End Exercise 1

	// Release our acquired COM interfaces (because we are done with them).
	ReleaseCOM(dxgiDevice);
	ReleaseCOM(dxgiAdapter);
	ReleaseCOM(dxgiFactory);

	// 5. Create a render target view to the swap chain's back buffer.
	// The code from here on is all contained in D3DApp::OnResize(), so we just call that.
	// But there's a catch: if we're actually in a class that inherits from D3DApp, then this
	// function will actually call the inheriting class's OnResize.
	// We want this to occur, because the inheriting class's OnResize will usually do things like
	// initializing its perspective matrix (or simply matching its perspective matrix's aspect ratio to
	// the window's aspect ratio.)
	// If we had instead called D3DApp::OnResize() here, then the most likely scenario is that
	// nothing would appear in the window (since our perspective matrix would simply be the identity)
	// until we moved or resized the window!
	// (This was a problem I had for a while.)
	OnResize();

	// In the GitHub code, they just call OnResize. So maybe we should do that instead as well. TODO(neil).
	return true;
}

void D3DApp::OnResize() {
	// From GitHub.
	assert(md3dImmediateContext);
	assert(md3dDevice);
	assert(mSwapChain);

	// Release the old views, as they hold references to the buffers we
	// will be destroying. Also release the old depth/stencil buffer.

	ReleaseCOM(mRenderTargetView);
	ReleaseCOM(mDepthStencilView);
	ReleaseCOM(mDepthStencilBuffer);

	// Resize the swap chain and create the render target view.

	HR(mSwapChain->ResizeBuffers(1,
		mClientWidth,
		mClientHeight,
		DXGI_FORMAT_R8G8B8A8_UNORM,
		0));

	ID3D11Texture2D* backBuffer;
	HR(mSwapChain->GetBuffer(0, __uuidof(ID3D11Texture2D),
		reinterpret_cast<void**>(&backBuffer)));
	HR(md3dDevice->CreateRenderTargetView(backBuffer, 0,
		&mRenderTargetView));
	ReleaseCOM(backBuffer);

	// Create the depth/stencil buffer and view.
	// Copied from above.

	D3D11_TEXTURE2D_DESC depthStencilDesc;
	depthStencilDesc.Width = mClientWidth;
	depthStencilDesc.Height = mClientHeight;
	depthStencilDesc.MipLevels = 1;
	depthStencilDesc.ArraySize = 1;
	depthStencilDesc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;

	// Use 4X MSAA? -- must match swap chain MSAA values.
	if (mEnable4xMsaa) {
		depthStencilDesc.SampleDesc.Count = 4;
		depthStencilDesc.SampleDesc.Quality = m4xMsaaQuality - 1;
	}
	else {
		// No MSAA
		depthStencilDesc.SampleDesc.Count = 1;
		depthStencilDesc.SampleDesc.Quality = 0;
	}

	depthStencilDesc.Usage = D3D11_USAGE_DEFAULT;
	depthStencilDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL;
	depthStencilDesc.CPUAccessFlags = 0;
	depthStencilDesc.MiscFlags = 0;

	HR(md3dDevice->CreateTexture2D(
		&depthStencilDesc,      // Description of texture to create.
		0,
		&mDepthStencilBuffer)); // Return pointer to depth/stencil buffer.

	HR(md3dDevice->CreateDepthStencilView(
		mDepthStencilBuffer, // Resource we want to create a view to.
		0,
		&mDepthStencilView)); // Return depth/stencil view.

							  // 7. Bind the render target view and depth/stencil view to the output
							  // merger stage of the rendering pipeline so that they can be used
							  // by Direct3D.

	md3dImmediateContext->OMSetRenderTargets(
		1, &mRenderTargetView, mDepthStencilView);

	// 8. Set the viewport.
	mScreenViewport.TopLeftX = 0.0f;
	mScreenViewport.TopLeftY = 0.0f;
	mScreenViewport.Width = static_cast<float>(mClientWidth);
	mScreenViewport.Height = static_cast<float>(mClientHeight);
	mScreenViewport.MinDepth = 0.0f;
	mScreenViewport.MaxDepth = 1.0f;

	/*mScreenViewport.TopLeftX = 100.0f;
	mScreenViewport.TopLeftY = 150.0f;
	mScreenViewport.Width = 500.0f;
	mScreenViewport.Height = 400.0f;
	mScreenViewport.MinDepth = 0.0f;
	mScreenViewport.MaxDepth = 1.0f;*/

	md3dImmediateContext->RSSetViewports(1, &mScreenViewport);
}

void D3DApp::CalculateFrameStats() {
	// Code computes the average frames per second, and also the
	// average time it takes to render one frame. These stats
	// are appended to the window caption bar.

	static int frameCount = 0;
	static float timeElapsed = 0.0f;

	frameCount++; // hooray

				  // Compute averages over a one-second period.
				  // TODO (neil): Can we make this smoother
				  // and update more frequently?
	if ((mTimer.TotalTime() - timeElapsed) >= 1.0f) {
		float fps = (float)frameCount; // /1 s
		float mspf = 1000.0f / fps;

		std::wostringstream outs;
		outs.precision(6);
		outs << mMainWndCaption << L"    "
			<< L"FPS: " << fps << L"    "
			<< L"Frame Time: " << mspf << L" (ms)";
		SetWindowText(mhMainWnd, outs.str().c_str());

		// Reset for next average.
		frameCount = 0;
		timeElapsed += 1.0f;
	}
}

LRESULT D3DApp::MsgProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
	switch (msg) {
	case WM_ACTIVATE:
		if (LOWORD(wParam) == WA_INACTIVE) {
			mAppPaused = true;
			mTimer.Stop();
			// odprintf("WM_ACTIVATE WA_INACTIVE");
		}
		else {
			mAppPaused = false;
			mTimer.Start();
			// odprintf("WM_ACTIVATE not inactive");
		}
		return 0;

		// WM_SIZE is sent when the user is resizing the window.
	case WM_SIZE:
		// (From GitHub.)
		// Save the new client area dimensions.
		mClientWidth = LOWORD(lParam);
		mClientHeight = HIWORD(lParam);
		if (md3dDevice) {
			if (wParam == SIZE_MINIMIZED) {
				mAppPaused = true;
				mMinimized = true;
				mMaximized = false;
				// odprintf("SIZE_MINIMIZED: %i %i", mClientWidth, mClientHeight);
			}
			else if (wParam == SIZE_MAXIMIZED) {
				mAppPaused = false;
				mMinimized = false;
				mMaximized = true;
				// odprintf("SIZE_MAXIMIZED: %i %i", mClientWidth, mClientHeight);
				OnResize();
			}
			else if (wParam == SIZE_RESTORED) {
				// What are we restoring from?
				if (mMinimized) {
					mAppPaused = false;
					mMinimized = false;
					// odprintf("SIZE_RESTORED (from Minimized): %i %i", mClientWidth, mClientHeight);
					OnResize();
				}
				else if (mMaximized) {
					mAppPaused = false;
					mMaximized = false;
					// odprintf("SIZE_RESTORED (from Maximized): %i %i", mClientWidth, mClientHeight);
					OnResize();
				}
				else if (mResizing) {
					// wait until we finish resizing.
					// odprintf("SIZE_RESTORED (still resizing): %i %i", mClientWidth, mClientHeight);
				}
				else {
					// Other API call, e.g.
					// SetWindowPos or SetFullscreenState.
					// odprintf("SIZE_RESTORED (other): %i %i", mClientWidth, mClientHeight);
					OnResize();
				}
			}
			else {
				// odprintf("wParam was %i: %i %i", wParam, mClientWidth, mClientHeight);
			}
		}
		else {
			// odprintf("Device not initialized yet, wParam was %i: %i %i", wParam, mClientWidth, mClientHeight);
		}
		return 0;

		// WM_ENTERSIZEMOVE is sent when the user grabs the resize bars.
	case WM_ENTERSIZEMOVE:
		mAppPaused = true;
		mResizing = true;
		mTimer.Stop();
		// odprintf("WM_ENTERSIZEMOVE: %i %i", mClientWidth, mClientHeight);
		return 0;

		// WM_EXITSIZEMOVE is sent when the user releases the resize bars.
		// Here we reset everything based on the new window dimensions.
	case WM_EXITSIZEMOVE:
		mAppPaused = false;
		mResizing = false;
		mTimer.Start();
		// odprintf("WM_EXITSIZEMOVE: %i %i", mClientWidth, mClientHeight);
		OnResize();
		return 0;

		// WM_DESTROY is sent when the window is being destroyed.
	case WM_DESTROY:
		// odprintf("WM_DESTROY: %i %i", mClientWidth, mClientHeight);
		PostQuitMessage(0);
		return 0;

		// The WM_MENUCHAR message is sent when a menu is active and the user presses
		// a key that does not correspond to any mnemonic or accelerator key.
	case WM_MENUCHAR:
		// Don't beep when we alt-enter. (Q, neil: does it usually beep?)
		// odprintf("WM_MENUCHAR: %i %i", mClientWidth, mClientHeight);
		return MAKELRESULT(0, MNC_CLOSE);

		// Catch this message to prevent the window from becoming too small.
	case WM_GETMINMAXINFO:
		((MINMAXINFO*)lParam)->ptMinTrackSize.x = 200;
		((MINMAXINFO*)lParam)->ptMinTrackSize.y = 200;
		// odprintf("WM_GETMINMAXINFO: %i %i", mClientWidth, mClientHeight);
		return 0;

	case WM_LBUTTONDOWN:
	case WM_MBUTTONDOWN:
	case WM_RBUTTONDOWN:
		OnMouseDown(wParam, GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
		return 0;

	case WM_LBUTTONUP:
	case WM_MBUTTONUP:
	case WM_RBUTTONUP:
		OnMouseUp(wParam, GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
		return 0;

	case WM_MOUSEMOVE:
		OnMouseMove(wParam, GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
		return 0;

	case WM_CHAR:
		OnCharacterKey((char)wParam);
		return 0;
		// default:
		// odprintf("Unclear message: %i", msg);


	}

	return DefWindowProc(hwnd, msg, wParam, lParam);
}