//***************************************************************************************
// d3dUtil.cpp by Frank Luna (C) 2011 All Rights Reserved.
//***************************************************************************************

#include "d3dUtil.h"
#include "../Libraries/DirectXTK/DDSTextureLoader.h"

// System libraries for determining file modified dates
#include <sys/types.h>
#include <sys/stat.h>

ID3D11ShaderResourceView* d3dHelper::CreateTexture2DArraySRV(
	ID3D11Device* device, ID3D11DeviceContext* context,
	std::vector<std::wstring>& filenames)
{
	//
	// Load the texture elements individually from file.  These textures
	// won't be used by the GPU (0 bind flags), they are just used to 
	// load the image data from file.  We use the STAGING usage so the
	// CPU can read the resource.
	//

	UINT size = static_cast<UINT>(filenames.size());

	std::vector<ID3D11Texture2D*> srcTex(size);
	for (UINT i = 0; i < size; ++i)
	{
		CreateDDSTextureFromFileEx(device, filenames[i].c_str(),
			0, // max size
			D3D11_USAGE_STAGING, // only allow copy from gpu to cpu
			0, // bind flags
			D3D11_CPU_ACCESS_WRITE | D3D11_CPU_ACCESS_READ,
			0,
			false, //force SRGB
			(ID3D11Resource**)&srcTex[i], 0);
	}

	//
	// Create the texture array.  Each element in the texture 
	// array has the same format/dimensions.
	//

	D3D11_TEXTURE2D_DESC texElementDesc;
	srcTex[0]->GetDesc(&texElementDesc);

	D3D11_TEXTURE2D_DESC texArrayDesc;
	texArrayDesc.Width = texElementDesc.Width;
	texArrayDesc.Height = texElementDesc.Height;
	texArrayDesc.MipLevels = texElementDesc.MipLevels;
	texArrayDesc.ArraySize = size;
	texArrayDesc.Format = texElementDesc.Format;
	texArrayDesc.SampleDesc.Count = 1;
	texArrayDesc.SampleDesc.Quality = 0;
	texArrayDesc.Usage = D3D11_USAGE_DEFAULT; // gpu rw
	texArrayDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
	texArrayDesc.CPUAccessFlags = 0;
	texArrayDesc.MiscFlags = 0;

	ID3D11Texture2D* texArray = 0;
	HR(device->CreateTexture2D(&texArrayDesc, 0, &texArray));

	//
	// Copy individual texture elements into texture array.
	//

	// for each texture element...
	for (UINT texElement = 0; texElement < size; ++texElement)
	{
		// for each mipmap level...
		for (UINT mipLevel = 0; mipLevel < texElementDesc.MipLevels; ++mipLevel)
		{
			D3D11_MAPPED_SUBRESOURCE mappedTex2D;
			// get the mipLevel'th level of srcTex[texElement]
			HR(context->Map(srcTex[texElement], mipLevel, D3D11_MAP_READ, 0, &mappedTex2D));

			// We can set the box to something nonzero here to update only a portion
			// of the subresource - nice!
			// copy the mapped subresource to texArray's appropriate subresource
			context->UpdateSubresource(texArray,
				D3D11CalcSubresource(mipLevel, texElement, texElementDesc.MipLevels),
				0, mappedTex2D.pData, mappedTex2D.RowPitch, mappedTex2D.DepthPitch);

			// put the mipLevel'th level of srcTex[texElement] back
			context->Unmap(srcTex[texElement], mipLevel);
		}
	}

	// Interestingly, we could have done this at texture creation time (which we would
	// have needed to do if we were creating an IMMUTABLE resource instead of a DEFAULT
	// one) by initializing the array of D3D11_SUBRESOURCE_DATA objects and giving it
	// to the CreateTexture2D call.

	//
	// Create a resource view to the texture array.
	//

	D3D11_SHADER_RESOURCE_VIEW_DESC viewDesc;
	viewDesc.Format = texArrayDesc.Format;
	viewDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2DARRAY;
	viewDesc.Texture2DArray.MostDetailedMip = 0;
	viewDesc.Texture2DArray.MipLevels = texArrayDesc.MipLevels;
	viewDesc.Texture2DArray.FirstArraySlice = 0;
	viewDesc.Texture2DArray.ArraySize = size;

	ID3D11ShaderResourceView* texArraySRV = 0;
	HR(device->CreateShaderResourceView(texArray, &viewDesc, &texArraySRV));

	//
	// Cleanup--we only need the resource view.
	//

	ReleaseCOM(texArray);

	for (UINT i = 0; i < size; ++i)
		ReleaseCOM(srcTex[i]);

	return texArraySRV;
}

/*ID3D11ShaderResourceView* d3dHelper::CreateRandomTexture1DSRV(ID3D11Device* device)
{
//
// Create the random data.
//
XMFLOAT4 randomValues[1024];

for (int i = 0; i < 1024; ++i)
{
randomValues[i].x = MathHelper::RandF(-1.0f, 1.0f);
randomValues[i].y = MathHelper::RandF(-1.0f, 1.0f);
randomValues[i].z = MathHelper::RandF(-1.0f, 1.0f);
randomValues[i].w = MathHelper::RandF(-1.0f, 1.0f);
}

D3D11_SUBRESOURCE_DATA initData;
initData.pSysMem = randomValues;
initData.SysMemPitch = 1024 * sizeof(XMFLOAT4);
initData.SysMemSlicePitch = 0;

//
// Create the texture.
//
D3D11_TEXTURE1D_DESC texDesc;
texDesc.Width = 1024;
texDesc.MipLevels = 1;
texDesc.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
texDesc.Usage = D3D11_USAGE_IMMUTABLE;
texDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
texDesc.CPUAccessFlags = 0;
texDesc.MiscFlags = 0;
texDesc.ArraySize = 1;

ID3D11Texture1D* randomTex = 0;
HR(device->CreateTexture1D(&texDesc, &initData, &randomTex));

//
// Create the resource view.
//
D3D11_SHADER_RESOURCE_VIEW_DESC viewDesc;
viewDesc.Format = texDesc.Format;
viewDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE1D;
viewDesc.Texture1D.MipLevels = texDesc.MipLevels;
viewDesc.Texture1D.MostDetailedMip = 0;

ID3D11ShaderResourceView* randomTexSRV = 0;
HR(device->CreateShaderResourceView(randomTex, &viewDesc, &randomTexSRV));

ReleaseCOM(randomTex);

return randomTexSRV;
}*/

void ExtractFrustumPlanes(XMFLOAT4 planes[6], CXMMATRIX M)
{
	XMFLOAT4X4 P;
	XMStoreFloat4x4(&P, M);

	//
	// Left
	//
	planes[0].x = P(0, 3) + P(0, 0);
	planes[0].y = P(1, 3) + P(1, 0);
	planes[0].z = P(2, 3) + P(2, 0);
	planes[0].w = P(3, 3) + P(3, 0);

	//
	// Right
	//
	planes[1].x = P(0, 3) - P(0, 0);
	planes[1].y = P(1, 3) - P(1, 0);
	planes[1].z = P(2, 3) - P(2, 0);
	planes[1].w = P(3, 3) - P(3, 0);

	//
	// Bottom
	//
	planes[2].x = P(0, 3) + P(0, 1);
	planes[2].y = P(1, 3) + P(1, 1);
	planes[2].z = P(2, 3) + P(2, 1);
	planes[2].w = P(3, 3) + P(3, 1);

	//
	// Top
	//
	planes[3].x = P(0, 3) - P(0, 1);
	planes[3].y = P(1, 3) - P(1, 1);
	planes[3].z = P(2, 3) - P(2, 1);
	planes[3].w = P(3, 3) - P(3, 1);

	//
	// Near
	//
	planes[4].x = P(0, 2);
	planes[4].y = P(1, 2);
	planes[4].z = P(2, 2);
	planes[4].w = P(3, 2);

	//
	// Far
	//
	planes[5].x = P(0, 3) - P(0, 2);
	planes[5].y = P(1, 3) - P(1, 2);
	planes[5].z = P(2, 3) - P(2, 2);
	planes[5].w = P(3, 3) - P(3, 2);

	// Normalize the plane equations.
	for (int i = 0; i < 6; ++i)
	{
		XMVECTOR v = XMPlaneNormalize(XMLoadFloat4(&planes[i]));
		XMStoreFloat4(&planes[i], v);
	}
}

// NB utils
// Note: With regards to Visual Studio also compiling files,
// this function can be potentially misleading!
ID3DBlob* d3dUtil::CompileShader(const std::wstring& filename,
	const D3D_SHADER_MACRO* pDefines,
	const std::string& entryPoint,
	const std::string& target) {
	DWORD shaderFlags = 0;
#if defined(DEBUG) || defined(_DEBUG)
	shaderFlags |= D3D10_SHADER_DEBUG;
	shaderFlags |= D3D10_SHADER_SKIP_OPTIMIZATION;
#endif

	ID3DBlob* compiledShader = 0;
	ID3DBlob* compilationMsgs = 0;

	// If a .cso version of the file already exists, use that
	// instead of recompiling the file:
	// (Note that we don't write out our own .cso versions of files here)
	size_t lastIndex = filename.find_last_of(L".");
	assert(lastIndex != std::string::npos); // Assert that our filenames must have extensions
	std::wstring rawName = filename.substr(0, lastIndex);
	std::wstring csoName = rawName + L".cso";

	// See https://docs.microsoft.com/en-us/previous-versions/visualstudio/visual-studio-6.0/aa273365(v=vs.60)
	// for more information on _stat and _wstat
	bool useCompiledVersion = false;
	struct _stat result;
	if (_wstat(filename.c_str(), &result) == 0) {
		time_t srcModTime = result.st_atime;

		struct _stat result2;
		if (_wstat(csoName.c_str(), &result2) == 0) {
			time_t cmpModTime = result2.st_atime;

			if (cmpModTime >= srcModTime) {
				useCompiledVersion = true;
			}
		}
	}

	if (useCompiledVersion) {
		// Use the compiled version
		// See https://stackoverflow.com/questions/5020204/loading-a-precompiled-hlsl-shader-into-memory-for-use-with-createpixelshader?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
		D3DReadFileToBlob(csoName.c_str(), &compiledShader);

		return compiledShader;
	}
	else {
		// Compile it ourselves
		// For the standard file include: Three cheers for
		// https://github.com/Microsoft/FX11/wiki/D3DX11CompileEffectFromMemory

		HRESULT shr = D3DCompileFromFile(filename.c_str(),
			pDefines,
			D3D_COMPILE_STANDARD_FILE_INCLUDE,
			entryPoint.c_str(), target.c_str(), shaderFlags, 0, &compiledShader, &compilationMsgs);

		// compilationMsgs can store errors or warnings.
		if (compilationMsgs != 0) {

			// Suppress warning X4717
			std::stringstream f((char*)compilationMsgs->GetBufferPointer());
			std::string line;
			std::stringstream outss;
			std::string token = "warning X4717";
			bool warningX4717 = false; // since we append the extra warning to the end
			bool otherErrors = false;
			while (std::getline(f, line)) {
				if (line.compare(0, token.length(), token) == 0) {
					warningX4717 = true;
				}
				else {
					outss << line << "\n";
					otherErrors = true;
				}
			}

			if (warningX4717) {
				outss << "(effects 11 deprecation warning suppressed)\n";
			}

			if (otherErrors) {
				MessageBoxA(0, outss.str().c_str(), "Message from D3D compiler", 0);
			}
			ReleaseCOM(compilationMsgs); // otherwise, compilationMsgs is null, so we don't need to release it.
		}

		// Even if there are no compilationMsgs, check to make sure there
		// were no other errors.
		if (FAILED(shr)) {
			HR(shr);
		}

		return compiledShader;
	}
	
}