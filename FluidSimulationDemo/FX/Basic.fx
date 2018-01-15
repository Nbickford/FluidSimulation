//***************************************************************************************
// color.fx by Frank Luna (C) 2011 All Rights Reserved.
//
// Transforms and colors geometry.
//***************************************************************************************

cbuffer cbPerObject
{
	float4x4 gWorldViewProj;
};

// Non-numeric values cannot be added to a cbuffer.
Texture2D gDiffuseMap;

SamplerState samMipMap {
	Filter = MIN_LINEAR_MAG_POINT_MIP_LINEAR;
	AddressU = CLAMP;
	AddressV = CLAMP;
};

struct VertexIn
{
	float3 PosL  : POSITION;
	float2 TexCoord : TEXCOORD0;
};

struct VertexOut
{
	float4 PosH  : SV_POSITION;
	float2 TexCoord : TEXCOORD0;
};

VertexOut VS(VertexIn vin)
{
	VertexOut vout;

	// Transform to homogeneous clip space.
	vout.PosH = mul(float4(vin.PosL, 1.0f), gWorldViewProj);

	// Just pass texture coordinate to the pixel shader.
	vout.TexCoord = vin.TexCoord;

	return vout;
}

float4 PS(VertexOut pin) : SV_Target
{
	// Uncomment this line to use iq's nicer bilinear filtering, from
	// http://www.iquilezles.org/www/articles/texture/texture.htm
	// and https://www.shadertoy.com/view/XsfGDn!
	//float2 texSize;
	//gDiffuseMap.GetDimensions(texSize.x, texSize.y);
	//float2 uv = pin.TexCoord*texSize + 0.5f;
	//float2 iuv = floor(uv);
	//float2 fuv = frac(uv);
	//uv = iuv + fuv*fuv*(3.0 - 2.0*fuv);
	//pin.TexCoord = (uv - 0.5f) / texSize;

	float4 rgba = gDiffuseMap.Sample(samMipMap, pin.TexCoord);
	// Since the texture's color is stored in linear space, we need to convert it to SRGB for display.
	// (note: we don't handle alpha in a consistent way here, since we assume the texture
	// is always opaque - if alpha meant coverage, then we would only want to apply
    // gamma correction to the first three components)
	//rgba = pow(rgba, float4(1.0/2.2f, 1.0/2.2f, 1.0/2.2f, 1.0/2.2f));
	return rgba;
}

technique11 ColorTech
{
	pass P0
	{
		SetVertexShader(CompileShader(vs_5_0, VS()));
		SetGeometryShader(NULL);
		SetPixelShader(CompileShader(ps_5_0, PS()));
	}
}