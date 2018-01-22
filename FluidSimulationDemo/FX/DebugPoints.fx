cbuffer cbPerFrame
{
	float4 gScreenSize;
};

cbuffer cbPerObject
{
	float4x4 gWorldViewProj;
};

struct VertexIn
{
	float3 PosL  : POSITION;
};

struct VertexOut
{
	float4 PosH  : SV_POSITION;
};

//struct GeoOut
//{
//	float4 PosH   : SV_POSITION;
//	uint   PrimID : SV_PrimitiveID;
//};

VertexOut VS(VertexIn vin)
{
	VertexOut vout;

	// Transform to homogeneous clip space.
	vout.PosH = mul(float4(vin.PosL, 1.0f), gWorldViewProj);

	return vout;
}

// Expand each point into a 2px * 2px quad (4 vertices)
//[maxvertexcount(4)]
//void GS(point VertexOut gin[1],
//	uint primID : SV_PrimitiveID,
//	inout TriangleStream<GeoOut> triStream) // Outputs vertices as a triangle strip
//{
	// Position in homogeneous clip space
//	float4 pos = gin[0].PosH;

//}

float4 PS(VertexOut pin) : SV_Target
{
	// Just return a constant color
	return float4(1.0f, 0.0f, 0.0f, 1.0f);
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