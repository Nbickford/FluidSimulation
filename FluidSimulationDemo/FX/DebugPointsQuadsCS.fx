cbuffer cbPerObject
{
	float4x4 gWorld;
	float4x4 gViewProj;
	float gPtSize;
};

struct VertexOut
{
	float4 PosH  : SV_POSITION;
};

struct GeoOut
{
	float4 PosH    : SV_POSITION;
	float3 NormalW : NORMAL;
	uint   PrimID  : SV_PrimitiveID;
};

struct Particle {
	float3 Pos;
	float3 Vel;
};

StructuredBuffer<Particle> gParticles;

VertexOut VS(uint vid : SV_VertexID)
{
	VertexOut vout;

	// the 16 in here should be the texture size (although this is just for debug purposes)
	float3 pos = 2.0f*(gParticles[vid].Pos + 0.5f / 16.0f) - 1.0f;

	// Transform to world space
	vout.PosH = mul(float4(pos, 1.0f), gWorld);

	return vout;
}

// Expand each point into a 2px * 2px quad (4 vertices)
[maxvertexcount(4)]
void GS(point VertexOut gin[1],
	uint primID : SV_PrimitiveID,
	inout TriangleStream<GeoOut> triStream) // Outputs vertices as a triangle strip
{
	// Position in world space
	float4 pos = gin[0].PosH;
	// Just add a small size to the x and y coordinates

	float hSize = gPtSize*0.5f;

	float4 v[4];
	v[0] = float4(pos.x - hSize, pos.y + hSize, pos.z, 1.0f);
	v[1] = float4(pos.x + hSize, pos.y + hSize, pos.z, 1.0f);
	v[2] = float4(pos.x - hSize, pos.y - hSize, pos.z, 1.0f);
	v[3] = float4(pos.x + hSize, pos.y - hSize, pos.z, 1.0f);

	GeoOut gout;
	[unroll]
	for (int i = 0; i < 4; ++i) {
		// Transform to homogeneous clip space
		gout.PosH    = mul(v[i], gViewProj);
		gout.NormalW = float3(0.0f, 0.0f, -1.0f);
		gout.PrimID  = primID;

		triStream.Append(gout);
	}
}

float4 PS(GeoOut pin) : SV_Target
{
	// Just return a constant color
	return float4(1.0f, 0.0f, 0.0f, 1.0f);
}

technique11 ColorTech
{
	pass P0
	{
		SetVertexShader(CompileShader(vs_5_0, VS()));
		SetGeometryShader(CompileShader(gs_5_0, GS()));
		SetPixelShader(CompileShader(ps_5_0, PS()));
	}
}