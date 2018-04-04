// Render.fx
//   Given a level set, array-of-particles data structure, and other information (e.g. camera parameters),
// this shader basically just renders the scene!
// We're taking a Shadertoy-like approach to this, in that the vertex shader draws a full-screen triangle,
// while the pixel shader does all of the actual raymarching and computation.

#include "gpParticleStruct.hlsli"

// CONSTANTS
// View-/like/ matrix
// Actual layout:
// [ -u- 0]  - right
// [ -v- 0]  - up
// [ -w- 0]  - fwds
// [ -Q- 0]  - camera position
cbuffer cbConstants {
	float4x4 mView;
};

// PHI
Texture3D<float> gPhi;
Texture3D<uint> gOffsets;
StructuredBuffer<Particle> gParticles;

// Volume parameters
// r^ (must be equal to m_pRadius in Simulation.h)
static const float mRHatCells = 1.0f;
static float mRHat;
// Drawn particle radius (must be strictly less than mRHatCells!)
static const float mRCells = 0.5f;
static float mR;
// Search radius (usually 2x average particle spacing...or 1 cell)
static const float mSearchCells = 1.0f;
static float mSearch;
// Grid dimensions and inverse grid dimensions
static float3 mM;
static float3 invmM;

// Effects framework linear sampler state
sampler sam = sampler_state {
	Texture = gPhi;
	FILTER = MIN_MAG_LINEAR_MIP_POINT;
	AddressU = Clamp;
	AddressV = Clamp;
	AddressW = Clamp;
};

struct VertexOut
{
	float4 PosH  : SV_POSITION;
	float2 fragCoord : TEXCOORD0;
};

VertexOut VS(uint vid : SV_VertexID)
{
	VertexOut vout;

	// Buffer-less full-screen triangle trick from Timothy
	// Lottes' FXAA Whitepaper: http://developer.download.nvidia.com/assets/gamedev/files/sdk/11/FXAA_WhitePaper.pdf
	vout.fragCoord = float2((vid << 1) & 2, vid & 2);
	vout.PosH = float4(vout.fragCoord*float2(2.0f, -2.0f) + float2(-1.0f, 1.0f), 0.0f, 1.0f);

	return vout;
}

static const float w = 0.1;
static const float largeNum = 100000.0f;

// Accessing particle lists
int3 previousCell(int3 p) {
	int3 ret = p - int3(1, 0, 0);
	if (ret.x < 0) {
		ret = ret + int3(mM.x, -1, 0);
		if (ret.y < 0) {
			ret = ret + int3(0, mM.y, -1);
		}
	}
	return ret;
}

uint2 getOffsetStartEnd(int3 p) {
	int3 pc = previousCell(p);
	return uint2(gOffsets[pc], gOffsets[p]);
}

// Returns the distance along the given ray to the floor, or largeNum if there was no hit.
float traceFloor(float3 co, float3 ci) {
	// (co+ci*t).y = -0.5-w
	// => t = (-0.5-w-co.y)/ci.y
	float t = (-0.5 - w - co.y) / ci.y;
	if (t < 0.0) {
		return largeNum;
	}
	else {
		return t;
	}
}

// Returns the distance along the given ray to the first and second intersections of the AABB
// or largeNum if there was no hit.
float2 traceAABB(float3 co, float3 ci, float3 bmin, float3 bmax) {
	// Based off of the code from https://tavianator.com/fast-branchless-raybounding-box-intersections/.
	// and Real-Time Rendering (3rd ed.), pg. 743.
	float3 ciInv = 1.0f / ci;
	float3 t1 = (bmin - co)*ciInv;
	float3 t2 = (bmax - co)*ciInv;

	float3 tmin = min(t1, t2);
	float3 tmax = max(t1, t2);

	float tm = max(max(tmin.x, tmin.y), tmin.z);
	float tM = min(min(tmax.x, tmax.y), tmax.z);
	if (tM >= tm) {
		return float2(tm, tM);
	}
	else {
		return float2(largeNum, largeNum);
	}
}

float kernel(float3 x, float3 xi) {
	float d2 = dot(x - xi, x - xi) / (mSearch*mSearch);
	return max(0.0f, 1 - d2 * (3 - d2 * (3 - d2)));
	//float z = (1.0f - dot(x - xi, x - xi)/(mSearch*mSearch));
	//return max(0.0f, z*z*z);
}

float mapFine(float3 p) {
	// For the particle's cell and each of its 26 neighbors...(!)
	float3 accXm = float3(0.0f, 0.0f, 0.0f);
	float accSum = 0.0f;
	int3 pCell = int3(floor(p*mM));
	for (int z = -1; z <= 1; z++) {
		for (int y = -1; y <= 1; y++) {
			for (int x = -1; x <= 1; x++) {
				int3 dpCell = pCell + int3(x, y, z);
				/*if (dpCell.x<0 || dpCell.y<0 || dpCell.z<0 || dpCell.x + 1>mM.x || dpCell.y + 1>mM.y || dpCell.z + 1>mM.z) {
					continue;
				}*/
				uint2 os = getOffsetStartEnd(dpCell);
				for (uint i = os.x; i < os.y; i++) {
					Particle particle = gParticles[i];
					float3 rpos = particle.pos + invmM * 0.5f; // move particle to [0,1]^3
					float k = kernel(p, rpos);
					accXm += k * rpos;
					accSum += k;
				}
			}
		}
	}

	if (accSum == 0.0f) {
		return mSearch;
	}
	else {
		accXm /= accSum;
		return distance(p, accXm) - mR;
	}
}

float mapCoarse(float3 p) {
	float dt = distance(p, invmM*(floor(mM*p) + float3(0.5f, 0.5f, 0.5f)));
	return (gPhi.SampleLevel(sam, p, 0))*invmM.x + mRHat - dt;
}

// Traces through the water surface
float4 traceWater(float3 co, float3 ci, float maxT) {
	// Shift for convenience
	float3 po = co + float3(0.5f, 0.5f, 0.5f);
	// Our ray through the fluid volume is now po+t*ci

	float t = 0.0f;
	float dt;
	// For now, just look up the value of Phi at co.
	// Initial coarse step
	float3 p;
	for (int i = 0; i < 32; i++) {
		// Distance to closest cell center ("|cs-x|")
		p = po + t * ci;
		
		// Lower bound on distance to surface (debug version - original still works just fine)
		//float dt = (gPhi.SampleLevel(sam, p, 0)-sqrt(3)/2)*invmM.x + mRHat;
		dt = mapCoarse(p);

		if (dt > mSearch) {
			// Continue coarse step
			dt -= mR;
			t += dt;
		}
		else {
			// Fine step
			break;
		}

		if (t > maxT || i==31) {
			return float4(10.0f, 10.0f, 10.0f, 10.0f);
		}
	}

	//float v = mapFine(p);
	//return float3(v, 10.0f*v, 100.0f*v);

	// Fine step
	for (int j = 0; j < 16; j++) {
		p = po + t * ci;
		dt = mapCoarse(p);
		if (dt > mSearch) {
			// Coarse step
			dt -= mR;
		}
		else {
			dt = mapFine(p);
		}
		t += dt;

		if (dt < 0.01) {
			break;
		}
		if (t > maxT) {
			return float4(10.0f, 10.0f, 10.0f, 10.0f);
		}
	}

	return float4(po + t * ci, dt);
}

float map(float3 p) {
	return gPhi.SampleLevel(sam, p, 0)*invmM.x;
}

/*float4 traceWater(float3 co, float3 ci, float maxT) {
	// Shift for convenience
	float3 po = co + float3(0.5f, 0.5f, 0.5f);
	// oUr vAlUe oF mPhI iS oUr lEvEl sEt
	float t = 0.0f;
	float3 p;
	float dt;
	for (int i = 0; i < 64; i++) {
		p = po + t * ci;
		dt = map(p);
		t += dt;

		if (dt < 0.001f || t>maxT) {
			break;
		}
	}

	return float4(po + t * ci, dt);
}*/

float3 computeGradient(float4 pdt) {
	float dt = pdt.w;
	float3 p = pdt.xyz;
	float3 e = float3(0.001f, 0.0f, 0.0f);
	return float3(map(p + e),
		map(p + e.yxy),
		map(p + e.yyx)) - float3(dt, dt, dt);
}

float3 computeGradient2(float4 pdt) {
	float3 p = pdt.xyz;
	float center = mapFine(p);
	float3 e = float3(0.001f, 0.0f, 0.0f);
	float3 res = float3(mapFine(p + e),
		mapFine(p + e.yxy),
		mapFine(p + e.yyx)) - float3(center, center, center);
	return res;
}

// Pixel shader.
float4 PS(VertexOut pin) : SV_Target
{
	// Transform fragCoord to UV
	float2 uv = float2(-1.0f, 1.0f) + float2(2.0f, -2.0f)*pin.fragCoord;
	// For the moment, we'll just pretend our view coordinates are
	// u (right) = {1,0,0}, v (up) = {0,0.75,0}, w (fwds) = {0,0,1}, and Q (camera position) = { 0,0,-3}:
	float3 co = mul(float4(0.0f, 0.0f, 0.0f, 1.0f), mView).xyz;
	float3 ci = mul(float4(uv.x, uv.y, 1.0f, 0.0f), mView).xyz;
	ci = normalize(ci);

	// Compute parameters for water
	// Inverse grid size
	float sizeX, sizeY, sizeZ;
	gPhi.GetDimensions(sizeX, sizeY, sizeZ);
	mM = float3(sizeX, sizeY, sizeZ);
	invmM = 1.0f / mM;
	// We only support cubic textures at the moment :(
	mRHat = mRHatCells * invmM.x;
	mR = mRCells * invmM.x;
	mSearch = mSearchCells * invmM.x;

	// OK.
	// Our scene model is:
	// - Water located in [-0.5,0.5]^3.
	// - Glass of some width w outside the box of water,
	// on all sides except the top.
	// - A matte white floor beneath the glass box, at y=-0.5-w.
	// Lighting model: just tracing rays into the sky (since our floor is the only non-Dirac BRDF):
	// - A circular spotlight of some sort - we can determine the angle of a ray from this light and work from there.
	// Raytracing paths:
	// Geometry: camera -> [glass -> (some number of internal bounces?) +] -> water -> (reflection from water->floor or sky) + (water transmission ->floor or sky)
	// (If we go from the glass immediately into the water, that's the same as just starting with the water transmission step)
	// Or        camera -> floor -> shade floor (caustics)

	// Get intersection with plane
	float h = traceFloor(co, ci);
	// Get intersection with outer glass surface
	float hBox = traceAABB(co, ci, float3(-0.5 - w, -0.5 - w, -0.5 - w), float3(0.5 + w, 0.5, 0.5 + w)).x;
	if (hBox < largeNum) {
		// Get intersection with inner glass surface
		float3 pBox = co + hBox * ci;
		float2 hInnGlass = traceAABB(pBox, ci, float3(-0.5, -0.5, -0.5), float3(0.5, 0.5, 0.5));
		if (hInnGlass.x < largeNum) {
			// Trace through the water until we get a hit!
			float3 pInnGlass = pBox + hInnGlass.x*ci;
			float4 posWater = traceWater(pInnGlass, ci, hInnGlass.y - hInnGlass.x);
			float3 normal = normalize(computeGradient2(posWater));
			return float4(0.5f+0.5f*normal, 1.0f);
		}

		return float4(pBox, 1.0f);
	}

	// Sky/land
	if (h < largeNum) {
		float fV = 0.95f;
		return float4(fV, fV, fV, 1.0f);
	}
	else {
		float sV = 0.5f;
		return float4(sV, sV, sV, 1.0f);
	}
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