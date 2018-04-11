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

// Grid dimensions and inverse grid dimensions
static float3 mM;
static float3 invmM;

static float3 mGroundColor = float3(0.8f, 0.8f, 0.8f);
static float3 mSpot = normalize(float3(-0.7f, 0.05f, 0.5f)); // must be normalized

static const float3 betaR = float3(3.8e-6f, 13.5e-6f, 33.1e-6f);
static const float betaM = 21e-6f;

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

static const float w = 0.02; // Wall size
static const float largeNum = 100000.0f; // Generic large number to use if rays don't hit

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

// Given a point in [0,1], gives the distance to the fluid at p.
float map(float3 p) {
#if 1
	// Use iq's smoothstep trick to get more continuous interpolation results:
	// http://iquilezles.org/www/articles/texture/texture.htm
	float3 mp = mM * p + float3(0.5f, 0.5f, 0.5f);
	float3 i = floor(mp);
	float3 f = mp - i;

	f.xz = f.xz*f.xz*f.xz*(f.xz*(f.xz*6.0 - 15.0) + 10.0);
	//f.xz = f.xz*f.xz*(3.0f-2.0f*f.xz);
	//f = float3(0.5f, 0.5f, 0.5f);
	//f = lerp(f, f * f*(3.0f - 2.0f*f), 1.0f);
	// "two quartercircles" <- this works really nicely!!
	//float3 f2 = frac(f + float3(0.5f, 0.5f, 0.5f)) - float3(0.5f, 0.5f, 0.5f);
	//f = float3(0.5f, 0.5f, 0.5f) + sign(f-float3(0.5f, 0.5f, 0.5f))*(0.5*sqrt(abs(1.0f - 4.0f*f2*f2)));

	// "middle 0" - OK, but very blocky
	//f = 3 * f - 6 * f*f + 4 * f*f*f;
	// inverse of "middle 0":
	//float3 f2 = 2.0f*f - float3(1.0f, 1.0f, 1.0f);
	//f = 0.5f*(sign(f2)*pow(abs(f2), 1.0f / 3.0f) + float3(1.0f, 1.0f, 1.0f));

	// piecewise Hermite
	//float3 f2 = 2.0f*frac(f + float3(0.5f, 0.5f, 0.5f)) - float3(1.0f, 1.0f, 1.0f);
	//float3 af2 = abs(f2);
	//f = float3(0.5f, 0.5f, 0.5f) * sign(f)*
	// all the points and derivatives
	//f.xz = f.xz * (1 + f.xz * (8 + f.xz * (-32 + f.xz * (40 - 16*f.xz))));

	// nice Gaussian thing
	float a = 1.395952;
	float s2 = 0.892135*0.892135;
	//f.xz = a * (float2(1.0f, 1.0f) - exp(-f.xz * f.xz / s2));
	//f = a * (float3(1.0f, 1.0f, 1.0f) - exp(-f * f / s2));
	mp = i + f;

	p = invmM * (mp - float3(0.5f, 0.5f, 0.5f));
#endif
	return gPhi.SampleLevel(sam, p, 0)*invmM.x;
}

// Returns the distance along the given ray to the first and second intersections of the AABB
// or largeNum if there was no hit, as well as the normals at the first and second intersections.
float2 intersectAABB(float3 co, float3 ci, float3 bmin, float3 bmax, out float3 norm1, out float3 norm2) {
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
		// Tricky normal computation (note: may not be normal at edges)
		// 1 at values where tm and tmin are equal; 0 otherwise
		norm1 = sign(tmin - float3(tm, tm, tm)) + float3(1.0f, 1.0f, 1.0f);
		norm1 = sign(t1 - t2)*norm1; // Sign

		// 1 at values where tM and tmax are equal; 0 if tM<tmax
		norm2 = sign(float3(tM, tM, tM) - tmax) + float3(1.0f, 1.0f, 1.0f);
		norm2 = sign(t2 - t1)*norm2; // Sign

		return float2(tm, tM);
	}
	else {
		return float2(largeNum, largeNum);
	}
}

// Given a ray /direction/ pointing from a volume with refractive index n1
// entering into a volume with refractive index n2 and a ray normal (opposite of RTR pg. 231),
// this function returns the weight of the reflected ray, the direction of
// the reflected ray, and the direction of the transmitted ray.
// (Assumes that dot(ci, n)<0.0 and that ci and n are normalized)
float fresnelTR(float3 ci, float3 n, float n1, float n2, out float3 rayRefl, out float3 rayTrans) {
	// Schlick's approximation to Fresnel
	// I think this is right
	float rf0 = pow((n2 - n1) / (n2 + n1), 2);
	float3 minusCi = -ci; // l=-ci
	float cosThetai = dot(n, minusCi);

	float fresnel = lerp(rf0, 1.0f, pow(1 - cosThetai, 5));

	// Reflection
	rayRefl = 2 * cosThetai*n + ci;

	// Transmission
	// From https://www.scratchapixel.com/lessons/3d-basic-rendering/introduction-to-shading/reflection-refraction-fresnel
	float eta = n1 / n2;
	float k = 1.0f - eta * eta * (1.0f - cosThetai * cosThetai);
	if (k < 0) {
		// Total internal reflection!
		rayTrans = float3(0.0f, 0.0f, 0.0f);
		return 1;
	}
	else {
		k = sqrt(k);
		rayTrans = eta * ci + (eta*cosThetai - k)*n;
		return fresnel;
	}
}

// Simplified HLSL port of the sky code from https://www.shadertoy.com/view/ld3XRr
// color conversion
float3 hsv2rgb(float3 c)
{
	float4 K = float4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
	float3 p = abs(frac(c.xxx + K.xyz) * 6.0 - K.www);
	return c.z * lerp(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
}

float3 SKY_grad(float h, float fTime)
{
	//Gradient values sampled from a reference image.
	const float3 r1 = float3(195. / 255., 43. / 255., 6. / 255.);
	const float3 r2 = float3(228. / 255., 132. / 255., 28. / 255.);
	const float3 bg1 = float3(168. / 255., 139. / 255., 83. / 255.);
	const float3 bl1 = float3(86. / 255., 120. / 255., 147. / 255.);
	const float3 bl2 = float3(96. / 255., 130. / 255., 158. / 255.);
	const float3 bl3 = float3(96. / 255., 130. / 255., 218. / 255.);

	h = h - h * 0.25*sin(fTime);
	float3 c;
	if (h<0.25)
		c = lerp(r1, r2, 4.*h);
	else if (h<0.5)
		c = lerp(r2, bg1, 4.*(h - 0.25));
	else
		c = lerp(bg1, bl2, 2.*(h - 0.5));

	float light = 1.0 + 0.25*sin(fTime);
	return lerp(c, bl3, 0.25 + 0.25*sin(fTime))*light;
}

float3 sampleEnvironment(float3 dir) {
	float fTime = -1.95;

	//Pseudo - Rayleigh scattering (daylight blue)
	float anglePosSun_FromOrigin = acos(dot(dir, mSpot));
	anglePosSun_FromOrigin = clamp(anglePosSun_FromOrigin, 0., 3.1415926);
	float posAngle = asin(dir.y);

	float fAtmosphereThickness = 2.0;
	float fTraversalDistance = 0.35*cos(sqrt(clamp(12.3*posAngle, 0.0, 100.0)) - 0.8) + 0.65;

	float dayV = 0.25 + 0.666*(0.3 + fTraversalDistance)*(dot(dir, mSpot) + 1.0) / 2.0;
	float dayS = 0.9 - fTraversalDistance / 1.60;
	float dayH = lerp(0.61, 0.65, dir.y);

	float3 day = hsv2rgb(float3(dayH, dayS, dayV));
	float3 gradS = SKY_grad(0.75 - 0.75*dot(dir, mSpot)*clamp(1.0 - 3.0*dir.y, 0.0, 1.0)*fTraversalDistance, fTime);
	float3 gradF = (gradS + day) / 2.0;
	
	// SUN
	//1/x for rapid rise close from d=0
	//2^abs(x) for soft long range ramp down
	float d = length(mSpot - dir)*10.;
	float I = 0.015 / abs(d) + pow(2., -abs(d*2.))*0.4;
	float3 c = float3(255. / 255., 213. / 255., 73. / 255.);
	gradF += c * I*8.0;

	//Distribute the excess R light on other components
	if (gradF.x > 1.0)
		gradF = gradF + float3(0, (gradF.x - 1.0) / 1.5, (gradF.x - 1.0) / 0.75);

	// Gamma correct

	return pow(abs(gradF), float3(0.4545, 0.4545, 0.4545));
}

// Given a ray, traces the ray through the glass, returning
// - The distance along the ray to the first intersection, or largeNum if there was a miss
// - The primary transmission ray
// - How much to weight it by
// - The (already weighted) sum of the reflections
float traceGlass(float3 co, float3 ci, out float3 primRayCo, out float3 primRayCi, out float primAlpha, out float3 reflSum) {
	// Obtain the primary hit and transmission ray directly
	float3 norm1, norm2;
	primRayCo = float3(largeNum, largeNum, largeNum);
	primRayCi = float3(largeNum, largeNum, largeNum);
	primAlpha = 0.0f;
	reflSum = float3(largeNum, largeNum, largeNum);

	float3 boxLow = float3(-0.5 - w, -0.5 - w, -0.5 - w);
	float3 boxHi = float3(0.5 + w, 0.5, 0.5 + w);
	float3 innLow = float3(-0.5, -0.5, -0.5);
	float3 innHi = float3(0.5, 0.5, 0.5);

	// First intersection
	float hMain;
	// Did we start out inside the inner box?
	if (abs(co.x) < 0.51f && abs(co.y) < 0.51f && abs(co.z) < 0.51f) {
		hMain = intersectAABB(co, ci, boxLow, boxHi, norm1, norm2).y;
		norm1 = -norm2;
	}
	else {
		// from outside the box
		hMain = intersectAABB(co, ci, boxLow, boxHi, norm1, norm2).x;
	}

	if (hMain >= largeNum) {
		return largeNum;
	}

	float3 pOuter = co + ci * hMain;

	// Special case for inner box (this isn't perfect, and I'm not sure why)
	if (pOuter.y > 0.49f && abs(pOuter.x) <= 0.503f && abs(pOuter.z) <= 0.503f) {
		primRayCo = pOuter;
		primRayCi = ci;
		primAlpha = 1.0f;
		reflSum = float3(0.0, 0.0, 0.0);
		return hMain;
	}

	float3 reflVec1;
	float3 transVec1;
	float fresnel = fresnelTR(ci, norm1, 1.0f, 1.5f, reflVec1, transVec1);

	reflSum = fresnel * sampleEnvironment(reflVec1);
	float innRayWeight = 1 - fresnel; // The weight of the ray inside the glass
	// at the current bounce

	float primRaySet = 0.0f; // 1 iff the primary ray is determined

	// Further bounces
	// innRayWeight = weight of current ray; reflVec1 = direction of current ray, p = position of current ray
	float3 p = pOuter;
	reflVec1 = transVec1;
	float h;
	for (int i = 0; i < 8; i++) {
		// From whichever point we are, trace to a box
		h = intersectAABB(p, reflVec1, innLow, innHi, norm1, norm2).x;
		float innIf1 = 1;
		if (h <= 0.01 || h >= largeNum) {
			h = intersectAABB(p, reflVec1, boxLow, boxHi, norm1, norm2).y;
			norm1 = -norm2;
			innIf1 = 0.0f;
		}
		p = p + reflVec1 * h;

		if (innIf1 < 0.5f) {
			// On the outside, so in the air
			fresnel = fresnelTR(reflVec1, norm1, 1.5, 1.0, reflVec1, transVec1);
			reflSum += innRayWeight * (1 - fresnel)*sampleEnvironment(transVec1);
			innRayWeight *= fresnel;
		}
		else {
			// On the inner box
			float phi = map(primRayCo + 0.001*primRayCi + float3(0.5f, 0.5f, 0.5f));
			if (phi < 0.0 || true) {
				// In the water in the box
				fresnel = fresnelTR(reflVec1, norm1, 1.5, 1.333, reflVec1, transVec1);
			}
			else {
				// In the air in the box
				fresnel = fresnelTR(reflVec1, norm1, 1.5, 1.0, reflVec1, transVec1);
			}
			primAlpha += innRayWeight * (1 - fresnel);
			innRayWeight = innRayWeight * fresnel;

			if (primRaySet < 0.5f && primAlpha>0.0f) {
				// Set the ray
				primRayCo = p;
				primRayCi = transVec1; // might be 0 if we only have total internal reflection!
				primRaySet = 1.0f;
			}
		}
	}

	// for the moment, just return that!
	return hMain;
}

// Given a ray origin, direction, and maximum trace distance,
// this method returns the (shifted!!!) position of the intersection of
// the given ray with the water and the distance along the ray.
// If there was no intersection, the distance is still set to maxT.
float4 intersectWater(float3 co, float3 ci, float maxT) {
	// Shift for convenience
	float3 p0 = co + float3(0.5f, 0.5f, 0.5f);
	float3 p = p0;

	// Do we start off inside or outside the water?
	float initialPhi = map(p);
	if (initialPhi > 0.0f || p.y>0.9999) {
		// Starting outside the water, tracing inwards.
		float t = 0.0f;
		float dt;
		for (int i = 0; i < 64; i++) {
			dt = map(p);
			t += dt;

			if (dt < 0.001f || t>=maxT) {
				break;
			}
			p = p0 + t * ci;
		}

		t = min(t, maxT);
		p = p0 + t * ci;
		return float4(p, t);
	}
	else {
		// Starting inside the water, tracing outwards.
		// We'll step forward by some amount each step until we exit
		// the surface; then we'll march back to where we were.
		// This is a very simple method, and will not always be correct.
		// The condition we need to satisfy is that we won't skip over any
		// internal structures and re-enter the fluid volume.
		// This has some glitches to it, but we'll keep it for the moment.
		float stepAmt = invmM.x; // currently 1 cell
		float t = 0.0f;
		float dt;
		for (int i = 0; i < 128; i++) {
			dt = map(p);
			t += stepAmt;
			if (dt >= 0.0f) {
				// we're successfully outside the water!
				break;
			}
			if (t >= maxT) {
				// Went outside the box; return the point exactly at the box edge
				return float4(p0+maxT*ci, maxT);
			}

			p = p0 + t * ci;
		}

		// Now that we're outside the water, we need to step
		// back until we're at the fluid interface.
		for (int j = 0; i < 48; i++) { // Do we really need this many iterations?
			dt = -map(p);
			t += dt;

			if (dt > -0.001f) { // no t>=maxT here
				break;
			}
			
			p = p0 + t * ci;
		}

		return float4(p, t);
	}
}

float3 computeGradient(float4 pdt) {
	float3 p = pdt.xyz;
	// Handle edges well
	if (p.y > 0.999) {
		return float3(0.0f, 1.0f, 0.0f);
	}

	float dt = map(p);
	float3 e = float3(0.005f, 0.0f, 0.0f);
	return float3(map(p + e),
		map(p + e.yxy),
		map(p + e.yyx)) - float3(dt, dt, dt);
}

// Traces from the water at the given ray for 0 bounces,
// and returns the resulting color.
float3 traceWater0(float3 co, float3 ci) {
	float3 primRayCo, primRayCi, reflSum;
	float primAlpha;
	traceGlass(co, ci, primRayCo, primRayCi, primAlpha, reflSum);
	return primAlpha * sampleEnvironment(primRayCi) + reflSum;
}

// Traces from the water at the given ray for 1 bounce,
// generating 2 rays, and returns the resulting color.
float3 traceWater1(float3 co, float3 ci) {
	// Step a tiny amount away from the surface
	co = co + 0.001*ci;
	// Compute max trace distance
	float3 null1, null2;
	float maxT = intersectAABB(co, ci, float3(-0.5, -0.5, -0.5), float3(0.5, 0.5, 0.5), null1, null2).y;
	// Trace into the water
	float4 hWater = intersectWater(co, ci, maxT);
	float3 intersectionPoint = hWater.xyz - float3(0.5, 0.5, 0.5); // need to adjust for shift
	if (hWater.w >= maxT) {
		// Did not hit the water
		return traceWater0(intersectionPoint, ci);
	}
	else {
		// Hit the water; generate a single reflection and transmission ray!
		// We'll just analytically determine whether we were coming from inside or outside the water
		float3 norm = normalize(computeGradient(hWater)); // these are the right coordinates for computeGradient
		float n1 = 1.000; float n2 = 1.333;
		// If dot(norm, ci)>0.0f, then we were coming from inside the water (so swap n1 and n2, and flip the normal)
		if (dot(norm, ci) > 0.0f) {
			n1 = 1.333;
			n2 = 1.000;
			norm = -norm;
		}
		// Compute reflection and transmission vectors
		float3 rayRefl;
		float3 rayTrans;
		float fresnel = fresnelTR(ci, norm, n1, n2, rayRefl, rayTrans);
		return fresnel * traceWater0(intersectionPoint, rayRefl) + (1 - fresnel)*traceWater0(intersectionPoint, rayTrans);
	}
}

// Traces from the water at the given ray for 2 bounces,
// generating 4 rays, and returns the resulting color.
float3 traceWater2(float3 co, float3 ci) {
	// Step a tiny amount away from the surface
	co = co + 0.001*ci;
	// Compute max trace distance
	float3 null1, null2;
	float maxT = intersectAABB(co, ci, float3(-0.5, -0.5, -0.5), float3(0.5, 0.5, 0.5), null1, null2).y;
	// Trace into the water
	float4 hWater = intersectWater(co, ci, maxT);
	float3 intersectionPoint = hWater.xyz - float3(0.5, 0.5, 0.5); // need to adjust for shift
	if (hWater.w >= maxT) {
		// Did not hit the water
		return traceWater0(intersectionPoint, ci); // this should indeed be traceWater0
	}
	else {
		// Hit the water; generate a single reflection and transmission ray!
		// We'll just analytically determine whether we were coming from inside or outside the water
		float3 norm = normalize(computeGradient(hWater)); // these are the right coordinates for computeGradient
		float n1 = 1.000; float n2 = 1.333;
		// If dot(norm, ci)>0.0f, then we were coming from inside the water (so swap n1 and n2, and flip the normal)
		if (dot(norm, ci) > 0.0f) {
			n1 = 1.333;
			n2 = 1.000;
			norm = -norm;
		}
		// Compute reflection and transmission vectors
		float3 rayRefl;
		float3 rayTrans;
		float fresnel = fresnelTR(ci, norm, n1, n2, rayRefl, rayTrans);
		return fresnel * traceWater1(intersectionPoint, rayRefl) + (1 - fresnel)*traceWater1(intersectionPoint, rayTrans);
	}
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

	float3 col = float3(0.0f, 0.0f, 0.0f);

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

	// Draw the cube
	float3 primRayCo, primRayCi, reflSum;
	float transWeight;
	float firstHit = traceGlass(co, ci, primRayCo, primRayCi, transWeight, reflSum);

	if (firstHit < largeNum) {
		if (transWeight == 0.0) {
			col = reflSum;
		}

		// Trace into the water
		col = traceWater2(primRayCo, primRayCi);
	}
	else {

		// We didn't hit the cube, so render the plane:
		float h = traceFloor(co, ci);
		if (h >= largeNum || true) { // We no longer have ground
			// Didn't hit the plane
			col = sampleEnvironment(ci);
		}
		else {
			// Hit the plane
			col = mGroundColor;
		}
	}

	return float4(pow(abs(col), float3(2.2, 2.2, 2.2)), 1.0f);
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