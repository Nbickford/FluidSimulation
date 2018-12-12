#include "gpParticleStruct.hlsli"
#include "gpStdParameters.hlsli"

// Register reference: https://msdn.microsoft.com/en-us/library/windows/desktop/hh447206(v=vs.85).aspx
// cbuffer reference: https://msdn.microsoft.com/en-us/library/windows/desktop/bb509581(v=vs.85).aspx
// Usage reference: https://github.com/walbourn/directx-sdk-samples/blob/master/BasicCompute11/BasicCompute11.cpp

// Sampler states (for MAC grids)
SamplerState sam : register(s0);

// Data sources
Texture3D gMACU : register(t0);
Texture3D gMACV : register(t1);
Texture3D gMACW : register(t2);
// Outputs
RWStructuredBuffer<Particle> gParticles : register(u0);

// MAC Cell interpolation code (p in _worldspace_ coordinates)
float3 InterpolateMACCell(float3 p) {
	// Interesting problem: Our textures have samples located at half-pixels!
	// How can we take p in world-space and convert it to MAC-grid-space?
	// Answer: When a MAC velocity grid points in a particular direction, it is one
	// cell longer in that direction. When it doesn't, it is exactly the length
	// of the box in that direction (and the half-samples line up). 
	// Thus, we sample the U grid with coordinates (eX, Y, Z), where Y and Z
	// are the normal grid coordinates, but eX comes from (via the point-slope formula)
	// eX = (mX/(mX+1))x+1/(mX+1)
	// (We fit -1/(2mX) |-> (1/2)/(mX+1) and (mX-1/2)/mX |-> (mX+1/2)/(mX+1).

	// Actually, when sampling not in that direction, we want -1/(2mY)->0 and
	// (mY-1/2)/mY->1, so we need to add 1/(2mY) to those coordinates. (That's because the
	// actual pixel values are at half-pixels.)

	float3 ep = (mM * p + 1) / (mM+1);
	float3 sp = p + 0.5f / mM;
	return float3(
		gMACU.SampleLevel(sam, float3(ep.x, sp.y, sp.z), 0.0f).r,
		gMACV.SampleLevel(sam, float3(sp.x, ep.y, sp.z), 0.0f).r,
		gMACW.SampleLevel(sam, float3(sp.x, sp.y, ep.z), 0.0f).r
		);
}

// The number of threads in each thread group
// (we tell the GPU how many thread groups to dispatch
// in the call ID3D11DeviceContext::Dispatch(x, y, z))
// Usually, we want the product of these three numbers to
// be at least a factor of 32, although a factor of 64 works
// better on ATI cards.
[numthreads(64, 1, 1)]
void main( uint3 DTid : SV_DispatchThreadID )
{
	float3 p = gParticles[DTid.x].pos;

	// We'll use RK3 interpolation!
	// This was surprisingly difficult to find, but according to an article in GPU Gems 5
	// (source: https://books.google.com/books?id=uIDSBQAAQBAJ&pg=PA60&lpg=PA60&dq=using+flip+with+runge-kutta&source=bl&ots=XMJL03mmLe&sig=ZVlwK4HpjKePBfpq9ew1T4zeuUI&hl=en&sa=X&ved=0ahUKEwiWtcjpgdfZAhVIYK0KHVz1BVsQ6AEIeTAI#v=onepage&q=using%20flip%20with%20runge-kutta&f=false)
	// (search key: "using FLIP with runge-kutta")
	// we should actually update the particle's position entirely using interpolation on the grid,
	// and only update the particle's velocity using the grid-update step in FLIP.

	float3 k1 = InterpolateMACCell(p);
	float3 k2 = InterpolateMACCell(p + 0.5*mDT*k1);
	float3 k3 = InterpolateMACCell(p + 0.75*mDT*k2);

	float3 minV = -0.4f / mM;
	float3 maxV = 1.0f - 0.6f / mM;
	gParticles[DTid.x].pos = clamp(p + mDT*((2.0 / 9.0)*k1 + (3.0 / 9.0)*k2 + (4.0 / 9.0)*k3), minV, maxV);
}
