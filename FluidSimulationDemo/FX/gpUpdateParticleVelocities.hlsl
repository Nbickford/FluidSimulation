// gpUpdateParticleVelocities.hlsl
// Given new and old velocity fields and a list of particles,
// this shader updates the particle velocities using a hybrid
// PIC/FLIP method based off of the parameter mAlpha.
//
// This code is syntactically based upon the MAC grid lookup
// code in gpAdvect.hlsl.

// Standard parameters and particle structure
#include "gpParticleStruct.hlsli"
#include "gpStdParameters.hlsli"

SamplerState sam : register(s0);

// Inputs: New and old MAC grids
Texture3D gMACU : register(t0);
Texture3D gMACV : register(t1);
Texture3D gMACW : register(t2);
Texture3D gOldMACU : register(t3);
Texture3D gOldMACV : register(t4);
Texture3D gOldMACW : register(t5);

// In/Out: Structured Particle buffer
RWStructuredBuffer<Particle> gParticles : register(u0);

// MAC Cell interpolation code (note: p in _worldspace_ coordinates)
float3 InterpolateNewMACCell(float3 p) {
	// From gpAdvect.hlsl

	float3 ep = (mM * p + 1) / (mM + 1);
	float3 sp = p + 0.5f / mM;
	return float3(
		gMACU.SampleLevel(sam, float3(ep.x, sp.y, sp.z), 0.0f).r,
		gMACV.SampleLevel(sam, float3(sp.x, ep.y, sp.z), 0.0f).r,
		gMACW.SampleLevel(sam, float3(sp.x, sp.y, ep.z), 0.0f).r
		);
}

float3 InterpolateOldMACCell(float3 p) {
	// From gpAdvect.hlsl

	float3 ep = (mM * p + 1) / (mM + 1);
	float3 sp = p + 0.5f / mM;
	return float3(
		gOldMACU.SampleLevel(sam, float3(ep.x, sp.y, sp.z), 0.0f).r,
		gOldMACV.SampleLevel(sam, float3(sp.x, ep.y, sp.z), 0.0f).r,
		gOldMACW.SampleLevel(sam, float3(sp.x, sp.y, ep.z), 0.0f).r
		);
}

[numthreads(64, 1, 1)]
void main( uint3 DTid : SV_DispatchThreadID )
{
	// u^new = (1-alpha)*u_old + newgrid - (1-alpha)*oldgrid
	Particle p = gParticles[DTid.x];

	gParticles[DTid.x].vel = (1.0f - mAlpha)*(p.vel) + InterpolateNewMACCell(p.pos) - (1.0f - mAlpha)*InterpolateOldMACCell(p.pos);
}