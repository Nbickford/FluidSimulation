// Given a list of particles, a list of offsets of cells, and
// a (linear float) buffer to which to write the particles, this compute shader
// copies each particle into a position inside its array pointed
// to by its cell, changing the list in the process.

#include "gpParticleStruct.hlsli"
#include "gpStdParameters.hlsli"

StructuredBuffer<Particle> gInputParticles : register(t0);
RWTexture3D<uint> gOffsets : register(u0);
RWStructuredBuffer<Particle> gOutputParticles : register(u1);

[numthreads(64, 1, 1)]
void main( uint3 DTid : SV_DispatchThreadID )
{
	if (DTid.x >= mNumParticles) {
		return;
	}

	// particle->cell code must match that in gpCountParticles.hlsl
	Particle particle = gInputParticles[DTid.x];
	float3 p = particle.pos; // in [-1/(2mM),1-1/(2mM)].
	uint3 cellID = uint3(mM * p + 0.5f);

	// Atomically get the value of gOffsets[cellID] and add 1 to it
	uint myIndex;
	InterlockedAdd(gOffsets[cellID], 1, myIndex);

	// Store the particle
	gOutputParticles[myIndex] = particle;
}