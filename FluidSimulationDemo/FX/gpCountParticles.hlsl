// Given a StructuredBuffer of particles, counts the number of particles in each grid cell
// and stores the result in the given RWTexture3D<uint>.
// PRECONDITION: gOutputCounts must be cleared to 0.

#include "gpParticleStruct.hlsli"
#include "gpStdParameters.hlsli"

StructuredBuffer<Particle> gInputParticles : register(t0);
RWTexture3D<uint> gOutputCounts;

[numthreads(64, 1, 1)]
void main( uint3 DTid : SV_DispatchThreadID )
{
	// Don't count particles beyond the end of the array
	if (DTid.x >= mNumParticles) {
		return;
	}

	float3 p = gInputParticles[DTid.x].pos; // in [-1/(2mM),1-1/(2mM)].
	// For casting and conversion rules in HLSL, see
	// https://msdn.microsoft.com/en-us/library/windows/desktop/bb172396(v=vs.85).aspx.
	uint3 cellID = uint3(mM * p + 0.5f);
	// Interesting note: We'd do this a totally different way if we were multithreading
	// this on a CPU.
	InterlockedAdd(gOutputCounts[cellID], 1);
}