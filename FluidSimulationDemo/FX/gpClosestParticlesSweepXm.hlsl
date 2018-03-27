// gpClosestParticlesSweepXm.hlsl
// This is one of the compute shader passes for extrapolating closest-
// particle and level set information to the rest of the grid, using
// a fast sweeping technique (for this file, in x-).

// Uses same parameters as in gpComputeClosestParticleNeighbors.hlsl.
#include "gpParticleStruct.hlsli"
#include "gpStdParameters.hlsli"
// Array of cells of particles data structure
Texture3D<uint> gOffsets : register(t0);
StructuredBuffer<Particle> gParticles : register(t1);
// Pointers to closest particles for each cell. Should be initially set to 0s.
RWTexture3D<uint> gClosestParticles : register(u0);
// Level set. Should be initially set to infinities.
RWTexture3D<float> gPhi : register(u1);


// Each thread just scans along x, looking at x- while moving in x+.
[numthreads(1, 8, 8)]
void main( uint3 DTid : SV_DispatchThreadID )
{
	uint3 myCellPos = DTid;
	// These will be weird to start out with, but should be OK.
	uint   lastClosestParticle = gClosestParticles[myCellPos];
	float3 lastClosestParticlePosition = mM * gParticles[lastClosestParticle].pos;

	uint xSize = uint(mM.x);
	for (uint x = 1; x < xSize; x++) {
		myCellPos.x = x;
		// We modify the cell at myCellPos and gPhi if the distance to lastClosestParticle
		// is better than the current distance (gPhi). Otherwise, the cell's current distance
		// is better, and se we update lastClosestParticle.
		float distToLast = distance(lastClosestParticlePosition, myCellPos) - mParticleRadius;
		if (distToLast < gPhi[myCellPos]) {
			gPhi[myCellPos] = distToLast;
			gClosestParticles[myCellPos] = lastClosestParticle;
		}
		else {
			lastClosestParticle = gClosestParticles[myCellPos];
			lastClosestParticlePosition = mM * gParticles[lastClosestParticle].pos;
		}
	}
}