// gpComputeClosestParticleNeighbors.hlsl
// Computes the indices of the closest particles to the center
// of each grid cell either containing particles or adjacent
// (even diagonally) to a grid cell containing particles.

#include "gpParticleStruct.hlsli"

// Parameters:
// An mMX*mMY*mMZ array of unsigned integers.
// [elt-1] holds the offset of the particles in cell elt
// in the structured buffer of particles.
Texture3D<uint> gOffsets : register(t0);
// Holds an array of particles for each grid cell.
StructuredBuffer<Particle> gParticles : register(t1);
// A 3D array of pointers to each cell center's closest particles.
RWTexture3D<uint> gClosestParticles : register(u0);
// A 3D array of the minimum distance from each cell center to
// any particle. Should be initially set to infinities.
RWTexture3D<float> gPhi : register(u1);

#include "gpParticleIndexing.hlsli"

// We'll use groupshared memory to cache array offsets
// and counts.

// Size of the thread group on a side
#define N 8
#define CacheSize (N+2)

// These would ideally be 3D arrays, but HLSL only has 1D arrays.
// Combined size: 2*4*CacheSize^3 bytes -
// so we should have N<12.6 for optimal performance.
groupshared uint2 kOffsetSizeCache[CacheSize*CacheSize*CacheSize];

// Input in [-1,N]^3 to a position in kOffsetSizeCache.
int index3(int3 p) {
	return (p.x + 1) + CacheSize * ((p.y + 1) + CacheSize * (p.z + 1));
}

[numthreads(N, N, N)]
void main(int3 groupThreadID : SV_GroupThreadID,
	uint3 DTid : SV_DispatchThreadID )
{
	// Each thread on the periphery loads the offsets
	// and counts of those they are the closest to (at most 7.)
	// This is a bit tricky - can it be phrased better (switch table?)?
	// Center
	int mykIndex = index3(groupThreadID);
	kOffsetSizeCache[mykIndex] = getOffsetSize(DTid);
	// "Normal"
	// (a tricky hack - each component -1 or 1 at each relevant edge)
	int4 dp = int4(sign((groupThreadID - float3(1, 1, 1)) / (N - 1)), 0);
	// 7 samples
	// Left/right
	if (dp.x != 0) {
		kOffsetSizeCache[index3(groupThreadID + dp.xww)] = getOffsetSize(DTid + dp.xww);
		// xy plane diagonals
		if (dp.y != 0) {
			kOffsetSizeCache[index3(groupThreadID + dp.xyw)] = getOffsetSize(DTid + dp.xyw);
			// xyz diagonals
			if (dp.z != 0) {
				kOffsetSizeCache[index3(groupThreadID + dp.xyz)] = getOffsetSize(DTid + dp.xyz);
			}
		}
		// xz plane diagonals
		if (dp.z != 0) {
			kOffsetSizeCache[index3(groupThreadID + dp.xwz)] = getOffsetSize(DTid + dp.xwz);
		}
	}
	// Up/down
	if (dp.y != 0) {
		kOffsetSizeCache[index3(groupThreadID + dp.wyw)] = getOffsetSize(DTid + dp.wyw);
		// yz plane diagonals
		if (dp.z != 0) {
			kOffsetSizeCache[index3(groupThreadID + dp.wyz)] = getOffsetSize(DTid + dp.wyz);
		}
	}
	// Forwards/back
	if (dp.z != 0) {
		kOffsetSizeCache[index3(groupThreadID + dp.wwz)] = getOffsetSize(DTid + dp.wwz);
	}

	// Wait for all threads to finish
	GroupMemoryBarrierWithGroupSync();

	// For each neighboring cell, iterate through its particles, finding
	// the closest one.
	//[unroll]
	for (int z = -1; z <= 1; z++) {
		//[unroll]
		for (int y = -1; y <= 1; y++) {
			//[unroll]
			for (int x = -1; x <= 1; x++) {
				uint2 offSize = kOffsetSizeCache[mykIndex + x + CacheSize * y + CacheSize * CacheSize*z];
				uint endEx = offSize.x + offSize.y;
				// Iterate through particles
				for (uint i = offSize.x; i < endEx; i++) {
					// Get that particle's position in cell-space
					float3 p = gParticles[i].pos * mM;
					// Compute distance (might be easier than squared distance?)
					float dist = distance(p, DTid) - mParticleRadius;
					if (dist < gPhi[DTid]) {
						gPhi[DTid] = dist;
						gClosestParticles[DTid] = i;
					}
				}
			}
		}
	}

	// and we should be done!
}