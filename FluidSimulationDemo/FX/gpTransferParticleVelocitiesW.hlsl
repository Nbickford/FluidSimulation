// gpTransferParticleVelocitiesW.hlsl
// Transfers particles to a MAC W velocity grid by having each grid point
// look at the particles at its nearest (18) cells and computing weights
// using a trilinear filter:
// 1-     ^
//      /   \
// 0- |/  |  \|
//   -1   0   1

// Uses same parameters as in gpComputeClosestParticleNeighbors.hlsl.
#include "gpParticleStruct.hlsli"
// Array of cells of particles data structure
Texture3D<uint> gOffsets : register(t0);
StructuredBuffer<Particle> gParticles : register(t1);
// Closest-particle information
Texture3D<uint> gClosestParticles : register(t2);
// The MAC W array to write to. All values will be set by this routine.
RWTexture3D<float> gMacW : register(u0);
// Interesting question: If we also use Phi as an input, can we do the extrapolation
// here as well?

#include "gpParticleIndexing.hlsli"

[numthreads(4, 4, 4)]
void main( uint3 DTid : SV_DispatchThreadID )
{
	// Cell-space MAC sampling position.
	float3 samplePosition = float3(DTid.x, DTid.y, DTid.z - 0.5f);
	// Wall velocities are always zero.
	if (samplePosition.z<0.0f || samplePosition.z>mM.z-1.0f) {
		gMacW[DTid] = 0;
		return;
	}

	// We look at cells [-1,0] in Z and [-1, 0, 1] in X and Y.
	float accVel = 0.0f;
	float accWeight = 0.0f;
	//[unroll]
	for (int dz = -1; dz <= 0; dz++) {
		//[unroll]
		for (int dy = -1; dy <= 1; dy++) {
			//[unroll]
			for (int dx = -1; dx <= 1; dx++) {
				int3 lookupCell = int3(DTid) + int3(dx, dy, dz);
				// Optional: Do range check for lookupCell here.
				uint2 osx = getOffsetStartEnd(lookupCell);
				for (uint i = osx.x; i < osx.y; i++) {
					Particle particle = gParticles[i];
					float3 cellPos = particle.pos * mM;

					float3 alphaVec = max(0.0f, 1.0f - abs(cellPos - samplePosition));
					float alpha = alphaVec.x*alphaVec.y*alphaVec.z;

					accVel    += particle.vel.z*alpha;
					accWeight += alpha;
				}
			}
		}
	}

	float zero_thresh = 0.01f;
	if (accWeight < zero_thresh) {
		// Let's match the Simulation3D code as closely as possible, using extrapolation!
		gMacW[DTid] = 1.#INF;
		/*
		// Just take velocity of closest particle
		Particle p1 = gParticles[gClosestParticles[DTid - uint3(0, 0, 1)]];
		Particle p2 = gParticles[gClosestParticles[DTid]];
		float d1 = distance(p1.pos*mM, samplePosition); // we don't subtract mParticleRadius here
		float d2 = distance(p2.pos*mM, samplePosition);
		if (d1 < d2) {
			gMacW[DTid] = p1.vel.z;
		}
		else {
			gMacW[DTid] = p2.vel.z;
		}*/
	}
	else {
		// Divide through
		gMacW[DTid] = accVel / accWeight;
	}
}