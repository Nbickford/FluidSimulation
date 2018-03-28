// gpTransferParticleVelocitiesV.hlsl
// Transfers particles to a MAC V velocity grid by having each grid point
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
// The MAC V array to write to. All values will be set by this routine.
RWTexture3D<float> gMacV : register(u0);
// Interesting question: If we also use Phi as an input, can we do the extrapolation
// here as well?

#include "gpParticleIndexing.hlsli"

[numthreads(4, 4, 4)]
void main( uint3 DTid : SV_DispatchThreadID )
{
	// Cell-space MAC sampling position.
	float3 samplePosition = float3(DTid.x, DTid.y - 0.5f, DTid.z);
	// Wall velocities are always zero.
	if (samplePosition.y<0.0f || samplePosition.y>mM.y-1.0f) {
		gMacV[DTid] = 0;
		return;
	}

	// We look at cells [-1,0] in Y and [-1, 0, 1] in X and Z.
	float accVel = 0.0f;
	float accWeight = 0.0f;
	//[unroll]
	for (int dz = -1; dz <= 1; dz++) {
		//[unroll]
		for (int dy = -1; dy <= 0; dy++) {
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

					accVel    += particle.vel.y*alpha;
					accWeight += alpha;
				}
			}
		}
	}

	if (accWeight == 0.0f) {
		// Just take velocity of closest particle
		Particle p1 = gParticles[gClosestParticles[DTid - uint3(0, 1, 0)]];
		Particle p2 = gParticles[gClosestParticles[DTid]];
		float d1 = distance(p1.pos*mM, samplePosition); // we don't subtract mParticleRadius here
		float d2 = distance(p2.pos*mM, samplePosition);
		if (d1 < d2) {
			gMacV[DTid] = p1.vel.y;
		}
		else {
			gMacV[DTid] = p2.vel.y;
		}
	}
	else {
		// Divide through
		gMacV[DTid] = accVel / accWeight;
	}
}