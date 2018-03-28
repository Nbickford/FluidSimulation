// gpParticleIndexing.hlsli
// Contains routines for indexing into the shifted-prefix-sum offset array.
// Assumes that we have the following array of cells of particles data structure:
//
// Texture3D<uint> gOffsets : register(t0);
// StructuredBuffer<Particle> gParticles : register(t1);
//

#include "gpStdParameters.hlsli"

// Takes an index in [0,mM]^3 and returns the index of the previous
// cell when laid out as a buffer.
// Note: As currently written, this is expensive.
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

// Gets the offset and length of the array containing
// the particles in cell p in gParticles, or [0,0] if
// p is outside the array.
uint2 getOffsetSize(int3 p) {
	int3 pc = previousCell(p);
	uint offset = gOffsets[pc];
	// actual offset into array, "next offset - actual offset"
	return uint2(offset, gOffsets[p] - offset);
}

// Gets the inclusive start and exclusive end of the array
// containing the particles in cell p in gParticles, or [0,0]
// if p is outside the array.
uint2 getOffsetStartEnd(int3 p) {
	int3 pc = previousCell(p);
	return uint2(gOffsets[pc], gOffsets[p]);
}