// gpExtrapolateParticleVelocities.hlsl
// Given a 3D grid where each value is either valid (denoted by its value),
// or invalid (denoted by +1.#INF), this compute shader will fill in each
// invalid value neigboring any valid value with the average of its valid
// neighbors.
// We could repeat this process to actually extrapolate to the entire grid;
// however, for now we'll just set values not neighborinng a valid value to
// 0, denoting that they are well within the air.
//
// We define the set of neighbors of a cell using an array in the code; for
// now, it's the standard 6-neighbor set in 3D.

// In/Out: An array.
RWTexture3D<float> gArray : register(u0);

[numthreads(4, 4, 4)]
void main( uint3 DTid : SV_DispatchThreadID )
{
	if (isinf(gArray[DTid])) {
		int3 id2 = int3(DTid);
		int3 dirs[6] = { int3(1,0,0), int3(-1,0,0), int3(0,1,0), int3(0,-1,0), int3(0,0,1), int3(0,0,-1) };
		float numNeighbors = 0.0f;
		float sumNeighbors = 0.0f;
		[unroll]
		for (int i = 0; i < 6; i++) {
			int3 otherid = id2 + dirs[i];
			float val = gArray[otherid];
			if (!isinf(val)) {
				sumNeighbors += val;
				numNeighbors += 1.0f;
			}
		}

		if (numNeighbors != 0.0f) {
			gArray[DTid] = sumNeighbors / numNeighbors;
		}
		else {
			// Cell is well within the air
			gArray[DTid] = 0.0f;
		}
	}
}