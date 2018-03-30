// gpProjectComputeDiagCoeffs.hlsl
// Computes the diagonal coefficients of the linear system in the Project()
// step, incorporating the ghost fluid technique to handle interface boundaries.

#include "gpStdParameters.hlsli" // Standard parameters (for solid boundaries)
// Inputs: Phi
Texture3D<float> gPhi : register(t0);
// Outputs: diagCoeffs
RWTexture3D<float> gDiagCoeffs : register(u0);

[numthreads(4, 4, 4)]
void main( uint3 DTid : SV_DispatchThreadID )
{
	// Essentially, gDiagCoeffs[x,y,z] is 0 if in air.
	// Otherwise, it's equal to the # of non-solid neighbors
	// plus level set things.

	float myPhi = gPhi[DTid];
	if (myPhi >= 0.0f) {
		// We won't be accessing this value of gDiagCoeffs,
		// so we just don't have to modify it.
		return;
	}

	float maxLSRatio = 999.0f;
	// Expression for counting # of non-solid neighbors when mM>1
	// (mM.x-1)-x : takes mM.x-1 to 0
	// min(p, mM-1-p)-0.5: takes vals to bottom-left octant, shifting
	// those on edges to negative values
	// max(0, sign(...)): 0 if on edge, 1 if not on edge
	// .{1,1,1} + 3: sums values and adds 3
	float3 temp = float3(DTid);
	float numNeighbors = 3 + dot(max(0, sign(min(temp, mM - 1.0f - temp) - 0.5f)), float3(1.0f, 1.0f, 1.0f));

	// Ghost fluids
	// Thanks to the HLSL policy of returning 0 for out-of-bounds
	// accesses, this is really simple!
	float myPhiRecip = 1.0f / myPhi;
	int3 pos = int3(DTid);
	numNeighbors += clamp(-gPhi[pos + int3(-1, 0, 0)]*myPhiRecip, 0.0f, maxLSRatio)
		          + clamp(-gPhi[pos + int3( 1, 0, 0)]*myPhiRecip, 0.0f, maxLSRatio)
		          + clamp(-gPhi[pos + int3( 0,-1, 0)]*myPhiRecip, 0.0f, maxLSRatio)
		          + clamp(-gPhi[pos + int3( 0, 1, 0)]*myPhiRecip, 0.0f, maxLSRatio)
		          + clamp(-gPhi[pos + int3( 0, 0,-1)]*myPhiRecip, 0.0f, maxLSRatio)
		          + clamp(-gPhi[pos + int3( 0, 0, 1)]*myPhiRecip, 0.0f, maxLSRatio);

	gDiagCoeffs[DTid] = numNeighbors;
}