// gpProjectIteration.hlsl
// Given a linear system given implicitly by diagCoeffs, Phi, and the RHS b,
// this shader runs one iteration of red-black checkerboard successive over-relaxation*
// on the array of pressures, p.

// *Somewhat problematically, we don't currently enforce thread synchronization
// between the two steps of checkerboard SOR, but the result should still converge
// to the correct result. One good question is whether it's worth it to separate this
// into two steps, or if there's a more efficient way of implementing this inner loop.

// Standard parameters (we pass in the SOR parameter omega through mAlpha)
#include "gpStdParameters.hlsli"

// Inputs: diagCoeffs, Phi, b
Texture3D<float> gDiagCoeffs : register(t0);
Texture3D<float> gPhi : register(t1);
Texture3D<float> gB : register(t2);
// Outputs: p
RWTexture3D<float> gP : register(u0);


// Each thread will take care of a 2x1x1 region, so each thread group
// handles an 8x4x4 region.
[numthreads(4, 4, 4)]
void main( uint3 DTid : SV_DispatchThreadID )
{
	// For each step
	// [unroll]
	int step = 0;
		// Compute shifted checkerboard position based off of step
		// (this ensures pos.x+pos.y+pos.z == step (mod 2))
		int3 pos = int3(((step + DTid.y + DTid.z) % 2) + 2*DTid.x, DTid.y, DTid.z);

		// If this cell is air, then there's no equation for this cell.
		// Q: Does adding the following line help or hinder performance?
		// (If we remove it, we have to watch out for numNeighbors below being 0)
		if (gPhi[pos] >= 0.0) return;

		float numNeighbors = gDiagCoeffs[pos];

		// Literal translation of corresponding Simulation3D.cpp code
		// (in the future, we might store a lot of this in a list of places to look)
		// Note that bounds checks work because of HLSL out-of-range behavior
		float neighborMinusSum = 0.0f;
		if (gPhi[pos + int3(-1, 0, 0)] < 0.0) {
			neighborMinusSum -= gP[pos + int3(-1, 0, 0)];
		}
		if (gPhi[pos + int3( 1, 0, 0)] < 0.0) {
			neighborMinusSum -= gP[pos + int3( 1, 0, 0)];
		}
		if (gPhi[pos + int3( 0,-1, 0)] < 0.0) {
			neighborMinusSum -= gP[pos + int3( 0,-1, 0)];
		}
		if (gPhi[pos + int3( 0, 1, 0)] < 0.0) {
			neighborMinusSum -= gP[pos + int3( 0, 1, 0)];
		}
		if (gPhi[pos + int3(0, 0,-1)] < 0.0) {
			neighborMinusSum -= gP[pos + int3( 0, 0,-1)];
		}
		if (gPhi[pos + int3(0, 0, 1)] < 0.0) {
			neighborMinusSum -= gP[pos + int3( 0, 0, 1)];
		}

		// Successive over-relaxation
		gP[pos] = (1 - mAlpha)*gP[pos] + mAlpha * (gB[pos] - neighborMinusSum) / numNeighbors;
}