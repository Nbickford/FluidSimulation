// gpProjectComputeRHS.hlsl
// Computes the right-hand-side b of the linear system representing the projection
// problem to solve. More specifically, b[x,y,z] is just the discrete divergence
// at (x,y,z) times a scale factor, -dx * m_rho / dt (dx = real-world distance between cells)

#include "gpStdParameters.hlsli" // Standard parameters

// Inputs: Velocity fields
Texture3D<float> gMacU : register(t0);
Texture3D<float> gMacV : register(t1);
Texture3D<float> gMacW : register(t2);
// Outputs: RHS of projection equation
RWTexture3D<float> gB : register(u0);

[numthreads(4, 4, 4)]
void main( uint3 DTid : SV_DispatchThreadID )
{
	// Note: At the moment, we're taking m_CellsPerMeter to be mM.x. This may not hold in the future.
	float dx = 1.0f / mM.x;
	float mRho = 1000.0f; // Density of water, from Simulation.h
	float scale = -dx * mRho / mDT;

	// We also take solidVel to be 0.
	float div = gMacU[DTid + uint3(1, 0, 0)] - gMacU[DTid]
		      + gMacV[DTid + uint3(0, 1, 0)] - gMacV[DTid]
		      + gMacW[DTid + uint3(0, 0, 1)] - gMacW[DTid];
	gB[DTid] = scale * div;
}