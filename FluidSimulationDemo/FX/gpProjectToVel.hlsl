// gpProjectToVel.hlsl
// Computes new velocity fields U, V, and W
// from the computed pressure solution and Phi.

#include "gpStdParameters.hlsli" // Standard parameters

// Inputs: pressure P, Phi
Texture3D<float> gP : register(t0);
Texture3D<float> gPhi : register(t1);
// Outputs: U, V, and W
RWTexture3D<float> gU : register(u0);
RWTexture3D<float> gV : register(u1);
RWTexture3D<float> gW : register(u2);

// Despite appearances, this shader should be invoked on a grid
// of size at least (mM.x-1)*(mM.y-1)*(mM.z-1).
[numthreads(4, 4, 4)]
void main( uint3 DTid : SV_DispatchThreadID )
{
	// Note: At the moment, we're taking m_CellsPerMeter to be mM.x. This may not hold in the future.
	// This scale factor must be the negative inverse of that in gpProjectComputeRHS.hlsl.
	float dx = 1.0f / mM.x;
	float mRho = 1000.0f; // Density of water, from Simulation.h
	float scale = mDT / (dx * mRho);

	float myPhi = gPhi[DTid];
	float myP = gP[DTid];

	// Transcription of relevant code from Simulation.cpp
	float maxLSRatio = 1000.0f;

	// U
	// Don't modify edges (x<mM.x-1)
	if (float(DTid.x) < mM.x - 1.5f) {
		uint3 oidx = DTid + uint3(1, 0, 0); // other index
		float phiO = gPhi[oidx];
		float myU = gU[oidx];
		// Open question: Can we do better than having four cases here?
		if (myPhi < 0.0) {
			if (phiO < 0.0) {
				gU[oidx] = myU - scale * (gP[oidx] - myP);
			}
			else { // phiO >= 0.0
				gU[oidx] = myU + scale * myP * (1 + clamp(-phiO / myPhi, 0.0f, maxLSRatio));
			}
		}
		else { // myPhi >= 0.0
			if (phiO < 0.0) {
				gU[oidx] = myU - scale * gP[oidx] * (1 + clamp(-myPhi / phiO, 0.0f, maxLSRatio));
			}
			else { // phiO >= 0.0
				gU[oidx] = 0.0f; // in the air
			}
		}
	}

	// V
	// Don't modify edges (y<mM.y-1)
	if (float(DTid.y) < mM.y - 1.5f) {
		uint3 oidx = DTid + uint3(0, 1, 0);
		float phiO = gPhi[oidx];
		float myV = gV[oidx];

		if (myPhi < 0.0) {
			if (phiO < 0.0) {
				gV[oidx] = myV - scale * (gP[oidx] - myP);
			}
			else { // phiO >= 0.0
				gV[oidx] = myV + scale * myP * (1 + clamp(-phiO / myPhi, 0.0f, maxLSRatio));
			}
		}
		else { // myPhi >= 0.0
			if (phiO < 0.0) {
				gV[oidx] = myV - scale * gP[oidx] * (1 + clamp(-myPhi / phiO, 0.0f, maxLSRatio));
			}
			else { // phiO >= 0.0
				gV[oidx] = 0.0f; // in the air
			}
		}
	}

	// W
	// Don't modify edges (z<mM.z-1)
	if (float(DTid.z) < mM.z - 1.5f) {
		uint3 oidx = DTid + uint3(0, 0, 1);
		float phiO = gPhi[oidx];
		float myW = gW[oidx];

		if (myPhi < 0.0) {
			if (phiO < 0.0) {
				gW[oidx] = myW - scale * (gP[oidx] - myP);
			}
			else { // phiO >= 0.0
				gW[oidx] = myW + scale * myP * (1 + clamp(-phiO / myPhi, 0.0f, maxLSRatio));
			}
		}
		else { // myPhi >= 0.0
			if (phiO < 0.0) {
				gW[oidx] = myW - scale * gP[oidx] * (1 + clamp(-myPhi / phiO, 0.0f, maxLSRatio));
			}
			else { // phiO >= 0.0
				gW[oidx] = 0.0f; // in the air
			}
		}
	}
	// end
}