// gpAddBodyForces.hlsl
// This is pretty simple: it just adds constant gravity (-9.81 m/s^2) to the MAC V grid!
// Ideally, we'd take into account m_CellsPerMeter here, but for now we just need dt.

#include "gpStdParameters.hlsli"

// The MAC V array to read and write to.
RWTexture3D<float> gMacV : register(u0);

[numthreads(4, 4, 4)]
void main( uint3 DTid : SV_DispatchThreadID )
{
	// Q: Should we be keeping our edge velocities at 0 here?
	gMacV[DTid] -= 9.81*mDT;
}