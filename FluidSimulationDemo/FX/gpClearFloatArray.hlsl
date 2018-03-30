// Like gpClearIntArray, this sets every value in a floating-point array
// to the value alpha indicated in the standard cbuffer.

#include "gpStdParameters.hlsli"

RWTexture3D<float> gArrayToClear;

[numthreads(4, 4, 4)]
void main( uint3 DTid : SV_DispatchThreadID )
{
	// The syntax for +infinity in HLSL seems to be 1.#INF, based off of
	// https://github.com/KhronosGroup/glslang/issues/672.
	gArrayToClear[DTid] = mAlpha;
}