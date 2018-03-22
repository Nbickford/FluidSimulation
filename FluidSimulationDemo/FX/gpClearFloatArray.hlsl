// Like gpClearIntArray, this sets every value in a floating-point array
// to some value, which is infinity in this case.
// TODO: Have this value be specified in a cbuffer instead.

RWTexture3D<float> gArrayToClear;

[numthreads(4, 4, 4)]
void main( uint3 DTid : SV_DispatchThreadID )
{
	// The syntax for +infinity in HLSL seems to be 1.#INF, based off of
	// https://github.com/KhronosGroup/glslang/issues/672.
	gArrayToClear[DTid] = 1.#INF;
}