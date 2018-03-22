// For some reason, there doesn't seem to be a simple way to clear a 3D texture on the GPU
// (perhaps there is?), so this compute shader just sets every element of the output
// array to 0.

RWTexture3D<uint> gArrayToClear;

[numthreads(4, 4, 4)]
void main( uint3 DTid : SV_DispatchThreadID )
{
	gArrayToClear[DTid] = 0;
}