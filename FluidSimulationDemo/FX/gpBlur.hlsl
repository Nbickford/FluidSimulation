// gpBlur.hlsl
// Performs a really simple blur intended for level sets to smooth things out slightly

RWTexture3D<float> grid;

[numthreads(4, 4, 4)]
void main( uint3 DTid : SV_DispatchThreadID )
{
	int3 del = int3(1, 0, 0);
	grid[DTid] = (grid[DTid] +
		grid[DTid + del.xyy]
		+ grid[DTid - del.xyy]
		+ grid[DTid + del.yxy]
		+ grid[DTid - del.yxy]
		+ grid[DTid + del.yyx]
		+ grid[DTid - del.yyx]) / 7.0f;
}