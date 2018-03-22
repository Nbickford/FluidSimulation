// Includes standard parameters cbuffer.

cbuffer Parameters:register(b0) {
	float3 mM : packoffset(c0);
	float mDT : packoffset(c0.w);
	float mAlpha : packoffset(c1.x);
	float mParticleRadius : packoffset(c1.y);
	uint mNumParticles:packoffset(c2.x);
};