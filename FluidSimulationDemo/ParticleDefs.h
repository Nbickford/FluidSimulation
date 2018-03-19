//************************************************************
// ParticleDefs.h
// Contains definitions of particle types used throughout
// simulation engines.
//
// Authors:
//   Neil Bickford
//************************************************************

#ifndef NB_FLUIDSIMULATION_PARTICLEDEFS_H
#define NB_FLUIDSIMULATION_PARTICLEDEFS_H

#include "MathHelper.h"
using namespace DirectX;

// Represents a fluid particle in 2D.
struct Particle {
	// Positions in real-world units (meters)
	float X;
	float Y;
	// Velocity in real-world units (m/s)
	float uX;
	float uY;

	Particle()
		:X(0.f), Y(0.f), uX(0.f), uY(0.f) {
	}

	Particle(float px, float py, XMFLOAT2 vel)
		:X(px), Y(py), uX(vel.x), uY(vel.y) {
	}

	Particle(float px, float py, float ux, float uy)
		:X(px), Y(py), uX(ux), uY(uy) {
	}
};

// Represents a fluid particle in 3D.
struct Particle3 {
	// Positions in real-world units (meters)
	float X;
	float Y;
	float Z;
	// Velocity in real-world units (m/s)
	float uX;
	float uY;
	float uZ;

	Particle3()
		:X(0.f), Y(0.f), Z(0.f), uX(0.f), uY(0.f), uZ(0.f) {
	}

	Particle3(float px, float py, float pz, XMFLOAT3 vel)
		:X(px), Y(py), Z(pz), uX(vel.x), uY(vel.y), uZ(vel.z) {
	}

	Particle3(float px, float py, float pz, float ux, float uy, float uz)
		:X(px), Y(py), Z(pz), uX(ux), uY(uy), uZ(uz) {
	}
};

#endif