//***********************************************
// GameTimer.h
// Provides a clock for timing applications.
// From Frank Luna's Direct3D 11 book.
//***********************************************

#ifndef DX11_GAMETIMER_H
#define DX11_GAMETIMER_H

class GameTimer {
public:
	GameTimer();

	float TotalTime() const; // in seconds
	float DeltaTime() const; // in seconds

	void Reset(); // Call before message loop.
	void Start(); // Call when unpaused.
	void Stop(); // Call when paused.
	void Tick(); // Call every frame.

private:
	double mSecondsPerCount;
	double mDeltaTime;

	// note to self: __int64 is the same as inttypes.h's int64_t.
	__int64 mBaseTime; // Time the timer was created or reset.
	__int64 mPausedTime; // If the timer is running, this counts the amount of time
						 // for which the timer was stopped.
	__int64 mStopTime; // The time at which the timer was stopped.
	__int64 mPrevTime; // previous time; used in Tick to compute delta time.
	__int64 mCurrTime; // the current time

	bool mStopped;
};

#endif // DX11_GAMETIMER_H