//***********************************************
// GameTimer.cpp
// Provides a clock for timing applications.
// From Frank Luna's Direct3D 11 book.
//***********************************************

#include <Windows.h>
#include "GameTimer.h"

GameTimer::GameTimer() :
	mSecondsPerCount(0.0), mDeltaTime(-1.0), mBaseTime(0),
	mPausedTime(0), mPrevTime(0), mCurrTime(0), mStopped(false)
{
	__int64 countsPerSec;
	QueryPerformanceFrequency((LARGE_INTEGER*)&countsPerSec);
	mSecondsPerCount = 1.0 / (double)countsPerSec;
}

float GameTimer::TotalTime() const {
	// Comments omitted - see book, pg. 123!
	if (mStopped) {
		// Don't count the time that has passed since we stopped.
		// Remove previous paused times.
		return (float)((mStopTime - mPausedTime - mBaseTime)*mSecondsPerCount);
	}
	else {
		return (float)((mCurrTime - mPausedTime - mBaseTime)*mSecondsPerCount);
	}
}

float GameTimer::DeltaTime() const {
	return (float)mDeltaTime;
}

void GameTimer::Reset() {
	__int64 currTime;
	QueryPerformanceCounter((LARGE_INTEGER*)&currTime);

	mBaseTime = currTime;
	mPrevTime = currTime;
	mStopTime = 0;
	mStopped = false;
}

void GameTimer::Stop() {
	// If we are already stopped, don't do anything.
	if (!mStopped) {
		__int64 currTime;
		QueryPerformanceCounter((LARGE_INTEGER*)&currTime);

		// Otherwise, save the time we stopped at, and set
		// the Boolean flag indicating the timer is stopped.
		mStopTime = currTime;
		mStopped = true;
	}
}

void GameTimer::Start() {
	__int64 startTime;
	QueryPerformanceCounter((LARGE_INTEGER*)&startTime);

	// Determine the amount of time we've been paused
	// If we were never paused, there's nothing to do.
	if (mStopped) {
		mPausedTime += (startTime - mStopTime);

		mPrevTime = startTime;
		mCurrTime = startTime; // Not actually necessary; just easier to think about.
		mStopTime = 0;
		mStopped = false;
	}
}

void GameTimer::Tick() {
	if (mStopped) {
		mDeltaTime = 0.0;
		return;
	}

	// Get the time this frame.
	__int64 currTime;
	QueryPerformanceCounter((LARGE_INTEGER*)&currTime);
	mCurrTime = currTime;

	// Time difference between this frame and the previous.
	mDeltaTime = (mCurrTime - mPrevTime)*mSecondsPerCount;

	// Prepare for next frame.
	mPrevTime = mCurrTime;

	// Force nonnegative. The DXSDK's CDXUTTimer mentions that if the
	// processor goes into a power save mode or we get shuffled to another
	// processor, then mDeltaTime can be negative.
	if (mDeltaTime < 0.0) {
		mDeltaTime = 0.0;
	}
}