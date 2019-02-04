//*****************************************************************
// GPUProfiler.h
// Header file for a query-based GPU profiler.
// Based on Nathan Reed's GPU Profiling 101 code: http://reedbeta.com/blog/gpu-profiling-101/
//
// Authors:
//   Neil Bickford
//*****************************************************************

#ifndef FSD_GPUPROFILER_H
#define FSD_GPUPROFILER_H

#include "d3dUtil.h"

// GPU Profiler Mark enumeration. Must be ordered consecutively.
enum GPUProfilerMark {
    GPU_PROFILER_MARK_BEGIN_FRAME, // This is called by BeginFrame
    // Application timestamps go here
    GPU_PROFILER_MARK_UPDATE,
    GPU_PROFILER_MARK_DRAW,
    // Application timestamps end here
    GPU_PROFILER_MARK_END_FRAME, // This is called by EndFrame
    GPU_PROFILER_MARK_COUNT
};

class GPUProfiler {
public:
    // Allocates and initializes GPU profiler resources.
    void Initialize(ID3D11Device* context);

    // Releases all GPU profiler resources.
    void ReleaseResources();

    // Makes the GPU profiler start recording a frame.
    // Note that Initialize must be called before this is called!
    void BeginFrame(ID3D11DeviceContext* context);

    // Adds an instruction to the GPU queue to store the value of the GPU's
    // internal clock when it hits this instruction.
    void TimestampComplete(ID3D11DeviceContext* context, GPUProfilerMark mark);

    // Tells the GPU profiler that the frame has just ended, and to start
    // collecting frame timing information from the GPU.
    // NOTE: Blocks and waits for last frame's timing information to be available
    // unless disabled!
    void EndFrame(ID3D11DeviceContext* context, bool skipLastFrameCollectBlock = false);

    // Returns last frame's recorded time in seconds between the given
    // profiler mark and the one before it.
    float DT(GPUProfilerMark mark);
private:
    int m_framesCollected = 0;
    ID3D11Query* m_disjointQueries[2]; // Double-buffered disjoint timer values
    ID3D11Query* m_timestampQueries[2][GPU_PROFILER_MARK_COUNT]; // Double-buffered GPU recorded clock values for each GPUProfilerMark
    float m_collectedTimes[GPU_PROFILER_MARK_COUNT - 1]; // Time between each non-beginning timestamp and the one before it in seconds
};

#endif // FSD_GPUPROFILER_H