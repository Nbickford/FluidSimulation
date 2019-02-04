//*****************************************************************
// GPUProfiler.cpp
// A query-based GPU profiler.
// Based on Nathan Reed's GPU Profiling 101 code: http://reedbeta.com/blog/gpu-profiling-101/
//
// Authors:
//   Neil Bickford
//*****************************************************************

#include "GPUProfiler.h"
#include "odprintf.h"

void GPUProfiler::Initialize(ID3D11Device* device) {
    // TODO(nbickford); Add HRESULT checks here
    D3D11_QUERY_DESC disjointQueryDesc{ D3D11_QUERY_TIMESTAMP_DISJOINT, NULL };

    D3D11_QUERY_DESC timestampQueryDesc{ D3D11_QUERY_TIMESTAMP, NULL };

    for (int frameIdx = 0; frameIdx < 2; ++frameIdx) {
        device->CreateQuery(&disjointQueryDesc, &m_disjointQueries[frameIdx]);

        for (int markIdx = 0; markIdx < GPU_PROFILER_MARK_COUNT; ++markIdx) {
            device->CreateQuery(&timestampQueryDesc, &m_timestampQueries[frameIdx][markIdx]);
        }
    }
}

void GPUProfiler::ReleaseResources() {
    for (int frameIdx = 0; frameIdx < 2; ++frameIdx) {
        ReleaseCOM(m_disjointQueries[frameIdx]);

        for (int markIdx = 0; markIdx < GPU_PROFILER_MARK_COUNT; ++markIdx) {
            ReleaseCOM(m_timestampQueries[frameIdx][markIdx]);
        }
    }
}

void GPUProfiler::BeginFrame(ID3D11DeviceContext* context) const {
    int captureFrame = (m_framesCollected % 2);
    context->Begin(m_disjointQueries[captureFrame]);
    TimestampComplete(context, GPU_PROFILER_MARK_BEGIN_FRAME);
}

void GPUProfiler::TimestampComplete(ID3D11DeviceContext* context, GPUProfilerMark mark) const {
    int captureFrame = (m_framesCollected % 2);
    context->End(m_timestampQueries[captureFrame][mark]);
}

void GPUProfiler::EndFrame(ID3D11DeviceContext* context, bool skipLastFrameCollectBlock) {
    int captureFrame = (m_framesCollected % 2);
    TimestampComplete(context, GPU_PROFILER_MARK_END_FRAME);
    context->End(m_disjointQueries[captureFrame]);

    // Do we have a double-buffered frame we've captured timing data from?
    if (m_framesCollected <= 0) {
        m_framesCollected = 1;
        return;
    }

    // Try to collect last frame's timing data
    int otherFrame = ((m_framesCollected + 1) % 2);
    D3D11_QUERY_DATA_TIMESTAMP_DISJOINT tsDisjoint;
    while ((context->GetData(m_disjointQueries[otherFrame], &tsDisjoint, sizeof(tsDisjoint), 0) == S_FALSE) && (!skipLastFrameCollectBlock)) {
        // Block until we get our timing data (!)
        Sleep(1); // in milliseconds
    }

    // Were timestamps disjoint during our previous frame? If so, don't update statistics
    if (tsDisjoint.Disjoint) {
        return;
    }

    UINT64 timestamps[GPU_PROFILER_MARK_COUNT];
    for (int markIdx = 0; markIdx < GPU_PROFILER_MARK_COUNT; ++markIdx) {
        context->GetData(m_timestampQueries[otherFrame][markIdx], &timestamps[markIdx], sizeof(UINT64), 0);
    }

    float frameTimerFrequency = static_cast<float>(tsDisjoint.Frequency);
    for (int markIdx = 1; markIdx < GPU_PROFILER_MARK_COUNT; ++markIdx) {
        m_collectedTimes[markIdx - 1] = static_cast<float>(timestamps[markIdx] - timestamps[markIdx - 1]) / frameTimerFrequency;
    }

    m_framesCollected++; // Next frame
}

float GPUProfiler::DT(GPUProfilerMark mark) const {
    return m_collectedTimes[mark - 1];
}