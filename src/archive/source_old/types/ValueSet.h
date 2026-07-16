#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// ValueSet — freshness / validity envelope for a sensor group.
//
// Extracted from Inputs.h (sprint 047-001) so that PoseEstimate.h and other
// state-layer headers can include it without pulling in the full Inputs.h.
//
// lagMs      : expected sensor latency in ms; initialised from RobotConfig.
// lastUpdMs  : system time (ms) of the most recent valid reading.
// valid      : true once at least one reading has been received.
// ---------------------------------------------------------------------------
struct ValueSet {
    uint32_t lagMs;
    uint32_t lastUpdMs;
    bool     valid;
};
