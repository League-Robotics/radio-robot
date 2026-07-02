#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// rgbToHSV — convert RGBC raw sensor values to HSV floats.
//
// R,G,B are raw uint16_t sensor counts. We normalise by C (clear/ambient)
// to get [0,1] floating-point channels, then convert to HSV.
//
// h returned in [0, 360); s,v in [0, 1].
// If C == 0 (dark), returns h=0, s=0, v=0.
//
// Moved out of StopCondition.cpp (CR-15 item 7, sprint 066) — a general
// color-space conversion is not a stop-condition concern; this is its home
// for any future caller (e.g. a COLOR command, a diagnostics readout).
// ---------------------------------------------------------------------------
void rgbToHSV(uint16_t rRaw, uint16_t gRaw, uint16_t bRaw, uint16_t cRaw,
              float& h, float& s, float& v);
