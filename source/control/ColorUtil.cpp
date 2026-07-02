// ColorUtil.cpp — RGBC → HSV color-space conversion.
//
// Extracted verbatim from StopCondition.cpp (CR-15 item 7, sprint 066);
// resolves the existing FIXME at StopCondition.cpp:27 ("Why is there a
// color function in the StopCondition module?"). Behavior-preserving move —
// the formula is unchanged. StopCondition::evaluate()'s Kind::COLOR branch
// calls this.

#include "ColorUtil.h"

void rgbToHSV(uint16_t rRaw, uint16_t gRaw, uint16_t bRaw, uint16_t cRaw,
              float& h, float& s, float& v)
{
    if (cRaw == 0) { h = 0.0f; s = 0.0f; v = 0.0f; return; }
    float r = (float)rRaw / (float)cRaw;
    float g = (float)gRaw / (float)cRaw;
    float b = (float)bRaw / (float)cRaw;
    // Clamp to [0,1].
    if (r > 1.0f) r = 1.0f;
    if (g > 1.0f) g = 1.0f;
    if (b > 1.0f) b = 1.0f;

    float cmax = r; if (g > cmax) cmax = g; if (b > cmax) cmax = b;
    float cmin = r; if (g < cmin) cmin = g; if (b < cmin) cmin = b;
    float delta = cmax - cmin;
    v = cmax;
    s = (cmax > 0.0f) ? (delta / cmax) : 0.0f;
    if (delta < 1e-6f) {
        h = 0.0f;
        return;
    }
    if (cmax == r) {
        h = 60.0f * ((g - b) / delta);
    } else if (cmax == g) {
        h = 60.0f * (((b - r) / delta) + 2.0f);
    } else {
        h = 60.0f * (((r - g) / delta) + 4.0f);
    }
    if (h < 0.0f) h += 360.0f;
}
