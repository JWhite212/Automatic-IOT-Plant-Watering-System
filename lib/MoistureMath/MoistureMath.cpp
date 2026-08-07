#include "MoistureMath.h"

namespace plantwater {

int32_t mapClamped(int32_t x, int32_t in_min, int32_t in_max,
                   int32_t out_min, int32_t out_max) {
    if (in_max == in_min) {
        return out_min;
    }
    if (x <= in_min) return out_min;
    if (x >= in_max) return out_max;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int adcToMoisturePct(int adcRaw, int adcZeroPct, int adcFullPct) {
    return static_cast<int>(mapClamped(adcRaw, adcZeroPct, adcFullPct, 0, 100));
}

int distanceToTankPct(float distanceCm, float fullCm, float emptyCm) {
    if (emptyCm <= fullCm) return 0;
    if (distanceCm <= fullCm) return 100;
    if (distanceCm >= emptyCm) return 0;
    float pct = (emptyCm - distanceCm) / (emptyCm - fullCm) * 100.0f;
    return static_cast<int>(pct);
}

bool pumpCooldownElapsed(long hoursSinceLast, int cooldownHours) {
    if (hoursSinceLast < 0) return false;
    return hoursSinceLast >= cooldownHours;
}

bool tankSafeToPump(int tankPct, int minPct) {
    return tankPct >= minPct;
}

float rollingMean(float currentMean, float newSample, int windowSize) {
    if (windowSize <= 1) return newSample;
    float alpha = 1.0f / static_cast<float>(windowSize);
    return currentMean + alpha * (newSample - currentMean);
}

} // namespace plantwater
