// MoistureMath.h - pure math for soil-moisture / tank calibration and pump gating.
// Header-only-friendly helpers with no Arduino dependency so they can be unit
// tested on the host (see test/test_native_moisture_math/).

#pragma once

#include <stdint.h>

namespace plantwater {

// Linear map with clamping to the [out_min, out_max] range.
// Mirrors Arduino's map() but clamps at the endpoints and rejects a zero-width
// input range instead of dividing by zero.
int32_t mapClamped(int32_t x, int32_t in_min, int32_t in_max,
                   int32_t out_min, int32_t out_max);

// Convert raw ADC reading to soil-moisture percentage (0..100).
// Uses the project's calibration endpoints by default: ADC 2500 -> 0%,
// ADC 6000 -> 100% (see MOISTURE_ADC_ZERO_PCT / MOISTURE_ADC_FULL_PCT in main).
int adcToMoisturePct(int adcRaw, int adcZeroPct = 2500, int adcFullPct = 6000);

// Convert HC-SR04 distance (cm) to tank fill percentage (0..100).
// Full when distanceCm == fullCm, empty when distanceCm == emptyCm.
int distanceToTankPct(float distanceCm,
                      float fullCm = 1.0f, float emptyCm = 33.0f);

// True if the pump may fire given hoursSinceLast and the configured cooldown.
// Negative hoursSinceLast (clock skew / stale RTC) is treated as "not ready".
bool pumpCooldownElapsed(long hoursSinceLast, int cooldownHours);

// True if the tank has enough water to safely run the pump.
bool tankSafeToPump(int tankPct, int minPct);

// Exponential-moving-average step: given the current running mean and a new
// sample, return the updated mean weighted as if over `windowSize` samples.
// Cheaper than a ring buffer and won't overflow.
float rollingMean(float currentMean, float newSample, int windowSize);

} // namespace plantwater
