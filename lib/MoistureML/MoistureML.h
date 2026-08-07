// MoistureML.h - on-device logistic-regression forecast of "needs water soon".
//
// Trained offline by scripts/train_logistic_regression.py against the feature
// vector (moisturePct, temperatureC, humidityPct, log1p(luxRaw), hoursSinceWater)
// -> probability that soil moisture will drop below the pump threshold within
// MOISTURE_ML_HORIZON_HOURS (see MoistureMLModel.h).
//
// Coefficients are baked into MoistureMLModel.h so inference is one dot product
// plus a sigmoid - safe to call from the sensor loop on an ESP32.

#pragma once

namespace plantwater {

struct MoistureFeatures {
    float moisturePct;        // 0..100 (from adcToMoisturePct)
    float temperatureC;       // averaged DHT reading
    float humidityPct;        // 0..100
    float luxRaw;             // raw BH1750 reading; internally log1p'd
    float hoursSinceWater;    // (now - LastPumpUnixSeconds) / 3600
};

// Predicted probability (0..1) that watering will be needed within the model's
// horizon window. See MoistureMLModel.h for the trained horizon and threshold.
float predictNeedsWaterSoon(const MoistureFeatures& features);

// Convenience wrapper: true when the predicted probability exceeds the
// model's tuned decision threshold.
bool shouldPreemptivelyWater(const MoistureFeatures& features);

} // namespace plantwater
