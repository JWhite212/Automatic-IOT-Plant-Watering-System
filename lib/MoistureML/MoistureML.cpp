#include "MoistureML.h"
#include "MoistureMLModel.h"

#include <math.h>

namespace plantwater {

namespace {

inline float sigmoid(float z) {
    // Clamp extreme inputs so expf() never overflows the float range.
    if (z >= 20.0f)  return 1.0f;
    if (z <= -20.0f) return 0.0f;
    return 1.0f / (1.0f + expf(-z));
}

inline float standardise(float x, float mean, float std) {
    if (std <= 0.0f) return 0.0f;
    return (x - mean) / std;
}

} // namespace

float predictNeedsWaterSoon(const MoistureFeatures& f) {
    const float lux = f.luxRaw < 0.0f ? 0.0f : f.luxRaw;
    const float feats[5] = {
        f.moisturePct,
        f.temperatureC,
        f.humidityPct,
        log1pf(lux),
        f.hoursSinceWater
    };

    float z = MOISTURE_ML_INTERCEPT;
    for (int i = 0; i < 5; ++i) {
        const float xn = standardise(feats[i], MOISTURE_ML_FEATURE_MEAN[i],
                                                MOISTURE_ML_FEATURE_STD[i]);
        z += MOISTURE_ML_COEF[i] * xn;
    }
    return sigmoid(z);
}

bool shouldPreemptivelyWater(const MoistureFeatures& f) {
    return predictNeedsWaterSoon(f) >= MOISTURE_ML_THRESHOLD;
}

} // namespace plantwater
