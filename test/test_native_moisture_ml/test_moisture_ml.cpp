// Native Unity tests for plantwater::MoistureML.
// Run with:  pio test -e native

#include <unity.h>

#include "MoistureML.h"

using plantwater::MoistureFeatures;
using plantwater::predictNeedsWaterSoon;
using plantwater::shouldPreemptivelyWater;

namespace {

MoistureFeatures baseline() {
    return MoistureFeatures{
        /* moisturePct    */ 45.0f,
        /* temperatureC   */ 21.0f,
        /* humidityPct    */ 55.0f,
        /* luxRaw         */ 500.0f,
        /* hoursSinceWater*/ 30.0f,
    };
}

} // namespace

void setUp() {}
void tearDown() {}

// main.cpp includes only MoistureML.h but reads these constants, so the public
// header has to re-export them. Guards against the include regressing.
void test_model_constants_are_visible_via_public_header() {
    TEST_ASSERT_GREATER_THAN_INT(0, plantwater::MOISTURE_ML_HORIZON_HOURS);
    TEST_ASSERT_TRUE(plantwater::MOISTURE_ML_THRESHOLD > 0.0f &&
                     plantwater::MOISTURE_ML_THRESHOLD < 1.0f);
}

void test_prediction_is_a_valid_probability() {
    const float p = predictNeedsWaterSoon(baseline());
    TEST_ASSERT_TRUE(p >= 0.0f && p <= 1.0f);
}

void test_wet_soil_predicts_low_probability() {
    MoistureFeatures f = baseline();
    f.moisturePct = 95.0f;
    f.hoursSinceWater = 1.0f;
    TEST_ASSERT_TRUE(predictNeedsWaterSoon(f) < 0.2f);
    TEST_ASSERT_FALSE(shouldPreemptivelyWater(f));
}

void test_dry_hot_sunny_stale_predicts_high_probability() {
    MoistureFeatures f = baseline();
    f.moisturePct = 12.0f;
    f.temperatureC = 30.0f;
    f.humidityPct = 30.0f;
    f.luxRaw = 25000.0f;
    f.hoursSinceWater = 120.0f;
    TEST_ASSERT_TRUE(predictNeedsWaterSoon(f) > 0.8f);
    TEST_ASSERT_TRUE(shouldPreemptivelyWater(f));
}

void test_probability_is_monotonic_in_dryness() {
    MoistureFeatures wet = baseline();
    MoistureFeatures dry = baseline();
    wet.moisturePct = 70.0f;
    dry.moisturePct = 20.0f;
    TEST_ASSERT_TRUE(predictNeedsWaterSoon(dry) > predictNeedsWaterSoon(wet));
}

void test_probability_is_monotonic_in_time_since_water() {
    MoistureFeatures fresh = baseline();
    MoistureFeatures stale = baseline();
    fresh.hoursSinceWater = 2.0f;
    stale.hoursSinceWater = 96.0f;
    TEST_ASSERT_TRUE(predictNeedsWaterSoon(stale) > predictNeedsWaterSoon(fresh));
}

void test_probability_is_monotonic_in_temperature() {
    MoistureFeatures cool = baseline();
    MoistureFeatures hot  = baseline();
    cool.temperatureC = 15.0f;
    hot.temperatureC  = 32.0f;
    TEST_ASSERT_TRUE(predictNeedsWaterSoon(hot) > predictNeedsWaterSoon(cool));
}

void test_probability_is_monotonic_in_humidity_inverse() {
    MoistureFeatures humid = baseline();
    MoistureFeatures dryAir = baseline();
    humid.humidityPct = 85.0f;
    dryAir.humidityPct = 25.0f;
    TEST_ASSERT_TRUE(predictNeedsWaterSoon(dryAir) > predictNeedsWaterSoon(humid));
}

void test_negative_lux_is_treated_as_zero() {
    MoistureFeatures a = baseline();
    MoistureFeatures b = baseline();
    a.luxRaw = -10.0f;
    b.luxRaw = 0.0f;
    TEST_ASSERT_EQUAL_FLOAT(predictNeedsWaterSoon(b), predictNeedsWaterSoon(a));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_model_constants_are_visible_via_public_header);
    RUN_TEST(test_prediction_is_a_valid_probability);
    RUN_TEST(test_wet_soil_predicts_low_probability);
    RUN_TEST(test_dry_hot_sunny_stale_predicts_high_probability);
    RUN_TEST(test_probability_is_monotonic_in_dryness);
    RUN_TEST(test_probability_is_monotonic_in_time_since_water);
    RUN_TEST(test_probability_is_monotonic_in_temperature);
    RUN_TEST(test_probability_is_monotonic_in_humidity_inverse);
    RUN_TEST(test_negative_lux_is_treated_as_zero);
    return UNITY_END();
}
