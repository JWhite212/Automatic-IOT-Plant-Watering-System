// Native Unity tests for plantwater::MoistureMath.
// Run with:  pio test -e native

#include <unity.h>

#include "MoistureMath.h"

using plantwater::adcToMoisturePct;
using plantwater::distanceToTankPct;
using plantwater::mapClamped;
using plantwater::pumpCooldownElapsed;
using plantwater::rollingMean;
using plantwater::tankSafeToPump;

void setUp() {}
void tearDown() {}

void test_map_clamped_endpoints() {
    TEST_ASSERT_EQUAL_INT32(0,   mapClamped(0,   0, 100, 0, 100));
    TEST_ASSERT_EQUAL_INT32(100, mapClamped(100, 0, 100, 0, 100));
    TEST_ASSERT_EQUAL_INT32(50,  mapClamped(50,  0, 100, 0, 100));
}

void test_map_clamped_clamps_out_of_range() {
    TEST_ASSERT_EQUAL_INT32(0,   mapClamped(-99, 0, 100, 0, 100));
    TEST_ASSERT_EQUAL_INT32(100, mapClamped(999, 0, 100, 0, 100));
}

void test_map_clamped_zero_width_input_is_safe() {
    TEST_ASSERT_EQUAL_INT32(42, mapClamped(50, 100, 100, 42, 88));
}

void test_adc_to_moisture_pct_uses_project_calibration() {
    // Endpoints from src/main.cpp: 2500 -> 0%, 6000 -> 100%.
    TEST_ASSERT_EQUAL_INT(0,   adcToMoisturePct(2500));
    TEST_ASSERT_EQUAL_INT(100, adcToMoisturePct(6000));
    TEST_ASSERT_EQUAL_INT(50,  adcToMoisturePct(4250));
    TEST_ASSERT_EQUAL_INT(0,   adcToMoisturePct(1000));   // clamps low
    TEST_ASSERT_EQUAL_INT(100, adcToMoisturePct(9000));   // clamps high
}

void test_distance_to_tank_pct_uses_project_calibration() {
    // Endpoints from src/main.cpp: 1cm -> 100% full, 33cm -> 0%.
    TEST_ASSERT_EQUAL_INT(100, distanceToTankPct(1.0f));
    TEST_ASSERT_EQUAL_INT(0,   distanceToTankPct(33.0f));
    TEST_ASSERT_EQUAL_INT(50,  distanceToTankPct(17.0f));
    TEST_ASSERT_EQUAL_INT(100, distanceToTankPct(0.5f));  // sensor closer than full
    TEST_ASSERT_EQUAL_INT(0,   distanceToTankPct(99.0f)); // beyond empty
}

void test_pump_cooldown_elapsed() {
    TEST_ASSERT_FALSE(pumpCooldownElapsed(11, 12)); // just short of cooldown
    TEST_ASSERT_TRUE(pumpCooldownElapsed(12, 12));  // exactly at cooldown
    TEST_ASSERT_TRUE(pumpCooldownElapsed(48, 12));  // long past cooldown
    TEST_ASSERT_FALSE(pumpCooldownElapsed(-5, 12)); // clock skew guard
}

void test_tank_safe_to_pump() {
    TEST_ASSERT_FALSE(tankSafeToPump(4, 5));
    TEST_ASSERT_TRUE(tankSafeToPump(5, 5));
    TEST_ASSERT_TRUE(tankSafeToPump(80, 5));
}

void test_rolling_mean_converges_to_signal() {
    float mean = 0.0f;
    for (int i = 0; i < 500; ++i) {
        mean = rollingMean(mean, 42.0f, 20);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 42.0f, mean);
}

void test_rolling_mean_single_sample_window_passes_through() {
    TEST_ASSERT_EQUAL_FLOAT(9.0f, rollingMean(3.0f, 9.0f, 1));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_map_clamped_endpoints);
    RUN_TEST(test_map_clamped_clamps_out_of_range);
    RUN_TEST(test_map_clamped_zero_width_input_is_safe);
    RUN_TEST(test_adc_to_moisture_pct_uses_project_calibration);
    RUN_TEST(test_distance_to_tank_pct_uses_project_calibration);
    RUN_TEST(test_pump_cooldown_elapsed);
    RUN_TEST(test_tank_safe_to_pump);
    RUN_TEST(test_rolling_mean_converges_to_signal);
    RUN_TEST(test_rolling_mean_single_sample_window_passes_through);
    return UNITY_END();
}
