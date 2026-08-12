#include <errno.h>
#include <stdint.h>

#include <zephyr/ztest.h>

#include "app_config.h"
#include "battery_math.h"
#include "encoder_math.h"

ZTEST_SUITE(encoder_math, NULL, NULL, NULL, NULL, NULL);

ZTEST(encoder_math, test_adc_normalization)
{
	zassert_equal(encoder_adc_normalize(ENCODER_VOUT_MIN_CODE), 0U);
	zassert_equal(encoder_adc_normalize(ENCODER_VOUT_MAX_CODE),
		      ENCODER_ADC_MAX_CODE);
	zassert_within(encoder_adc_normalize(ENCODER_ADC_HALF_TURN),
		       ENCODER_ADC_HALF_TURN, 1U);
}

ZTEST(encoder_math, test_circular_delta)
{
	zassert_equal(encoder_angle_delta(2U, 4094U), 4);
	zassert_equal(encoder_angle_delta(4094U, 2U), -4);
	zassert_equal(encoder_angle_delta(1200U, 1000U), 200);
	zassert_equal(encoder_angle_delta(1000U, 1200U), -200);
}

ZTEST(encoder_math, test_filter_rejects_outliers)
{
	const uint16_t samples[ENCODER_FILTER_SAMPLE_COUNT] = {
		1000U, 1001U, 999U, 1002U, 998U, 1000U, 500U, 1500U,
	};
	uint16_t angle = 0U;

	zassert_ok(encoder_angle_filter(samples, &angle));
	zassert_within(angle, 1000U, 2U);
}

ZTEST(encoder_math, test_filter_crosses_zero_without_half_turn_glitch)
{
	const uint16_t samples[ENCODER_FILTER_SAMPLE_COUNT] = {
		4094U, 4095U, 0U, 1U, 4093U, 2U, 4095U, 0U,
	};
	uint16_t angle = ENCODER_ADC_HALF_TURN;

	zassert_ok(encoder_angle_filter(samples, &angle));
	zassert_true(angle >= 4092U || angle <= 3U,
		     "filtered wrap angle was %u", angle);
}

ZTEST(encoder_math, test_filter_validates_arguments)
{
	uint16_t angle;
	const uint16_t samples[ENCODER_FILTER_SAMPLE_COUNT] = { 0U };

	zassert_equal(encoder_angle_filter(NULL, &angle), -EINVAL);
	zassert_equal(encoder_angle_filter(samples, NULL), -EINVAL);
}

ZTEST(encoder_math, test_scroll_accumulator_preserves_remainder)
{
	int32_t remainder = 0;
	const int16_t direction = ENCODER_SCROLL_DIRECTION;
	const int16_t almost_one_step = ENCODER_COUNTS_PER_SCROLL_STEP - 1;

	zassert_equal(encoder_scroll_accumulate(almost_one_step, &remainder), 0);
	zassert_equal(remainder, almost_one_step * direction);
	zassert_equal(encoder_scroll_accumulate(1, &remainder), direction);
	zassert_equal(remainder, 0);
	zassert_equal(encoder_scroll_accumulate(
		-(int16_t)(ENCODER_COUNTS_PER_SCROLL_STEP + 1), &remainder),
		-direction);
	zassert_equal(remainder, -direction);
}

ZTEST(encoder_math, test_one_detent_produces_all_high_resolution_units)
{
	int32_t remainder = 0;

	zassert_equal(encoder_scroll_accumulate(
		ENCODER_COUNTS_PER_SCROLL_DETENT, &remainder),
		ENCODER_SCROLL_DIRECTION * BLE_HID_WHEEL_RESOLUTION_MULTIPLIER);
	zassert_equal(remainder, 0);
}

ZTEST(encoder_math, test_negative_high_resolution_step_stays_signed)
{
	int32_t remainder = 0;

	zassert_equal(encoder_scroll_accumulate(
		-(int16_t)ENCODER_COUNTS_PER_SCROLL_STEP, &remainder),
		-ENCODER_SCROLL_DIRECTION);
	zassert_equal(remainder, 0);
}

ZTEST(encoder_math, test_battery_voltage_table)
{
	zassert_equal(battery_voltage_to_percent(3200), 0U);
	zassert_equal(battery_voltage_to_percent(3300), 0U);
	zassert_equal(battery_voltage_to_percent(3500), 25U);
	zassert_equal(battery_voltage_to_percent(3700), 50U);
	zassert_equal(battery_voltage_to_percent(3950), 75U);
	zassert_equal(battery_voltage_to_percent(4200), 100U);
	zassert_equal(battery_voltage_to_percent(4300), 100U);
}
