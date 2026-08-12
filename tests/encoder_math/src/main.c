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
	const int16_t counts_per_step = ENCODER_COUNTS_PER_SCROLL_DETENT /
		BLE_HID_WHEEL_RESOLUTION_MULTIPLIER;
	const int16_t almost_one_step = counts_per_step - 1;

	zassert_equal(encoder_scroll_accumulate(almost_one_step,
		BLE_HID_WHEEL_RESOLUTION_MULTIPLIER, &remainder), 0);
	zassert_equal(remainder, almost_one_step * direction);
	zassert_equal(encoder_scroll_accumulate(1,
		BLE_HID_WHEEL_RESOLUTION_MULTIPLIER, &remainder), direction);
	zassert_equal(remainder, 0);
	zassert_equal(encoder_scroll_accumulate(
		-(int16_t)(counts_per_step + 1),
		BLE_HID_WHEEL_RESOLUTION_MULTIPLIER, &remainder),
		-direction);
	zassert_equal(remainder, -direction);
}

ZTEST(encoder_math, test_one_detent_produces_all_high_resolution_units)
{
	int32_t remainder = 0;

	zassert_equal(encoder_scroll_accumulate(
		ENCODER_COUNTS_PER_SCROLL_DETENT,
		BLE_HID_WHEEL_RESOLUTION_MULTIPLIER, &remainder),
		ENCODER_SCROLL_DIRECTION * BLE_HID_WHEEL_RESOLUTION_MULTIPLIER);
	zassert_equal(remainder, 0);
}

ZTEST(encoder_math, test_negative_high_resolution_step_stays_signed)
{
	int32_t remainder = 0;

	zassert_equal(encoder_scroll_accumulate(
		-(int16_t)(ENCODER_COUNTS_PER_SCROLL_DETENT /
			BLE_HID_WHEEL_RESOLUTION_MULTIPLIER),
		BLE_HID_WHEEL_RESOLUTION_MULTIPLIER, &remainder),
		-ENCODER_SCROLL_DIRECTION);
	zassert_equal(remainder, 0);
}

ZTEST(encoder_math, test_runtime_resolution_modes_preserve_distance)
{
	const uint8_t multipliers[] = { 1U, 8U, 16U };

	for (size_t index = 0U; index < ARRAY_SIZE(multipliers); ++index) {
		int32_t remainder = 0;

		zassert_equal(encoder_scroll_accumulate(
			ENCODER_COUNTS_PER_SCROLL_DETENT, multipliers[index],
			&remainder),
			ENCODER_SCROLL_DIRECTION *
				BLE_HID_WHEEL_RESOLUTION_MULTIPLIER);
		zassert_equal(remainder, 0);
	}
}

ZTEST(encoder_math, test_runtime_resolution_modes_change_granularity)
{
	int32_t remainder = 0;

	zassert_equal(encoder_scroll_accumulate(63, 1U, &remainder), 0);
	zassert_equal(encoder_scroll_accumulate(1, 1U, &remainder),
		ENCODER_SCROLL_DIRECTION * 16);
	zassert_equal(remainder, 0);

	zassert_equal(encoder_scroll_accumulate(7, 8U, &remainder), 0);
	zassert_equal(encoder_scroll_accumulate(1, 8U, &remainder),
		ENCODER_SCROLL_DIRECTION * 2);
	zassert_equal(remainder, 0);

	zassert_equal(encoder_scroll_accumulate(3, 16U, &remainder), 0);
	zassert_equal(encoder_scroll_accumulate(1, 16U, &remainder),
		ENCODER_SCROLL_DIRECTION);
	zassert_equal(remainder, 0);
}

ZTEST(encoder_math, test_invalid_runtime_resolution_is_rejected)
{
	int32_t remainder = 7;

	zassert_equal(encoder_scroll_accumulate(32, 0U, &remainder), 0);
	zassert_equal(remainder, 7);
	zassert_equal(encoder_scroll_accumulate(32, 3U, &remainder), 0);
	zassert_equal(remainder, 7);
}

ZTEST(encoder_math, test_acceleration_is_identity_when_disabled)
{
	struct encoder_acceleration_state state;
	const int16_t deltas[] = { 7, 25, -11, 0, 127 };

	encoder_acceleration_reset(&state, false);
	for (size_t index = 0U; index < ARRAY_SIZE(deltas); ++index) {
		zassert_equal(encoder_acceleration_apply(
			&state, deltas[index], false, true), deltas[index]);
		zassert_equal(state.factor_q8, ENCODER_ACCELERATION_Q8_ONE);
	}
}

ZTEST(encoder_math, test_acceleration_starts_after_45_degrees)
{
	struct encoder_acceleration_state state;

	encoder_acceleration_reset(&state, true);
	zassert_equal(encoder_acceleration_apply(&state, 500, true, true), 500);
	zassert_equal(encoder_acceleration_apply(&state, 12, true, true), 12);
	zassert_equal(state.activation_counts,
		      ENCODER_ACCELERATION_ACTIVATION_COUNTS);
	zassert_equal(state.factor_q8, ENCODER_ACCELERATION_Q8_ONE);
	zassert_true(encoder_acceleration_apply(&state, 12, true, true) > 12);
	zassert_true(state.factor_q8 > ENCODER_ACCELERATION_Q8_ONE);
}

ZTEST(encoder_math, test_acceleration_curve_linear_midpoint_and_cap)
{
	const uint32_t midpoint =
		(ENCODER_ACCELERATION_SPEED_MIN_COUNTS_S +
		 ENCODER_ACCELERATION_SPEED_MAX_COUNTS_S) / 2U;

	zassert_equal(encoder_acceleration_factor_q8(
		ENCODER_ACCELERATION_SPEED_MIN_COUNTS_S),
		ENCODER_ACCELERATION_Q8_ONE);
	zassert_within(encoder_acceleration_factor_q8(midpoint),
		       (ENCODER_ACCELERATION_Q8_ONE +
			ENCODER_ACCELERATION_Q8_MAX) / 2U, 1U);
	zassert_equal(encoder_acceleration_factor_q8(
		ENCODER_ACCELERATION_SPEED_MAX_COUNTS_S),
		ENCODER_ACCELERATION_Q8_MAX);
	zassert_equal(encoder_acceleration_factor_q8(UINT32_MAX),
		      ENCODER_ACCELERATION_Q8_MAX);
}

ZTEST(encoder_math, test_acceleration_fixed_point_remainder_is_symmetric)
{
	int32_t positive_remainder = 0;
	int32_t negative_remainder = 0;
	const uint16_t factor_q8 = ENCODER_ACCELERATION_Q8_ONE +
		(ENCODER_ACCELERATION_Q8_ONE / 2U);

	zassert_equal(encoder_acceleration_scale_q8(
		1, factor_q8, &positive_remainder), 1);
	zassert_equal(positive_remainder, 128);
	zassert_equal(encoder_acceleration_scale_q8(
		1, factor_q8, &positive_remainder), 2);
	zassert_equal(positive_remainder, 0);

	zassert_equal(encoder_acceleration_scale_q8(
		-1, factor_q8, &negative_remainder), -1);
	zassert_equal(negative_remainder, -128);
	zassert_equal(encoder_acceleration_scale_q8(
		-1, factor_q8, &negative_remainder), -2);
	zassert_equal(negative_remainder, 0);
}

ZTEST(encoder_math, test_acceleration_output_is_symmetric_by_direction)
{
	struct encoder_acceleration_state positive;
	struct encoder_acceleration_state negative;

	encoder_acceleration_reset(&positive, true);
	encoder_acceleration_reset(&negative, true);
	for (size_t sample = 0U; sample < 25U; ++sample) {
		const int32_t positive_delta = encoder_acceleration_apply(
			&positive, 32, true, true);
		const int32_t negative_delta = encoder_acceleration_apply(
			&negative, -32, true, true);

		zassert_equal(positive_delta, -negative_delta);
		zassert_equal(positive.factor_q8, negative.factor_q8);
	}
}

ZTEST(encoder_math, test_acceleration_resets_on_reverse)
{
	struct encoder_acceleration_state state;

	encoder_acceleration_reset(&state, true);
	for (size_t sample = 0U; sample < 20U; ++sample) {
		(void)encoder_acceleration_apply(&state, 40, true, true);
	}
	zassert_true(state.factor_q8 > ENCODER_ACCELERATION_Q8_ONE);
	zassert_equal(encoder_acceleration_apply(&state, -17, true, true), -17);
	zassert_equal(state.factor_q8, ENCODER_ACCELERATION_Q8_ONE);
	zassert_equal(state.direction, -1);
	zassert_equal(state.activation_counts, 17U);
	zassert_equal(state.fractional_remainder_q8, 0);
}

ZTEST(encoder_math, test_acceleration_resets_after_pause_and_skips_first_delta)
{
	struct encoder_acceleration_state state;
	const size_t idle_samples = ENCODER_ACCELERATION_IDLE_RESET_MS /
		ENCODER_ACTIVE_POLL_PERIOD_MS;

	encoder_acceleration_reset(&state, true);
	for (size_t sample = 0U; sample < 20U; ++sample) {
		(void)encoder_acceleration_apply(&state, 40, true, true);
	}
	for (size_t sample = 0U; sample < idle_samples; ++sample) {
		zassert_equal(encoder_acceleration_apply(
			&state, 0, true, true), 0);
	}
	zassert_equal(state.factor_q8, ENCODER_ACCELERATION_Q8_ONE);
	zassert_equal(state.speed_counts_per_s, 0U);
	zassert_equal(state.direction, 0);
	zassert_equal(state.activation_counts, 0U);

	zassert_equal(encoder_acceleration_apply(&state, 100, true, true), 100);
	zassert_equal(state.direction, 0);
	zassert_equal(state.activation_counts, 0U);
	zassert_equal(encoder_acceleration_apply(&state, 10, true, true), 10);
	zassert_equal(state.direction, 1);
	zassert_equal(state.activation_counts, 10U);
}

ZTEST(encoder_math, test_acceleration_state_change_restarts_at_x1)
{
	struct encoder_acceleration_state state;

	encoder_acceleration_reset(&state, true);
	for (size_t sample = 0U; sample < 20U; ++sample) {
		(void)encoder_acceleration_apply(&state, 40, true, true);
	}
	zassert_true(state.factor_q8 > ENCODER_ACCELERATION_Q8_ONE);
	zassert_equal(encoder_acceleration_apply(&state, 23, false, true), 23);
	zassert_equal(state.factor_q8, ENCODER_ACCELERATION_Q8_ONE);
	zassert_equal(state.activation_counts, 0U);
	zassert_equal(encoder_acceleration_apply(&state, 23, true, true), 23);
	zassert_equal(state.activation_counts, 23U);
}

ZTEST(encoder_math, test_idle_samples_produce_no_scroll_tail)
{
	struct encoder_acceleration_state state;
	int32_t scroll_remainder = 0;

	encoder_acceleration_reset(&state, true);
	for (size_t sample = 0U; sample < 80U; ++sample) {
		const int32_t accelerated_delta = encoder_acceleration_apply(
			&state, 0, true, true);

		zassert_equal(accelerated_delta, 0);
		zassert_equal(encoder_scroll_accumulate(
			accelerated_delta,
			BLE_HID_WHEEL_RESOLUTION_MULTIPLIER,
			&scroll_remainder), 0);
	}
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
