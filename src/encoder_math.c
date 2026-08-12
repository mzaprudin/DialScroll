#include "encoder_math.h"

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/sys/util.h>

BUILD_ASSERT(ENCODER_VOUT_MIN_CODE < ENCODER_VOUT_MAX_CODE,
	     "VOUT calibration range must be positive");
BUILD_ASSERT(ENCODER_VOUT_MAX_CODE <= ENCODER_ADC_MAX_CODE,
	     "VOUT calibration range must fit the ADC");
BUILD_ASSERT(ENCODER_FILTER_SAMPLE_COUNT > (2U * ENCODER_FILTER_DISCARD_COUNT),
	     "The trimmed mean must retain at least one ADC sample");
BUILD_ASSERT((ENCODER_COUNTS_PER_SCROLL_DETENT %
	      BLE_HID_WHEEL_RESOLUTION_MULTIPLIER) == 0,
	     "A Wheel detent must contain a whole number of high-resolution units");
BUILD_ASSERT((ENCODER_SCROLL_DIRECTION == 1) || (ENCODER_SCROLL_DIRECTION == -1),
	     "Encoder direction must be +1 or -1");
BUILD_ASSERT((ENCODER_ACCELERATION_BUCKET_MS %
	      ENCODER_ACTIVE_POLL_PERIOD_MS) == 0U,
	     "Acceleration buckets must contain whole active samples");
BUILD_ASSERT(ENCODER_ACCELERATION_Q8_ONE == 256U,
	     "Acceleration fixed point must use Q8");
BUILD_ASSERT(ENCODER_ACCELERATION_Q8_MAX ==
	     (2U * ENCODER_ACCELERATION_Q8_ONE),
	     "Acceleration maximum must be x2");

static int int32_compare(const void *left, const void *right)
{
	const int32_t left_value = *(const int32_t *)left;
	const int32_t right_value = *(const int32_t *)right;

	return (left_value > right_value) - (left_value < right_value);
}

uint16_t encoder_adc_normalize(uint16_t raw_sample)
{
	const uint32_t span = ENCODER_VOUT_MAX_CODE - ENCODER_VOUT_MIN_CODE;
	uint32_t offset;

	if (raw_sample <= ENCODER_VOUT_MIN_CODE) {
		return 0U;
	}
	if (raw_sample >= ENCODER_VOUT_MAX_CODE) {
		return ENCODER_ADC_MAX_CODE;
	}

	offset = raw_sample - ENCODER_VOUT_MIN_CODE;
	return (uint16_t)((offset * ENCODER_ADC_MAX_CODE + (span / 2U)) / span);
}

int16_t encoder_angle_delta(uint16_t current, uint16_t previous)
{
	int32_t delta = (int32_t)current - (int32_t)previous;

	if (delta > (int32_t)ENCODER_ADC_HALF_TURN) {
		delta -= (int32_t)ENCODER_ADC_MODULUS;
	} else if (delta < -(int32_t)ENCODER_ADC_HALF_TURN) {
		delta += (int32_t)ENCODER_ADC_MODULUS;
	}

	return (int16_t)delta;
}

int encoder_angle_filter(const uint16_t samples[ENCODER_FILTER_SAMPLE_COUNT],
			 uint16_t *angle)
{
	int32_t unwrapped[ENCODER_FILTER_SAMPLE_COUNT];
	uint16_t reference;
	int32_t sum = 0;
	int32_t mean;

	if (samples == NULL || angle == NULL) {
		return -EINVAL;
	}
	reference = encoder_adc_normalize(samples[0]);

	for (size_t index = 0U; index < ARRAY_SIZE(unwrapped); ++index) {
		const uint16_t normalized = encoder_adc_normalize(samples[index]);

		unwrapped[index] = (int32_t)reference +
			encoder_angle_delta(normalized, reference);
	}

	qsort(unwrapped, ARRAY_SIZE(unwrapped), sizeof(unwrapped[0]), int32_compare);

	for (size_t index = ENCODER_FILTER_DISCARD_COUNT;
	     index < ARRAY_SIZE(unwrapped) - ENCODER_FILTER_DISCARD_COUNT;
	     ++index) {
		sum += unwrapped[index];
	}

	mean = sum / (int32_t)(ARRAY_SIZE(unwrapped) -
		(2U * ENCODER_FILTER_DISCARD_COUNT));
	mean %= (int32_t)ENCODER_ADC_MODULUS;
	if (mean < 0) {
		mean += (int32_t)ENCODER_ADC_MODULUS;
	}

	*angle = (uint16_t)mean;
	return 0;
}

int32_t encoder_scroll_accumulate(int32_t delta, uint8_t resolution_multiplier,
				  int32_t *remainder)
{
	const int32_t directed_delta = (int32_t)delta * ENCODER_SCROLL_DIRECTION;
	int32_t counts_per_step;
	int32_t report_units_per_step;
	int32_t scroll_delta;

	if ((remainder == NULL) || (resolution_multiplier == 0U) ||
	    ((BLE_HID_WHEEL_RESOLUTION_MULTIPLIER % resolution_multiplier) != 0U) ||
	    ((ENCODER_COUNTS_PER_SCROLL_DETENT % resolution_multiplier) != 0U)) {
		return 0;
	}
	counts_per_step = ENCODER_COUNTS_PER_SCROLL_DETENT / resolution_multiplier;
	report_units_per_step =
		BLE_HID_WHEEL_RESOLUTION_MULTIPLIER / resolution_multiplier;

	*remainder += directed_delta;
	scroll_delta = *remainder / counts_per_step;
	*remainder -= scroll_delta * counts_per_step;

	/* The HID descriptor always advertises x16 as its maximum. Lower runtime
	 * modes preserve total distance by emitting coarser chunks: x8 emits two
	 * x16 units and legacy/off emits sixteen x16 units at a time.
	 */
	return scroll_delta * report_units_per_step;
}

void encoder_acceleration_reset(struct encoder_acceleration_state *state,
				bool enabled)
{
	if (state == NULL) {
		return;
	}

	memset(state, 0, sizeof(*state));
	state->enabled = enabled;
	state->factor_q8 = ENCODER_ACCELERATION_Q8_ONE;
}

uint16_t encoder_acceleration_factor_q8(uint32_t speed_counts_per_s)
{
	const uint32_t speed_span = ENCODER_ACCELERATION_SPEED_MAX_COUNTS_S -
		ENCODER_ACCELERATION_SPEED_MIN_COUNTS_S;
	uint32_t factor;

	if (speed_counts_per_s <= ENCODER_ACCELERATION_SPEED_MIN_COUNTS_S) {
		return ENCODER_ACCELERATION_Q8_ONE;
	}
	if (speed_counts_per_s >= ENCODER_ACCELERATION_SPEED_MAX_COUNTS_S) {
		return ENCODER_ACCELERATION_Q8_MAX;
	}

	factor = ENCODER_ACCELERATION_Q8_ONE +
		(((speed_counts_per_s -
		   ENCODER_ACCELERATION_SPEED_MIN_COUNTS_S) *
		  ENCODER_ACCELERATION_Q8_ONE) / speed_span);
	return (uint16_t)factor;
}

int32_t encoder_acceleration_scale_q8(int32_t delta, uint16_t factor_q8,
				      int32_t *fractional_remainder_q8)
{
	int32_t scaled;
	int32_t result;

	if ((fractional_remainder_q8 == NULL) ||
	    (factor_q8 < ENCODER_ACCELERATION_Q8_ONE) ||
	    (factor_q8 > ENCODER_ACCELERATION_Q8_MAX)) {
		return delta;
	}

	scaled = delta * (int32_t)factor_q8 + *fractional_remainder_q8;
	result = scaled / (int32_t)ENCODER_ACCELERATION_Q8_ONE;
	*fractional_remainder_q8 = scaled -
		(result * (int32_t)ENCODER_ACCELERATION_Q8_ONE);
	return result;
}

static void encoder_acceleration_window_prepare(
	struct encoder_acceleration_state *state)
{
	const uint8_t samples_per_bucket = ENCODER_ACCELERATION_BUCKET_MS /
		ENCODER_ACTIVE_POLL_PERIOD_MS;

	if (state->samples_in_bucket < samples_per_bucket) {
		return;
	}

	state->bucket_index = (uint8_t)((state->bucket_index + 1U) %
		ENCODER_ACCELERATION_BUCKET_COUNT);
	state->window_counts -= state->buckets[state->bucket_index];
	state->buckets[state->bucket_index] = 0U;
	state->samples_in_bucket = 0U;
}

static void encoder_acceleration_speed_update(
	struct encoder_acceleration_state *state)
{
	state->speed_counts_per_s =
		(state->window_counts * 1000U +
		 (ENCODER_ACCELERATION_WINDOW_MS / 2U)) /
		ENCODER_ACCELERATION_WINDOW_MS;
}

int32_t encoder_acceleration_apply(struct encoder_acceleration_state *state,
				   int16_t delta, bool enabled,
				   bool include_in_estimate)
{
	uint32_t magnitude;
	int8_t direction;
	int32_t accelerated_delta;

	if (state == NULL) {
		return delta;
	}

	if ((state->enabled != enabled) || !include_in_estimate) {
		encoder_acceleration_reset(state, enabled);
		if (!include_in_estimate) {
			return delta;
		}
	}

	if (!enabled) {
		return delta;
	}

	if (delta == 0) {
		state->idle_ms += ENCODER_ACTIVE_POLL_PERIOD_MS;
		if (state->idle_ms >= ENCODER_ACCELERATION_IDLE_RESET_MS) {
			encoder_acceleration_reset(state, true);
			state->skip_next_motion = true;
			return 0;
		}

		encoder_acceleration_window_prepare(state);
		encoder_acceleration_speed_update(state);
		state->factor_q8 = state->activation_counts >=
			ENCODER_ACCELERATION_ACTIVATION_COUNTS ?
			encoder_acceleration_factor_q8(state->speed_counts_per_s) :
			ENCODER_ACCELERATION_Q8_ONE;
		++state->samples_in_bucket;
		return 0;
	}
	if (state->skip_next_motion) {
		encoder_acceleration_reset(state, true);
		return delta;
	}

	direction = delta > 0 ? 1 : -1;
	if ((state->direction != 0) && (state->direction != direction)) {
		encoder_acceleration_reset(state, true);
	}
	state->direction = direction;
	state->idle_ms = 0U;

	encoder_acceleration_window_prepare(state);
	encoder_acceleration_speed_update(state);
	state->factor_q8 = state->activation_counts >=
		ENCODER_ACCELERATION_ACTIVATION_COUNTS ?
		encoder_acceleration_factor_q8(state->speed_counts_per_s) :
		ENCODER_ACCELERATION_Q8_ONE;
	accelerated_delta = encoder_acceleration_scale_q8(
		delta, state->factor_q8, &state->fractional_remainder_q8);

	magnitude = delta < 0 ? (uint32_t)(-(int32_t)delta) : (uint32_t)delta;
	state->buckets[state->bucket_index] += magnitude;
	state->window_counts += magnitude;
	state->activation_counts = MIN(
		state->activation_counts + magnitude,
		(uint32_t)ENCODER_ACCELERATION_ACTIVATION_COUNTS);
	encoder_acceleration_speed_update(state);
	++state->samples_in_bucket;

	return accelerated_delta;
}
