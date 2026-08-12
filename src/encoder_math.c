#include "encoder_math.h"

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>

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

int32_t encoder_scroll_accumulate(int16_t delta, uint8_t resolution_multiplier,
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
