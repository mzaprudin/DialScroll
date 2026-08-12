#ifndef BLESCROLL_ENCODER_MATH_H_
#define BLESCROLL_ENCODER_MATH_H_

#include <stddef.h>
#include <stdint.h>

#include "app_config.h"

uint16_t encoder_adc_normalize(uint16_t raw_sample);
int16_t encoder_angle_delta(uint16_t current, uint16_t previous);
int encoder_angle_filter(const uint16_t samples[ENCODER_FILTER_SAMPLE_COUNT],
			 uint16_t *angle);
int32_t encoder_scroll_accumulate(int16_t delta, uint8_t resolution_multiplier,
				  int32_t *remainder);

#endif /* BLESCROLL_ENCODER_MATH_H_ */
