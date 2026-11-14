#ifndef BLESCROLL_ENCODER_MATH_H_
#define BLESCROLL_ENCODER_MATH_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_config.h"

struct encoder_acceleration_state {
	uint32_t buckets[ENCODER_ACCELERATION_BUCKET_COUNT];
	uint32_t window_counts;
	uint32_t activation_counts;
	uint32_t speed_counts_per_s;
	int32_t fractional_remainder_q8;
	uint16_t idle_ms;
	uint16_t factor_q8;
	uint8_t bucket_index;
	uint8_t samples_in_bucket;
	int8_t direction;
	bool enabled;
	bool skip_next_motion;
};

struct encoder_angle_ema_state {
	bool valid;
	int32_t angle_q8;
};

uint16_t encoder_adc_normalize(uint16_t raw_sample);
uint16_t encoder_mt6701_decode_angle(uint8_t msb, uint8_t lsb);
uint16_t encoder_as5600_decode_angle(uint8_t msb, uint8_t lsb);
int16_t encoder_angle_delta(uint16_t current, uint16_t previous);
void encoder_angle_ema_reset(struct encoder_angle_ema_state *state);
uint16_t encoder_angle_ema_update(
	struct encoder_angle_ema_state *state, uint16_t raw_angle);
int encoder_angle_filter(const uint16_t samples[ENCODER_FILTER_SAMPLE_COUNT],
			 uint16_t *angle);
int32_t encoder_scroll_accumulate(int32_t delta, uint8_t resolution_multiplier,
				  int32_t *remainder);
void encoder_acceleration_reset(struct encoder_acceleration_state *state,
				bool enabled);
uint16_t encoder_acceleration_factor_q8(uint32_t speed_counts_per_s);
int32_t encoder_acceleration_scale_q8(int32_t delta, uint16_t factor_q8,
				      int32_t *fractional_remainder_q8);
int32_t encoder_acceleration_apply(struct encoder_acceleration_state *state,
				   int16_t delta, bool enabled,
				   bool include_in_estimate);

#endif /* BLESCROLL_ENCODER_MATH_H_ */
