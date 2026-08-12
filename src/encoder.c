#include "encoder.h"

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "app_config.h"
#include "ble_hid.h"
#include "diagnostics.h"
#include "encoder_math.h"
#include "scroll_acceleration.h"
#include "scroll_resolution.h"

LOG_MODULE_REGISTER(encoder, LOG_LEVEL_INF);

#define USER_NODE DT_PATH(zephyr_user)

#if !DT_NODE_EXISTS(USER_NODE) || !DT_NODE_HAS_PROP(USER_NODE, io_channels)
#error "app.overlay must define zephyr,user io-channels"
#endif

#if !DT_NODE_HAS_PROP(USER_NODE, encoder_power_gpios)
#error "app.overlay must define zephyr,user encoder-power-gpios"
#endif

static const struct adc_dt_spec encoder_adc = ADC_DT_SPEC_GET_BY_IDX(USER_NODE, 0);
static const struct gpio_dt_spec encoder_power =
	GPIO_DT_SPEC_GET(USER_NODE, encoder_power_gpios);
static atomic_t encoder_power_enabled;
static atomic_t encoder_active;
static atomic_t encoder_angle_valid;
static atomic_t encoder_last_angle;
static atomic_t encoder_last_delta;
static atomic_t encoder_acceleration_speed_counts_s;
static atomic_t encoder_runtime_acceleration_factor_q8 =
	ATOMIC_INIT(ENCODER_ACCELERATION_Q8_ONE);
static atomic_t encoder_acceleration_direction;
static atomic_t encoder_acceleration_activation_counts;

#if defined(CONFIG_BLESCROLL_ENCODER_STARTUP_DIAGNOSTIC)
struct encoder_startup_sample {
	uint32_t start_us;
	uint32_t end_us;
	uint16_t raw_adc;
	uint16_t angle;
	int16_t error;
};

static struct encoder_startup_sample
	startup_samples[ENCODER_STARTUP_CAPTURE_MAX_SAMPLES];
#endif

static int encoder_power_set(bool enabled)
{
	int err = gpio_pin_set_dt(&encoder_power, enabled ? 1 : 0);

	if (err < 0) {
		LOG_ERR("Encoder power %s failed: %d", enabled ? "on" : "off", err);
	} else {
		atomic_set(&encoder_power_enabled, enabled ? 1 : 0);
	}

	return err;
}

static int encoder_adc_read(uint16_t *raw_adc)
{
	int16_t raw_sample = 0;
	int err;
	struct adc_sequence sequence = {
		.buffer = &raw_sample,
		.buffer_size = sizeof(raw_sample),
	};

	err = adc_sequence_init_dt(&encoder_adc, &sequence);
	if (err < 0) {
		return err;
	}

	err = adc_read_dt(&encoder_adc, &sequence);
	if (err < 0) {
		return err;
	}

	*raw_adc = (uint16_t)CLAMP(raw_sample, 0, ENCODER_ADC_MAX_CODE);
	return 0;
}

static int encoder_angle_read(uint16_t *angle)
{
	uint16_t samples[ENCODER_FILTER_SAMPLE_COUNT];
	int err;

	for (size_t index = 0U; index < ARRAY_SIZE(samples); ++index) {
		err = encoder_adc_read(&samples[index]);
		if (err < 0) {
			LOG_ERR("SAADC read failed: %d", err);
			return err;
		}
	}

	err = encoder_angle_filter(samples, angle);
	if (err == 0) {
		atomic_set(&encoder_last_angle, *angle);
		atomic_set(&encoder_angle_valid, 1);
	}
	return err;
}

static void encoder_sleep_until(int64_t deadline_ms)
{
	const int64_t remaining_ms = deadline_ms - k_uptime_get();

	if (remaining_ms > 0) {
		k_sleep(K_MSEC(remaining_ms));
	}
}

static bool encoder_requires_ble_connection(void)
{
	return !IS_ENABLED(CONFIG_BLESCROLL_DIAGNOSTIC_MODE) ||
	       IS_ENABLED(CONFIG_BLESCROLL_DIAGNOSTIC_BLE);
}

static bool encoder_diagnostics_enabled(void)
{
	return IS_ENABLED(CONFIG_BLESCROLL_ENCODER_DIAGNOSTICS) ||
	       diagnostics_is_enabled();
}

static uint32_t encoder_idle_poll_period(int64_t now_ms, int64_t last_motion_ms)
{
	if ((last_motion_ms >= 0) &&
	    ((now_ms - last_motion_ms) < ENCODER_IDLE_FAST_DURATION_MS)) {
		return ENCODER_IDLE_FAST_POLL_PERIOD_MS;
	}

	return ENCODER_IDLE_SLOW_POLL_PERIOD_MS;
}

static const char *encoder_direction_name(int32_t direction)
{
	if (direction > 0) {
		return "+";
	}
	if (direction < 0) {
		return "-";
	}

	return "none";
}

static void encoder_acceleration_publish(
	const struct encoder_acceleration_state *acceleration)
{
	atomic_set(&encoder_acceleration_speed_counts_s,
		   acceleration->speed_counts_per_s);
	atomic_set(&encoder_runtime_acceleration_factor_q8,
		   acceleration->factor_q8);
	atomic_set(&encoder_acceleration_direction, acceleration->direction);
	atomic_set(&encoder_acceleration_activation_counts,
		   acceleration->activation_counts);
}

static void encoder_acceleration_runtime_reset(
	struct encoder_acceleration_state *acceleration)
{
	encoder_acceleration_reset(acceleration, scroll_acceleration_get());
	encoder_acceleration_publish(acceleration);
}

static void encoder_angle_log(
	uint16_t angle, int16_t delta, int32_t accelerated_delta,
	int32_t scroll_delta,
	const struct encoder_acceleration_state *acceleration)
{
	const uint32_t centidegrees =
		((uint32_t)angle * ENCODER_FULL_TURN_CENTIDEGREES) /
		ENCODER_ADC_MODULUS;
	const uint32_t speed_hundredths_rps =
		(acceleration->speed_counts_per_s * 100U +
		 (ENCODER_ADC_MODULUS / 2U)) / ENCODER_ADC_MODULUS;
	const uint32_t factor_hundredths =
		((uint32_t)acceleration->factor_q8 * 100U +
		 (ENCODER_ACCELERATION_Q8_ONE / 2U)) /
		ENCODER_ACCELERATION_Q8_ONE;
	const uint32_t activation_centidegrees =
		(acceleration->activation_counts * ENCODER_FULL_TURN_CENTIDEGREES) /
		ENCODER_ADC_MODULUS;

	LOG_INF("angle=%u.%02u raw=%u delta=%d accelerated=%d wheel=%d ble=%s "
		"acceleration=%s speed=%u.%02u_rps factor=x%u.%02u "
		"dir=%s activation=%u.%02u_deg",
		centidegrees / 100U, centidegrees % 100U, angle, delta,
		accelerated_delta, scroll_delta,
		ble_hid_is_connected() ? "connected" : "offline",
		scroll_acceleration_name_get(), speed_hundredths_rps / 100U,
		speed_hundredths_rps % 100U, factor_hundredths / 100U,
		factor_hundredths % 100U,
		encoder_direction_name(acceleration->direction),
		activation_centidegrees / 100U, activation_centidegrees % 100U);
}

static void encoder_motion_process(uint16_t angle, int16_t delta,
				   bool significant, bool include_in_estimate,
				   struct encoder_acceleration_state *acceleration,
				   int32_t *scroll_remainder,
				   uint8_t *scroll_multiplier,
				   int64_t *last_diagnostic_ms,
				   uint32_t *diagnostic_burst_remaining)
{
	const bool connected = ble_hid_is_connected();
	int32_t accelerated_delta;
	int32_t scroll_delta;
	const int64_t now_ms = k_uptime_get();
	atomic_set(&encoder_last_delta, delta);

	if (!connected) {
		encoder_acceleration_runtime_reset(acceleration);
		accelerated_delta = significant ? delta : 0;
		/* Never replay motion that happened before the host connected. */
		*scroll_remainder = 0;
		scroll_delta = 0;
	} else {
		const uint8_t current_multiplier =
			scroll_resolution_multiplier_get();

		accelerated_delta = encoder_acceleration_apply(
			acceleration, significant ? delta : 0,
			scroll_acceleration_get(), include_in_estimate);
		encoder_acceleration_publish(acceleration);

		if (*scroll_multiplier != current_multiplier) {
			/* Do not reinterpret a partial step accumulated using the old
			 * granularity. Already queued HID units remain valid x16 units.
			 */
			*scroll_remainder = 0;
			*scroll_multiplier = current_multiplier;
		}
		scroll_delta = encoder_scroll_accumulate(
			accelerated_delta, current_multiplier, scroll_remainder);
	}

	if (scroll_delta != 0) {
		ble_hid_scroll_enqueue(scroll_delta);
	}

	if (encoder_diagnostics_enabled() &&
	    ((*diagnostic_burst_remaining > 0U) ||
	     ((now_ms - *last_diagnostic_ms) >= ENCODER_DIAGNOSTIC_PERIOD_MS))) {
		encoder_angle_log(angle, delta, accelerated_delta, scroll_delta,
				  acceleration);
		*last_diagnostic_ms = now_ms;
		if (*diagnostic_burst_remaining > 0U) {
			--*diagnostic_burst_remaining;
		}
	}
}

static uint16_t encoder_active_run(uint16_t first_angle, int16_t first_delta,
				   struct encoder_acceleration_state *acceleration,
				   int32_t *scroll_remainder,
				   uint8_t *scroll_multiplier,
				   int64_t *last_motion_out_ms)
{
	uint16_t reference_angle = first_angle;
	uint16_t sampled_angle = first_angle;
	int64_t last_motion_ms = k_uptime_get();
	int64_t last_diagnostic_ms = last_motion_ms - ENCODER_DIAGNOSTIC_PERIOD_MS;
	int64_t next_sample_ms = last_motion_ms;
	uint32_t diagnostic_burst_remaining = ENCODER_DIAGNOSTIC_BURST_COUNT;

	atomic_set(&encoder_active, 1);
	LOG_INF("Encoder active; sensor remains powered");
	encoder_motion_process(first_angle, first_delta, true, false, acceleration,
			       scroll_remainder,
			       scroll_multiplier,
			       &last_diagnostic_ms, &diagnostic_burst_remaining);

	while (true) {
		int16_t delta;
		int err;
		const int64_t now_ms = k_uptime_get();

		if (encoder_requires_ble_connection() && !ble_hid_is_connected()) {
			break;
		}

		if ((now_ms - last_motion_ms) >= ENCODER_ACTIVE_TIMEOUT_MS) {
			break;
		}

		next_sample_ms += ENCODER_ACTIVE_POLL_PERIOD_MS;
		encoder_sleep_until(next_sample_ms);

		err = encoder_angle_read(&sampled_angle);
		if (err < 0) {
			break;
		}

		delta = encoder_angle_delta(sampled_angle, reference_angle);
		if (abs(delta) <= ENCODER_ACTIVE_NOISE_COUNTS) {
			encoder_motion_process(sampled_angle, delta, false, true,
				       acceleration, scroll_remainder,
				       scroll_multiplier, &last_diagnostic_ms,
				       &diagnostic_burst_remaining);
			continue;
		}

		reference_angle = sampled_angle;
		last_motion_ms = k_uptime_get();
		encoder_motion_process(sampled_angle, delta, true, true,
				       acceleration, scroll_remainder,
				       scroll_multiplier,
				       &last_diagnostic_ms,
				       &diagnostic_burst_remaining);
	}

	LOG_INF("Encoder idle after %u ms without movement", ENCODER_ACTIVE_TIMEOUT_MS);
	atomic_set(&encoder_active, 0);
	encoder_acceleration_runtime_reset(acceleration);
	*last_motion_out_ms = last_motion_ms;
	return sampled_angle;
}

void encoder_diagnostic_print(void)
{
	const bool acceleration_enabled = scroll_acceleration_get();
	const uint32_t angle = (uint32_t)atomic_get(&encoder_last_angle);
	const uint32_t centidegrees =
		(angle * ENCODER_FULL_TURN_CENTIDEGREES) / ENCODER_ADC_MODULUS;
	const uint32_t speed_counts_s = acceleration_enabled ?
		(uint32_t)atomic_get(&encoder_acceleration_speed_counts_s) : 0U;
	const uint32_t speed_hundredths_rps =
		(speed_counts_s * 100U + (ENCODER_ADC_MODULUS / 2U)) /
		ENCODER_ADC_MODULUS;
	const uint32_t factor_q8 = acceleration_enabled ?
		(uint32_t)atomic_get(&encoder_runtime_acceleration_factor_q8) :
		ENCODER_ACCELERATION_Q8_ONE;
	const uint32_t factor_hundredths =
		(factor_q8 * 100U + (ENCODER_ACCELERATION_Q8_ONE / 2U)) /
		ENCODER_ACCELERATION_Q8_ONE;
	const uint32_t activation_counts = acceleration_enabled ?
		(uint32_t)atomic_get(&encoder_acceleration_activation_counts) : 0U;
	const uint32_t activation_centidegrees =
		(activation_counts * ENCODER_FULL_TURN_CENTIDEGREES) /
		ENCODER_ADC_MODULUS;

	printk("encoder: state=%s power=%s angle=%s",
	       atomic_get(&encoder_active) ? "active" : "idle",
	       atomic_get(&encoder_power_enabled) ? "on" : "off",
	       atomic_get(&encoder_angle_valid) ? "valid" : "unavailable");
	if (atomic_get(&encoder_angle_valid)) {
		printk(" raw=%u (%u.%02u deg) delta=%d", angle,
		       centidegrees / 100U, centidegrees % 100U,
		       (int)atomic_get(&encoder_last_delta));
	}
	printk("\r\n");
	printk("acceleration: %s speed=%u.%02u rps factor=x%u.%02u "
	       "direction=%s activation=%u.%02u deg\r\n",
	       scroll_acceleration_name_get(), speed_hundredths_rps / 100U,
	       speed_hundredths_rps % 100U, factor_hundredths / 100U,
	       factor_hundredths % 100U,
	       encoder_direction_name(acceleration_enabled ?
		atomic_get(&encoder_acceleration_direction) : 0),
	       activation_centidegrees / 100U, activation_centidegrees % 100U);
	printk("acceleration profile: activation=45.00 deg window=%u ms "
	       "x1.00<=0.40 rps linear x2.00>=2.00 rps\r\n",
	       ENCODER_ACCELERATION_WINDOW_MS);
	printk("encoder config: idle=%u/%u ms settle=%u ms active=%u ms "
	       "timeout=%u ms noise=%d counts\r\n",
	       ENCODER_IDLE_FAST_POLL_PERIOD_MS,
	       ENCODER_IDLE_SLOW_POLL_PERIOD_MS,
	       ENCODER_POWER_STABILIZE_MS, ENCODER_ACTIVE_POLL_PERIOD_MS,
	       ENCODER_ACTIVE_TIMEOUT_MS, ENCODER_ACTIVE_NOISE_COUNTS);
}

int encoder_init(void)
{
	int err;

	if (!gpio_is_ready_dt(&encoder_power)) {
		LOG_ERR("Encoder power GPIO is not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&encoder_power, GPIO_OUTPUT_INACTIVE);
	if (err < 0) {
		LOG_ERR("Encoder power GPIO setup failed: %d", err);
		return err;
	}

	if (!adc_is_ready_dt(&encoder_adc)) {
		LOG_ERR("SAADC is not ready");
		return -ENODEV;
	}

	err = adc_channel_setup_dt(&encoder_adc);
	if (err < 0) {
		LOG_ERR("SAADC channel setup failed: %d", err);
		return err;
	}

	LOG_INF("MT6701: OUT=AIN0/P0.02, power=P0.06, idle=%u/%u ms, active=%u ms",
		ENCODER_IDLE_FAST_POLL_PERIOD_MS, ENCODER_IDLE_SLOW_POLL_PERIOD_MS,
		ENCODER_ACTIVE_POLL_PERIOD_MS);
	LOG_INF("MT6701 power stabilization is %u ms", ENCODER_POWER_STABILIZE_MS);

	return 0;
}

void encoder_startup_diagnostic_run(void)
{
#if defined(CONFIG_BLESCROLL_ENCODER_STARTUP_DIAGNOSTIC)
	uint32_t power_on_cycles;
	uint32_t sample_count = 0U;
	int power_err;

	power_err = encoder_power_set(true);
	if (power_err < 0) {
		printk("Encoder power-on failed: %d\r\n", power_err);
		return;
	}
	power_on_cycles = k_cycle_get_32();

	while (sample_count < ARRAY_SIZE(startup_samples)) {
		struct encoder_startup_sample *const sample =
			&startup_samples[sample_count];
		uint32_t now_cycles = k_cycle_get_32();
		uint32_t elapsed_us = k_cyc_to_us_floor32(now_cycles - power_on_cycles);
		int err;

		if (elapsed_us >= ENCODER_STARTUP_CAPTURE_DURATION_US) {
			break;
		}

		sample->start_us = (uint32_t)elapsed_us;
		err = encoder_adc_read(&sample->raw_adc);
		sample->angle = encoder_adc_normalize(sample->raw_adc);
		sample->end_us = k_cyc_to_us_floor32(
			k_cycle_get_32() - power_on_cycles);
		sample->error = (int16_t)err;
		++sample_count;
	}

	(void)encoder_power_set(false);
	printk("\r\n--- capture: %u samples, window=%u us ---\r\n",
	       sample_count, ENCODER_STARTUP_CAPTURE_DURATION_US);
	printk("index,start_us,end_us,raw_adc,angle_raw,angle_cdeg,error\r\n");
	for (uint32_t index = 0U; index < sample_count; ++index) {
		const struct encoder_startup_sample *const sample =
			&startup_samples[index];
		const uint32_t centidegrees =
			((uint32_t)sample->angle * ENCODER_FULL_TURN_CENTIDEGREES) /
			ENCODER_ADC_MODULUS;

		printk("%u,%u,%u,%u,%u,%u,%d\r\n", index, sample->start_us,
		       sample->end_us, sample->raw_adc, sample->angle,
		       centidegrees, sample->error);
		k_sleep(K_MSEC(4));
	}
	printk("--- end capture; encoder power is off ---\r\n");
#else
	/* The regular firmware never calls the diagnostic entry point. */
#endif
}

void encoder_run(void)
{
	uint16_t idle_reference = 0U;
	struct encoder_acceleration_state acceleration;
	int32_t scroll_remainder = 0;
	uint8_t scroll_multiplier = scroll_resolution_multiplier_get();
	bool reference_valid = false;
	int64_t last_motion_ms = -1;
	int64_t next_probe_ms = k_uptime_get();

	encoder_acceleration_runtime_reset(&acceleration);

	while (true) {
		uint16_t angle;
		int16_t delta;
		int err;
		int64_t now_ms;
		uint32_t idle_poll_period_ms;

		encoder_sleep_until(next_probe_ms);
		now_ms = k_uptime_get();

		if (encoder_requires_ble_connection() && !ble_hid_is_connected()) {
			/* Ignore all motion while there is no host to receive it. */
			reference_valid = false;
			scroll_remainder = 0;
			scroll_multiplier = scroll_resolution_multiplier_get();
			encoder_acceleration_runtime_reset(&acceleration);
			last_motion_ms = -1;
			next_probe_ms = now_ms + ENCODER_IDLE_SLOW_POLL_PERIOD_MS;
			continue;
		}

		idle_poll_period_ms = encoder_idle_poll_period(now_ms, last_motion_ms);
		next_probe_ms = now_ms + idle_poll_period_ms;

		err = encoder_power_set(true);
		if (err < 0) {
			continue;
		}

		k_sleep(K_MSEC(ENCODER_POWER_STABILIZE_MS));
		err = encoder_angle_read(&angle);
		if (err < 0) {
			(void)encoder_power_set(false);
			continue;
		}

		if (!reference_valid) {
			idle_reference = angle;
			reference_valid = true;
			/* Startup has no recent motion, so begin in the low-duty 200 ms
			 * probe cadence. A real active run records its own fast window.
			 */
			last_motion_ms = k_uptime_get() - ENCODER_IDLE_FAST_DURATION_MS;
			(void)encoder_power_set(false);
			LOG_INF("Encoder baseline captured: raw=%u", angle);
			continue;
		}

		delta = encoder_angle_delta(angle, idle_reference);
		if (abs(delta) <= ENCODER_IDLE_MOTION_THRESHOLD_COUNTS) {
			(void)encoder_power_set(false);
			continue;
		}

		idle_reference = encoder_active_run(angle, delta, &acceleration,
					    &scroll_remainder,
					    &scroll_multiplier,
					    &last_motion_ms);
		(void)encoder_power_set(false);
		next_probe_ms = k_uptime_get() + ENCODER_IDLE_FAST_POLL_PERIOD_MS;
	}
}
