#include "battery.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "app_config.h"
#include "battery_math.h"

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

#define USER_NODE DT_PATH(zephyr_user)
static const struct adc_dt_spec battery_adc = ADC_DT_SPEC_GET_BY_IDX(USER_NODE, 1);
static uint8_t last_battery_level;
static bool battery_level_valid;
static atomic_t diagnostic_battery_valid;
static atomic_t diagnostic_battery_mv;
static atomic_t diagnostic_battery_pin_mv;
static atomic_t diagnostic_battery_raw;
static atomic_t diagnostic_battery_level;
static struct k_work_delayable battery_update_work;

static int battery_voltage_read(int32_t *millivolts, int32_t *pin_millivolts,
				int32_t *raw_average)
{
	int64_t pin_mv_sum = 0;
	int64_t raw_sum = 0;

	for (size_t index = 0U;
	     index < BATTERY_ADC_DISCARD_COUNT + BATTERY_ADC_SAMPLE_COUNT;
	     ++index) {
		int16_t raw_sample = 0;
		int32_t pin_mv;
		int err;
		struct adc_sequence sequence = {
			.buffer = &raw_sample,
			.buffer_size = sizeof(raw_sample),
		};

		err = adc_sequence_init_dt(&battery_adc, &sequence);
		if (err < 0) {
			return err;
		}
		err = adc_read_dt(&battery_adc, &sequence);
		if (err < 0) {
			return err;
		}
		pin_mv = MAX(raw_sample, 0);
		err = adc_raw_to_millivolts_dt(&battery_adc, &pin_mv);
		if (err < 0) {
			return err;
		}
		if (index >= BATTERY_ADC_DISCARD_COUNT) {
			raw_sum += MAX(raw_sample, 0);
			pin_mv_sum += pin_mv;
		}
	}

	*raw_average = (int32_t)(raw_sum / BATTERY_ADC_SAMPLE_COUNT);
	*pin_millivolts = (int32_t)(pin_mv_sum / BATTERY_ADC_SAMPLE_COUNT);
	*millivolts = (int32_t)((pin_mv_sum *
		(BATTERY_DIVIDER_TOP_OHMS + BATTERY_DIVIDER_BOTTOM_OHMS)) /
		(BATTERY_ADC_SAMPLE_COUNT * BATTERY_DIVIDER_BOTTOM_OHMS));
	return 0;
}

static void battery_update_work_handler(struct k_work *work)
{
	int32_t millivolts;
	int32_t pin_millivolts;
	int32_t raw_average;
	uint8_t level;
	int err;

	ARG_UNUSED(work);
	err = battery_voltage_read(&millivolts, &pin_millivolts, &raw_average);
	if (err < 0) {
		LOG_WRN("Battery ADC read failed: %d", err);
		goto schedule_next;
	}

	level = battery_voltage_to_percent(millivolts);
	atomic_set(&diagnostic_battery_mv, millivolts);
	atomic_set(&diagnostic_battery_pin_mv, pin_millivolts);
	atomic_set(&diagnostic_battery_raw, raw_average);
	atomic_set(&diagnostic_battery_level, level);
	atomic_set(&diagnostic_battery_valid, 1);
	if (!battery_level_valid || level != last_battery_level) {
		err = bt_bas_set_battery_level(level);
		if (err < 0) {
			LOG_WRN("Battery level update failed: %d", err);
		} else {
			last_battery_level = level;
			battery_level_valid = true;
		}
	}
	LOG_INF("Battery: %d mV, %u%%", millivolts, level);

schedule_next:
	(void)k_work_reschedule(&battery_update_work, K_MSEC(BATTERY_UPDATE_PERIOD_MS));
}

int battery_init(void)
{
	int err;

	if (!adc_is_ready_dt(&battery_adc)) {
		return -ENODEV;
	}
	err = adc_channel_setup_dt(&battery_adc);
	if (err < 0) {
		return err;
	}
	k_work_init_delayable(&battery_update_work, battery_update_work_handler);
	LOG_INF("Battery monitor: AIN2/P0.04, divider %u/%u ohm, update=%u ms",
		BATTERY_DIVIDER_TOP_OHMS, BATTERY_DIVIDER_BOTTOM_OHMS,
		BATTERY_UPDATE_PERIOD_MS);
	(void)k_work_reschedule(&battery_update_work,
		K_MSEC(BATTERY_FIRST_UPDATE_DELAY_MS));
	return 0;
}

void battery_diagnostic_print(void)
{
	if (!atomic_get(&diagnostic_battery_valid)) {
		printk("battery: no sample yet; update=%u ms\r\n",
		       BATTERY_UPDATE_PERIOD_MS);
		return;
	}

	printk("battery: %d mV (ADC pin: %d mV, raw: %d), %u%%; update=%u ms\r\n",
	       (int)atomic_get(&diagnostic_battery_mv),
	       (int)atomic_get(&diagnostic_battery_pin_mv),
	       (int)atomic_get(&diagnostic_battery_raw),
	       (unsigned int)atomic_get(&diagnostic_battery_level),
	       BATTERY_UPDATE_PERIOD_MS);
}
