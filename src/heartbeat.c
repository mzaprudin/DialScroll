#include "heartbeat.h"

#include <errno.h>
#include <stdbool.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define HEARTBEAT_OFF_TIME K_SECONDS(8)
#define HEARTBEAT_ON_TIME K_MSEC(150)

LOG_MODULE_REGISTER(heartbeat, LOG_LEVEL_INF);

static const struct gpio_dt_spec heartbeat_led =
	GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static bool heartbeat_led_on;

static void heartbeat_work_handler(struct k_work *work);

K_WORK_DELAYABLE_DEFINE(heartbeat_work, heartbeat_work_handler);

static void heartbeat_work_handler(struct k_work *work)
{
	int err;

	ARG_UNUSED(work);

	heartbeat_led_on = !heartbeat_led_on;
	err = gpio_pin_set_dt(&heartbeat_led, heartbeat_led_on);
	if (err < 0) {
		LOG_WRN("Heartbeat LED update failed: %d", err);
	}

	(void)k_work_reschedule(&heartbeat_work,
				heartbeat_led_on ? HEARTBEAT_ON_TIME :
						   HEARTBEAT_OFF_TIME);
}

int heartbeat_init(void)
{
	int err;

	if (!gpio_is_ready_dt(&heartbeat_led)) {
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT_INACTIVE);
	if (err < 0) {
		return err;
	}

	heartbeat_led_on = false;
	(void)k_work_schedule(&heartbeat_work, HEARTBEAT_OFF_TIME);
	return 0;
}
