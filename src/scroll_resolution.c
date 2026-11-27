#include "scroll_resolution.h"

#include "scroll_acceleration.h"
#include "scroll_direction.h"
#include "scroll_sensitivity.h"
#include "encoder_mode.h"
#include "app_config.h"

#include <errno.h>
#include <string.h>

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(scroll_resolution, LOG_LEVEL_INF);

#define SCROLL_SETTINGS_SUBTREE "blescroll"
#define SCROLL_SETTINGS_NAME "scroll_resolution"
#define SCROLL_SETTINGS_KEY SCROLL_SETTINGS_SUBTREE "/" SCROLL_SETTINGS_NAME
#define ACCELERATION_SETTINGS_NAME "acceleration"
#define ACCELERATION_SETTINGS_KEY \
	SCROLL_SETTINGS_SUBTREE "/" ACCELERATION_SETTINGS_NAME
#define ENCODER_MODE_SETTINGS_NAME "encoder_mode"
#define ENCODER_MODE_SETTINGS_KEY \
	SCROLL_SETTINGS_SUBTREE "/" ENCODER_MODE_SETTINGS_NAME
#define DIRECTION_SETTINGS_NAME "scroll_direction"
#define DIRECTION_SETTINGS_KEY SCROLL_SETTINGS_SUBTREE "/" DIRECTION_SETTINGS_NAME
#define SENSITIVITY_SETTINGS_NAME "scroll_sensitivity"
#define SENSITIVITY_SETTINGS_KEY SCROLL_SETTINGS_SUBTREE "/" SENSITIVITY_SETTINGS_NAME

static atomic_t current_mode = ATOMIC_INIT(SCROLL_RESOLUTION_DEFAULT_MODE);
static atomic_t acceleration_enabled =
	ATOMIC_INIT(SCROLL_ACCELERATION_DEFAULT_ENABLED ? 1 : 0);
static atomic_t encoder_mode = ATOMIC_INIT(ENCODER_MODE_DEFAULT);
static atomic_t scroll_direction = ATOMIC_INIT(ENCODER_SCROLL_DIRECTION);
static atomic_t sensitivity_percent = ATOMIC_INIT(SCROLL_SENSITIVITY_DEFAULT_PERCENT);
static atomic_t settings_ready;

static bool scroll_resolution_mode_valid(uint8_t mode)
{
	return mode <= SCROLL_RESOLUTION_MODE_X16;
}

static bool encoder_mode_valid(uint8_t mode)
{
	return mode <= ENCODER_MODE_ADC;
}

static int scroll_settings_set(const char *name, size_t len,
			       settings_read_cb read_cb, void *cb_arg)
{
	const char *next;
	uint8_t stored_mode;
	bool is_resolution;
	bool is_acceleration;
	bool is_encoder_mode;
	bool is_direction;
	bool is_sensitivity;
	const char *optional_name;
	ssize_t read;

	is_resolution = settings_name_steq(name, SCROLL_SETTINGS_NAME, &next) &&
		next == NULL;
	is_acceleration = settings_name_steq(
		name, ACCELERATION_SETTINGS_NAME, &next) && next == NULL;
	is_encoder_mode = settings_name_steq(
		name, ENCODER_MODE_SETTINGS_NAME, &next) && next == NULL;
	is_direction = settings_name_steq(
		name, DIRECTION_SETTINGS_NAME, &next) && next == NULL;
	is_sensitivity = settings_name_steq(
		name, SENSITIVITY_SETTINGS_NAME, &next) && next == NULL;
	optional_name = is_encoder_mode ? "encoder mode" : "scroll direction";
	if (is_sensitivity) {
		optional_name = "scroll sensitivity";
	}
	if (!is_resolution && !is_acceleration && !is_encoder_mode &&
	    !is_direction && !is_sensitivity) {
		return -ENOENT;
	}
	if (len != sizeof(stored_mode)) {
		if (is_encoder_mode || is_direction || is_sensitivity) {
			LOG_WRN("Ignoring stored %s with invalid length %u",
				optional_name, (unsigned int)len);
			return 0;
		}
		return -EINVAL;
	}

	read = read_cb(cb_arg, &stored_mode, sizeof(stored_mode));
	if (read < 0) {
		if (is_encoder_mode || is_direction || is_sensitivity) {
			LOG_WRN("Ignoring unreadable stored %s: %d",
				optional_name, (int)read);
			return 0;
		}
		return (int)read;
	}
	if (read != sizeof(stored_mode)) {
		if (is_encoder_mode || is_direction || is_sensitivity) {
			LOG_WRN("Ignoring truncated stored %s", optional_name);
			return 0;
		}
		return -EINVAL;
	}
	if (is_resolution) {
		if (!scroll_resolution_mode_valid(stored_mode)) {
			LOG_WRN("Ignoring invalid stored scroll resolution mode %u",
				stored_mode);
			return 0;
		}

		atomic_set(&current_mode, stored_mode);
		return 0;
	}
	if (is_acceleration) {
		if (stored_mode > 1U) {
			LOG_WRN("Ignoring invalid stored acceleration state %u",
				stored_mode);
			return 0;
		}

		atomic_set(&acceleration_enabled, stored_mode);
		return 0;
	}
	if (is_sensitivity) {
		if (!scroll_sensitivity_percent_valid(stored_mode)) {
			LOG_WRN("Ignoring invalid stored scroll sensitivity %u", stored_mode);
			return 0;
		}

		atomic_set(&sensitivity_percent, stored_mode);
		return 0;
	}
	if (is_direction) {
		if (stored_mode > 1U) {
			LOG_WRN("Ignoring invalid stored scroll direction %u", stored_mode);
			return 0;
		}

		/* Store 0 for +1 and 1 for -1, without signed-byte conversions. */
		atomic_set(&scroll_direction, stored_mode == 0U ? 1 : -1);
		return 0;
	}
	if (is_encoder_mode) {
		if (!encoder_mode_valid(stored_mode)) {
			LOG_WRN("Ignoring invalid stored encoder mode %u", stored_mode);
			return 0;
		}

		atomic_set(&encoder_mode, stored_mode);
		return 0;
	}

	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(scroll_resolution, SCROLL_SETTINGS_SUBTREE,
			       NULL, scroll_settings_set, NULL, NULL);

int scroll_resolution_init(void)
{
	int err;

	atomic_set(&current_mode, SCROLL_RESOLUTION_DEFAULT_MODE);
	atomic_set(&acceleration_enabled,
		   SCROLL_ACCELERATION_DEFAULT_ENABLED ? 1 : 0);
	atomic_set(&encoder_mode, ENCODER_MODE_DEFAULT);
	atomic_set(&scroll_direction, ENCODER_SCROLL_DIRECTION);
	atomic_set(&sensitivity_percent, SCROLL_SENSITIVITY_DEFAULT_PERCENT);

	if (!IS_ENABLED(CONFIG_SETTINGS)) {
		atomic_set(&settings_ready, 1);
		return 0;
	}

	err = settings_subsys_init();
	if (err < 0) {
		return err;
	}
	err = settings_load_subtree(SCROLL_SETTINGS_SUBTREE);
	if (err < 0) {
		return err;
	}

	atomic_set(&settings_ready, 1);
	LOG_INF("Settings loaded: resolution=%s acceleration=%s encoder=%s "
		"direction=%s sensitivity=%u%%",
		scroll_resolution_name_get(), scroll_acceleration_name_get(),
		encoder_mode_name(encoder_mode_get()), scroll_direction_name_get(),
		scroll_sensitivity_get());
	return 0;
}

int scroll_resolution_set(enum scroll_resolution_mode mode)
{
	const uint8_t stored_mode = (uint8_t)mode;
	int err;

	if (!scroll_resolution_mode_valid(stored_mode)) {
		return -EINVAL;
	}
	if (!atomic_get(&settings_ready)) {
		return -EAGAIN;
	}
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		err = settings_save_one(SCROLL_SETTINGS_KEY, &stored_mode,
					sizeof(stored_mode));
		if (err < 0) {
			return err;
		}
	}

	atomic_set(&current_mode, stored_mode);
	LOG_INF("Scroll resolution changed to %s", scroll_resolution_name_get());
	return 0;
}

enum scroll_resolution_mode scroll_resolution_get(void)
{
	return (enum scroll_resolution_mode)atomic_get(&current_mode);
}

uint8_t scroll_resolution_multiplier_get(void)
{
	switch (scroll_resolution_get()) {
	case SCROLL_RESOLUTION_MODE_OFF:
		return SCROLL_RESOLUTION_OFF_MULTIPLIER;
	case SCROLL_RESOLUTION_MODE_X8:
		return SCROLL_RESOLUTION_X8_MULTIPLIER;
	case SCROLL_RESOLUTION_MODE_X16:
	default:
		return SCROLL_RESOLUTION_X16_MULTIPLIER;
	}
}

const char *scroll_resolution_name_get(void)
{
	switch (scroll_resolution_get()) {
	case SCROLL_RESOLUTION_MODE_OFF:
		return "off (legacy x1)";
	case SCROLL_RESOLUTION_MODE_X8:
		return "x8";
	case SCROLL_RESOLUTION_MODE_X16:
	default:
		return "x16";
	}
}

int scroll_acceleration_set(bool enabled)
{
	const uint8_t stored_enabled = enabled ? 1U : 0U;
	int err;

	if (!atomic_get(&settings_ready)) {
		return -EAGAIN;
	}
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		err = settings_save_one(ACCELERATION_SETTINGS_KEY, &stored_enabled,
					sizeof(stored_enabled));
		if (err < 0) {
			return err;
		}
	}

	atomic_set(&acceleration_enabled, stored_enabled);
	LOG_INF("Scroll acceleration changed to %s",
		scroll_acceleration_name_get());
	return 0;
}

bool scroll_acceleration_get(void)
{
	return atomic_get(&acceleration_enabled) != 0;
}

const char *scroll_acceleration_name_get(void)
{
	return scroll_acceleration_get() ? "enabled" : "disabled";
}

int scroll_direction_set(int8_t direction)
{
	const uint8_t stored_reversed = direction < 0 ? 1U : 0U;
	int err;

	if ((direction != 1) && (direction != -1)) {
		return -EINVAL;
	}
	if (!atomic_get(&settings_ready)) {
		return -EAGAIN;
	}
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		err = settings_save_one(DIRECTION_SETTINGS_KEY, &stored_reversed,
					sizeof(stored_reversed));
		if (err < 0) {
			return err;
		}
	}

	atomic_set(&scroll_direction, direction);
	LOG_INF("Scroll direction changed to %s", scroll_direction_name_get());
	return 0;
}

int8_t scroll_direction_get(void)
{
	return (int8_t)atomic_get(&scroll_direction);
}

const char *scroll_direction_name_get(void)
{
	return scroll_direction_get() > 0 ? "normal (+1)" : "reversed (-1)";
}

int scroll_sensitivity_set(uint8_t percent)
{
	int err;

	if (!scroll_sensitivity_percent_valid(percent)) {
		return -EINVAL;
	}
	if (!atomic_get(&settings_ready)) {
		return -EAGAIN;
	}
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		err = settings_save_one(SENSITIVITY_SETTINGS_KEY, &percent, sizeof(percent));
		if (err < 0) {
			return err;
		}
	}

	atomic_set(&sensitivity_percent, percent);
	LOG_INF("Scroll sensitivity changed to %u%%", percent);
	return 0;
}

int scroll_sensitivity_cycle(void)
{
	const uint8_t current = scroll_sensitivity_get();
	const uint8_t next = current > SCROLL_SENSITIVITY_STEP_PERCENT ?
		current - SCROLL_SENSITIVITY_STEP_PERCENT : SCROLL_SENSITIVITY_MAX_PERCENT;

	return scroll_sensitivity_set(next);
}

uint8_t scroll_sensitivity_get(void)
{
	return (uint8_t)atomic_get(&sensitivity_percent);
}

int encoder_mode_set(enum encoder_mode mode)
{
	const uint8_t stored_mode = (uint8_t)mode;
	int err;

	if (!encoder_mode_valid(stored_mode)) {
		return -EINVAL;
	}
	if (!atomic_get(&settings_ready)) {
		return -EAGAIN;
	}
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		err = settings_save_one(ENCODER_MODE_SETTINGS_KEY, &stored_mode,
					sizeof(stored_mode));
		if (err < 0) {
			return err;
		}
	}

	atomic_set(&encoder_mode, stored_mode);
	LOG_INF("Encoder mode changed to %s", encoder_mode_name(mode));
	return 0;
}

enum encoder_mode encoder_mode_get(void)
{
	return (enum encoder_mode)atomic_get(&encoder_mode);
}

const char *encoder_mode_name(enum encoder_mode mode)
{
	return mode == ENCODER_MODE_ADC ? "ADC" : "I2C";
}
