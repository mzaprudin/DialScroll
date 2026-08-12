#include "scroll_resolution.h"

#include "scroll_acceleration.h"

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

static atomic_t current_mode = ATOMIC_INIT(SCROLL_RESOLUTION_DEFAULT_MODE);
static atomic_t acceleration_enabled =
	ATOMIC_INIT(SCROLL_ACCELERATION_DEFAULT_ENABLED ? 1 : 0);
static atomic_t settings_ready;

static bool scroll_resolution_mode_valid(uint8_t mode)
{
	return mode <= SCROLL_RESOLUTION_MODE_X16;
}

static int scroll_settings_set(const char *name, size_t len,
			       settings_read_cb read_cb, void *cb_arg)
{
	const char *next;
	uint8_t stored_mode;
	bool is_resolution;
	bool is_acceleration;
	ssize_t read;

	is_resolution = settings_name_steq(name, SCROLL_SETTINGS_NAME, &next) &&
		next == NULL;
	is_acceleration = settings_name_steq(
		name, ACCELERATION_SETTINGS_NAME, &next) && next == NULL;
	if (!is_resolution && !is_acceleration) {
		return -ENOENT;
	}
	if (len != sizeof(stored_mode)) {
		return -EINVAL;
	}

	read = read_cb(cb_arg, &stored_mode, sizeof(stored_mode));
	if (read < 0) {
		return (int)read;
	}
	if (read != sizeof(stored_mode)) {
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
	LOG_INF("Scroll settings loaded: resolution=%s acceleration=%s",
		scroll_resolution_name_get(), scroll_acceleration_name_get());
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
