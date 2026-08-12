#include "scroll_resolution.h"

#include <errno.h>
#include <string.h>

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(scroll_resolution, LOG_LEVEL_INF);

#define SCROLL_SETTINGS_SUBTREE "blescroll"
#define SCROLL_SETTINGS_NAME "scroll_resolution"
#define SCROLL_SETTINGS_KEY SCROLL_SETTINGS_SUBTREE "/" SCROLL_SETTINGS_NAME

static atomic_t current_mode = ATOMIC_INIT(SCROLL_RESOLUTION_DEFAULT_MODE);
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
	ssize_t read;

	if (!settings_name_steq(name, SCROLL_SETTINGS_NAME, &next) || next != NULL) {
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
	if (!scroll_resolution_mode_valid(stored_mode)) {
		LOG_WRN("Ignoring invalid stored scroll resolution mode %u",
			stored_mode);
		return 0;
	}

	atomic_set(&current_mode, stored_mode);
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(scroll_resolution, SCROLL_SETTINGS_SUBTREE,
			       NULL, scroll_settings_set, NULL, NULL);

int scroll_resolution_init(void)
{
	int err;

	atomic_set(&current_mode, SCROLL_RESOLUTION_DEFAULT_MODE);

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
	LOG_INF("Scroll resolution loaded: %s", scroll_resolution_name_get());
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
