#include <errno.h>
#include <string.h>

#include <zephyr/settings/settings.h>
#include <zephyr/ztest.h>

#include "encoder_mode.h"
#include "app_config.h"
#include "scroll_acceleration.h"
#include "scroll_direction.h"
#include "scroll_resolution.h"
#include "scroll_sensitivity.h"

struct memory_value {
	bool present;
	uint8_t value;
};

static struct memory_value stored_resolution;
static struct memory_value stored_acceleration;
static struct memory_value stored_encoder_mode;
static struct memory_value stored_direction;
static struct memory_value stored_sensitivity;
static struct settings_store memory_store;
static bool fail_encoder_mode_save;
static bool encoder_mode_bad_length;
static bool fail_direction_save;
static bool direction_bad_length;
static ssize_t direction_read_result;
static bool fail_sensitivity_save;
static bool sensitivity_bad_length;
static ssize_t sensitivity_read_result;

static ssize_t memory_read_cb(void *cb_arg, void *data, size_t len)
{
	const struct memory_value *value = cb_arg;

	if (len < sizeof(value->value)) {
		return -ENOMEM;
	}
	if ((value == &stored_direction) &&
	    (direction_read_result != sizeof(value->value))) {
		return direction_read_result;
	}
	if ((value == &stored_sensitivity) &&
	    (sensitivity_read_result != sizeof(value->value))) {
		return sensitivity_read_result;
	}
	*(uint8_t *)data = value->value;
	return sizeof(value->value);
}

static bool subtree_matches(const char *subtree, const char *name)
{
	if (subtree == NULL) {
		return true;
	}

	return settings_name_steq(name, subtree, NULL) ||
	       settings_name_steq(subtree, name, NULL);
}

static int memory_load_value(const struct settings_load_arg *arg,
			     const char *name, struct memory_value *value,
			     size_t stored_len)
{
	if (!value->present || !subtree_matches(arg->subtree, name)) {
		return 0;
	}
	if (arg->cb != NULL) {
		return arg->cb(name, stored_len, memory_read_cb,
			       value, arg->param);
	}

	return settings_call_set_handler(name, stored_len,
					 memory_read_cb, value, arg);
}

static int memory_load(struct settings_store *store,
		       const struct settings_load_arg *arg)
{
	int err;

	ARG_UNUSED(store);
	err = memory_load_value(arg, "blescroll/scroll_resolution",
				&stored_resolution, sizeof(stored_resolution.value));
	if (err < 0) {
		return err;
	}
	err = memory_load_value(arg, "blescroll/acceleration",
				&stored_acceleration, sizeof(stored_acceleration.value));
	if (err < 0) {
		return err;
	}
	err = memory_load_value(arg, "blescroll/encoder_mode",
				 &stored_encoder_mode,
				 encoder_mode_bad_length ? 2U :
				 sizeof(stored_encoder_mode.value));
	if (err < 0) {
		return err;
	}
	err = memory_load_value(arg, "blescroll/scroll_direction",
				 &stored_direction, direction_bad_length ? 2U :
				 sizeof(stored_direction.value));
	if (err < 0) {
		return err;
	}
	return memory_load_value(arg, "blescroll/scroll_sensitivity",
				 &stored_sensitivity, sensitivity_bad_length ? 2U :
				 sizeof(stored_sensitivity.value));
}

static int memory_save(struct settings_store *store, const char *name,
		       const char *value, size_t len)
{
	struct memory_value *destination;

	ARG_UNUSED(store);
	if (strcmp(name, "blescroll/scroll_resolution") == 0) {
		destination = &stored_resolution;
	} else if (strcmp(name, "blescroll/acceleration") == 0) {
		destination = &stored_acceleration;
	} else if (strcmp(name, "blescroll/encoder_mode") == 0) {
		if (fail_encoder_mode_save) {
			return -EIO;
		}
		destination = &stored_encoder_mode;
	} else if (strcmp(name, "blescroll/scroll_direction") == 0) {
		if (fail_direction_save) {
			return -EIO;
		}
		destination = &stored_direction;
	} else if (strcmp(name, "blescroll/scroll_sensitivity") == 0) {
		if (fail_sensitivity_save) {
			return -EIO;
		}
		destination = &stored_sensitivity;
	} else {
		return -ENOENT;
	}
	if (len != sizeof(destination->value)) {
		return -EINVAL;
	}

	destination->value = *(const uint8_t *)value;
	destination->present = true;
	return 0;
}

static const struct settings_store_itf memory_store_itf = {
	.csi_load = memory_load,
	.csi_save = memory_save,
};

static void *scroll_settings_setup(void)
{
	zassert_ok(settings_subsys_init());
	memory_store.cs_itf = &memory_store_itf;
	settings_src_register(&memory_store);
	settings_dst_register(&memory_store);
	return NULL;
}

static void scroll_settings_before(void *fixture)
{
	ARG_UNUSED(fixture);
	stored_resolution = (struct memory_value) {
		.present = true,
		.value = SCROLL_RESOLUTION_MODE_X8,
	};
	stored_acceleration = (struct memory_value) { 0 };
	stored_encoder_mode = (struct memory_value) { 0 };
	stored_direction = (struct memory_value) { 0 };
	stored_sensitivity = (struct memory_value) { 0 };
	fail_encoder_mode_save = false;
	encoder_mode_bad_length = false;
	fail_direction_save = false;
	direction_bad_length = false;
	direction_read_result = sizeof(stored_direction.value);
	fail_sensitivity_save = false;
	sensitivity_bad_length = false;
	sensitivity_read_result = sizeof(stored_sensitivity.value);
}

ZTEST_SUITE(scroll_settings, NULL, scroll_settings_setup,
	    scroll_settings_before, NULL, NULL);

ZTEST(scroll_settings, test_old_nvs_defaults_acceleration_to_off)
{
	zassert_ok(scroll_resolution_init());
	zassert_equal(scroll_resolution_get(), SCROLL_RESOLUTION_MODE_X8);
	zassert_false(scroll_acceleration_get());
	zassert_equal(encoder_mode_get(), ENCODER_MODE_I2C);
	zassert_equal(scroll_direction_get(), ENCODER_SCROLL_DIRECTION);
	zassert_equal(scroll_sensitivity_get(), 100U);
}

ZTEST(scroll_settings, test_acceleration_survives_reload)
{
	zassert_ok(scroll_resolution_init());
	zassert_ok(scroll_acceleration_set(true));
	zassert_true(stored_acceleration.present);
	zassert_equal(stored_acceleration.value, 1U);

	/* A second init resets defaults before loading, like a reboot. */
	zassert_ok(scroll_resolution_init());
	zassert_true(scroll_acceleration_get());

	zassert_ok(scroll_acceleration_set(false));
	zassert_equal(stored_acceleration.value, 0U);
	zassert_ok(scroll_resolution_init());
	zassert_false(scroll_acceleration_get());
}

ZTEST(scroll_settings, test_encoder_modes_survive_reload)
{
	zassert_ok(scroll_resolution_init());
	zassert_ok(encoder_mode_set(ENCODER_MODE_ADC));
	zassert_true(stored_encoder_mode.present);
	zassert_equal(stored_encoder_mode.value, ENCODER_MODE_ADC);

	zassert_ok(scroll_resolution_init());
	zassert_equal(encoder_mode_get(), ENCODER_MODE_ADC);

	zassert_ok(encoder_mode_set(ENCODER_MODE_I2C));
	zassert_equal(stored_encoder_mode.value, ENCODER_MODE_I2C);
	zassert_ok(scroll_resolution_init());
	zassert_equal(encoder_mode_get(), ENCODER_MODE_I2C);
}

ZTEST(scroll_settings, test_invalid_encoder_mode_is_ignored)
{
	stored_encoder_mode = (struct memory_value) {
		.present = true,
		.value = 0xffU,
	};

	zassert_ok(scroll_resolution_init());
	zassert_equal(encoder_mode_get(), ENCODER_MODE_I2C);
}

ZTEST(scroll_settings, test_malformed_encoder_mode_is_ignored)
{
	stored_encoder_mode = (struct memory_value) {
		.present = true,
		.value = ENCODER_MODE_ADC,
	};
	encoder_mode_bad_length = true;

	zassert_ok(scroll_resolution_init());
	zassert_equal(encoder_mode_get(), ENCODER_MODE_I2C);
}

ZTEST(scroll_settings, test_encoder_mode_changes_only_after_successful_save)
{
	zassert_ok(scroll_resolution_init());
	fail_encoder_mode_save = true;

	zassert_equal(encoder_mode_set(ENCODER_MODE_ADC), -EIO);
	zassert_equal(encoder_mode_get(), ENCODER_MODE_I2C);
}

ZTEST(scroll_settings, test_scroll_directions_survive_reload)
{
	zassert_ok(scroll_resolution_init());
	/* The same read/negate/set operation is used by command R. */
	zassert_ok(scroll_direction_set(-scroll_direction_get()));
	zassert_true(stored_direction.present);
	zassert_equal(stored_direction.value, ENCODER_SCROLL_DIRECTION > 0 ? 1U : 0U);
	zassert_ok(scroll_resolution_init());
	zassert_equal(scroll_direction_get(), -ENCODER_SCROLL_DIRECTION);

	zassert_ok(scroll_direction_set(-scroll_direction_get()));
	zassert_ok(scroll_resolution_init());
	zassert_equal(scroll_direction_get(), ENCODER_SCROLL_DIRECTION);
	zassert_equal(scroll_resolution_get(), SCROLL_RESOLUTION_MODE_X8);
	zassert_false(scroll_acceleration_get());
	zassert_equal(encoder_mode_get(), ENCODER_MODE_I2C);
}

ZTEST(scroll_settings, test_invalid_scroll_direction_is_ignored)
{
	stored_direction = (struct memory_value) {
		.present = true,
		.value = 0xffU,
	};

	zassert_ok(scroll_resolution_init());
	zassert_equal(scroll_direction_get(), ENCODER_SCROLL_DIRECTION);
}

ZTEST(scroll_settings, test_malformed_scroll_direction_is_ignored)
{
	stored_direction = (struct memory_value) {
		.present = true,
		.value = ENCODER_SCROLL_DIRECTION > 0 ? 1U : 0U,
	};
	direction_bad_length = true;
	zassert_ok(scroll_resolution_init());
	zassert_equal(scroll_direction_get(), ENCODER_SCROLL_DIRECTION);

	direction_bad_length = false;
	direction_read_result = -EIO;
	zassert_ok(scroll_resolution_init());
	zassert_equal(scroll_direction_get(), ENCODER_SCROLL_DIRECTION);

	direction_read_result = 0;
	zassert_ok(scroll_resolution_init());
	zassert_equal(scroll_direction_get(), ENCODER_SCROLL_DIRECTION);
}

ZTEST(scroll_settings, test_scroll_direction_changes_only_after_successful_save)
{
	zassert_ok(scroll_resolution_init());
	fail_direction_save = true;

	zassert_equal(scroll_direction_set(-ENCODER_SCROLL_DIRECTION), -EIO);
	zassert_equal(scroll_direction_get(), ENCODER_SCROLL_DIRECTION);
	zassert_false(stored_direction.present);
}

ZTEST(scroll_settings, test_invalid_scroll_direction_is_rejected)
{
	zassert_ok(scroll_resolution_init());
	zassert_equal(scroll_direction_set(0), -EINVAL);
	zassert_equal(scroll_direction_set(2), -EINVAL);
	zassert_equal(scroll_direction_set(-2), -EINVAL);
	zassert_equal(scroll_direction_get(), ENCODER_SCROLL_DIRECTION);
	zassert_false(stored_direction.present);
}

ZTEST(scroll_settings, test_sensitivity_presets_survive_reload)
{
	const uint8_t expected[] = { 75U, 50U, 25U, 100U };

	zassert_ok(scroll_resolution_init());
	zassert_ok(scroll_direction_set(-ENCODER_SCROLL_DIRECTION));
	for (size_t index = 0U; index < ARRAY_SIZE(expected); ++index) {
		zassert_ok(scroll_sensitivity_cycle());
		zassert_equal(scroll_sensitivity_get(), expected[index]);
		zassert_true(stored_sensitivity.present);
		zassert_equal(stored_sensitivity.value, expected[index]);
		zassert_ok(scroll_resolution_init());
		zassert_equal(scroll_sensitivity_get(), expected[index]);
		zassert_equal(scroll_direction_get(), -ENCODER_SCROLL_DIRECTION);
		zassert_equal(scroll_resolution_get(), SCROLL_RESOLUTION_MODE_X8);
		zassert_false(scroll_acceleration_get());
		zassert_equal(encoder_mode_get(), ENCODER_MODE_I2C);
	}
}

ZTEST(scroll_settings, test_invalid_stored_sensitivity_is_ignored)
{
	const uint8_t invalid[] = { 0U, 26U, 101U, 255U };

	for (size_t index = 0U; index < ARRAY_SIZE(invalid); ++index) {
		stored_sensitivity = (struct memory_value) {
			.present = true,
			.value = invalid[index],
		};
		zassert_ok(scroll_resolution_init());
		zassert_equal(scroll_sensitivity_get(), 100U);
		zassert_equal(scroll_resolution_get(), SCROLL_RESOLUTION_MODE_X8);
	}
}

ZTEST(scroll_settings, test_malformed_stored_sensitivity_is_ignored)
{
	stored_sensitivity = (struct memory_value) {
		.present = true,
		.value = 25U,
	};
	sensitivity_bad_length = true;
	zassert_ok(scroll_resolution_init());
	zassert_equal(scroll_sensitivity_get(), 100U);

	sensitivity_bad_length = false;
	sensitivity_read_result = -EIO;
	zassert_ok(scroll_resolution_init());
	zassert_equal(scroll_sensitivity_get(), 100U);

	sensitivity_read_result = 0;
	zassert_ok(scroll_resolution_init());
	zassert_equal(scroll_sensitivity_get(), 100U);
}

ZTEST(scroll_settings, test_sensitivity_changes_only_after_successful_save)
{
	zassert_ok(scroll_resolution_init());
	zassert_ok(scroll_sensitivity_set(75U));
	fail_sensitivity_save = true;

	zassert_equal(scroll_sensitivity_cycle(), -EIO);
	zassert_equal(scroll_sensitivity_get(), 75U);
	zassert_equal(stored_sensitivity.value, 75U);
	zassert_ok(scroll_resolution_init());
	zassert_equal(scroll_sensitivity_get(), 75U);
}

ZTEST(scroll_settings, test_invalid_sensitivity_is_rejected)
{
	const uint8_t invalid[] = { 0U, 1U, 24U, 26U, 99U, 101U, 255U };

	zassert_ok(scroll_resolution_init());
	for (size_t index = 0U; index < ARRAY_SIZE(invalid); ++index) {
		zassert_equal(scroll_sensitivity_set(invalid[index]), -EINVAL);
		zassert_equal(scroll_sensitivity_get(), 100U);
		zassert_false(stored_sensitivity.present);
	}
}
