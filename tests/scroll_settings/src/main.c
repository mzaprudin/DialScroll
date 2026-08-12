#include <errno.h>
#include <string.h>

#include <zephyr/settings/settings.h>
#include <zephyr/ztest.h>

#include "scroll_acceleration.h"
#include "scroll_resolution.h"

struct memory_value {
	bool present;
	uint8_t value;
};

static struct memory_value stored_resolution;
static struct memory_value stored_acceleration;
static struct settings_store memory_store;

static ssize_t memory_read_cb(void *cb_arg, void *data, size_t len)
{
	const struct memory_value *value = cb_arg;

	if (len < sizeof(value->value)) {
		return -ENOMEM;
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
			     const char *name, struct memory_value *value)
{
	if (!value->present || !subtree_matches(arg->subtree, name)) {
		return 0;
	}
	if (arg->cb != NULL) {
		return arg->cb(name, sizeof(value->value), memory_read_cb,
			       value, arg->param);
	}

	return settings_call_set_handler(name, sizeof(value->value),
					 memory_read_cb, value, arg);
}

static int memory_load(struct settings_store *store,
		       const struct settings_load_arg *arg)
{
	int err;

	ARG_UNUSED(store);
	err = memory_load_value(arg, "blescroll/scroll_resolution",
				&stored_resolution);
	if (err < 0) {
		return err;
	}
	return memory_load_value(arg, "blescroll/acceleration",
				 &stored_acceleration);
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
}

ZTEST_SUITE(scroll_settings, NULL, scroll_settings_setup,
	    scroll_settings_before, NULL, NULL);

ZTEST(scroll_settings, test_old_nvs_defaults_acceleration_to_off)
{
	zassert_ok(scroll_resolution_init());
	zassert_equal(scroll_resolution_get(), SCROLL_RESOLUTION_MODE_X8);
	zassert_false(scroll_acceleration_get());
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
