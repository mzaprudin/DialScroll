#include "ble_hid.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <bluetooth/services/hids.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "app_config.h"
#include "scroll_resolution.h"

LOG_MODULE_REGISTER(ble_hid, LOG_LEVEL_INF);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LENGTH (sizeof(DEVICE_NAME) - 1U)
#define BASE_USB_HID_SPEC_VERSION 0x0111U
#define HID_WHEEL_MIN (-127)
#define HID_WHEEL_MAX 127
#define HID_RESOLUTION_MULTIPLIER_MASK 0x03U
#define HID_RESOLUTION_MULTIPLIER_ENABLED 0x01U

BT_HIDS_DEF(hids, BLE_HID_INPUT_REPORT_SIZE, BLE_HID_FEATURE_REPORT_SIZE);

static struct bt_conn *active_connection;
static K_MUTEX_DEFINE(connection_mutex);
static atomic_t advertising;
static atomic_t pending_scroll;
static atomic_t notification_in_progress;
static uint32_t scroll_retry_count;
static bool report_mode = true;
static bool high_resolution_scroll_enabled;
static int32_t legacy_scroll_remainder;
static bool stale_bond_clear_pending;
static bt_addr_le_t stale_bond_addr;
static uint16_t active_interval;
static uint16_t active_latency;
static uint16_t active_timeout;
static bt_security_t active_security_level;

static void advertising_work_handler(struct k_work *work);
static void scroll_work_handler(struct k_work *work);
static int advertising_start(void);

K_WORK_DEFINE(advertising_work, advertising_work_handler);
K_WORK_DELAYABLE_DEFINE(scroll_work, scroll_work_handler);

static const struct bt_data advertising_data[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
		      (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
		      (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static const struct bt_data scan_response_data[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LENGTH),
};

static const uint8_t report_map[] = {
	0x05, 0x01,       /* Usage Page (Generic Desktop) */
	0x09, 0x02,       /* Usage (Mouse) */
	0xA1, 0x01,       /* Collection (Application) */
	0x85, BLE_HID_INPUT_REPORT_ID, /* Report ID */
	0x09, 0x01,       /* Usage (Pointer) */
	0xA1, 0x00,       /* Collection (Physical) */
	0x05, 0x09,       /* Usage Page (Button) */
	0x19, 0x01,       /* Usage Minimum (Button 1) */
	0x29, 0x03,       /* Usage Maximum (Button 3) */
	0x15, 0x00,       /* Logical Minimum (0) */
	0x25, 0x01,       /* Logical Maximum (1) */
	0x95, 0x03,       /* Report Count (3) */
	0x75, 0x01,       /* Report Size (1) */
	0x81, 0x02,       /* Input (Data, Variable, Absolute) */
	0x95, 0x01,       /* Report Count (1) */
	0x75, 0x05,       /* Report Size (5) */
	0x81, 0x01,       /* Input (Constant) */
	0x05, 0x01,       /* Usage Page (Generic Desktop) */
	0x09, 0x30,       /* Usage (X) */
	0x09, 0x31,       /* Usage (Y) */
	0x15, 0x81,       /* Logical Minimum (-127) */
	0x25, 0x7f,       /* Logical Maximum (127) */
	0x75, 0x08,       /* Report Size (8) */
	0x95, 0x02,       /* Report Count (2) */
	0x81, 0x06,       /* Input (Data, Variable, Relative) */

	/* The multiplier and Wheel must share a Logical Collection. */
	0xA1, 0x02,       /* Collection (Logical) */
	0x85, BLE_HID_FEATURE_REPORT_ID, /* Report ID */
	0x09, 0x48,       /* Usage (Resolution Multiplier) */
	0x95, 0x01,       /* Report Count (1) */
	0x75, 0x02,       /* Report Size (2) */
	0x15, 0x00,       /* Logical Minimum (0) */
	0x25, 0x01,       /* Logical Maximum (1) */
	0x35, 0x01,       /* Physical Minimum (1x) */
	0x45, BLE_HID_WHEEL_RESOLUTION_MULTIPLIER, /* Physical Maximum */
	0x55, 0x00,       /* Unit Exponent (0) */
	0xB1, 0x02,       /* Feature (Data, Variable, Absolute) */
	0x35, 0x00,       /* Physical Minimum (reset) */
	0x45, 0x00,       /* Physical Maximum (reset) */
	0x75, 0x06,       /* Report Size (6) */
	0xB1, 0x01,       /* Feature (Constant) */
	0x85, BLE_HID_INPUT_REPORT_ID, /* Report ID */
	0x09, 0x38,       /* Usage (Wheel) */
	0x15, 0x81,       /* Logical Minimum (-127) */
	0x25, 0x7f,       /* Logical Maximum (127) */
	0x75, 0x08,       /* Report Size (8) */
	0x95, 0x01,       /* Report Count (1) */
	0x81, 0x06,       /* Input (Data, Variable, Relative) */
	0xC0,             /* End Collection (Logical) */
	0xC0,             /* End Collection (Physical) */
	0xC0,             /* End Collection (Application) */
};

static void resolution_multiplier_feature_handler(struct bt_hids_rep *report,
					  struct bt_conn *connection, bool write)
{
	uint8_t selector;
	bool enabled;
	bool changed = false;

	if (!write || (report == NULL) || (report->data == NULL) ||
	    (report->size < BLE_HID_FEATURE_REPORT_SIZE)) {
		return;
	}

	selector = report->data[0] & HID_RESOLUTION_MULTIPLIER_MASK;
	if (selector > HID_RESOLUTION_MULTIPLIER_ENABLED) {
		LOG_WRN("Host wrote invalid Wheel Resolution Multiplier selector %u",
			selector);
		return;
	}
	enabled = selector == HID_RESOLUTION_MULTIPLIER_ENABLED;

	k_mutex_lock(&connection_mutex, K_FOREVER);
	if ((active_connection == connection) &&
	    (high_resolution_scroll_enabled != enabled)) {
		high_resolution_scroll_enabled = enabled;
		legacy_scroll_remainder = 0;
		atomic_set(&pending_scroll, 0);
		changed = true;
	}
	k_mutex_unlock(&connection_mutex);

	if (changed) {
		LOG_INF("High-resolution Wheel %s (effective multiplier x%u)",
			enabled ? "enabled" : "disabled",
			enabled ? BLE_HID_WHEEL_RESOLUTION_MULTIPLIER : 1U);
	}
}

static void protocol_mode_changed(enum bt_hids_pm_evt event,
				  struct bt_conn *connection)
{
	k_mutex_lock(&connection_mutex, K_FOREVER);
	if (active_connection == connection) {
		report_mode = event != BT_HIDS_PM_EVT_BOOT_MODE_ENTERED;
	}
	k_mutex_unlock(&connection_mutex);

	if (event == BT_HIDS_PM_EVT_BOOT_MODE_ENTERED) {
		LOG_WRN("Host selected HID boot mode; Wheel is unavailable");
	} else if (event == BT_HIDS_PM_EVT_REPORT_MODE_ENTERED) {
		LOG_INF("Host selected HID report mode");
	}
}

static int hids_init(void)
{
	struct bt_hids_init_param parameters = { 0 };
	struct bt_hids_inp_rep *input_report;
	struct bt_hids_outp_feat_rep *feature_report;

	parameters.rep_map.data = report_map;
	parameters.rep_map.size = sizeof(report_map);
	parameters.info.bcd_hid = BASE_USB_HID_SPEC_VERSION;
	parameters.info.b_country_code = 0U;
	parameters.info.flags = BT_HIDS_REMOTE_WAKE | BT_HIDS_NORMALLY_CONNECTABLE;
	parameters.pm_evt_handler = protocol_mode_changed;

	input_report = &parameters.inp_rep_group_init.reports[BLE_HID_INPUT_REPORT_INDEX];
	input_report->size = BLE_HID_INPUT_REPORT_SIZE;
	input_report->id = BLE_HID_INPUT_REPORT_ID;
	parameters.inp_rep_group_init.cnt = 1U;

	feature_report = &parameters.feat_rep_group_init.reports[
		BLE_HID_FEATURE_REPORT_INDEX];
	feature_report->size = BLE_HID_FEATURE_REPORT_SIZE;
	feature_report->id = BLE_HID_FEATURE_REPORT_ID;
	feature_report->handler = resolution_multiplier_feature_handler;
	parameters.feat_rep_group_init.cnt = 1U;
	parameters.is_mouse = true;

	return bt_hids_init(&hids, &parameters);
}

static int advertising_start(void)
{
	int err;

	if (!atomic_cas(&advertising, 0, 1)) {
		printk("bt_le_adv_start: already active\r\n");
		return -EALREADY;
	}

	printk("bt_le_adv_start: calling\r\n");
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2,
			      advertising_data, ARRAY_SIZE(advertising_data),
			      scan_response_data, ARRAY_SIZE(scan_response_data));
	printk("bt_le_adv_start: %d\r\n", err);
	if (err < 0) {
		atomic_set(&advertising, 0);
		LOG_ERR("BLE advertising failed: %d", err);
		return err;
	}

	LOG_INF("BLE advertising started as %s", DEVICE_NAME);
	return 0;
}

static void advertising_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	(void)advertising_start();
}

static void connected(struct bt_conn *connection, uint8_t connection_error)
{
	char address[BT_ADDR_LE_STR_LEN];
	int err;

	atomic_set(&advertising, 0);
	bt_addr_le_to_str(bt_conn_get_dst(connection), address, sizeof(address));

	if (connection_error != 0U) {
		LOG_WRN("BLE connection to %s failed: 0x%02x %s", address,
			connection_error, bt_hci_err_to_str(connection_error));
		k_work_submit(&advertising_work);
		return;
	}

	err = bt_hids_connected(&hids, connection);
	if (err < 0) {
		LOG_ERR("HIDS connection setup failed: %d", err);
		(void)bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}

	k_mutex_lock(&connection_mutex, K_FOREVER);
	active_connection = bt_conn_ref(connection);
	report_mode = true;
	active_interval = 0U;
	active_latency = 0U;
	active_timeout = 0U;
	active_security_level = BT_SECURITY_L1;
	high_resolution_scroll_enabled = false;
	legacy_scroll_remainder = 0;
	atomic_set(&pending_scroll, 0);
	k_mutex_unlock(&connection_mutex);

	LOG_INF("BLE connected: %s", address);

	err = bt_conn_set_security(connection, BT_SECURITY_L2);
	if ((err < 0) && (err != -EALREADY)) {
		LOG_WRN("BLE security request failed: %d", err);
	}
}

static void disconnected(struct bt_conn *connection, uint8_t reason)
{
	char address[BT_ADDR_LE_STR_LEN];
	struct bt_conn *connection_to_release = NULL;
	bt_addr_le_t bond_to_clear;
	bool clear_stale_bond = false;
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(connection), address, sizeof(address));
	err = bt_hids_disconnected(&hids, connection);
	if (err < 0) {
		LOG_WRN("HIDS disconnect handling failed: %d", err);
	}

	k_mutex_lock(&connection_mutex, K_FOREVER);
	if (active_connection == connection) {
		connection_to_release = active_connection;
		active_connection = NULL;
	}
	active_interval = 0U;
	active_latency = 0U;
	active_timeout = 0U;
	active_security_level = BT_SECURITY_L1;
	if (stale_bond_clear_pending &&
	    bt_addr_le_eq(&stale_bond_addr, bt_conn_get_dst(connection))) {
		bt_addr_le_copy(&bond_to_clear, &stale_bond_addr);
		stale_bond_clear_pending = false;
		clear_stale_bond = true;
	}
	atomic_set(&pending_scroll, 0);
	atomic_set(&notification_in_progress, 0);
	scroll_retry_count = 0U;
	high_resolution_scroll_enabled = false;
	legacy_scroll_remainder = 0;
	k_mutex_unlock(&connection_mutex);

	if (connection_to_release != NULL) {
		bt_conn_unref(connection_to_release);
	}
	if (clear_stale_bond) {
		err = bt_unpair(BT_ID_DEFAULT, &bond_to_clear);
		if (err < 0) {
			LOG_ERR("Failed to clear stale bond for %s: %d", address, err);
		} else {
			LOG_INF("Cleared stale bond for %s; retry pairing", address);
		}
	}

	(void)k_work_cancel_delayable(&scroll_work);
	LOG_INF("BLE disconnected: %s, reason 0x%02x %s", address, reason,
		bt_hci_err_to_str(reason));
	k_work_submit(&advertising_work);
}

static void security_changed(struct bt_conn *connection, bt_security_t level,
			     enum bt_security_err security_error)
{
	char address[BT_ADDR_LE_STR_LEN];
	bool stale_bond = false;

	bt_addr_le_to_str(bt_conn_get_dst(connection), address, sizeof(address));
	k_mutex_lock(&connection_mutex, K_FOREVER);
	if (active_connection == connection) {
		active_security_level = level;
	}
	k_mutex_unlock(&connection_mutex);
	if (security_error == BT_SECURITY_ERR_SUCCESS) {
		LOG_INF("BLE secured: %s, level %u", address, level);
	} else {
		LOG_WRN("BLE security failed: %s, level %u, error %d %s", address,
			level, security_error,
			bt_security_err_to_str(security_error));

		/* A host that removed this device no longer has its old key, while
		 * our NVS may still contain the corresponding unauthenticated bond.
		 * Zephyr intentionally rejects overwriting it with a new Just Works
		 * bond. Delete only that peer after the failed connection is gone;
		 * the next pairing attempt can then create a fresh key.
		 */
		stale_bond = (level == BT_SECURITY_L1) &&
			((security_error == BT_SECURITY_ERR_AUTH_REQUIREMENT) ||
			 (security_error == BT_SECURITY_ERR_PIN_OR_KEY_MISSING));
		if (stale_bond) {
			k_mutex_lock(&connection_mutex, K_FOREVER);
			bt_addr_le_copy(&stale_bond_addr,
					bt_conn_get_dst(connection));
			stale_bond_clear_pending = true;
			k_mutex_unlock(&connection_mutex);
			LOG_WRN("Stale bond suspected for %s; it will be cleared on disconnect",
				address);
		}
	}
}

static void le_param_updated(struct bt_conn *connection, uint16_t interval,
			     uint16_t latency, uint16_t timeout)
{
	const uint32_t interval_hundredths_ms = (uint32_t)interval * 125U;

	k_mutex_lock(&connection_mutex, K_FOREVER);
	if (active_connection == connection) {
		active_interval = interval;
		active_latency = latency;
		active_timeout = timeout;
	}
	k_mutex_unlock(&connection_mutex);
	LOG_INF("BLE connection interval=%u.%02u ms, latency=%u, timeout=%u ms",
		interval_hundredths_ms / 100U, interval_hundredths_ms % 100U,
		latency, (uint32_t)timeout * 10U);
}

BT_CONN_CB_DEFINE(connection_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
	.le_param_updated = le_param_updated,
};

static void notification_complete(struct bt_conn *connection, void *user_data)
{
	bool is_active_connection;

	ARG_UNUSED(user_data);

	k_mutex_lock(&connection_mutex, K_FOREVER);
	is_active_connection = active_connection == connection;
	k_mutex_unlock(&connection_mutex);
	if (!is_active_connection) {
		return;
	}

	atomic_set(&notification_in_progress, 0);
	if (atomic_get(&pending_scroll) != 0) {
		(void)k_work_reschedule(&scroll_work, K_NO_WAIT);
	}
}

static struct bt_conn *active_connection_get(bool *is_report_mode)
{
	struct bt_conn *connection = NULL;

	k_mutex_lock(&connection_mutex, K_FOREVER);
	if (active_connection != NULL) {
		connection = bt_conn_ref(active_connection);
	}
	*is_report_mode = report_mode;
	k_mutex_unlock(&connection_mutex);

	return connection;
}

static void scroll_work_handler(struct k_work *work)
{
	uint8_t report[BLE_HID_INPUT_REPORT_SIZE] = { 0U };
	struct bt_conn *connection;
	atomic_val_t pending;
	int32_t chunk;
	bool is_report_mode;
	int err;

	ARG_UNUSED(work);

	if (!atomic_cas(&notification_in_progress, 0, 1)) {
		return;
	}

	pending = atomic_get(&pending_scroll);
	if (pending == 0) {
		atomic_set(&notification_in_progress, 0);
		return;
	}

	connection = active_connection_get(&is_report_mode);
	if (connection == NULL) {
		atomic_set(&pending_scroll, 0);
		atomic_set(&notification_in_progress, 0);
		return;
	}

	if (!is_report_mode) {
		LOG_WRN("Host selected HID boot mode; Wheel is unavailable");
		atomic_set(&pending_scroll, 0);
		atomic_set(&notification_in_progress, 0);
		bt_conn_unref(connection);
		return;
	}

	/* Send the movement accumulated during this BLE interval. High-resolution
	 * units are small enough to combine without creating a legacy-sized jump,
	 * and draining the queue prevents a delayed scrolling tail.
	 */
	chunk = CLAMP((int32_t)pending, HID_WHEEL_MIN, HID_WHEEL_MAX);
	report[3] = (uint8_t)(int8_t)chunk;
	err = bt_hids_inp_rep_send(&hids, connection, BLE_HID_INPUT_REPORT_INDEX,
				   report, sizeof(report), notification_complete);

	if (err == 0) {
		k_mutex_lock(&connection_mutex, K_FOREVER);
		if (active_connection == connection) {
			atomic_sub(&pending_scroll, chunk);
			scroll_retry_count = 0U;
		}
		k_mutex_unlock(&connection_mutex);
		bt_conn_unref(connection);
		return;
	}
	bt_conn_unref(connection);

	atomic_set(&notification_in_progress, 0);
	++scroll_retry_count;
	if (scroll_retry_count >= BLE_HID_SCROLL_MAX_RETRIES) {
		LOG_WRN("Dropping queued Wheel delta after %u retries: %d",
			scroll_retry_count, err);
		atomic_set(&pending_scroll, 0);
		scroll_retry_count = 0U;
		return;
	}

	(void)k_work_reschedule(&scroll_work, K_MSEC(BLE_HID_SCROLL_RETRY_MS));
}

int ble_hid_init(void)
{
	int err;

	err = hids_init();
	if (err < 0) {
		LOG_ERR("HIDS initialization failed: %d", err);
		return err;
	}

	err = bt_enable(NULL);
	if (err < 0) {
		LOG_ERR("Bluetooth initialization failed: %d", err);
		return err;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		err = settings_load();
		if (err < 0) {
			LOG_WRN("BLE settings load failed: %d", err);
		}
	}

	LOG_INF("Bluetooth initialized");
	return advertising_start();
}

void ble_hid_scroll_enqueue(int32_t delta)
{
	atomic_val_t old_value;
	atomic_val_t new_value;
	int64_t candidate;
	int32_t report_delta;
	bool accepted = false;

	if (delta == 0) {
		return;
	}

	k_mutex_lock(&connection_mutex, K_FOREVER);
	if (active_connection != NULL) {
		if (high_resolution_scroll_enabled) {
			report_delta = delta;
		} else {
			/* Collapse high-resolution units back into legacy detents for
			 * hosts that did not enable the Feature report.
			 */
			candidate = (int64_t)legacy_scroll_remainder + delta;
			report_delta = (int32_t)(candidate /
				BLE_HID_WHEEL_RESOLUTION_MULTIPLIER);
			legacy_scroll_remainder = (int32_t)(candidate -
				((int64_t)report_delta *
				 BLE_HID_WHEEL_RESOLUTION_MULTIPLIER));
		}

		if (report_delta == 0) {
			k_mutex_unlock(&connection_mutex);
			return;
		}

		do {
			old_value = atomic_get(&pending_scroll);
			candidate = (int64_t)old_value + (int64_t)report_delta;
			new_value = (atomic_val_t)CLAMP(candidate,
					  -(int64_t)BLE_HID_PENDING_LIMIT,
					  (int64_t)BLE_HID_PENDING_LIMIT);
		} while (!atomic_cas(&pending_scroll, old_value, new_value));
		accepted = true;
	}
	k_mutex_unlock(&connection_mutex);

	if (accepted) {
		(void)k_work_reschedule(&scroll_work, K_NO_WAIT);
	}
}

bool ble_hid_is_connected(void)
{
	bool connected;

	k_mutex_lock(&connection_mutex, K_FOREVER);
	connected = active_connection != NULL;
	k_mutex_unlock(&connection_mutex);

	return connected;
}

void ble_hid_diagnostic_print(void)
{
	bool connected;
	bool high_resolution;
	bool current_report_mode;
	uint16_t interval;
	uint16_t latency;
	uint16_t timeout;
	bt_security_t security_level;
	uint32_t retries;

	k_mutex_lock(&connection_mutex, K_FOREVER);
	connected = active_connection != NULL;
	high_resolution = high_resolution_scroll_enabled;
	current_report_mode = report_mode;
	interval = active_interval;
	latency = active_latency;
	timeout = active_timeout;
	security_level = active_security_level;
	retries = scroll_retry_count;
	k_mutex_unlock(&connection_mutex);

	printk("ble: %s security=L%u report-mode=%s host-high-resolution=%s "
	       "descriptor-multiplier=x%u\r\n",
	       connected ? "connected" : "not-connected", security_level,
	       current_report_mode ? "yes" : "no",
	       high_resolution ? "enabled" : "disabled",
	       high_resolution ? BLE_HID_WHEEL_RESOLUTION_MULTIPLIER : 1U);
	printk("scroll resolution: mode=%u (%s), persistent=yes\r\n",
	       (unsigned int)scroll_resolution_get(),
	       scroll_resolution_name_get());
	if (interval != 0U) {
		const uint32_t interval_hundredths_ms = (uint32_t)interval * 125U;

		printk("ble timing: interval=%u.%02u ms latency=%u timeout=%u ms\r\n",
		       interval_hundredths_ms / 100U,
		       interval_hundredths_ms % 100U, latency,
		       (uint32_t)timeout * 10U);
	}
	printk("hid: pending=%d notification=%s retries=%u\r\n",
	       (int)atomic_get(&pending_scroll),
	       atomic_get(&notification_in_progress) ? "active" : "idle",
	       retries);
}
