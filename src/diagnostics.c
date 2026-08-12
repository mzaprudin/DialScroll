#include "diagnostics.h"

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "battery.h"
#include "ble_hid.h"
#include "encoder.h"
#include "scroll_acceleration.h"
#include "scroll_resolution.h"

#if defined(CONFIG_BLESCROLL_RUNTIME_DIAGNOSTICS)
#include <nrfx_power.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/usb/usbd.h>

#define DIAGNOSTIC_LOG_BACKEND_NAME "log_backend_uart"
#define DIAGNOSTIC_USBD_CONTEXT_NAME "cdc_acm_serial"

static const struct device *diagnostic_uart;
static struct usbd_context *diagnostic_usbd;
static const struct log_backend *diagnostic_log_backend;
static atomic_t diagnostic_enabled;
static atomic_t usb_configured;

static void diagnostic_command_work_handler(struct k_work *work);

K_MSGQ_DEFINE(diagnostic_command_queue, sizeof(uint8_t), 16, 1);
K_WORK_DEFINE(diagnostic_command_work, diagnostic_command_work_handler);

static bool vbus_is_present(void)
{
	return nrfx_power_usbstatus_get() != NRFX_POWER_USB_STATE_DISCONNECTED;
}

static void diagnostic_snapshot_print(void)
{
	printk("uptime=%lld ms vbus=%s usb=%s\r\n",
	       (long long)k_uptime_get(),
	       vbus_is_present() ? "present" : "absent",
	       atomic_get(&usb_configured) ? "configured" : "not-configured");
	ble_hid_diagnostic_print();
	encoder_diagnostic_print();
	battery_diagnostic_print();
}

static int diagnostic_log_enable(void)
{
	if (diagnostic_log_backend == NULL) {
		return -ENODEV;
	}

	if (!diagnostic_log_backend->cb->initialized) {
		log_backend_init(diagnostic_log_backend);
	}
	if (!log_backend_is_active(diagnostic_log_backend)) {
		log_backend_enable(diagnostic_log_backend,
				   diagnostic_log_backend->cb->ctx,
				   LOG_LEVEL_INF);
	}

	return 0;
}

static void diagnostic_enable(void)
{
	if (!atomic_get(&usb_configured) || !vbus_is_present()) {
		return;
	}

	int err = diagnostic_log_enable();

	if (err < 0) {
		printk("\r\nDiagnostics could not start: log backend error %d\r\n",
		       err);
		return;
	}

	atomic_set(&diagnostic_enabled, 1);
	printk("\r\n=== blescroll diagnostics enabled ===\r\n");
	diagnostic_snapshot_print();
}

static void diagnostic_help_print(void)
{
	printk("\r\n=== blescroll commands ===\r\n");
	printk("D  enable diagnostics / print snapshot\r\n");
	printk("d  disable diagnostics\r\n");
	printk("h  print this help\r\n");
	printk("0  Wheel resolution off (legacy x1)\r\n");
	printk("1  Wheel resolution x8\r\n");
	printk("2  Wheel resolution x16 (default)\r\n");
	printk("A  enable x1..x2 scroll acceleration and save\r\n");
	printk("a  disable scroll acceleration and save (default)\r\n");
	printk("Current resolution: %u (%s)\r\n",
	       (unsigned int)scroll_resolution_get(),
	       scroll_resolution_name_get());
	printk("Current acceleration: %s; activation=45 deg, window=80 ms, "
	       "x1.00<=0.40 rps, linear, x2.00>=2.00 rps\r\n",
	       scroll_acceleration_name_get());
}

static void diagnostic_resolution_set(uint8_t command)
{
	const enum scroll_resolution_mode mode =
		(enum scroll_resolution_mode)(command - '0');
	const int err = scroll_resolution_set(mode);

	if (err < 0) {
		printk("\r\nScroll resolution change failed: %d\r\n", err);
		return;
	}

	printk("\r\nScroll resolution saved: %u (%s)\r\n",
	       (unsigned int)scroll_resolution_get(),
	       scroll_resolution_name_get());
}

static void diagnostic_acceleration_set(bool enabled)
{
	const int err = scroll_acceleration_set(enabled);

	if (err < 0) {
		printk("\r\nScroll acceleration change failed (NVS): %d\r\n", err);
		return;
	}

	printk("\r\nScroll acceleration saved: %s\r\n",
	       scroll_acceleration_name_get());
}

static void diagnostic_disable(bool acknowledge)
{
	if (acknowledge) {
		printk("\r\n=== blescroll diagnostics disabled ===\r\n");
	}
	atomic_set(&diagnostic_enabled, 0);

	if ((diagnostic_log_backend != NULL) &&
	    log_backend_is_active(diagnostic_log_backend)) {
		log_backend_disable(diagnostic_log_backend);
	}
}

static void diagnostic_command_work_handler(struct k_work *work)
{
	uint8_t command;

	ARG_UNUSED(work);

	while (k_msgq_get(&diagnostic_command_queue, &command, K_NO_WAIT) == 0) {
		/* A command may have been queued immediately before VBUS was
		 * removed. Never reactivate diagnostics after USB has gone away.
		 */
		if (!atomic_get(&usb_configured) || !vbus_is_present()) {
			diagnostic_disable(false);
			continue;
		}

		if (command == 'D') {
			diagnostic_enable();
		} else if (command == 'd') {
			diagnostic_disable(true);
		} else if (command == 'h') {
			diagnostic_help_print();
		} else if (command == 'A') {
			diagnostic_acceleration_set(true);
		} else if (command == 'a') {
			diagnostic_acceleration_set(false);
		} else if ((command >= '0') && (command <= '2')) {
			diagnostic_resolution_set(command);
		}
	}
}

static void diagnostic_uart_callback(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		uint8_t received[16];
		int length;

		if (!uart_irq_rx_ready(dev)) {
			continue;
		}

		length = uart_fifo_read(dev, received, sizeof(received));
		for (int index = 0; index < length; ++index) {
			if ((received[index] == 'D') || (received[index] == 'd') ||
			    (received[index] == 'h') ||
			    (received[index] == 'A') || (received[index] == 'a') ||
			    ((received[index] >= '0') && (received[index] <= '2'))) {
				if (k_msgq_put(&diagnostic_command_queue,
					       &received[index], K_NO_WAIT) == 0) {
					(void)k_work_submit(&diagnostic_command_work);
				}
			}
		}
	}
}

static void diagnostic_usb_message(struct usbd_context *const context,
				   const struct usbd_msg *const message)
{
	int err;

	switch (message->type) {
	case USBD_MSG_VBUS_READY:
		err = usbd_enable(context);
		if ((err < 0) && (err != -EALREADY)) {
			return;
		}
		break;

	case USBD_MSG_VBUS_REMOVED:
		diagnostic_disable(false);
		atomic_set(&usb_configured, 0);
		uart_irq_rx_disable(diagnostic_uart);
		k_msgq_purge(&diagnostic_command_queue);
		err = usbd_disable(context);
		ARG_UNUSED(err);
		break;

	case USBD_MSG_CONFIGURATION:
		atomic_set(&usb_configured, message->status != 0);
		if (message->status != 0) {
			uart_irq_rx_enable(diagnostic_uart);
		} else {
			diagnostic_disable(false);
			uart_irq_rx_disable(diagnostic_uart);
			k_msgq_purge(&diagnostic_command_queue);
		}
		break;

	default:
		break;
	}
}

static struct usbd_context *diagnostic_usbd_find(void)
{
	STRUCT_SECTION_FOREACH(usbd_context, context) {
		if (strcmp(context->name, DIAGNOSTIC_USBD_CONTEXT_NAME) == 0) {
			return context;
		}
	}

	return NULL;
}
#endif /* CONFIG_BLESCROLL_RUNTIME_DIAGNOSTICS */

int diagnostics_init(void)
{
#if defined(CONFIG_BLESCROLL_RUNTIME_DIAGNOSTICS)
	int err;

	diagnostic_uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	if (!device_is_ready(diagnostic_uart)) {
		return -ENODEV;
	}

	diagnostic_usbd = diagnostic_usbd_find();
	if (diagnostic_usbd == NULL) {
		return -ENODEV;
	}
	diagnostic_log_backend =
		log_backend_get_by_name(DIAGNOSTIC_LOG_BACKEND_NAME);
	if (diagnostic_log_backend == NULL) {
		return -ENODEV;
	}
	/* Runtime filters otherwise keep their compiled levels even while the
	 * only backend is inactive, which would waste CPU and fill the deferred
	 * log buffer. Keep every source at NONE until an uppercase D arrives.
	 */
	for (uint32_t source = 0U; source < log_src_cnt_get(0); ++source) {
		(void)log_filter_set(diagnostic_log_backend, 0, source,
				     LOG_LEVEL_NONE);
	}

	err = uart_irq_callback_user_data_set(diagnostic_uart,
					      diagnostic_uart_callback, NULL);
	if (err < 0) {
		return err;
	}

	err = usbd_msg_register_cb(diagnostic_usbd, diagnostic_usb_message);
	if (err < 0) {
		return err;
	}

	/* VBUS may already be present before main() registers the message
	 * callback. In that case the original ready event has already passed.
	 */
	if (vbus_is_present()) {
		err = usbd_enable(diagnostic_usbd);
		if ((err < 0) && (err != -EALREADY)) {
			return err;
		}
	}
#endif
	return 0;
}

bool diagnostics_is_enabled(void)
{
#if defined(CONFIG_BLESCROLL_RUNTIME_DIAGNOSTICS)
	return atomic_get(&diagnostic_enabled) != 0;
#else
	return false;
#endif
}
