#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include "ble_hid.h"
#include "battery.h"
#include "diagnostics.h"
#include "encoder.h"
#include "scroll_resolution.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

static struct k_sem diagnostic_trigger_sem;
static unsigned char diagnostic_trigger_byte;

static void diagnostic_uart_callback(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (!uart_irq_rx_ready(dev)) {
			continue;
		}

		unsigned char received;
		int read = uart_fifo_read(dev, &received, 1);
		if (read > 0) {
			diagnostic_trigger_byte = received;
			uart_irq_rx_disable(dev);
			k_sem_give(&diagnostic_trigger_sem);
			return;
		}
	}
}

static int diagnostic_start_wait(void)
{
	const struct device *const console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	int err;

	if (!IS_ENABLED(CONFIG_BLESCROLL_DIAGNOSTIC_START_GATE)) {
		return 0;
	}

	if (!device_is_ready(console)) {
		return -ENODEV;
	}

	k_sem_init(&diagnostic_trigger_sem, 0, 1);
	err = uart_irq_callback_user_data_set(console, diagnostic_uart_callback, NULL);
	if (err < 0) {
		return err;
	}

	printk("\r\n=== blescroll diagnostic startup ===\r\n");
	printk("Waiting for USB CDC trigger byte...\r\n");
	uart_irq_rx_enable(console);

	k_sem_take(&diagnostic_trigger_sem, K_FOREVER);
	printk("USB trigger received: 0x%02x\r\n", diagnostic_trigger_byte);
	return 0;
}

static int startup_diagnostic_trigger_prepare(void)
{
	const struct device *const console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	int err;

	if (!device_is_ready(console)) {
		return -ENODEV;
	}

	k_sem_init(&diagnostic_trigger_sem, 0, 1);
	err = uart_irq_callback_user_data_set(console, diagnostic_uart_callback, NULL);
	if (err < 0) {
		return err;
	}

	return 0;
}

static void startup_diagnostic_trigger_wait(void)
{
	printk("Send any byte to capture the first 50 ms after encoder power-on...\r\n");
	uart_irq_rx_enable(DEVICE_DT_GET(DT_CHOSEN(zephyr_console)));
	k_sem_take(&diagnostic_trigger_sem, K_FOREVER);
}

static bool bluetooth_should_start(void)
{
	return !IS_ENABLED(CONFIG_BLESCROLL_DIAGNOSTIC_MODE) ||
	       IS_ENABLED(CONFIG_BLESCROLL_DIAGNOSTIC_BLE);
}

static int bluetooth_and_battery_start(void)
{
	int err;

	if (!bluetooth_should_start()) {
		printk("Bluetooth disabled by diagnostic configuration\r\n");
		return 0;
	}

	if (IS_ENABLED(CONFIG_BLESCROLL_DIAGNOSTIC_MODE)) {
		printk("main: starting BLE initialization\r\n");
	}
	err = ble_hid_init();
	if (err < 0) {
		LOG_ERR("BLE startup failed: %d", err);
		return err;
	}

	err = battery_init();
	if (err < 0) {
		LOG_ERR("Battery monitor initialization failed: %d", err);
	}

	return 0;
}

int main(void)
{
	int err;

	err = diagnostics_init();
	if (err < 0) {
		LOG_ERR("Runtime diagnostics initialization failed: %d", err);
	}

	err = scroll_resolution_init();
	if (err < 0) {
		LOG_ERR("Scroll resolution initialization failed: %d", err);
	}

	if (IS_ENABLED(CONFIG_BLESCROLL_ENCODER_STARTUP_DIAGNOSTIC)) {
		err = startup_diagnostic_trigger_prepare();
		if (err < 0) {
			printk("startup diagnostic trigger setup failed: %d\r\n", err);
			return err;
		}

		err = bluetooth_and_battery_start();
		if (err < 0) {
			return err;
		}

		err = encoder_init();
		if (err < 0) {
			printk("encoder startup failed: %d\r\n", err);
			return err;
		}

		printk("\r\n=== MT6701 startup diagnostic ===\r\n");
		printk("Bluetooth during diagnostics: %s\r\n",
		       bluetooth_should_start() ? "enabled" : "disabled");
		while (true) {
			startup_diagnostic_trigger_wait();
			encoder_startup_diagnostic_run();
		}
	}

	err = diagnostic_start_wait();
	if (err < 0) {
		printk("diagnostic_start_wait: %d\r\n", err);
		return err;
	}

	err = bluetooth_and_battery_start();
	if (err < 0) {
		return err;
	}

	LOG_INF("Starting encoder initialization");
	err = encoder_init();
	if (err < 0) {
		LOG_ERR("Encoder startup failed: %d", err);
		return err;
	}

	LOG_INF("Startup complete");
	encoder_run();
	return 0;
}
