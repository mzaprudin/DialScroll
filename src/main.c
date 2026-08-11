#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct device *const usb_cdc = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static void usb_cdc_irq_handler(const struct device *dev, void *user_data)
{
	unsigned char received[64];
	int count;

	ARG_UNUSED(user_data);

	if (!uart_irq_update(dev)) {
		return;
	}

	while (uart_irq_rx_ready(dev)) {
		count = uart_fifo_read(dev, received, sizeof(received));
		if (count <= 0) {
			break;
		}

		for (int index = 0; index < count; ++index) {
			received[index]++;
		}

		uart_fifo_fill(dev, received, count);
	}
}

static void blink_thread(void *arg1, void *arg2, void *arg3)
{
	int ret;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	if (!gpio_is_ready_dt(&led)) {
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return;
	}

	while (true) {
		for (int blink = 0; blink < 2; ++blink) {
			gpio_pin_set_dt(&led, 1);
			k_sleep(K_MSEC(250));

			gpio_pin_set_dt(&led, 0);
			k_sleep(K_MSEC(250));
		}

		k_sleep(K_SECONDS(2));
	}
}

K_THREAD_DEFINE(blink_thread_id, 512, blink_thread, NULL, NULL, NULL, 7, 0, 0);

int main(void)
{
	if (!device_is_ready(usb_cdc)) {
		return 0;
	}

	uart_irq_callback_user_data_set(usb_cdc, usb_cdc_irq_handler, NULL);
	uart_irq_rx_enable(usb_cdc);

	k_sleep(K_FOREVER);
}
