/* SPDX-License-Identifier: Apache-2.0
 * Non-blocking CHSC6X polling for the 240x240 round display.
 * Raw controller reports are converted into edge events (down/up) plus the
 * latest coordinate; higher layers decide whether a button or slider owns it.
 */

#include "touch.h"

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "app_config.h"

static const struct device *const touch_i2c_dev =
	DEVICE_DT_GET(DT_NODELABEL(i2c1));

/* Round Display: D7/PA10 = active-low TP_INT, D0/PA0 = shared reset. */
static const struct gpio_dt_spec touch_irq = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpioa)),
	.pin = 10,
	.dt_flags = GPIO_ACTIVE_LOW | GPIO_PULL_UP,
};

static const struct gpio_dt_spec touch_reset = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpioa)),
	.pin = 0,
	.dt_flags = 0,
};

int touch_init(void)
{
	int ret;

	if (!device_is_ready(touch_i2c_dev) || !gpio_is_ready_dt(&touch_irq) ||
	    !gpio_is_ready_dt(&touch_reset)) {
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&touch_reset, GPIO_OUTPUT_HIGH);
	if (ret != 0) return ret;
	(void)gpio_pin_set_dt(&touch_reset, 0);
	k_msleep(20);
	(void)gpio_pin_set_dt(&touch_reset, 1);
	k_msleep(30);

	ret = gpio_pin_configure_dt(&touch_irq, GPIO_INPUT);
	if (ret != 0) return ret;
	ret = i2c_configure(touch_i2c_dev,
		I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_STANDARD));
	if (ret == 0) {
		printk("CHSC6X touch ready at 0x%02x\n", TOUCH_I2C_ADDRESS);
	}
	return ret;
}

int touch_poll(struct touch_state *state)
{
	uint8_t point[5];
	int active;
	int ret;

	state->was_pressed = state->pressed;
	state->just_pressed = false;
	state->just_released = false;
	active = gpio_pin_get_dt(&touch_irq);
	if (active < 0) return active;

	if (active == 0) {
		state->pressed = false;
		state->just_released = state->was_pressed;
		if (state->just_released) {
			printk("TOUCH UP last_x=%u last_y=%u\n", state->x, state->y);
		}
		return 0;
	}

	/* CHSC6X point protocol: plain five-byte read, no register address. */
	ret = i2c_read(touch_i2c_dev, point, sizeof(point), TOUCH_I2C_ADDRESS);
	if (ret != 0) return ret;
	if (point[0] != 0x01U) {
		state->pressed = false;
		state->just_released = state->was_pressed;
		return 0;
	}

	state->pressed = true;
	state->just_pressed = !state->was_pressed;
	state->x = point[2];
	state->y = point[4];
	if (state->just_pressed) {
		printk("TOUCH DOWN x=%u y=%u raw=%02x.%02x.%02x.%02x.%02x\n",
		       state->x, state->y, point[0], point[1], point[2],
		       point[3], point[4]);
	}
	return 0;
}
