/* SPDX-License-Identifier: Apache-2.0
 * Read-only raw encoder calibration for a reCamera-style LK gimbal.
 *
 * The main loop drains only 0x90 replies, advances the guided capture flow,
 * polls touch input, and refreshes the display. No motor-control API is linked
 * into this project.
 */
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "app_config.h"
#include "calibration.h"
#include "round_ui.h"
#include "touch.h"

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
static const struct device *const gpiob = DEVICE_DT_GET(DT_NODELABEL(gpiob));
#define CAN_STB_PIN 14U
CAN_MSGQ_DEFINE(motor_rx_msgq, 32);

static int configure_can(void)
{
	const struct can_filter filter = { .id = 0U, .mask = 0U, .flags = 0U };
	int ret;

	if (!device_is_ready(can_dev) || !device_is_ready(gpiob)) return -ENODEV;
	ret = gpio_pin_configure(gpiob, CAN_STB_PIN, GPIO_OUTPUT_LOW);
	if (ret == 0) ret = gpio_pin_set_raw(gpiob, CAN_STB_PIN, 0);
	if (ret == 0) ret = can_set_mode(can_dev, CAN_MODE_NORMAL);
	if (ret == 0) ret = can_set_bitrate(can_dev, APP_CAN_BITRATE);
	if (ret == 0) ret = can_add_rx_filter_msgq(can_dev, &motor_rx_msgq, &filter);
	if (ret >= 0) ret = can_start(can_dev);
	if (ret != 0) printk("CAN INIT ERROR ret=%d (%s)\n", ret, lkm_can_error_name(ret));
	else printk("CAN READY: fixed classic 1 Mbps, read-only calibration\n");
	return ret;
}

int main(void)
{
	struct calibration_state state = {0};
	struct touch_state touch = {0};
	int64_t next_ui = 0;
	int64_t next_log = 0;
	bool can_ready = false;
	int ret;

	printk("LKM GIMBAL RAW ENCODER CALIBRATION - READ ONLY\n");
	printk("IDs: YAW MS3506=0x%03x PITCH MS3008=0x%03x\n", YAW_CAN_ID, PITCH_CAN_ID);
	printk("SAFETY: firmware never sends 0x88, 0xA4, 0xA8, or any motion command\n");
	lkm_motor_init(&state.yaw, can_dev, YAW_CAN_ID, "YAW MS3506", 0.0f, 360.0f);
	lkm_motor_init(&state.pitch, can_dev, PITCH_CAN_ID, "PITCH MS3008", 0.0f, 360.0f);
	state.step = CAL_YAW_MIN;
	ret = round_ui_init();
	if (ret != 0) return ret;
	(void)round_ui_render(&state);
	k_msleep(100);
	ret = touch_init();
	if (ret != 0) printk("TOUCH INIT ERROR ret=%d\n", ret);
	ret = configure_can();
	if (ret == 0) can_ready = true;
	state.started_ms = k_uptime_get();
	state.last_query_ms = state.started_ms;
	printk("Wait %u ms for both motor controllers, then alternate read-only 0x90 queries\n",
	       MOTOR_BOOT_DELAY_MS);

	while (true) {
		struct can_frame frame;
		int64_t now = k_uptime_get();

		while (k_msgq_get(&motor_rx_msgq, &frame, K_NO_WAIT) == 0) {
			(void)lkm_motor_process_frame(&state.yaw, &frame, now);
			(void)lkm_motor_process_frame(&state.pitch, &frame, now);
		}
		if (can_ready && now - state.started_ms >= MOTOR_BOOT_DELAY_MS &&
		    now - state.last_query_ms >= ENCODER_QUERY_GAP_MS) {
			struct lkm_motor *motor = state.query_pitch_next ? &state.pitch : &state.yaw;
			if (!motor->tx_pending) (void)lkm_motor_read_encoder(motor);
			state.query_pitch_next = !state.query_pitch_next;
			state.last_query_ms = now;
		}
		ret = touch_poll(&touch);
		if (ret == 0 && round_ui_handle_touch(&state, &touch)) {
			(void)round_ui_render(&state);
			next_ui = now + UI_REFRESH_MS;
		}
		if (now >= next_log) {
			printk("CAL LIVE step=%s yaw(enc=%u raw=%u off=%u rx=%u) pitch(enc=%u raw=%u off=%u rx=%u)\n",
			       calibration_step_name(state.step), state.yaw.encoder_count,
			       state.yaw.encoder_raw, state.yaw.encoder_offset, state.yaw.rx_count,
			       state.pitch.encoder_count, state.pitch.encoder_raw,
			       state.pitch.encoder_offset, state.pitch.rx_count);
			next_log = now + 1000;
		}
		if (now >= next_ui) {
			(void)round_ui_render(&state);
			next_ui = now + UI_REFRESH_MS;
		}
		k_msleep(10);
	}
}
