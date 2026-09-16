/* SPDX-License-Identifier: Apache-2.0
 * Application composition for the dual-axis LK gimbal.
 *
 * The motor thread is the only execution context that drains CAN RX and
 * advances the gimbal state machine. The UI thread submits small in-memory
 * requests and renders snapshots outside the mutex, so display I/O cannot
 * delay feedback supervision or motion safety checks.
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
#include "gimbal.h"
#include "round_ui.h"
#include "touch.h"

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
static const struct device *const gpiob = DEVICE_DT_GET(DT_NODELABEL(gpiob));
#define CAN_STB_PIN 14U
/* CAN driver RX endpoint. Frames are consumed only by motor_task(). */
CAN_MSGQ_DEFINE(motor_rx_msgq, 32);
/* Protects app_gimbal while UI requests and motor state updates cross threads. */
K_MUTEX_DEFINE(gimbal_mutex);
K_THREAD_STACK_DEFINE(motor_thread_stack, 4096);
static struct k_thread motor_thread_data;
static struct gimbal app_gimbal;

static const char *can_state_name(enum can_state state)
{
	switch (state) {
	case CAN_STATE_ERROR_ACTIVE: return "ERROR_ACTIVE";
	case CAN_STATE_ERROR_WARNING: return "ERROR_WARNING";
	case CAN_STATE_ERROR_PASSIVE: return "ERROR_PASSIVE";
	case CAN_STATE_BUS_OFF: return "BUS_OFF";
	case CAN_STATE_STOPPED: return "STOPPED";
	default: return "UNKNOWN";
	}
}

static int force_can_transceiver_active(const char *stage)
{
	int level;
	int ret;
	if (!device_is_ready(gpiob)) return -ENODEV;
	ret = gpio_pin_configure(gpiob, CAN_STB_PIN, GPIO_OUTPUT_LOW);
	if (ret == 0) ret = gpio_pin_set_raw(gpiob, CAN_STB_PIN, 0);
	if (ret != 0) return ret;
	level = gpio_pin_get_raw(gpiob, CAN_STB_PIN);
	printk("CAN STB stage=%s PB14 raw=%d (%s)\n", stage, level,
	       level == 0 ? "ACTIVE" : "STANDBY - ERROR");
	return level == 0 ? 0 : (level < 0 ? level : -EIO);
}

static int configure_can(void)
{
	const struct can_filter filter = { .id = 0U, .mask = 0U, .flags = 0U };
	int ret;
	if (!device_is_ready(can_dev)) return -ENODEV;
	ret = force_can_transceiver_active("before-can-start");
	if (ret == 0) ret = can_set_mode(can_dev, CAN_MODE_NORMAL);
	if (ret == 0) ret = can_set_bitrate(can_dev, APP_CAN_BITRATE);
	if (ret == 0) ret = can_add_rx_filter_msgq(can_dev, &motor_rx_msgq, &filter);
	if (ret >= 0) ret = can_start(can_dev);
	if (ret == 0) ret = force_can_transceiver_active("after-can-start");
	if (ret != 0) {
		printk("CAN INIT ERROR ret=%d (%s)\n", ret, lkm_can_error_name(ret));
		return ret;
	}
	printk("CAN READY: CLASSIC NORMAL, fixed %u bps, fixed IDs 0x%03x/0x%03x\n",
	       APP_CAN_BITRATE, GIMBAL_YAW_CAN_ID, GIMBAL_PITCH_CAN_ID);
	return 0;
}

static void copy_gimbal_snapshot(struct gimbal *snapshot)
{
	/* Copying is fast; display rendering happens after the lock is released. */
	k_mutex_lock(&gimbal_mutex, K_FOREVER);
	*snapshot = app_gimbal;
	k_mutex_unlock(&gimbal_mutex);
}

static void motor_task(void *unused1, void *unused2, void *unused3)
{
	int64_t next_can_report = 0;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);
	while (true) {
		struct can_frame frame;
		struct gimbal snapshot;
		int64_t now_ms = k_uptime_get();

		/* Keep all CAN/state-machine ownership in this high-priority thread. */
		k_mutex_lock(&gimbal_mutex, K_FOREVER);
		while (k_msgq_get(&motor_rx_msgq, &frame, K_NO_WAIT) == 0) {
			gimbal_process_frame(&app_gimbal, &frame, now_ms);
		}
		gimbal_tick(&app_gimbal, now_ms);
		snapshot = app_gimbal;
		k_mutex_unlock(&gimbal_mutex);

		if (now_ms >= next_can_report) {
			enum can_state state;
			struct can_bus_err_cnt errors;
			int ret = can_get_state(can_dev, &state, &errors);

			if (ret == 0) {
				printk("CAN STATUS state=%s tx_err=%u rx_err=%u yaw_rx=%u pitch_rx=%u yaw=%.1f/%.1f pitch=%.1f/%.1f\n",
				       can_state_name(state), errors.tx_err_cnt,
				       errors.rx_err_cnt, snapshot.yaw.rx_count,
				       snapshot.pitch.rx_count,
				       (double)snapshot.yaw.angle_deg,
				       (double)snapshot.yaw_target_deg,
				       (double)snapshot.pitch.angle_deg,
				       (double)snapshot.pitch_target_deg);
			}
			next_can_report = now_ms + 2000;
		}
		k_msleep(APP_MOTOR_TASK_PERIOD_MS);
	}
}

int main(void)
{
	struct gimbal snapshot;
	struct touch_state touch = {0};
	int64_t next_ui_refresh = 0;
	uint32_t touch_errors = 0;
	bool can_ready = false;
	int ret;

	printk("XIAO STM32C5 dual LKM gimbal: fixed classic CAN 1 Mbps\n");
	printk("Yaw MS3506=0x%03x, Pitch MS3008=0x%03x; scan disabled\n",
	       GIMBAL_YAW_CAN_ID, GIMBAL_PITCH_CAN_ID);
	gimbal_init(&app_gimbal, can_dev);
	ret = round_ui_init();
	if (ret != 0) return ret;
	copy_gimbal_snapshot(&snapshot);
	(void)round_ui_render(&snapshot);
	k_msleep(100);
	ret = touch_init();
	if (ret != 0) gimbal_set_fault(&app_gimbal, "touch init failed");
	ret = configure_can();
	if (ret != 0) gimbal_set_fault(&app_gimbal, "CAN init failed");
	else can_ready = true;
	copy_gimbal_snapshot(&snapshot);
	(void)round_ui_render(&snapshot);
	printk("Control: LKM 0xA8 incremental internal position loop; touch START enables motion\n");
	printk("Motor task: independent %u ms non-blocking poll; UI never sends CAN\n",
	       APP_MOTOR_TASK_PERIOD_MS);
	printk("Motor startup guard: %u ms; fixed mechanical center yaw %.1f pitch %.1f\n",
	       GIMBAL_MOTOR_BOOT_DELAY_MS, (double)GIMBAL_YAW_CENTER_DEG,
	       (double)GIMBAL_PITCH_CENTER_DEG);
	if (can_ready) {
		(void)k_thread_create(&motor_thread_data, motor_thread_stack,
			K_THREAD_STACK_SIZEOF(motor_thread_stack), motor_task,
			NULL, NULL, NULL, K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
	}

	while (true) {
		int64_t now_ms = k_uptime_get();
		ret = touch_poll(&touch);
		if (ret == 0) {
			bool changed;

			touch_errors = 0;
			k_mutex_lock(&gimbal_mutex, K_FOREVER);
			changed = round_ui_handle_touch(&app_gimbal, &touch);
			snapshot = app_gimbal;
			k_mutex_unlock(&gimbal_mutex);
			if (changed) {
				(void)round_ui_render(&snapshot);
				next_ui_refresh = now_ms + APP_UI_REFRESH_MS;
			}
		} else if (++touch_errors == 1U || touch_errors % 100U == 0U) {
			printk("Touch read error: %d count=%u\n", ret, touch_errors);
		}
		if (now_ms >= next_ui_refresh) {
			copy_gimbal_snapshot(&snapshot);
			(void)round_ui_render(&snapshot);
			next_ui_refresh = now_ms + APP_UI_REFRESH_MS;
		}
		k_msleep(10);
	}
}
