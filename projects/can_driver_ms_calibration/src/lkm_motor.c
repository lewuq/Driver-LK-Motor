/* SPDX-License-Identifier: Apache-2.0
 * Read-only LK CAN transport used by the calibration firmware.
 *
 * This translation unit deliberately contains only command 0x90. Keeping all
 * enable/stop/position opcodes out of the binary makes accidental motion a
 * compile-time design error rather than a runtime convention.
 */

#include "lkm_motor.h"

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define LKM_CMD_READ_ENCODER          0x90U

static uint16_t get_u16_le(const uint8_t *src)
{
	return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static void tx_complete(const struct device *dev, int error, void *user_data)
{
	struct lkm_motor *motor = user_data;

	ARG_UNUSED(dev);
	motor->last_tx_error = error;
	motor->tx_pending = false;
}

static int send_frame(struct lkm_motor *motor, const uint8_t data[8])
{
	struct can_frame frame = {
		.id = motor->can_id,
		.dlc = 8,
		.flags = 0U,
	};

	int ret;
	int64_t now_ms;

	/* Keep at most one outstanding frame per motor. This bounds the total
	 * number of FDCAN TX buffers used when a node is still booting or absent. */
	if (motor->tx_pending) {
		return -EBUSY;
	}
	memcpy(frame.data, data, 8U);
	/* A NULL callback makes Zephyr wait for bus ACK. Keep the UI loop alive
	 * when the bus is disconnected by always using asynchronous transmission.
	 */
	motor->tx_pending = true;
	ret = can_send(motor->can_dev, &frame, K_NO_WAIT, tx_complete, motor);
	if (ret != 0) {
		motor->tx_pending = false;
	}
	now_ms = k_uptime_get();
	if (ret != 0 && (ret != motor->last_tx_error ||
	    now_ms - motor->last_error_log_ms >= 1000)) {
		printk("CAN TX ERROR motor=%s id=0x%03x cmd=0x%02x ret=%d (%s)\n",
		       motor->name, motor->can_id, data[0], ret,
		       lkm_can_error_name(ret));
		motor->last_error_log_ms = now_ms;
	}
	if (ret == 0 && motor->last_tx_error != 0) {
		printk("CAN TX RECOVERED motor=%s id=0x%03x cmd=0x%02x\n",
		       motor->name, motor->can_id, data[0]);
	}
	motor->last_tx_error = ret;
	return ret;
}

const char *lkm_can_error_name(int error)
{
	switch (error) {
	case 0: return "OK";
	case -EAGAIN: return "NO_TX_BUFFER";
	case -EINVAL: return "INVALID_FRAME";
	case -ENETDOWN: return "CAN_STOPPED";
	case -ENETUNREACH: return "CAN_BUS_OFF";
	case -ETIMEDOUT: return "TX_TIMEOUT_OR_NO_ACK";
	case -EBUSY: return "TX_ALREADY_PENDING";
	case -EIO: return "TX_FAILED_OR_NO_ACK";
	case -ENODEV: return "CAN_DEVICE_NOT_READY";
	case -ENOTSUP: return "MODE_NOT_SUPPORTED";
	default: return "CAN_DRIVER_ERROR";
	}
}

static int send_simple_command(struct lkm_motor *motor, uint8_t command)
{
	uint8_t data[8] = {0};
	data[0] = command;
	return send_frame(motor, data);
}

void lkm_motor_init(struct lkm_motor *motor, const struct device *can_dev,
		    uint16_t can_id, const char *name,
		    float hard_min_deg, float hard_max_deg)
{
	memset(motor, 0, sizeof(*motor));
	motor->can_dev = can_dev;
	motor->can_id = can_id;
	motor->name = name;
	motor->hard_min_deg = hard_min_deg;
	motor->hard_max_deg = hard_max_deg;
}

int lkm_motor_read_encoder(struct lkm_motor *motor)
{
	return send_simple_command(motor, LKM_CMD_READ_ENCODER);
}

bool lkm_motor_process_frame(struct lkm_motor *motor,
			     const struct can_frame *frame, int64_t now_ms)
{
	uint16_t encoder_raw;

	if ((frame->flags & CAN_FRAME_IDE) != 0U || frame->id != motor->can_id ||
	    can_dlc_to_bytes(frame->dlc) != 8U ||
	    frame->data[0] != LKM_CMD_READ_ENCODER) {
		return false;
	}

	motor->last_command = frame->data[0];
	encoder_raw = get_u16_le(&frame->data[2]);
	motor->encoder_count = encoder_raw;
	motor->encoder_raw = get_u16_le(&frame->data[4]);
	motor->encoder_offset = get_u16_le(&frame->data[6]);

	if (encoder_raw > LKM_ENCODER_MAX_COUNT) {
		return false;
	}
	motor->angle_deg = ((float)encoder_raw * 360.0f) / 32768.0f;
	motor->last_feedback_ms = now_ms;
	motor->feedback_valid = true;
	motor->rx_count++;
	if (motor->rx_count == 1U) {
		printk("CAN RX OK motor=%s id=0x%03x data=%02x.%02x.%02x.%02x.%02x.%02x.%02x.%02x encoder=%u raw=%u offset=%u\n",
		       motor->name, motor->can_id, frame->data[0], frame->data[1],
		       frame->data[2], frame->data[3], frame->data[4], frame->data[5],
		       frame->data[6], frame->data[7], motor->encoder_count,
		       motor->encoder_raw, motor->encoder_offset);
		uint16_t expected_encoder =
			(uint16_t)((motor->encoder_raw - motor->encoder_offset) &
				   LKM_ENCODER_MAX_COUNT);
		if (motor->encoder_count != expected_encoder) {
			printk("CAN 0x90 FORMAT WARNING motor=%s reported_encoder=%u expected_from_raw_offset=%u; use encoderRaw only for calibration\n",
			       motor->name, motor->encoder_count, expected_encoder);
		}
	}
	return true;
}

bool lkm_motor_feedback_fresh(const struct lkm_motor *motor, int64_t now_ms,
			      int64_t max_age_ms)
{
	return motor->feedback_valid && now_ms - motor->last_feedback_ms <= max_age_ms;
}
