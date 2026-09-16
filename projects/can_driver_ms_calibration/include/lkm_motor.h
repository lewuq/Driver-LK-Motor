#ifndef LKM_MOTOR_H
#define LKM_MOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>

#define LKM_ENCODER_MAX_COUNT 32767U

struct lkm_motor {
	const struct device *can_dev;
	uint16_t can_id;
	const char *name;
	float hard_min_deg;
	float hard_max_deg;
	float angle_deg;
	uint16_t encoder_count;
	uint16_t encoder_raw;
	uint16_t encoder_offset;
	float speed_dps;
	int16_t power_raw;
	uint8_t temperature_c;
	uint8_t last_command;
	int64_t last_feedback_ms;
	bool feedback_valid;
	volatile bool tx_pending;
	volatile int last_tx_error;
	int64_t last_error_log_ms;
	uint32_t rx_count;
};

void lkm_motor_init(struct lkm_motor *motor, const struct device *can_dev,
		    uint16_t can_id, const char *name,
		    float hard_min_deg, float hard_max_deg);
/* The calibration transport intentionally exposes only the read-only encoder
 * query. Motion-related APIs do not exist in this project. */
int lkm_motor_read_encoder(struct lkm_motor *motor);
bool lkm_motor_process_frame(struct lkm_motor *motor,
			     const struct can_frame *frame, int64_t now_ms);
bool lkm_motor_feedback_fresh(const struct lkm_motor *motor, int64_t now_ms,
			      int64_t max_age_ms);
const char *lkm_can_error_name(int error);

#endif /* LKM_MOTOR_H */
