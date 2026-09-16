#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>
#include "lkm_motor.h"

enum calibration_step {
	CAL_YAW_MIN,
	CAL_YAW_CENTER,
	CAL_YAW_MAX,
	CAL_PITCH_MIN,
	CAL_PITCH_CENTER,
	CAL_PITCH_MAX,
	CAL_COMPLETE,
};

struct encoder_sample {
	bool valid;
	uint16_t encoder;
	uint16_t raw;
	uint16_t offset;
};

struct calibration_state {
	struct lkm_motor yaw;
	struct lkm_motor pitch;
	enum calibration_step step;
	struct encoder_sample samples[6];
	bool capture_pressed;
	bool query_pitch_next;
	int64_t started_ms;
	int64_t last_query_ms;
};

const char *calibration_step_name(enum calibration_step step);
bool calibration_capture(struct calibration_state *state);
void calibration_print_result(const struct calibration_state *state);

#endif
