/* SPDX-License-Identifier: Apache-2.0
 * Six-point, operator-driven circular encoder calibration.
 *
 * Endpoint-derived midpoints are emitted as configuration because physical
 * stops are more repeatable than a visually positioned manual center. The
 * captured center remains in diagnostics to reveal setup asymmetry or error.
 */
#include "calibration.h"

#include <math.h>
#include <zephyr/sys/printk.h>
#include "app_config.h"

static int32_t wrapped_delta(uint16_t from, uint16_t to)
{
	/* Return the shortest signed displacement on a 15-bit circular encoder. */
	int32_t delta = (int32_t)to - (int32_t)from;

	if (delta > (int32_t)ENCODER_COUNTS_PER_REV / 2) {
		delta -= ENCODER_COUNTS_PER_REV;
	} else if (delta < -(int32_t)ENCODER_COUNTS_PER_REV / 2) {
		delta += ENCODER_COUNTS_PER_REV;
	}
	return delta;
}

static uint16_t wrapped_count(int32_t value)
{
	value %= ENCODER_COUNTS_PER_REV;
	if (value < 0) {
		value += ENCODER_COUNTS_PER_REV;
	}
	return (uint16_t)value;
}

const char *calibration_step_name(enum calibration_step step)
{
	switch (step) {
	case CAL_YAW_MIN: return "MOVE YAW TO MIN";
	case CAL_YAW_CENTER: return "MOVE YAW TO CENTER";
	case CAL_YAW_MAX: return "MOVE YAW TO MAX";
	case CAL_PITCH_MIN: return "MOVE PITCH TO MIN";
	case CAL_PITCH_CENTER: return "MOVE PITCH TO CENTER";
	case CAL_PITCH_MAX: return "MOVE PITCH TO MAX";
	case CAL_COMPLETE: return "CALIBRATION COMPLETE";
	default: return "UNKNOWN";
	}
}

bool calibration_capture(struct calibration_state *state)
{
	struct lkm_motor *motor;
	struct encoder_sample *sample;

	if (state->step == CAL_COMPLETE) {
		state->step = CAL_YAW_MIN;
		for (size_t i = 0; i < 6; i++) state->samples[i].valid = false;
		printk("CAL RESET: begin again at YAW MIN\n");
		return true;
	}
	motor = state->step <= CAL_YAW_MAX ? &state->yaw : &state->pitch;
	if (!motor->feedback_valid || motor->last_command != 0x90U) {
		printk("CAL CAPTURE BLOCKED step=%s: no fresh 0x90 feedback\n",
		       calibration_step_name(state->step));
		return false;
	}
	sample = &state->samples[state->step];
	sample->valid = true;
	sample->encoder = motor->encoder_count;
	sample->raw = motor->encoder_raw;
	sample->offset = motor->encoder_offset;
	printk("CAL CAPTURE step=%s motor=%s id=0x%03x encoder_field=%u raw=%u offset_field=%u hex_raw=0x%04x (mapping_uses_raw_only)\n",
	       calibration_step_name(state->step), motor->name, motor->can_id,
	       sample->encoder, sample->raw, sample->offset, sample->raw);
	state->step++;
	if (state->step == CAL_COMPLETE) calibration_print_result(state);
	return true;
}

static void print_axis_result(const char *name,
			      const struct encoder_sample *minimum,
			      const struct encoder_sample *center,
			      const struct encoder_sample *maximum,
			      float physical_span_deg, float center_deg)
{
	/* The two signed half-spans establish direction and verify that all three
	 * points lie on one continuous mechanical path around the count wrap. */
	int32_t first_half = wrapped_delta(minimum->raw, center->raw);
	int32_t second_half = wrapped_delta(center->raw, maximum->raw);
	int32_t span = first_half + second_half;
	int32_t rounded_half_span = span >= 0 ? (span + 1) / 2 : (span - 1) / 2;
	uint16_t derived_center_raw =
		wrapped_count((int32_t)minimum->raw + rounded_half_span);
	int direction = span >= 0 ? 1 : -1;
	float counts_per_degree = fabsf((float)span) / physical_span_deg;
	float midpoint_error_deg = counts_per_degree > 0.0f ?
		((float)(first_half - second_half) * 0.5f) /
		(counts_per_degree * (float)direction) : 0.0f;

	printk("CAL RESULT %s min_raw=%u captured_center_raw=%u derived_center_raw=%u max_raw=%u d_min_center=%d d_center_max=%d span=%d direction=%+d counts_per_deg=%.4f midpoint_error_deg=%.2f\n",
	       name, minimum->raw, center->raw, derived_center_raw, maximum->raw,
	       first_half, second_half, span, direction,
	       (double)counts_per_degree, (double)midpoint_error_deg);
	printk("CAL CONFIG %s_CENTER_RAW=%u %s_DIRECTION=%d %s_COUNTS_PER_DEG=%.4f %s_CENTER_DEG=%.1f\n",
	       name, derived_center_raw, name, direction, name,
	       (double)counts_per_degree, name, (double)center_deg);
	if ((first_half > 0) != (second_half > 0) || counts_per_degree < 50.0f ||
	    fabsf(midpoint_error_deg) > 2.0f) {
		printk("CAL WARNING %s samples are inconsistent; repeat capture without crossing a wrong mechanical path\n",
		       name);
	}
}

void calibration_print_result(const struct calibration_state *state)
{
	for (size_t i = 0; i < 6; i++) {
		if (!state->samples[i].valid) return;
	}
	printk("CAL COMPLETE: READ-ONLY; no motor enable or motion command was sent\n");
	print_axis_result("YAW", &state->samples[CAL_YAW_MIN],
		&state->samples[CAL_YAW_CENTER], &state->samples[CAL_YAW_MAX],
		345.0f, 172.5f);
	print_axis_result("PITCH", &state->samples[CAL_PITCH_MIN],
		&state->samples[CAL_PITCH_CENTER], &state->samples[CAL_PITCH_MAX],
		180.0f, 90.0f);
}
