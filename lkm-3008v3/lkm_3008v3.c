#include "lkm_3008v3.h"

#include <string.h>

#define LKM_3008V3_CMD_DISABLE             0x80U
#define LKM_3008V3_CMD_STOP                0x81U
#define LKM_3008V3_CMD_ENABLE              0x88U
#define LKM_3008V3_CMD_READ_ENCODER        0x90U
#define LKM_3008V3_CMD_READ_STATE_2        0x9CU
#define LKM_3008V3_CMD_REL_POSITION_SPEED  0xA8U

static void put_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
}

static void put_i32_le(uint8_t *dst, int32_t value)
{
    uint32_t raw = (uint32_t)value;
    dst[0] = (uint8_t)raw;
    dst[1] = (uint8_t)(raw >> 8);
    dst[2] = (uint8_t)(raw >> 16);
    dst[3] = (uint8_t)(raw >> 24);
}

static uint16_t get_u16_le(const uint8_t *src)
{
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static int16_t get_i16_le(const uint8_t *src)
{
    return (int16_t)get_u16_le(src);
}

static int32_t wrapped_delta(uint16_t from, uint16_t to)
{
    int32_t delta = (int32_t)to - (int32_t)from;
    if (delta > (int32_t)LKM_3008V3_ENCODER_COUNTS_PER_REV / 2) {
        delta -= LKM_3008V3_ENCODER_COUNTS_PER_REV;
    } else if (delta < -(int32_t)LKM_3008V3_ENCODER_COUNTS_PER_REV / 2) {
        delta += LKM_3008V3_ENCODER_COUNTS_PER_REV;
    }
    return delta;
}

static int send_command(struct lkm_3008v3 *motor, uint8_t command)
{
    uint8_t data[LKM_3008V3_CAN_FRAME_SIZE] = {0};
    if (motor == NULL || motor->config.send == NULL) {
        return LKM_3008V3_ERROR_ARGUMENT;
    }
    data[0] = command;
    return motor->config.send(motor->config.send_user, motor->config.can_id,
                              data, sizeof(data));
}

void lkm_3008v3_default_config(struct lkm_3008v3_config *config,
                             lkm_3008v3_can_send_fn send,
                             void *send_user)
{
    if (config == NULL) {
        return;
    }
    *config = (struct lkm_3008v3_config) {
        .can_id = LKM_3008V3_DEFAULT_CAN_ID,
        .mechanical_min_deg = 0.0f,
        .mechanical_max_deg = 180.0f,
        .center_raw = 8445U,
        .encoder_direction = 1,
        .counts_per_deg = 93.4083f,
        .center_deg = 90.0f,
        .send = send,
        .send_user = send_user
    };
}

int lkm_3008v3_init(struct lkm_3008v3 *motor,
                  const struct lkm_3008v3_config *config)
{
    if (motor == NULL || config == NULL || config->send == NULL ||
        config->can_id > 0x7FFU || config->encoder_direction == 0 ||
        config->counts_per_deg <= 0.0f ||
        config->mechanical_min_deg >= config->mechanical_max_deg) {
        return LKM_3008V3_ERROR_ARGUMENT;
    }
    memset(motor, 0, sizeof(*motor));
    motor->config = *config;
    return LKM_3008V3_OK;
}

int lkm_3008v3_enable(struct lkm_3008v3 *motor) { return send_command(motor, LKM_3008V3_CMD_ENABLE); }
int lkm_3008v3_disable(struct lkm_3008v3 *motor) { return send_command(motor, LKM_3008V3_CMD_DISABLE); }
int lkm_3008v3_stop(struct lkm_3008v3 *motor) { return send_command(motor, LKM_3008V3_CMD_STOP); }
int lkm_3008v3_read_encoder(struct lkm_3008v3 *motor) { return send_command(motor, LKM_3008V3_CMD_READ_ENCODER); }
int lkm_3008v3_read_state(struct lkm_3008v3 *motor) { return send_command(motor, LKM_3008V3_CMD_READ_STATE_2); }

int lkm_3008v3_set_position(struct lkm_3008v3 *motor,
                          float target_deg,
                          uint16_t max_speed_dps)
{
    uint8_t data[LKM_3008V3_CAN_FRAME_SIZE] = {0};
    float physical_delta;
    float motor_delta;
    int32_t command_angle;

    if (motor == NULL || motor->config.send == NULL) {
        return LKM_3008V3_ERROR_ARGUMENT;
    }
    if (target_deg < motor->config.mechanical_min_deg ||
        target_deg > motor->config.mechanical_max_deg || max_speed_dps == 0U) {
        return LKM_3008V3_ERROR_RANGE;
    }
    if (!motor->feedback_valid) {
        return LKM_3008V3_ERROR_NOT_READY;
    }

    physical_delta = target_deg - motor->angle_deg;
    motor_delta = (float)motor->config.encoder_direction * physical_delta;
    command_angle = (int32_t)(motor_delta * 100.0f +
                    (motor_delta >= 0.0f ? 0.5f : -0.5f));
    if (command_angle == 0) {
        return LKM_3008V3_OK;
    }
    data[0] = LKM_3008V3_CMD_REL_POSITION_SPEED;
    put_u16_le(&data[2], max_speed_dps);
    put_i32_le(&data[4], command_angle);
    return motor->config.send(motor->config.send_user, motor->config.can_id,
                              data, sizeof(data));
}

bool lkm_3008v3_process_frame(struct lkm_3008v3 *motor,
                            uint16_t standard_id,
                            const uint8_t *data,
                            size_t length,
                            int64_t timestamp_ms)
{
    int32_t delta;
    if (motor == NULL || data == NULL || length != LKM_3008V3_CAN_FRAME_SIZE ||
        standard_id != motor->config.can_id) {
        return false;
    }
    motor->last_command = data[0];
    if (data[0] == LKM_3008V3_CMD_READ_ENCODER) {
        motor->encoder_count = get_u16_le(&data[2]);
        motor->encoder_raw = get_u16_le(&data[4]);
        motor->encoder_offset = get_u16_le(&data[6]);
        if (motor->encoder_raw > LKM_3008V3_ENCODER_MAX_COUNT) {
            return false;
        }
        delta = wrapped_delta(motor->config.center_raw, motor->encoder_raw);
        motor->angle_deg = motor->config.center_deg +
            (float)motor->config.encoder_direction * (float)delta /
            motor->config.counts_per_deg;
        motor->last_feedback_ms = timestamp_ms;
        motor->feedback_valid = true;
    } else {
        motor->temperature_c = data[1];
        motor->power_raw = get_i16_le(&data[2]);
        motor->speed_dps = (float)get_i16_le(&data[4]);
        motor->encoder_count = get_u16_le(&data[6]);
    }
    motor->rx_count++;
    return true;
}

bool lkm_3008v3_feedback_fresh(const struct lkm_3008v3 *motor,
                             int64_t now_ms,
                             int64_t max_age_ms)
{
    return motor != NULL && motor->feedback_valid && max_age_ms >= 0 &&
           now_ms >= motor->last_feedback_ms &&
           now_ms - motor->last_feedback_ms <= max_age_ms;
}
