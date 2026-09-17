#ifndef LKM_3506V3_H
#define LKM_3506V3_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LKM_3506V3_DEFAULT_CAN_ID          0x141U
#define LKM_3506V3_CAN_FRAME_SIZE          8U
#define LKM_3506V3_ENCODER_COUNTS_PER_REV  32768U
#define LKM_3506V3_ENCODER_MAX_COUNT       32767U

enum lkm_3506v3_result {
    LKM_3506V3_OK = 0,
    LKM_3506V3_ERROR_ARGUMENT = -1,
    LKM_3506V3_ERROR_NOT_READY = -2,
    LKM_3506V3_ERROR_RANGE = -3
};

/* Implement this callback with the target board's classic-CAN driver.
 * data is valid only for the duration of the callback; an asynchronous CAN
 * adapter must copy the eight bytes into its own queue before returning. */
typedef int (*lkm_3506v3_can_send_fn)(void *user,
                                      uint16_t standard_id,
                                      const uint8_t data[LKM_3506V3_CAN_FRAME_SIZE],
                                      size_t length);

struct lkm_3506v3_config {
    uint16_t can_id;
    float mechanical_min_deg;
    float mechanical_max_deg;
    uint16_t center_raw;
    int8_t encoder_direction;
    float counts_per_deg;
    float center_deg;
    lkm_3506v3_can_send_fn send;
    void *send_user;
};

struct lkm_3506v3 {
    struct lkm_3506v3_config config;
    float angle_deg;
    float speed_dps;
    int16_t power_raw;
    uint16_t encoder_count;
    uint16_t encoder_raw;
    uint16_t encoder_offset;
    uint8_t temperature_c;
    uint8_t last_command;
    uint32_t rx_count;
    int64_t last_feedback_ms;
    bool feedback_valid;
};

void lkm_3506v3_default_config(struct lkm_3506v3_config *config,
                               lkm_3506v3_can_send_fn send,
                               void *send_user);
int lkm_3506v3_init(struct lkm_3506v3 *motor,
                    const struct lkm_3506v3_config *config);
int lkm_3506v3_enable(struct lkm_3506v3 *motor);
int lkm_3506v3_disable(struct lkm_3506v3 *motor);
int lkm_3506v3_stop(struct lkm_3506v3 *motor);
int lkm_3506v3_read_encoder(struct lkm_3506v3 *motor);
int lkm_3506v3_read_state(struct lkm_3506v3 *motor);
int lkm_3506v3_set_position(struct lkm_3506v3 *motor,
                            float target_deg,
                            uint16_t max_speed_dps);
bool lkm_3506v3_process_frame(struct lkm_3506v3 *motor,
                              uint16_t standard_id,
                              const uint8_t *data,
                              size_t length,
                              int64_t timestamp_ms);
bool lkm_3506v3_feedback_fresh(const struct lkm_3506v3 *motor,
                               int64_t now_ms,
                               int64_t max_age_ms);

#ifdef __cplusplus
}
#endif

#endif /* LKM_3506V3_H */
