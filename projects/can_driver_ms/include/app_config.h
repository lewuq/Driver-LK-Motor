#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define APP_CAN_BITRATE                 1000000U
#define APP_CONTROL_PERIOD_MS           50U
#define APP_MOTOR_TASK_PERIOD_MS        10U
#define APP_UI_REFRESH_MS               180U

/* LKM arbitration IDs are 0x140 + node ID. */
#define GIMBAL_YAW_CAN_ID               0x141U
#define GIMBAL_PITCH_CAN_ID             0x142U

#define GIMBAL_YAW_HARD_MIN_DEG         0.0f
#define GIMBAL_YAW_HARD_MAX_DEG         345.0f
#define GIMBAL_PITCH_HARD_MIN_DEG       0.0f
#define GIMBAL_PITCH_HARD_MAX_DEG       180.0f

/* Conservative working limits used by the touch UI. */
#define GIMBAL_YAW_CONTROL_MIN_DEG      1.0f
#define GIMBAL_YAW_CONTROL_MAX_DEG      344.0f
#define GIMBAL_PITCH_CONTROL_MIN_DEG    1.0f
#define GIMBAL_PITCH_CONTROL_MAX_DEG    175.0f

#define GIMBAL_YAW_CENTER_DEG           172.5f
#define GIMBAL_PITCH_CENTER_DEG         90.0f
/* Keep motion locked until raw min/center/max values are measured with
 * can_driver_ms_calibration and the relative mapping is applied here. */
#define GIMBAL_RELATIVE_CALIBRATION_VALID 1

/* Relative raw-encoder -> mechanical-degree mapping derived from two complete
 * min/max calibration runs. CENTER_RAW is the circular midpoint of the two
 * measured endpoints, not the less-repeatable manually captured center.
 * DIRECTION is +1 when raw counts increase with physical angle, -1 otherwise. */
#define GIMBAL_ENCODER_COUNTS_PER_REV   32768U
#define GIMBAL_YAW_MAP_CENTER_RAW       12178U
#define GIMBAL_YAW_MAP_DIRECTION        1
#define GIMBAL_YAW_MAP_COUNTS_PER_DEG   91.8087f
#define GIMBAL_YAW_MAP_CENTER_DEG       172.5f

#define GIMBAL_PITCH_MAP_CENTER_RAW     8445U
#define GIMBAL_PITCH_MAP_DIRECTION      1
#define GIMBAL_PITCH_MAP_COUNTS_PER_DEG 93.4083f
#define GIMBAL_PITCH_MAP_CENTER_DEG     90.0f

#define GIMBAL_HOME_SPEED_DPS           30U
#define GIMBAL_UI_SPEED_DPS             90U
#define GIMBAL_CENTER_TOLERANCE_DEG     2.0f
#define GIMBAL_CENTER_DIVERGENCE_DEG    3.0f
#define GIMBAL_LIMIT_FEEDBACK_MARGIN_DEG 1.0f
#define GIMBAL_TARGET_DEADBAND_DEG      2.0f
#define GIMBAL_IDLE_STOP_TOLERANCE_DEG  1.0f
#define GIMBAL_FEEDBACK_POLL_MS         100U
#define GIMBAL_START_FEEDBACK_MAX_AGE_MS 1000U
#define GIMBAL_FEEDBACK_TIMEOUT_MS      3000U
#define GIMBAL_CENTER_TIMEOUT_MS        20000U
#define GIMBAL_FEEDBACK_STALE_MS        2000U
#define GIMBAL_MOTOR_BOOT_DELAY_MS      2000U
#define GIMBAL_DISARM_PROBE_PERIOD_MS   500U
#define GIMBAL_FEEDBACK_QUERY_GAP_MS    50U
#define ROUND_LCD_WIDTH                 240
#define ROUND_LCD_HEIGHT                240
#define TOUCH_I2C_ADDRESS               0x2eU

#endif /* APP_CONFIG_H */
