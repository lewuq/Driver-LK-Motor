#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define APP_CAN_BITRATE               1000000U
#define YAW_CAN_ID                    0x141U
#define PITCH_CAN_ID                  0x142U
#define MOTOR_BOOT_DELAY_MS           2000U
#define ENCODER_QUERY_GAP_MS          100U
#define UI_REFRESH_MS                 150U
#define ENCODER_COUNTS_PER_REV        32768U

#define ROUND_LCD_WIDTH               240
#define ROUND_LCD_HEIGHT              240
#define TOUCH_I2C_ADDRESS             0x2eU

#endif
