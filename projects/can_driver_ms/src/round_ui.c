/* SPDX-License-Identifier: Apache-2.0
 * Round-display renderer and touch-to-event mapping.
 *
 * This module does not perform CAN I/O. Button handlers queue state-machine
 * requests and sliders update targets; the motor thread executes them later.
 */

#include "round_ui.h"

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include "app_config.h"

#define COLOR_WHITE       0xffffU
#define COLOR_RED         0xf800U
/* Main palette: Seeed green #8FC31F and deep teal #004966. */
#define COLOR_PRIMARY     0x8e03U
#define COLOR_SECONDARY   0x024cU

#define SLIDER_X          30
#define SLIDER_WIDTH      180
#define SLIDER_HEIGHT     10
#define YAW_SLIDER_Y      80
#define PITCH_SLIDER_Y    154
#define SLIDER_TOUCH_PAD  16
#define CAL_BUTTON_X      55
#define CAL_BUTTON_Y      186
#define CAL_BUTTON_WIDTH  130
#define CAL_BUTTON_HEIGHT 30

static const struct device *const display_dev =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
static const struct pwm_dt_spec backlight =
	PWM_DT_SPEC_GET(DT_NODELABEL(display_backlight));

static uint16_t framebuffer[ROUND_LCD_WIDTH * ROUND_LCD_HEIGHT];
static bool action_button_pressed;
static const struct display_buffer_descriptor frame_desc = {
	.buf_size = sizeof(framebuffer),
	.width = ROUND_LCD_WIDTH,
	.height = ROUND_LCD_HEIGHT,
	.pitch = ROUND_LCD_WIDTH,
};

static uint16_t rgb565_be(uint16_t color)
{
	return (uint16_t)((color >> 8) | (color << 8));
}

static void set_pixel(int x, int y, uint16_t color)
{
	if ((unsigned int)x < ROUND_LCD_WIDTH && (unsigned int)y < ROUND_LCD_HEIGHT) {
		framebuffer[y * ROUND_LCD_WIDTH + x] = rgb565_be(color);
	}
}

static void clear_frame(uint16_t color)
{
	uint16_t value = rgb565_be(color);
	for (size_t i = 0; i < ARRAY_SIZE(framebuffer); ++i) framebuffer[i] = value;
}

static void fill_rect(int x, int y, int width, int height, uint16_t color)
{
	for (int py = y; py < y + height; ++py) {
		for (int px = x; px < x + width; ++px) set_pixel(px, py, color);
	}
}

static void draw_circle(int cx, int cy, int radius, uint16_t color)
{
	int x = radius;
	int y = 0;
	int error = 1 - radius;
	while (x >= y) {
		set_pixel(cx + x, cy + y, color); set_pixel(cx + y, cy + x, color);
		set_pixel(cx - y, cy + x, color); set_pixel(cx - x, cy + y, color);
		set_pixel(cx - x, cy - y, color); set_pixel(cx - y, cy - x, color);
		set_pixel(cx + y, cy - x, color); set_pixel(cx + x, cy - y, color);
		y++;
		if (error < 0) error += 2 * y + 1;
		else { x--; error += 2 * (y - x) + 1; }
	}
}

static void glyph5x7(char c, uint8_t glyph[5])
{
	static const uint8_t digits[10][5] = {
		{0x3e,0x51,0x49,0x45,0x3e},{0x00,0x42,0x7f,0x40,0x00},
		{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4b,0x31},
		{0x18,0x14,0x12,0x7f,0x10},{0x27,0x45,0x45,0x45,0x39},
		{0x3c,0x4a,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
		{0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1e}
	};
	static const uint8_t letters[26][5] = {
		{0x7e,0x11,0x11,0x11,0x7e},{0x7f,0x49,0x49,0x49,0x36},
		{0x3e,0x41,0x41,0x41,0x22},{0x7f,0x41,0x41,0x22,0x1c},
		{0x7f,0x49,0x49,0x49,0x41},{0x7f,0x09,0x09,0x09,0x01},
		{0x3e,0x41,0x49,0x49,0x7a},{0x7f,0x08,0x08,0x08,0x7f},
		{0x00,0x41,0x7f,0x41,0x00},{0x20,0x40,0x41,0x3f,0x01},
		{0x7f,0x08,0x14,0x22,0x41},{0x7f,0x40,0x40,0x40,0x40},
		{0x7f,0x02,0x0c,0x02,0x7f},{0x7f,0x04,0x08,0x10,0x7f},
		{0x3e,0x41,0x41,0x41,0x3e},{0x7f,0x09,0x09,0x09,0x06},
		{0x3e,0x41,0x51,0x21,0x5e},{0x7f,0x09,0x19,0x29,0x46},
		{0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7f,0x01,0x01},
		{0x3f,0x40,0x40,0x40,0x3f},{0x1f,0x20,0x40,0x20,0x1f},
		{0x3f,0x40,0x38,0x40,0x3f},{0x63,0x14,0x08,0x14,0x63},
		{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43}
	};
	memset(glyph, 0, 5);
	if (c >= '0' && c <= '9') { memcpy(glyph, digits[c - '0'], 5); return; }
	if (c >= 'a' && c <= 'z') c -= ('a' - 'A');
	if (c >= 'A' && c <= 'Z') { memcpy(glyph, letters[c - 'A'], 5); return; }
	switch (c) {
	case '-': glyph[0]=glyph[1]=glyph[2]=glyph[3]=glyph[4]=0x08; break;
	case '.': glyph[2]=0x60; break;
	case ':': glyph[2]=0x36; break;
	case '/': glyph[0]=0x20;glyph[1]=0x10;glyph[2]=0x08;glyph[3]=0x04;glyph[4]=0x02;break;
	case '?': glyph[0]=0x02;glyph[1]=0x01;glyph[2]=0x51;glyph[3]=0x09;glyph[4]=0x06;break;
	default: break;
	}
}

static void draw_char(int x, int y, char c, uint16_t color)
{
	uint8_t glyph[5];
	glyph5x7(c, glyph);
	for (int col = 0; col < 5; ++col) {
		for (int row = 0; row < 7; ++row) {
			if ((glyph[col] & BIT(row)) != 0U) set_pixel(x + col, y + row, color);
		}
	}
}

static void draw_text(int x, int y, const char *text, uint16_t color)
{
	while (*text != '\0' && x < ROUND_LCD_WIDTH - 5) {
		draw_char(x, y, *text++, color);
		x += 6;
	}
}

static void draw_text_centered(int y, const char *text, uint16_t color)
{
	int width = (int)strlen(text) * 6 - 1;
	draw_text((ROUND_LCD_WIDTH - width) / 2, y, text, color);
}

static int slider_position(float value, float minimum, float maximum)
{
	float ratio = (value - minimum) / (maximum - minimum);
	if (ratio < 0.0f) ratio = 0.0f;
	if (ratio > 1.0f) ratio = 1.0f;
	return SLIDER_X + (int)(ratio * (float)SLIDER_WIDTH + 0.5f);
}

static float slider_value(int x, float minimum, float maximum)
{
	if (x < SLIDER_X) x = SLIDER_X;
	if (x > SLIDER_X + SLIDER_WIDTH) x = SLIDER_X + SLIDER_WIDTH;
	return minimum + ((float)(x - SLIDER_X) / (float)SLIDER_WIDTH) *
		(maximum - minimum);
}

static void draw_slider(int y, float target, float actual, float minimum, float maximum)
{
	int target_x = slider_position(target, minimum, maximum);
	int actual_x = slider_position(actual, minimum, maximum);
	fill_rect(SLIDER_X, y, SLIDER_WIDTH + 1, SLIDER_HEIGHT, COLOR_WHITE);
	fill_rect(SLIDER_X, y, target_x - SLIDER_X + 1, SLIDER_HEIGHT, COLOR_PRIMARY);
	fill_rect(actual_x - 1, y - 3, 3, SLIDER_HEIGHT + 6, COLOR_SECONDARY);
	fill_rect(target_x - 3, y - 4, 7, SLIDER_HEIGHT + 8, COLOR_WHITE);
}

int round_ui_init(void)
{
	int ret;
	if (!device_is_ready(display_dev) || !pwm_is_ready_dt(&backlight)) return -ENODEV;
	ret = pwm_set_dt(&backlight, backlight.period,
			 backlight.period * 80U / 100U);
	if (ret != 0) return ret;
	ret = display_set_pixel_format(display_dev, PIXEL_FORMAT_RGB_565);
	if (ret != 0) return ret;
	display_blanking_off(display_dev);
	return 0;
}

bool round_ui_handle_touch(struct gimbal *gimbal, const struct touch_state *touch)
{
	if (!touch->pressed) {
		if (action_button_pressed) {
			action_button_pressed = false;
			return true;
		}
		return false;
	}

	if (touch->just_pressed && touch->x >= CAL_BUTTON_X &&
	    touch->x <= CAL_BUTTON_X + CAL_BUTTON_WIDTH &&
	    touch->y >= CAL_BUTTON_Y &&
	    touch->y <= CAL_BUTTON_Y + CAL_BUTTON_HEIGHT) {
		action_button_pressed = true;
		printk("UI BUTTON %s hit at x=%u y=%u\n",
		       gimbal->state == GIMBAL_BOOT ? "START" : "CALIBRATE",
		       touch->x, touch->y);
		if (gimbal->state == GIMBAL_BOOT) {
			gimbal_request_start(gimbal);
		} else {
			gimbal_request_calibration(gimbal);
		}
		return true;
	}

	if (gimbal->state != GIMBAL_READY ||
	    touch->x < SLIDER_X - SLIDER_TOUCH_PAD ||
	    touch->x > SLIDER_X + SLIDER_WIDTH + SLIDER_TOUCH_PAD) {
		return false;
	}
	if (touch->y >= YAW_SLIDER_Y - SLIDER_TOUCH_PAD &&
	    touch->y <= YAW_SLIDER_Y + SLIDER_HEIGHT + SLIDER_TOUCH_PAD) {
		return gimbal_set_yaw_target(gimbal,
			slider_value(touch->x, GIMBAL_YAW_CONTROL_MIN_DEG,
				     GIMBAL_YAW_CONTROL_MAX_DEG));
	}
	if (touch->y >= PITCH_SLIDER_Y - SLIDER_TOUCH_PAD &&
	    touch->y <= PITCH_SLIDER_Y + SLIDER_HEIGHT + SLIDER_TOUCH_PAD) {
		return gimbal_set_pitch_target(gimbal,
			slider_value(touch->x, GIMBAL_PITCH_CONTROL_MIN_DEG,
				     GIMBAL_PITCH_CONTROL_MAX_DEG));
	}
	return false;
}

int round_ui_render(const struct gimbal *gimbal)
{
	char line[48];
	uint16_t state_color = gimbal->state == GIMBAL_FAULT ? COLOR_RED :
		(gimbal->state == GIMBAL_READY ? COLOR_PRIMARY : COLOR_WHITE);
	uint16_t button_fill = action_button_pressed ? COLOR_WHITE :
		(gimbal->state == GIMBAL_FAULT ? COLOR_RED : COLOR_PRIMARY);
	uint16_t button_text = action_button_pressed ?
		(gimbal->state == GIMBAL_FAULT ? COLOR_RED : COLOR_SECONDARY) :
		(gimbal->state == GIMBAL_FAULT ? COLOR_WHITE : COLOR_SECONDARY);

	clear_frame(COLOR_SECONDARY);
	draw_circle(120, 120, 116, COLOR_PRIMARY);
	draw_circle(120, 120, 115, COLOR_PRIMARY);
	draw_text_centered(14, "GIMBAL CAN 1M", COLOR_PRIMARY);
	snprintk(line, sizeof(line), "STATE %s", gimbal_state_name(gimbal->state));
	draw_text_centered(28, line, state_color);

	snprintk(line, sizeof(line), "YAW 141  %5.1f / %5.1f",
		(double)gimbal->yaw.angle_deg, (double)gimbal->yaw_target_deg);
	draw_text_centered(52, line, COLOR_WHITE);
	draw_slider(YAW_SLIDER_Y, gimbal->yaw_target_deg, gimbal->yaw.angle_deg,
		GIMBAL_YAW_CONTROL_MIN_DEG, GIMBAL_YAW_CONTROL_MAX_DEG);
	draw_text(SLIDER_X, 96, "1", COLOR_WHITE);
	draw_text(SLIDER_X + SLIDER_WIDTH - 17, 96, "345", COLOR_WHITE);

	snprintk(line, sizeof(line), "PITCH 142 %5.1f / %5.1f",
		(double)gimbal->pitch.angle_deg, (double)gimbal->pitch_target_deg);
	draw_text_centered(126, line, COLOR_WHITE);
	draw_slider(PITCH_SLIDER_Y, gimbal->pitch_target_deg, gimbal->pitch.angle_deg,
		GIMBAL_PITCH_CONTROL_MIN_DEG, GIMBAL_PITCH_CONTROL_MAX_DEG);
	draw_text(SLIDER_X, 170, "1", COLOR_WHITE);
	draw_text(SLIDER_X + SLIDER_WIDTH - 17, 170, "175", COLOR_WHITE);

	fill_rect(CAL_BUTTON_X, CAL_BUTTON_Y, CAL_BUTTON_WIDTH,
		  CAL_BUTTON_HEIGHT, button_fill);
	draw_text_centered(CAL_BUTTON_Y + 11,
		gimbal->state == GIMBAL_BOOT ? "START" : "CALIBRATE",
		button_text);
	if (gimbal->state == GIMBAL_READY) {
		draw_text_centered(219, "SLIDERS READY", COLOR_PRIMARY);
	} else if (gimbal->state == GIMBAL_BOOT) {
		draw_text_centered(219, "TAP START TO ENABLE", COLOR_WHITE);
	} else if (gimbal->state == GIMBAL_FAULT) {
		draw_text_centered(219, "FAULT - TAP CAL", COLOR_RED);
	} else {
		draw_text_centered(219, "WAIT OR TAP CAL", COLOR_WHITE);
	}

	return display_write(display_dev, 0, 0, &frame_desc, framebuffer);
}
