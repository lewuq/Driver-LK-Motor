/* SPDX-License-Identifier: Apache-2.0
 * Calibration UI for the round display.
 *
 * The button records the latest already-received encoder sample. It never
 * sends a CAN command and cannot enable a motor.
 */
#include "round_ui.h"

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>
#include "app_config.h"

#define WHITE 0xffffU
#define GREEN 0x8e03U
#define TEAL  0x024cU
#define RED   0xf800U
#define BUTTON_X 45
#define BUTTON_Y 184
#define BUTTON_W 150
#define BUTTON_H 34

static const struct device *const display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
static const struct pwm_dt_spec backlight = PWM_DT_SPEC_GET(DT_NODELABEL(display_backlight));
static uint16_t framebuffer[ROUND_LCD_WIDTH * ROUND_LCD_HEIGHT];
static const struct display_buffer_descriptor desc = {
	.buf_size = sizeof(framebuffer), .width = ROUND_LCD_WIDTH,
	.height = ROUND_LCD_HEIGHT, .pitch = ROUND_LCD_WIDTH,
};

static uint16_t be(uint16_t c) { return (uint16_t)((c >> 8) | (c << 8)); }
static void pixel(int x, int y, uint16_t c)
{
	if ((unsigned)x < ROUND_LCD_WIDTH && (unsigned)y < ROUND_LCD_HEIGHT)
		framebuffer[y * ROUND_LCD_WIDTH + x] = be(c);
}
static void clear(uint16_t c)
{
	uint16_t v = be(c);
	for (size_t i = 0; i < ARRAY_SIZE(framebuffer); i++) framebuffer[i] = v;
}
static void rect(int x, int y, int w, int h, uint16_t c)
{
	for (int py = y; py < y + h; py++)
		for (int px = x; px < x + w; px++) pixel(px, py, c);
}
static void circle(int cx, int cy, int r, uint16_t c)
{
	int x = r, y = 0, e = 1 - r;
	while (x >= y) {
		pixel(cx+x,cy+y,c); pixel(cx+y,cy+x,c); pixel(cx-y,cy+x,c); pixel(cx-x,cy+y,c);
		pixel(cx-x,cy-y,c); pixel(cx-y,cy-x,c); pixel(cx+y,cy-x,c); pixel(cx+x,cy-y,c);
		y++; if (e < 0) e += 2*y+1; else { x--; e += 2*(y-x)+1; }
	}
}
static void glyph(char c, uint8_t g[5])
{
	static const uint8_t d[10][5] = {
		{0x3e,0x51,0x49,0x45,0x3e},{0,0x42,0x7f,0x40,0},{0x42,0x61,0x51,0x49,0x46},
		{0x21,0x41,0x45,0x4b,0x31},{0x18,0x14,0x12,0x7f,0x10},{0x27,0x45,0x45,0x45,0x39},
		{0x3c,0x4a,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},{0x36,0x49,0x49,0x49,0x36},
		{0x06,0x49,0x49,0x29,0x1e}};
	static const uint8_t a[26][5] = {
		{0x7e,0x11,0x11,0x11,0x7e},{0x7f,0x49,0x49,0x49,0x36},{0x3e,0x41,0x41,0x41,0x22},
		{0x7f,0x41,0x41,0x22,0x1c},{0x7f,0x49,0x49,0x49,0x41},{0x7f,9,9,9,1},
		{0x3e,0x41,0x49,0x49,0x7a},{0x7f,8,8,8,0x7f},{0,0x41,0x7f,0x41,0},
		{0x20,0x40,0x41,0x3f,1},{0x7f,8,0x14,0x22,0x41},{0x7f,0x40,0x40,0x40,0x40},
		{0x7f,2,0x0c,2,0x7f},{0x7f,4,8,0x10,0x7f},{0x3e,0x41,0x41,0x41,0x3e},
		{0x7f,9,9,9,6},{0x3e,0x41,0x51,0x21,0x5e},{0x7f,9,0x19,0x29,0x46},
		{0x46,0x49,0x49,0x49,0x31},{1,1,0x7f,1,1},{0x3f,0x40,0x40,0x40,0x3f},
		{0x1f,0x20,0x40,0x20,0x1f},{0x3f,0x40,0x38,0x40,0x3f},{0x63,0x14,8,0x14,0x63},
		{7,8,0x70,8,7},{0x61,0x51,0x49,0x45,0x43}};
	memset(g, 0, 5); if (c >= '0' && c <= '9') { memcpy(g,d[c-'0'],5); return; }
	if (c >= 'a' && c <= 'z') c -= 32; if (c >= 'A' && c <= 'Z') memcpy(g,a[c-'A'],5);
	else if (c == '-') memset(g,0x08,5); else if (c == ':') g[2]=0x36;
}
static void text(int x, int y, const char *s, uint16_t c)
{
	while (*s && x < 235) { uint8_t g[5]; glyph(*s++,g); for(int i=0;i<5;i++) for(int j=0;j<7;j++) if(g[i]&BIT(j)) pixel(x+i,y+j,c); x+=6; }
}
static void centered(int y, const char *s, uint16_t c)
{
	text((ROUND_LCD_WIDTH - ((int)strlen(s)*6-1))/2, y, s, c);
}

int round_ui_init(void)
{
	int ret;
	if (!device_is_ready(display_dev) || !pwm_is_ready_dt(&backlight)) return -ENODEV;
	ret = pwm_set_dt(&backlight, backlight.period, backlight.period * 80U / 100U);
	if (ret == 0) ret = display_set_pixel_format(display_dev, PIXEL_FORMAT_RGB_565);
	if (ret == 0) display_blanking_off(display_dev);
	return ret;
}

bool round_ui_handle_touch(struct calibration_state *state, const struct touch_state *touch)
{
	if (!touch->pressed) {
		if (state->capture_pressed) { state->capture_pressed = false; return true; }
		return false;
	}
	if (touch->just_pressed && touch->x >= BUTTON_X && touch->x <= BUTTON_X+BUTTON_W &&
	    touch->y >= BUTTON_Y && touch->y <= BUTTON_Y+BUTTON_H) {
		state->capture_pressed = true;
		printk("UI CAPTURE hit x=%d y=%d step=%s\n", touch->x, touch->y,
		       calibration_step_name(state->step));
		(void)calibration_capture(state);
		return true;
	}
	return false;
}

int round_ui_render(const struct calibration_state *state)
{
	char line[48];
	uint16_t button = state->capture_pressed ? WHITE : GREEN;
	uint16_t button_text = state->capture_pressed ? TEAL : TEAL;

	clear(TEAL); circle(120,120,116,GREEN); circle(120,120,115,GREEN);
	centered(14,"ENCODER CAL READ ONLY",GREEN);
	centered(31,calibration_step_name(state->step),
		 state->step == CAL_COMPLETE ? GREEN : WHITE);
	snprintk(line,sizeof(line),"YAW RAW %5u",state->yaw.encoder_raw); centered(60,line,WHITE);
	snprintk(line,sizeof(line),"ENC %5u OFF %5u",state->yaw.encoder_count,state->yaw.encoder_offset); centered(78,line,WHITE);
	snprintk(line,sizeof(line),"PITCH RAW %5u",state->pitch.encoder_raw); centered(108,line,WHITE);
	snprintk(line,sizeof(line),"ENC %5u OFF %5u",state->pitch.encoder_count,state->pitch.encoder_offset); centered(126,line,WHITE);
	centered(153,"MOVE BY HAND - DISABLED",RED);
	rect(BUTTON_X,BUTTON_Y,BUTTON_W,BUTTON_H,button);
	centered(BUTTON_Y+13,state->step == CAL_COMPLETE ? "RESET" : "CAPTURE",button_text);
	centered(225,"NO ENABLE NO MOTION",GREEN);
	return display_write(display_dev,0,0,&desc,framebuffer);
}
