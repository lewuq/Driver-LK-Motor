#ifndef TOUCH_H
#define TOUCH_H

#include <stdbool.h>

struct touch_state {
	bool pressed;
	bool was_pressed;
	bool just_pressed;
	bool just_released;
	int x;
	int y;
};

int touch_init(void);
int touch_poll(struct touch_state *state);

#endif /* TOUCH_H */
