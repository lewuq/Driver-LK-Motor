#ifndef ROUND_UI_H
#define ROUND_UI_H

#include <stdbool.h>
#include "calibration.h"
#include "touch.h"

int round_ui_init(void);
int round_ui_render(const struct calibration_state *state);
bool round_ui_handle_touch(struct calibration_state *state,
			   const struct touch_state *touch);

#endif
