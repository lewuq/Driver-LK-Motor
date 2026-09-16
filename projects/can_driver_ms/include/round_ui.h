#ifndef ROUND_UI_H
#define ROUND_UI_H

#include <stdbool.h>
#include "gimbal.h"
#include "touch.h"

int round_ui_init(void);
bool round_ui_handle_touch(struct gimbal *gimbal, const struct touch_state *touch);
int round_ui_render(const struct gimbal *gimbal);

#endif /* ROUND_UI_H */
