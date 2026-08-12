#ifndef BLESCROLL_SCROLL_RESOLUTION_H_
#define BLESCROLL_SCROLL_RESOLUTION_H_

#include <stdint.h>

enum scroll_resolution_mode {
	SCROLL_RESOLUTION_MODE_OFF = 0,
	SCROLL_RESOLUTION_MODE_X8 = 1,
	SCROLL_RESOLUTION_MODE_X16 = 2,
};

#define SCROLL_RESOLUTION_DEFAULT_MODE SCROLL_RESOLUTION_MODE_X16
#define SCROLL_RESOLUTION_OFF_MULTIPLIER 1U
#define SCROLL_RESOLUTION_X8_MULTIPLIER 8U
#define SCROLL_RESOLUTION_X16_MULTIPLIER 16U

int scroll_resolution_init(void);
int scroll_resolution_set(enum scroll_resolution_mode mode);
enum scroll_resolution_mode scroll_resolution_get(void);
uint8_t scroll_resolution_multiplier_get(void);
const char *scroll_resolution_name_get(void);

#endif /* BLESCROLL_SCROLL_RESOLUTION_H_ */
