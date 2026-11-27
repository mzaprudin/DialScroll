#ifndef BLESCROLL_SCROLL_DIRECTION_H_
#define BLESCROLL_SCROLL_DIRECTION_H_

#include <stdint.h>

int scroll_direction_set(int8_t direction);
int8_t scroll_direction_get(void);
const char *scroll_direction_name_get(void);

#endif /* BLESCROLL_SCROLL_DIRECTION_H_ */
