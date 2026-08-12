#ifndef BLESCROLL_SCROLL_ACCELERATION_H_
#define BLESCROLL_SCROLL_ACCELERATION_H_

#include <stdbool.h>

#define SCROLL_ACCELERATION_DEFAULT_ENABLED false

int scroll_acceleration_set(bool enabled);
bool scroll_acceleration_get(void);
const char *scroll_acceleration_name_get(void);

#endif /* BLESCROLL_SCROLL_ACCELERATION_H_ */
