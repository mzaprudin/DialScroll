#ifndef BLESCROLL_SCROLL_SENSITIVITY_H_
#define BLESCROLL_SCROLL_SENSITIVITY_H_

#include <stdbool.h>
#include <stdint.h>

#define SCROLL_SENSITIVITY_MAX_PERCENT 100U
#define SCROLL_SENSITIVITY_STEP_PERCENT 25U
#define SCROLL_SENSITIVITY_DEFAULT_PERCENT SCROLL_SENSITIVITY_MAX_PERCENT

static inline bool scroll_sensitivity_percent_valid(uint8_t percent)
{
	return percent >= SCROLL_SENSITIVITY_STEP_PERCENT &&
	       percent <= SCROLL_SENSITIVITY_MAX_PERCENT &&
	       (percent % SCROLL_SENSITIVITY_STEP_PERCENT) == 0U;
}

int scroll_sensitivity_set(uint8_t percent);
int scroll_sensitivity_cycle(void);
uint8_t scroll_sensitivity_get(void);

#endif /* BLESCROLL_SCROLL_SENSITIVITY_H_ */
