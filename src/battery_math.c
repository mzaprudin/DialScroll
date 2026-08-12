#include "battery_math.h"

#include "app_config.h"

uint8_t battery_voltage_to_percent(int32_t millivolts)
{
	if (millivolts <= BATTERY_EMPTY_MV) {
		return 0U;
	}
	if (millivolts >= BATTERY_FULL_MV) {
		return 100U;
	}
	if (millivolts < BATTERY_MID_MV) {
		return (uint8_t)(((millivolts - BATTERY_EMPTY_MV) * 50) /
			(BATTERY_MID_MV - BATTERY_EMPTY_MV));
	}
	return (uint8_t)(50 + ((millivolts - BATTERY_MID_MV) * 50) /
		(BATTERY_FULL_MV - BATTERY_MID_MV));
}
