#ifndef BLESCROLL_APP_CONFIG_H_
#define BLESCROLL_APP_CONFIG_H_

/* Common MT6701 angle geometry and the lower-resolution SAADC input. */
#define ENCODER_ANGLE_RESOLUTION_BITS        14U
#define ENCODER_ANGLE_MODULUS                (1U << ENCODER_ANGLE_RESOLUTION_BITS)
#define ENCODER_ANGLE_MAX_CODE               (ENCODER_ANGLE_MODULUS - 1U)
#define ENCODER_ANGLE_HALF_TURN              (ENCODER_ANGLE_MODULUS / 2U)
#define ENCODER_ADC_RESOLUTION_BITS          12U
#define ENCODER_ADC_MODULUS                  (1U << ENCODER_ADC_RESOLUTION_BITS)
#define ENCODER_ADC_MAX_CODE                 (ENCODER_ADC_MODULUS - 1U)
#define ENCODER_FULL_TURN_CENTIDEGREES       36000U

/*
 * Default MT6701 analog output spans 0..VDD. Override these endpoints after
 * bench calibration if the sensor EEPROM is configured for 10..90% clamps.
 */
#define ENCODER_VOUT_MIN_CODE                0U
#define ENCODER_VOUT_MAX_CODE                ENCODER_ADC_MAX_CODE

/* Power-cycled idle probing. Bench data shows valid VOUT after about 32.8 ms. */
#define ENCODER_IDLE_FAST_POLL_PERIOD_MS     100U
#define ENCODER_IDLE_SLOW_POLL_PERIOD_MS     200U
#define ENCODER_IDLE_FAST_DURATION_MS        10000U
#define ENCODER_POWER_STABILIZE_MS           33U

/* Active tracking. */
#define ENCODER_ACTIVE_POLL_PERIOD_MS        2U
#define ENCODER_ACTIVE_TIMEOUT_MS            400U

/* Eight readings, each with SAADC oversampling from Devicetree. */
#define ENCODER_FILTER_SAMPLE_COUNT          8U
#define ENCODER_FILTER_DISCARD_COUNT         1U

/* Circular 14-bit angle counts. Tune after observing real hardware. */
#define ENCODER_IDLE_MOTION_THRESHOLD_COUNTS 32
#define ENCODER_ACTIVE_NOISE_COUNTS          16
#define ENCODER_COUNTS_PER_SCROLL_DETENT     256
#define ENCODER_SCROLL_DIRECTION             1

/* Optional scroll acceleration. All multipliers use unsigned Q8. */
#define ENCODER_ACCELERATION_Q8_ONE                 256U
#define ENCODER_ACCELERATION_Q8_MAX                 512U
#define ENCODER_ACCELERATION_ACTIVATION_COUNTS      2048U
#define ENCODER_ACCELERATION_IDLE_RESET_MS          100U
#define ENCODER_ACCELERATION_BUCKET_MS              10U
#define ENCODER_ACCELERATION_BUCKET_COUNT           8U
#define ENCODER_ACCELERATION_WINDOW_MS              \
	(ENCODER_ACCELERATION_BUCKET_MS * ENCODER_ACCELERATION_BUCKET_COUNT)
#define ENCODER_ACCELERATION_SPEED_MIN_COUNTS_S     6552U
#define ENCODER_ACCELERATION_SPEED_MAX_COUNTS_S     32768U

/* Avoid flooding diagnostics if an unpowered or absent I2C sensor NACKs. */
#define ENCODER_I2C_ERROR_LOG_PERIOD_MS             1000U

/*
 * The HID descriptor advertises x16. Runtime modes can coarsen those x16 units
 * to x8 or legacy x1 without changing the resulting scroll distance. The host
 * selects whether it understands high-resolution units through the Feature
 * report; unsupported hosts keep using one report unit per detent.
 */
#define BLE_HID_WHEEL_RESOLUTION_MULTIPLIER  16U

/* Do not flood USB CDC while sampling the encoder at 500 Hz. */
#define ENCODER_DIAGNOSTIC_BURST_COUNT       20U
#define ENCODER_DIAGNOSTIC_PERIOD_MS         25U
#define ENCODER_STARTUP_CAPTURE_DURATION_US  50000U
#define ENCODER_STARTUP_CAPTURE_MAX_SAMPLES  1024U

/* HID Wheel is an 8-bit signed relative field. */
#define BLE_HID_INPUT_REPORT_ID              1U
#define BLE_HID_INPUT_REPORT_SIZE            4U
#define BLE_HID_INPUT_REPORT_INDEX           0U
#define BLE_HID_FEATURE_REPORT_ID            2U
#define BLE_HID_FEATURE_REPORT_SIZE          1U
#define BLE_HID_FEATURE_REPORT_INDEX         0U
#define BLE_HID_SCROLL_RETRY_MS              5U
#define BLE_HID_SCROLL_MAX_RETRIES           25U
#define BLE_HID_PENDING_LIMIT                2048

/* Battery monitor: BAT+ -> 806 kohm -> P0.04 -> 2 Mohm -> GND. */
#define BATTERY_DIVIDER_TOP_OHMS             806000U
#define BATTERY_DIVIDER_BOTTOM_OHMS          2000000U
#define BATTERY_ADC_SAMPLE_COUNT             8U
#define BATTERY_ADC_DISCARD_COUNT            1U
#define BATTERY_UPDATE_PERIOD_MS             60000U
#define BATTERY_FIRST_UPDATE_DELAY_MS        1000U
#define BATTERY_EMPTY_MV                     3300U
#define BATTERY_MID_MV                       3700U
#define BATTERY_FULL_MV                      4200U

#endif /* BLESCROLL_APP_CONFIG_H_ */
