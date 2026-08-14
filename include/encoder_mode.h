#ifndef BLESCROLL_ENCODER_MODE_H_
#define BLESCROLL_ENCODER_MODE_H_

enum encoder_mode {
	ENCODER_MODE_I2C = 0,
	ENCODER_MODE_ADC = 1,
};

#define ENCODER_MODE_DEFAULT ENCODER_MODE_I2C

int encoder_mode_set(enum encoder_mode mode);
enum encoder_mode encoder_mode_get(void);
const char *encoder_mode_name(enum encoder_mode mode);

#endif /* BLESCROLL_ENCODER_MODE_H_ */
