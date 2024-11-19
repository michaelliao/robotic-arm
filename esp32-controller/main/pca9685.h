/*************************************************** 
  Copied and modified from: https://github.com/brainelectronics/esp32-pca9685/blob/master/components/pca9685/pca9685.h

  This is a library for the PCA9685 LED PWM Driver

  This chip is connected via I2C, 2 pins are required to interface. The PWM frequency is set for all pins, the PWM for each individually. The set PWM is active as long as the chip is powered.

  Written by Jonas Scharpf <jonas@brainelectronics.de>
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifndef PCA9685_DRIVER_H
#define PCA9685_DRIVER_H

#include <stdint.h>
#include "esp_err.h"

extern void pca9685_set_adress(uint8_t addr);
extern void pca9685_reset(void);
extern void pca9685_set_freq(uint16_t freq);
extern void pca9685_set_channel_pwm(uint8_t num, uint16_t pwm);
extern uint16_t pca9685_get_channel_pwm(uint8_t num);

extern esp_err_t generic_write_i2c_register_two_words(uint8_t regaddr, uint16_t valueOn, uint16_t valueOff);
extern esp_err_t generic_write_i2c_register_word(uint8_t regaddr, uint16_t value);
extern esp_err_t generic_write_i2c_register(uint8_t regaddr, uint8_t value);
extern esp_err_t generic_read_i2c_register_word(uint8_t regaddr, uint16_t* value);
extern esp_err_t generic_read_two_i2c_register(uint8_t regaddr, uint8_t* valueA, uint8_t* valueB);

#endif /* PCA9685_DRIVER_H */
