/***************************************************
  Copied and modified from: https://github.com/brainelectronics/esp32-pca9685/blob/master/components/pca9685/pca9685.c

  This is a library for the PCA9685 LED PWM Driver

  This chip is connected via I2C, 2 pins are required to interface. The PWM frequency is set for all pins, the PWM for each individually. The set PWM is active as long as the chip is powered.

  Written by Jonas Scharpf <jonas@brainelectronics.de>
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "pca9685.h"
#include <freertos/FreeRTOS.h>
#include <driver/i2c.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "esp_err.h"
#include <errno.h>
#include "esp_log.h"
#include "esp_system.h"

// Register addresses from data sheet
#define PCA9685_MODE1_REG               (uint8_t)0x00
#define PCA9685_MODE2_REG               (uint8_t)0x01
#define PCA9685_SUBADR1_REG             (uint8_t)0x02
#define PCA9685_SUBADR2_REG             (uint8_t)0x03
#define PCA9685_SUBADR3_REG             (uint8_t)0x04
#define PCA9685_ALLCALL_REG             (uint8_t)0x05
#define PCA9685_LED0_REG                (uint8_t)0x06          // Start of LEDx regs, 4B per reg, 2B on phase, 2B off phase, little-endian
#define PCA9685_PRESCALE_REG            (uint8_t)0xFE
#define PCA9685_ALLLED_REG              (uint8_t)0xFA

// Mode1 register values
#define PCA9685_MODE1_RESTART           (uint8_t)0x80
#define PCA9685_MODE1_EXTCLK            (uint8_t)0x40
#define PCA9685_MODE1_AUTOINC           (uint8_t)0x20
#define PCA9685_MODE1_SLEEP             (uint8_t)0x10
#define PCA9685_MODE1_SUBADR1           (uint8_t)0x08
#define PCA9685_MODE1_SUBADR2           (uint8_t)0x04
#define PCA9685_MODE1_SUBADR3           (uint8_t)0x02
#define PCA9685_MODE1_ALLCALL           (uint8_t)0x01

// Mode2 register values
#define PCA9685_MODE2_OUTDRV_TPOLE      (uint8_t)0x04
#define PCA9685_MODE2_INVRT             (uint8_t)0x10
#define PCA9685_MODE2_OUTNE_TPHIGH      (uint8_t)0x01
#define PCA9685_MODE2_OUTNE_HIGHZ       (uint8_t)0x02
#define PCA9685_MODE2_OCH_ONACK         (uint8_t)0x08

#define PCA9685_SW_RESET                (uint8_t)0x06          // Sent to address 0x00 to reset all devices on Wire line
#define PCA9685_PWM_FULL                (uint16_t)0x1000    // Special value for full on/full off LEDx modes
#define PCA9685_PWM_MASK                (uint16_t)0x0FFF    // Mask for 12-bit/4096 possible phase positions

#define PCA9685_CHANNEL_COUNT           (uint8_t)16

#define ACK_CHECK_EN    0x1     /*!< I2C master will check ack from slave */
#define ACK_CHECK_DIS   0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL         0x0     /*!< I2C ack value */
#define NACK_VAL        0x1     /*!< I2C nack value */

#define LED0_ON_L       0x6     /*!< LED0 output and brightness control byte 0 */
#define LED0_ON_H       0x7     /*!< LED0 output and brightness control byte 1 */
#define LED0_OFF_L      0x8     /*!< LED0 output and brightness control byte 2 */
#define LED0_OFF_H      0x9     /*!< LED0 output and brightness control byte 3 */

#define ALLLED_ON_L     0xFA    /*!< load all the LEDn_ON registers, byte 0 (turn 0-7 channels on) */
#define ALLLED_ON_H     0xFB    /*!< load all the LEDn_ON registers, byte 1 (turn 8-15 channels on) */
#define ALLLED_OFF_L    0xFC    /*!< load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off) */
#define ALLLED_OFF_H    0xFD    /*!< load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off) */
#define CLOCK_FREQ      25000000  /*!< 25MHz default osc clock */

static const char *TAG = "pca9685";

// default address is 0x40:
static uint8_t PCA9685_ADDR = 0x40;

/**
 * Set the adress of PCA9685.
 */
void pca9685_set_adress(uint8_t addr)
{
    PCA9685_ADDR = addr;
    ESP_LOGI(TAG, "init: address = 0x%x.", PCA9685_ADDR);
}

/**
 * Write two 16 bit values to the same register.
 */
esp_err_t generic_write_i2c_register_two_words(uint8_t regaddr, uint16_t valueOn, uint16_t valueOff)
{
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, valueOn & 0xff, ACK_VAL);
    i2c_master_write_byte(cmd, valueOn >> 8, NACK_VAL);
    i2c_master_write_byte(cmd, valueOff & 0xff, ACK_VAL);
    i2c_master_write_byte(cmd, valueOff >> 8, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * Write a 16 bit value to a register.
 */
esp_err_t generic_write_i2c_register_word(uint8_t regaddr, uint16_t value)
{
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value & 0xff, ACK_VAL);
    i2c_master_write_byte(cmd, value >> 8, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * Write a 8 bit value to a register.
 */
esp_err_t generic_write_i2c_register(uint8_t regaddr, uint8_t value)
{
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * Read two 8 bit values from the same register.
 */
esp_err_t generic_read_two_i2c_register(uint8_t regaddr, uint8_t* valueA, uint8_t* valueB)
{
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, PCA9685_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, valueA, ACK_VAL);
    i2c_master_read_byte(cmd, valueB, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

/**
 * Read a 16 bit value from a register.
 */
esp_err_t generic_read_i2c_register_word(uint8_t regaddr, uint16_t* value)
{
    esp_err_t ret;

    uint8_t valueA;
    uint8_t valueB;

    ret = generic_read_two_i2c_register(regaddr, &valueA, &valueB);
    if (ret != ESP_OK) {
        return ret;
    }

    *value = (valueB << 8) | valueA;

    return ret;
}

/**
 * Reset the PCA9685.
 */
void pca9685_reset(void)
{
    ESP_LOGI(TAG, "reset...");
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, PCA9685_MODE1_REG, ACK_CHECK_EN);   // 0x0 = "Mode register 1"
    i2c_master_write_byte(cmd, PCA9685_MODE1_RESTART | PCA9685_MODE1_AUTOINC, ACK_CHECK_EN);    // 0x80 = "Reset"
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(ret);
    vTaskDelay(50 / portTICK_PERIOD_MS);
}

/**
 * Set the frequency of PCA9685.
 */
void pca9685_set_freq(uint16_t freq)
{
    // Set prescaler
    // calculation on page 25 of datasheet
    uint8_t prescale_val = CLOCK_FREQ / (4096 * freq) - 1;
    if (prescale_val < 3) {
        prescale_val = 3;
    }
    ESP_LOGI(TAG, "set freq: %d, prescale: %d.", freq, prescale_val);
    
    // The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1.
    uint8_t mode1Reg;
    uint8_t any;
    ESP_ERROR_CHECK(generic_read_two_i2c_register(PCA9685_MODE1_REG, &mode1Reg, &any));
    mode1Reg = (mode1Reg & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
    generic_write_i2c_register(PCA9685_MODE1_REG, mode1Reg);
    ESP_ERROR_CHECK(generic_write_i2c_register(PCA9685_PRESCALE_REG, prescale_val));

    // It takes 500us max for the oscillator to be up and running once SLEEP bit has been set to logic 0.
    mode1Reg = (mode1Reg & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART;
    ESP_ERROR_CHECK(generic_write_i2c_register(PCA9685_MODE1_REG, mode1Reg));
    vTaskDelay(5 / portTICK_PERIOD_MS);
}

/**
 * @brief      Sets the pwm of the pin
 *
 * @param[in]  num   The pin number
 * @param[in]  on    On time
 * @param[in]  off   Off time
 * 
 * @return     result of command
 */
void pca9685_set_channel_pwm(uint8_t channel, uint16_t pwm)
{
    if (channel >= PCA9685_CHANNEL_COUNT) {
        ESP_LOGW(TAG, "invalid channel: %d", channel);
        return;
    }
    uint8_t pinAddress = PCA9685_LED0_REG + (channel << 2);
    ESP_LOGI(TAG, "set channel %d (addr = %d) pwm: %d", channel, pinAddress, pwm);
    ESP_ERROR_CHECK(generic_write_i2c_register_two_words(pinAddress, 0, pwm));
}

/**
 * Get the pwm of a pin detail.
 */
esp_err_t _pca9685_get_channel_pwm_detail(uint8_t channel, uint8_t* dataReadOn0, uint8_t* dataReadOn1, uint8_t* dataReadOff0, uint8_t* dataReadOff1)
{
    esp_err_t ret;
    uint8_t pinAddress = LED0_ON_L + (channel << 2);
    ret = generic_read_two_i2c_register(pinAddress, dataReadOn0, dataReadOn1);
    if (ret != ESP_OK) {
        return ret;
    }

    pinAddress = LED0_OFF_L + (channel << 2);
    ret = generic_read_two_i2c_register(pinAddress, dataReadOff0, dataReadOff1);

    return ret;
}

/**
 * Get the pwm of a channel.
 */
uint16_t pca9685_get_channel_pwm(uint8_t channel)
{
    uint8_t readPWMValueOn0;
    uint8_t readPWMValueOn1;
    uint8_t readPWMValueOff0;
    uint8_t readPWMValueOff1;

    ESP_ERROR_CHECK(_pca9685_get_channel_pwm_detail(channel, &readPWMValueOn0, &readPWMValueOn1, &readPWMValueOff0, &readPWMValueOff1));

    ESP_LOGI(TAG, "channel %d: %d, %d", channel, ((readPWMValueOn1 << 8) | readPWMValueOn0), ((readPWMValueOff1 << 8) | readPWMValueOff0));
    return (readPWMValueOff1 << 8) | readPWMValueOff0;
}
