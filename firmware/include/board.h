#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

/* Pin assignments come directly from docs/original-hwdef.dat. */
#define BMI_ACCEL_CS_PIN GPIO_PIN_4
#define BMI_GYRO_CS_PIN GPIO_PIN_8
#define BMI_ACCEL_DRDY_PIN GPIO_PIN_6
#define BMI_GYRO_DRDY_PIN GPIO_PIN_7
#define BMI_GPIO GPIOI

void board_init(void);
void board_delay_us(uint32_t us);
uint64_t board_micros(void);
bool board_spi_transfer(bool accelerometer, const uint8_t *tx, uint8_t *rx,
                        size_t length);
void board_sampling_irq_enable(void);
void board_output_timer_enable(void);
void board_leds(bool red, bool green, bool blue);
_Noreturn void board_fatal(unsigned code);

#endif
