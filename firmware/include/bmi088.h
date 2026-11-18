#ifndef BMI088_H
#define BMI088_H

#include <stdbool.h>
#include <stdint.h>

/* 0 = success; 3/4 = accelerometer/gyro identification, 5 = configuration. */
unsigned bmi088_init(void);
void bmi088_irq(bool accelerometer, uint64_t timestamp_us);

#endif
