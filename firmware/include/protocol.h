#ifndef IMU_PROTOCOL_H
#define IMU_PROTOCOL_H

#include <stdint.h>
#include "fusion.h"

void imu_encode_frame_v2(uint8_t frame[48], uint32_t sequence, uint64_t timestamp_us,
                      const int16_t accel[3], const int16_t gyro[3],
                      uint32_t accel_age_us, uint32_t gyro_age_us,
                      uint16_t flags, uint16_t tick_late_us);

void imu_encode_frame(uint8_t frame[76], uint32_t sequence, uint64_t timestamp_us,
                      const int16_t accel[3], const int16_t gyro[3],
                      uint32_t accel_age_us, uint32_t gyro_age_us,
                      uint16_t flags, uint16_t tick_late_us, const attitude_t *attitude);
#endif
