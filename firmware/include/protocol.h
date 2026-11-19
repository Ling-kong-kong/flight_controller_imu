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
#define IMU_CONTROL_SIZE 32u
/* Exact, CRC-checked v1/type16 recalibration command; reserved fields are zero. */
bool imu_decode_recalibrate(const uint8_t frame[32], uint32_t *request_id);
void imu_encode_control_reply(uint8_t frame[32], uint32_t request_id,
                              uint64_t timestamp_us, uint32_t sequence);
#endif
