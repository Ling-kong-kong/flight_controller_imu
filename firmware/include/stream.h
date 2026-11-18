#ifndef STREAM_H
#define STREAM_H

#include <stdbool.h>
#include <stdint.h>

#define IMU_FRAME_SIZE 48u
#define IMU_RING_CAPACITY 512u
#define IMU_OUTPUT_PERIOD_US 1000u
#define IMU_HISTORY_CAPACITY 8u
#define IMU_ACCEL 1u
#define IMU_GYRO 2u
#define IMU_FLAG_TIMING 0x0001u
#define IMU_FLAG_ACCEL_SPI_ERROR 0x0002u
#define IMU_FLAG_GYRO_SPI_ERROR 0x0004u
#define IMU_FLAG_OVERFLOW 0x0008u
#define IMU_FLAG_SESSION 0x0010u
#define IMU_FLAG_ACCEL_STALE 0x0020u
#define IMU_FLAG_GYRO_STALE 0x0040u
#define IMU_FLAG_ACCEL_CADENCE 0x0080u
#define IMU_FLAG_GYRO_CADENCE 0x0100u
#define IMU_FLAG_HISTORY_MISSING 0x0200u

typedef struct {
    uint32_t drdy[2];
    uint32_t spi_errors[2];
    uint32_t timing_warnings[2];
    uint32_t output_ticks;
    uint32_t valid_pairs;
    uint32_t missing_pairs;
    uint32_t timer_skipped;
    uint32_t ring_drops;
    uint32_t queued_pairs;
    uint32_t usb_frames;
    uint32_t usb_sessions;
    uint32_t high_water_frames;
    uint32_t tick_late_max_us;
    uint32_t accel_age_max_us;
    uint32_t gyro_age_max_us;
    uint32_t pair_skew_max_us;
    uint32_t last_target_low;
    uint32_t last_target_high;
    uint32_t last_tick_late_us;
    uint32_t last_frame_sequence;
} imu_diagnostics_t;

/* Counters are also available via SWD; no text/status is mixed into USB data. */
extern volatile imu_diagnostics_t g_imu_diagnostics;
extern volatile uint8_t g_last_imu_frame[IMU_FRAME_SIZE];

void stream_set_active(bool active);
void stream_sample(uint8_t type, uint64_t timestamp_us,
                   const int16_t xyz[3], bool valid, bool read_was_slow);
void stream_output_tick(uint64_t target_us, uint64_t actual_us, uint32_t skipped_ticks);
void stream_usb_task(void);
bool stream_is_active(void);

#endif
