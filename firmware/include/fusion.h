#ifndef IMU_FUSION_H
#define IMU_FUSION_H
#include <stdbool.h>
#include <stdint.h>

#define FUSION_WINDOW 200u
/* Detector ONLY: y += alpha*(x-y), alpha=1-exp(-2*pi*30/1000).
 * Fixed 1 kHz grid; gaps reset the filter rather than changing its bandwidth. */
#define FUSION_STATIONARY_GYRO_ALPHA 0.1717958187f
/* Fixed installation hypothesis, NOT a verified board orientation. Raw output
 * is never rotated. ROTATION_PITCH_180 maps chip axes to provisional FRD. */
#ifndef FUSION_INSTALLATION_PITCH_180
#define FUSION_INSTALLATION_PITCH_180 1
#endif

typedef struct {
    float q[4]; /* body FRD -> world NED, scalar first */
    float euler[3]; /* 3-2-1 roll/pitch/yaw, degrees; relative yaw */
    bool valid;
    bool calibrating;
    bool reset;
} attitude_t;

typedef struct {
    /* Raw accel (g), detector-filtered gyro (deg/s); not fusion inputs. */
    float window[FUSION_WINDOW][6];
    float stationary_gyro[3];
    bool stationary_gyro_seeded;
    double sum[6], sum2[6];
    unsigned count, next;
    double calibration_sum[6];
    unsigned calibration_count;
    uint64_t stationary_start, previous_tick, last_good_gyro, previous_source;
    float bias[3];
    bool started, have_source, initialized, reset_pending;
    uint32_t resets;
    attitude_t attitude;
} fusion_t;

void fusion_reset(fusion_t *state);
void fusion_update(fusion_t *state, uint64_t timestamp_us,
                   const int16_t accel[3], const int16_t gyro[3],
                   uint32_t accel_age_us, uint32_t gyro_age_us, uint16_t flags);
#endif
