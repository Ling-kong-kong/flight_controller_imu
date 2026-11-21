#ifndef MAG_HEADING_H
#define MAG_HEADING_H
#include <stdbool.h>
#include <stdint.h>

#define MAG_HEADING_QUEUE 8u
#define MAG_HEADING_MAX_AGE_US 100000u
#define MAG_HEADING_RECOVERY_US 800000u
#define MAG_HEADING_RECOVERY_SAMPLES 20u
#define MAG_HEADING_KP 0.15f
#define MAG_HEADING_MAX_RATE_DPS 1.0f
/* IMU2 v3 magnetic status bits; no change to frame length or attitude-valid semantics. */
#define MAG_HEADING_AIDING 0x2000u
#define MAG_HEADING_REJECTED 0x4000u
#define MAG_HEADING_REFERENCE 0x8000u

typedef enum {
    MAG_HEADING_WAITING, MAG_HEADING_OK, MAG_HEADING_IMU,
    MAG_HEADING_MOTION, MAG_HEADING_QUALITY, MAG_HEADING_TIME,
    MAG_HEADING_SEQUENCE, MAG_HEADING_STALE, MAG_HEADING_OVERFLOW,
    MAG_HEADING_NORM, MAG_HEADING_HORIZONTAL, MAG_HEADING_TILT, /* reserved legacy reason */
    MAG_HEADING_DIP, MAG_HEADING_INNOVATION, MAG_HEADING_UNSTABLE,
    MAG_HEADING_REASON_COUNT
} mag_heading_reason_t;

typedef struct {
    uint64_t timestamp_us;
    uint32_t sequence;
    float chip_ut[3];
    uint16_t flags;
    uint8_t error;
} magnetic_sample_t;

typedef struct {
    uint32_t submitted, accepted, rejected, overflows, aid_ticks, references;
    uint32_t reason_counts[MAG_HEADING_REASON_COUNT];
    uint32_t reason;
    float innovation_deg, reference_offset_deg, reference_dip_deg;
    float correction_rate_dps;
    bool reference_ready, aiding;
} mag_heading_diagnostics_t;

typedef struct {
    magnetic_sample_t queue[MAG_HEADING_QUEUE];
    unsigned head, count;
    bool have_previous, reference_ready, aiding;
    uint32_t previous_sequence, recovery_count;
    uint64_t previous_time, last_accepted, recovery_start;
    float offset, dip, innovation, candidate_offset, candidate_dip;
    double offset_sin, offset_cos, dip_sum;
    mag_heading_diagnostics_t diagnostics;
} mag_heading_t;

/* Main context only, independent of USB. State is zero-initializable. */
void mag_heading_submit(mag_heading_t *s, const magnetic_sample_t *sample);
/* q is FRD->NED (unit, finite; squared norm outside [0.5,1.5] rejected).
 * dt seconds, gyro norm deg/s. No Euler angles required. Only a LEFT world-Z
 * rotation is applied; body gravity direction and gyro bias are unchanged.
 * Euler roll/pitch stay unchanged away from their representation singularity. */
void mag_heading_step(mag_heading_t *s, uint64_t timestamp_us, float q[4],
                      float dt,
                      bool imu_valid, bool gravity_ok, float gyro_norm_dps);
uint16_t mag_heading_flags(const mag_heading_t *s);
#endif
