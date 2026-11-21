/* Host-only adapter: replay the real firmware fusion, never a Python model. */
#include "fusion.h"
#include "stream.h"
#include <string.h>

static fusion_t state;
void replay_reset(void) { memset(&state, 0, sizeof(state)); fusion_reset(&state); }
void replay_recalibrate(void) { fusion_reset(&state); }
unsigned replay_update(uint64_t t, const int16_t *a, const int16_t *g,
                       uint32_t aa, uint32_t ga, uint16_t flags)
{
    fusion_update(&state, t, a, g, aa, ga, flags);
    return (state.attitude.valid ? IMU_FLAG_ATTITUDE_VALID : 0u) |
           (state.attitude.calibrating ? IMU_FLAG_CALIBRATING : 0u) |
           (state.attitude.reset ? IMU_FLAG_FUSION_RESET : 0u) | fusion_magnetic_flags(&state);
}
void replay_values(float *out)
{
    memcpy(out, state.attitude.q, 4u*sizeof(float));
    memcpy(out+4, state.attitude.euler, 3u*sizeof(float));
    memcpy(out+7, state.bias, 3u*sizeof(float)); /* rad/s */
}
unsigned replay_heading_zero(void) { return fusion_heading_zero(&state); }
unsigned replay_calibration_phase(void) { return (unsigned)state.phase; }
unsigned replay_resets(void) { return state.resets; }

void replay_mag(uint64_t t, uint32_t sequence, const float *ut, uint16_t flags, uint8_t error)
{
    const magnetic_sample_t sample = {.timestamp_us=t, .sequence=sequence,
        .chip_ut={ut[0],ut[1],ut[2]}, .flags=flags, .error=error};
    fusion_submit_mag(&state, &sample);
}
/* ABI-stable flat counters/values, no host dependency on C struct padding. */
void replay_magnetic(uint32_t *counts, float *values)
{
    const mag_heading_diagnostics_t *d = &state.magnetic.diagnostics;
    counts[0]=d->submitted; counts[1]=d->accepted; counts[2]=d->rejected;
    counts[3]=d->overflows; counts[4]=d->aid_ticks; counts[5]=d->references;
    counts[6]=d->reason; counts[7]=d->reference_ready; counts[8]=d->aiding;
    for (unsigned i=0; i<MAG_HEADING_REASON_COUNT; ++i) counts[9+i]=d->reason_counts[i];
    values[0]=d->innovation_deg; values[1]=d->reference_offset_deg; values[2]=d->reference_dip_deg;
}
