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
           (state.attitude.reset ? IMU_FLAG_FUSION_RESET : 0u);
}
void replay_values(float *out)
{
    memcpy(out, state.attitude.q, 4u*sizeof(float));
    memcpy(out+4, state.attitude.euler, 3u*sizeof(float));
    memcpy(out+7, state.bias, 3u*sizeof(float)); /* rad/s */
}
unsigned replay_resets(void) { return state.resets; }
