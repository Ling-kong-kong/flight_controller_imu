#include "fusion.h"
#include "stream.h"
#include <math.h>
#include <string.h>

#define RAD (0.01745329251994329577f)
#define DEG (57.29577951308232f)

static float norm3(const float *v)
{
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

/* Break only continuous detector/calibration history, not a valid attitude.
 * Ordinary motion fails the gate below without clearing this window/filter. */
static void reset_stationary_detector(fusion_t *s)
{
    s->count = s->next = 0;
    memset(s->sum, 0, sizeof(s->sum));
    memset(s->sum2, 0, sizeof(s->sum2));
    memset(s->stationary_gyro, 0, sizeof(s->stationary_gyro));
    s->stationary_gyro_seeded = false;
    s->stationary_start = 0;
    s->calibration_count = 0;
    memset(s->calibration_sum, 0, sizeof(s->calibration_sum));
}

void fusion_reset(fusion_t *s)
{
    const uint32_t resets = s->resets + 1u;
    memset(s, 0, sizeof(*s));
    s->resets = resets;
    s->attitude.q[0] = 1.0f;
    s->attitude.calibrating = true;
    s->attitude.reset = true;
    s->reset_pending = true;
}

static void angles(attitude_t *a)
{
    const float w=a->q[0], x=a->q[1], y=a->q[2], z=a->q[3];
    const float sinp = 2.0f*(w*y-z*x);
    a->euler[0] = atan2f(2.0f*(w*x+y*z), 1.0f-2.0f*(x*x+y*y))*DEG;
    a->euler[1] = asinf(fmaxf(-1.0f, fminf(1.0f, sinp)))*DEG;
    a->euler[2] = atan2f(2.0f*(w*z+x*y), 1.0f-2.0f*(y*y+z*z))*DEG;
}

void fusion_update(fusion_t *s, uint64_t t, const int16_t accel[3],
                   const int16_t gyro[3], uint32_t aa, uint32_t ga, uint16_t flags)
{
    bool reset = s->reset_pending;
    s->reset_pending = false;
    s->attitude.reset = false;
    if ((!s->started && !reset) || (s->started && (t <= s->previous_tick || t-s->previous_tick > 10000u)) ||
        (s->last_good_gyro && t-s->last_good_gyro > 10000u)) {
        fusion_reset(s);
        reset = true;
        s->reset_pending = false;
    }
    const float dt = s->started ? fminf((float)(t-s->previous_tick)*1e-6f, 0.001f) : 0.0f;
    /* A missed grid tick breaks continuous calibration/statistics, but <=10ms
     * is not a fusion restart: only the current gyro interval is integrated. */
    const bool contiguous = !s->started || t-s->previous_tick == 1000u;
    if (!contiguous || aa > 1000u || aa > t || (flags & IMU_FLAG_ACCEL_SPI_ERROR)) {
        reset_stationary_detector(s);
    }
    s->started = true;
    s->previous_tick = t;
    s->attitude.reset = reset;
    const uint64_t source = ga <= t ? t-ga : 0u;
    const bool gyro_ok = ga <= 1000u && ga <= t &&
        !(flags & IMU_FLAG_GYRO_SPI_ERROR) &&
        (!s->have_source || source > s->previous_source);
    s->attitude.valid = false;
    s->attitude.calibrating = !s->initialized;
    if (!gyro_ok) {
        /* A bad input breaks both filter and continuous statistics history. */
        reset_stationary_detector(s);
        return;
    }
    s->last_good_gyro = t;
    s->previous_source = source;
    s->have_source = true;
    float v[6];
    for (unsigned i=0; i<3u; ++i) {
        const float sign = (FUSION_INSTALLATION_PITCH_180 && i != 1u) ? -1.0f : 1.0f;
        v[i] = sign * (float)accel[i] * (24.0f/32768.0f);
        v[i+3] = sign * (float)gyro[i] * (2000.0f/32768.0f);
    }
    const bool accel_ok = aa <= 1000u && aa <= t && !(flags & IMU_FLAG_ACCEL_SPI_ERROR);
    if (accel_ok) {
        for (unsigned i=0; i<3u; ++i) {
            if (!s->stationary_gyro_seeded) s->stationary_gyro[i] = v[i+3];
            else s->stationary_gyro[i] += FUSION_STATIONARY_GYRO_ALPHA *
                                          (v[i+3]-s->stationary_gyro[i]);
        }
        s->stationary_gyro_seeded = true;
        for (unsigned i=0; i<6u; ++i) {
            /* Never feed this filtered gyro into bias estimation/integration.
             * Accel statistics and Mahony gravity gating remain unfiltered. */
            const float sample = i < 3u ? v[i] : s->stationary_gyro[i-3u];
            const float old = s->count == FUSION_WINDOW ? s->window[s->next][i] : 0.0f;
            s->sum[i] += sample-old;
            s->sum2[i] += sample*sample-old*old;
            s->window[s->next][i] = sample;
        }
        if (s->count < FUSION_WINDOW) ++s->count;
        s->next = (s->next+1u)%FUSION_WINDOW;
    }
    bool accel_quiet = s->count == FUSION_WINDOW;
    bool gyro_quiet = accel_quiet;
    for (unsigned i=0; i<6u; ++i) {
        const float mean = s->sum[i]/(double)(s->count ? s->count : 1u);
        const float variance = fmaxf(0.0f, s->sum2[i]/(double)(s->count ? s->count : 1u)-(double)mean*mean);
        if (i<3u && variance > 0.02f*0.02f) accel_quiet = false;
        if (i>=3u && variance > 0.5f*0.5f) gyro_quiet = false;
    }
    const float an = norm3(v);
    const bool gravity_ok = accel_ok && fabsf(an-1.0f) <= 0.1f && accel_quiet;
    if (!s->initialized) {
        const bool stationary = gravity_ok && gyro_quiet && norm3(v+3)<5.0f;
        if (!stationary) {
            s->stationary_start = 0;
            s->calibration_count = 0;
            memset(s->calibration_sum, 0, sizeof(s->calibration_sum));
            return;
        }
        if (!s->stationary_start) s->stationary_start = t;
        for (unsigned i=0; i<6u; ++i) s->calibration_sum[i] += v[i];
        ++s->calibration_count;
        if (t-s->stationary_start < 2000000u) return;
        float mean[6];
        for (unsigned i=0; i<6u; ++i) mean[i]=(float)(s->calibration_sum[i]/s->calibration_count);
        for (unsigned i=0; i<3u; ++i) s->bias[i]=mean[i+3]*RAD;
        /* Accelerometer measures specific force, opposite NED gravity. */
        const float roll=atan2f(-mean[1], -mean[2]);
        const float pitch=atan2f(mean[0], sqrtf(mean[1]*mean[1]+mean[2]*mean[2]));
        const float cr=cosf(roll*0.5f), sr=sinf(roll*0.5f);
        const float cp=cosf(pitch*0.5f), sp=sinf(pitch*0.5f);
        s->attitude.q[0]=cr*cp; s->attitude.q[1]=sr*cp;
        s->attitude.q[2]=cr*sp; s->attitude.q[3]=-sr*sp;
        s->initialized=true;
    } else {
        float omega[3];
        for (unsigned i=0; i<3u; ++i) omega[i]=v[i+3]*RAD-s->bias[i];
        float *q=s->attitude.q;
        const float w=q[0], x=q[1], y=q[2], z=q[3];
        if (gravity_ok) {
            const float d[3]={-v[0]/an,-v[1]/an,-v[2]/an};
            const float p[3]={2.0f*(x*z-w*y),2.0f*(y*z+w*x),1.0f-2.0f*(x*x+y*y)};
            /* Mahony Kp=2 s^-1, Ki=0. Bias is estimated only at calibration. */
            omega[0]+=2.0f*(d[1]*p[2]-d[2]*p[1]);
            omega[1]+=2.0f*(d[2]*p[0]-d[0]*p[2]);
            omega[2]+=2.0f*(d[0]*p[1]-d[1]*p[0]);
        }
        const float h=0.5f*dt, gx=omega[0], gy=omega[1], gz=omega[2];
        q[0]+=(-x*gx-y*gy-z*gz)*h; q[1]+=(w*gx+y*gz-z*gy)*h;
        q[2]+=(w*gy-x*gz+z*gx)*h; q[3]+=(w*gz+x*gy-y*gx)*h;
        const float n=sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
        for (unsigned i=0;i<4u;++i) q[i]/=n;
    }
    s->attitude.valid=true;
    s->attitude.calibrating=false;
    angles(&s->attitude);
}
