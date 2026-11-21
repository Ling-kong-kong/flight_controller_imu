#include "mag_heading.h"
#include "mag_calibration.h"
#include <math.h>

#define RAD 0.01745329251994329577f
#define DEG 57.29577951308232f

static float wrap(float a) { return atan2f(sinf(a), cosf(a)); }
static void reject(mag_heading_t *s, mag_heading_reason_t reason)
{
    if (s->diagnostics.reason != (uint32_t)reason || s->aiding || s->recovery_count)
        ++s->diagnostics.reason_counts[reason];
    s->diagnostics.reason = reason;
    s->aiding = false;
    s->diagnostics.aiding = false;
    s->recovery_count = 0;
    s->offset_sin = s->offset_cos = s->dip_sum = 0.0;
    s->innovation = 0.0f;
}

void mag_heading_submit(mag_heading_t *s, const magnetic_sample_t *sample)
{
    ++s->diagnostics.submitted;
    if (s->count == MAG_HEADING_QUEUE) {
        /* Never process a pre-loss backlog as a continuous recovery window. */
        s->head = s->count = 0;
        ++s->diagnostics.overflows;
        reject(s, MAG_HEADING_OVERFLOW);
        return;
    }
    s->queue[(s->head+s->count)%MAG_HEADING_QUEUE] = *sample;
    ++s->count;
}

static mag_heading_reason_t observation(mag_heading_t *s, const magnetic_sample_t *m,
                                        uint64_t t, const float q[4],
                                        float *offset, float *dip)
{
    if (s->have_previous && m->timestamp_us <= s->previous_time) return MAG_HEADING_TIME;
    const bool gap = s->have_previous && m->sequence-s->previous_sequence != 1u;
    const bool stale_gap = s->have_previous && m->timestamp_us-s->previous_time > MAG_HEADING_MAX_AGE_US;
    s->previous_time = m->timestamp_us;
    s->previous_sequence = m->sequence;
    s->have_previous = true;
    if (!q) return MAG_HEADING_IMU;
    if (gap) return MAG_HEADING_SEQUENCE;
    if (t-m->timestamp_us > MAG_HEADING_MAX_AGE_US || stale_gap) return MAG_HEADING_STALE;
    if ((m->flags & 31u) != 15u || m->error) return MAG_HEADING_QUALITY;
    float chip[3], body[3];
    for (unsigned i=0; i<3u; ++i) {
        if (!isfinite(m->chip_ut[i])) return MAG_HEADING_QUALITY;
        chip[i] = m->chip_ut[i]-mag_bias_ut[i];
    }
    for (unsigned i=0; i<3u; ++i) {
        body[i] = 0.0f;
        for (unsigned j=0; j<3u; ++j) body[i] += mag_correction[i][j]*chip[j];
        /* EXPERIMENTAL independent installation hypothesis [-X,-Y,+Z]. */
        if (i != 2u) body[i] = -body[i];
    }
    const float norm = sqrtf(body[0]*body[0]+body[1]*body[1]+body[2]*body[2]);
    if (!isfinite(norm) || fabsf(norm/MAG_REFERENCE_NORM_UT-1.0f)>0.20f) return MAG_HEADING_NORM;
    /* q is normalized FRD->NED. Compare the WORLD horizontal magnetic
     * vector with a frozen reference, not an independent Euler yaw observation.
     * No pitch singularity: observability depends on the horizontal field. */
    const float w=q[0], x=q[1], y=q[2], z=q[3];
    const float wx=(1.0f-2.0f*(y*y+z*z))*body[0]+2.0f*(x*y-w*z)*body[1]+2.0f*(x*z+w*y)*body[2];
    const float wy=2.0f*(x*y+w*z)*body[0]+(1.0f-2.0f*(x*x+z*z))*body[1]+2.0f*(y*z-w*x)*body[2];
    const float wz=2.0f*(x*z-w*y)*body[0]+2.0f*(y*z+w*x)*body[1]+(1.0f-2.0f*(x*x+y*y))*body[2];
    const float horizontal = hypotf(wx, wy);
    if (horizontal/norm < 0.20f) return MAG_HEADING_HORIZONTAL;
    /* Positive world-Z correction increases azimuth, decreasing this offset. */
    *offset = atan2f(-wy, wx);
    *dip = atan2f(wz, horizontal);
    if (s->reference_ready) {
        if (fabsf(*dip-s->dip)>10.0f*RAD) return MAG_HEADING_DIP;
        if (fabsf(wrap(*offset-s->offset))>25.0f*RAD) return MAG_HEADING_INNOVATION;
    }
    if (!s->aiding && s->recovery_count &&
               (fabsf(wrap(*offset-s->candidate_offset))>5.0f*RAD ||
                fabsf(*dip-s->candidate_dip)>5.0f*RAD)) {
        return MAG_HEADING_UNSTABLE;
    }
    return MAG_HEADING_OK;
}

void mag_heading_step(mag_heading_t *s, uint64_t t, float q[4],
                      float dt,
                      bool imu_valid, bool gravity_ok, float gyro_norm_dps)
{
    s->diagnostics.correction_rate_dps=0.0f;
    mag_heading_reason_t current = MAG_HEADING_OK;
    /* Reject non-finite/degenerate/grossly non-unit attitudes. Normalize a
     * local copy for projection; do not mutate the six-axis q when rejecting. */
    float unit[4];
    const float n2=q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3];
    const bool q_ok=isfinite(n2) && n2>=0.5f && n2<=1.5f;
    if (q_ok) {
        const float inverse_norm=1.0f/sqrtf(n2);
        for (unsigned i=0; i<4u; ++i) unit[i]=q[i]*inverse_norm;
    }
    if (!imu_valid || !q_ok || !isfinite(gyro_norm_dps)) current = MAG_HEADING_IMU;
    else if (!gravity_ok || gyro_norm_dps > (s->reference_ready ? 90.0f : 5.0f))
        current = MAG_HEADING_MOTION;
    if (current != MAG_HEADING_OK) reject(s, current);
    while (s->count && s->queue[s->head].timestamp_us <= t) {
        const magnetic_sample_t m = s->queue[s->head];
        s->head = (s->head+1u)%MAG_HEADING_QUEUE;
        --s->count;
        float offset=0.0f, dip=0.0f;
        mag_heading_reason_t reason = observation(s, &m, t, q_ok ? unit : 0, &offset, &dip);
        if (current != MAG_HEADING_OK) reason = current;
        if (reason != MAG_HEADING_OK) {
            ++s->diagnostics.rejected;
            reject(s, reason);
            continue;
        }
        ++s->diagnostics.accepted;
        s->last_accepted = m.timestamp_us;
        if (!s->aiding) {
            if (!s->recovery_count) {
                s->recovery_start = m.timestamp_us;
                s->candidate_offset = offset; s->candidate_dip = dip;
            }
            ++s->recovery_count;
            s->offset_sin += sinf(offset); s->offset_cos += cosf(offset);
            s->dip_sum += dip;
            if (s->recovery_count >= MAG_HEADING_RECOVERY_SAMPLES &&
                m.timestamp_us-s->recovery_start >= MAG_HEADING_RECOVERY_US) {
                if (!s->reference_ready) {
                    /* Freeze world magnetic azimuth in the CURRENT relative
                     * world frame: late reference must not erase real turns. */
                    s->offset = atan2f((float)s->offset_sin, (float)s->offset_cos);
                    s->dip = (float)(s->dip_sum/s->recovery_count);
                    s->reference_ready = true;
                    ++s->diagnostics.references;
                }
                s->aiding = true;
            }
        }
        if (s->aiding) s->innovation = wrap(offset-s->offset);
        s->diagnostics.reason = s->aiding ? MAG_HEADING_OK : MAG_HEADING_WAITING;
    }
    if (s->last_accepted && t-s->last_accepted > MAG_HEADING_MAX_AGE_US && current == MAG_HEADING_OK)
        reject(s, MAG_HEADING_STALE);
    if (s->aiding && current == MAG_HEADING_OK && dt>0.0f && dt<=0.001001f) {
        const float max_rate = MAG_HEADING_MAX_RATE_DPS*RAD;
        const float rate = fmaxf(-max_rate, fminf(max_rate, MAG_HEADING_KP*s->innovation));
        s->diagnostics.correction_rate_dps=rate*DEG;
        const float delta = rate*dt;
        const float c=cosf(delta*0.5f), z=sinf(delta*0.5f);
        const float w=q[0], x=q[1], y=q[2], v=q[3];
        q[0]=c*w-z*v; q[1]=c*x-z*y; q[2]=c*y+z*x; q[3]=c*v+z*w;
        const float n=sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
        for (unsigned i=0; i<4u; ++i) q[i]/=n;
        /* Hold innovation, NOT a held heading target that would damp real gyro
         * motion between 25 Hz observations. Do not count held data as samples. */
        s->innovation = wrap(s->innovation-delta);
        ++s->diagnostics.aid_ticks;
    }
    s->diagnostics.innovation_deg = s->innovation*DEG;
    s->diagnostics.reference_offset_deg = s->offset*DEG;
    s->diagnostics.reference_dip_deg = s->dip*DEG;
    s->diagnostics.reference_ready = s->reference_ready;
    s->diagnostics.aiding = s->aiding;
}

uint16_t mag_heading_flags(const mag_heading_t *s)
{
    return (s->aiding ? MAG_HEADING_AIDING : MAG_HEADING_REJECTED) |
           (s->reference_ready ? MAG_HEADING_REFERENCE : 0u);
}
