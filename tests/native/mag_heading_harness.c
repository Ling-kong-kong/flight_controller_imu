#include "mag_heading.h"
#include <math.h>
#include <string.h>
static mag_heading_t state;
static float values[5];
void test_reset(void) { memset(&state, 0, sizeof(state)); }
void test_submit(uint64_t t, uint32_t seq, const float *ut, uint16_t flags, uint8_t error)
{
    magnetic_sample_t m={.timestamp_us=t,.sequence=seq,.flags=flags,.error=error};
    memcpy(m.chip_ut,ut,sizeof(m.chip_ut)); mag_heading_submit(&state,&m);
}
float test_step(uint64_t t, float r, float p, float y, float rate, bool valid, bool gravity)
{
    const float rad=0.01745329251994329577f, deg=57.29577951308232f;
    const float cr=cosf(r*rad/2), sr=sinf(r*rad/2), cp=cosf(p*rad/2), sp=sinf(p*rad/2);
    const float cy=cosf(y*rad/2), sy=sinf(y*rad/2);
    float q[4]={cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy};
    mag_heading_step(&state,t,q,.001f,valid,gravity,rate);
    values[0]=state.offset*deg; values[1]=state.dip*deg;
    values[2]=atan2f(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]))*deg;
    values[3]=asinf(fmaxf(-1.0f,fminf(1.0f,2*(q[0]*q[2]-q[3]*q[1]))))*deg;
    values[4]=q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3];
    return atan2f(sinf((atan2f(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]))-y*rad)),
                  cosf((atan2f(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]))-y*rad)))*deg;
}
unsigned test_flags(void) { return mag_heading_flags(&state); }
float test_value(unsigned i) { return values[i]; }

void test_step_q(uint64_t t, float q[4], float dt, float rate, bool valid, bool gravity)
{
    mag_heading_step(&state,t,q,dt,valid,gravity,rate);
}
unsigned test_reason(void) { return state.diagnostics.reason; }
float test_innovation(void) { return state.diagnostics.innovation_deg; }
