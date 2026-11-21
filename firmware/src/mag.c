#include "mag.h"
#include "board.h"
#include "board_i2c.h"
#include "stream.h"
#include <math.h>
#include <string.h>

volatile mag_diagnostics_t g_mag_diagnostics = {.address=0x10, .rep_xy=4, .rep_z=7};
struct bmm150_trim_registers g_mag_trim;
/* All I/O runs cooperatively in main, never in DRDY/TIM2. No bus scan,
 * soft-reset, write to another address, or global I2C reset is performed. */
enum { RETRY, POWER, WAKE, ID, TRIM1, TRIM2, TRIM3,
       SLEEP, REPXY, REPZ, AXES, NORMAL, VERIFY, CHECK_READY, SAMPLE };
static unsigned phase;
static bool pending;
static uint8_t data[16], trim_bytes[16];
static uint64_t due, read_start, last_ready;

static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (uint16_t)p[1]<<8); }
static void put16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void put32(uint8_t *p, uint32_t v)
{
    for (unsigned i=0; i<4u; ++i) p[i]=(uint8_t)(v>>(8u*i));
}
static BMM150_INTF_RET_TYPE no_read(uint8_t r, uint8_t *p, uint32_t n, void *c)
{ (void)r; (void)p; (void)n; (void)c; return -1; }
static BMM150_INTF_RET_TYPE no_write(uint8_t r, const uint8_t *p, uint32_t n, void *c)
{ (void)r; (void)p; (void)n; (void)c; return -1; }
static void no_delay(uint32_t t, void *c) { (void)t; (void)c; }

bool mag_compensate(const uint8_t bytes[8], const struct bmm150_trim_registers *t,
                    int16_t raw[3], uint16_t *hall, float ut[3])
{
    raw[0]=(int16_t)((int8_t)bytes[1]*32 + (bytes[0]>>3));
    raw[1]=(int16_t)((int8_t)bytes[3]*32 + (bytes[2]>>3));
    raw[2]=(int16_t)((int8_t)bytes[5]*128 + (bytes[4]>>1));
    *hall=(uint16_t)((uint16_t)bytes[7]*64 + (bytes[6]>>2));
    ut[0]=ut[1]=ut[2]=0.0f;
    if (raw[0]==-4096 || raw[1]==-4096 || raw[2]==-16384 || *hall==0 ||
        t->dig_z1==0 || t->dig_z2==0 || t->dig_xyz1==0) return false;
    const float denominator=(float)t->dig_z2+(float)t->dig_z1*(float)*hall/32768.0f;
    if (!isfinite(denominator) || fabsf(denominator)<1.0e-6f) return false;
    uint8_t copy[8], context=0;
    memcpy(copy,bytes,sizeof(copy)); /* Bosch aux API modifies the raw bytes. */
    struct bmm150_dev dev={0};
    dev.trim_data=*t;
    dev.read=no_read; dev.write=no_write; dev.delay_us=no_delay; dev.intf_ptr=&context;
    struct bmm150_mag_data result;
    if (bmm150_aux_mag_data(copy,&result,&dev)!=BMM150_OK ||
        !isfinite(result.x) || !isfinite(result.y) || !isfinite(result.z)) return false;
    ut[0]=result.x; ut[1]=result.y; ut[2]=result.z;
    return true;
}

static void publish(uint64_t now, uint16_t flags, const int16_t raw[3], uint16_t hall,
                    const float ut[3])
{
    uint8_t frame[MAG_FRAME_SIZE]={ 'I','M','U','2',1,4,72,0 };
    const uint32_t sequence = g_mag_diagnostics.sequence++;
    const magnetic_sample_t sample = {
        .timestamp_us=now, .sequence=sequence,
        .chip_ut={ut ? ut[0] : 0.0f, ut ? ut[1] : 0.0f, ut ? ut[2] : 0.0f},
        .flags=flags, .error=g_mag_diagnostics.error
    };
    /* Feed fusion directly even without DTR. The USB slot is NOT an input. */
    stream_magnetic_sample(&sample);
    put32(frame+8,sequence);
    put32(frame+12,(uint32_t)now); put32(frame+16,(uint32_t)(now>>32));
    for (unsigned i=0; i<3u; ++i) {
        put16(frame+20+2*i,raw ? (uint16_t)raw[i] : 0u);
        float value=ut ? ut[i] : 0.0f;
        uint32_t bits;
        _Static_assert(sizeof(float)==4u,"binary32 required");
        memcpy(&bits,&value,4); put32(frame+28+4*i,bits);
    }
    put16(frame+26,hall); put16(frame+40,flags);
    frame[42]=g_mag_diagnostics.address; frame[43]=g_mag_diagnostics.chip_id;
    frame[44]=g_mag_diagnostics.state; frame[45]=g_mag_diagnostics.error;
    frame[46]=g_mag_diagnostics.rep_xy; frame[47]=g_mag_diagnostics.rep_z;
    put32(frame+48,g_mag_diagnostics.bus_errors);
    put32(frame+52,g_mag_diagnostics.invalid_samples);
    put32(frame+56,g_mag_diagnostics.not_ready);
    put32(frame+60,g_mag_diagnostics.tx_overwrites);
    put32(frame+64,g_mag_diagnostics.read_us);
    uint32_t crc=UINT32_MAX;
    for (unsigned i=0; i<68u; ++i) {
        crc^=frame[i];
        for (unsigned b=0; b<8u; ++b) crc=(crc>>1)^((crc&1u)?0xedb88320u:0u);
    }
    put32(frame+68,crc^UINT32_MAX);
    if (stream_publish_mag(frame)) ++g_mag_diagnostics.tx_overwrites;
}
static void fail(uint8_t error, uint64_t now)
{
    g_mag_diagnostics.error=error;
    g_mag_diagnostics.state=0;
    g_mag_diagnostics.configured=false;
    g_mag_diagnostics.trim_ok=false;
    phase=RETRY; pending=false; due=now+1000000u;
    publish(now,MAG_DIAGNOSTIC,NULL,0,NULL);
}
static bool decode_trim(void)
{
    bool all0=true, allff=true;
    for (unsigned i=0; i<sizeof(trim_bytes); ++i) {
        all0=all0 && trim_bytes[i]==0; allff=allff && trim_bytes[i]==0xff;
    }
    g_mag_trim.dig_x1=(int8_t)trim_bytes[0]; g_mag_trim.dig_y1=(int8_t)trim_bytes[1];
    g_mag_trim.dig_z4=(int16_t)le16(trim_bytes+2);
    g_mag_trim.dig_x2=(int8_t)trim_bytes[4]; g_mag_trim.dig_y2=(int8_t)trim_bytes[5];
    g_mag_trim.dig_z2=(int16_t)le16(trim_bytes+6); g_mag_trim.dig_z1=le16(trim_bytes+8);
    g_mag_trim.dig_xyz1=le16(trim_bytes+10)&0x7fffu;
    g_mag_trim.dig_z3=(int16_t)le16(trim_bytes+12);
    g_mag_trim.dig_xy2=(int8_t)trim_bytes[14]; g_mag_trim.dig_xy1=trim_bytes[15];
    return !all0 && !allff && g_mag_trim.dig_z1 && g_mag_trim.dig_z2 && g_mag_trim.dig_xyz1;
}
void mag_task(void)
{
    uint64_t now=board_micros();
    if (pending) {
        int result=board_i2c_poll();
        if (!result) return;
        pending=false;
        if (result<0) { ++g_mag_diagnostics.bus_errors; fail(1,now); return; }
        switch (phase) {
        case POWER: due=now+3000u; break;
        case ID:
            g_mag_diagnostics.chip_id=data[0];
            if (data[0]!=0x32) { fail(2,now); return; }
            break;
        case TRIM1: memcpy(trim_bytes,data,2); break;
        case TRIM2: memcpy(trim_bytes+2,data,4); break;
        case TRIM3:
            memcpy(trim_bytes+6,data,10);
            if (!decode_trim()) { fail(3,now); return; }
            g_mag_diagnostics.trim_ok=true;
            break;
        case VERIFY:
            if (!(data[0]&1u) || (data[1]&0x3eu)!=0x30 || (data[3]&0x38u)!=0 ||
                data[6]!=4 || data[7]!=7) { fail(4,now); return; }
            g_mag_diagnostics.configured=true;
            g_mag_diagnostics.error=0;
            last_ready=now;
            break;
        case CHECK_READY:
            /* Reading 0x48 plus STOP acknowledges DRDY. Only start a locked
             * XYZ/RHALL burst once a new measurement was already available.
             * Inspecting ready late in an unready burst races conversion and
             * can incorrectly publish the old shadow-locked vector. */
            if (!(data[0]&1u)) {
                ++g_mag_diagnostics.not_ready;
                if (now-last_ready>200000u) { fail(5,now); return; }
                due=now+10000u;
                return;
            }
            break; /* Immediately advance to SAMPLE; ready has been consumed. */
        case SAMPLE: {
            g_mag_diagnostics.read_us=(uint32_t)(now-read_start);
            last_ready=now;
            int16_t raw[3]; uint16_t hall; float ut[3];
            bool valid=mag_compensate(data,&g_mag_trim,raw,&hall,ut);
            if (valid) ++g_mag_diagnostics.samples;
            else ++g_mag_diagnostics.invalid_samples;
            g_mag_diagnostics.error=valid ? 0 : 6;
            publish(now,MAG_READY|MAG_TRIM_OK|MAG_CONFIGURED|(valid?MAG_VALID:0),raw,hall,ut);
            phase=CHECK_READY;
            due=now+10000u;
            return;
        }
        default: break;
        }
        ++phase;
    }
    if (now<due) return;
    if (phase==RETRY) {
        ++g_mag_diagnostics.retries;
        g_mag_diagnostics.chip_id=0;
        memset(&g_mag_trim,0,sizeof(g_mag_trim));
        phase=POWER;
    }
    if (phase==WAKE) ++phase;
    uint8_t reg=0, size=1; bool read=false;
    switch (phase) {
    case POWER: g_mag_diagnostics.state=1; reg=0x4b; data[0]=1; break;
    case ID: g_mag_diagnostics.state=2; reg=0x40; read=true; break;
    case TRIM1: g_mag_diagnostics.state=3; reg=0x5d; size=2; read=true; break;
    case TRIM2: reg=0x62; size=4; read=true; break;
    case TRIM3: reg=0x68; size=10; read=true; break;
    case SLEEP: g_mag_diagnostics.state=4; reg=0x4c; data[0]=0x36; break;
    case REPXY: reg=0x51; data[0]=4; break;
    case REPZ: reg=0x52; data[0]=7; break;
    case AXES: reg=0x4e; data[0]=0; break;
    case NORMAL: reg=0x4c; data[0]=0x30; break;
    case VERIFY: g_mag_diagnostics.state=5; reg=0x4b; size=8; read=true; break;
    case CHECK_READY: g_mag_diagnostics.state=6; reg=0x48; read=true; break;
    case SAMPLE: g_mag_diagnostics.state=6; reg=0x42; size=8; read=true; break;
    default: return;
    }
    read_start=now;
    if (!board_i2c_start(0x10,reg,data,size,read)) {
        ++g_mag_diagnostics.bus_errors; fail(1,now); return;
    }
    pending=true;
}
