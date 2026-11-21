#include "mag.h"
#include "stream.h"
#include "board_i2c.h"
#include <string.h>
#include <stdlib.h>
static uint64_t now, next_ready, complete_at, started_at;
static unsigned conversions;
static uint8_t snapshot[16];
static uint8_t regs[256], last_reg, length, *buffer;
static bool reading, pending;
static unsigned faults, frame_count;
static uint8_t output[128][72];
static magnetic_sample_t submitted[128];
static unsigned submitted_count;
void stream_magnetic_sample(const magnetic_sample_t *sample)
{ submitted[submitted_count++%128] = *sample; }
unsigned mock_submitted(void) { return submitted_count; }
const magnetic_sample_t *mock_magnetic(unsigned i) { return &submitted[i%128]; }
uint64_t board_micros(void) { return now; }
_Noreturn void board_fatal(unsigned code) { (void)code; abort(); }
static void set16(unsigned r, uint16_t v) { regs[r]=(uint8_t)v; regs[r+1]=(uint8_t)(v>>8); }
void mock_init(void)
{
    regs[0x40]=0x32;
    regs[0x5d]=(uint8_t)-5; regs[0x5e]=6;
    set16(0x62,(uint16_t)-1000); regs[0x64]=(uint8_t)-7; regs[0x65]=8;
    set16(0x68,(uint16_t)-2000); set16(0x6a,30000); set16(0x6c,25000);
    set16(0x6e,(uint16_t)-100); regs[0x70]=(uint8_t)-3; regs[0x71]=40;
    set16(0x42,(uint16_t)(123u<<3)); set16(0x44,(uint16_t)((uint16_t)-456<<3));
    set16(0x46,(uint16_t)(500u<<1)); set16(0x48,(uint16_t)(12000u<<2));
}
void mock_faults(unsigned v) { faults=v; }
void mock_reg(unsigned r, unsigned v) { regs[r]=(uint8_t)v; }
unsigned mock_count(void) { return frame_count; }
const uint8_t *mock_frame(unsigned i) { return output[i%128]; }
void mock_run(unsigned us)
{
    uint64_t end=now+us;
    while (now<end) {
        now+=100;
        /* Conversion clock is independent of polling. A data burst locks the
         * vector at START, but a later conversion may assert DRDY before the
         * RHALL/ready byte is read (Bosch datasheet 4.3.1 shadow registers). */
        if (next_ready && now>=next_ready) {
            next_ready+=40000;
            if (!(faults&2u)) {
                ++conversions;
                if (faults&32u) set16(0x42,(uint16_t)((123u+conversions)<<3));
                regs[0x48]|=1u;
            }
        }
        mag_task();
    }
}
bool board_i2c_start(uint8_t address, uint8_t reg, uint8_t *p, uint8_t n, bool read)
{
    if (address!=0x10 || pending) abort();
    pending=true; last_reg=reg; buffer=p; length=n; reading=read;
    started_at=now;
    complete_at=now+((read && reg==0x42) ? 1100u : (read && reg==0x48) ? 400u : 100u);
    if (read) memcpy(snapshot,regs+reg,n);
    return true;
}
int board_i2c_poll(void)
{
    if (!pending) abort();
    if (now<complete_at) return 0;
    pending=false;
    if (faults&1u) return -1;
    if (!reading) {
        memcpy(regs+last_reg,buffer,length);
        if (last_reg==0x4c && buffer[0]==0x30) next_ready=now+40000;
    } else {
        memcpy(buffer,snapshot,length);
        if (last_reg==0x42) {
            /* Data bytes are shadow-locked; DRDY can change mid-burst. */
            buffer[6]=(uint8_t)((buffer[6]&~1u)|(regs[0x48]&1u));
            /* STOP releases shadow. A conversion during the locked burst
             * becomes available for the next read. Otherwise acknowledge. */
            if (!(next_ready-40000>started_at && next_ready-40000<=now))
                regs[0x48]&=(uint8_t)~1u;
        } else if (last_reg==0x48) {
            buffer[0]=regs[0x48];
            regs[0x48]&=(uint8_t)~1u;
        }
        if (last_reg==0x40 && faults&4u) buffer[0]=0;
        if (last_reg==0x4b && faults&8u) buffer[7]=0;
        if (last_reg==0x68 && faults&16u) memset(buffer,0,length);
    }
    return 1;
}
bool stream_publish_mag(const uint8_t p[STREAM_MAG_SIZE])
{
    memcpy(output[frame_count++%128],p,72);
    return false;
}
/* Pure helper wrapper with a controlled factory-trim fixture. */
bool mock_compensate(const uint8_t *bytes, int16_t *raw, uint16_t *hall, float *ut)
{
    struct bmm150_trim_registers t={0};
    t.dig_x1=-5; t.dig_y1=6; t.dig_x2=-7; t.dig_y2=8;
    t.dig_z1=30000; t.dig_z2=-2000; t.dig_z3=-100; t.dig_z4=-1000;
    t.dig_xyz1=25000; t.dig_xy1=40; t.dig_xy2=-3;
    return mag_compensate(bytes,&t,raw,hall,ut);
}
