#include "board.h"
#include "stream.h"
#include "fusion.h"
#include "protocol.h"
#include "tusb.h"

#include <setjmp.h>
#include <stdlib.h>
#include <string.h>

/* The output is accepted CDC bytes. Its capacity is independently controlled
 * by each test, so a full endpoint FIFO cannot accidentally consume records. */
static uint8_t output[1024u * 1024u];
static uint32_t output_size;
static uint32_t capacity;
static uint32_t rx_pending, rx_offset;
static uint8_t rx_data[4096];
static uint32_t clears;
static uint32_t flushes;
static uint64_t now;
static bool connected;
static bool suspended;
static bool partial_write;
static bool catch_fatal;
static jmp_buf fatal_jump;
uint32_t mock_primask;

void tud_cdc_line_state_cb(uint8_t interface, bool dtr, bool rts);
void tud_umount_cb(void);
void tud_suspend_cb(bool remote_wakeup_enabled);

uint64_t board_micros(void) { return now; }

_Noreturn void board_fatal(unsigned code)
{
    if (catch_fatal) {
        longjmp(fatal_jump, (int)code);
    }
    abort();
}

bool tud_cdc_connected(void) { return connected; }
bool tud_suspended(void) { return suspended; }
bool tud_cdc_write_clear(void) { ++clears; return true; }
uint32_t tud_cdc_available(void) { return rx_pending; }

uint32_t tud_cdc_read(void *buffer, uint32_t count)
{
    const uint32_t copied = (count < rx_pending) ? count : rx_pending;
    memcpy(buffer, rx_data+rx_offset, copied);
    rx_offset += copied;
    rx_pending -= copied;
    return copied;
}

uint32_t tud_cdc_write_available(void) { return capacity; }

uint32_t tud_cdc_write(void const *buffer, uint32_t count)
{
    uint32_t copied = (count < capacity) ? count : capacity;
    if (partial_write && (copied != 0u)) {
        --copied;
        partial_write = false;
    }
    if (output_size + copied > sizeof(output)) {
        abort();
    }
    memcpy(output + output_size, buffer, copied);
    output_size += copied;
    capacity -= copied;
    return copied;
}

uint32_t tud_cdc_write_flush(void) { ++flushes; return 0; }

void mock_reset(void)
{
    now = 10000u;
    connected = false;
    suspended = false;
    partial_write = false;
    catch_fatal = false;
    mock_primask = 0;
    output_size = 0;
    capacity = 0;
    rx_pending = 0;
    clears = 0;
    flushes = 0;
    stream_set_active(false);
    g_imu_diagnostics = (imu_diagnostics_t){0};
}

void mock_connected(bool value) { connected = value; }
void mock_suspended(bool value) { suspended = value; }
void mock_partial_write(bool value) { partial_write = value; }
void mock_set_rx(uint32_t value) { memset(rx_data,0,sizeof(rx_data)); rx_offset=0; rx_pending = value; }
void mock_rx_data(const uint8_t *data, uint32_t size) {
    if (size > sizeof(rx_data)) abort();
    memcpy(rx_data,data,size); rx_offset=0; rx_pending=size;
}
uint32_t mock_get_rx(void) { return rx_pending; }
uint32_t mock_get_clears(void) { return clears; }
uint32_t mock_get_flushes(void) { return flushes; }
uint32_t mock_output_size(void) { return output_size; }
uint32_t mock_ring_capacity(void) { return IMU_RING_CAPACITY; }
uint8_t const *mock_output(void) { return output; }

unsigned mock_tick(uint64_t timestamp, uint32_t available)
{
    now = timestamp;
    capacity = available;
    const unsigned code = (unsigned)setjmp(fatal_jump);
    if (code == 0u) {
        catch_fatal = true;
        stream_usb_task();
    }
    catch_fatal = false;
    return code;
}

void mock_line_state(bool dtr)
{
    connected = dtr;
    tud_cdc_line_state_cb(0u, dtr, false);
}

void mock_suspend_event(void)
{
    suspended = true;
    tud_suspend_cb(false);
}

void mock_unmount_event(void)
{
    connected = false;
    tud_umount_cb();
}

/* Main-loop work and fusion accessors: no ISR-side processing in the mock. */
void mock_process_at(uint64_t timestamp) { now = timestamp; stream_process_task(); }
uint32_t mock_fusion_drops(void) { return g_imu_diagnostics.fusion_queue_drops; }
uint32_t mock_fusion_resets(void) { return g_imu_diagnostics.fusion_resets; }
uint32_t mock_ring_drops(void) { return g_imu_diagnostics.ring_drops; }
static fusion_t test_fusion;
void mock_fusion_reset(void) { memset(&test_fusion, 0, sizeof(test_fusion)); fusion_reset(&test_fusion); }
void mock_fusion_update(uint64_t t, const int16_t *a, const int16_t *g,
                        uint32_t aa, uint32_t ga, uint16_t flags) {
    fusion_update(&test_fusion, t, a, g, aa, ga, flags);
}
float mock_attitude(unsigned i) {
    return i < 4u ? test_fusion.attitude.q[i] : test_fusion.attitude.euler[i-4u];
}
unsigned mock_attitude_flags(void) {
    return (test_fusion.attitude.valid ? IMU_FLAG_ATTITUDE_VALID : 0u) |
           (test_fusion.attitude.calibrating ? IMU_FLAG_CALIBRATING : 0u) |
           (test_fusion.attitude.reset ? IMU_FLAG_FUSION_RESET : 0u);
}
unsigned mock_test_resets(void) { return test_fusion.resets; }
/* Read-only diagnostics for detector numerical/reset contract tests. */
float mock_stationary_gyro(unsigned i) { return test_fusion.stationary_gyro[i]; }
unsigned mock_stationary_count(void) { return test_fusion.count; }
float mock_bias(unsigned i) { return test_fusion.bias[i]; }
void mock_encode_v3(uint8_t *out) {
    const int16_t a[3] = {-32768,32767,-1}, g[3] = {123,-456,789};
    const attitude_t q = {.q={1,0,0,0},.euler={0,0,90},.valid=true};
    imu_encode_frame(out,0x12345678,0x123456789abcdef0ULL,a,g,624,499,0x0800,321,&q);
}

/* Submit through the real stream fusion path, independent of USB TX. */
void mock_submit_mag(uint64_t t, uint32_t seq, const float *ut, uint16_t flags, uint8_t error) {
    const magnetic_sample_t m = {.timestamp_us=t, .sequence=seq,
        .chip_ut={ut[0],ut[1],ut[2]}, .flags=flags, .error=error};
    stream_magnetic_sample(&m);
}
