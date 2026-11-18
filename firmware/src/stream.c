#include "stream.h"
#include "board.h"
#include "protocol.h"
#include "tusb.h"

_Static_assert((IMU_RING_CAPACITY & (IMU_RING_CAPACITY - 1u)) == 0u,
               "Ring capacity must be a power of two");
_Static_assert((IMU_HISTORY_CAPACITY & (IMU_HISTORY_CAPACITY - 1u)) == 0u,
               "History capacity must be a power of two");

volatile imu_diagnostics_t g_imu_diagnostics;
volatile uint8_t g_last_imu_frame[IMU_FRAME_SIZE];
typedef struct {
    uint64_t timestamp_us;
    int16_t xyz[3];
} source_sample_t;
static source_sample_t history[2][IMU_HISTORY_CAPACITY];
static uint32_t history_next[2];
static uint32_t history_count[2];
static uint8_t ring[IMU_RING_CAPACITY][IMU_FRAME_SIZE];
static volatile uint32_t produced;
static volatile uint32_t consumed;
static volatile bool active;
static uint32_t sequence;
static uint64_t previous_us[2];
static bool have_previous[2];
static uint16_t pending_flags;
static uint64_t last_flush_us;

bool stream_is_active(void)
{
    return active;
}

void stream_set_active(bool enabled)
{
    /* Main context only. TIM2 is the queue producer. Keep source history across
     * sessions so the first output tick already has real sample candidates. */
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    active = false;
    produced = 0;
    consumed = 0;
    sequence = 0;
    pending_flags = enabled ? IMU_FLAG_SESSION : 0u;
    if (enabled) {
        ++g_imu_diagnostics.usb_sessions;
    }
    __DMB();
    active = enabled;
    __set_PRIMASK(saved);
    last_flush_us = board_micros();
}

void stream_sample(uint8_t type, uint64_t timestamp_us,
                   const int16_t xyz[3], bool valid, bool read_was_slow)
{
    /* EXTI only: capture native samples. Output/USB pacing never causes extra
     * SPI reads and never rewrites the actual DRDY service timestamps. */
    const unsigned index = (type == IMU_ACCEL) ? 0u : 1u;
    ++g_imu_diagnostics.drdy[index];
    if (!valid) {
        ++g_imu_diagnostics.spi_errors[index];
    }
    const uint64_t delta = timestamp_us - previous_us[index];
    const uint32_t expected = (type == IMU_ACCEL) ? 625u : 500u;
    if (read_was_slow || (have_previous[index] &&
        ((delta < expected * 3u / 4u) || (delta > expected * 3u / 2u)))) {
        pending_flags |= (index == 0u) ? IMU_FLAG_ACCEL_CADENCE : IMU_FLAG_GYRO_CADENCE;
        ++g_imu_diagnostics.timing_warnings[index];
    }
    previous_us[index] = timestamp_us;
    have_previous[index] = true;
    if (!valid) {
        pending_flags |= (index == 0u) ? IMU_FLAG_ACCEL_SPI_ERROR : IMU_FLAG_GYRO_SPI_ERROR;
        return;
    }
    source_sample_t *sample = &history[index][history_next[index]];
    sample->timestamp_us = timestamp_us;
    for (unsigned axis = 0; axis < 3u; ++axis) {
        sample->xyz[axis] = xyz[axis];
    }
    history_next[index] = (history_next[index] + 1u) & (IMU_HISTORY_CAPACITY - 1u);
    if (history_count[index] < IMU_HISTORY_CAPACITY) {
        ++history_count[index];
    }
}

static void restore_flags(uint16_t flags)
{
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    pending_flags |= flags;
    __set_PRIMASK(saved);
}

void stream_output_tick(uint64_t target_us, uint64_t actual_us, uint32_t skipped_ticks)
{
    /* TIM2 compare ISR only. Select the latest *past* raw value for each die.
     * A late timer interrupt must not silently select a sample from the future.
     * Snapshot and take sticky flags atomically against the higher-priority
     * EXTI producer; subsequently arriving errors remain pending for next time. */
    source_sample_t selected[2] = {0};
    bool found[2] = {false, false};
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    for (unsigned sensor = 0; sensor < 2u; ++sensor) {
        for (unsigned slot = 0; slot < history_count[sensor]; ++slot) {
            const source_sample_t *sample = &history[sensor][slot];
            if ((sample->timestamp_us <= target_us) &&
                (!found[sensor] || sample->timestamp_us > selected[sensor].timestamp_us)) {
                selected[sensor] = *sample;
                found[sensor] = true;
            }
        }
    }
    uint16_t flags = pending_flags;
    pending_flags = 0;
    __set_PRIMASK(saved);

    g_imu_diagnostics.output_ticks += skipped_ticks + 1u;
    g_imu_diagnostics.timer_skipped += skipped_ticks;
    sequence += skipped_ticks;
    const uint32_t seq = sequence++;
    const uint64_t late64 = (actual_us >= target_us) ? actual_us - target_us : 0u;
    const uint32_t late = (late64 > UINT32_MAX) ? UINT32_MAX : (uint32_t)late64;
    if ((late > 100u) || (skipped_ticks != 0u) || (actual_us < target_us)) {
        flags |= IMU_FLAG_TIMING;
    }
    if (late > g_imu_diagnostics.tick_late_max_us) {
        g_imu_diagnostics.tick_late_max_us = late;
    }
    g_imu_diagnostics.last_target_low = (uint32_t)target_us;
    g_imu_diagnostics.last_target_high = (uint32_t)(target_us >> 32);
    g_imu_diagnostics.last_tick_late_us = late;
    g_imu_diagnostics.last_frame_sequence = seq;
    if (!found[0] || !found[1]) {
        ++g_imu_diagnostics.missing_pairs;
        restore_flags(flags | IMU_FLAG_HISTORY_MISSING);
        return;
    }
    ++g_imu_diagnostics.valid_pairs;
    uint32_t age[2];
    for (unsigned sensor = 0; sensor < 2u; ++sensor) {
        const uint64_t age64 = target_us - selected[sensor].timestamp_us;
        age[sensor] = (age64 > UINT32_MAX) ? UINT32_MAX : (uint32_t)age64;
        if (age[sensor] > IMU_OUTPUT_PERIOD_US) {
            flags |= (sensor == 0u) ? IMU_FLAG_ACCEL_STALE : IMU_FLAG_GYRO_STALE;
        }
    }
    if (age[0] > g_imu_diagnostics.accel_age_max_us) {
        g_imu_diagnostics.accel_age_max_us = age[0];
    }
    if (age[1] > g_imu_diagnostics.gyro_age_max_us) {
        g_imu_diagnostics.gyro_age_max_us = age[1];
    }
    const uint32_t skew = (age[0] > age[1]) ? age[0] - age[1] : age[1] - age[0];
    if (skew > g_imu_diagnostics.pair_skew_max_us) {
        g_imu_diagnostics.pair_skew_max_us = skew;
    }
    uint8_t frame[IMU_FRAME_SIZE];
    imu_encode_frame(frame, seq, target_us, selected[0].xyz, selected[1].xyz,
                      age[0], age[1], flags,
                      (late > UINT16_MAX) ? UINT16_MAX : (uint16_t)late);
    /* Keep a genuine latest pair available via SWD even without an open USB. */
    for (unsigned i = 0; i < IMU_FRAME_SIZE; ++i) {
        g_last_imu_frame[i] = frame[i];
    }
    if (!active) {
        return;
    }
    const uint32_t head = produced;
    const uint32_t used = head - consumed;
    if (used >= IMU_RING_CAPACITY) {
        restore_flags((flags & ~(IMU_FLAG_ACCEL_STALE | IMU_FLAG_GYRO_STALE)) |
                       IMU_FLAG_OVERFLOW);
        ++g_imu_diagnostics.ring_drops;
        return;
    }
    for (unsigned i = 0; i < IMU_FRAME_SIZE; ++i) {
        ring[head & (IMU_RING_CAPACITY - 1u)][i] = frame[i];
    }
    __DMB();
    produced = head + 1u;
    ++g_imu_diagnostics.queued_pairs;
    if (used + 1u > g_imu_diagnostics.high_water_frames) {
        g_imu_diagnostics.high_water_frames = used + 1u;
    }
}

void stream_usb_task(void)
{
    /* CDC DTR marks a host reader. Close/reopen starts a fresh bounded queue. */
    const bool connected = tud_cdc_connected() && !tud_suspended();
    if (connected != active) {
        (void)tud_cdc_write_clear();
        stream_set_active(connected);
    }
    /* Incoming bytes have no commands; consume them to keep CDC RX healthy. */
    uint8_t discard[64];
    if (tud_cdc_available() != 0u) {
        (void)tud_cdc_read(discard, sizeof(discard));
    }
    if (!active) {
        return;
    }
    const uint64_t now = board_micros();
    if (now - last_flush_us < 1000u) {
        return;
    }
    last_flush_us = now;
    /* Up to 16 frames per pass lets a backlog catch up; normally one frame. */
    for (unsigned sent = 0; sent < 16u; ++sent) {
        const uint32_t tail = consumed;
        if ((tail == produced) || (tud_cdc_write_available() < IMU_FRAME_SIZE)) {
            break;
        }
        __DMB();
        const uint32_t written = tud_cdc_write(
            ring[tail & (IMU_RING_CAPACITY - 1u)], IMU_FRAME_SIZE);
        if (written != IMU_FRAME_SIZE) {
            /* Single main-context writer + checked FIFO capacity guarantees
             * whole records. Fail visibly if this invariant ever changes. */
            board_fatal(8u);
        }
        __DMB();
        consumed = tail + 1u;
        ++g_imu_diagnostics.usb_frames;
    }
    (void)tud_cdc_write_flush();
}

void tud_cdc_line_state_cb(uint8_t interface, bool dtr, bool rts)
{
    (void)interface;
    (void)rts;
    /* Handle close/open even if TinyUSB processes both events in one task. */
    const bool enabled = dtr && tud_cdc_connected() && !tud_suspended();
    if (enabled != active) {
        (void)tud_cdc_write_clear();
        stream_set_active(enabled);
    }
}

void tud_umount_cb(void)
{
    stream_set_active(false);
}

void tud_suspend_cb(bool remote_wakeup_enabled)
{
    (void)remote_wakeup_enabled;
    stream_set_active(false);
}
