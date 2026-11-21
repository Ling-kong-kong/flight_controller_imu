#include "stream.h"
#include "board.h"
#include "protocol.h"
#include "tusb.h"
#include <string.h>

_Static_assert((IMU_RING_CAPACITY & (IMU_RING_CAPACITY - 1u)) == 0u,
               "Ring capacity must be a power of two");
_Static_assert((IMU_HISTORY_CAPACITY & (IMU_HISTORY_CAPACITY - 1u)) == 0u,
               "History capacity must be a power of two");

volatile imu_diagnostics_t g_imu_diagnostics;
volatile mag_heading_diagnostics_t g_mag_fusion_diagnostics;
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
static uint8_t mag_pending[STREAM_MAG_SIZE];
static bool mag_pending_ready;

bool stream_publish_mag(const uint8_t frame[STREAM_MAG_SIZE])
{
    if (!active) return false;
    const bool overwritten = mag_pending_ready;
    memcpy(mag_pending, frame, sizeof(mag_pending));
    mag_pending_ready = true;
    return overwritten;
}

typedef struct {
    uint64_t time;
    int16_t accel[3], gyro[3];
    uint32_t age[2], sequence, session;
    uint16_t flags, late;
    bool send;
} fusion_sample_t;
static fusion_sample_t fusion_queue[IMU_FUSION_QUEUE_CAPACITY];
static volatile uint32_t fusion_head, fusion_tail;
static volatile bool fusion_overflow;
static uint32_t session;
static fusion_t fusion;
void stream_magnetic_sample(const magnetic_sample_t *sample)
{
    fusion_submit_mag(&fusion, sample);
    g_mag_fusion_diagnostics = fusion.magnetic.diagnostics;
}
/* Main-context-only control state; ISR only advances fusion_head. */
static uint8_t control_rx[IMU_CONTROL_SIZE], control_reply[IMU_CONTROL_SIZE];
static unsigned control_used;
static uint32_t control_id, control_boundary, control_operation, calibration_id;
static uint8_t fusion_diag_pending[IMU_FUSION_DIAG_SIZE];
static bool fusion_diag_ready;
static uint64_t fusion_diag_time;
static bool control_requested, control_reply_ready, control_cached;

static void control_receive(uint8_t byte)
{
    control_rx[control_used++] = byte;
    if (control_used < IMU_CONTROL_SIZE) return;
    uint32_t id, operation;
    if (imu_decode_control(control_rx, &id, &operation)) {
        if (control_cached && id == control_id && operation == control_operation) {
            control_reply_ready = true; /* Retry ACK, never repeat the reset. */
        } else if (!control_requested && !control_reply_ready) {
            control_id = id;
            control_operation = operation;
            control_boundary = fusion_head; /* Exclude the entire old backlog. */
            control_requested = true;
            control_cached = false;
        }
        control_used = 0;
    } else {
        memmove(control_rx, control_rx+1, IMU_CONTROL_SIZE-1u);
        control_used = IMU_CONTROL_SIZE-1u;
    }
}

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
    ++session;
    mag_pending_ready = false;
    fusion_diag_ready = false;
    fusion_diag_time = 0;
    control_used = 0;
    control_requested = control_reply_ready = control_cached = false;
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
    const uint32_t head = fusion_head;
    if (head-fusion_tail >= IMU_FUSION_QUEUE_CAPACITY) {
        fusion_overflow = true;
        ++g_imu_diagnostics.fusion_queue_drops;
        restore_flags(flags | IMU_FLAG_OVERFLOW);
        return;
    }
    fusion_sample_t *sample = &fusion_queue[head % IMU_FUSION_QUEUE_CAPACITY];
    sample->time = target_us;
    sample->sequence = seq;
    sample->session = session;
    sample->send = active;
    sample->flags = flags;
    sample->late = (late > UINT16_MAX) ? UINT16_MAX : (uint16_t)late;
    for (unsigned i=0; i<3u; ++i) {
        sample->accel[i] = selected[0].xyz[i];
        sample->gyro[i] = selected[1].xyz[i];
    }
    sample->age[0] = age[0]; sample->age[1] = age[1];
    __DMB();
    fusion_head = head+1u;
    if (head+1u-fusion_tail > g_imu_diagnostics.fusion_high_water)
        g_imu_diagnostics.fusion_high_water = head+1u-fusion_tail;
}

void stream_process_task(void)
{
    /* Only this main-context consumer performs floating point, CRC and encoding.
     * Overflow discards the pre-loss backlog: never publish a valid attitude
     * from before a known fusion discontinuity. USB congestion is independent. */
    for (unsigned n=0; n<32u; ++n) {
        fusion_sample_t sample;
        const uint32_t saved = __get_PRIMASK();
        __disable_irq();
        if (fusion_overflow) {
            g_imu_diagnostics.fusion_queue_drops += fusion_head-fusion_tail;
            for (uint32_t i=fusion_tail; i!=fusion_head; ++i) {
                const fusion_sample_t *queued = &fusion_queue[i % IMU_FUSION_QUEUE_CAPACITY];
                if (active && queued->session == session)
                    pending_flags |= queued->flags & IMU_FLAG_SESSION;
            }
            fusion_tail = fusion_head;
            fusion_overflow = false;
            __set_PRIMASK(saved);
            fusion_reset(&fusion);
            fusion_diag_ready = false;
            g_mag_fusion_diagnostics = fusion.magnetic.diagnostics;
            g_imu_diagnostics.fusion_resets = fusion.resets;
            return;
        }
        if (fusion_tail == fusion_head) {
            __set_PRIMASK(saved);
            /* No samples at all is also a discontinuity. Reset once, not on
             * every idle pass, and preserve the pending reset event. */
            const uint64_t now = board_micros();
            if (fusion.started && now > fusion.previous_tick &&
                now-fusion.previous_tick > 10000u) {
                fusion_reset(&fusion);
                fusion_diag_ready = false;
            }
            g_mag_fusion_diagnostics = fusion.magnetic.diagnostics;
            g_imu_diagnostics.fusion_resets = fusion.resets;
            return;
        }
        const uint32_t sample_index = fusion_tail;
        sample = fusion_queue[fusion_tail % IMU_FUSION_QUEUE_CAPACITY];
        ++fusion_tail;
        __set_PRIMASK(saved);
        const bool execute = control_requested && (int32_t)(sample_index-control_boundary) >= 0;
        if (execute && control_operation == 1u) {
            fusion_reset(&fusion);
            calibration_id = control_id;
        }
        fusion_update(&fusion, sample.time, sample.accel, sample.gyro,
                      sample.age[0], sample.age[1], sample.flags);
        if (execute) {
            const unsigned status = control_operation == 2u ? fusion_heading_zero(&fusion) : 0u;
            imu_encode_operation_reply(control_reply, control_id, control_operation, status,
                                       sample.time, sample.sequence);
            control_requested = false;
            control_cached = control_reply_ready = true;
        }
        uint8_t diagnostic[IMU_FUSION_DIAG_SIZE];
        const bool publish_diag = active && (execute || sample.time-fusion_diag_time >= 100000u);
        if (publish_diag) {
            imu_encode_fusion_diagnostics(diagnostic, &fusion, sample.time,
                                         sample.sequence, calibration_id);
        }
        /* An ISR may have overflowed while floating-point work ran. The next
         * pass discards backlog and resets; do not publish this result. */
        if (fusion_overflow) continue;
        g_imu_diagnostics.fusion_resets = fusion.resets;
        g_mag_fusion_diagnostics = fusion.magnetic.diagnostics;
        uint16_t flags = sample.flags | fusion_magnetic_flags(&fusion);
        if (fusion.attitude.calibrating) flags |= IMU_FLAG_CALIBRATING;
        if (fusion.attitude.valid) flags |= IMU_FLAG_ATTITUDE_VALID;
        if (fusion.attitude.reset) flags |= IMU_FLAG_FUSION_RESET;
        uint8_t frame[IMU_FRAME_SIZE];
        imu_encode_frame(frame, sample.sequence, sample.time, sample.accel, sample.gyro,
                         sample.age[0], sample.age[1], flags, sample.late, &fusion.attitude);
        /* Recheck and publish atomically with respect to the producer. Encoding
         * and all floating point stay outside this short byte-copy section. */
        const uint32_t publish_saved = __get_PRIMASK();
        __disable_irq();
        if (fusion_overflow) {
            __set_PRIMASK(publish_saved);
            continue;
        }
        for (unsigned i=0; i<IMU_FRAME_SIZE; ++i) g_last_imu_frame[i] = frame[i];
        if (!sample.send || !active || sample.session != session) {
            __set_PRIMASK(publish_saved);
            continue;
        }
        if (publish_diag) {
            memcpy(fusion_diag_pending, diagnostic, sizeof(diagnostic));
            fusion_diag_time = sample.time;
            fusion_diag_ready = true; /* Publish only after the overflow recheck. */
        }
        const uint32_t head = produced;
        const uint32_t used = head-consumed;
        if (used >= IMU_RING_CAPACITY) {
            restore_flags((flags & 0x03ffu & ~(IMU_FLAG_ACCEL_STALE | IMU_FLAG_GYRO_STALE)) |
                          IMU_FLAG_OVERFLOW);
            ++g_imu_diagnostics.ring_drops;
            __set_PRIMASK(publish_saved);
            continue;
        }
        for (unsigned i=0; i<IMU_FRAME_SIZE; ++i)
            ring[head & (IMU_RING_CAPACITY-1u)][i] = frame[i];
        __DMB();
        produced = head+1u;
        ++g_imu_diagnostics.queued_pairs;
        if (used+1u > g_imu_diagnostics.high_water_frames)
            g_imu_diagnostics.high_water_frames = used+1u;
        __set_PRIMASK(publish_saved);
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
    /* Bounded sliding command parser: at most 64 incoming bytes per pass. */
    uint8_t incoming[64];
    const uint32_t received = tud_cdc_read(incoming, sizeof(incoming));
    if (active) {
        for (uint32_t i=0; i<received; ++i) control_receive(incoming[i]);
    }
    if (!active) {
        return;
    }
    const uint64_t now = board_micros();
    if (now - last_flush_us < 1000u) {
        return;
    }
    last_flush_us = now;
    if (control_reply_ready) {
        if (tud_cdc_write_available() < IMU_CONTROL_SIZE) return;
        if (tud_cdc_write(control_reply, IMU_CONTROL_SIZE) != IMU_CONTROL_SIZE)
            board_fatal(8u);
        control_reply_ready = false;
    }
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
    if (mag_pending_ready && consumed == produced &&
        tud_cdc_write_available() >= STREAM_MAG_SIZE) {
        if (tud_cdc_write(mag_pending, STREAM_MAG_SIZE) != STREAM_MAG_SIZE)
            board_fatal(8u);
        mag_pending_ready = false;
    }
    if (fusion_diag_ready && consumed == produced &&
        tud_cdc_write_available() >= IMU_FUSION_DIAG_SIZE) {
        if (tud_cdc_write(fusion_diag_pending, IMU_FUSION_DIAG_SIZE) != IMU_FUSION_DIAG_SIZE)
            board_fatal(8u);
        fusion_diag_ready = false;
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
