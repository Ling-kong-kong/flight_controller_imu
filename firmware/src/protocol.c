#include "protocol.h"

static void put16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
}

static void put32(uint8_t *out, uint32_t value)
{
    for (unsigned i = 0; i < 4u; ++i) {
        out[i] = (uint8_t)(value >> (8u * i));
    }
}

void imu_encode_frame(uint8_t frame[48], uint32_t sequence, uint64_t timestamp_us,
                      const int16_t accel[3], const int16_t gyro[3],
                      uint32_t accel_age_us, uint32_t gyro_age_us,
                      uint16_t flags, uint16_t tick_late_us)
{
    frame[0] = 'I';
    frame[1] = 'M';
    frame[2] = 'U';
    frame[3] = '2';
    frame[4] = 2u;
    frame[5] = 3u;
    put16(frame + 6, 48u);
    put32(frame + 8, sequence);
    put32(frame + 12, (uint32_t)timestamp_us);
    put32(frame + 16, (uint32_t)(timestamp_us >> 32));
    for (unsigned i = 0; i < 3u; ++i) {
        put16(frame + 20u + 2u * i, (uint16_t)accel[i]);
        put16(frame + 26u + 2u * i, (uint16_t)gyro[i]);
    }
    put32(frame + 32, accel_age_us);
    put32(frame + 36, gyro_age_us);
    put16(frame + 40, flags);
    put16(frame + 42, tick_late_us);

    /* CRC-32/ISO-HDLC, identical to zlib.crc32(frame[:44]). */
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0; i < 44u; ++i) {
        crc ^= frame[i];
        for (unsigned bit = 0; bit < 8u; ++bit) {
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
        }
    }
    put32(frame + 44, crc ^ UINT32_MAX);
}
