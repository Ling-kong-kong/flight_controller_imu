#include "protocol.h"
#include <string.h>

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

void imu_encode_frame_v2(uint8_t frame[48], uint32_t sequence, uint64_t timestamp_us,
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

void imu_encode_frame(uint8_t frame[76], uint32_t sequence, uint64_t timestamp_us,
                      const int16_t accel[3], const int16_t gyro[3],
                      uint32_t accel_age_us, uint32_t gyro_age_us,
                      uint16_t flags, uint16_t tick_late_us, const attitude_t *a)
{
    imu_encode_frame_v2(frame, sequence, timestamp_us, accel, gyro,
                        accel_age_us, gyro_age_us, flags, tick_late_us);
    frame[4]=3u;
    put16(frame+6,76u);
    for (unsigned i=0;i<7u;++i) {
        const float value = i<4u ? a->q[i] : a->euler[i-4u];
        uint32_t bits;
        _Static_assert(sizeof(float)==4u, "Protocol requires binary32 float");
        memcpy(&bits,&value,sizeof(bits));
        put32(frame+44u+4u*i,bits);
    }
    uint32_t crc=UINT32_MAX;
    for (unsigned i=0;i<72u;++i) {
        crc^=frame[i];
        for (unsigned bit=0;bit<8u;++bit)
            crc=(crc>>1)^((crc&1u)?0xEDB88320u:0u);
    }
    put32(frame+72,crc^UINT32_MAX);
}

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1]<<8 | (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24;
}

static uint32_t control_crc(const uint8_t *p)
{
    uint32_t crc = UINT32_MAX;
    for (unsigned i=0; i<28u; ++i) {
        crc ^= p[i];
        for (unsigned bit=0; bit<8u; ++bit)
            crc = (crc>>1) ^ ((crc&1u) ? 0xEDB88320u : 0u);
    }
    return crc ^ UINT32_MAX;
}

bool imu_decode_recalibrate(const uint8_t frame[32], uint32_t *request_id)
{
    static const uint8_t header[8] = {'I','M','U','2',1,0x10,32,0};
    if (memcmp(frame, header, sizeof(header)) != 0 || get32(frame+8)==0u ||
        get32(frame+12)!=1u || get32(frame+16)!=0u || get32(frame+20)!=0u ||
        get32(frame+24)!=0u || get32(frame+28)!=control_crc(frame)) return false;
    *request_id = get32(frame+8);
    return true;
}

void imu_encode_control_reply(uint8_t frame[32], uint32_t request_id,
                              uint64_t timestamp_us, uint32_t sequence)
{
    const uint8_t header[8] = {'I','M','U','2',1,0x11,32,0};
    memcpy(frame,header,sizeof(header));
    put32(frame+8,request_id);
    put32(frame+12,0u); /* Executed, NOT calibration complete. */
    put32(frame+16,(uint32_t)timestamp_us);
    put32(frame+20,(uint32_t)(timestamp_us>>32));
    put32(frame+24,sequence);
    put32(frame+28,control_crc(frame));
}
