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

static uint32_t packet_crc(const uint8_t *p, unsigned length)
{
    uint32_t crc = UINT32_MAX;
    for (unsigned i=0; i<length; ++i) {
        crc ^= p[i];
        for (unsigned bit=0; bit<8u; ++bit)
            crc = (crc>>1) ^ ((crc&1u) ? 0xEDB88320u : 0u);
    }
    return crc ^ UINT32_MAX;
}

bool imu_decode_control(const uint8_t frame[32], uint32_t *request_id, uint32_t *operation)
{
    static const uint8_t header[8] = {'I','M','U','2',1,0x10,32,0};
    if (memcmp(frame, header, sizeof(header)) != 0 || get32(frame+8)==0u ||
        (get32(frame+12)!=1u && get32(frame+12)!=2u) || get32(frame+16)!=0u || get32(frame+20)!=0u ||
        get32(frame+24)!=0u || get32(frame+28)!=packet_crc(frame, 28u)) return false;
    *request_id = get32(frame+8);
    *operation = get32(frame+12);
    return true;
}

void imu_encode_operation_reply(uint8_t frame[32], uint32_t request_id,
    uint32_t operation, uint32_t status, uint64_t timestamp_us, uint32_t sequence)
{
    const uint8_t header[8] = {'I','M','U','2',1,0x11,32,0};
    memcpy(frame,header,sizeof(header));
    put32(frame+8,request_id);
    frame[5] = operation == 2u ? 0x13u : 0x11u;
    put32(frame+12,status); /* Executed, NOT calibration complete. */
    put32(frame+16,(uint32_t)timestamp_us);
    put32(frame+20,(uint32_t)(timestamp_us>>32));
    put32(frame+24,sequence);
    put32(frame+28,packet_crc(frame, 28u));
}

bool imu_decode_recalibrate(const uint8_t frame[32], uint32_t *request_id)
{
    uint32_t operation, id;
    if (!imu_decode_control(frame, &id, &operation) || operation != 1u) return false;
    *request_id = id;
    return true;
}
void imu_encode_control_reply(uint8_t frame[32], uint32_t request_id,
                              uint64_t timestamp_us, uint32_t sequence)
{
    imu_encode_operation_reply(frame, request_id, 1u, 0u, timestamp_us, sequence);
}
void imu_encode_fusion_diagnostics(uint8_t frame[56], const fusion_t *s,
    uint64_t timestamp_us, uint32_t sequence, uint32_t request_id)
{
    const uint8_t header[8] = {'I','M','U','2',1,0x12,56,0};
    memcpy(frame, header, sizeof(header));
    put32(frame+8, (uint32_t)timestamp_us);
    put32(frame+12, (uint32_t)(timestamp_us>>32));
    put32(frame+16, sequence);
    put32(frame+20, (uint32_t)s->phase);
    put32(frame+24, s->magnetic.diagnostics.reason);
    put32(frame+28, s->magnetic.recovery_count);
    uint64_t elapsed = s->magnetic.recovery_count && timestamp_us >= s->magnetic.recovery_start
        ? (timestamp_us-s->magnetic.recovery_start)/1000u : 0u;
    put32(frame+32, elapsed > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed);
    uint32_t bits;
    memcpy(&bits, &s->magnetic.diagnostics.innovation_deg, 4);
    put32(frame+36, bits);
    memcpy(&bits, &s->magnetic.diagnostics.correction_rate_dps, 4);
    put32(frame+40, bits);
    put32(frame+44, fusion_magnetic_flags(s) | (s->attitude.valid ? 2048u : 0u)
        | (s->attitude.calibrating ? 1024u : 0u) | (s->attitude.reset ? 4096u : 0u));
    put32(frame+48, request_id);
    put32(frame+52, packet_crc(frame, 52u));
}
