#include "bmi088.h"
#include "board.h"
#include "stream.h"

enum {
    ACC_CHIP_ID = 0x00,
    ACC_DATA = 0x12,
    ACC_CONF = 0x40,
    ACC_RANGE = 0x41,
    ACC_INT1_IO_CONF = 0x53,
    ACC_INT_MAP_DATA = 0x58,
    ACC_PWR_CONF = 0x7C,
    ACC_PWR_CTRL = 0x7D,
    ACC_SOFTRESET = 0x7E,
    GYR_CHIP_ID = 0x00,
    GYR_DATA = 0x02,
    GYR_RANGE = 0x0F,
    GYR_BANDWIDTH = 0x10,
    GYR_LPM1 = 0x11,
    GYR_SOFTRESET = 0x14,
    GYR_INT_CTRL = 0x15,
    GYR_INT_IO_CONF = 0x16,
    GYR_INT_MAP = 0x18,
};

static bool read_registers(bool accel, uint8_t address, uint8_t *out, unsigned n)
{
    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};
    const unsigned prefix = accel ? 2u : 1u;
    if (n + prefix > sizeof(tx)) {
        return false;
    }
    tx[0] = address | 0x80u;
    if (!board_spi_transfer(accel, tx, rx, n + prefix)) {
        return false;
    }
    for (unsigned i = 0; i < n; ++i) {
        out[i] = rx[i + prefix];
    }
    return true;
}

static bool write_register(bool accel, uint8_t address, uint8_t value)
{
    const uint8_t tx[2] = {address & 0x7Fu, value};
    uint8_t rx[2];
    return board_spi_transfer(accel, tx, rx, sizeof(tx));
}

static bool expect_register(bool accel, uint8_t address, uint8_t value, uint8_t mask)
{
    uint8_t actual = 0;
    return read_registers(accel, address, &actual, 1u) &&
           ((actual & mask) == (value & mask));
}

static bool configure_register(bool accel, uint8_t address, uint8_t value,
                               uint8_t mask)
{
    if (!write_register(accel, address, value)) {
        return false;
    }
    board_delay_us(100u);
    return expect_register(accel, address, value, mask);
}

unsigned bmi088_init(void)
{
    uint8_t ignored;
    /* The first accel CS rising edge selects SPI; its response is undefined. */
    (void)read_registers(true, ACC_CHIP_ID, &ignored, 1u);
    board_delay_us(1000u);
    if (!expect_register(true, ACC_CHIP_ID, 0x1Eu, 0xFFu)) {
        return 3u;
    }
    if (!expect_register(false, GYR_CHIP_ID, 0x0Fu, 0xFFu)) {
        return 4u;
    }
    if (!write_register(true, ACC_SOFTRESET, 0xB6u) ||
        !write_register(false, GYR_SOFTRESET, 0xB6u)) {
        return 5u;
    }
    board_delay_us(50000u); /* Gyro needs at least 30 ms after reset. */
    (void)read_registers(true, ACC_CHIP_ID, &ignored, 1u);
    board_delay_us(1000u);
    if (!expect_register(true, ACC_CHIP_ID, 0x1Eu, 0xFFu) ||
        !expect_register(false, GYR_CHIP_ID, 0x0Fu, 0xFFu)) {
        return 5u;
    }

    if (!write_register(true, ACC_PWR_CONF, 0x00u)) {
        return 5u;
    }
    board_delay_us(5000u);
    if (!write_register(true, ACC_PWR_CTRL, 0x04u)) {
        return 5u;
    }
    board_delay_us(50000u);

    /* Raw, native axes. No rotation, calibration, resampling or fusion. */
    if (!expect_register(true, ACC_PWR_CONF, 0x00u, 0x03u) ||
        !expect_register(true, ACC_PWR_CTRL, 0x04u, 0x04u) ||
        !configure_register(true, ACC_CONF, 0xACu, 0xFFu) || /* 1600 Hz normal. */
        !configure_register(true, ACC_RANGE, 0x03u, 0x03u) || /* +/-24 g. */
        !configure_register(true, ACC_INT1_IO_CONF, 0x0Au, 0x0Fu) ||
        !configure_register(true, ACC_INT_MAP_DATA, 0x04u, 0x44u) ||
        /* Reset already selected normal mode. Avoid another power-mode write:
         * a real gyro power-mode transition requires 30 ms without SPI. */
        !expect_register(false, GYR_LPM1, 0x00u, 0xA0u) ||
        !configure_register(false, GYR_RANGE, 0x00u, 0x07u) || /* +/-2000 dps. */
        !configure_register(false, GYR_BANDWIDTH, 0x01u, 0x0Fu) || /* 2 kHz/230 Hz. */
        !configure_register(false, GYR_INT_IO_CONF, 0x01u, 0x03u) ||
        !configure_register(false, GYR_INT_MAP, 0x01u, 0x81u) ||
        !configure_register(false, GYR_INT_CTRL, 0x80u, 0x80u)) {
        return 5u;
    }
    /* Allow new ODR/filter settings to settle before enabling EXTI. */
    board_delay_us(50000u);
    return 0u;
}

void bmi088_irq(bool accelerometer, uint64_t timestamp_us)
{
    uint8_t data[6] = {0};
    int16_t xyz[3] = {0};
    const bool valid = read_registers(accelerometer,
                                      accelerometer ? ACC_DATA : GYR_DATA,
                                      data, sizeof(data));
    if (valid) {
        for (unsigned i = 0; i < 3u; ++i) {
            xyz[i] = (int16_t)((uint16_t)data[i * 2u] |
                               ((uint16_t)data[i * 2u + 1u] << 8));
        }
    }
    stream_sample(accelerometer ? IMU_ACCEL : IMU_GYRO, timestamp_us,
                   xyz, valid, (board_micros() - timestamp_us) > 100u);
}
