#include "board.h"
#include "board_i2c.h"

static uint8_t *buffer;
static uint8_t count, position, register_address, slave_address;
static bool reading, busy, address_sent, read_started;
static uint64_t started;

void board_i2c_init(void)
{
    __HAL_RCC_I2C4_CONFIG(RCC_I2C4CLKSOURCE_D3PCLK1);
    __HAL_RCC_I2C4_CLK_ENABLE();
    GPIO_InitTypeDef cfg = {0};
    cfg.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    cfg.Mode = GPIO_MODE_AF_OD;
    cfg.Pull = GPIO_NOPULL; /* External pull-ups required; do not mask a PCB fault. */
    cfg.Speed = GPIO_SPEED_FREQ_LOW;
    cfg.Alternate = GPIO_AF4_I2C4;
    HAL_GPIO_Init(GPIOF, &cfg);
    I2C4->CR1 = 0;
    /* APB4 kernel=100MHz: tPRESC=100ns, low/high=5us each plus
     * analogue filter/synchronizer and edge delays => <=100kHz.
     * SCLDEL=1us, SDADEL=200ns. Electrical edges need scope validation. */
    I2C4->TIMINGR = 0x90923131u;
    I2C4->ICR = 0x3f38u;
    I2C4->CR1 = I2C_CR1_PE;
    busy = false;
}

bool board_i2c_start(uint8_t address, uint8_t reg, uint8_t *data, uint8_t size, bool read)
{
    if (busy || data == NULL || size == 0u || size > 16u || address > 0x7fu)
        return false;
    buffer = data; count = size; position = 0;
    register_address = reg; slave_address = address;
    reading = read; address_sent = read_started = false;
    started = board_micros();
    busy = true;
    /* A BUSY line must time out instead of blocking IMU processing. */
    if ((I2C4->ISR & I2C_ISR_BUSY) == 0u) {
        I2C4->ICR = 0x3f38u;
        I2C4->CR2 = ((uint32_t)address << 1) | I2C_CR2_START |
            ((uint32_t)(read ? 1u : size+1u) << 16) |
            (read ? 0u : I2C_CR2_AUTOEND);
        address_sent = true; /* transaction issued, register byte still pending */
    }
    return true;
}

int board_i2c_poll(void)
{
    if (!busy) return -1;
    uint32_t status = I2C4->ISR;
    if ((status & (I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR)) != 0u ||
        board_micros()-started > 5000u) {
        I2C4->CR1 &= ~I2C_CR1_PE;
        I2C4->CR2 = 0;
        I2C4->ICR = 0x3f38u;
        I2C4->CR1 |= I2C_CR1_PE;
        busy = false;
        return -1;
    }
    if (!address_sent) return 0; /* busy-at-start: fail at deadline, never hijack bus */
    if (!read_started && (status & I2C_ISR_TXIS) != 0u) {
        if (position == 0u) {
            I2C4->TXDR = register_address;
            position = 1u;
        } else if (!reading && position <= count) {
            I2C4->TXDR = buffer[position-1u];
            ++position;
        }
    }
    if (reading && !read_started && (status & I2C_ISR_TC) != 0u) {
        position = 0;
        read_started = true;
        I2C4->CR2 = ((uint32_t)slave_address << 1) | I2C_CR2_RD_WRN |
            I2C_CR2_START | I2C_CR2_AUTOEND | ((uint32_t)count << 16);
    }
    if (read_started && (status & I2C_ISR_RXNE) != 0u && position < count)
        buffer[position++] = (uint8_t)I2C4->RXDR;
    if ((status & I2C_ISR_STOPF) != 0u) {
        I2C4->ICR = I2C_ICR_STOPCF;
        busy = false;
        return position == (reading ? count : count+1u) ? 1 : -1;
    }
    return 0;
}
