#include "board.h"
#include "bmi088.h"
#include "stream.h"
#include "tusb.h"

int main(void)
{
    board_init();
    if (!tusb_init()) {
        board_fatal(9u);
    }

    const unsigned error = bmi088_init();
    if (error != 0u) {
        board_fatal(error);
    }
    board_sampling_irq_enable();
    board_delay_us(3000u); /* Fill both raw histories before starting 1 kHz. */
    board_output_timer_enable();

    uint64_t health_check_us = board_micros();
    uint32_t previous_drdy[2] = {0, 0};

    for (;;) {
        tud_task();
        stream_usb_task();
        stream_process_task();
        const uint64_t now = board_micros();
        if (now - health_check_us >= 100000u) {
            for (unsigned i = 0; i < 2u; ++i) {
                const uint32_t count = g_imu_diagnostics.drdy[i];
                if (count == previous_drdy[i]) {
                    board_fatal(7u); /* A missing DRDY wire must be visible. */
                }
                previous_drdy[i] = count;
            }
            health_check_us = now;
        }
        const bool spi_error = (g_imu_diagnostics.spi_errors[0] != 0u) ||
                               (g_imu_diagnostics.spi_errors[1] != 0u);
        board_leds(spi_error, stream_is_active(),
                   ((now / 500000u) & 1u) != 0u);
    }
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void OTG_FS_IRQHandler(void)
{
    tud_int_handler(0);
}

void HardFault_Handler(void)
{
    board_fatal(6u);
}

void MemManage_Handler(void)
{
    board_fatal(6u);
}

void BusFault_Handler(void)
{
    board_fatal(6u);
}

void UsageFault_Handler(void)
{
    board_fatal(6u);
}
