#include "board.h"
#include "bmi088.h"
#include "stream.h"

#ifndef FLASH_OFFSET_KB
#define FLASH_OFFSET_KB 0
#endif

static volatile uint32_t timer_wraps;
static uint64_t next_output_us;

static void dwt_start(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* Cortex-M7 DWT can be locked after reset. */
    DWT->LAR = 0xC5ACCE55u;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void board_delay_us(uint32_t us)
{
    const uint32_t cycles_per_us = SystemCoreClock / 1000000u;
    while (us != 0u) {
        const uint32_t step = (us > 1000u) ? 1000u : us;
        const uint32_t start = DWT->CYCCNT;
        while ((uint32_t)(DWT->CYCCNT - start) < step * cycles_per_us) {
            __NOP();
        }
        us -= step;
    }
}

static void clock_init(void)
{
    if (HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY) != HAL_OK) {
        board_fatal(1u);
    }
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /* A bounded wait also works before SysTick has its final clock. */
    const uint32_t power_start = DWT->CYCCNT;
    while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY) == 0u) {
        if ((uint32_t)(DWT->CYCCNT - power_start) > SystemCoreClock / 10u) {
            board_fatal(1u);
        }
    }

    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 4;
    osc.PLL.PLLN = 200;
    osc.PLL.PLLP = 2;
    osc.PLL.PLLQ = 8;
    osc.PLL.PLLR = 2;
    osc.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    osc.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    osc.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        board_fatal(1u);
    }

    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 |
                    RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.SYSCLKDivider = RCC_SYSCLK_DIV1;
    clk.AHBCLKDivider = RCC_HCLK_DIV2;
    clk.APB3CLKDivider = RCC_APB3_DIV2;
    clk.APB1CLKDivider = RCC_APB1_DIV2;
    clk.APB2CLKDivider = RCC_APB2_DIV2;
    clk.APB4CLKDivider = RCC_APB4_DIV2;
    /* CPU 400 MHz / AHB 200 MHz, within older H743 revisions' limits. */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) {
        board_fatal(1u);
    }
    SystemCoreClockUpdate();

    RCC_PeriphCLKInitTypeDef peripheral = {0};
    peripheral.PeriphClockSelection = RCC_PERIPHCLK_SPI123 | RCC_PERIPHCLK_USB;
    peripheral.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    peripheral.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
    peripheral.PLL3.PLL3M = 4;
    peripheral.PLL3.PLL3N = 96;
    peripheral.PLL3.PLL3P = 2;
    peripheral.PLL3.PLL3Q = 8;
    peripheral.PLL3.PLL3R = 2;
    peripheral.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
    peripheral.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
    peripheral.PLL3.PLL3FRACN = 0;
    if (HAL_RCCEx_PeriphCLKConfig(&peripheral) != HAL_OK) {
        board_fatal(1u);
    }
}

static void output_init(GPIO_TypeDef *port, uint32_t pins, GPIO_PinState state,
                        bool open_drain)
{
    HAL_GPIO_WritePin(port, (uint16_t)pins, state);
    GPIO_InitTypeDef cfg = {0};
    cfg.Pin = pins;
    cfg.Mode = open_drain ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_OUTPUT_PP;
    cfg.Pull = GPIO_NOPULL;
    cfg.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &cfg);
}

static void gpio_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();

    output_init(GPIOE, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_SET, true);
    output_init(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET, false); /* Heater OFF. */
    output_init(GPIOG, GPIO_PIN_15, GPIO_PIN_SET, false); /* Ethernet OFF. */
    output_init(GPIOG, GPIO_PIN_4 | GPIO_PIN_10, GPIO_PIN_RESET, false);
    output_init(GPIOI, BMI_ACCEL_CS_PIN | BMI_GYRO_CS_PIN | GPIO_PIN_9,
                GPIO_PIN_SET, false);
    output_init(GPIOH, GPIO_PIN_5, GPIO_PIN_SET, false);
    output_init(GPIOG, GPIO_PIN_7, GPIO_PIN_SET, false);

    /* The supplied hwdef has four active-high sensor power domains. */
    output_init(GPIOI, GPIO_PIN_11, GPIO_PIN_SET, false);
    output_init(GPIOF, GPIO_PIN_4, GPIO_PIN_SET, false);
    output_init(GPIOE, GPIO_PIN_7, GPIO_PIN_SET, false);
    output_init(GPIOG, GPIO_PIN_8, GPIO_PIN_SET, false);

    GPIO_InitTypeDef cfg = {0};
    cfg.Mode = GPIO_MODE_AF_PP;
    cfg.Pull = GPIO_NOPULL;
    cfg.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    cfg.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    cfg.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &cfg);
    cfg.Pin = GPIO_PIN_2;
    cfg.Alternate = GPIO_AF7_SPI3; /* PB2 is AF7, NOT PC10/11's AF6. */
    HAL_GPIO_Init(GPIOB, &cfg);

    cfg.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    cfg.Alternate = GPIO_AF10_OTG1_FS;
    HAL_GPIO_Init(GPIOA, &cfg);
    cfg.Pin = GPIO_PIN_9;
    cfg.Mode = GPIO_MODE_INPUT;
    cfg.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &cfg);
}

static void timer_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->CR1 = 0;
    TIM2->PSC = 199u; /* APB1 timer clock = 200 MHz -> 1 MHz. */
    TIM2->ARR = UINT32_MAX;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;
    TIM2->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIM2_IRQn, 2u); /* DRDY first, paired snapshot second. */
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 = TIM_CR1_CEN;
}

void TIM2_IRQHandler(void)
{
    /* EXTI can preempt TIM2. Clear UIF and increment the high word atomically
     * so board_micros never observes a cleared flag with an old high word. */
    if ((TIM2->SR & TIM_SR_UIF) != 0u) {
        const uint32_t saved = __get_PRIMASK();
        __disable_irq();
        TIM2->SR = ~TIM_SR_UIF;
        ++timer_wraps;
        __set_PRIMASK(saved);
    }
    if ((TIM2->SR & TIM_SR_CC1IF) != 0u) {
        TIM2->SR = ~TIM_SR_CC1IF;
        const uint64_t actual = board_micros();
        uint32_t skipped = 0;
        if (actual > next_output_us) {
            skipped = (uint32_t)((actual - next_output_us) / IMU_OUTPUT_PERIOD_US);
        }
        const uint64_t target = next_output_us +
                                (uint64_t)skipped * IMU_OUTPUT_PERIOD_US;
        next_output_us = target + IMU_OUTPUT_PERIOD_US;
        TIM2->CCR1 = (uint32_t)next_output_us;
        /* A higher-priority DRDY can preempt while programming CCR1. If the
         * new compare time already passed, request service instead of waiting
         * for the next 71-minute counter wrap. Signed comparison is wrap-safe
         * because the next compare is at most one millisecond away. */
        if ((int32_t)(TIM2->CNT - (uint32_t)next_output_us) >= 0) {
            TIM2->EGR = TIM_EGR_CC1G;
        }
        stream_output_tick(target, actual, skipped);
    }
}

void board_output_timer_enable(void)
{
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    /* Absolute grid on the same 1 MHz timer as both DRDY timestamps. */
    next_output_us = (board_micros() / IMU_OUTPUT_PERIOD_US + 1u) *
                     IMU_OUTPUT_PERIOD_US;
    TIM2->CCR1 = (uint32_t)next_output_us;
    TIM2->SR = ~TIM_SR_CC1IF;
    TIM2->DIER |= TIM_DIER_CC1IE;
    if ((int32_t)(TIM2->CNT - (uint32_t)next_output_us) >= 0) {
        TIM2->EGR = TIM_EGR_CC1G;
    }
    __set_PRIMASK(saved);
}

uint64_t board_micros(void)
{
    const uint32_t saved = __get_PRIMASK();
    __disable_irq();
    uint32_t high = timer_wraps;
    uint32_t low = TIM2->CNT;
    /* Covers a wrap whose IRQ is pending, including a wrap during this read. */
    if ((TIM2->SR & TIM_SR_UIF) != 0u) {
        ++high;
        low = TIM2->CNT;
    }
    __set_PRIMASK(saved);
    return ((uint64_t)high << 32) | low;
}

static void spi_init(void)
{
    __HAL_RCC_SPI3_CLK_ENABLE();
    SPI_HandleTypeDef spi = {0};
    spi.Instance = SPI3;
    spi.Init.Mode = SPI_MODE_MASTER;
    spi.Init.Direction = SPI_DIRECTION_2LINES;
    spi.Init.DataSize = SPI_DATASIZE_8BIT;
    spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    spi.Init.CLKPhase = SPI_PHASE_2EDGE;
    spi.Init.NSS = SPI_NSS_SOFT;
    spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; /* 100/16 = 6.25 MHz. */
    spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi.Init.TIMode = SPI_TIMODE_DISABLE;
    spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    spi.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    spi.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    spi.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    spi.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    spi.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    spi.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    spi.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    spi.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    spi.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init(&spi) != HAL_OK) {
        board_fatal(2u);
    }
}

bool board_spi_transfer(bool accelerometer, const uint8_t *tx, uint8_t *rx,
                        size_t length)
{
    /* Called by one EXTI vector only after startup: no concurrent SPI owner.
     * A DWT deadline works inside an IRQ even when SysTick cannot preempt it.
     * No HAL polling timeout or DMA/cache ownership is involved. */
    if ((length == 0u) || (length > 16u)) {
        return false;
    }
    const uint16_t cs = accelerometer ? BMI_ACCEL_CS_PIN : BMI_GYRO_CS_PIN;
    const uint32_t deadline_cycles = SystemCoreClock / 5000u; /* 200 us maximum. */
    const uint32_t started = DWT->CYCCNT;
    size_t sent = 0;
    size_t received = 0;
    bool ok = true;
    SPI3->CR1 &= ~SPI_CR1_SPE;
    SPI3->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_OVRC |
                  SPI_IFCR_UDRC | SPI_IFCR_MODFC | SPI_IFCR_SUSPC;
    SPI3->CR2 = (uint32_t)length;
    /* Establish CPOL-high before selecting either die, including first use. */
    SPI3->CR1 |= SPI_CR1_SPE;
    HAL_GPIO_WritePin(BMI_GPIO, cs, GPIO_PIN_RESET);
    board_delay_us(1u);
    SPI3->CR1 |= SPI_CR1_CSTART;

    while (received < length) {
        const uint32_t status = SPI3->SR;
        if (((status & SPI_SR_TXP) != 0u) && (sent < length)) {
            *(__IO uint8_t *)&SPI3->TXDR = tx[sent++];
        }
        if ((status & SPI_SR_RXP) != 0u) {
            rx[received++] = *(__IO uint8_t *)&SPI3->RXDR;
        }
        if (((status & (SPI_SR_OVR | SPI_SR_UDR | SPI_SR_MODF)) != 0u) ||
            ((uint32_t)(DWT->CYCCNT - started) > deadline_cycles)) {
            ok = false;
            break;
        }
    }
    while (ok && ((SPI3->SR & SPI_SR_EOT) == 0u)) {
        if ((uint32_t)(DWT->CYCCNT - started) > deadline_cycles) {
            ok = false;
        }
    }
    if (!ok) {
        /* Abort a timed-out transaction before releasing CS. */
        SPI3->CR1 &= ~SPI_CR1_SPE;
    }
    board_delay_us(1u);
    HAL_GPIO_WritePin(BMI_GPIO, cs, GPIO_PIN_SET);
    SPI3->CR1 &= ~SPI_CR1_SPE;
    return ok;
}

void board_sampling_irq_enable(void)
{
    GPIO_InitTypeDef cfg = {0};
    cfg.Pin = BMI_ACCEL_DRDY_PIN | BMI_GYRO_DRDY_PIN;
    cfg.Mode = GPIO_MODE_IT_RISING;
    cfg.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(BMI_GPIO, &cfg);
    __HAL_GPIO_EXTI_CLEAR_IT(BMI_ACCEL_DRDY_PIN | BMI_GYRO_DRDY_PIN);
    NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    NVIC_SetPriority(EXTI9_5_IRQn, 1u);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void EXTI9_5_IRQHandler(void)
{
    /* Snapshot both flags/timestamps before reading either sensor. New edges
     * during SPI remain pending and cause a subsequent IRQ invocation. */
    const uint32_t pending = EXTI->PR1 & (BMI_ACCEL_DRDY_PIN | BMI_GYRO_DRDY_PIN);
    const uint64_t timestamp = board_micros();
    EXTI->PR1 = pending;
    if ((pending & BMI_GYRO_DRDY_PIN) != 0u) {
        bmi088_irq(false, timestamp);
    }
    if ((pending & BMI_ACCEL_DRDY_PIN) != 0u) {
        bmi088_irq(true, timestamp);
    }
}

void board_init(void)
{
    SCB->VTOR = FLASH_BASE + FLASH_OFFSET_KB * 1024u;
    __DSB();
    __ISB();
    HAL_Init();
    dwt_start();
    gpio_init();
    clock_init();
    SCB_EnableICache();
    /* D-cache intentionally stays disabled; USB buffers reside in D1 SRAM. */
    timer_init();
    spi_init();
    HAL_PWREx_EnableUSBVoltageDetector();
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    HAL_NVIC_SetPriority(SysTick_IRQn, 15u, 0u);
    NVIC_SetPriority(OTG_FS_IRQn, 3u);
    board_delay_us(50000u); /* Sensor rails/gyro POR stabilization. */
}

void board_leds(bool red, bool green, bool blue)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, red ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, green ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, blue ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

_Noreturn void board_fatal(unsigned code)
{
    __disable_irq();
    for (;;) {
        for (unsigned i = 0; i < code; ++i) {
            board_leds(true, false, false);
            board_delay_us(150000u);
            board_leds(false, false, false);
            board_delay_us(150000u);
        }
        board_delay_us(1000000u);
    }
}
