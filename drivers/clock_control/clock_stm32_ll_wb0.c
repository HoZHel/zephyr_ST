/*
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_pwr.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_radio.h>
#include <stm32_ll_system.h>
#include <stm32_ll_utils.h>
#include <zephyr/toolchain.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/common/sys_io.h>
#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>

/* Driver definitions */
#define RCC_REG(_reg_offset) (DT_REG_ADDR(STM32_CLOCK_CONTROL_NODE) + (_reg_offset))
#define RADIO_CTRL_IRQn	21	/* This IRQn is purposedly excluded from CMSIS and must be declared manually */
#define MIN_CALIB_WND_SIZE 16	/* MR_BLE User Manual recommendation to ensure good measurements from hardware */
#define MAX_CALIB_WND_SIZE 256	/* Hardware limitation */

/* Device tree node definitions */
#define DT_RCC_SLOWCLK_NODE	DT_PHANDLE(STM32_CLOCK_CONTROL_NODE, slow_clock)
#define DT_LSI_NODE		DT_NODELABEL(clk_lsi)

/* Device tree properties definitions */
#define STM32_WB0_CLKSYS_PRESCALER		\
	DT_PROP(STM32_CLOCK_CONTROL_NODE, clksys_prescaler)
#define STM32_WB0_LSI_MEASURE_WINDOW_SIZE	\
	DT_PROP(DT_LSI_NODE, measurement_window_size)

#if DT_NODE_HAS_PROP(STM32_CLOCK_CONTROL_NODE, slow_clock)
	#if DT_SAME_NODE(DT_RCC_SLOWCLK_NODE, DT_LSI_NODE)
		#define STM32_WB0_SLOWCLK_SRC	LL_RCC_LSCO_CLKSOURCE_LSI
	#elif DT_SAME_NODE(DT_RCC_SLOWCLK_NODE, DT_NODELABEL(clk_lse))
		#define STM32_WB0_SLOWCLK_SRC	LL_RCC_LSCO_CLKSOURCE_LSE
	#elif DT_SAME_NODE(DT_RCC_SLOWCLK_NODE, DT_NODELABEL(clk_16mhz_div512))
		#define STM32_WB0_SLOWCLK_SRC	LL_RCC_LSCO_CLKSOURCE_HSI64M_DIV2048
	#else
		#error invalid device selected for slow-clock
	#endif
#else /* !DT_NODE_HAS_PROP(STM32_CLOCK_CONTROL_NODE, slow_clock) */
	/* Fail build if user enabled a slow-clock-driven peripheral without
	 * specifying slow-clock on RCC to prevent runtime problems.
	 *
	 * Slow-clock-driven peripherals are:
	 *  - RTC
	 *  - IWDG
	 *  - MR_BLE (wakeup block)
	 */
	#if DT_NODE_HAS_STATUS(DT_NODELABEL(rtc), okay)
	#error Enabling RTC requires selecting slow-clock on RCC node
	#elif DT_NODE_HAS_STATUS(DT_NODELABEL(iwdg), okay)
	#error Enabling IWDG requires selecting slow-clock on RCC node
	#elif DT_NODE_HAS_STATUS(DT_NODELABEL(ble_rf), okay)
	#error Enabling BLE Radio requires selecting slow-clock on RCC node
	#endif
#endif /* DT_NODE_HAS_PROP(STM32_CLOCK_CONTROL_NODE, slow_clock) */

#if DT_NODE_HAS_PROP(DT_LSI_NODE, runtime_measurement_interval)
	#define STM32_WB0_RUNTIME_LSI_CALIBRATION 1
	#define STM32_WB0_LSI_RUNTIME_CALIB_INTERVAL 	\
		DT_PROP(DT_LSI_NODE, runtime_measurement_interval)
#endif /* DT_NODE_HAS_PROP(clk_lsi, runtime_calibration_settings) */

/* Verify device tree properties are correct */
BUILD_ASSERT(!IS_ENABLED(STM32_SYSCLK_SRC_HSE) || STM32_WB0_CLKSYS_PRESCALER != 64,
	"clksys-prescaler cannot be 64 when SYSCLK source is Direct HSE");
#ifdef STM32_LSI_ENABLED
BUILD_ASSERT(IN_RANGE(STM32_WB0_LSI_MEASURE_WINDOW_SIZE,
	MIN_CALIB_WND_SIZE, MAX_CALIB_WND_SIZE),
	"clk_lsi: invalid boot-calibration-window-size "
	"(must be between 16 and 256)");
#endif /* STM32_LSI_ENABLED */

/* Check clock configuration allows MR_BLE IP to work.
 * This IP is required to perform LSI measurements.
 */
#if defined(STM32_LSI_ENABLED)
#	if defined(STM32_SYSCLK_SRC_HSI)
		/* When using HSI without PLL, the "16MHz" output is not actually 16MHz, since
		 * the RC64M generator is imprecise. In this configuration, MR_BLE is broken.
		 * The CPU and MR_BLE must be running at 32MHz for MR_BLE to work with HSI.
		 */
		BUILD_ASSERT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC >= 32000000U,
			"STM32WB0: system clock frequency must be at least 32MHz to use LSI");
#	else
		/* In PLL or Direct HSE mode, the clock is stable, so 16MHz can be used. */
		BUILD_ASSERT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC >= 16000000U,
			"STM32WB0: system clock frequency must be at least 16MHz to use LSI");
#	endif /* STM32_SYSCLK_SRC_HSI */
#endif /* CONFIG_BT || STM32_LSI_ENABLED */


/** Kconfig -> LL conversion */
#define fn_mco_prescaler(v) LL_RCC_MCO_DIV_ ## v
#define mco_prescaler(v) fn_mco1_prescaler(v)

#if CONFIG_CLOCK_STM32_MCO1_SRC_NOCLOCK
	#define MCO_SOURCE		LL_RCC_MCOSOURCE_NOCLOCK
#elif CONFIG_CLOCK_STM32_MCO1_SRC_HSE
	#define MCO_SOURCE		LL_RCC_MCOSOURCE_HSE
#elif CONFIG_CLOCK_STM32_MCO1_SRC_HSI
	#define MCO_SOURCE		LL_RCC_MCOSOURCE_HSI
#elif CONFIG_CLOCK_STM32_MCO1_SRC_SYSCLK
	/* This should really be called CLK_SYS, as SYSCLK on
	 * STM32WB0 is actually a fixed 32/64MHz frequency...
	 */
	#define MCO_SOURCE		LL_RCC_MCOSOURCE_SYSCLK
#else
	/* Remaining unsupported sources:
	 *  - HSI64M / 2048 (32kHz derived from HSI)
	 *  - SMPS (4MHz, derived from SYSCLK)
	 *  - ADC (16MHz, derived from SYSCLK)
	 */
#endif


/**
 * @brief Variable holding the "current frequency of LSI", according
 * to the measurement process. This variable is updated each time
 * a new measurement of the LSI frequency is performed.
 */
static volatile uint32_t LsiFrequency = STM32_LSI_FREQ;

/**
 * @brief Perform a measurement of the LSI frequency and updates
 * the @p LsiFrequency global variable based on the results.
 *
 * @param wait_event	Semaphore to wait for completion of the measurement
 * 			If NULL, RADIO_CONTROL registers are polled instead.
 */
static void measure_lsi_frequency(struct k_sem *wait_event)
{
	uint32_t fast_clock_cycles_elapsed;

	/* Ensure calibration flag is clear */
	LL_RADIO_TIMER_ClearFlag_LSICalibrationEnded(RADIO_CTRL);

	/* Setup the calibration parameters
	 *
	 * NOTE: (size - 1) is required to get the correct count,
	 * because the value in the register is one less than the
	 * actual number of periods requested for calibration.
	 */
	LL_RADIO_TIMER_SetLSIWindowCalibrationLength(
		RADIO_CTRL, (STM32_WB0_LSI_MEASURE_WINDOW_SIZE - 1));

	/* Start LSI calibration */
	LL_RADIO_TIMER_StartLSICalibration(RADIO_CTRL);

	if (wait_event) {
		/* Wait for semaphore to be signaled */
		k_sem_take(wait_event, K_FOREVER);
	} else {
		while (!LL_RADIO_TIMER_IsActiveFlag_LSICalibrationEnded(RADIO_CTRL)) {
			/* Wait for calibration to finish (polling) */
		}

		/* Clear calibration complete flag / interrupt */
		LL_RADIO_TIMER_ClearFlag_LSICalibrationEnded(RADIO_CTRL);
	}

	/* Read calibration results */
	fast_clock_cycles_elapsed = LL_RADIO_TIMER_GetLSIPeriod(RADIO_CTRL);

	/**
	 * Calculate LSI frequency from calibration results and update
	 * the corresponding global variable
	 *
	 * LSI calibration counts the amount of 16MHz clock half-periods that
	 * occur until a certain amount of slow clock cycles have been observed.
	 *
	 * @p fast_clock_cycles_elapsed is the number of 16MHz clock half-periods
	 * elapsed while waiting for @p STM32_WB0_LSI_MEASURE_WINDOW_SIZE LSI periods
	 * to occur. The LSI frequency can be calculated the following way:
	 *
	 * t = <number of periods counted> / <clock frequency>
	 *
	 * ==> Time taken for calibration:
	 *
	 * tCALIB = @p fast_clock_cycles_elapsed / (2 * 16MHz)
	 *
	 * ==> LSI period:
	 *
	 * tLSI = tCALIB / @p STM32_WB0_LSI_MEASURE_WINDOW_SIZE
	 *
	 * 	   ( @p fast_clock_cycles_elapsed / (2 * 16MHz) )
	 *      = ------------------------------------------------
	 *             @p STM32_WB0_LSI_MEASURE_WINDOW_SIZE
	 *
	 * ==> LSI frequency:
	 *
	 * fLSI = (1 / tLSI)
	 *
	 *             @p STM32_WB0_LSI_MEASURE_WINDOW_SIZE
	 *      = ------------------------------------------------
	 * 	   ( @p fast_clock_cycles_elapsed / (2 * 16MHz) )
	 *
	 *         (2 * 16MHz) * @p STM32_WB0_LSI_MEASURE_WINDOW_SIZE
	 *      = -----------------------------------------------------
	 *                   @p fast_clock_cycles_elapsed
	 *
	 * NOTE: The division must be performed first to avoid 32-bit overflow.
	 */
	LsiFrequency = (32000000U / fast_clock_cycles_elapsed) * STM32_WB0_LSI_MEASURE_WINDOW_SIZE;
}

/** @brief Verifies clock is part of active clock configuration */
__unused
static int check_domain_clock_ok(uint32_t src_clk)
{
	int r = 0;

	switch (src_clk) {
	case STM32_SRC_SYSCLK:
		break;
	/* TODO: should those two be removed? */
	case STM32_SRC_LSE:
		if (!IS_ENABLED(STM32_LSE_ENABLED)) {
			r = -ENOTSUP;
		}
		break;
	case STM32_SRC_LSI:
		if (!IS_ENABLED(STM32_LSI_ENABLED)) {
			r = -ENOTSUP;
		}
		break;
	/* TODO: support CLK_SYS? (would be just for PKA periph) */
	case STM32_SRC_CLKSLOWMUX:
		break;
	case STM32_SRC_CLK16MHZ:
		break;
	case STM32_SRC_CLK32MHZ:
		break;
	default:
		return -ENOTSUP;
	}

	return r;
}

static inline int stm32_clock_control_on(const struct device *dev,
					 clock_control_subsys_t sub_system)
{
	struct stm32_pclken *pclken = (struct stm32_pclken *)(sub_system);
	const mem_addr_t reg = RCC_REG(pclken->bus);
	volatile uint32_t temp;

	ARG_UNUSED(dev);
	if (!IN_RANGE(pclken->bus, STM32_PERIPH_BUS_MIN, STM32_PERIPH_BUS_MAX)) {
		/* Attempting to change domain clock */
		return -ENOTSUP;
	}

	sys_set_bits(reg, pclken->enr);

	/* Read back register to be blocked by RCC
	 * until peripheral clock enabling is complete */
	temp = sys_read32(reg);
	UNUSED(temp);

	return 0;
}

static inline int stm32_clock_control_off(const struct device *dev,
					  clock_control_subsys_t sub_system)
{
	struct stm32_pclken *pclken = (struct stm32_pclken *)(sub_system);
	const mem_addr_t reg = RCC_REG(pclken->bus);

	ARG_UNUSED(dev);
	if (!IN_RANGE(pclken->bus, STM32_PERIPH_BUS_MIN, STM32_PERIPH_BUS_MAX)) {
		/* Attempting to change domain clock */
		return -ENOTSUP;
	}

	sys_clear_bits(reg, pclken->enr);

	return 0;
}

static inline int stm32_clock_control_configure(const struct device *dev,
						clock_control_subsys_t sub_system,
						void *data)
{
	struct stm32_pclken *pclken = (struct stm32_pclken *)(sub_system);
	mem_addr_t reg;
	int err;

	ARG_UNUSED(dev);
	ARG_UNUSED(data);

	err = check_domain_clock_ok(pclken->bus);
	if (err < 0) {
		/* Attempt to configure an unavailable or invalid src clock */
		return err;
	}

	reg = RCC_REG(STM32_CLOCK_REG_GET(pclken->enr));

	sys_clear_bits(reg, STM32_CLOCK_MASK_GET(pclken->enr) << STM32_CLOCK_SHIFT_GET(pclken->enr));
	sys_set_bits(reg, STM32_CLOCK_VAL_GET(pclken->enr) << STM32_CLOCK_SHIFT_GET(pclken->enr));

	return 0;
}

static inline int get_apb0_periph_clkrate(uint32_t enr, uint32_t* rate,
	uint32_t slow_clock, uint32_t sysclk, uint32_t clk_sys)
{
	switch (enr) {
	/* Slow clock peripherals: RTC & IWDG */
	case LL_APB0_GRP1_PERIPH_RTC:
	case LL_APB0_GRP1_PERIPH_WDG:
		*rate = slow_clock;
		break;

	/* SYSCLK peripherals: all timers */
	/* TODO: this may not be valid for WB06/WB07 */
#if defined(TIM1)
	case LL_APB0_GRP1_PERIPH_TIM1:
#endif
#if defined(TIM2)
	case LL_APB0_GRP1_PERIPH_TIM2:
#endif
#if defined(TIM16)
	case LL_APB0_GRP1_PERIPH_TIM16:
#endif
#if defined(TIM17)
	case LL_APB0_GRP1_PERIPH_TIM17:
#endif
		*rate = sysclk;
		break;

	/* CLK_SYS peripherals: SYSCFG */
	case LL_APB0_GRP1_PERIPH_SYSCFG:
		*rate = clk_sys;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static inline int get_apb1_periph_clkrate(uint32_t enr, uint32_t* rate,
	uint32_t clk_sys)
{
	/* TODO: define these as HSI_VALUE / 4, HSI_VALUE / 2 and HSI_VALUE? */
	const uint32_t clk_16mhz = 16000000U;
	const uint32_t clk_32mhz = 32000000U;
	const uint32_t __unused clk_64mhz = 64000000U;
	switch (enr) {
#if defined(SPI1)
	case LL_APB1_GRP1_PERIPH_SPI1:
		*rate = clk_sys;
		break;
#endif
#if defined(SPI2)
	case LL_APB1_GRP1_PERIPH_SPI2:
		switch (LL_RCC_GetSPI2I2SClockSource()) {
		case LL_RCC_SPI2_I2S_CLK16M:
			*rate = clk_16mhz;
			break;
		case LL_RCC_SPI2_I2S_CLK32M:
			*rate = clk_32mhz;
			break;
		default:
			CODE_UNREACHABLE;
		}
		break;
#endif
	case LL_APB1_GRP1_PERIPH_SPI3:
		switch (LL_RCC_GetSPI3I2SClockSource()) {
		case LL_RCC_SPI3_I2S_CLK16M:
			*rate = clk_16mhz;
			break;
		case LL_RCC_SPI3_I2S_CLK32M:
			*rate = clk_32mhz;
			break;
#if defined(LL_RCC_SPI3_I2S_CLK64M)
		case LL_RCC_SPI3_I2S_CLK64M:
			*rate = clk_64mhz;
			break;
#endif
		default:
			CODE_UNREACHABLE;
		}
		break;

	/* TODO: BlueNRG-LP I2C is clocked @ 16MHz?*/
	case LL_APB1_GRP1_PERIPH_I2C1:
#if defined(I2C2)
	case LL_APB1_GRP1_PERIPH_I2C2:
#endif
		*rate = clk_sys;
		break;

	case LL_APB1_GRP1_PERIPH_ADCDIG:
	case LL_APB1_GRP1_PERIPH_ADCANA:
	case (LL_APB1_GRP1_PERIPH_ADCDIG | LL_APB1_GRP1_PERIPH_ADCANA):
	/* ADC has two bits for enable... support all combinations */
	/* TODO: BlueNRG-LP ADC is clocked @ 32MHz?*/
		*rate = clk_16mhz;
		break;

	case LL_APB1_GRP1_PERIPH_USART1:
		*rate = clk_16mhz;
		break;

	case LL_APB1_GRP1_PERIPH_LPUART1:
#if !defined(RCC_CFGR_LPUCLKSEL)
		*rate = clk_16mhz;
#else
		switch (LL_RCC_GetLPUARTClockSource()) {
		case LL_RCC_LPUCLKSEL_CLK16M:
			*rate = clk_16mhz;
			break;
		case LL_RCC_LPUCLKSEL_CLKLSE:
			*rate = LSE_VALUE;
			break;
		default:
			CODE_UNREACHABLE;
		}
#endif
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int stm32_clock_control_get_subsys_rate(const struct device *dev,
						clock_control_subsys_t sub_system,
						uint32_t *rate)
{

	struct stm32_pclken *pclken = (struct stm32_pclken *)(sub_system);
	uint32_t sysclk, slow_clock, clk_sys;
	ARG_UNUSED(dev);

	/* Obtain SYSCLK frequency by checking which source drives high-speed clock tree.
	 * If Direct HSE is enabled, the high-speed tree is clocked by HSE @ 32MHz.
	 * Otherwise, the high-speed tree is clocked by the RC64MPLL clock @ 64MHz.
	 *
	 * NOTE: it is NOT possible to use the usual 'SystemCoreClock * Prescaler' approach on
	 * STM32WB0 because the prescaler configuration is not affected by input clock variation:
	 * setting CLKSYSDIV = 1 results in 32MHz CLK_SYS, regardless of SYSCLK being 32 or 64MHZ.
	 */
	if (LL_RCC_DIRECT_HSE_IsEnabled()) {
		sysclk = HSE_VALUE;
	} else {
		sysclk = HSI_VALUE;
	}

	/* Obtain CLK_SYS (AHB0) frequency by using the CLKSYSDIV prescaler value.
	 *
	 * NOTE: LL_RCC_GetRC64MPLLPrescaler is strictly identical to LL_RCC_GetDirectHSEPrescaler
	 * and can be used regardless of which source is driving the high-speed clock tree.
	 *
	 * NOTE: the prescaler value must be interpreted as if source clock is 64MHz, regardless
	 * of which source is actually driving the high-speed clock tree. This allows using the
	 * following formula for calcualtions.
	 *
	 * NOTE: (x >> y) is equivalent to (x / 2^y) or (x / (1 << y)).
	 */
	/* TODO: use HSI_VALUE instead of 64M? */
	clk_sys = (64000000U) >> LL_RCC_GetRC64MPLLPrescaler();

	/* Obtain slow clock tree source by reading RCC_CFGR->LCOSEL.
	 * From this, we can deduce at which frequency the slow clock tree is running.
	 */
	switch (LL_RCC_LSCO_GetSource()) {
	case LL_RCC_LSCO_CLKSOURCE_LSE:
		slow_clock = LSE_VALUE;
		break;
	case LL_RCC_LSCO_CLKSOURCE_LSI:
		slow_clock = LsiFrequency;
		break;
	case LL_RCC_LSCO_CLKSOURCE_HSI64M_DIV2048:
		slow_clock = HSI_VALUE / 2048;
		break;
	default:
		__ASSERT(0, "Illegal slow clock source!");
		CODE_UNREACHABLE;
	}

	switch (pclken->bus) {
	case STM32_CLOCK_BUS_AHB0:
	/* All peripherals on AHB0 are clocked by CLK_SYS. */
		*rate = clk_sys;
		break;
	case STM32_CLOCK_BUS_APB0:
		return get_apb0_periph_clkrate(pclken->enr, rate,
			slow_clock, sysclk, clk_sys);
	case STM32_CLOCK_BUS_APB1:
		return get_apb1_periph_clkrate(pclken->enr, rate,
			clk_sys);
	case STM32_SRC_SYSCLK:
		*rate = sysclk;
		break;
	case STM32_SRC_LSE:
		*rate = LSE_VALUE;
		break;
	case STM32_SRC_LSI:
		*rate = LsiFrequency;
		break;
	case STM32_SRC_CLKSLOWMUX:
		*rate = slow_clock;
		break;
	case STM32_SRC_CLK16MHZ:
		*rate = 16000000U; //TODO: HSI_VALUE / 4?
		break;
	case STM32_SRC_CLK32MHZ:
		*rate = 32000000U; //TODO: HSI_VALUE / 2?
		break;
	default:
	case STM32_CLOCK_BUS_APB2:
		/* The only periperhal on APB2 is the MR_BLE radio. However,
		 * is is clocked by two sources that run at different frequencies,
		 * and we are unable to determine which one this API's caller cares
		 * about. For this reason, return ENOTSUP anyways.
		 *
		 * Note that since the only driver that might call this API is the
		 * Bluetooth driver, and since it can already determine both frequencies
		 * very easily, this should not pose any problem.
		 */
		return -ENOTSUP;
	}

	return 0;
}

static enum clock_control_status stm32_clock_control_get_status(const struct device *dev,
								clock_control_subsys_t sub_system)
{
	struct stm32_pclken *pclken = (struct stm32_pclken *)sub_system;

	ARG_UNUSED(dev);

	if (IN_RANGE(pclken->bus, STM32_PERIPH_BUS_MIN, STM32_PERIPH_BUS_MAX)) {
		/* Bus / gated clock */
		if ((sys_read32(RCC_REG(pclken->bus)) & pclken->enr) == pclken->enr) {
			return CLOCK_CONTROL_STATUS_ON;
		} else {
			return CLOCK_CONTROL_STATUS_OFF;
		}
	} else {
		/* Domain clock */
		if (check_domain_clock_ok(pclken->bus) == 0) {
			return CLOCK_CONTROL_STATUS_ON;
		} else {
			return CLOCK_CONTROL_STATUS_OFF;
		}
	}

}

static struct clock_control_driver_api stm32_clock_control_api = {
	.on = stm32_clock_control_on,
	.off = stm32_clock_control_off,
	.get_rate = stm32_clock_control_get_subsys_rate,
	.get_status = stm32_clock_control_get_status,
	.configure = stm32_clock_control_configure,
};

static void set_up_fixed_clock_sources(void)
{

	if (IS_ENABLED(STM32_HSE_ENABLED)) {
#if defined(STM32_HSE_BYPASS)
		/* Check if need to enable HSE bypass feature or not */
		if (IS_ENABLED(STM32_HSE_BYPASS)) {
			LL_RCC_HSE_EnableBypass();
		} else {
			LL_RCC_HSE_DisableBypass();
		}
#endif
		/* Enable HSE */
		LL_RCC_HSE_Enable();
		while (LL_RCC_HSE_IsReady() != 1) {
			/* Wait for HSE ready */
		}
	}

	if (IS_ENABLED(STM32_HSI_ENABLED)) {
		/* Enable HSI if not enabled */
		if (LL_RCC_HSI_IsReady() != 1) {
			/* Enable HSI */
			LL_RCC_HSI_Enable();
			while (LL_RCC_HSI_IsReady() != 1) {
			/* Wait for HSI ready */
			}
		}
	}

	if (IS_ENABLED(STM32_LSI_ENABLED)) {
		LL_RCC_LSI_Enable();
		while (LL_RCC_LSI_IsReady() != 1) {
			/* Wait for LSI ready */
		}
	}

	if (IS_ENABLED(STM32_LSE_ENABLED)) {
#if STM32_LSE_DRIVING
		/* Configure driving capability */
		LL_RCC_LSE_SetDriveCapability(STM32_LSE_DRIVING << RCC_CSSWCR_LSEDRV_Pos);
#endif
		/* Unconditionally disable pull-up & pull-down on LSE pins */
		LL_PWR_SetNoPullB(LL_PWR_GPIO_BIT_12 | LL_PWR_GPIO_BIT_13);

		if (IS_ENABLED(STM32_LSE_BYPASS)) {
			/* Configure LSE bypass */
			LL_RCC_LSE_EnableBypass();
		}

		/* Enable LSE Oscillator (32.768 kHz) */
		LL_RCC_LSE_Enable();
		while (!LL_RCC_LSE_IsReady()) {
			/* Wait for LSE ready */
		}
	}
}

static void configure_mco(void)
{
#ifndef CONFIG_CLOCK_STM32_MCO1_SRC_NOCLOCK
	LL_RCC_ConfigMCO(MCO_SOURCE,
			 mco1_prescaler(CONFIG_CLOCK_STM32_MCO1_DIV));
#endif /* CONFIG_CLOCK_STM32_MCO1_SRC_NOCLOCK */
}

/* The STM32WB0 prescaler division factor defines vary depending on
 * whether SYSCLK runs at 32MHz (Direct HSE) or 64MHz (RC64MPLL).
 * The following helper macro wraps this difference.
 */
#if defined(STM32_SYSCLK_SRC_HSE)
#define LL_PRESCALER(x) LL_RCC_DIRECT_HSE_DIV_ ##x
#else
#define LL_PRESCALER(x) LL_RCC_RC64MPLL_DIV_ ##x
#endif

/**
 * @brief Converts the Kconfig STM32_WB0_CLKSYS_PRESCALER option
 * to a LL_RCC_RC64MPLL_DIV_x value understandable by the LL.
 */
static inline uint32_t kconfig_to_ll_prescaler(uint32_t kcfg_pre) {
	switch (kcfg_pre) {
	case 1:
		return LL_PRESCALER(1);
	case 2:
		return LL_PRESCALER(2);
	case 4:
		return LL_PRESCALER(4);
	case 8:
		return LL_PRESCALER(8);
	case 16:
		return LL_PRESCALER(16);
	case 32:
		return LL_PRESCALER(32);
#if !defined(STM32_SYSCLK_SRC_HSE)
	/* A prescaler value of 64 is only valid when running
	 * off RC64MPLL because CLK_SYS must be at least 1MHz
	 */
	case 64:
		return LL_RCC_RC64MPLL_DIV_64;
#endif
	}
	CODE_UNREACHABLE;
}
#undef LL_PRESCALER	/* Undefine helper macro */

#if defined(STM32_WB0_RUNTIME_LSI_CALIBRATION)
K_SEM_DEFINE(lsi_measurement_sema, 0, 1);

#define NUM_SLOW_CLOCK_PERIPHERALS 3

/**
 * Reserve one slot for each slow clock peripheral to ensure each
 * peripheral's driver can register a callback to cope with clock drift.
 */
static lsi_update_cb_t lsi_update_callbacks[NUM_SLOW_CLOCK_PERIPHERALS];

int stm32wb0_register_lsi_update_callback(lsi_update_cb_t cb)
{
	for (size_t i = 0; i < ARRAY_SIZE(lsi_update_callbacks); i++)
	{
		if (lsi_update_callbacks[i] == NULL)
		{
			lsi_update_callbacks[i] = cb;
			return 0;
		}
	}
	return -ENOMEM;
}

static void radio_ctrl_isr(void)
{
	/* Clear calibration complete flag / interrupt */
	LL_RADIO_TIMER_ClearFlag_LSICalibrationEnded(RADIO_CTRL);

	/* Release the measurement thread */
	k_sem_give(&lsi_measurement_sema);
}

static void lsi_rt_measure_loop(void)
{
	uint32_t old, new;
	while (1)
	{
		/* Sleep until calibration interval elapses */
		k_sleep(K_MSEC(STM32_WB0_LSI_RUNTIME_CALIB_INTERVAL));

		old = LsiFrequency;

		/* Ensure the MR_BLE IP clock is enabled. */
		if (!LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_MRBLE)) {
			LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_MRBLE);
		}

		/* Perform measurement, making sure we sleep on the semaphore
		 * signaled by the "measurement complete" interrupt handler */
		measure_lsi_frequency(&lsi_measurement_sema);

		new = LsiFrequency;

		/* If LSI frequency changed, invoke all registered callbacks */
		if (new != old) {
			for (size_t i = 0; i < ARRAY_SIZE(lsi_update_callbacks); i++) {
				if (lsi_update_callbacks[i]) {
					lsi_update_callbacks[i](LsiFrequency);
				}
			}
		}
	}
}

/* TODO: Kconfig option for stack size? */
/* TODO: Kconfig option for thread prio? */
#define LSI_RTM_THREAD_STACK_SIZE 512
#define LSI_RTM_THREAD_PRIORITY K_LOWEST_THREAD_PRIO

K_KERNEL_THREAD_DEFINE(lsi_rt_measurement_thread,
		LSI_RTM_THREAD_STACK_SIZE,
		lsi_rt_measure_loop,
		NULL, NULL, NULL,
		LSI_RTM_THREAD_PRIORITY, /* No options */ 0,
		/* No delay (automatic start by kernel) */ 0);
#endif

int stm32_clock_control_init(const struct device *dev) {

	ARG_UNUSED(dev);

	/* Set flash latency according to target CLK_SYS frequency:
	 *  - 1 wait state when CLK_SYS > 32MHz (i.e., 64MHz configuration)
	 *  - 0 wait states otherwise (CLK_SYS <= 32MHz)
	 */
	LL_FLASH_SetLatency(
	(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC >= 32000000U)
			? LL_FLASH_LATENCY_1
			: LL_FLASH_LATENCY_0
	);

	/* Unconditionally enable SYSCFG clock for other drivers */
	LL_APB0_GRP1_EnableClock(LL_APB0_GRP1_PERIPH_SYSCFG);

	/* Set up indiviual enabled clocks */
	set_up_fixed_clock_sources();

	/* Set up the slow clock mux */
#if defined(STM32_WB0_SLOWCLK_SRC)
	LL_RCC_LSCO_SetSource(STM32_WB0_SLOWCLK_SRC);
#endif

#if defined(STM32_SYSCLK_SRC_HSE)
	/* Select Direct HSE as SYSCLK source */
	LL_RCC_DIRECT_HSE_Enable();

	while (LL_RCC_DIRECT_HSE_IsEnabled() == 0) {
		/* Wait until Direct HSE is ready */
	}
#elif defined(STM32_SYSCLK_SRC_HSI) || defined(STM32_SYSCLK_SRC_PLL)
	/* Select RC64MPLL (HSI/PLL) block as SYSCLK source. */
	LL_RCC_DIRECT_HSE_Disable();

#	if defined(STM32_SYSCLK_SRC_PLL)
BUILD_ASSERT(IS_ENABLED(STM32_HSE_ENABLED),
	"STM32WB0 PLL requires HSE to be enabled!");

	/* Turn on the PLL part of RC64MPLL block */
	LL_RCC_RC64MPLL_Enable();
	while(LL_RCC_RC64MPLL_IsReady() == 0) {
		/* Wait until PLL is ready */
	}

#	endif /* STM32_SYSCLK_SRC_PLL */
#endif /* STM32_SYSCLK_SRC_* */

	/* Set CLK_SYS prescaler */
	LL_RCC_SetRC64MPLLPrescaler(
		kconfig_to_ll_prescaler(STM32_WB0_CLKSYS_PRESCALER));

	SystemCoreClock = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;

	/* configure MCO based on Kconfig */
	configure_mco();

	//TODO: LL_RCC_HSE_SetCapacitorTuning
	//TODO: LL_RCC_HSE_SetCurrentControl

	if (IS_ENABLED(STM32_LSI_ENABLED)) {
		/* TODO: break build if CPU / BLE clock configuration is insufficient */

		/* Enable MR_BLE clock for LSI measurement.
		 * This is needed because we use part of the MR_BLE hardware
		 * to perform this measurement.
		 */
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_MRBLE);

		/* Perform a measure of the LSI frequency */
		measure_lsi_frequency(NULL);

		/* Disable the MR_BLE clock after the measurement */
		LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_MRBLE);
	}

#if STM32_WB0_RUNTIME_LSI_CALIBRATION
	/* Enable LSI measurement complete IRQ at NVIC level */
	IRQ_CONNECT(RADIO_CTRL_IRQn, IRQ_PRIO_LOWEST,
		radio_ctrl_isr, NULL, 0);
	irq_enable(RADIO_CTRL_IRQn);

	/* Unmask IRQ at peripheral level */
	LL_RADIO_TIMER_EnableLSICalibrationIT(RADIO_CTRL);
#endif

	return 0;
}

/**
 * @brief RCC device.
 * Note that priority is intentionally set to 1,
 * so that the RCC init runs just after SoC init
 */
DEVICE_DT_DEFINE(STM32_CLOCK_CONTROL_NODE,
		    &stm32_clock_control_init,
		    NULL, NULL, NULL,
		    PRE_KERNEL_1,
		    CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		    &stm32_clock_control_api);
