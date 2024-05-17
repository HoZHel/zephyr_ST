
/*
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System/hardware module for STM32WB0 processor
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_pwr.h>
#include <stm32_ll_system.h>
#include <stm32_ll_radio.h>
#include <zephyr/logging/log.h>
#include <zephyr/toolchain.h>
#include <cmsis_core.h>
#include <stdint.h>

#include <system_stm32wb0x.h>

#define LOG_LEVEL CONFIG_SOC_LOG_LEVEL
LOG_MODULE_REGISTER(soc);

/**
 * Hardware trimming definitions
*/
#define VALIDITY_TAG      0xFCBCECCC  /*!< TAG value indicating trimming area is valid */
#define VALIDITY_LOCATION 0x10001EF8  /*!< ROM address of trimming values validity tag */

/**
 * CMSIS System Core Clock: global variable holding the system core clock,
 * which is the frequency supplied to the SysTick timer and processor core.
 *
 * On STM32WB0 series, after RESET, the system clock frequency is 16MHz.
 */
uint32_t SystemCoreClock = 16000000U;

/**
 * RAM Virtual Register: special structure located at the start
 * of SRAM0; used by the UART bootloader and the Low Power Manager.
 * Data type definition comes from @file system_stm32wb0xx.h
 */
Z_GENERIC_SECTION("stm32wb0_RAM_VR")
RAM_VR_TypeDef __used RAM_VR;

#if defined(CONFIG_BT)
/**
 * SRAM0 memory reserved for usage by the MR_BLE Radio hardware.
 *
 * N.B.: radio driver defines CFG_BLE_NUM_RADIO_TASKS.
 */
Z_GENERIC_SECTION("stm32wb0_BLUE_RAM")
static uint8_t __used __blue_RAM[sizeof(GLOBALSTATMACH_TypeDef) +
		CFG_BLE_NUM_RADIO_TASKS * sizeof(STATMACH_TypeDef)];
#endif


/** Convert Kconfig options to LL definitions if SMPS is enabled */
#if defined(STM32WB0_SMPS_ENABLED)

#if defined(CONFIG_SMPS_BOM_1)
#	define	BOARD_SMPS_BOM	LL_PWR_SMPS_BOM1
#elif defined(CONFIG_SMPS_BOM_2)
#	define	BOARD_SMPS_BOM	LL_PWR_SMPS_BOM2
#elif defined(CONFIG_SMPS_BOM_3)
#	define	BOARD_SMPS_BOM	LL_PWR_SMPS_BOM3
#else
#	error	Cannot enable SMPS if no SMPS BOM is selected.
#endif /* CONFIG_SMPS_BOM_* */

#if defined(CONFIG_SMPS_DEEPSTOP_MODE_OPEN)
#define	SMPS_LOW_POWER_MODE	LL_PWR_SMPS_LPOPEN
#elif defined(CONFIG_SMPS_DEEPSTOP_MODE_BYPASS)
#define SMPS_LOW_POWER_MODE	LL_PWR_NO_SMPS_LPOPEN
#endif /* CONFIG_SMPS_LOW_POWER_* */

#if defined(CONFIG_SMPS_CURRENT_LIMIT_2_5_MA)
#	define SMPS_CURRENT_LIMIT	LL_PWR_SMPS_PRECH_LIMIT_CUR_2_5
#elif defined(CONFIG_SMPS_CURRENT_LIMIT_5_MA)
#	define SMPS_CURRENT_LIMIT	LL_PWR_SMPS_PRECH_LIMIT_CUR_5
#elif defined(CONFIG_SMPS_CURRENT_LIMIT_10_MA)
#	define SMPS_CURRENT_LIMIT	LL_PWR_SMPS_PRECH_LIMIT_CUR_10
#elif defined(CONFIG_SMPS_CURRENT_LIMIT_20_MA)
#	define SMPS_CURRENT_LIMIT	LL_PWR_SMPS_PRECH_LIMIT_CUR_20
#endif /* SMPS_CURRENT_LIMIT_* */

#if defined(CONFIG_SMPS_OUTPUT_VOLTAGE_1_20V)
#	define SMPS_OUTPUT_VOLTAGE	LL_PWR_SMPS_OUTPUT_VOLTAGE_1V20
#elif defined(CONFIG_SMPS_OUTPUT_VOLTAGE_1_25V)
#	define SMPS_OUTPUT_VOLTAGE	LL_PWR_SMPS_OUTPUT_VOLTAGE_1V25
#elif defined(CONFIG_SMPS_OUTPUT_VOLTAGE_1_30V)
#	define SMPS_OUTPUT_VOLTAGE	LL_PWR_SMPS_OUTPUT_VOLTAGE_1V30
#elif defined(CONFIG_SMPS_OUTPUT_VOLTAGE_1_35V)
#	define SMPS_OUTPUT_VOLTAGE	LL_PWR_SMPS_OUTPUT_VOLTAGE_1V35
#elif defined(CONFIG_SMPS_OUTPUT_VOLTAGE_1_40V)
#	define SMPS_OUTPUT_VOLTAGE	LL_PWR_SMPS_OUTPUT_VOLTAGE_1V40
#elif defined(CONFIG_SMPS_OUTPUT_VOLTAGE_1_45V)
#	define SMPS_OUTPUT_VOLTAGE	LL_PWR_SMPS_OUTPUT_VOLTAGE_1V45
#elif defined(CONFIG_SMPS_OUTPUT_VOLTAGE_1_50V)
#	define SMPS_OUTPUT_VOLTAGE	LL_PWR_SMPS_OUTPUT_VOLTAGE_1V50
#elif defined(CONFIG_SMPS_OUTPUT_VOLTAGE_1_55V)
#	define SMPS_OUTPUT_VOLTAGE	LL_PWR_SMPS_OUTPUT_VOLTAGE_1V55
#elif defined(CONFIG_SMPS_OUTPUT_VOLTAGE_1_60V)
#	define SMPS_OUTPUT_VOLTAGE	LL_PWR_SMPS_OUTPUT_VOLTAGE_1V60
#elif defined(CONFIG_SMPS_OUTPUT_VOLTAGE_1_65V)
#	define SMPS_OUTPUT_VOLTAGE	LL_PWR_SMPS_OUTPUT_VOLTAGE_1V65
#elif defined(CONFIG_SMPS_OUTPUT_VOLTAGE_1_70V)
#	define SMPS_OUTPUT_VOLTAGE	LL_PWR_SMPS_OUTPUT_VOLTAGE_1V70
#elif defined(CONFIG_SMPS_OUTPUT_VOLTAGE_1_75V)
#	define SMPS_OUTPUT_VOLTAGE	LL_PWR_SMPS_OUTPUT_VOLTAGE_1V75
#elif defined(CONFIG_SMPS_OUTPUT_VOLTAGE_1_80V)
#	define SMPS_OUTPUT_VOLTAGE	LL_PWR_SMPS_OUTPUT_VOLTAGE_1V80
#elif defined(CONFIG_SMPS_OUTPUT_VOLTAGE_1_85V)
#	define SMPS_OUTPUT_VOLTAGE	LL_PWR_SMPS_OUTPUT_VOLTAGE_1V85
#elif defined(CONFIG_SMPS_OUTPUT_VOLTAGE_1_90V)
#	define SMPS_OUTPUT_VOLTAGE	LL_PWR_SMPS_OUTPUT_VOLTAGE_1V90
#elif defined(CONFIG_SMPS_OUTPUT_VOLTAGE_1_95V)
#	define SMPS_OUTPUT_VOLTAGE	LL_PWR_SMPS_OUTPUT_VOLTAGE_1V95
#endif /* SMPS_OUTPUT_VOLTAGE_* */

#endif /* STM32WB0_SMPS_ENABLED */

/**
 * @brief Applies default values to trimming registers
 */
static void apply_default_trimming(void)
{
	uint32_t main_regulator, smps_out_voltage, hsi_calib;
#if defined(PWR_ENGTRIM_TRIM_LSI_LPMU)
	hsi_calib = 0x1E;
	main_regulator = 0x08;
	smps_out_voltage = 0x03;

	LL_PWR_SetLSILPMUTrim(8);
#else
	hsi_calib = 0x1F;
	main_regulator = 0x0A;
	smps_out_voltage = 0x03;

	LL_RCC_LSI_SetTrimming(8);
#endif
	/* Set HSI Calibration Trimming value */
	LL_RCC_HSI_SetCalibTrimming(hsi_calib);

	/* Set Main Regulator voltage Trimming value */
	LL_PWR_SetMRTrim(main_regulator);

	/* Set SMPS output voltage Trimming value */
	LL_PWR_SetSMPSTrim(smps_out_voltage);
}

static void configure_smps(void)
{
	/* Save power by setting SMPS clock to 4MHz */
	LL_RCC_SetSMPSPrescaler(LL_RCC_SMPS_DIV_4);

#if !defined(STM32WB0_SMPS_ENABLED)
	/* Disable SMPS */
	LL_PWR_SetSMPSMode(LL_PWR_NO_SMPS);

	while (LL_PWR_IsSMPSReady()) {
		/* Wait for SMPS to turn off */
	}
#else
	/* Configure SMPS low-power mode */
	LL_PWR_SetSMPSOpenMode(SMPS_LOW_POWER_MODE);

	/* Configure SMPS BOM selection */
	LL_PWR_SetSMPSBOM(BOARD_SMPS_BOM);

	/* Enable SMPS */
	LL_PWR_SetSMPSMode(LL_PWR_SMPS);

	while (!LL_PWR_IsSMPSReady()) {
		/* Wait for SMPS to turn on */
	}

	/* Place SMPS in PRECHARGE (BYPASS) mode.
	 * This is required to change SMPS output voltage,
	 * so we can do it unconditionally.
	 */
	LL_PWR_SetSMPSPrechargeMode(LL_PWR_SMPS_PRECHARGE);
	while (LL_PWR_IsSMPSinRUNMode()) {
		/* Wait for SMPS to enter PRECHARGE mode */
	}

#	if defined(CONFIG_POWER_SUPPLY_SMPS_PRECHARE_SUPPLIES_LDO)
	/**
	 * SMPS should remain in PRECHARGE mode.
	 * However, the current limit should be configured.
	 */
	LL_PWR_SetSMPSPrechargeLimitCurrent(SMPS_CURRENT_LIMIT);
#	else
	/**
	 * SMPS should be in RUN mode.
	 * Configure the target output voltage then exit PRECHARGE.
	 */
	LL_PWR_SMPS_SetOutputVoltageLevel(SMPS_OUTPUT_VOLTAGE);

	/* Exit PRECHARGE mode (returns in RUN mode) */
	LL_PWR_SetSMPSPrechargeMode(LL_PWR_NO_SMPS_PRECHARGE);
	while (!LL_PWR_IsSMPSinRUNMode()) {
		/* Wait for SMPS to enter RUN mode */
	}
#	endif /* CONFIG_POWER_SUPPLY_SMPS_PRECHARE_SUPPLIES_LDO */
#endif /* !STM32WB0_SMPS_ENABLED */
}

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning,
 * so the init priority has to be 0 (zero).
 *
 * @return 0
 */
static int stm32wb0_init(void)
{
	/* Update CMSIS SystemCoreClock variable (CLK_SYS) */
	/* On reset, the 64MHz HSI is selected as input to
	 * the SYSCLKPRE prescaler, set to 4, resulting in
	 * CLK_SYS being equal to 16MHz.
	 */
	SystemCoreClock = 16000000;

	/* Remap address 0 to user flash memory */
	LL_SYSCFG_SetRemapMemory(LL_SYSCFG_REMAP_FLASH);

	/* Store in RAM the AppBase information */
	RAM_VR.AppBase = SCB->VTOR; /* TODO: should we use _vector_table instead? */

	/* Enable all the RAM banks in retention during DEEPSTOP */
	LL_PWR_EnableRAMBankRet(LL_PWR_RAMRET_1);
#if defined(LL_PWR_RAMRET_2)
	LL_PWR_EnableRAMBankRet(LL_PWR_RAMRET_2);
#endif
#if defined(LL_PWR_RAMRET_3)
	LL_PWR_EnableRAMBankRet(LL_PWR_RAMRET_3);
#endif

#if defined(PWR_CR2_GPIORET)
	/* Disable GPIO retention of DEEPSTOP configuration.
	 * This is required to be able to reconfigure GPIO pins
	 * after waking up from DEEPSTOP.
	 */
	LL_PWR_DisableGPIORET();
#endif

	/* Apply default trimming values if engineering flash is blank.
	 * Otherwise, engineering flash values have already been loaded by hardware.
	 */
	if (sys_read32(VALIDITY_LOCATION) != VALIDITY_TAG) {
		apply_default_trimming();
	}

	/* Configure SMPS step-down converter */
	configure_smps();

	return 0;
}

SYS_INIT(stm32wb0_init, PRE_KERNEL_1, 0);
