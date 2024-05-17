/*
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file SoC configuration macros for the STM32WB0 family processors.
 *
 */


#ifndef _STM32WB0_SOC_H_
#define _STM32WB0_SOC_H_

#ifndef _ASMLANGUAGE

#if defined(CONFIG_POWER_SUPPLY_SMPS_SUPPLIES_LDO) || \
	defined(CONFIG_POWER_SUPPLY_SMPS_PRECHARE_SUPPLIES_LDO)
#define STM32WB0_SMPS_ENABLED	1
#endif /* CONFIG_POWER_SUPPLY_* */

#include <stm32wb0x.h>

#endif /* !_ASMLANGUAGE */

#endif /* _STM32N6_SOC_H_ */
