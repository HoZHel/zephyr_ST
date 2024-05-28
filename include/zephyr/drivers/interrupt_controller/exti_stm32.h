/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for External interrupt/event controller in STM32 MCUs
 *
 * Based on reference manuals:
 *   RM0008 Reference Manual: STM32F101xx, STM32F102xx, STM32F103xx, STM32F105xx
 *   and STM32F107xx advanced ARM(r)-based 32-bit MCUs
 * and
 *   RM0368 Reference manual STM32F401xB/C and STM32F401xD/E
 *   advanced ARM(r)-based 32-bit MCUs
 *
 * Chapter 10.2: External interrupt/event controller (EXTI)
 *
 */

#ifndef ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_INTC_EXTI_STM32_H_
#define ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_INTC_EXTI_STM32_H_

#include <zephyr/types.h>

#if defined(CONFIG_SOC_SERIES_STM32WB0)
#define STM32_EXTI_API_TYPE uint32_t
#else
/* TODO: this is a defect? */
#define STM32_EXTI_API_TYPE int
#endif

/**
 * @brief Dummy value indicating an invalid EXTI line
 *
 * @note This value is NOT accepted by the EXTI drivers.
 * It is simply defined for internal usage by other drivers,
 * for example as a fallback value for an optional DTS property.
 */
#define STM32_EXTI_LINE_NONE	0xFFFFFFFFU

/**
 * @brief enable EXTI interrupt for specific line
 *
 * @param line EXTI# line
 */
void stm32_exti_enable(STM32_EXTI_API_TYPE line);

/**
 * @brief disable EXTI interrupt for specific line
 *
 * @param line EXTI# line
 */
void stm32_exti_disable(STM32_EXTI_API_TYPE line);

/**
 * @brief EXTI trigger flags
 */
enum stm32_exti_trigger {
	/* clear trigger */
	STM32_EXTI_TRIG_NONE  = 0x0,
	/* trigger on rising edge */
	STM32_EXTI_TRIG_RISING  = 0x1,
	/* trigger on falling edge */
	STM32_EXTI_TRIG_FALLING = 0x2,
	/* trigger on both rising & falling edge */
	STM32_EXTI_TRIG_BOTH = 0x3,

	/* Values with high bit set cannot be OR'ed with any other */

	/* trigger on high level */
	STM32_GPIO_IRQ_TRIG_HIGH_LEVEL = 0x80,
	/* trigger on low level */
	STM32_GPIO_IRQ_TRIG_LOW_LEVEL = 0x81
};

/**
 * @brief set EXTI interrupt line triggers
 *
 * @param line EXTI# line
 * @param trg  OR'ed stm32_exti_trigger flags
 *
 * TODO: rename me!
 */
void stm32_exti_trigger(STM32_EXTI_API_TYPE line, STM32_EXTI_API_TYPE trg);

/* callback for exti interrupt */
typedef void (*stm32_exti_callback_t) (STM32_EXTI_API_TYPE line, void *user);

/**
 * @brief set EXTI interrupt callback
 *
 * @param line EXTI# line
 * @param cb   user callback
 * @param data user data
 */
int stm32_exti_set_callback(STM32_EXTI_API_TYPE line, stm32_exti_callback_t cb, void *data);

/**
 * @brief unset EXTI interrupt callback
 *
 * @param line EXTI# line
 */
void stm32_exti_unset_callback(STM32_EXTI_API_TYPE line);

/* EXTI driver extensions */

#if defined(CONFIG_SOC_SERIES_STM32WB0)

/**
 * @brief Obtains the interrupt line associated to a (port, pin) pair
 *
 * @param port	GPIO port
 * @param pin	Pin on GPIO port @p port
 *
 * @return 0			No interrupt line available for (port, pin)
 * @return Any other value	LL_EXT_LINE_xxx define corresponding to (port, pin)
 *
 * @note If stm32wb0_gpio_intr_is_line_supported() returns false, this returns 0.
 */
uint32_t stm32wb0_gpio_port_pin_to_intr_line(uint32_t port, uint32_t pin);

/**
 * @brief Checks if GPIO interrupts are available for one or more lines
 *
 * @param line	Bitmask containing one (or more) LL_EXTI_LINE_xxx define
 *		corresponding to the lines to check
 *
 * @return true if all lines in @p line support GPIO interrupts, false otherwise
 *
 * @note If this function returns "false" for a given value of "line", using
 * said value as the "line" parameter for any other function exposed by this
 * driver may result in undefined behavior and should not be done.
 */
bool stm32wb0_is_line_supported_for_gpio_intr(uint32_t line);

#endif /* defined(CONFIG_SOC_SERIES_STM32WB0) */

#endif /* ZEPHYR_DRIVERS_INTERRUPT_CONTROLLER_INTC_EXTI_STM32_H_ */
