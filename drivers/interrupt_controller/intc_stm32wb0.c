/*
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for STM32WB0 GPIO interrupt controller
 *
 * In this file, "EXTI" should be understood as "GPIO interrupt controller".
 * There is no "External interrupt/event controller (EXTI)" in STM32WB0 MCUs.
 */

#define EXTI_NODE DT_INST(0, st_stm32_exti)

#include <errno.h>

#include <zephyr/device.h>
#include <soc.h>
#include <stm32_ll_system.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/interrupt_controller/exti_stm32.h>
#include <zephyr/dt-bindings/pinctrl/stm32-pinctrl-common.h>	/* For PORTA/PORTB defines */
#include <zephyr/irq.h>

/* TODO: this is not _too_ bad... but could we do better? */
static const uint32_t SUPPORTED_PORT_A_LINES =
#if defined(CONFIG_SOC_STM32WB06XX) || defined(CONFIG_SOC_STM32WB07XX)
	LL_EXTI_LINE_PA4 | LL_EXTI_LINE_PA5 | LL_EXTI_LINE_PA6 | LL_EXTI_LINE_PA7 |
	LL_EXTI_LINE_PA12 | LL_EXTI_LINE_PA13 | LL_EXTI_LINE_PA14 | LL_EXTI_LINE_PA15 |
#endif
	LL_EXTI_LINE_PA0 | LL_EXTI_LINE_PA1 | LL_EXTI_LINE_PA2 | LL_EXTI_LINE_PA3 |
	LL_EXTI_LINE_PA8 | LL_EXTI_LINE_PA9 | LL_EXTI_LINE_PA10 | LL_EXTI_LINE_PA11;

static const uint32_t SUPPORTED_PORT_B_LINES =
#if defined(CONFIG_SOC_STM32WB06XX) || defined(CONFIG_SOC_STM32WB07XX)
	LL_EXTI_LINE_PB8 | LL_EXTI_LINE_PB9 | LL_EXTI_LINE_PB10 | LL_EXTI_LINE_PB11 |
#endif
	LL_EXTI_LINE_PB0 | LL_EXTI_LINE_PB1 | LL_EXTI_LINE_PB2 | LL_EXTI_LINE_PB3 |
	LL_EXTI_LINE_PB4 | LL_EXTI_LINE_PB5 | LL_EXTI_LINE_PB6 | LL_EXTI_LINE_PB7 |
	LL_EXTI_LINE_PB12 | LL_EXTI_LINE_PB13 | LL_EXTI_LINE_PB14 | LL_EXTI_LINE_PB15;

#define NUM_GPIO_INTERRUPTS	DT_NUM_IRQS(EXTI_NODE)


#define NUM_GPIOS_ON_PORT(p) \
	DT_PROP_BY_IDX(EXTI_NODE, line_ranges, UTIL_INC(UTIL_X2(p)))

#define IRQN_OF_GPIO_PORT(p) \
	DT_IRQ_BY_IDX(EXTI_NODE, p, irq)

/* wrapper for user callback */
struct __user_cb {
	stm32_exti_callback_t user_fn;
	void *user_data;
};

/* wrapper for ISR argument block */
struct __z_isr_argblock {
	/* LL define for first line on GPIO port
	 * (must be least significant bit of the port's defines)
	 */
	uint32_t port_first_line;
	/* Pointer to first element of user_callbacks_table
	 * array that corresponds to this GPIO line
	 */
	struct __user_cb *cb_table;
};

/* driver data */
struct stm32wb0_gpio_intc_data {
	/* per-port user callbacks */
	struct __user_cb user_callbacks_table[
		NUM_GPIOS_ON_PORT(STM32_PORTA) +
		NUM_GPIOS_ON_PORT(STM32_PORTB)
	];
};

/**
 * @brief Obtain the IRQ number for a given line
 *
 * @param line		LL_EXTI_LINE_xxx define corresponding to GPIO pin
 *
 * @return 0xFF is GPIO interrupt not supported, an IRQn_Type otherwise
 */
static IRQn_Type stm32wb0_irqn_for_line(uint32_t line) {
	if ((line & SUPPORTED_PORT_A_LINES) != 0) {
		return IRQN_OF_GPIO_PORT(STM32_PORTA);
	} else if ((line & SUPPORTED_PORT_B_LINES) != 0) {
		return IRQN_OF_GPIO_PORT(STM32_PORTB);
	} else {
		return 0xFF;
	}
}

/**
 * @brief Retrieves the user callback block for a given line
 */
static struct __user_cb *stm32wb0_user_cb_block_for_line(uint32_t line) {
	const struct device *const dev = DEVICE_DT_GET(EXTI_NODE);
	struct stm32wb0_gpio_intc_data *data = dev->data;

	/* See definition of LL_EXTI_LINE_Pxx to understand this. */
	uint32_t line_cb_idx = LOG2(line);

	return data->user_callbacks_table + line_cb_idx;
}

uint32_t stm32wb0_gpio_port_pin_to_intr_line(uint32_t port, uint32_t pin) {
	uint32_t line = (1 << pin);

	if (port == STM32_PORTA) {
		line <<= SYSCFG_IO_DTR_PA0_DT_Pos;
	} else if (port == STM32_PORTB) {
		line <<= SYSCFG_IO_DTR_PB0_DT_Pos;
	} else {
		CODE_UNREACHABLE;
	}

	if (!stm32wb0_is_line_supported_for_gpio_intr(line)) {
		return 0;
	} else {
		return line;
	}
}

bool stm32wb0_is_line_supported_for_gpio_intr(uint32_t line) {
	const uint32_t supported_lines =
		(SUPPORTED_PORT_A_LINES | SUPPORTED_PORT_B_LINES);

	return ((line & supported_lines) == line);
}

void stm32_exti_enable(uint32_t line)
{
	uint32_t irqn = stm32wb0_irqn_for_line(line);

	/* Check that the line supports interrupts */
	if (irqn == 0xFF) {
		__ASSERT_NO_MSG(0);
	}

	/* Enable line interrupt at INTC level */
	LL_EXTI_EnableIT(line);

	/* Enable INTC interrupt in NVIC */
	irq_enable(irqn);
}

void stm32_exti_disable(uint32_t line)
{
	__ASSERT_NO_MSG(stm32wb0_is_line_supported_for_gpio_intr(line));

	LL_EXTI_DisableIT(line);
}

void stm32_exti_trigger(uint32_t line, uint32_t trigger)
{
	__ASSERT_NO_MSG(stm32wb0_is_line_supported_for_gpio_intr(line));

	switch (trigger) {
	case STM32_EXTI_TRIG_NONE:
		/* TODO: On STM32WB0, not possible to disable triggers?
		 * Is doing nothing good enough??
		 */
		break;
	case STM32_EXTI_TRIG_RISING:
        	LL_EXTI_EnableEdgeDetection(line);
		LL_EXTI_DisableBothEdgeTrig(line);
        	LL_EXTI_EnableRisingTrig(line);
		break;
	case STM32_EXTI_TRIG_FALLING:
        	LL_EXTI_EnableEdgeDetection(line);
		LL_EXTI_DisableBothEdgeTrig(line);
        	LL_EXTI_DisableRisingTrig(line);
		break;
	case STM32_EXTI_TRIG_BOTH:
        	LL_EXTI_EnableEdgeDetection(line);
        	LL_EXTI_EnableBothEdgeTrig(line);
		break;
	case STM32_GPIO_IRQ_TRIG_HIGH_LEVEL:
        	LL_EXTI_DisableEdgeDetection(line);
        	LL_EXTI_EnableRisingTrig(line);
		break;
	case STM32_GPIO_IRQ_TRIG_LOW_LEVEL:
        	LL_EXTI_DisableEdgeDetection(line);
        	LL_EXTI_DisableRisingTrig(line);
		break;
	default:
		__ASSERT_NO_MSG(0);
		break;
	}

	/**
	 * HACK: on STM32WB0, it is NOT possible to disable EXTI,
	 * unlike on other STM32 MCUs. The consequence is that,
	 * when we switch the trigger type, it is possible that
	 * a "triggered" event is still pending from the previous
	 * configuration, which would generate a spurious interrupt.
	 *
	 * Clear pending flag of this line to - hopefully - prevent this.
	 */
	LL_EXTI_ClearFlag(line);
}

int stm32_exti_set_callback(uint32_t line, stm32_exti_callback_t cb, void *userdata)
{
	struct __user_cb *cb_blk = stm32wb0_user_cb_block_for_line(line);

	if ((cb_blk->user_fn == cb) && (cb_blk->user_data == userdata)) {
		return 0;
	}

	/* If line already has a callback, return EBUSY */
	if (cb_blk->user_fn != NULL) {
		return -EBUSY;
	}

	cb_blk->user_fn = cb;
	cb_blk->user_data = userdata;

	return 0;
}

void stm32_exti_unset_callback(uint32_t line)
{
	struct __user_cb *cb_blk = stm32wb0_user_cb_block_for_line(line);

	cb_blk->user_fn = cb_blk->user_data = NULL;
}

/* Interrupt subroutines */
static void stm32wb0_gpio_isr(const void *userdata) {
	const struct __z_isr_argblock *arg = userdata;
	const struct __user_cb *cb_table = arg->cb_table;
	const uint32_t max_lines_per_port =
		MAX(NUM_GPIOS_ON_PORT(STM32_PORTA), NUM_GPIOS_ON_PORT(STM32_PORTB));

	/* TODO: seeing this... is it really worth it? */
	BUILD_ASSERT(max_lines_per_port == 16);

	uint32_t line = arg->port_first_line;

	for (uint32_t i = 0; i < max_lines_per_port; i++, line <<= 1) {
		if (LL_EXTI_IsActiveFlag(line) != 0) {
			/* clear pending interrupt */
			LL_EXTI_ClearFlag(line);

			/* execute user callback if registered */
			if (cb_table[i].user_fn != NULL) {
				cb_table[i].user_fn(line, cb_table[i].user_data);
			}
		}
	}
}

/* define driver data early for usage by the following macros */
static struct stm32wb0_gpio_intc_data gpio_intc_data;

/**
 * This macro creates the argument block for the ISR corresponding
 * to the 'port' GPIO port, and calls IRQ_CONNECT for each interrupt.
 *
 * @param node	EXTI Device Tree node
 * @param pidx	GPIO port index
 * @param plin	GPIO port line (LL define)
 */
#define STM32WB0_INIT_INTC_FOR_GPIO_PORT_INNER(node, pidx, plin) \
	static const struct __z_isr_argblock port ##pidx ##_argblock =		\
		{ plin, gpio_intc_data.user_callbacks_table +			\
			DT_PROP_BY_IDX(node, line_ranges, UTIL_X2(pidx)) };	\
										\
	IRQ_CONNECT(DT_IRQ_BY_IDX(node, pidx, irq),				\
		DT_IRQ_BY_IDX(node, pidx, priority),				\
		stm32wb0_gpio_isr, &port ##pidx ##_argblock, 0)

#define STM32WB0_INIT_INTC_FOR_GPIO_PORT(_port)					\
	STM32WB0_INIT_INTC_FOR_GPIO_PORT_INNER(EXTI_NODE,			\
		STM32_PORT ##_port, LL_EXTI_LINE_P ##_port ## 0)

/**
 * @brief initialize GPIO interrupt controller device driver
 */
static int stm32wb0_gpio_intc_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	STM32WB0_INIT_INTC_FOR_GPIO_PORT(A);

	STM32WB0_INIT_INTC_FOR_GPIO_PORT(B);

	return 0;
}

DEVICE_DT_DEFINE(EXTI_NODE, &stm32wb0_gpio_intc_init,
		 NULL,
		 &gpio_intc_data, NULL,
		 PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY,
		 NULL);
