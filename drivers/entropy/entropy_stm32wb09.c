/**
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32wb09_rng

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#include <errno.h>

#include <soc.h>
#include <stm32_ll_rng.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(entropy_stm32wb0, CONFIG_ENTROPY_LOG_LEVEL);

/**
 * RM0505 §14.4 "TRNG functional description":
 *  To use the TRNG peripheral the system clock frequency must be
 *  at least 32 MHz. See also: §6.2.2 "Peripheral clock details".
 */
BUILD_ASSERT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC >= (32 * 1000 * 1000),
	"STM32WB0: system clock frequency must be at least 32MHz to use TRNG");


/**
 * Driver private structures
 */

/**
 * @brief Entropy pool metadata
 *
 * Data structure used to keep track of entropy pool contents.
 * The actually entropy bytes are stored somewhere else.
 */
struct entropy_pool_metadata {
	uint16_t available;
	uint8_t read_pointer;
	uint8_t write_pointer;

	const uint8_t modulo_mask;
	const uint8_t refill_threshold;
};

struct wb09_trng_driver_config {
	struct stm32_pclken clk;
};

struct wb09_trng_driver_data {
	RNG_TypeDef *const reg;
	struct entropy_pool_metadata isr_ep;
	struct entropy_pool_metadata thr_ep;

	/**
	 * This semaphore is used to keep track of RNG state.
	 * When RNG is enabled, the semaphore is 0.
	 * When RNG is disabled, the semaphore is 1.
	 */
	struct k_sem rng_enable_sem;

	/**
	 * This semaphore is signaled when new entropy bytes
	 * are avaiable in the thread entropy pool.
	 */
	struct k_sem thr_rng_avail_sem;

	/* declare pools at the end to reduce padding */
	uint8_t isr_pool[CONFIG_ENTROPY_STM32_ISR_POOL_SIZE];
	uint8_t thr_pool[CONFIG_ENTROPY_STM32_THR_POOL_SIZE];
};

/**
 * Driver private forward declarations
 */
static const struct wb09_trng_driver_config drv_config;
static struct wb09_trng_driver_data drv_data;

/**
 * @brief Obtain random data from entropy pool
 *
 * @param ep	Entropy pool metadata
 * @param pool	Entropy pool buffer
 * @param dst	Destination buffer
 * @param size	Number of random bytes requested in @p dst
 * @returns Number of random bytes written to @p dst
 */
static uint16_t entropy_pool_read(struct entropy_pool_metadata *ep,
			uint8_t *pool, uint8_t *dst, uint16_t size);

/**
 * @brief Add random data to entropy pool
 *
 * @param ep	Entropy pool metadata
 * @param pool	Entropy pool buffer
 * @param data	Randomly generated 32-bit value
 *
 * @returns 0 on success, -ENOBUFS if entropy pool is full
 */
static int entropy_pool_write(struct entropy_pool_metadata *ep,
			uint8_t *pool, uint32_t data);

/**
 * @brief Is entropy pool full?
 *
 * @param ep	Entropy pool metadata
 *
 * @returns true if pool is full, false otherwise
 */
static bool entropy_pool_is_full(struct entropy_pool_metadata *ep);

/**
 * Driver private definitions
 */
#define TRNG_FIFO_SIZE	4	/* in 32-bit words */
#define TRNG_IRQN	DT_INST_IRQN(0)

/**
 * Driver utility functions
 */
static inline void LL_RNG_ClearFlag_FF_FULL(RNG_TypeDef *RNGx)
{
	WRITE_REG(RNGx->IRQ_SR, RNG_IRQ_SR_FF_FULL_IRQ);
}

static inline void LL_RNG_ClearFlag_ERROR(RNG_TypeDef *RNGx)
{
	WRITE_REG(RNGx->IRQ_SR, RNG_IRQ_SR_ERROR_IRQ);
}

/**
 * Driver private functions
 */

/**
 * @brief Turn on the TRNG
 *
 * @note Must be called with irq_lock held.
 */
static void turn_on_trng(struct wb09_trng_driver_data *data)
{
	RNG_TypeDef *rng = data->reg;
	int res = k_sem_take(&data->rng_enable_sem, K_NO_WAIT);
	if (res < 0) {
		/* RNG already on - nothing to do. */
		return;
	}

	//TODO: rework?
	/* Acquire power management locks */
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	if (IS_ENABLED(CONFIG_PM_S2RAM)) {
		pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
	}

	/* Turn on RNG */
	LL_RNG_Enable(rng);
	while (LL_RNG_IsActiveFlag_DISABLED(rng)) {
		/* Wait for RNG to be enabled */
	}

	/* Erratum?! Need to reset AES (and restart health tests),
	 * else IP hangs on AES_DOUT_ERROR health test failure
	 */
	LL_RNG_SetAesReset(rng, 1);
}

/**
 * @brief Turn off the TRNG
 *
 * @note Must be called with irq_lock held
 */
static void turn_off_trng(struct wb09_trng_driver_data *data)
{
	RNG_TypeDef *rng = data->reg;

	/* Turn off RNG */
	LL_RNG_Disable(rng);
	while (!LL_RNG_IsActiveFlag_DISABLED(rng)) {
		/* Wait for RNG to be disabled */
	}

	/* Mark RNG as disabled in semaphore */
	k_sem_give(&data->rng_enable_sem);
}

/**
 * @returns true if driver entropy pools are full, false otherwise
 */
static bool driver_entropy_pools_are_full(struct wb09_trng_driver_data *data)
{
	return entropy_pool_is_full(&data->isr_ep) && entropy_pool_is_full(&data->thr_ep);
}

static void wb09_trng_isr(struct wb09_trng_driver_data *data)
{
	RNG_TypeDef *rng = data->reg;

	/* Interrupt cause: RNG FIFO is full */
	if (LL_RNG_GetFfFullIrq(rng)) {
		/* Write the random data from FIFO to entropy pools */
		bool wrote_to_thr_pool = false;
		for (unsigned i = 0; i < TRNG_FIFO_SIZE; i++) {
			uint32_t rnd = LL_RNG_GetRndVal(rng);
			int res = entropy_pool_write(&data->isr_ep, data->isr_pool, rnd);
			if (res >= 0) {
				continue;
			}

			/* ISR pool is full - try to fill thread pool instead */
			res = entropy_pool_write(&data->thr_ep, data->thr_pool, rnd);
			if (res >= 0) {
				wrote_to_thr_pool = true;
				continue;
			}

			/* Both pools are full - stop processing */
			break;
		}

		/* Clear interrupt flag */
		LL_RNG_ClearFlag_FF_FULL(rng);

		/* Signal "new data available" semaphore if needed */
		if (wrote_to_thr_pool) {
			k_sem_give(&data->thr_rng_avail_sem);
		}

		/* Stop TRNG if driver pools are full.
		 * This has to be done with interrupts suspended to
		 * prevent race conditions with higher priority ISRs.
		 */
		unsigned key = irq_lock();
		if (driver_entropy_pools_are_full(data)) {
			turn_off_trng(data);
		}
		irq_unlock(key);
	}

	/* Interrupt cause: TRNG health test error */
	if (LL_RNG_GetErrorIrq(rng)) {
		LOG_ERR("TRNG health test error occured");
		//TODO: dump registers?

		/* Restart the TRNG (must be done atomically) */
		unsigned key = irq_lock();
		turn_off_trng(data);
		LL_RNG_ClearFlag_ERROR(rng); //TODO: not needed?
		turn_on_trng(data);
		irq_unlock(key);
	}
}

/**
 * Driver subsystem API implementation
 */
static int wb09_trng_get_entropy(const struct device *dev, uint8_t *buffer, uint16_t length)
{
	struct wb09_trng_driver_data *data = dev->data;

	/* Reset "data available" semaphore */
	unsigned key = irq_lock();
	k_sem_reset(&data->thr_rng_avail_sem);
	irq_unlock(key);

	while (length > 0) {
		uint16_t read = entropy_pool_read(&data->thr_ep,
					data->thr_pool, buffer, length);
		buffer += read;
		length -= read;

		if (length > 0) {
			k_sem_take(&data->thr_rng_avail_sem, K_FOREVER);
		}
	}

	return 0;
}

static int wb09_trng_get_entropy_from_isr(const struct device *dev, uint8_t *buffer,
					uint16_t length, uint32_t flags)
{
	struct wb09_trng_driver_data *data = dev->data;
	RNG_TypeDef *rng = data->reg;
	int trng_irq_enabled;
	unsigned key;

	uint16_t pool_read = entropy_pool_read(&data->isr_ep, data->isr_pool, buffer, length);
	if (pool_read == length || !(flags & ENTROPY_BUSYWAIT)) {
		/* Either we fullfilled the request from ISR pool, or this is a non-blocking call.
		 * In both cases, we have nothing more to do, so return now.
		 */
		return pool_read;
	}

	/* Blocking call: read data from TRNG until buffer is filled
	 *
	 * To ensure re-entrancy, we must prevent the TRNG ISR from executing,
	 * by masking the TRNG interrupt at NVIC level. However, since this
	 * function can be re-entered, only the earliest call must unmask
	 * the interrupt once it completes.
	 */
	key = irq_lock();
		trng_irq_enabled = irq_is_enabled(TRNG_IRQN);
		irq_disable(TRNG_IRQN);
	irq_unlock(key);

	/* Take into account partial fill-up from ISR pool */
	buffer += pool_read;
	length -= pool_read;

	/* Make sure the TRNG is turned on */
	turn_on_trng(data);

	do {
		while (!LL_RNG_IsActiveFlag_VAL_READY(rng)) {
			/* Clear the "interrupt pending" bit in NVIC.
			 * This ensures the next interrupt from TRNG will raise the pending bit,
			 * which in turns sets the internal event register of the CPU thanks to
			 * SEVONPEND being enabled during arch initialization.
			 */
			NVIC_ClearPendingIRQ(TRNG_IRQN);

			/* Ensure all memory transactions complete */
			barrier_dsync_fence_full();

			/* Place Cortex-M core in low-power standby state */
			__WFE();

			/* Clear the event register (TODO: why?) */
			__SEV();
			__WFE();
		}

		/* Write value from TRNG to user buffer */
		uint32_t random_val = LL_RNG_GetRndVal(rng);
		size_t copy_length = MIN(length, sizeof(random_val));

		memcpy(buffer, &random_val, copy_length);

		buffer += copy_length;
		length -= copy_length;
	} while (length > 0);

	if (trng_irq_enabled) {
		/* Re-enable the TRNG interrupt if we disabled it */
		key = irq_lock();
			irq_enable(TRNG_IRQN);
		irq_unlock(key);
	}

	return length;
}

static const struct entropy_driver_api entropy_stm32wb09_api = {
	.get_entropy = wb09_trng_get_entropy,
	.get_entropy_isr = wb09_trng_get_entropy_from_isr
};

static int wb09_trng_init(const struct device *dev)
{
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	const struct wb09_trng_driver_config *config = dev->config;
	struct wb09_trng_driver_data *data = dev->data;
	RNG_TypeDef *rng = data->reg;
	int err;

	k_sem_init(&data->rng_enable_sem, 1, 1);
	k_sem_init(&data->thr_rng_avail_sem, 0, 1);

	if (!device_is_ready(clk)) {
		LOG_ERR("Clock control device not ready");
		return -ENODEV;
	}

	err = clock_control_on(clk, (clock_control_subsys_t)&config->clk);
	if (err < 0) {
		LOG_ERR("Failed to turn on TRNG clock");
		return err;
	}

	/* Turn on RNG IP to generate some entropy */
	turn_on_trng(data);

	/* Attach ISR and unmask RNG interrupt in NVIC */
	IRQ_CONNECT(TRNG_IRQN, DT_INST_IRQ(0, priority),
		wb09_trng_isr, &drv_data, 0);

	irq_enable(TRNG_IRQN);


	/* Enable RNG FIFO full and error interrupts */
	LL_RNG_EnableEnFfFullIrq(rng);
	LL_RNG_EnableEnErrorIrq(rng);

	return 0;
}

/**
 * Driver power management implementation
 */

//TODO

/**
 * Entropy pool implementation
 */

BUILD_ASSERT((CONFIG_ENTROPY_STM32_ISR_POOL_SIZE &
	      (CONFIG_ENTROPY_STM32_ISR_POOL_SIZE - 1)) == 0,
	     "The CONFIG_ENTROPY_STM32_ISR_POOL_SIZE must be a power of 2!");

BUILD_ASSERT((CONFIG_ENTROPY_STM32_THR_POOL_SIZE &
	      (CONFIG_ENTROPY_STM32_THR_POOL_SIZE - 1)) == 0,
	     "The CONFIG_ENTROPY_STM32_THR_POOL_SIZE must be a power of 2!");

#if CONFIG_ENTROPY_STM32_ISR_POOL_SIZE != 256 || CONFIG_ENTROPY_STM32_THR_POOL_SIZE != 256
#	define EP_WRAPAROUND(ep, ptr) ((uint8_t)((ptr) & (ep->modulo_mask)))
#else
/* both pools have size = 256, we get modulo for free due to uint8_t */
#	define EP_WRAPAROUND(ep, ptr)  ((uint8_t)(ptr))
#endif

#define EP_CAPACITY(ep) ((uint16_t)ep->modulo_mask + 1U)

static uint16_t entropy_pool_read(struct entropy_pool_metadata *ep,
			uint8_t *pool, uint8_t *dst, uint16_t size)
{
	unsigned key = irq_lock();
	const uint16_t read_length = MIN(ep->available, size);
	size_t copy_size = read_length;

	if ( EP_WRAPAROUND(ep, ep->read_pointer + read_length) < ep->read_pointer ) {
		/* two-pass copy required because we wrap around */
		size_t first_pass_size = ((size_t)ep->modulo_mask - ep->read_pointer + 1);
		memcpy(dst, &pool[ep->read_pointer], first_pass_size);

		//TODO: remove me
		__ASSERT_NO_MSG(EP_WRAPAROUND(ep, (size_t)ep->read_pointer + first_pass_size) == 0);

		ep->read_pointer = 0;
		dst += first_pass_size;
		copy_size -= first_pass_size;
	}

	memcpy(dst, &pool[ep->read_pointer], copy_size);

	/* despite copy being contiguous, we need to wrap around
	 * in case we copied all bytes up to the end of the buffer
	 */
	ep->read_pointer = EP_WRAPAROUND(ep, (size_t)ep->read_pointer + copy_size);

	/* update availability count */
	ep->available -= read_length;

	/* if we are below threshold, start RNG to fill pools */
	if (ep->available <= (uint16_t)ep->refill_threshold) {
		turn_on_trng(&drv_data);
	}

	irq_unlock(key);
	return read_length;
}

static int entropy_pool_write(struct entropy_pool_metadata *ep,
				uint8_t *pool, uint32_t data)
{
	unsigned key = irq_lock();
	uint16_t remaining_empty = EP_CAPACITY(ep) - ep->available;
	uint16_t added = MIN(sizeof(data), remaining_empty);
	if (remaining_empty == 0) {
		return -ENOBUFS;
	}

	for (uint16_t i = 0; i < added; i++) {
		/* write one byte of random data */
		pool[ep->write_pointer] = (uint8_t)(data & 0xFF);

		/* increment write pointer */
		ep->write_pointer = EP_WRAPAROUND(ep, ep->write_pointer + 1);

		/* prepare another byte of random data for next write  */
		data >>= 8;
	}

	/* increment pool population */
	ep->available += added;

	irq_unlock(key);
	return 0;
}

static bool entropy_pool_is_full(struct entropy_pool_metadata *ep)
{
	unsigned key = irq_lock();
	uint16_t remaining_empty = EP_CAPACITY(ep) - ep->available;

	irq_unlock(key);
	return remaining_empty == 0;
}

#define ENTROPY_POOL_INITIALIZER(size, threshold) {	\
		.available = 0,				\
		.read_pointer = 0,			\
		.write_pointer = 0,			\
		.modulo_mask = (size - 1),		\
		.refill_threshold = threshold}

#undef EP_WRAPAROUND


/**
 * Driver device instantiation
 */

PM_DEVICE_DT_INST_DEFINE(0, entropy_stm32_rng_pm_action);

static const struct wb09_trng_driver_config drv_config = {
	.clk = STM32_CLOCK_INFO(0, DT_DRV_INST(0))
};

static struct wb09_trng_driver_data drv_data = {
	.reg = (RNG_TypeDef*)DT_INST_REG_ADDR(0),
	.isr_ep = ENTROPY_POOL_INITIALIZER(
		CONFIG_ENTROPY_STM32_ISR_POOL_SIZE,
		CONFIG_ENTROPY_STM32_ISR_THRESHOLD),
	.thr_ep = ENTROPY_POOL_INITIALIZER(
		CONFIG_ENTROPY_STM32_THR_POOL_SIZE,
		CONFIG_ENTROPY_STM32_THR_THRESHOLD),
};

DEVICE_DT_INST_DEFINE(0,
		    wb09_trng_init,
		    PM_DEVICE_DT_INST_GET(0),
		    &drv_data, &drv_config, PRE_KERNEL_1,
		    CONFIG_ENTROPY_INIT_PRIORITY,
		    &entropy_stm32wb09_api);
