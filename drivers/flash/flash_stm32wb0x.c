/*
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_flash_controller

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/flash/stm32_flash_api_extensions.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/math_extras.h>

/* <soc.h> also brings "stm32wb0x_hal_flash.h",
 * which provides some macros used in this file
 */
#include <soc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_system.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_stm32wb0x, CONFIG_FLASH_LOG_LEVEL);



/**
 * Driver private definitions & assertions
 */
#if defined(CONFIG_SOC_STM32WB09XX)
#	define PAGES_IN_FLASH 256
#elif defined(CONFIG_SOC_STM32WB07XX) \
	|| defined(CONFIG_SOC_STM32WB06XX)
#	define PAGES_IN_FLASH 128
#elif defined(CONFIG_SOC_STM32WB05XX)
#	define PAGES_IN_FLASH 96
#endif

#define FLASH_PTR(off)  ((void *)(DT_REG_ADDR(DT_INST(0, st_stm32_nv_flash)) + ((uint32_t)off)))

#define WRITE_BLOCK_SIZE \
	DT_PROP(DT_INST(0, soc_nv_flash), write_block_size)

#define WORD_SIZE	WRITE_BLOCK_SIZE

#define ERASE_BLOCK_SIZE \
	DT_PROP(DT_INST(0, soc_nv_flash), erase_block_size)

/**
 * @brief Mask of interrupts handled by the driver.
 * All interrupts except READOK are currently handled.
 */
#define DRIVER_IRQ_MASK	(FLASH_IT_CMDDONE | FLASH_IT_CMDSTART \
			| FLASH_IT_CMDERR | FLASH_IT_ILLCMD)

/**
 * Driver private structures
 */
struct flash_wb0x_data {
	/** Used to serialize write/erase operations */
	struct k_sem write_lock;

	/** Signaled by the ISR when an interrupt occurs */
	struct k_sem irq_sema;

	/** Content of IRQSTAT register fetched by ISR */
	volatile uint32_t irq_flags;

	/** Flash size, in bytes */
	size_t flash_size;
};

/**
 * Driver private utility functions
 */
static size_t ll_flash_get_size_in_bytes(void)
{
	/* FLASH.SIZE contains the highest flash address supported
	 * on this MCU, which is also the number of words in flash
	 * minus one.
	 */
	const uint32_t words_in_flash =
		READ_BIT(FLASH->SIZE, FLASH_FLASH_SIZE_FLASH_SIZE) + 1;

	return words_in_flash * WORD_SIZE;
}

static uint32_t fetch_u32(const uint32_t *ptr)
{
	/**
	 * Fetch word using sys_get_le32, which performs byte-sized
	 * reads instead of word-sized. This is important as ptr may
	 * be unaligned. We also want to use le32 because the data is
	 * stored in little-endian inside the flash.
	 */
	return sys_get_le32((const uint8_t *)ptr);
}

/**
 * @brief Returns the associated error to IRQ flags.
 *
 * @returns a negative error value
 */
static int error_from_irq_flags(uint32_t flags)
{
	/**
	 * Only two errors are expected:
	 *  - illegal command
	 *  - command error
	 */
	if (flags & FLASH_FLAG_ILLCMD) {
		return -EINVAL;
	}

	if (flags & FLASH_FLAG_CMDERR) {
		return -EIO;
	}

	/* "no entry" -> couldn't find a corresponding error */
	return -ENOENT;
}

static bool is_valid_flash_range(const struct device *dev,
				off_t offset, uint32_t len)
{
	const struct flash_wb0x_data *data = dev->data;
	uint32_t offset_plus_len;

		/* (offset + len) must not overflow */
	return !u32_add_overflow(offset, len, &offset_plus_len)
		/* offset must be a valid offset in flash */
		&& IN_RANGE(offset, 0, data->flash_size - 1)
		/* (offset + len) must be in [0; flash size]
		 * because it is equal to the last accessed
		 * byte in flash plus one (an access of `len`
		 * bytes starting at `offset` touches bytes
		 * `offset` to `offset + len` EXCLUDED)
		 */
		&& IN_RANGE(end, 0, data->flash_size);
}

static bool is_writeable_flash_range(const struct device *dev,
				off_t offset, uint32_t len)
{
	if ((offset % WRITE_BLOCK_SIZE) != 0
		|| (len % WRITE_BLOCK_SIZE) != 0) {
		return false;
	}

	return is_valid_flash_range(dev, offset, len);
}

static bool is_erasable_flash_range(const struct device *dev,
				off_t offset, uint32_t len)
{
	if ((offset % ERASE_BLOCK_SIZE) != 0
		|| (len % ERASE_BLOCK_SIZE) != 0) {
		return false;
	}

	return is_valid_flash_range(dev, offset, len);
}

/**
 * Driver private functions
 */

static uint32_t wait_flash_controller_event(struct flash_wb0x_data *data)
{
	uint32_t flags;

	if (k_is_in_isr()) {
		/** In ISR context, poll flash registers */
		do {
			flags = FLASH->IRQRAW;
		} while (flags == 0);

		/** Acknowledge the interrupts we're about to return */
		FLASH->IRQRAW = flags;
	} else {
		/** Synchronize with ISR and return what it read */
		k_sem_take(&data->irq_sema, K_FOREVER);
		flags = data->irq_flags;
	}

	return flags;
}

static int execute_flash_command(struct flash_wb0x_data *data, uint8_t cmd)
{
	uint32_t irq_flags;
	int res = 0; /* Assume success by default */

	/* Clear all pending interrupt bits */
	FLASH->IRQRAW = FLASH->IRQMASK;

	/* Reset IRQ event semaphore & status flags */
	k_sem_reset(&data->irq_sema);
	data->irq_flags = 0;

	if (k_is_in_isr()) {
		/* Mask all interrupts (polling mode) */
		LL_FLASH_DisableIT(FLASH, DRIVER_IRQ_MASK);
	}

	/* Start command */
	FLASH->COMMAND = cmd;

	/* Wait for CMDSTART */
	irq_flags = wait_flash_controller_event(data);

	if (!(irq_flags & FLASH_IT_CMDSTART)) {
		res = error_from_irq_flags(irq_flags);
		goto out;
	}

	/* Wait for CMDDONE if it has not occurred */
	if (!(irq_flags & FLASH_IT_CMDDONE)) {
		irq_flags = wait_flash_controller_event(data);
		if (!(irq_flags & FLASH_IT_CMDDONE)) {
			res = error_from_irq_flags(irq_flags);
			goto out;
		}
	}

out:
	if (k_is_in_isr()) {
		/* Unmask all interrupts if previously disabled */
		LL_FLASH_EnableIT(FLASH, DRIVER_IRQ_MASK);
	}
	return res;
}

int erase_page_range(struct flash_wb0x_data *data,
		uint32_t start_page, uint32_t page_count)
{
	int res = 0;

	__ASSERT_NO_MSG(start_page < PAGES_IN_FLASH);
	__ASSERT_NO_MSG((start_page + page_count - 1) < PAGES_IN_FLASH);

	for (uint32_t i = start_page;
		i < (start_page + page_count);
		i++) {
		/* ADDRESS[16:9] = XADR[10:3] = page address to erase
		 * ADDRESS[8:0]  = 0 (row / word address = 0)
		 */
		FLASH->ADDRESS = (i << 9);

		res = execute_flash_command(data, FLASH_CMD_ERASE_PAGES);
		if (res < 0) {
			break;
		}
	}

	return res;
}

int write_word_range(struct flash_wb0x_data *data, uint32_t start_word,
			uint32_t num_words, const void *buf)
{
	const size_t WORDS_IN_BURST = 4;
	uint32_t remaining = num_words;
	uint32_t write_addr = start_word;
	/**
	 * Note that @p buf may not be aligned to 32-bit boundary.
	 * However, declaring src as uint32_t* makes the address
	 * increment by 4 every time we do src++, which makes it
	 * behave like the other counters in this function.
	 */
	const uint32_t *src = buf;
	int res = 0;

	/* TODO: take advantage of the BURSTWRITE command.
	 * However, the last attempt at using it was a disaster...
	 */
	while (remaining > 0) {
		FLASH->ADDRESS = write_addr;
		FLASH->DATA0 = fetch_u32(src);

		res = execute_flash_command(data, FLASH_CMD_WRITE);
		if (res < 0) {
			return res;
		}

		src++;
		write_addr++;
		remaining--;
	}

	__ASSERT_NO_MSG(remaining == 0);

	return res;
}

void flash_wb0x_isr(const struct device *dev)
{
	struct flash_wb0x_data *data = dev->data;

	/** Read interrupt flags */
	uint32_t flags = FLASH->IRQSTAT;

	/** Clear interrupt flags */
	FLASH->IRQSTAT = flags;

	/**
	 * Append the interrupt flags to driver data.
	 * This MUST be a bitwise OR and not
	 * Note that this we MUST bitwise OR rather than set
	 * the variable to ensure the waiter thread gets ALL
	 * flags, regardless of when it gets scheduled.
	 * Otherwise, it could miss CMDSTART if CMDDONE occurs
	 * too soon after it.
	 */
	data->irq_flags |= flags;

	/** Signal the IRQ event semaphore */
	k_sem_give(&data->irq_sema);
}

/**
 * Driver subsystem API implementation
 */
int flash_wb0x_read(const struct device *dev, off_t offset,
			void *buffer, size_t len)
{
	if (!len) {
		return 0;
	}

	if (!is_valid_flash_range(dev, offset, len)) {
		return -EINVAL;
	}

	memcpy(buffer, FLASH_PTR(offset), len);

	return 0;
}

int flash_wb0x_write(const struct device *dev, off_t offset,
			const void *buffer, size_t len)
{
	struct flash_wb0x_data *data = dev->data;
	int res;

	if (!len) {
		return 0;
	}

	if (!is_writeable_flash_range(dev, offset, len)) {
		return -EINVAL;
	}

	/* Acquire driver lock */
	res = k_sem_take(&data->write_lock, K_NO_WAIT);
	if (res < 0) {
		return res;
	}

	res = write_word_range(data, ((uint32_t)offset / WORD_SIZE),
		(len / WORD_SIZE), buffer);

	/* Release driver lock */
	k_sem_give(&data->write_lock);

	return res;
}

int flash_wb0x_erase(const struct device *dev, off_t offset, size_t size)
{
	struct flash_wb0x_data *data = dev->data;
	int res;

	if (!size) {
		return 0;
	}

	if (!is_erasable_flash_range(dev, offset, size)) {
		return -EINVAL;
	}

	/* Acquire driver lock */
	res = k_sem_take(&data->write_lock, K_NO_WAIT);
	if (res < 0) {
		return res;
	}

	uint32_t start_page = ((uint32_t)offset / ERASE_BLOCK_SIZE);
	uint32_t page_count = (size / ERASE_BLOCK_SIZE);

	res = erase_page_range(data, start_page, page_count);

	/* Release driver lock */
	k_sem_give(&data->write_lock);

	return res;
}

const struct flash_parameters *flash_wb0x_get_parameters(
					const struct device *dev)
{
	static const struct flash_parameters fp = {
		.write_block_size = WRITE_BLOCK_SIZE,
		.erase_value = 0xff,
	};

	return &fp;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
void flash_wb0x_pages_layout(const struct device *dev,
				const struct flash_pages_layout **layout,
				size_t *layout_size)
{
	/**
	 * STM32WB0 flash: single bank, 2KiB pages
	 * (the number of pages depends on MCU)
	 */
	static const struct flash_pages_layout fpl[] = {{
		.pages_count = PAGES_IN_FLASH,
		.pages_size = FLASH_PAGE_SIZE
	}};

	*layout = fpl;
	*layout_size = ARRAY_SIZE(fpl);
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

#if defined(CONFIG_FLASH_EX_OP_ENABLED)
int flash_wb0x_ex_op(const struct device *dev, uint16_t code,
			const uintptr_t in, void *out)
{
	/* Extended operations are not supported */
	return -ENOTSUP;
}
#endif /* CONFIG_FLASH_EX_OP_ENABLED */

static const struct flash_driver_api flash_wb0x_api = {
	.erase = flash_wb0x_erase,
	.write = flash_wb0x_write,
	.read = flash_wb0x_read,
	.get_parameters = flash_wb0x_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_wb0x_pages_layout,
#endif
#ifdef CONFIG_FLASH_EX_OP_ENABLED
	.ex_op = flash_wb0x_ex_op,
#endif
};

int stm32wb0x_flash_init(const struct device *dev)
{
	struct flash_wb0x_data *data = dev->data;

	k_sem_init(&data->write_lock, 1, 1);
	k_sem_init(&data->irq_sema, 0, 1);

	data->flash_size = ll_flash_get_size_in_bytes();

	/**
	 * Unmask the following interrupts:
	 *  - Command done
	 *  - Command start
	 *  - Command error
	 *  - Illegal command
	 * (READOK remains masked)
	 */
	LL_FLASH_EnableIT(FLASH, DRIVER_IRQ_MASK);

	/* Attach ISR and enable interrupt at NVIC level */
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority),
		flash_wb0x_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	return 0;
}

/**
 * Driver device instantiation
 */
static struct flash_wb0x_data wb0x_flash_drv_data;

DEVICE_DT_INST_DEFINE(0, stm32wb0x_flash_init, NULL,
		    &wb0x_flash_drv_data, NULL, POST_KERNEL,
		    CONFIG_FLASH_INIT_PRIORITY, &flash_wb0x_api);
