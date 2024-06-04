/*
 * Copyright (c) 2024 Arduino SA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_LLEXT_PRIV_H_
#define ZEPHYR_SUBSYS_LLEXT_PRIV_H_

#include <zephyr/kernel.h>
#include <zephyr/llext/llext.h>

/*
 * Memory management (llext_mem.c)
 */

int llext_copy_strings(struct llext_loader *ldr, struct llext *ext);
int llext_copy_sections(struct llext_loader *ldr, struct llext *ext);
void llext_free_sections(struct llext *ext);

static inline void *llext_alloc(size_t bytes)
{
	extern struct k_heap llext_heap;

	return k_heap_alloc(&llext_heap, bytes, K_NO_WAIT);
}

static inline void *llext_aligned_alloc(size_t align, size_t bytes)
{
	extern struct k_heap llext_heap;

	return k_heap_aligned_alloc(&llext_heap, align, bytes, K_NO_WAIT);
}

static inline void llext_free(void *ptr)
{
	extern struct k_heap llext_heap;

	k_heap_free(&llext_heap, ptr);
}

#endif /* ZEPHYR_SUBSYS_LLEXT_PRIV_H_ */
