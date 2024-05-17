/*
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

SECTION_PROLOGUE(stm32wb0_RAM_VR, 0x20000000 (NOLOAD), )
{
	/* For historical reasons, leave the first word of
	 * SRAM0 unused, even though it could store data.
	 */
	. += 4;

	KEEP(*(stm32wb0_RAM_VR));
} GROUP_LINK_IN(RAMABLE_REGION)

SECTION_PROLOGUE(BLUE_RAM_section, 0x200000C0 (NOLOAD), )
{
	KEEP(*(stm32wb0_BLUE_RAM));
} GROUP_LINK_IN(RAMABLE_REGION)
