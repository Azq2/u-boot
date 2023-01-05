// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2023 Kirill Zhumarin <kirill.zhumarin@gmail.com>
 */

#include <config.h>
#include <linux/linkage.h>
#include <asm/macro.h>
#include <asm/psci.h>
#include <asm/arch/psci.h>

	.pushsection ._secure.text, "ax"

	.arch_extension	sec

	.align	5

psci_platform_table:
	.word	SUNXI_PSCI_DRAM_DVFS_REQ
	.word	sunxi_dram_dvfs_req
	.word	0
	.word	0
.global psci_platform_table

	.popsection
