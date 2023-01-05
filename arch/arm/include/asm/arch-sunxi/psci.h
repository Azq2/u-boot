// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2023 Kirill Zhumarin <kirill.zhumarin@gmail.com>
 */

#ifndef __SUNXI_ARM_PSCI_H__
#define __SUNXI_ARM_PSCI_H__

#include <asm/psci.h>

#ifndef __ASSEMBLY__
#include <linux/bitops.h>
#endif

/* SUNXI PSCI interface */
#define SUNXI_PSCI_FN_BASE			0x83000000
#define SUNXI_PSCI_FN(n)			(SUNXI_PSCI_FN_BASE + (n))

#define SUNXI_PSCI_DRAM_DVFS_REQ	SUNXI_PSCI_FN(0x33)

#ifndef __ASSEMBLY__
#include <asm/types.h>
#include <linux/bitops.h>
#endif /* ! __ASSEMBLY__ */

#endif /* __SUNXI_ARM_PSCI_H__ */
