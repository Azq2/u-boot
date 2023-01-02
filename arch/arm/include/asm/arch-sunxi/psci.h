/*
 * Copyright (C) 2013 - ARM Ltd
 * Author: Marc Zyngier <marc.zyngier@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __ARM_SUNXI_PSCI_H__
#define __ARM_SUNXI_PSCI_H__

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

#endif /* __ARM_SUNXI_PSCI_H__ */
