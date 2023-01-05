// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016
 * Author: Chen-Yu Tsai <wens@csie.org>
 *
 * Based on assembly code by Marc Zyngier <marc.zyngier@arm.com>,
 * which was based on code by Carl van Schaik <carl@ok-labs.com>.
 */
#include <config.h>
#include <common.h>
#include <asm/cache.h>

#include <asm/arch/cpu.h>
#include <asm/arch/cpucfg.h>
#include <asm/arch/prcm.h>
#include <asm/arch/clock.h>
#include <asm/arch/dram.h>
#include <asm/armv7.h>
#include <asm/gic.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/psci.h>
#include <asm/secure.h>
#include <asm/system.h>

#include <linux/bitops.h>

#define USE_LOCKING_BEFORE_DDRFRQ	0

#define __irq		__attribute__ ((interrupt ("IRQ")))

#define	GICD_BASE	(SUNXI_GIC400_BASE + GIC_DIST_OFFSET)
#define	GICC_BASE	(SUNXI_GIC400_BASE + GIC_CPU_OFFSET_A15)

/*
 * R40 is different from other single cluster SoCs.
 *
 * The power clamps are located in the unused space after the per-core
 * reset controls for core 3. The secondary core entry address register
 * is in the SRAM controller address range.
 */
#define SUN8I_R40_PWROFF			(0x110)
#define SUN8I_R40_PWR_CLAMP(cpu)		(0x120 + (cpu) * 0x4)
#define SUN8I_R40_SRAMC_SOFT_ENTRY_REG0		(0xbc)

static volatile bool cpu_lock_state[4] __secure_data = { 0 };

static void __secure cp15_write_cntp_tval(u32 tval)
{
	asm volatile ("mcr p15, 0, %0, c14, c2, 0" : : "r" (tval));
}

static void __secure cp15_write_cntp_ctl(u32 val)
{
	asm volatile ("mcr p15, 0, %0, c14, c2, 1" : : "r" (val));
}

static u32 __secure cp15_read_cntp_ctl(void)
{
	u32 val;

	asm volatile ("mrc p15, 0, %0, c14, c2, 1" : "=r" (val));

	return val;
}

#define ONE_MS (CONFIG_COUNTER_FREQUENCY / 1000)
#define ONE_US (CONFIG_COUNTER_FREQUENCY / 1000000)

static void __secure __udelay(u32 us)
{
	u32 reg = ONE_US * us;

	cp15_write_cntp_tval(reg);
	isb();
	cp15_write_cntp_ctl(3);

	do {
		isb();
		reg = cp15_read_cntp_ctl();
	} while (!(reg & BIT(2)));

	cp15_write_cntp_ctl(0);
	isb();
}

static void __secure __mdelay(u32 ms)
{
	u32 reg = ONE_MS * ms;

	cp15_write_cntp_tval(reg);
	isb();
	cp15_write_cntp_ctl(3);

	do {
		isb();
		reg = cp15_read_cntp_ctl();
	} while (!(reg & BIT(2)));

	cp15_write_cntp_ctl(0);
	isb();
}

static void __secure clamp_release(u32 __maybe_unused *clamp)
{
#if defined(CONFIG_MACH_SUN6I) || defined(CONFIG_MACH_SUN7I) || \
	defined(CONFIG_MACH_SUN8I_H3) || \
	defined(CONFIG_MACH_SUN8I_R40)
	u32 tmp = 0x1ff;
	do {
		tmp >>= 1;
		writel(tmp, clamp);
	} while (tmp);

	__mdelay(10);
#endif
}

static void __secure clamp_set(u32 __maybe_unused *clamp)
{
#if defined(CONFIG_MACH_SUN6I) || defined(CONFIG_MACH_SUN7I) || \
	defined(CONFIG_MACH_SUN8I_H3) || \
	defined(CONFIG_MACH_SUN8I_R40)
	writel(0xff, clamp);
#endif
}

static void __secure sunxi_power_switch(u32 *clamp, u32 *pwroff, bool on,
					int cpu)
{
	if (on) {
		/* Release power clamp */
		clamp_release(clamp);

		/* Clear power gating */
		clrbits_le32(pwroff, BIT(cpu));
	} else {
		/* Set power gating */
		setbits_le32(pwroff, BIT(cpu));

		/* Activate power clamp */
		clamp_set(clamp);
	}
}

#ifdef CONFIG_MACH_SUN8I_R40
/* secondary core entry address is programmed differently on R40 */
static void __secure sunxi_set_entry_address(void *entry)
{
	writel((u32)entry,
	       SUNXI_SRAMC_BASE + SUN8I_R40_SRAMC_SOFT_ENTRY_REG0);
}
#else
static void __secure sunxi_set_entry_address(void *entry)
{
	struct sunxi_cpucfg_reg *cpucfg =
		(struct sunxi_cpucfg_reg *)SUNXI_CPUCFG_BASE;

	writel((u32)entry, &cpucfg->priv0);
}
#endif

#ifdef CONFIG_MACH_SUN7I
/* sun7i (A20) is different from other single cluster SoCs */
static void __secure sunxi_cpu_set_power(int __always_unused cpu, bool on)
{
	struct sunxi_cpucfg_reg *cpucfg =
		(struct sunxi_cpucfg_reg *)SUNXI_CPUCFG_BASE;

	sunxi_power_switch(&cpucfg->cpu1_pwr_clamp, &cpucfg->cpu1_pwroff,
			   on, 0);
}
#elif defined CONFIG_MACH_SUN8I_R40
static void __secure sunxi_cpu_set_power(int cpu, bool on)
{
	struct sunxi_cpucfg_reg *cpucfg =
		(struct sunxi_cpucfg_reg *)SUNXI_CPUCFG_BASE;

	sunxi_power_switch((void *)cpucfg + SUN8I_R40_PWR_CLAMP(cpu),
			   (void *)cpucfg + SUN8I_R40_PWROFF,
			   on, cpu);
}
#else /* ! CONFIG_MACH_SUN7I && ! CONFIG_MACH_SUN8I_R40 */
static void __secure sunxi_cpu_set_power(int cpu, bool on)
{
	struct sunxi_prcm_reg *prcm =
		(struct sunxi_prcm_reg *)SUNXI_PRCM_BASE;

	sunxi_power_switch(&prcm->cpu_pwr_clamp[cpu], &prcm->cpu_pwroff,
			   on, cpu);
}
#endif /* CONFIG_MACH_SUN7I */

void __secure sunxi_cpu_power_off(u32 cpuid)
{
	struct sunxi_cpucfg_reg *cpucfg =
		(struct sunxi_cpucfg_reg *)SUNXI_CPUCFG_BASE;
	u32 cpu = cpuid & 0x3;

	/* Wait for the core to enter WFI */
	while (1) {
		if (readl(&cpucfg->cpu[cpu].status) & BIT(2))
			break;
		__mdelay(1);
	}

	/* Assert reset on target CPU */
	writel(0, &cpucfg->cpu[cpu].rst);

	/* Lock CPU (Disable external debug access) */
	clrbits_le32(&cpucfg->dbg_ctrl1, BIT(cpu));

	/* Power down CPU */
	sunxi_cpu_set_power(cpuid, false);

	/* Unlock CPU (Disable external debug access) */
	setbits_le32(&cpucfg->dbg_ctrl1, BIT(cpu));
}

static u32 __secure cp15_read_scr(void)
{
	u32 scr;

	asm volatile ("mrc p15, 0, %0, c1, c1, 0" : "=r" (scr));

	return scr;
}

static void __secure cp15_write_scr(u32 scr)
{
	asm volatile ("mcr p15, 0, %0, c1, c1, 0" : : "r" (scr));
	isb();
}

static int __secure sunxi_gpio_output(u32 pin, u32 val)
{
	u32 dat;
	u32 bank = GPIO_BANK(pin);
	u32 num = GPIO_NUM(pin);
	struct sunxi_gpio *pio = BANK_TO_GPIO(bank);

	dat = readl(&pio->dat);
	if (val)
		dat |= 0x1 << num;
	else
		dat &= ~(0x1 << num);

	writel(dat, &pio->dat);

	return 0;
}

static bool __secure is_all_secondary_cpu_locked() {
	int i;
	dsb();
	psci_v7_flush_dcache_all();
	for (i = 0; i < 4; i++) {
		if (!cpu_lock_state[i])
			return false;
	}
	return true;
}

static bool __secure is_all_secondary_cpu_unlocked() {
	int i;
	dsb();
	psci_v7_flush_dcache_all();
	for (i = 0; i < 4; i++) {
		if (cpu_lock_state[i])
			return false;
	}
	return true;
}

void __secure __gpio_debug(u32 v) {
	sunxi_gpio_output(SUNXI_GPA(9), v & 1);
	sunxi_gpio_output(SUNXI_GPA(10), (v >> 1) & 1);
	sunxi_gpio_output(SUNXI_GPA(20), (v >> 2) & 1);
	sunxi_gpio_output(SUNXI_GPA(15), (v >> 3) & 1);
}

/*
 * Although this is an FIQ handler, the FIQ is processed in monitor mode,
 * which means there's no FIQ banked registers. This is the same as IRQ
 * mode, so use the IRQ attribute to ask the compiler to handler entry
 * and return.
 */
void __secure __irq psci_fiq_enter(void)
{
	u32 scr, reg, cpu, irqn;

	/* Switch to secure mode */
	scr = cp15_read_scr();
	cp15_write_scr(scr & ~BIT(0));

	/* Validate reason based on IAR and acknowledge */
	reg = readl(GICC_BASE + GICC_IAR);
	irqn = reg & 0x3FF;

	/* Skip spurious interrupts 1022 and 1023 */
	if (reg == 1023 || reg == 1022)
		goto out;

	/* Get CPU number */
	cpu = (reg >> 10) & 0x7;

	/* End of interrupt */
	writel(reg, GICC_BASE + GICC_EOIR);
	dsb();

	switch (irqn) {
		case 14:
			cpu_lock_state[psci_get_cpu_id()] = true;
			while (cpu_lock_state[cpu]) {
				dsb();
				psci_v7_flush_dcache_all();
			}
			cpu_lock_state[psci_get_cpu_id()] = false;
			dsb();
			psci_v7_flush_dcache_all();
			
		break;
		
		case 15:
			/* Power off the CPU */
			sunxi_cpu_power_off(cpu);
		break;
	}

out:
	/* Restore security level */
	cp15_write_scr(scr);
}

int __secure psci_cpu_on(u32 __always_unused unused, u32 mpidr, u32 pc,
			 u32 context_id)
{
	struct sunxi_cpucfg_reg *cpucfg =
		(struct sunxi_cpucfg_reg *)SUNXI_CPUCFG_BASE;
	u32 cpu = (mpidr & 0x3);

	/* store target PC and context id */
	psci_save(cpu, pc, context_id);

	/* Set secondary core power on PC */
	sunxi_set_entry_address(&psci_cpu_entry);

	/* Assert reset on target CPU */
	writel(0, &cpucfg->cpu[cpu].rst);

	/* Invalidate L1 cache */
	clrbits_le32(&cpucfg->gen_ctrl, BIT(cpu));

	/* Lock CPU (Disable external debug access) */
	clrbits_le32(&cpucfg->dbg_ctrl1, BIT(cpu));

	/* Power up target CPU */
	sunxi_cpu_set_power(cpu, true);

	/* De-assert reset on target CPU */
	writel(BIT(1) | BIT(0), &cpucfg->cpu[cpu].rst);

	/* Unlock CPU (Disable external debug access) */
	setbits_le32(&cpucfg->dbg_ctrl1, BIT(cpu));

	return ARM_PSCI_RET_SUCCESS;
}

s32 __secure psci_cpu_off(void)
{
	psci_cpu_off_common();

	/* Ask CPU0 via SGI15 to pull the rug... */
	writel(BIT(16) | 15, GICD_BASE + GICD_SGIR);
	dsb();

	/* Wait to be turned off */
	while (1)
		wfi();
}

void __secure psci_arch_init(void)
{
	u32 reg;

	/* SGI14 & SGI15 as Group-0 */
	clrbits_le32(GICD_BASE + GICD_IGROUPRn, BIT(14) | BIT(15));

	/* Set SGI14 & SGI15 priority to 0 */
	writeb(0, GICD_BASE + GICD_IPRIORITYRn + 14);
	writeb(0, GICD_BASE + GICD_IPRIORITYRn + 15);

	/* Be cool with non-secure */
	writel(0xff, GICC_BASE + GICC_PMR);

	/* Switch FIQEn on */
	setbits_le32(GICC_BASE + GICC_CTLR, BIT(3));

	reg = cp15_read_scr();
	reg |= BIT(2);  /* Enable FIQ in monitor mode */
	reg &= ~BIT(0); /* Secure mode */
	cp15_write_scr(reg);
}

#define MCTL_COM_BASE               ((void *)(0x1c62000))
#define MCTL_CTL_BASE               ((void *)(0x1c63000))

#define _CCM_PLL_DDR_REG            (CCM_PLL_BASE  +  0x20)
#define MC_WORK_MODE                (MCTL_COM_BASE +  0x00)
#define PIR                         (MCTL_CTL_BASE +  0x00)
#define PWRCTL                      (MCTL_CTL_BASE +  0x04)
#define PGSR0                       (MCTL_CTL_BASE +  0x10)
#define STATR                       (MCTL_CTL_BASE +  0x18)
#define DTCR                        (MCTL_CTL_BASE +  0xc0)
#define ODTMAP                      (MCTL_CTL_BASE + 0x120)
#define DXnGCR0(x)                  (MCTL_CTL_BASE + 0x344 + 0x80*(x))

s32 __secure sunxi_dram_dvfs_req(u32 __always_unused function_id,
				 u32 __always_unused freq, u32 flags)
{
	u32 rank_num, reg_val, cpu_mask = 0;
	unsigned int i = 0;

#if USE_LOCKING_BEFORE_DDRFRQ
	__gpio_debug(0);

	cpu_lock_state[psci_get_cpu_id()] = true;

	// Put all other CPU's into secure mode
	for (i = 0; i < 4; i++) {
		if (psci_get_cpu_id() != i) {
			cpu_mask |= 1 << i;
			cpu_lock_state[i] = false;
		}
	}
	writel((cpu_mask << 16) | 14, GICD_BASE + GICD_SGIR);
	dsb();
	
__gpio_debug(0b101);
	while (!is_all_secondary_cpu_locked()) {
		/*
		__gpio_debug((
			((cpu_lock_state[0] ? 1 : 0) << 0) |
			((cpu_lock_state[1] ? 1 : 0) << 1) |
			((cpu_lock_state[2] ? 1 : 0) << 2) |
			((cpu_lock_state[3] ? 1 : 0) << 3)
		));
		*/
	}
__gpio_debug(0);
#endif

	rank_num = readl(MC_WORK_MODE) & 0x1;

	/* 1. enter self-refresh and disable all master access */
	reg_val = readl(PWRCTL);
	reg_val |= (0x1<<0);
	reg_val |= (0x1<<8);
__gpio_debug(0b1000);
	psci_v7_flush_dcache_all();
	writel(reg_val, PWRCTL);
	__udelay(1);
__gpio_debug(0);

__gpio_debug(0b100);
	/* make sure enter self-refresh */
	while ((readl(STATR) & 0x7) != 0x3)
		;
__gpio_debug(0);

	/* 2.Update PLL setting and wait 1ms */
	reg_val = readl(SUNXI_CCM_BASE + 0x20);
	reg_val |= (1U << 20);
	writel(reg_val, SUNXI_CCM_BASE + 0x20);
	__udelay(1000);

	/* 3.set PIR register issue phy reset and DDL calibration */
	if (rank_num) {
		reg_val = readl(DTCR);
		reg_val &= ~(0x3<<24);
		reg_val |= (0x3<<24);
		writel(reg_val, DTCR);
	} else {
		reg_val = readl(DTCR);
		reg_val &= ~(0x3<<24);
		reg_val |= (0x1<<24);
		writel(reg_val, DTCR);
	}

	/* trigger phy reset and DDL calibration */
	writel(0x40000061, PIR);
	/* add 1us delay here */
	__udelay(1);

__gpio_debug(0b001);
	/* wait for DLL Lock */
	while ((readl(PGSR0) & 0x1) != 0x1)
		;
__gpio_debug(0);

	/*4.setting ODT configure */
	if (!(flags & BIT(0))) {
		/* turn off DRAMC ODT */
		for (i = 0; i < 4; i++) {
			reg_val = readl(DXnGCR0(i));
			reg_val &= ~(0x3U<<4);
			reg_val |= (0x2<<4);
			writel(reg_val, DXnGCR0(i));
		}
	} else {
		/* turn on DRAMC dynamic ODT */
		for (i = 0; i < 4; i++) {
			reg_val = readl(DXnGCR0(i));
			reg_val &= ~(0x3U<<4);
			writel(reg_val, DXnGCR0(i));
		}
	}

	/* 5.exit self-refresh and enable all master access */
	reg_val = readl(PWRCTL);
	reg_val &= ~(0x1<<0);
	reg_val &= ~(0x1<<8);
	writel(reg_val, PWRCTL);
	__udelay(1);

__gpio_debug(0b1000);
	/* make sure exit self-refresh */
	while ((readl(STATR) & 0x7) != 0x1)
__gpio_debug(0);
		;
	
#if USE_LOCKING_BEFORE_DDRFRQ
	cpu_lock_state[psci_get_cpu_id()] = false;

__gpio_debug(0b111);
	while (!is_all_secondary_cpu_unlocked());
__gpio_debug(0);
#endif
	
	return 0;
}
