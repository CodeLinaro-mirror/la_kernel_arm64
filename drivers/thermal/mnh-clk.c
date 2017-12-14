/*
 *
 * MNH Clock Driver
 * Copyright (c) 2016-2017, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/intel-hwio.h>
#include <linux/spinlock.h>
#include <soc/mnh/mnh-hwio-scu.h>
#include <soc/mnh/mnh-hwio-cpu.h>
#include <soc/mnh/mnh-hwio-ddr-ctl.h>
#include "mnh-clk.h"

#define PLL_UNLOCK 0x4CD9
#define REF_FREQ_SEL 0x8
#define MAX_STR_COPY 9
#define LP4_LPC_FREQ_SWITCH 0x8A
#define SYS200_CLK_KHZ 200000

#define DEVICE_NAME "mnh_freq_cooling"

#define LP_CMD_EXIT_LP 0x81

#define MR_READ_SBIT 23
#define LP_CMD_SBIT 5

#define INIT_REFRESH_RATE 0x06

/* consider moving to struct */
u8 previous_refresh_rate;
u32 mnh_ddr_refresh_msec = 100;
struct delayed_work mnh_ddr_adjust_refresh_work;

#define MNH_CPU_IN(reg) \
HW_IN(mnh_dev->cpuaddr, CPU, reg)
#define MNH_DDR_CTL_IN(reg) \
HW_IN(mnh_dev->ddraddr, DDR_CTL, reg)
#define MNH_DDR_CTL_INf(reg, fld) \
HW_INf(mnh_dev->ddraddr, DDR_CTL, reg, fld)
#define MNH_DDR_CTL_OUTf(reg, fld, val) \
HW_OUTf(mnh_dev->ddraddr, DDR_CTL, reg, fld, val)
#define MNH_DDR_CTL_OUT(reg, val) \
HW_OUT(mnh_dev->ddraddr, DDR_CTL, reg, val)

/* If IPU clock is driven by CPU_IPU PLL
 * calculate IPU divider based on CPU clk and divider
 * CPU CLK < 850: IPU_CLK_DIV = ((CPU_CLK_DIV+1)*2-1)
 * CPU CLK > 850: IPU_CLK_DIV = ((CPU_CLK_DIV+1)*2)
 */
#define IPU_DIV_BY_CPU(freq, div) \
	((freq < 850) ? ((div+1)*2-1):((div+1)*2))

int mnh_ddr_clr_int_status(void);

enum mnh_refclk_type {
	REFCLK_KHZ_19200 = 0,
	REFCLK_KHZ_24000,
	REFCLK_KHZ_MAX
};

enum mnh_pll_type {
	CPU_CLK = 0,
	IPU_CLK
};

enum mnh_lpddr_lpc_rsp_type {
	LPC_CMD_NOERR = 0,
	LPC_CMD_ERR = 1
};

enum mnh_ipu_clk_src {
	CPU_IPU_PLL = 0,/*Both CPU and IPU Clock is derived from same PLL */
	IPU_PLL	/* IPU Clock is derived from IPU PLL */
};

struct freq_reg_table {
	char *freq_str;
	int fbdiv;
	int postdiv1;
	int postdiv2;
	int clk_div;
};

/* PLL reference clk table */
static uint32_t refclk_khz_tables[] = {
	19200,	/* 19.2 MHz */
	24000	/* 24.0 MHz */
};

/* SYS200 frequency calculation tables
 * SYS200  FBDIV FBDIV	POSTDIV1 POSTDIV2 FOUTPOSTDIV CLKDIV CLKFreq
 * CPU	 200	 104	 2	  1	   200.000	0	 200.000
 * IPU	 200	 104	 2	  1	   200.000	1	 100.000
 */
static const struct freq_reg_table sys200_reg_tables[] = {
	{"200", 104, 2, 1, 0},	/* CPU, 200 MHz */
	{"100", 104, 2, 1, 1}	/* IPU, 100 MHz */
};

/* CPU clock frequency calculation table */
static const struct freq_reg_table cpu_reg_tables[][CPU_FREQ_MAX+1] = {
	/* refclk = 19.2 MHz */
	{
		{"200", 125, 6, 2, 0},	/* 200 MHz */
		{"400", 125, 6, 1, 0},	/* 400 MHz */
		{"600", 125, 4, 1, 0},	/* 600 MHz */
		{"800", 125, 3, 1, 0},	/* 800 MHz */
		{"950", 99, 2, 1, 0}	/* 950 MHz */
	},
	/* refclk = 24.0 MHz */
	{
		{"200", 100, 6, 2, 0},	/* 200 MHz */
		{"400", 100, 6, 1, 0},	/* 400 MHz */
		{"600", 100, 4, 1, 0},	/* 600 MHz */
		{"800", 100, 3, 1, 0},	/* 800 MHz */
		{"950", 79, 2, 1, 0}	/* 948 MHz */
	}
};

/* IPU clock frequency calculation table */
static const struct freq_reg_table ipu_reg_tables[][IPU_FREQ_MAX+1] = {
	/* refclk = 19.2 MHz */
	{
		{"100", 125, 6, 2, 1},	/* 100 MHz */
		{"200", 125, 6, 1, 1},	/* 200 MHz */
		{"300", 125, 4, 1, 1},	/* 300 MHz */
		{"400", 125, 6, 1, 0},	/* 400 MHz */
		{"425", 133, 6, 1, 0}	/* 425 MHz */
	},
	/* refclk = 24.0 MHz */
	{
		{"100", 100, 6, 2, 1},	/* 100 MHz */
		{"200", 100, 6, 1, 1},	/* 200 MHz */
		{"300", 100, 4, 1, 1},	/* 300 MHz */
		{"400", 100, 6, 1, 0},	/* 400 MHz */
		{"425", 106, 6, 1, 0}	/* 424 MHz */
	}
};

struct mnh_freq_cooling_device {
	struct device *dev;
	void __iomem *regs;
	void __iomem *ddraddr;
	void __iomem *cpuaddr;
	int ddr_irq;
	struct completion ddr_mrr;
	u32 ddr_mrr4;
	struct completion ddr_lp_cmd;
	enum mnh_cpu_freq_type cpu_freq;
	enum mnh_lpddr_freq_type ddr_freq;
	enum mnh_refclk_type refclk;
	const struct freq_reg_table *cpu_pllcfg;
	const struct freq_reg_table *ipu_pllcfg;
	spinlock_t reset_lock;
};

static struct mnh_freq_cooling_device *mnh_dev;

/* Frequency calculation by PLL configuration
 * @cfg: struct freq_reg_table with pll config information.
 * Return: freq MHz.
 *
 * This returns current frequency in MHz unit
 * freq = (refclk*fbdiv)/((postdiv1*postdiv2)*(clk_div+1))
 */
static int mnh_freq_get_by_pll(struct freq_reg_table cfg, int sys200)
{
	uint32_t freq_khz;

	if (sys200)
		freq_khz = (SYS200_CLK_KHZ)/(cfg.clk_div+1);
	else
		freq_khz = (refclk_khz_tables[mnh_dev->refclk]*cfg.fbdiv)/
			((cfg.postdiv1*cfg.postdiv2)*(cfg.clk_div+1));

	return (freq_khz/1000);
}

static int mnh_get_cpu_pllcfg(struct freq_reg_table *pllcfg)
{
	int sys200;

	/* Check CPU is in SYS200 mode */
	sys200 = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE);

	/* Calculate frequency by PLL configuration */
	pllcfg->fbdiv = HW_INf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV,
			       FBDIV);
	pllcfg->postdiv1 = HW_INf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV,
				  POSTDIV1);
	pllcfg->postdiv2 = HW_INf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV,
				  POSTDIV2);
	pllcfg->clk_div = HW_INf(mnh_dev->regs, SCU, CCU_CLK_DIV, CPU_CLK_DIV);

	return mnh_freq_get_by_pll(*pllcfg, sys200);
}

static int mnh_get_ipu_pllcfg(struct freq_reg_table *pllcfg)
{
	int sys200, clk_src;

	/* Check IPU is in SYS200 mode */
	clk_src = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC);
	sys200 = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE);
	if (sys200 && (clk_src == IPU_PLL))
		sys200 = 0;

	/* Calculate frequency by PLL configuration */
	if (clk_src == CPU_IPU_PLL) {
		pllcfg->fbdiv = HW_INf(mnh_dev->regs, SCU,
				       CPU_IPU_PLL_INTGR_DIV, FBDIV);
		pllcfg->postdiv1 = HW_INf(mnh_dev->regs, SCU,
					  CPU_IPU_PLL_INTGR_DIV, POSTDIV1);
		pllcfg->postdiv2 = HW_INf(mnh_dev->regs, SCU,
					  CPU_IPU_PLL_INTGR_DIV, POSTDIV2);
	} else {
		pllcfg->fbdiv = HW_INf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV,
				       FBDIV);
		pllcfg->postdiv1 = HW_INf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV,
					  POSTDIV1);
		pllcfg->postdiv2 = HW_INf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV,
					  POSTDIV2);
	}
	pllcfg->clk_div = HW_INf(mnh_dev->regs, SCU, CCU_CLK_DIV, IPU_CLK_DIV);

	return mnh_freq_get_by_pll(*pllcfg, sys200);
}

static int mnh_get_cpu_freq(void)
{
	struct freq_reg_table pllcfg;

	return mnh_get_cpu_pllcfg(&pllcfg);
}

static int mnh_get_ipu_freq(void)
{
	struct freq_reg_table pllcfg;

	return mnh_get_ipu_pllcfg(&pllcfg);
}

int mnh_cpu_freq_to_index(void)
{
	int fbdiv, postdiv1, postdiv2, clk_div;
	int sys200, i;
	const struct freq_reg_table *table = cpu_reg_tables[mnh_dev->refclk];

	sys200 = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE);
	if (sys200)
		return 0;

	fbdiv = HW_INf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, FBDIV);
	postdiv1 = HW_INf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, POSTDIV1);
	postdiv2 = HW_INf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, POSTDIV2);
	clk_div = HW_INf(mnh_dev->regs, SCU, CCU_CLK_DIV, CPU_CLK_DIV);

	for (i = 0; i < ARRAY_SIZE(cpu_reg_tables[0]); i++) {
		if ((fbdiv == table[i].fbdiv) &&
		    (postdiv1 == table[i].postdiv1) &&
		    (postdiv2 == table[i].postdiv2) &&
		    (clk_div == table[i].clk_div))
			return i;
	}

	return -EINVAL;
}

int mnh_ipu_freq_to_index(void)
{
	int fbdiv, postdiv1, postdiv2, clk_div;
	int sys200, ipu_clk_src;
	int i;
	const struct freq_reg_table *table = ipu_reg_tables[mnh_dev->refclk];

	sys200 = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE);
	if (sys200)
		return 0;

	ipu_clk_src = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC);
	if (ipu_clk_src == CPU_IPU_PLL)
		return mnh_cpu_freq_to_index();

	fbdiv = HW_INf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV, FBDIV);
	postdiv1 = HW_INf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV, POSTDIV1);
	postdiv2 = HW_INf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV, POSTDIV2);
	clk_div = HW_INf(mnh_dev->regs, SCU, CCU_CLK_DIV, IPU_CLK_DIV);

	for (i = 0; i < ARRAY_SIZE(ipu_reg_tables[0]); i++) {
		if ((fbdiv == table[i].fbdiv) &&
		    (postdiv1 == table[i].postdiv1) &&
		    (postdiv2 == table[i].postdiv2) &&
		    (clk_div == table[i].clk_div))
			return i;
	}

	return -EINVAL;
}

/**
 * CPU clock controller
 * @index: int with frquency table index info.
 * Return: 0 on success, an error code otherwise.
 *
 * 1PLL(CPU_IPU PLL) for CPU/IPU clocks. Since CPU and IPU clocks are derived
 * from same PLL in this mode, there would be restriction on achedivable clock
 * frequencies for CPU and IPU clocks. IPU clock would be half of CPU clock. Any
 * frequency changes is achieved by changing FBDIV(integer feedback division) of
 * the PLL(PLL output = FBDIV * REFCLK frequency).
 * Default CPU_CLK_DIV : 1, IPU_CLK_DIV: 2
 * CLK = (REFCLK FREQ * FBDIV) / ((POSTDIV1 * POSTDIV2) * (CLK_DIV + 1))
 */
int mnh_cpu_freq_change(int index)
{
	int lock = 0;
	int sys200, ipu_div = 0;
	const struct freq_reg_table *pllcfg;
	bool use_sys200;
	int ipu_clk_src;
	int cpu_freq, old_cpu_freq;
	int clk_div;

	if (!mnh_dev)
		return -ENODEV;

	if (index < CPU_FREQ_MIN || index > CPU_FREQ_MAX)
		return -EINVAL;

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, index);

	/* Get PLL config from reg table */
	pllcfg = &mnh_dev->cpu_pllcfg[index];

	/* Get current SYS200 mode */
	sys200 = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE);

	/* Calculate frequency from PLL config */
	old_cpu_freq = mnh_get_cpu_freq();
	cpu_freq = mnh_freq_get_by_pll(*pllcfg, 0);

	/* Check to see if we need to change the frequency */
	if (old_cpu_freq == cpu_freq) {
		dev_dbg(mnh_dev->dev, "%s: already at desired frequency\n",
			__func__);
		return 0;
	}

	/* Check to see if we should use SYS200 mode */
	if (cpu_freq == 200) {
		use_sys200 = true;
		clk_div = 0;
	} else {
		use_sys200 = false;
		clk_div = pllcfg->clk_div;
	}

	/* Unlock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, PLL_UNLOCK);

	/* Switch to SYS200 mode */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE, 0x1);

	/* Read current IPU clock source */
	ipu_clk_src = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC);

	/* Latch current settings going to PLL */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 1);

	/* Configure CPU PLL */
	if (use_sys200) {
		/* Power down PLL and PLL output */
		HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, PD, 1);
		HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);
		HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 0);
	} else {
		/* Configure FBDIV first and set POSTDIV1, POSTDIV2
		* Compute dividers based on REF_FREQ_SEL hardware strap
		* Check FBDIV * REFCLK is witin VCO range (950-3800MHz)
		*/
		HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, FBDIV,
			pllcfg->fbdiv);
		HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, POSTDIV1,
			pllcfg->postdiv1);
		HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_INTGR_DIV, POSTDIV2,
			pllcfg->postdiv2);

		/* Set FOUTPOSTDIVPD = 1 to avoid glitches to output */
		HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);

		/* Apply the updated PLL configurations  */
		HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 0);

		/* Power up PLL */
		HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, PD, 0);

		/* Wait for minimum 128REFCLK, 6.7usec for 19.2MHz refclk
		* before checking PLL lock
		*/
		udelay(7);

		/* Check PLL is locked */
		do {
			lock = HW_INf(mnh_dev->regs,
				SCU, CPU_IPU_PLL_STS, LOCK);
		} while (lock != 1);

		/* Set FOUTPOSTDIVPD = 0 to ensure clk output is un-gated */
		HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 0);
	}

	/* Configure CPU_CLK_DIV */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, CPU_CLK_DIV, clk_div);

	/* If IPU clock is driven by CPU_IPU PLL,
	*  configure IPU divider based on CPU divider value
	*  to make sure IPU clock does not go over its limit
	*/
	if (ipu_clk_src == CPU_IPU_PLL) {
		ipu_div = IPU_DIV_BY_CPU(cpu_freq, clk_div);
		HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, IPU_CLK_DIV, ipu_div);
	}

	/* Set final CPU clock source */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE,
		use_sys200);

	mnh_dev->cpu_freq = index;

	/* Lock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, 0);

	return 0;
}
EXPORT_SYMBOL(mnh_cpu_freq_change);

/**
 * IPU clock controller
 * @index: int with frquency table index info.
 * Return: 0 on success, an error code otherwise.
 *
 * Until IPU clock is configured, IPU clock is driven from PCIe or CPU_IPU PLL,
 * and once it is configured by driver, IPU PLL is used to control IPU clock.
 * To turn off IPU PLL, CPU frequency needs to be set to 200MHz to put
 * both CPU and IPU into SYS200 mode.
 */
int mnh_ipu_freq_change(int index)
{
	int lock = 0;
	const struct freq_reg_table *pllcfg;
	bool use_cpu_clk_src;
	int ipu_freq, old_ipu_freq, cpu_freq;
	int clk_div;
	int ipu_clken;

	if (!mnh_dev)
		return -ENODEV;

	if (index < IPU_FREQ_MIN || index > IPU_FREQ_MAX)
		return -EINVAL;

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, index);

	/* Get PLL config from reg table */
	pllcfg = &mnh_dev->ipu_pllcfg[index];

	/* Calculate frequency from PLL config */
	old_ipu_freq = mnh_get_ipu_freq();
	cpu_freq = mnh_get_cpu_freq();
	ipu_freq = mnh_freq_get_by_pll(*pllcfg, 0);

	/* Check to see if we need to change the frequency */
	if (old_ipu_freq == ipu_freq) {
		dev_dbg(mnh_dev->dev, "%s: already at desired frequency\n",
			__func__);
		return 0;
	}

	/* TODO: determine if we can get to IPU frequency from CPU frequency */
	if ((cpu_freq == 200) && (ipu_freq == 100)) {
		use_cpu_clk_src = true;
		clk_div = 1;
	} else if ((cpu_freq == 200) && (ipu_freq == 200)) {
		use_cpu_clk_src = true;
		clk_div = 0;
	} else {
		use_cpu_clk_src = false;
		clk_div = pllcfg->clk_div;
	}

	/* Disable IPU_PLL clock output */
	ipu_clken = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLKEN);
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLKEN, 0x0);

	/* Unlock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, PLL_UNLOCK);

	/* Switch to stable clock before freq switch to avoid glitches */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC, CPU_IPU_PLL);

	/* Set FRZ_PLL_IN=1 to latch the current settings going to PLL */
	HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FRZ_PLL_IN, 1);

	/* Configure IPU PLL */
	if (use_cpu_clk_src) {
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, PD, 1);
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FRZ_PLL_IN, 0);
	} else {
		/* Configure FBDIV first and set POSTDIV1, POSTDIV2 */
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV, FBDIV, pllcfg->fbdiv);
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV, POSTDIV1,
			pllcfg->postdiv1);
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_INTGR_DIV, POSTDIV2,
			pllcfg->postdiv2);

		/* Set FOUTPOSTDIVPD = 1 to avoid glitches to output */
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);

		/* Apply the updated PLL configurations  */
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FRZ_PLL_IN, 0);

		/* Power up PLL */
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, PD, 0);

		/* Wait for minimum 128REFCLK, 6.7usec for 19.2MHz refclk
		* before checking PLL lock
		*/
		udelay(7);

		/* Check PLL is locked */
		do {
			lock = HW_INf(mnh_dev->regs, SCU, IPU_PLL_STS, LOCK);
		} while (lock != 1);

		/* Set FOUTPOSTDIVPD = 0 to ensure clk output is un-gated */
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FOUTPOSTDIVPD, 0);
	}

	/* Configure IPU_CLK_DIV */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, IPU_CLK_DIV, clk_div);

	/* Go back to IPU PLL output */
	if (!use_cpu_clk_src)
		HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC, IPU_PLL);

	/* Lock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, 0);

	/* Enable IPU_PLL clock output */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLKEN, ipu_clken);

	return 0;
}
EXPORT_SYMBOL(mnh_ipu_freq_change);

/**
 * LPDDR clock control driver
 * @index: int with frquency table index info.
 * Return: 0 on success, an error code otherwise.
 *
 * LPDDR clock is controlled by LPC instead of using direct PLL configuration.
 * Precondition:
 * LPDDR refclk pll should be enabled at cold boot and resume
 * LPDDR FSPx registers should be configured at cold boot and resume
 */
int mnh_lpddr_freq_change(int index)
{
	int status = 0;

	if (!mnh_dev)
		return -ENODEV;

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, index);

	if (index < LPDDR_FREQ_MIN || index > LPDDR_FREQ_MAX)
		return -EINVAL;

	/* Check the requested FSP is already in use */
	mnh_dev->ddr_freq = HW_INf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS,
		LPDDR4_CUR_FSP);
	if (mnh_dev->ddr_freq == index) {
		dev_dbg(mnh_dev->dev, "requested fsp%d is in use\n", index);
		return 0;
	}

	if (!HW_INxf(mnh_dev->regs, SCU,
		LPDDR4_FSP_SETTING, index, FSP_SYS200_MODE))
		mnh_lpddr_sys200_mode(false);

	/* Disable LPC SW override */
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_CFG,
		LP4_FSP_SW_OVERRIDE, 0);

	/* Configure FSP index */
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_CFG,
		LPC_FREQ_CHG_COPY_NUM, index);

	/* Configure LPC cmd for frequency switch */
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_CFG,
		LPC_EXT_CMD, LP4_LPC_FREQ_SWITCH);

	/* Initiate LPC cmd to LPDDR controller */
	dev_info(mnh_dev->dev, "lpddr freq switching from fsp%d to fsp%d\n",
		mnh_dev->ddr_freq, index);
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_CFG, LPC_EXT_CMD_REQ, 1);

	/* Wait until LPC cmd process is done */
	do {
		status = HW_INf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS,
			LPC_CMD_DONE);
	} while (status != 1);

	/* Clear LPC cmd status */
	HW_OUTf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS, LPC_CMD_DONE, 1);

	/* Check LPC error status */
	if (HW_INf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS, LPC_CMD_RSP)
		== LPC_CMD_ERR) {
		/* Clear error status */
		HW_OUTf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS,
			LPC_CMD_RSP, 1);
		dev_err(mnh_dev->dev, "Failed to process lpc cmd:0x%x\n",
			LP4_LPC_FREQ_SWITCH);
		return -1;
	}

	/* Check FSPx switch status */
	if (HW_INf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS, LPDDR4_CUR_FSP)
		!= index) {
		dev_err(mnh_dev->dev, "Failed to switch to fsp%d\n", index);
		return -1;
	}

	mnh_dev->ddr_freq = index;

	if (HW_INxf(mnh_dev->regs, SCU,
		LPDDR4_FSP_SETTING, index, FSP_SYS200_MODE))
		mnh_lpddr_sys200_mode(true);

	mnh_ddr_clr_int_status();
	return 0;
}
EXPORT_SYMBOL(mnh_lpddr_freq_change);

/**
 * LPDDR clock control driver
 * Return: 0 on success, an error code otherwise.
 *
 * LPDDR clock is derived from sys200 clk instead of separate lpddr clk
 */
int mnh_lpddr_sys200_mode(bool enable)
{
	int lock;

	if (!mnh_dev)
		return -ENODEV;

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, enable);
	/* Unlock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, PLL_UNLOCK);

	if (enable) {
		/* Power down LPDDR PLL */
		HW_OUTf(mnh_dev->regs, SCU, LPDDR4_REFCLK_PLL_CTRL, PD, 1);
		HW_OUTf(mnh_dev->regs, SCU, LPDDR4_REFCLK_PLL_CTRL, FOUTPOSTDIVPD, 1);
		HW_OUTf(mnh_dev->regs, SCU, LPDDR4_REFCLK_PLL_CTRL, BYPASS, 1);

	} else {
		/* Power up LPDDR PLL */
		HW_OUTf(mnh_dev->regs, SCU, LPDDR4_REFCLK_PLL_CTRL, FRZ_PLL_IN, 1);
		HW_OUTf(mnh_dev->regs, SCU, LPDDR4_REFCLK_PLL_CTRL, PD, 0);
		HW_OUTf(mnh_dev->regs, SCU, LPDDR4_REFCLK_PLL_CTRL, FOUTPOSTDIVPD, 0);
		HW_OUTf(mnh_dev->regs, SCU, LPDDR4_REFCLK_PLL_CTRL, BYPASS, 0);
		HW_OUTf(mnh_dev->regs, SCU, LPDDR4_REFCLK_PLL_CTRL, FRZ_PLL_IN, 0);
		/* Check PLL is locked */
		do {
			lock = HW_INf(mnh_dev->regs, SCU, LPDDR4_REFCLK_PLL_STS, LOCK);
		} while (lock != 1);
	}

	/* Lock PLL */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, 0);

	return 0;
}
EXPORT_SYMBOL(mnh_lpddr_sys200_mode);

/**
 * CPU and IPU SYS200 clock control driver
 * Return: 0 on success, an error code otherwise.
 *
 * CPU and IPU clock is derived from sys200 clk instead of separate plls
 */
int mnh_cpu_ipu_sys200_mode(void)
{
	int ipu_clk_src;

	if (!mnh_dev)
		return -ENODEV;

	dev_dbg(mnh_dev->dev, "%s\n", __func__);

	/* Unlock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, PLL_UNLOCK);

	/* Switch to SYS200 mode */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE, 0x1);

	/* Read current IPU clock source */
	ipu_clk_src = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC);

	if (ipu_clk_src == IPU_PLL) {
		/* Change clk source to CPU_IPU_PLL */
		HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL,
			IPU_CLK_SRC, CPU_IPU_PLL);

		/* IPU: Latch current settings to go into PLL */
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FRZ_PLL_IN, 1);

		/* IPU: Power down PLL */
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, PD, 1);
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL,
			FOUTPOSTDIVPD, 1);

		/* IPU: Apply PLL configurations */
		HW_OUTf(mnh_dev->regs, SCU, IPU_PLL_CTRL, FRZ_PLL_IN, 0);
	}

	/* Configure IPU_CLK_DIV */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, IPU_CLK_DIV,
	sys200_reg_tables[IPU_CLK].clk_div);

	/* Configure CPU_CLK_DIV */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_DIV, CPU_CLK_DIV,
		sys200_reg_tables[CPU_CLK].clk_div);

	/* CPU_IPU: Latch current settings to go into PLL */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 1);

	/* CPU_IPU: Power down PLL */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, PD, 1);
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FOUTPOSTDIVPD, 1);

	/* CPU_IPU: Apply PLL configurations */
	HW_OUTf(mnh_dev->regs, SCU, CPU_IPU_PLL_CTRL, FRZ_PLL_IN, 0);

	mnh_dev->cpu_freq = 0;

	/* Lock PLL access */
	HW_OUTf(mnh_dev->regs, SCU, PLL_PASSCODE, PASSCODE, 0);

	return 0;
}
EXPORT_SYMBOL(mnh_cpu_ipu_sys200_mode);

/* read entire int_status */
u64 mnh_ddr_int_status(void)
{
	u64 int_stat;

	if (!mnh_dev)
		return -ENODEV;

	int_stat = ((u64)MNH_DDR_CTL_IN(228) << 32) | MNH_DDR_CTL_IN(227);
	return int_stat;
}
EXPORT_SYMBOL(mnh_ddr_int_status);

/* clear entire int_status */
int mnh_ddr_clr_int_status(void)
{
	u64 stat = 0;

	if (!mnh_dev)
		return -ENODEV;

	MNH_DDR_CTL_OUT(230, 0x0F);
	MNH_DDR_CTL_OUT(229, 0xFFFFFFFF);

	stat = mnh_ddr_int_status();
	if (stat) {
		pr_err("%s: int stat not all clear: %llx\n",
			__func__, stat);
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL(mnh_ddr_clr_int_status);

/* read single bit in int_status */
static u32 mnh_ddr_int_status_bit(u8 sbit)
{
	u64 status = 0;
	const u32 max_int_status_bit = 35;
	if (sbit > max_int_status_bit)
		return -EINVAL;

	status = mnh_ddr_int_status();
	status &= (1 << sbit);
	return status;
}

/* clear single bit in int_status */
static int mnh_ddr_clr_int_status_bit(u8 sbit)
{
	const u32 max_int_status_bit = 35;
	const u32 first_upper_bit = 32;
	if (sbit > max_int_status_bit)
		return -EINVAL;

	if (sbit >= first_upper_bit)
		MNH_DDR_CTL_OUT(230, 1 << (sbit - first_upper_bit));
	else
		MNH_DDR_CTL_OUT(229, 1 << sbit);

	if (mnh_ddr_int_status_bit(sbit)) {
		pr_err("%s: bit %d is still set.\n",
			__func__, sbit);
		return -1;
	}
	return 0;
}

static int mnh_ddr_send_lp_cmd(u8 cmd)
{
	unsigned long timeout = msecs_to_jiffies(100);

	dev_dbg(mnh_dev->dev,
		"%s sending cmd: 0x%x\n",
		__func__, cmd);

	reinit_completion(&mnh_dev->ddr_lp_cmd);
	MNH_DDR_CTL_OUTf(112, LP_CMD, cmd);

	if (!wait_for_completion_timeout(&mnh_dev->ddr_lp_cmd,
		timeout)) {
		dev_err(mnh_dev->dev,
		"%s ERROR timeout sending cmd: 0x%02x\n",
		__func__, cmd);
		return -ETIMEDOUT;
	}
	return 0;
}

static void mnh_ddr_enable_lp(void)
{
	u32 fsp = 0;

	/*
	These are roughly scaled to the frequency of the fsp
	*/
	const u32 sleep_val[LPDDR_FREQ_NUM_FSPS] = {
		0x04, 0x10, 0x40, 0x60 };

	fsp = HW_INf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS,
				LPDDR4_CUR_FSP);

	if (fsp >= LPDDR_FREQ_NUM_FSPS)
		fsp = LPDDR_FREQ_MAX;

	dev_dbg(mnh_dev->dev, "%s\n", __func__);

	MNH_DDR_CTL_OUTf(124, LP_AUTO_SR_MC_GATE_IDLE,
		sleep_val[fsp]);

	MNH_DDR_CTL_OUTf(122, LP_AUTO_MEM_GATE_EN, 0x4);
	MNH_DDR_CTL_OUTf(122, LP_AUTO_ENTRY_EN, 0x4);
	MNH_DDR_CTL_OUTf(122, LP_AUTO_EXIT_EN, 0xF);
}

static void mnh_ddr_disable_lp(void)
{
	dev_dbg(mnh_dev->dev, "%s\n", __func__);
	MNH_DDR_CTL_OUTf(124, LP_AUTO_SR_MC_GATE_IDLE, 0x00);
	MNH_DDR_CTL_OUTf(122, LP_AUTO_MEM_GATE_EN, 0x0);
	MNH_DDR_CTL_OUTf(122, LP_AUTO_ENTRY_EN, 0x0);
	MNH_DDR_CTL_OUTf(122, LP_AUTO_EXIT_EN, 0x0);
	mnh_ddr_send_lp_cmd(LP_CMD_EXIT_LP);
}

/*
val0 and val1 are the values for modereg, read from
chip 0 and chip 1 respectively.
*/
int mnh_ddr_read_mode_reg(u8 modereg, u8 *val0, u8 *val1)
{
	int ret = 0;
	const u64 readable = 0x00000000030C51f1;
	u32 val = 0;
	unsigned long timeout = 0;
	u32 peripheral_mrr_data = 0;

	if ((modereg >= 64) ||
		((readable & ((u64)1 << modereg)) == 0)) {
		pr_err("%s %d is not readable.\n",
			__func__, modereg);
		*val0 = 0;
		*val1 = 0;
		return -1;
	}

	val = 0xFF & modereg;
	val |= 1 << 16;

	mnh_ddr_disable_lp();

	timeout = msecs_to_jiffies(50);
	reinit_completion(&mnh_dev->ddr_mrr);
	MNH_DDR_CTL_OUTf(141, READ_MODEREG, val);

	if (!wait_for_completion_timeout(&mnh_dev->ddr_mrr,
			timeout)) {
		pr_err("%s timeout on mrr\n", __func__);
		mnh_ddr_enable_lp();
		return -ETIMEDOUT;
	}

	mnh_ddr_enable_lp();

	peripheral_mrr_data =
		MNH_DDR_CTL_INf(142, PERIPHERAL_MRR_DATA);

	*val0 = 0xFF & peripheral_mrr_data;
	*val1 = 0xFF & (peripheral_mrr_data >> 16);
	dev_dbg(mnh_dev->dev,
		"%s values: 0x%02x 0x%02x\n",
		__func__, *val0, *val1);
	return ret;
}

int mnh_ddr_adjust_refresh_suspend(void)
{
	int ret = cancel_delayed_work(&mnh_ddr_adjust_refresh_work);
	/*
	AP will re-init (resume) ddr with hottest settings
	and we will adjust appropriately on resume,
	just like cold-boot case.
	*/
	previous_refresh_rate = INIT_REFRESH_RATE;
	return ret;
}

int mnh_ddr_adjust_refresh_resume(void)
{
	return schedule_delayed_work(&mnh_ddr_adjust_refresh_work,
			mnh_ddr_refresh_msec);
}

static u16 mnh_ddr_update_refresh(u8 old_rate, u16 old_interval,
	u8 new_rate)
{
	u16 new_interval;

	/* mask out everything but refresh rate */
	old_rate &= 0x07;
	new_rate &= 0x07;

	if ((new_rate == 0) ||
		(new_rate == 0x7)) {
		panic("ddr temp exceeds parameters: 0x%02x "
			" and preventing undeterministic side effects now",
			new_rate);
	}

	if (/* no changes */
		(old_rate == new_rate) ||
		/* same range */
		((old_rate == 0x5) && (new_rate == 0x6)) ||
		((old_rate == 0x6) && (new_rate == 0x5)))
		return old_interval;

	if (new_rate > old_rate) {
		/* getting hotter */
		new_interval = old_interval >> (new_rate - old_rate);
	} else {
		/* getting cooler */
		if (old_rate > 0x5)
			old_rate = 0x5;
		new_interval = old_interval << (old_rate - new_rate);
	}

	pr_info("%s changed 0x%04x to 0x%04x\n",
		__func__, old_interval, new_interval);

	return new_interval;
}

static void mnh_ddr_adjust_refresh(u8 refresh_rate)
{
	u16 tref[LPDDR_FREQ_NUM_FSPS];
	int fsp;

	/* sanitize refresh rate */
	refresh_rate &= 0x07;

	if (refresh_rate != previous_refresh_rate) {
		pr_info("%s 0x%02x -> 0x%02x\n",
			__func__, previous_refresh_rate,
			refresh_rate);
		tref[0]		= MNH_DDR_CTL_INf(56,  TREF_F0);
		tref[1]		= MNH_DDR_CTL_INf(57,  TREF_F1);
		tref[2]		= MNH_DDR_CTL_INf(58,  TREF_F2);
		tref[3]		= MNH_DDR_CTL_INf(59,  TREF_F3);

		for (fsp = 0; fsp < LPDDR_FREQ_NUM_FSPS; fsp++) {
			tref[fsp] =
				mnh_ddr_update_refresh(previous_refresh_rate,
					tref[fsp],
					refresh_rate);
		}

		MNH_DDR_CTL_OUTf(56,  TREF_F0,     tref[0]);
		MNH_DDR_CTL_OUTf(57,  TREF_F1,     tref[1]);
		MNH_DDR_CTL_OUTf(58,  TREF_F2,     tref[2]);
		MNH_DDR_CTL_OUTf(59,  TREF_F3,     tref[3]);
		previous_refresh_rate = refresh_rate;
	}
}

static void mnh_ddr_adjust_refresh_worker(struct work_struct *work)
{
	u8 val0 = 0, val1 = 0;
	u32 combined_val = 0;
	const u8 rr_fld = 0x07;
	u8 refresh_rate;
	const u8 tuf_fld = 0x80;
	int ret = 0;
	int got_tuf = 0;

	ret = mnh_ddr_read_mode_reg(4, &val0, &val1);
	mnh_dev->ddr_mrr4 = (u32)val1 << 16 | (u32)val0;

	if (ret) {
		dev_err(mnh_dev->dev,
			"%s ERROR refresh rate read failed. %d\n",
			__func__, ret);
		goto refresh_again;
	}

	refresh_rate = rr_fld & val0;
	if (refresh_rate < (rr_fld & val1)) {
		dev_dbg(mnh_dev->dev,
			"%s rrs don't match: 0x%02x != 0x%02x"
			"deferring to higher temp die rate",
			__func__, val0, val1);
		refresh_rate = rr_fld & val1;
	}

	if ((tuf_fld & val0) ||
		(tuf_fld & val1)) {
		pr_debug("%s TUF 0x%02x 0x%02x\n",
			 __func__, val0, val1);
		got_tuf = 1;
	}

	mnh_ddr_adjust_refresh(refresh_rate);

	if (!ret) {
		/* AP can easily read this from here */
		combined_val = (u32)val0 | (u32)val1 << 16;
		HW_OUTx(mnh_dev->regs, SCU,
			GPS, 3, combined_val);
	}
refresh_again:
	schedule_delayed_work(&mnh_ddr_adjust_refresh_work,
		msecs_to_jiffies(mnh_ddr_refresh_msec));
}

/**
 * Set the SCU clock gating at init
 * Return: 0 on success, an error code otherwise.
 *
 * enable == 1 sets the SCU to enable clock gating in general when CPU enters
 * L2 WFI state.
 */
int mnh_clock_init_gating(int enabled)
{
	unsigned long irq_flags;

	if (!mnh_dev)
		return -ENODEV;

	dev_dbg(mnh_dev->dev, "%s:%d\n", __func__, __LINE__);

	if (enabled != 1 && enabled != 0)
		return -EINVAL;

	spin_lock_irqsave(&mnh_dev->reset_lock, irq_flags);

	/* Add periph clk gates */
	HW_OUTf(mnh_dev->regs, SCU, RSTC, PERI_DMA_RST, enabled);

	spin_unlock_irqrestore(&mnh_dev->reset_lock, irq_flags);

	HW_OUTf(mnh_dev->regs, SCU, PERIPH_CLK_CTRL, PERI_DMA_CLKEN_SW,
		!enabled);
	HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, HALT_BTROM_PD_EN, enabled);
	HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, HALT_BTSRAM_PD_EN, enabled);
	HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, BTROM_SLP, enabled);
	HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, BTSRAM_DS, enabled);
	/* HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, HALT_AHBCG_EN, enabled); */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, HALT_BTSRAMCG_EN, enabled);
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, HALT_BTROMCG_EN, enabled);

	/* HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, HALT_LP4CG_EN, enabled); */
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, HALT_LP4_PLL_BYPCLK_CG_EN,
		enabled);
	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, LP4PHY_PLL_BYPASS_CLKEN,
		enabled);

	return 0;
}
EXPORT_SYMBOL(mnh_clock_init_gating);

int mnh_bypass_clock_gating(int enabled)
{
	unsigned long irq_flags;

	if (!mnh_dev)
		return -ENODEV;

	dev_dbg(mnh_dev->dev, "%s:%d\n", __func__, __LINE__);

	if (enabled != 1 && enabled != 0)
		return -EINVAL;

	if (enabled) {
		HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, HALT_CPUCG_EN,
			enabled);
		HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, HALT_CPUMEM_PD_EN,
			enabled);
		HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, CPU_L2MEM_DS,
			enabled);
		HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, CPU_L1MEM_DS,
			enabled);
		HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, HALT_LP4CMEM_PD_EN,
			enabled);
		HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, LP4C_MEM_DS,
			enabled);
	} else {
		HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, CPU_L2MEM_DS,
			enabled);
		HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, CPU_L1MEM_DS,
			enabled);
		HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, HALT_CPUMEM_PD_EN,
			enabled);
		HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, LP4C_MEM_DS,
			enabled);
		HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, HALT_LP4CMEM_PD_EN,
			enabled);
		HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, HALT_CPUCG_EN,
			enabled);
	}


	spin_lock_irqsave(&mnh_dev->reset_lock, irq_flags);

	HW_OUTf(mnh_dev->regs, SCU, RSTC, WDT_RST, enabled);

	spin_unlock_irqrestore(&mnh_dev->reset_lock, irq_flags);

	HW_OUTf(mnh_dev->regs, SCU, PERIPH_CLK_CTRL, PVT_CLKEN, !enabled);
	HW_OUTf(mnh_dev->regs, SCU, PERIPH_CLK_CTRL, WDT_CLKEN_SW, !enabled);

	return 0;
}
EXPORT_SYMBOL(mnh_bypass_clock_gating);

int mnh_pcie_axi_clock_enable(int enabled)
{
	if (!mnh_dev)
		return -ENODEV;

	if (enabled != 1 && enabled != 0)
		return -EINVAL;

	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, PCIE_AXI_CLKEN, enabled);

	return 0;
}
EXPORT_SYMBOL(mnh_pcie_axi_clock_enable);

int mnh_axi_clock_gating(int enabled)
{
	if (!mnh_dev)
		return -ENODEV;

	if (enabled != 1 && enabled != 0)
		return -EINVAL;

	HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, HALT_AXICG_EN, enabled);

	return 0;
}
EXPORT_SYMBOL(mnh_axi_clock_gating);

/**
 * Set the SCU clock gating for bypass mode
 * Return: 0 on success, an error code otherwise.
 *
 * enable == 1 sets the SCU to enable clock gating in general when CPU enters
 * L2 WFI state.
 */
int mnh_ipu_clock_gating(int enabled)
{
	if (!mnh_dev)
		return -ENOENT;

	dev_dbg(mnh_dev->dev, "%s\n", __func__);

	if (enabled != 1 && enabled != 0)
		return -EINVAL;

	if (enabled) {
		HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLKEN, !enabled);
		HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, IPU_MEM_DS, enabled);
		HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, IPU_MEM_SD, enabled);
	} else {
		HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, IPU_MEM_SD, enabled);
		HW_OUTf(mnh_dev->regs, SCU, MEM_PWR_MGMNT, IPU_MEM_DS, enabled);
		HW_OUTf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLKEN, !enabled);
	}

	return 0;
}
EXPORT_SYMBOL(mnh_ipu_clock_gating);

int mnh_ipu_reset(void)
{
	unsigned long irq_flags;

	dev_dbg(mnh_dev->dev, "%s\n", __func__);
	if (!mnh_dev)
		return -ENOENT;

	spin_lock_irqsave(&mnh_dev->reset_lock, irq_flags);

	HW_OUTf(mnh_dev->regs, SCU, RSTC, IPU_RST, 1);
	HW_OUTf(mnh_dev->regs, SCU, RSTC, IPU_RST, 0);

	spin_unlock_irqrestore(&mnh_dev->reset_lock, irq_flags);

	return 0;
}
EXPORT_SYMBOL(mnh_ipu_reset);

/* Current reference clock rate
 * Return: refclk index of mnh_refclk_type
 *
 * This returns current reference clk rate by index of mnh_refclk_type
 */
static int mnh_freq_check_refclk(uint32_t refclk_gpio)
{
	int ret, refclk = 0;

	if (gpio_is_valid(refclk_gpio)) {
		ret = gpio_request(refclk_gpio, "REFCLK");
		if (ret)
			goto end_gpio;
		ret = gpio_direction_input(refclk_gpio);
		if (ret)
			goto err_gpio;
		if (gpio_get_value(refclk_gpio))
			refclk = refclk | 0x1;

		pr_debug("%s gpio:%d refclk:%d\n", __func__,
			refclk_gpio, refclk);
	}
err_gpio:
	if (gpio_is_valid(refclk_gpio))
		gpio_free(refclk_gpio);
end_gpio:
	return refclk;
}

static ssize_t cpu_freq_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%dMHz\n", mnh_get_cpu_freq());
}

static ssize_t cpu_freq_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int i, err;

	dev_dbg(mnh_dev->dev, "%s: %s\n", __func__, buf);
	for (i = 0; i < ARRAY_SIZE(cpu_reg_tables[0]); i++) {
		if (!strncmp(buf, mnh_dev->cpu_pllcfg[i].freq_str,
			strlen(mnh_dev->cpu_pllcfg[i].freq_str))) {
			err = mnh_cpu_freq_change(i);
			if (!err)
				return count;
			else
				return -EIO;
		}
	}

	dev_err(mnh_dev->dev, "invalid freq: %s\n", buf);
	return -EINVAL;

}

static const char * const ipu_freq_str[] = {"100", "200", "300", "400", "425"};
static ssize_t ipu_freq_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%dMHz\n", mnh_get_ipu_freq());
}

static ssize_t ipu_freq_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int i, err;

	dev_dbg(mnh_dev->dev, "%s: %s\n", __func__, buf);
	for (i = 0; i < ARRAY_SIZE(ipu_reg_tables[0]); i++) {
		if (!strncmp(buf, mnh_dev->ipu_pllcfg[i].freq_str,
			strlen(mnh_dev->ipu_pllcfg[i].freq_str))) {
			err = mnh_ipu_freq_change(i);
			if (!err)
				return count;
			else
				return -EIO;
		}
	}

	dev_err(mnh_dev->dev, "invalid freq: %s\n", buf);
	return -EINVAL;
}

static ssize_t lpddr_freq_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	uint32_t var = HW_INf(mnh_dev->regs, SCU, LPDDR4_LOW_POWER_STS,
				LPDDR4_CUR_FSP);

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, var);
	return sprintf(buf, "FSP%d\n", var);
}

static ssize_t lpddr_freq_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int var = 0;
	int ret;

	ret = kstrtoint(buf, 10, &var);
	if (ret < 0)
		return ret;

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, var);
	if (!mnh_lpddr_freq_change(var))
		return count;
	else
		return -EIO;
}

static ssize_t ipu_clk_src_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int ipu_clk_src = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC);

	return scnprintf(buf, MAX_STR_COPY, "%s\n",
		(ipu_clk_src == CPU_IPU_PLL) ? "CPU_IPU":"IPU");
}

static ssize_t sys200_freq_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int sys200, clk_src;

	clk_src = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLK_SRC);
	sys200 = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, CPU_IPU_SYS200_MODE);
	if (sys200 && (clk_src == IPU_PLL))
		sys200 = 0;

	return sprintf(buf, "%d\n", sys200);
}

static ssize_t sys200_freq_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int var = 0;
	int ret;

	ret = kstrtoint(buf, 10, &var);
	if (ret < 0)
		return ret;

	if (var == 1) {
		dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, var);
		if (!mnh_cpu_ipu_sys200_mode())
			return count;
	}
	return -EIO;
}

static ssize_t ipu_clock_gating_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int clk_gated;

	clk_gated = !HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, IPU_CLKEN);

	return sprintf(buf, "%d\n", clk_gated);
}

static ssize_t ipu_clock_gating_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int var = 0;
	int ret;

	ret = kstrtoint(buf, 10, &var);
	if (ret < 0)
		return ret;

	if (var == 1 || var == 0) {
		dev_info(mnh_dev->dev, "%s: %d\n", __func__, var);
		if (!mnh_ipu_clock_gating(var))
			return count;
	}
	return -EIO;
}

static ssize_t bypass_clock_gating_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int clk_gated;

	clk_gated = HW_INf(mnh_dev->regs, SCU, CCU_CLK_CTL, HALT_CPUCG_EN);

	return sprintf(buf, "%d\n", clk_gated);
}

static ssize_t bypass_clock_gating_set(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	int var = 0;
	int ret;

	ret = kstrtoint(buf, 10, &var);
	if (ret < 0)
		return ret;

	if (var == 1 || var == 0) {
		dev_info(mnh_dev->dev, "%s: %d\n", __func__, var);
		if (!mnh_bypass_clock_gating(var))
			return count;
	}
	return -EIO;
}

static ssize_t lpddr_lp_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d\n",
		MNH_DDR_CTL_INf(122, LP_AUTO_ENTRY_EN));
}

static ssize_t lpddr_lp_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int var = 0;
	int ret;

	ret = kstrtoint(buf, 10, &var);
	if (ret < 0)
		return ret;

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, var);
	if (var == 1)
		mnh_ddr_enable_lp();
	else if (var == 0)
		mnh_ddr_disable_lp();
	else
		return -EIO;

	return count;
}

static ssize_t lpddr_sys200_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int sys200_mode = 0, fsp = 0;
	if (HW_INf(mnh_dev->regs, SCU,
			LPDDR4_LOW_POWER_CFG,
			LP4_FSP_SW_OVERRIDE)) {
		sys200_mode = HW_INf(mnh_dev->regs, SCU,
			CCU_CLK_CTL, LP4_AXI_SYS200_MODE);
	} else {
		fsp = HW_INf(mnh_dev->regs, SCU,
			LPDDR4_LOW_POWER_STS, LPDDR4_CUR_FSP);
		sys200_mode = HW_INxf(mnh_dev->regs, SCU,
			LPDDR4_FSP_SETTING, fsp, FSP_SYS200_MODE);
	}
	return sprintf(buf, "%d\n", sys200_mode);
}

static ssize_t lpddr_sys200_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int var = 0;
	int ret;

	ret = kstrtoint(buf, 10, &var);
	if (ret < 0)
		return ret;

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, var);
	if ((var == 0) || (var == 1)) {
		mnh_lpddr_sys200_mode(var);
	} else {
		dev_err(mnh_dev->dev, "%s: Invalid argument", __func__);
		return -EINVAL;
	}

	return count;
}

static ssize_t lpddr_mrr4_get(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "0x%08x\n", mnh_dev->ddr_mrr4);
}

static ssize_t dump_powerregs_get(struct device *dev,
				struct device_attribute *attr,	
				char *buf)
{
	int val = 0;
       	const char* origbuf = buf;
	val = HW_IN(mnh_dev->regs, SCU, GLOBAL_IRQ_STATUS_SET0);
	buf += sprintf(buf, "GLOBAL_IRQ_STATUS_SET0\t\t0x%x\n", val);
	val = HW_IN(mnh_dev->regs, SCU, GLOBAL_IRQ_STATUS_SET1);
	buf += sprintf(buf, "GBOBAL_IRQ_STATUS_SET1\t\t0x%x\n", val);

	val = HW_IN(mnh_dev->regs, SCU, GLOBAL_IRQ_HLT_RST_EN_SET0);
	buf += sprintf(buf, "GLOBAL_IRQ_HLT_RST_EN_SET0\t0x%x\n", val);
	val = HW_IN(mnh_dev->regs, SCU, GLOBAL_IRQ_HLT_RST_EN_SET1);
	buf += sprintf(buf, "GLOBAL_IRQ_HLT_RST_EN_SET1\t0x%x\n", val);

	val = HW_IN(mnh_dev->regs, SCU, GLOBAL_WAKE_EN_SET0);
	buf += sprintf(buf, "GLOBAL_WAKE_EN_SET0\t\t0x%x\n", val);
	val = HW_IN(mnh_dev->regs, SCU, GLOBAL_WAKE_EN_SET1);
	buf += sprintf(buf, "GLOBAL_WAKE_EN_SET1\t\t0x%x\n", val);

	val = HW_IN(mnh_dev->regs, SCU, SCU_IRQ_STATUS);
	buf += sprintf(buf, "SCU_IRQ_STATUS\t\t\t0x%x\n", val);
	val = HW_IN(mnh_dev->regs, SCU, SCU_IRQ_ENABLE);
	buf += sprintf(buf, "SCU_IRQ_ENABLE\t\t\t0x%x\n", val);

	val = HW_IN(mnh_dev->regs, SCU, CCU_CLK_CTL);
	buf += sprintf(buf, "CCU_CLK_CTL\t\t\t0x%x\n", val);

	val = HW_IN(mnh_dev->regs, SCU, PERIPH_CLK_CTRL);
	buf += sprintf(buf, "PERIPH_CLK_CTRL\t\t\t0x%x\n", val);

	val = HW_IN(mnh_dev->regs, SCU, RSTC);
	buf += sprintf(buf, "RSTC\t\t\t\t0x%x\n", val);

	val = HW_IN(mnh_dev->regs, SCU, MEM_PWR_MGMNT);
	buf += sprintf(buf, "MEM_PWR_MGMNT\t\t\t0x%x\n", val);

	/* cpu for now */	
	val = MNH_CPU_IN(STS);
	buf += sprintf(buf, "CPU STS\t\t\t\t0x%x\n", val);

	val = MNH_CPU_IN(PWR_MGMT_STS);
	buf += sprintf(buf, "CPU PWR_MGMT_STS\t\t0x%x\n", val);

	val = MNH_DDR_CTL_IN(227);
	buf += sprintf(buf, "DDR_CTL 227\t\t\t0x%x\n", val);

	val = MNH_DDR_CTL_IN(228);
	buf += sprintf(buf, "DDR_CTL 228\t\t\t0x%x\n", val);


	return (ssize_t) (buf - origbuf);
}

static DEVICE_ATTR(cpu_freq, S_IWUSR | S_IRUGO,
		cpu_freq_get, cpu_freq_set);
static DEVICE_ATTR(ipu_freq, S_IWUSR | S_IRUGO,
		ipu_freq_get, ipu_freq_set);
static DEVICE_ATTR(lpddr_freq, S_IWUSR | S_IRUGO,
		lpddr_freq_get, lpddr_freq_set);
static DEVICE_ATTR(ipu_clk_src, S_IRUGO,
		ipu_clk_src_get, NULL);
static DEVICE_ATTR(sys200, S_IWUSR | S_IRUGO,
		sys200_freq_get, sys200_freq_set);
static DEVICE_ATTR(ipu_clock_gating, S_IWUSR | S_IRUGO,
		ipu_clock_gating_get, ipu_clock_gating_set);
static DEVICE_ATTR(bypass_clock_gating, S_IWUSR | S_IRUGO,
		bypass_clock_gating_get, bypass_clock_gating_set);
static DEVICE_ATTR(lpddr_lp, S_IWUSR | S_IRUGO,
		lpddr_lp_get, lpddr_lp_set);
static DEVICE_ATTR(lpddr_sys200, S_IWUSR | S_IRUGO,
		lpddr_sys200_get, lpddr_sys200_set);
static DEVICE_ATTR(lpddr_mrr4, S_IRUGO,
		lpddr_mrr4_get, NULL);
static DEVICE_ATTR(dump_powerregs, S_IRUGO,
		dump_powerregs_get, NULL);

static int ddr_ctl_read_reg;
#define MAX_DDR_CTL_REG 558

static ssize_t ddr_ctl_read_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	int val;

	val = readl(mnh_dev->ddraddr + (4 * ddr_ctl_read_reg));

	return sprintf(buf, "0x%08x", val);
}

static ssize_t ddr_ctl_read_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int val = 0;
	int ret;

	ret = kstrtoint(buf, 10, &val);
	if (ret < 0)
		return ret;

	dev_dbg(mnh_dev->dev, "%s: %d\n", __func__, val);

	if ((val < 0) || (val > MAX_DDR_CTL_REG))
		return -EINVAL;

	ddr_ctl_read_reg = val;

	return count;
}
DEVICE_ATTR_RW(ddr_ctl_read);

static ssize_t ddr_ctl_write_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf,
				   size_t count)
{
	int val1 = 0, val2 = 0;
	int ret;
	char *str;

	str = strsep((char **)&buf, ";");
	if (!str)
		return -EINVAL;

	ret = kstrtoint(str, 10, &val1);
	if (ret < 0)
		return ret;

	if ((val1 < 0) || (val1 > MAX_DDR_CTL_REG))
		return -EINVAL;

	ret = kstrtoint(buf, 0, &val2);
	if (ret < 0)
		return ret;

	dev_dbg(mnh_dev->dev, "%s: reg %d, data 0x%08x\n", __func__,
		val1, val2);

	writel(val2, mnh_dev->ddraddr + (4 * val1));

	return count;
}
DEVICE_ATTR_WO(ddr_ctl_write);

static struct attribute *freq_dev_attributes[] = {
	&dev_attr_cpu_freq.attr,
	&dev_attr_ipu_freq.attr,
	&dev_attr_lpddr_freq.attr,
	&dev_attr_ipu_clk_src.attr,
	&dev_attr_sys200.attr,
	&dev_attr_ipu_clock_gating.attr,
	&dev_attr_bypass_clock_gating.attr,
	&dev_attr_lpddr_lp.attr,
	&dev_attr_lpddr_sys200.attr,
	&dev_attr_lpddr_mrr4.attr,
	&dev_attr_dump_powerregs.attr,
	&dev_attr_ddr_ctl_read.attr,
	&dev_attr_ddr_ctl_write.attr,
	NULL
};

static struct attribute_group mnh_freq_cooling_group = {
	.name = "mnh_freq_cool",
	.attrs = freq_dev_attributes
};


static int init_sysfs(struct device *dev, struct kobject *sysfs_kobj)
{
	int ret;

	ret = sysfs_create_group(sysfs_kobj, &mnh_freq_cooling_group);
	if (ret) {
		dev_err(dev, "Failed to create sysfs\n");
		return -EINVAL;
	}

	return 0;
}

static void clean_sysfs(void)
{
	sysfs_remove_group(kernel_kobj, &mnh_freq_cooling_group);
}

/**
 * Initial handler for ddr interrupts
 */
static irqreturn_t mnh_pm_handle_ddr_irq(int irq, void *dev_id)
{
	u64 status = mnh_ddr_int_status();

	dev_dbg(mnh_dev->dev,
		"%s status=0x%llx\n", __func__, status);

	if (status & (1 << MR_READ_SBIT)) {
		mnh_ddr_clr_int_status_bit(MR_READ_SBIT);
		complete(&mnh_dev->ddr_mrr);
	}

	if (status & (1 << LP_CMD_SBIT)) {
		mnh_ddr_clr_int_status_bit(LP_CMD_SBIT);
		complete(&mnh_dev->ddr_lp_cmd);
	}

	status = mnh_ddr_int_status();

	if (status) {
		mnh_ddr_clr_int_status();
		dev_dbg(mnh_dev->dev,
			"%s unhandled status=0x%llx, but cleared.\n", __func__, status);
	}

	/* return interrupt handled */
	return IRQ_HANDLED;
}
int mnh_clk_init(struct platform_device *pdev, void __iomem *baseadress)
{
	int ret = 0, err = 0;
	struct resource *res;
	struct mnh_freq_cooling_device *tmp_mnh_dev;
	uint32_t refclk_gpio;

	dev_info(&pdev->dev, "mnh_freq_cooling_init\n");

	tmp_mnh_dev = devm_kzalloc(&pdev->dev, sizeof(*tmp_mnh_dev),
			GFP_KERNEL);
	if (!tmp_mnh_dev)
		return -ENOMEM;

	/* Set baseadress for SCU */
	tmp_mnh_dev->regs = baseadress;
	tmp_mnh_dev->dev = &pdev->dev;
	init_completion(&tmp_mnh_dev->ddr_mrr);
	tmp_mnh_dev->ddr_mrr4 = 0xDEADDEAD;
	init_completion(&tmp_mnh_dev->ddr_lp_cmd);
	// spin_lock_init(&tmp_mnh_dev->irqlock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(tmp_mnh_dev->dev, "cannot get platform resources\n");
		ret = -ENOENT;
		goto mnh_probe_err;
	}
	tmp_mnh_dev->ddraddr = ioremap_nocache(res->start, resource_size(res));
	if (!tmp_mnh_dev->ddraddr) {
		dev_err(tmp_mnh_dev->dev, "unable to remap resources\n");
		ret = -ENOMEM;
		goto mnh_probe_err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		dev_err(tmp_mnh_dev->dev, "cannot get platform resources\n");
		ret = -ENOENT;
		goto mnh_probe_err;
	}
	tmp_mnh_dev->cpuaddr = ioremap_nocache(res->start, resource_size(res));
	if (!tmp_mnh_dev->cpuaddr) {
		dev_err(tmp_mnh_dev->dev, "unable to remap resources\n");
		ret = -ENOMEM;
		goto mnh_probe_err;
	}

	/* Check refclk rate */
	err = device_property_read_u32(tmp_mnh_dev->dev, "refclk-gpio",
		&refclk_gpio);
	if (!err) {
		tmp_mnh_dev->refclk = mnh_freq_check_refclk(refclk_gpio);
	} else {
		pr_err("unable to read refclk-gpio\n");
		ret = -ENOMEM;
		goto mnh_probe_err;
	}
	tmp_mnh_dev->cpu_pllcfg = (const struct freq_reg_table*)&cpu_reg_tables[tmp_mnh_dev->refclk];
	tmp_mnh_dev->ipu_pllcfg = (const struct freq_reg_table*)&ipu_reg_tables[tmp_mnh_dev->refclk];

	mnh_dev = tmp_mnh_dev;
	mnh_dev->cpu_freq = mnh_cpu_freq_to_index();

	mnh_dev->ddr_irq = platform_get_irq(pdev, 0);
	dev_dbg(mnh_dev->dev, "Allocate ddr irq %d\n", mnh_dev->ddr_irq);
	err = request_irq(mnh_dev->ddr_irq, mnh_pm_handle_ddr_irq,
			  IRQF_SHARED, DEVICE_NAME, mnh_dev->dev);
	if (err) {
		dev_err(mnh_dev->dev, "Could not allocated ddr irq\n");
		ret = -EINVAL;
		goto mnh_probe_err;
	}

	spin_lock_init(&mnh_dev->reset_lock);

	init_sysfs(mnh_dev->dev, kernel_kobj);
	previous_refresh_rate = INIT_REFRESH_RATE;
	INIT_DELAYED_WORK(&mnh_ddr_adjust_refresh_work,
		mnh_ddr_adjust_refresh_worker);
	schedule_delayed_work(&mnh_ddr_adjust_refresh_work,
		msecs_to_jiffies(mnh_ddr_refresh_msec));
	mnh_clock_init_gating(1);

	return 0;

mnh_probe_err:
	if (tmp_mnh_dev->ddraddr)
		iounmap(&tmp_mnh_dev->ddraddr);
	return ret;
}

void mnh_clk_clean(struct device *dev)
{
	clean_sysfs();
}
