/*
 *  Copyright (c) 2006-2007, Ingenic Semiconductor Inc.
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *  Copyright (c) 2010, Ulrich Hecht <ulrich.hecht@gmail.com>
 *  Copyright (c) 2010, Maarten ter Huurne <maarten@treewalker.org>
 *  JZ4740 SoC clock support
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General	 Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>

#include <asm/mach-jz4740/clock.h>
#include <asm/mach-jz4740/base.h>

#include "clock.h"

#define JZ_REG_CLOCK_CTRL	0x00
#define JZ_REG_CLOCK_LOW_POWER	0x04
#define JZ_REG_CLOCK_PLL	0x10
#define JZ_REG_CLOCK_GATE	0x20
#define JZ_REG_CLOCK_SLEEP_CTRL 0x24
#define JZ_REG_CLOCK_I2S	0x60
#define JZ_REG_CLOCK_LCD	0x64
#define JZ_REG_CLOCK_MMC	0x68
#define JZ_REG_CLOCK_UHC	0x6C
#define JZ_REG_CLOCK_SPI	0x74

#define JZ_CLOCK_CTRL_KO_ENABLE		30
#define JZ_CLOCK_CTRL_UDC_SRC_PLL	BIT(29)
#define JZ_CLOCK_CTRL_CHANGE_ENABLE	BIT(22)
#define JZ_CLOCK_CTRL_PLL_HALF		BIT(21)
#define JZ_CLOCK_CTRL_UDIV_OFFSET	23
#define JZ_CLOCK_CTRL_LDIV_OFFSET	16
#define JZ_CLOCK_CTRL_MDIV_OFFSET	12
#define JZ_CLOCK_CTRL_PDIV_OFFSET	 8
#define JZ_CLOCK_CTRL_HDIV_OFFSET	 4
#define JZ_CLOCK_CTRL_CDIV_OFFSET	 0
#define JZ_CLOCK_CTRL_UDIV_MASK		(0x3f << JZ_CLOCK_CTRL_UDIV_OFFSET)
#define JZ_CLOCK_CTRL_MDIV_MASK		(0x0f << JZ_CLOCK_CTRL_MDIV_OFFSET)
#define JZ_CLOCK_CTRL_PDIV_MASK		(0x0f << JZ_CLOCK_CTRL_PDIV_OFFSET)
#define JZ_CLOCK_CTRL_HDIV_MASK		(0x0f << JZ_CLOCK_CTRL_HDIV_OFFSET)
#define JZ_CLOCK_CTRL_CDIV_MASK		(0x0f << JZ_CLOCK_CTRL_CDIV_OFFSET)

#define JZ_CLOCK_GATE_UART0	0
#define JZ_CLOCK_GATE_TCU	1
#define JZ_CLOCK_GATE_RTC	2
#define JZ_CLOCK_GATE_I2C	3
#define JZ_CLOCK_GATE_SPI	4
#define JZ_CLOCK_GATE_AIC	5
#define JZ_CLOCK_GATE_I2S	6
#define JZ_CLOCK_GATE_MMC	7
#define JZ_CLOCK_GATE_ADC	8
#define JZ_CLOCK_GATE_CIM	9
#define JZ_CLOCK_GATE_LCD	10
#define JZ_CLOCK_GATE_UDC	11
#define JZ_CLOCK_GATE_DMAC	12
#define JZ_CLOCK_GATE_IPU	13
#define JZ_CLOCK_GATE_UHC	14
#define JZ_CLOCK_GATE_UART1	15

#define JZ_CLOCK_SPI_SRC_PLL		31

#define JZ_CLOCK_PLL_M_MASK		0x01ff
#define JZ_CLOCK_PLL_N_MASK		0x001f
#define JZ_CLOCK_PLL_OD_MASK		0x0003
#define JZ_CLOCK_PLL_STABLE		BIT(10)
#define JZ_CLOCK_PLL_BYPASS		BIT(9)
#define JZ_CLOCK_PLL_ENABLED		BIT(8)
#define JZ_CLOCK_PLL_STABLIZE_MASK	0x000f
#define JZ_CLOCK_PLL_M_OFFSET		23
#define JZ_CLOCK_PLL_N_OFFSET		18
#define JZ_CLOCK_PLL_OD_OFFSET		16
#define JZ_CLOCK_PLL_STABILIZE_OFFSET	0

#define JZ_CLOCK_LOW_POWER_MODE_DOZE BIT(2)
#define JZ_CLOCK_LOW_POWER_MODE_SLEEP BIT(0)

#define JZ_CLOCK_SLEEP_CTRL_SUSPEND_UHC BIT(7)
#define JZ_CLOCK_SLEEP_CTRL_ENABLE_UDC 6

#define JZ_REG_EMC_RTCNT	0x88
#define JZ_REG_EMC_RTCOR	0x8C

static void __iomem *jz4740_clk_base;
static spinlock_t jz4740_clk_lock;

static void __iomem *jz_emc_base;

static uint32_t jz_clk_reg_read(int reg)
{
	return readl(jz4740_clk_base + reg);
}
/*
static void jz_clk_reg_write_mask(int reg, uint32_t val, uint32_t mask)
{
	uint32_t val2;

	spin_lock(&jz4740_clk_lock);
	val2 = readl(jz4740_clk_base + reg);
	val2 &= ~mask;
	val2 |= val;
	writel(val2, jz4740_clk_base + reg);
	spin_unlock(&jz4740_clk_lock);
}
*/
static void jz_clk_reg_set_bits(int reg, uint32_t mask)
{
	uint32_t val;

	spin_lock(&jz4740_clk_lock);
	val = readl(jz4740_clk_base + reg);
	val |= mask;
	writel(val, jz4740_clk_base + reg);
	spin_unlock(&jz4740_clk_lock);
}

static void jz_clk_reg_clear_bits(int reg, uint32_t mask)
{
	uint32_t val;

	spin_lock(&jz4740_clk_lock);
	val = readl(jz4740_clk_base + reg);
	val &= ~mask;
	writel(val, jz4740_clk_base + reg);
	spin_unlock(&jz4740_clk_lock);
}

static unsigned long jz_clk_pll_calc_rate(unsigned long parent_rate,
	unsigned int in_div, unsigned int feedback, unsigned int out_div)
{
	return ((parent_rate / in_div) * feedback) / out_div;
}

static const int pllno[] = {1, 2, 2, 4};

static unsigned long jz4740_clk_pll_recalc_rate(struct clk_hw *hwclk,
	unsigned long parent_rate)
{
	uint32_t val;
	unsigned int in_div, feedback, out_div;

	val = jz_clk_reg_read(JZ_REG_CLOCK_PLL);

	if (val & JZ_CLOCK_PLL_BYPASS)
		return parent_rate;

	feedback = ((val >> 23) & 0x1ff) + 2;
	in_div = ((val >> 18) & 0x1f) + 2;
	out_div = pllno[(val >> 16) & 0x3];

	printk("recalc pll: %lu\n",  jz_clk_pll_calc_rate(parent_rate, in_div,
	feedback, out_div));

	return jz_clk_pll_calc_rate(parent_rate, in_div, feedback, out_div);
}

static const struct clk_ops jz4740_clk_pll_ops = {
    .recalc_rate = jz4740_clk_pll_recalc_rate,
};

#if 0

static void jz_clk_pll_calc_dividers(unsigned long rate,
	unsigned int *in_div, unsigned int *feedback, unsigned int *out_div)
{
	unsigned int target;

	/* The frequency after the input divider must be between 1 and 15 MHz.
	   The highest divider yields the best resolution. */
	*in_div = jz_clk_ext.rate / 1000000;
	if (*in_div >= 34)
		*in_div = 33;

	/* The frequency before the output divider must be between 100 and
	   500 MHz. The lowest target rate is more energy efficient. */
	if (rate < 25000000) {
		*out_div = 4;
		target = 25000000 * 4;
	} else if (rate <= 50000000) {
		*out_div = 4;
		target = rate * 4;
	} else if (rate <= 100000000) {
		*out_div = 2;
		target = rate * 2;
	} else if (rate <= 500000000) {
		*out_div = 1;
		target = rate;
	} else {
		*out_div = 1;
		target = 500000000;
	}

	/* Compute the feedback divider.
	   Since the divided input is at least 1 MHz and the target frequency
	   at most 500 MHz, the feedback will be at most 500 and will therefore
	   always fit in the 9-bit register.
	   Similarly, the divided input is at most 15 MHz and the target
	   frequency at least 100 MHz, so the feedback will be at least 6
	   where the minimum supported value is 2. */
	*feedback = ((target / 1000) * *in_div) / (jz_clk_ext.rate / 1000);
}

static unsigned long jz_clk_pll_round_rate(struct clk *clk, unsigned long rate)
{
	unsigned int in_div, feedback, out_div;
	/* The PLL frequency must be a multiple of 24 MHz, since the LCD pixel
	 * clock must be exactly 12 MHz for the TV-out to work.
	 * TODO: A multiple of 12 MHz for the PLL would work if the PLL would
	 *       not be divided by 2 before being passed to the set of derived
	 *       clocks that includes the LCD pixel clock.
	 * TODO: Systemwide decisions like this should be made by the board
	 *       support code, so add some kind of hook for that.
	 */
	unsigned long rate24 = (rate / 24000000) * 24000000;

	jz_clk_pll_calc_dividers(rate24, &in_div, &feedback, &out_div);
	return jz_clk_pll_calc_rate(in_div, feedback, out_div);
}


#define SDRAM_TREF 15625   /* Refresh period: 4096 refresh cycles/64ms */

static void sdram_set_pll(unsigned int pllin)
{
	unsigned int ns, sdramclock;

	ns = 1000000000 / pllin;
	sdramclock = (SDRAM_TREF / ns) / 64 + 1;
	if (sdramclock > 0xff) sdramclock = 0xff;
	/* Set refresh registers */
	writew(sdramclock, jz_emc_base + JZ_REG_EMC_RTCOR);
	writew(sdramclock, jz_emc_base + JZ_REG_EMC_RTCNT);
}

static int jz_clk_pll_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned int ctrl, plcr1;
	unsigned int feedback, in_div, out_div, pllout, pllout2;

	jz_clk_pll_calc_dividers(rate, &in_div, &feedback, &out_div);

	ctrl = jz_clk_reg_read(JZ_REG_CLOCK_CTRL);
	pllout = jz_clk_pll_calc_rate(in_div, feedback, out_div);
	pllout2 = (ctrl & JZ_CLOCK_CTRL_PLL_HALF) ? pllout : (pllout / 2);

	/* Init UHC clock */
	writel(pllout2 / 48000000 - 1, jz4740_clk_base + JZ_REG_CLOCK_UHC);

	plcr1 = ((feedback - 2) << JZ_CLOCK_PLL_M_OFFSET) |
		((in_div - 2) << JZ_CLOCK_PLL_N_OFFSET) |
		((out_div - 1) << JZ_CLOCK_PLL_OD_OFFSET) |
		(0x20 << JZ_CLOCK_PLL_STABILIZE_OFFSET) |
		JZ_CLOCK_PLL_ENABLED;

	sdram_set_pll(pllout);

	/* LCD pixclock */
	writel(pllout2 / 12000000 - 1, jz4740_clk_base + JZ_REG_CLOCK_LCD);

	/* configure PLL */
	__asm__ __volatile__(
		".set noreorder\n\t"
		".align 5\n"
		"sw %1,0(%0)\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		".set reorder\n\t"
		:
		: "r" (jz4740_clk_base + JZ_REG_CLOCK_PLL), "r" (plcr1));

	/* MtH: For some reason the MSC will have problems if this flag is not
	        restored, even though the MSC is supposedly the only divider
	        that is not affected by this flag. */
	jz_clk_reg_set_bits(JZ_REG_CLOCK_CTRL, JZ_CLOCK_CTRL_CHANGE_ENABLE);

	return 0;
}

static const unsigned int jz_clk_main_divs_inv[] = {
	-1,  0,  1,  2,  3, -1,  4, -1,  5, -1, -1, -1,  6, -1, -1, -1,
	 7, -1, -1, -1, -1, -1, -1, -1,  8, -1, -1, -1, -1, -1, -1, -1,
	 9
};

static struct main_clk jz_clk_cpu;

int clk_main_set_dividers(bool immediate, unsigned int cdiv, unsigned int hdiv,
			  unsigned int mdiv, unsigned int pdiv)
{
	unsigned int cdiv_enc, hdiv_enc, mdiv_enc, pdiv_enc;
	unsigned int ctrl;
	unsigned int tmp, wait;

	if (cdiv >= ARRAY_SIZE(jz_clk_main_divs_inv) ||
	    hdiv >= ARRAY_SIZE(jz_clk_main_divs_inv) ||
	    mdiv >= ARRAY_SIZE(jz_clk_main_divs_inv) ||
	    pdiv >= ARRAY_SIZE(jz_clk_main_divs_inv))
		return -EINVAL;
	cdiv_enc = jz_clk_main_divs_inv[cdiv];
	hdiv_enc = jz_clk_main_divs_inv[hdiv];
	mdiv_enc = jz_clk_main_divs_inv[mdiv];
	pdiv_enc = jz_clk_main_divs_inv[pdiv];
	if (cdiv_enc == (unsigned int)-1 ||
	    hdiv_enc == (unsigned int)-1 ||
	    mdiv_enc == (unsigned int)-1 ||
	    pdiv_enc == (unsigned int)-1)
		return -EINVAL;

	ctrl = jz_clk_reg_read(JZ_REG_CLOCK_CTRL);
	ctrl &= ~(JZ_CLOCK_CTRL_CHANGE_ENABLE |
		  JZ_CLOCK_CTRL_CDIV_MASK | JZ_CLOCK_CTRL_HDIV_MASK |
		  JZ_CLOCK_CTRL_MDIV_MASK | JZ_CLOCK_CTRL_PDIV_MASK);
	if (immediate) ctrl |= JZ_CLOCK_CTRL_CHANGE_ENABLE;
	ctrl |= (cdiv_enc << JZ_CLOCK_CTRL_CDIV_OFFSET) |
		(hdiv_enc << JZ_CLOCK_CTRL_HDIV_OFFSET) |
		(mdiv_enc << JZ_CLOCK_CTRL_MDIV_OFFSET) |
		(pdiv_enc << JZ_CLOCK_CTRL_PDIV_OFFSET);

	/* set dividers */
	/* delay loops lifted from the old Ingenic cpufreq driver */
	wait = ((clk_get_rate(&jz_clk_cpu.clk) / 1000000) * 500) / 1000;
	__asm__ __volatile__(
		".set noreorder\n\t"
		".align 5\n"
		"sw %2,0(%1)\n\t"
		"li %0,0\n\t"
		"1:\n\t"
		"bne %0,%3,1b\n\t"
		"addi %0, 1\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		".set reorder\n\t"
		: "=r" (tmp)
		: "r" (jz4740_clk_base + JZ_REG_CLOCK_CTRL), "r" (ctrl),
		  "r" (wait));

	return 0;
}
EXPORT_SYMBOL_GPL(clk_main_set_dividers);

#endif

static const struct clk_div_table jz4740_clk_main_divs[] = {
    { 0, 1 },
    { 1, 2 },
    { 2, 3 },
    { 3, 4 },
    { 4, 6 },
    { 5, 7 },
    { 6, 12 },
    { 7, 16 },
    { 8, 24 },
    { 9, 32 },
    { },
};

static struct clk *jz4740_register_main_clock(const char *name,
	unsigned int div_shift)
{
    return clk_register_divider_table(NULL, name, "PLL", 0,
	    jz4740_clk_base + JZ_REG_CLOCK_CTRL, div_shift, 4, 0,
	    jz4740_clk_main_divs, &jz4740_clk_lock);
}

static struct clk *jz4740_register_peripheral_clock(const char *name,
	const char *bypass_name, unsigned int mux_reg, unsigned int mux_shift,
	unsigned int div_reg, unsigned int div_shift, unsigned int div_width,
	unsigned int gate_reg, unsigned int gate_bit)
{
	char gate_name[20], mux_name[20], div_name[20];
	const char *mux_parents[2];
	char *gate_parent;
	unsigned int flags;
	struct clk *clk;

	snprintf(div_name, sizeof(div_name), "%s-div", name);
	snprintf(mux_name, sizeof(mux_name), "%s-mux", name);
	snprintf(gate_name, sizeof(gate_name), "%s-gate", name);

	clk = clk_register_divider(NULL, div_name, "PERIP CLK", 0,
			jz4740_clk_base + div_reg, div_shift, div_width, 0,
			&jz4740_clk_lock);

	if (bypass_name) {
		mux_parents[0] = bypass_name;
		mux_parents[1] = div_name;
		clk = clk_register_mux(NULL, mux_name, mux_parents, 2,
			CLK_SET_RATE_PARENT,
			jz4740_clk_base + mux_reg, mux_shift, 1, 0,
			&jz4740_clk_lock);
		gate_parent = mux_name;
	} else {
		gate_parent = div_name;
	}

	if (gate_reg) {
		if (gate_reg == JZ_REG_CLOCK_GATE)
			flags = CLK_GATE_SET_TO_DISABLE;
		else
			flags = 0;
		clk = clk_register_gate(NULL, gate_name, gate_parent,
			CLK_SET_RATE_PARENT,
			jz4740_clk_base + gate_reg, gate_bit,
			flags, &jz4740_clk_lock);
	}

	return clk;
}

static struct clk *jz4740_register_gate(const char *name,
	const char *parent_name, u8 gate_bit)
{
	return clk_register_gate(NULL, name, parent_name, 0,
		jz4740_clk_base + JZ_REG_CLOCK_GATE, gate_bit,
		CLK_GATE_SET_TO_DISABLE, &jz4740_clk_lock);
}

static const struct clk_div_table jz4740_perip_clk_div_table[] = {
    { 0, 2 },
    { 1, 1 },
    { },
};

static const char *pll_clk_parent = "EXCLK";

static struct clk_init_data pll_clk_init = {
	.name = "PLL",
	.ops = &jz4740_clk_pll_ops,
	.flags = 0,
	.parent_names = &pll_clk_parent,
	.num_parents = 1,
};

static struct clk_hw pll_clk = {
	.init = &pll_clk_init,
};

static void __init clk_register_clks(void)
{
	struct clk *clk;

	clk = clk_register_fixed_rate(NULL, "EXCLK", NULL, CLK_IS_ROOT,
		jz4740_clock_bdata.ext_rate);
	clk_register_clkdev(clk, "ext", "jz4740-pwm");

	clk_register(NULL, &pll_clk);

	clk_register_fixed_rate(NULL, "RTCLK_XI", NULL, CLK_IS_ROOT,
		jz4740_clock_bdata.rtc_rate);

	jz4740_register_main_clock("MCLK", JZ_CLOCK_CTRL_MDIV_OFFSET);
	jz4740_register_main_clock("PCLK", JZ_CLOCK_CTRL_PDIV_OFFSET);
	jz4740_register_main_clock("HCLK", JZ_CLOCK_CTRL_HDIV_OFFSET);
	jz4740_register_main_clock("CCLK", JZ_CLOCK_CTRL_CDIV_OFFSET);

	clk = jz4740_register_gate("rtc", "RTCLK_XI", JZ_CLOCK_GATE_RTC);
	clk_register_clkdev(clk, "rtc", "jz4740-rtc");
	clk_register_clkdev(clk, "rtc", "jz4740-wdt");
	clk_register_clkdev(clk, "rtc", "10003000.rtc");

#if 0
	jz4740_register_gate("uart0", "EXCLK", JZ_CLOCK_GATE_UART0);
	jz4740_register_gate("uart1", "EXCLK", JZ_CLOCK_GATE_UART1);

	clk_register_gate(NULL, "ko", "MCLK", 0,
		jz4740_clk_base + JZ_REG_CLOCK_CTRL, 30,
		0, &jz4740_clk_lock);
#endif

	jz4740_register_gate("ipu", "HCLK", JZ_CLOCK_GATE_IPU);

	clk = jz4740_register_gate("dma", "HCLK", JZ_CLOCK_GATE_DMAC);
	clk_register_clkdev(clk, "dma", "jz4740-dma");

	clk = jz4740_register_gate("adc", "EXCLK", JZ_CLOCK_GATE_ADC);
	clk_register_clkdev(clk, "adc", "jz4740-adc");

	jz4740_register_gate("i2c", "EXCLK", JZ_CLOCK_GATE_I2C);

	clk = jz4740_register_gate("aic", "EXCLK", JZ_CLOCK_GATE_AIC);
	clk_register_clkdev(clk, "aic", "jz4740-i2s");
	clk_register_clkdev(clk, "aic", "10020000.aic");

	clk_register_divider_table(NULL, "PERIP CLK", "PLL", 0,
		jz4740_clk_base + JZ_REG_CLOCK_CTRL, 21, 1, 0,
		jz4740_perip_clk_div_table, &jz4740_clk_lock);

	clk = jz4740_register_peripheral_clock("i2s", "EXCLK",
		JZ_REG_CLOCK_CTRL, 31,
		JZ_REG_CLOCK_I2S, 0, 9,
		JZ_REG_CLOCK_GATE, JZ_CLOCK_GATE_I2S);
	clk_register_clkdev(clk, "i2s", "jz4740-i2s");
	clk_register_clkdev(clk, "i2s", "10020000.aic");

	jz4740_register_peripheral_clock("spi", "EXCLK",
		JZ_REG_CLOCK_SPI, 31,
		JZ_REG_CLOCK_SPI, 0, 4,
		JZ_REG_CLOCK_GATE, JZ_CLOCK_GATE_SPI);

	clk = jz4740_register_peripheral_clock("lcd_pclk", NULL, 0, 0,
		JZ_REG_CLOCK_LCD, 0, 11,
		0, 0);
	clk_register_clkdev(clk, "lcd_pclk", "jz4740-fb");

	clk = jz4740_register_peripheral_clock("lcd", NULL, 0, 0,
		JZ_REG_CLOCK_CTRL, JZ_CLOCK_CTRL_LDIV_OFFSET, 5,
		JZ_REG_CLOCK_GATE, JZ_CLOCK_GATE_LCD);
	clk_register_clkdev(clk, "lcd", "jz4740-fb");

	clk = jz4740_register_peripheral_clock("mmc", NULL, 0, 0,
		JZ_REG_CLOCK_MMC, 0, 5,
		JZ_REG_CLOCK_GATE, JZ_CLOCK_GATE_MMC);
	clk_register_clkdev(clk, "mmc", "jz4740-mmc.0");
	clk_register_clkdev(clk, "mmc", "10021000.msc");

	clk = jz4740_register_peripheral_clock("uhc", NULL, 0, 0,
		JZ_REG_CLOCK_UHC, 0, 4,
		JZ_REG_CLOCK_GATE, JZ_CLOCK_GATE_UHC);
	clk_register_clkdev(clk, "uhc", "jz4740-ohci");

	clk = jz4740_register_peripheral_clock("udc", "EXCLK", JZ_REG_CLOCK_CTRL, 29,
		JZ_REG_CLOCK_CTRL, JZ_CLOCK_CTRL_UDIV_OFFSET, 6,
		JZ_REG_CLOCK_SLEEP_CTRL, JZ_CLOCK_SLEEP_CTRL_ENABLE_UDC);
	clk_register_clkdev(clk, "udc", "jz-udc");
}

void jz4740_clock_set_wait_mode(enum jz4740_wait_mode mode)
{
	switch (mode) {
	case JZ4740_WAIT_MODE_IDLE:
		jz_clk_reg_clear_bits(JZ_REG_CLOCK_LOW_POWER, JZ_CLOCK_LOW_POWER_MODE_SLEEP);
		break;
	case JZ4740_WAIT_MODE_SLEEP:
		jz_clk_reg_set_bits(JZ_REG_CLOCK_LOW_POWER, JZ_CLOCK_LOW_POWER_MODE_SLEEP);
		break;
	}
}

void jz4740_clock_udc_disable_auto_suspend(void)
{
	jz_clk_reg_clear_bits(JZ_REG_CLOCK_GATE, BIT(JZ_CLOCK_GATE_UDC));
}
EXPORT_SYMBOL_GPL(jz4740_clock_udc_disable_auto_suspend);

void jz4740_clock_udc_enable_auto_suspend(void)
{
	jz_clk_reg_set_bits(JZ_REG_CLOCK_GATE, BIT(JZ_CLOCK_GATE_UDC));
}
EXPORT_SYMBOL_GPL(jz4740_clock_udc_enable_auto_suspend);

void jz4740_clock_suspend(void)
{
	jz_clk_reg_set_bits(JZ_REG_CLOCK_GATE,
		JZ_CLOCK_GATE_TCU | JZ_CLOCK_GATE_DMAC | JZ_CLOCK_GATE_UART0);

	jz_clk_reg_clear_bits(JZ_REG_CLOCK_PLL, JZ_CLOCK_PLL_ENABLED);
}

void jz4740_clock_resume(void)
{
	uint32_t pll;

	jz_clk_reg_set_bits(JZ_REG_CLOCK_PLL, JZ_CLOCK_PLL_ENABLED);

	do {
		pll = jz_clk_reg_read(JZ_REG_CLOCK_PLL);
	} while (!(pll & JZ_CLOCK_PLL_STABLE));

	jz_clk_reg_clear_bits(JZ_REG_CLOCK_GATE,
		JZ_CLOCK_GATE_TCU | JZ_CLOCK_GATE_DMAC | JZ_CLOCK_GATE_UART0);
}

static int __init jz4740_clock_init(void)
{
	jz4740_clk_base = ioremap(JZ4740_CPM_BASE_ADDR, 0x100);
	if (!jz4740_clk_base)
		return -EBUSY;

	jz_emc_base = ioremap(JZ4740_EMC_BASE_ADDR, 0x100);
	if (!jz_emc_base)
		return -EBUSY;

	spin_lock_init(&jz4740_clk_lock);

	clk_register_clks();

	return 0;
}
arch_initcall(jz4740_clock_init);
