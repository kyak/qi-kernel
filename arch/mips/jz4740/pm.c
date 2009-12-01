/*
 * linux/arch/mips/jz4740/common/pm.c
 * 
 * JZ4740 Power Management Routines
 * 
 * Copyright (C) 2006 Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */

#include <linux/init.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <asm/mach-jz4740/regs.h>

extern void jz4740_intc_suspend(void);
extern void jz4740_intc_resume(void);
extern void jz_gpiolib_suspend(void);
extern void jz_gpiolib_resume(void);

static int jz_pm_enter(suspend_state_t state)
{
	unsigned long delta;
	unsigned long nfcsr = REG_EMC_NFCSR;
	uint32_t scr = REG_CPM_SCR;

	/* Preserve current time */
	delta = xtime.tv_sec - REG_RTC_RSR;

    /* Disable nand flash */
	REG_EMC_NFCSR = ~0xff;

 	udelay(100);

    /*stop udc and usb*/
	REG_CPM_SCR &= ~( 1<<6 | 1<<7);
	REG_CPM_SCR |= 0<<6 | 1<<7;

    jz_gpiolib_suspend();
    jz4740_intc_suspend();

 	/* Enter SLEEP mode */
	REG_CPM_LCR &= ~CPM_LCR_LPM_MASK;
	REG_CPM_LCR |= CPM_LCR_LPM_SLEEP;
	__asm__(".set\tmips3\n\t"
		"wait\n\t"
		".set\tmips0");

	/* Restore to IDLE mode */
	REG_CPM_LCR &= ~CPM_LCR_LPM_MASK;
	REG_CPM_LCR |= CPM_LCR_LPM_IDLE;

    /* Restore nand flash control register */
	REG_EMC_NFCSR = nfcsr;

    jz4740_intc_resume();
    jz_gpiolib_resume();

	/* Restore sleep control register */
	REG_CPM_SCR = scr;

	/* Restore current time */
	xtime.tv_sec = REG_RTC_RSR + delta;

	return 0;
}

static struct platform_suspend_ops jz_pm_ops = {
	.valid		= suspend_valid_only_mem,
	.enter		= jz_pm_enter,
};

/*
 * Initialize power interface
 */
int __init jz_pm_init(void)
{
	suspend_set_ops(&jz_pm_ops);
	return 0;

}
late_initcall(jz_pm_init);
