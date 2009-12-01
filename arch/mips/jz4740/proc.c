/*
 * linux/arch/mips/jz4740/proc.c
 * 
 * /proc/jz/ procfs for jz4740 on-chip modules.
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/page-flags.h>
#include <asm/uaccess.h>
#include <asm/pgtable.h>
#include <asm/mach-jz4740/regs.h>
#include <asm/mach-jz4740/clock.h>
#include <asm/mach-jz4740/ops.h>

//#define DEBUG 1
#undef DEBUG


struct proc_dir_entry *proc_jz_root;


/*
 * EMC Modules
 */
static int emc_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;

	len += sprintf (page+len, "SMCR(0-5): 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n", REG_EMC_SMCR0, REG_EMC_SMCR1, REG_EMC_SMCR2, REG_EMC_SMCR3, REG_EMC_SMCR4);
	len += sprintf (page+len, "SACR(0-5): 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n", REG_EMC_SACR0, REG_EMC_SACR1, REG_EMC_SACR2, REG_EMC_SACR3, REG_EMC_SACR4);
	len += sprintf (page+len, "DMCR:      0x%08x\n", REG_EMC_DMCR);
	len += sprintf (page+len, "RTCSR:     0x%04x\n", REG_EMC_RTCSR);
	len += sprintf (page+len, "RTCOR:     0x%04x\n", REG_EMC_RTCOR);
	return len;
}

/* 
 * Power Manager Module
 */
static int pmc_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;
	unsigned long lcr = REG_CPM_LCR;
	unsigned long clkgr = REG_CPM_CLKGR;

	len += sprintf (page+len, "Low Power Mode : %s\n", 
			((lcr & CPM_LCR_LPM_MASK) == (CPM_LCR_LPM_IDLE)) ?
			"IDLE" : (((lcr & CPM_LCR_LPM_MASK) == (CPM_LCR_LPM_SLEEP)) ? 
				  "SLEEP" : "HIBERNATE"));
	len += sprintf (page+len, "Doze Mode      : %s\n", 
			(lcr & CPM_LCR_DOZE_ON) ? "on" : "off");
	if (lcr & CPM_LCR_DOZE_ON)
		len += sprintf (page+len, "     duty      : %d\n", (int)((lcr & CPM_LCR_DOZE_DUTY_MASK) >> CPM_LCR_DOZE_DUTY_BIT));
	len += sprintf (page+len, "IPU            : %s\n",
			(clkgr & CPM_CLKGR_IPU) ? "stopped" : "running");
	len += sprintf (page+len, "DMAC           : %s\n",
			(clkgr & CPM_CLKGR_DMAC) ? "stopped" : "running");
	len += sprintf (page+len, "UHC            : %s\n",
			(clkgr & CPM_CLKGR_UHC) ? "stopped" : "running");
	len += sprintf (page+len, "UDC            : %s\n",
			(clkgr & CPM_CLKGR_UDC) ? "stopped" : "running");
	len += sprintf (page+len, "LCD            : %s\n",
			(clkgr & CPM_CLKGR_LCD) ? "stopped" : "running");
	len += sprintf (page+len, "CIM            : %s\n",
			(clkgr & CPM_CLKGR_CIM) ? "stopped" : "running");
	len += sprintf (page+len, "SADC           : %s\n",
			(clkgr & CPM_CLKGR_SADC) ? "stopped" : "running");
	len += sprintf (page+len, "MSC            : %s\n",
			(clkgr & CPM_CLKGR_MSC) ? "stopped" : "running");
	len += sprintf (page+len, "AIC1           : %s\n",
			(clkgr & CPM_CLKGR_AIC1) ? "stopped" : "running");
	len += sprintf (page+len, "AIC2           : %s\n",
			(clkgr & CPM_CLKGR_AIC2) ? "stopped" : "running");
	len += sprintf (page+len, "SSI            : %s\n",
			(clkgr & CPM_CLKGR_SSI) ? "stopped" : "running");
	len += sprintf (page+len, "I2C            : %s\n",
			(clkgr & CPM_CLKGR_I2C) ? "stopped" : "running");
	len += sprintf (page+len, "RTC            : %s\n",
			(clkgr & CPM_CLKGR_RTC) ? "stopped" : "running");
	len += sprintf (page+len, "TCU            : %s\n",
			(clkgr & CPM_CLKGR_TCU) ? "stopped" : "running");
	len += sprintf (page+len, "UART1          : %s\n",
			(clkgr & CPM_CLKGR_UART1) ? "stopped" : "running");
	len += sprintf (page+len, "UART0          : %s\n",
			(clkgr & CPM_CLKGR_UART0) ? "stopped" : "running");
	return len;
}

static int pmc_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	REG_CPM_CLKGR = simple_strtoul(buffer, 0, 16);
	return count;
}

/*
 * Clock Generation Module
 */
#define TO_MHZ(x) (x/1000000),(x%1000000)/10000
#define TO_KHZ(x) (x/1000),(x%1000)/10

static int cgm_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;
	unsigned int cppcr = REG_CPM_CPPCR;  /* PLL Control Register */
	unsigned int cpccr = REG_CPM_CPCCR;  /* Clock Control Register */
	unsigned int div[] = {1, 2, 3, 4, 6, 8, 12, 16, 24, 32};
	unsigned int od[4] = {1, 2, 2, 4};

	len += sprintf (page+len, "CPPCR          : 0x%08x\n", cppcr);
	len += sprintf (page+len, "CPCCR          : 0x%08x\n", cpccr);
	len += sprintf (page+len, "PLL            : %s\n", 
			(cppcr & CPM_CPPCR_PLLEN) ? "ON" : "OFF");
	len += sprintf (page+len, "m:n:o          : %d:%d:%d\n",
			__cpm_get_pllm() + 2,
			__cpm_get_plln() + 2,
			od[__cpm_get_pllod()]
		);
	len += sprintf (page+len, "C:H:M:P        : %d:%d:%d:%d\n", 
			div[__cpm_get_cdiv()],
			div[__cpm_get_hdiv()],
			div[__cpm_get_mdiv()],
			div[__cpm_get_pdiv()]
		);
	len += sprintf (page+len, "PLL Freq       : %3d.%02d MHz\n", TO_MHZ(__cpm_get_pllout()));
	len += sprintf (page+len, "CCLK           : %3d.%02d MHz\n", TO_MHZ(__cpm_get_cclk()));
	len += sprintf (page+len, "HCLK           : %3d.%02d MHz\n", TO_MHZ(__cpm_get_hclk()));
	len += sprintf (page+len, "MCLK           : %3d.%02d MHz\n", TO_MHZ(__cpm_get_mclk()));
	len += sprintf (page+len, "PCLK           : %3d.%02d MHz\n", TO_MHZ(__cpm_get_pclk()));
	len += sprintf (page+len, "LCDCLK         : %3d.%02d MHz\n", TO_MHZ(__cpm_get_lcdclk()));
	len += sprintf (page+len, "PIXCLK         : %3d.%02d KHz\n", TO_KHZ(__cpm_get_pixclk()));
	len += sprintf (page+len, "I2SCLK         : %3d.%02d MHz\n", TO_MHZ(__cpm_get_i2sclk()));
	len += sprintf (page+len, "USBCLK         : %3d.%02d MHz\n", TO_MHZ(__cpm_get_usbclk()));
	len += sprintf (page+len, "MSCCLK         : %3d.%02d MHz\n", TO_MHZ(__cpm_get_mscclk()));
	len += sprintf (page+len, "EXTALCLK       : %3d.%02d MHz\n", TO_MHZ(__cpm_get_extalclk()));
	len += sprintf (page+len, "RTCCLK         : %3d.%02d MHz\n", TO_MHZ(__cpm_get_rtcclk()));

	return len;
}

static int cgm_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	REG_CPM_CPCCR = simple_strtoul(buffer, 0, 16);
	return count;
}


extern void local_flush_tlb_all(void);

/* CP0 hazard avoidance. */
#define BARRIER __asm__ __volatile__(".set noreorder\n\t" \
				     "nop; nop; nop; nop; nop; nop;\n\t" \
				     ".set reorder\n\t")
void show_tlb(void)
{
#define ASID_MASK 0xFF

        unsigned long flags;
        unsigned int old_ctx;
	unsigned int entry;
	unsigned int entrylo0, entrylo1, entryhi;
	unsigned int pagemask;

	local_irq_save(flags);

	/* Save old context */
	old_ctx = (read_c0_entryhi() & 0xff);

	printk("TLB content:\n");
	entry = 0;
	while(entry < 32) {
		write_c0_index(entry);
		BARRIER;
		tlb_read();
		BARRIER;
		entryhi = read_c0_entryhi();
		entrylo0 = read_c0_entrylo0();
		entrylo1 = read_c0_entrylo1();
		pagemask = read_c0_pagemask();
		printk("%02d: ASID=%02d%s VA=0x%08x ", entry, entryhi & ASID_MASK, (entrylo0 & entrylo1 & 1) ? "(G)" : "   ", entryhi & ~ASID_MASK);
		printk("PA0=0x%08x C0=%x %s%s%s\n", (entrylo0>>6)<<12, (entrylo0>>3) & 7, (entrylo0 & 4) ? "Dirty " : "", (entrylo0 & 2) ? "Valid " : "Invalid ", (entrylo0 & 1) ? "Global" : "");
		printk("\t\t\t     PA1=0x%08x C1=%x %s%s%s\n", (entrylo1>>6)<<12, (entrylo1>>3) & 7, (entrylo1 & 4) ? "Dirty " : "", (entrylo1 & 2) ? "Valid " : "Invalid ", (entrylo1 & 1) ? "Global" : "");

		printk("\t\tpagemask=0x%08x", pagemask);
		printk("\tentryhi=0x%08x\n", entryhi);
		printk("\t\tentrylo0=0x%08x", entrylo0);
		printk("\tentrylo1=0x%08x\n", entrylo1);

		entry++;
	}
	BARRIER;
	write_c0_entryhi(old_ctx);

	local_irq_restore(flags);
}

/*
 * UDC hotplug
 */
#ifdef CONFIG_JZ_UDC_HOTPLUG
extern int jz_udc_active;	/* defined in drivers/char/jzchar/jz_udc_hotplug.c */
#endif

#ifndef GPIO_UDC_HOTPLUG
#define GPIO_UDC_HOTPLUG 86
#endif

static int udc_read_proc(char *page, char **start, off_t off,
			 int count, int *eof, void *data)
{
        int len = 0;

	if (__gpio_get_pin(GPIO_UDC_HOTPLUG)) {

#ifdef CONFIG_JZ_UDC_HOTPLUG

		/* Cable has connected, wait for disconnection. */
		__gpio_as_irq_fall_edge(GPIO_UDC_HOTPLUG);

		if (jz_udc_active)
			len += sprintf (page+len, "CONNECT_CABLE\n");
		else
			len += sprintf (page+len, "CONNECT_POWER\n");
#else
		len += sprintf (page+len, "CONNECT\n");
#endif
	}
	else {

#ifdef CONFIG_JZ_UDC_HOTPLUG
		/* Cable has disconnected, wait for connection. */
		__gpio_as_irq_rise_edge(GPIO_UDC_HOTPLUG);
#endif

		len += sprintf (page+len, "REMOVE\n");
	}
                                                                                                               
        return len;
}

/*
 * /proc/jz/xxx entry
 *
 */
static int __init jz_proc_init(void)
{
	struct proc_dir_entry *res;
	unsigned int virt_addr, i;

	proc_jz_root = proc_mkdir("jz", 0);

	/* External Memory Controller */
	res = create_proc_entry("emc", 0644, proc_jz_root);
	if (res) {
		res->read_proc = emc_read_proc;
		res->write_proc = NULL;
		res->data = NULL;
	}

	/* Power Management Controller */
	res = create_proc_entry("pmc", 0644, proc_jz_root);
	if (res) {
		res->read_proc = pmc_read_proc;
		res->write_proc = pmc_write_proc;
		res->data = NULL;
	}

	/* Clock Generation Module */
	res = create_proc_entry("cgm", 0644, proc_jz_root);
	if (res) {
		res->read_proc = cgm_read_proc;
		res->write_proc = cgm_write_proc;
		res->data = NULL;
	}

	/* udc hotplug */
	res = create_proc_entry("udc", 0644, proc_jz_root);
	if (res) {
		res->read_proc = udc_read_proc;
		res->write_proc = NULL;
		res->data = NULL;
	}

	return 0;
}

__initcall(jz_proc_init);
