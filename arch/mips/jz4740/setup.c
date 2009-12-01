/*
 * linux/arch/mips/jz4740/common/setup.c
 * 
 * JZ4740 common setup routines.
 * 
 * Copyright (C) 2006 Ingenic Semiconductor Inc.
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
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/ioport.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/irq.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>
#include <asm/pgtable.h>
#include <asm/time.h>
#include <asm/mach-jz4740/regs.h>
#include <asm/mach-jz4740/ops.h>
#include <asm/mach-jz4740/clock.h>
#include <asm/mach-jz4740/serial.h>

extern char * __init prom_getcmdline(void);
extern void __init jz_board_setup(void);
extern void jz_restart(char *);
extern void jz_halt(void);
extern void jz_power_off(void);
extern void jz_time_init(void);

static void __init sysclocks_setup(void)
{
#if 0
	printk("CPU clock: %dMHz, System clock: %dMHz, Peripheral clock: %dMHz, Memory clock: %dMHz\n",
	       (jz_clocks.cclk + 500000) / 1000000,
	       (jz_clocks.hclk + 500000) / 1000000,
	       (jz_clocks.pclk + 500000) / 1000000,
	       (jz_clocks.mclk + 500000) / 1000000);
#endif
}

static void __init soc_cpm_setup(void)
{
	/* Enable CKO to external memory */
	__cpm_enable_cko();

	/* CPU enters IDLE mode when executing 'wait' instruction */
	__cpm_idle_mode();

	/* Setup system clocks */
	sysclocks_setup();
}

static void __init soc_harb_setup(void)
{
//	__harb_set_priority(0x00);  /* CIM>LCD>DMA>ETH>PCI>USB>CBB */
//	__harb_set_priority(0x03);  /* LCD>CIM>DMA>ETH>PCI>USB>CBB */
//	__harb_set_priority(0x0a);  /* ETH>LCD>CIM>DMA>PCI>USB>CBB */
}

static void __init soc_emc_setup(void)
{
}

static void __init soc_dmac_setup(void)
{
	__dmac_enable_module();
}

static void __init jz_soc_setup(void)
{
	soc_cpm_setup();
	soc_harb_setup();
	soc_emc_setup();
	soc_dmac_setup();
}

static void __init jz_serial_setup(void)
{
#ifdef CONFIG_SERIAL_8250
	struct uart_port s;
	REG8(UART0_FCR) |= UARTFCR_UUE; /* enable UART module */
	memset(&s, 0, sizeof(s));
	s.flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST;
	s.iotype = SERIAL_IO_MEM;
	s.regshift = 2;
	s.uartclk = JZ_EXTAL;

	s.line = 0;
	s.membase = (u8 *)UART0_BASE;
	s.irq = JZ_IRQ_UART0;
	if (early_serial_setup(&s) != 0) {
		printk(KERN_ERR "Serial ttyS0 setup failed!\n");
	}

	s.line = 1;
	s.membase = (u8 *)UART1_BASE;
	s.irq = JZ_IRQ_UART1;
	if (early_serial_setup(&s) != 0) {
		printk(KERN_ERR "Serial ttyS1 setup failed!\n");
	}
#endif
}

void __init plat_mem_setup(void)
{
	char *argptr;

	argptr = prom_getcmdline();

	/* IO/MEM resources. Which will be the addtion value in `inX' and
	 * `outX' macros defined in asm/io.h */
	set_io_port_base(0);
	ioport_resource.start	= 0x00000000;
	ioport_resource.end	= 0xffffffff;
	iomem_resource.start	= 0x00000000;
	iomem_resource.end	= 0xffffffff;

	_machine_restart = jz_restart;
	_machine_halt = jz_halt;
	pm_power_off = jz_power_off;
	jz_soc_setup();
	jz_serial_setup();
}

