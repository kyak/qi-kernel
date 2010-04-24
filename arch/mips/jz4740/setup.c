/*
 *  Copyright (C) 2009-2010, Lars-Peter Clausen <lars@metafoo.de>
 *  JZ4740 setup code
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */


#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>

#include <asm/mach-jz4740/base.h>
#include <asm/mach-jz4740/clock.h>
#include <asm/mach-jz4740/serial.h>

#include "reset.h"
#include "clock.h"

static void __init jz4740_serial_setup(void)
{
#ifdef CONFIG_SERIAL_8250
	struct uart_port s;
	memset(&s, 0, sizeof(s));
	s.flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST;
	s.iotype = SERIAL_IO_MEM;
	s.regshift = 2;
	s.uartclk = jz4740_clock_bdata.ext_rate;

	s.line = 0;
	s.membase = (u8 *)JZ4740_UART0_BASE_ADDR;
	s.irq = JZ4740_IRQ_UART0;
	if (early_serial_setup(&s) != 0) {
		printk(KERN_ERR "Serial ttyS0 setup failed!\n");
	}

	s.line = 1;
	s.membase = (u8 *)JZ4740_UART1_BASE_ADDR;
	s.irq = JZ4740_IRQ_UART1;
	if (early_serial_setup(&s) != 0) {
		printk(KERN_ERR "Serial ttyS1 setup failed!\n");
	}
#endif
}
void __init plat_mem_setup(void)
{
	jz4740_reset_init();
	jz4740_serial_setup();
}

const char *get_system_type(void)
{
	return "JZ4740";
}
