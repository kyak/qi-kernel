/*
 * linux/arch/mips/jz4740/irq.c
 *
 * JZ4740 interrupt routines.
 *
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 * Author: <lhhuang@ingenic.cn>
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */
#include <asm/mach-jz4740/irq.h>
#include <linux/irq.h>

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/timex.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/mipsregs.h>
#include <asm/irq_cpu.h>

static void __iomem *jz_intc_base;
static uint32_t jz_intc_wakeup;
static uint32_t jz_intc_saved;

#define JZ_REG_BASE_INTC 0x10001000

#define JZ_REG_INTC_STATUS	0x00
#define JZ_REG_INTC_MASK	0x04
#define JZ_REG_INTC_SET_MASK	0x08
#define JZ_REG_INTC_CLEAR_MASK	0x0c
#define JZ_REG_INTC_PENDING	0x10

#define IRQ_BIT(x) BIT((x) - JZ_IRQ_BASE)

static void intc_irq_unmask(unsigned int irq)
{
	writel(IRQ_BIT(irq), jz_intc_base + JZ_REG_INTC_CLEAR_MASK);
}

static void intc_irq_mask(unsigned int irq)
{
	writel(IRQ_BIT(irq), jz_intc_base + JZ_REG_INTC_SET_MASK);
}

static void intc_irq_ack(unsigned int irq)
{
	writel(IRQ_BIT(irq), jz_intc_base + JZ_REG_INTC_PENDING);
}

static void intc_irq_end(unsigned int irq)
{
	if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS))) {
		intc_irq_unmask(irq);
	}
}

static int intc_irq_set_wake(unsigned int irq, unsigned int on)
{
	if (on)
		jz_intc_wakeup |= IRQ_BIT(irq);
	else
		jz_intc_wakeup &= ~IRQ_BIT(irq);

	return 0;
}

static struct irq_chip intc_irq_type = {
	.name =		"INTC",
	.mask =		intc_irq_mask,
	.unmask =	intc_irq_unmask,
	.ack =		intc_irq_ack,
	.end =		intc_irq_end,
	.set_wake =	intc_irq_set_wake,
};

static irqreturn_t jz4740_cascade(int irq, void *data)
{
	uint32_t irq_reg;
	irq_reg = readl(jz_intc_base + JZ_REG_INTC_PENDING);

	if (irq_reg) {
		generic_handle_irq(ffs(irq_reg) - 1 + JZ_IRQ_BASE);
		return IRQ_HANDLED;
	}

	return 0;
}

static struct irqaction jz4740_cascade_action = {
	.handler = jz4740_cascade,
	.name = "JZ4740 cascade interrupt"
};

void __init arch_init_irq(void)
{
	int i;
	mips_cpu_irq_init();

	jz_intc_base = ioremap(JZ_REG_BASE_INTC, 0x14);

	for (i = JZ_IRQ_BASE; i < JZ_IRQ_BASE + 32; i++) {
		intc_irq_mask(i);
		set_irq_chip_and_handler(i, &intc_irq_type, handle_level_irq);
	}

	setup_irq(2, &jz4740_cascade_action);
}

asmlinkage void plat_irq_dispatch(void)
{
	unsigned int pending = read_c0_status() & read_c0_cause() & ST0_IM;
	if (pending & STATUSF_IP2)
		jz4740_cascade(2, NULL);
	else if(pending & STATUSF_IP3)
		do_IRQ(3);
	else
		spurious_interrupt();
}

/* TODO: Use sysdev */
void jz4740_intc_suspend(void)
{
	jz_intc_saved = readl(jz_intc_base + JZ_REG_INTC_MASK);
    printk("intc wakeup: %d\n", jz_intc_wakeup);
	writel(~jz_intc_wakeup, jz_intc_base + JZ_REG_INTC_SET_MASK);
}

void jz4740_intc_resume(void)
{
	writel(~jz_intc_saved, jz_intc_base + JZ_REG_INTC_CLEAR_MASK);
}
