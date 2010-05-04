/*
 * i2c_jz47xx.c
 * I2C adapter for the INGENIC I2C bus access.
 *
 * Copyright (C) 2006 - 2008 Ingenic Semiconductor Inc.
 * Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <linux/module.h>
#include <asm/addrspace.h>
#include <asm/mach-jz4740/regs.h>
#include <asm/mach-jz4740/gpio.h>

#include "i2c-jz47xx.h"

#define JZ_REG_I2C_DR		0x000
#define JZ_REG_I2C_CR		0x004
#define JZ_REG_I2C_SR		0x008
#define JZ_REG_I2C_GR		0x00C

/* I2C Control Register (I2C_CR) */

#define JZ_I2C_CR_IEN		(1 << 4)
#define JZ_I2C_CR_STA		(1 << 3)
#define JZ_I2C_CR_STO		(1 << 2)
#define JZ_I2C_CR_AC		(1 << 1)
#define JZ_I2C_CR_I2CE		(1 << 0)

/* I2C Status Register (I2C_SR) */

#define JZ_I2C_SR_STX		(1 << 4)
#define JZ_I2C_SR_BUSY		(1 << 3)
#define JZ_I2C_SR_TEND		(1 << 2)
#define JZ_I2C_SR_DRF		(1 << 1)
#define JZ_I2C_SR_ACKF		(1 << 0)

#define I2C_REG(off)	REG8(0xB0042000 + off)
#define REG_I2C_DR	I2C_REG(0)
#define REG_I2C_CR	I2C_REG(4)
#define REG_I2C_SR	I2C_REG(8)

#define DEFAULT_I2C_CLOCK 100000

/* I2C protocol */
#define I2C_READ	1
#define I2C_WRITE	0

#define TIMEOUT         10000000
#define I2C_TIMEOUT	(HZ / 5)

struct jz_i2c {
	spinlock_t		lock;
	wait_queue_head_t	wait;
	struct i2c_msg		*msg;
	unsigned int		msg_num;
	unsigned int		slave_addr;
	struct i2c_adapter	adap;
	struct clk		*clk;
	void __iomem		*base;
	struct resource		*mem;
};

static struct jz_i2c *the_i2c;

static inline void i2c_jz_writeb(unsigned int val, unsigned char reg)
{
	writeb(val, the_i2c->base + reg);
}

static inline unsigned char i2c_jz_readb(unsigned int reg)
{
	return readb(the_i2c->base + reg);
}

static inline void jz_i2c_set_bits(unsigned int reg, unsigned int val)
{
	i2c_jz_writeb(i2c_jz_readb(reg) | val, reg);
}

static inline void jz_i2c_clear_bits(unsigned int reg, unsigned int val)
{
	i2c_jz_writeb(i2c_jz_readb(reg) & ~val, reg);
}

#define i2c_jz_enable() jz_i2c_set_bits(JZ_REG_I2C_CR, JZ_I2C_CR_I2CE)
#define i2c_jz_disable() jz_i2c_clear_bits(JZ_REG_I2C_CR, JZ_I2C_CR_I2CE)
#define i2c_jz_set_drf() jz_i2c_set_bits(JZ_REG_I2C_SR, JZ_I2C_SR_DRF)
#define i2c_jz_clear_drf() jz_i2c_clear_bits(JZ_REG_I2C_SR, JZ_I2C_SR_DRF)
#define i2c_jz_send_nack() jz_i2c_set_bits(JZ_REG_I2C_CR, JZ_I2C_CR_AC)
#define i2c_jz_send_ack() jz_i2c_clear_bits(JZ_REG_I2C_CR, JZ_I2C_CR_AC)
#define i2c_jz_send_start() jz_i2c_set_bits(JZ_REG_I2C_CR, JZ_I2C_CR_STA)
#define i2c_jz_send_stop() jz_i2c_set_bits(JZ_REG_I2C_CR, JZ_I2C_CR_STO)

/*
 * I2C interface
 */
static void i2c_jz_setclk(unsigned int rate)
{
	writew(clk_get_rate(the_i2c->clk) / (16 * rate), the_i2c->base + JZ_REG_I2C_GR);
}

static irqreturn_t i2c_jz_handle_irq(int irq, void *dev_id)
{
	struct jz_i2c *i2c = (struct jz_i2c *) dev_id;

	wake_up(&i2c->wait);
	dev_dbg(&i2c->adap.dev, "IRQ! SR = %08x\n", JZ_REG_I2C_SR);

	return IRQ_HANDLED;
}

static int xfer_read(__u16 addr, struct i2c_adapter *adap, unsigned char *buf, int length)
{
	unsigned char *b = buf;
	unsigned long timeout;
	unsigned char ackf;
	int ret = 0;

	dev_dbg(&adap->dev, "%s\n", __func__);

	clk_enable(the_i2c->clk);
	i2c_jz_setclk(DEFAULT_I2C_CLOCK);
	i2c_jz_enable();
	i2c_jz_clear_drf();

	if (length > 1) {
		i2c_jz_send_ack();
	} else {
		i2c_jz_send_nack();
	}

	i2c_jz_send_start();
	i2c_jz_writeb((addr << 1) | I2C_READ, JZ_REG_I2C_DR);

	timeout = jiffies + I2C_TIMEOUT;
	i2c_jz_set_drf();
	udelay(1000000 / DEFAULT_I2C_CLOCK * 5);

	timeout = jiffies + I2C_TIMEOUT;
	while (!(i2c_jz_readb(JZ_REG_I2C_SR) & JZ_I2C_SR_TEND))
		schedule();

	ackf = i2c_jz_readb(JZ_REG_I2C_SR) & JZ_I2C_SR_ACKF;
	dev_dbg(&adap->dev, "%s: Received %s\n", __func__, ackf ? "NACK" : "ACK");
	if (ackf) {
		ret = -EINVAL;
		goto end;
	}

	while (length--) {

//		ret = wait_event_interruptible_timeout(i2c->wait, __i2c_check_drf(), I2C_TIMEOUT);
//		if (ret < 0)
//			return ret;
//
//		if (ret) {
//			dev_dbg(&adap->dev, "DRF timeout, length = %d\n", length);
//			return -ETIMEDOUT;
//		}

		timeout = jiffies + I2C_TIMEOUT;
		while (!(i2c_jz_readb(JZ_REG_I2C_SR) & JZ_I2C_SR_DRF) && time_before(jiffies, timeout))
				schedule();

		if (!time_before(jiffies, timeout)) {
			dev_warn(&adap->dev, "DRF timeout, length = %d\n", length);
			dev_warn(&adap->dev, "%s: ACKF: %s\n", __func__, (i2c_jz_readb(JZ_REG_I2C_SR) & JZ_I2C_SR_ACKF) ? "NACK" : "ACK");
			ret = -ETIMEDOUT;
			goto end;
		}

		if (length == 1) {
			i2c_jz_send_nack();
			i2c_jz_send_stop();
		}

		*b++ = i2c_jz_readb(JZ_REG_I2C_DR);

		i2c_jz_clear_drf();
	}
end:
	i2c_jz_send_stop();
	udelay(1000000 / DEFAULT_I2C_CLOCK * 5);
	i2c_jz_disable();
	clk_disable(the_i2c->clk);

	return ret;
}


static int xfer_write(unsigned char addr, struct i2c_adapter *adap, unsigned char *buf, int length)
{
	int l = length + 1;
	int timeout;
	int ret = 0;

	dev_dbg(&adap->dev, "%s\n", __func__);

	clk_enable(the_i2c->clk);
	i2c_jz_setclk(DEFAULT_I2C_CLOCK);
	i2c_jz_enable();
	i2c_jz_clear_drf();
	i2c_jz_send_start();

	while (l--) {

		if (l == length)
			i2c_jz_writeb(addr << 1, JZ_REG_I2C_DR);
		else
			i2c_jz_writeb(*buf++, JZ_REG_I2C_DR);

		i2c_jz_set_drf();

		timeout = TIMEOUT;
		while ((i2c_jz_readb(JZ_REG_I2C_SR) & JZ_I2C_SR_DRF) && timeout)
			timeout--;

		if (!timeout) {
			dev_warn(&adap->dev, "DRF timeout, length = %d\n", length);
			ret = -ETIMEDOUT;
			goto end;
		}

		if (i2c_jz_readb(JZ_REG_I2C_SR) & JZ_I2C_SR_ACKF) {
			dev_dbg(&adap->dev, "NAK has been received during write\n");
			ret = -EINVAL;
			goto end;
		}
	}

end:
	i2c_jz_send_stop();
	while (!(i2c_jz_readb(JZ_REG_I2C_SR) & JZ_I2C_SR_TEND));
	udelay(1000000 / DEFAULT_I2C_CLOCK * 2);

	i2c_jz_disable();
	clk_disable(the_i2c->clk);
	return ret;
}

static int i2c_jz_xfer(struct i2c_adapter *adap, struct i2c_msg *pmsg, int num)
{
	int ret, i;

	dev_dbg(&adap->dev, "jz47xx_xfer: processing %d messages:\n", num);
	for (i = 0; i < num; i++) {
		dev_dbg(&adap->dev, " #%d: %sing %d byte%s %s 0x%02x\n", i,
			pmsg->flags & I2C_M_RD ? "read" : "writ",
			pmsg->len, pmsg->len > 1 ? "s" : "",
			pmsg->flags & I2C_M_RD ? "from" : "to",	pmsg->addr);

		if (pmsg->flags & I2C_M_RD)
			ret = xfer_read(pmsg->addr, adap, pmsg->buf, pmsg->len);
		else
			ret = xfer_write(pmsg->addr, adap, pmsg->buf, pmsg->len);

		if (ret)
			return ret;
		/* Wait until transfer is finished */
		dev_dbg(&adap->dev, "transfer complete\n");
		pmsg++;		/* next message */
	}
	return i;
}

static u32 i2c_jz_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm i2c_jz_algorithm = {
	.master_xfer	= i2c_jz_xfer,
	.functionality	= i2c_jz_functionality,
};

static int i2c_jz_probe(struct platform_device *dev)
{
	struct jz_i2c *i2c;
	struct i2c_jz_platform_data *plat = dev->dev.platform_data;
	int ret;
	struct resource *mem;

	mem = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&dev->dev, "Failed to get register memory resource\n");
		return -ENOENT;
	}

	mem = request_mem_region(mem->start, resource_size(mem), dev->name);
	if (!mem) {
		dev_err(&dev->dev, "Failed to request register memory resource\n");
		return -EBUSY;
	}

	i2c = kzalloc(sizeof(struct jz_i2c), GFP_KERNEL);
	if (!i2c) {
		printk("There is no enough memory\n");
		ret = -ENOMEM;
		goto emalloc;
	}

	the_i2c = i2c;

	ret = gpio_request(JZ_GPIO_PORTD(23), "I2C SDA");
	if (ret) {
		dev_err(&dev->dev, "Failed to request SDA pin\n");
		goto err_request_sda;
	}

	jz_gpio_set_function(JZ_GPIO_PORTD(23), JZ_GPIO_FUNC2);
	gpio_request(JZ_GPIO_PORTD(24), "I2C SCL");
	if (ret) {
		dev_err(&dev->dev, "Failed to request SCL pin\n");
		goto err_request_scl;
	}

	jz_gpio_set_function(JZ_GPIO_PORTD(24), JZ_GPIO_FUNC2);

	i2c->clk = clk_get(&dev->dev, "i2c");
	if (IS_ERR(i2c->clk)) {
		ret = PTR_ERR(i2c->clk);
		goto err_clk_get;
	}

	i2c->base = ioremap(mem->start, resource_size(mem));
	if (!i2c->base) {
		dev_err(&dev->dev, "Failed to ioremap register memory region\n");
		ret = -EBUSY;
		goto err_ioremap;
	}
	dev_info(&dev->dev, "I2C base addr is 0x%p (remapped from 0x%p)\n", i2c->base, mem->start);

	i2c_jz_setclk(DEFAULT_I2C_CLOCK);
	clk_enable(i2c->clk);
	i2c_jz_enable();

	i2c->adap.owner = THIS_MODULE;
	i2c->adap.algo = &i2c_jz_algorithm;
	i2c->adap.retries = 5;
	spin_lock_init(&i2c->lock);
	init_waitqueue_head(&i2c->wait);
	sprintf(i2c->adap.name, "jz_i2c-i2c.%u", dev->id);
	i2c->adap.algo_data = i2c;
	i2c->adap.dev.parent = &dev->dev;
	i2c_set_adapdata(&i2c->adap, i2c);

	if (plat) {
		i2c->adap.class = plat->class;
	}

//	ret = request_irq(IRQ_I2C, i2c_jz_handle_irq, 0, dev->name, i2c);
//	if (ret) {
//		dev_err(&dev->dev, "Unable to claim I2C IRQ\n");
//		goto err_irq;
//	}

	/*
	 * If "dev->id" is negative we consider it as zero.
	 * The reason to do so is to avoid sysfs names that only make
	 * sense when there are multiple adapters.
	 */
	i2c->adap.nr = dev->id != -1 ? dev->id : 0;
	/* ret = i2c_add_adapter(&i2c->adap); */
	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret < 0) {
		printk(KERN_INFO "I2C: Failed to add bus\n");
		goto eadapt;
	}

	platform_set_drvdata(dev, i2c);
	dev_info(&dev->dev, "JZ47xx i2c bus driver.\n");

	i2c_jz_disable();
	clk_disable(i2c->clk);

	return 0;
eadapt:
//	free_irq(IRQ_I2C, i2c);
//err_irq:
	i2c_jz_disable();
	clk_disable(i2c->clk);

	iounmap(i2c->base);
err_ioremap:
	clk_put(i2c->clk);
err_clk_get:
	gpio_free(JZ_GPIO_PORTD(23));
err_request_scl:
	gpio_free(JZ_GPIO_PORTD(24));
err_request_sda:
	kfree(i2c);
emalloc:
	release_mem_region(mem->start, resource_size(mem));
	return ret;
}

static int i2c_jz_remove(struct platform_device *dev)
{
	struct jz_i2c *i2c = platform_get_drvdata(dev);
	int rc;

//	free_irq(IRQ_I2C, i2c);
	i2c_jz_disable();
	clk_disable(i2c->clk);

	iounmap(i2c->base);
	clk_put(i2c->clk);
	gpio_free(JZ_GPIO_PORTD(23));
	gpio_free(JZ_GPIO_PORTD(24));
	kfree(i2c);
	release_mem_region(i2c->mem->start, resource_size(i2c->mem));
	rc = i2c_del_adapter(&i2c->adap);
	platform_set_drvdata(dev, NULL);
	return rc;
}

static struct platform_driver i2c_jz_driver = {
	.probe		= i2c_jz_probe,
	.remove		= i2c_jz_remove,
	.driver		= {
		.name	= "jz_i2c",
	},
};

static int __init i2c_adap_jz_init(void)
{
	return platform_driver_register(&i2c_jz_driver);
}

static void __exit i2c_adap_jz_exit(void)
{
	return platform_driver_unregister(&i2c_jz_driver);
}

MODULE_LICENSE("GPL");

module_init(i2c_adap_jz_init);
module_exit(i2c_adap_jz_exit);
