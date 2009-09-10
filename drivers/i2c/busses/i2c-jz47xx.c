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

#include <asm/irq.h>
#include <asm/io.h>
#include <linux/module.h>
#include <asm/addrspace.h>

#include <asm/jzsoc.h>
#include "i2c-jz47xx.h"


#define DEFAULT_I2C_CLOCK 100000

/* I2C protocol */
#define I2C_READ	1
#define I2C_WRITE	0

#define TIMEOUT         10000000
#define I2C_TIMEOUT	(HZ / 5)

unsigned short sub_addr = 0;
int addr_val = 0;
struct jz_i2c {
	spinlock_t		lock;
	wait_queue_head_t	wait;
	struct i2c_msg		*msg;
	unsigned int		msg_num;
	unsigned int		slave_addr;
	struct i2c_adapter	adap;
	struct clk		*clk;
};


/*
 * I2C interface
 */
void i2c_jz_setclk(unsigned int i2cclk)
{
	__i2c_set_clk(jz_clocks.extalclk, i2cclk);
}


void dump_regs(int step)
{
	printk("%d: I2C regs:\n", step);
	printk("SR: %08x\tCR: %08x\n", REG_I2C_SR, REG_I2C_CR);
}

int very_useful_function(unsigned long arg)
{
	if (arg)
		return arg;
	else
		return 111;
}

static irqreturn_t i2c_jz_handle_irq(int irq, void *dev_id)
{
	struct jz_i2c *i2c = (struct jz_i2c *) dev_id;

	wake_up(&i2c->wait);
	dev_dbg(&i2c->adap.dev, "IRQ! SR = %08x\n", REG_I2C_SR);

	return IRQ_HANDLED;
}

static int xfer_read(__u16 addr, struct i2c_adapter *adap, unsigned char *buf, int length)
{
	unsigned char *b = buf;
	unsigned long timeout;
	int ret = 0;

	dev_dbg(&adap->dev, "%s\n", __func__);

	__cpm_start_i2c();
	__i2c_set_clk(jz_clocks.extalclk, DEFAULT_I2C_CLOCK);
	__i2c_enable();
	__i2c_clear_drf();

	if (length)
		__i2c_send_ack();
	else
		__i2c_send_nack();

	__i2c_send_start();
	__i2c_write((addr << 1) | I2C_READ);

	timeout = jiffies + I2C_TIMEOUT;
	__i2c_set_drf();

	while (__i2c_transmit_ended() && time_before(jiffies, timeout))
		schedule();
	if (!time_before(jiffies, timeout)) {
		dev_dbg(&adap->dev, "%s: %d: timeout\n", __func__, __LINE__);
		ret = -ETIMEDOUT;
		goto end;
	}

	timeout = jiffies + I2C_TIMEOUT;
	while (!__i2c_transmit_ended())
		schedule();

	dev_dbg(&adap->dev, "%s: Received %s\n", __func__, __i2c_received_ack() ? "ACK" : "NACK");
	if (!__i2c_received_ack()) {
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
		while (!__i2c_check_drf() && time_before(jiffies, timeout))
				schedule();

		if (!time_before(jiffies, timeout)) {
			dev_dbg(&adap->dev, "DRF timeout, length = %d\n", length);
			dev_dbg(&adap->dev, "%s: ACKF: %s\n", __func__, __i2c_received_ack() ? "ACK" : "NACK");
			ret = -ETIMEDOUT;
			goto end;
		}

		if (length == 1) {
			__i2c_send_nack();
			__i2c_send_stop();
		}

		*b++ = __i2c_read();

		__i2c_clear_drf();
	}
end:
	__i2c_disable();
	__cpm_stop_i2c();

	return ret;
}


static int xfer_write(unsigned char addr, struct i2c_adapter *adap, unsigned char *buf, int length)
{
	int l = length + 1;
	int timeout;

	dev_dbg(&adap->dev, "%s\n", __func__);

	__cpm_start_i2c();
	__i2c_set_clk(jz_clocks.extalclk, DEFAULT_I2C_CLOCK);
	__i2c_enable();
	__i2c_clear_drf();
	__i2c_send_start();

	while (l--) {

		if (l == length)
			__i2c_write(addr << 1);
		else
			__i2c_write(*buf++);

		__i2c_set_drf();

		timeout = TIMEOUT;
		while (__i2c_check_drf() && timeout)
			timeout--;

		if (!timeout) {
			dev_dbg(&adap->dev, "DRF timeout, length = %d\n", length);
//			__i2c_send_stop();
			__i2c_disable();
			return -ETIMEDOUT;
		}

		if (!__i2c_received_ack()) {
			dev_dbg(&adap->dev, "NAK has been received during write\n");
			__i2c_disable();
			return -EINVAL;
		}
	}

	__i2c_send_stop();
	while (!__i2c_transmit_ended());

	__i2c_disable();
	__cpm_stop_i2c();
	return 0;
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

	__cpm_start_i2c();
	__i2c_set_clk(jz_clocks.extalclk, DEFAULT_I2C_CLOCK);
	__i2c_enable();

	i2c = kzalloc(sizeof(struct jz_i2c), GFP_KERNEL);
	if (!i2c) {
		printk("There is no enough memory\n");
		ret = -ENOMEM;
		goto emalloc;
	}

	i2c->adap.owner   = THIS_MODULE;
	i2c->adap.algo    = &i2c_jz_algorithm;
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

	__i2c_disable();
	__cpm_stop_i2c();

	return 0;
eadapt:
//	free_irq(IRQ_I2C, i2c);
//err_irq:
	__i2c_disable();
	__cpm_stop_i2c();
	kfree(i2c);
emalloc:
	return ret;
}

static int i2c_jz_remove(struct platform_device *dev)
{
	struct jz_i2c *i2c = platform_get_drvdata(dev);
	int rc;

//	free_irq(IRQ_I2C, i2c);
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
