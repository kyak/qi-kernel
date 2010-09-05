/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *  JZ4740 SoC DMA support
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>

#include <linux/dmapool.h>
#include <linux/scatterlist.h>

#include <linux/dma-mapping.h>
#include <asm/mach-jz4740/dma.h>
#include <asm/mach-jz4740/base.h>

#define JZ_REG_DMA_SRC_ADDR(x)		(0x00 + (x) * 0x20)
#define JZ_REG_DMA_DST_ADDR(x)		(0x04 + (x) * 0x20)
#define JZ_REG_DMA_TRANSFER_COUNT(x)	(0x08 + (x) * 0x20)
#define JZ_REG_DMA_REQ_TYPE(x)		(0x0C + (x) * 0x20)
#define JZ_REG_DMA_STATUS_CTRL(x)	(0x10 + (x) * 0x20)
#define JZ_REG_DMA_CMD(x)		(0x14 + (x) * 0x20)
#define JZ_REG_DMA_DESC_ADDR(x)		(0x18 + (x) * 0x20)

#define JZ_REG_DMA_CTRL			0x300
#define JZ_REG_DMA_IRQ			0x304
#define JZ_REG_DMA_DOORBELL		0x308
#define JZ_REG_DMA_DOORBELL_SET		0x30C

#define JZ_DMA_STATUS_CTRL_NO_DESC		BIT(31)
#define JZ_DMA_STATUS_CTRL_DESC_INV		BIT(6)
#define JZ_DMA_STATUS_CTRL_ADDR_ERR		BIT(4)
#define JZ_DMA_STATUS_CTRL_TRANSFER_DONE	BIT(3)
#define JZ_DMA_STATUS_CTRL_HALT			BIT(2)
#define JZ_DMA_STATUS_CTRL_COUNT_TERMINATE	BIT(1)
#define JZ_DMA_STATUS_CTRL_ENABLE		BIT(0)

#define JZ_DMA_CMD_SRC_INC			BIT(23)
#define JZ_DMA_CMD_DST_INC			BIT(22)
#define JZ_DMA_CMD_RDIL_MASK			(0xf << 16)
#define JZ_DMA_CMD_SRC_WIDTH_MASK		(0x3 << 14)
#define JZ_DMA_CMD_DST_WIDTH_MASK		(0x3 << 12)
#define JZ_DMA_CMD_INTERVAL_LENGTH_MASK		(0x7 << 8)
#define JZ_DMA_CMD_BLOCK_MODE			BIT(7)
#define JZ_DMA_CMD_DESC_VALID			BIT(4)
#define JZ_DMA_CMD_DESC_VALID_MODE		BIT(3)
#define JZ_DMA_CMD_VALID_IRQ_ENABLE		BIT(2)
#define JZ_DMA_CMD_TRANSFER_IRQ_ENABLE		BIT(1)
#define JZ_DMA_CMD_LINK_ENABLE			BIT(0)

#define JZ_DMA_CMD_FLAGS_OFFSET 22
#define JZ_DMA_CMD_RDIL_OFFSET 16
#define JZ_DMA_CMD_SRC_WIDTH_OFFSET 14
#define JZ_DMA_CMD_DST_WIDTH_OFFSET 12
#define JZ_DMA_CMD_TRANSFER_SIZE_OFFSET 8
#define JZ_DMA_CMD_MODE_OFFSET 7

#define JZ_DMA_CTRL_PRIORITY_MASK	(0x3 << 8)
#define JZ_DMA_CTRL_HALT		BIT(3)
#define JZ_DMA_CTRL_ADDRESS_ERROR	BIT(2)
#define JZ_DMA_CTRL_ENABLE		BIT(0)


static void __iomem *jz4740_dma_base;
static spinlock_t jz4740_dma_lock;
static struct dma_pool *jz4740_dma_desc_pool;

static inline uint32_t jz4740_dma_read(size_t reg)
{
	return readl(jz4740_dma_base + reg);
}

static inline void jz4740_dma_write(size_t reg, uint32_t val)
{
	writel(val, jz4740_dma_base + reg);
}

static inline void jz4740_dma_write_mask(size_t reg, uint32_t val, uint32_t mask)
{
	uint32_t val2;
	val2 = jz4740_dma_read(reg);
	val2 &= ~mask;
	val2 |= val;
	jz4740_dma_write(reg, val2);
}

struct jz4740_dma_desc {
	uint32_t *data;
	dma_addr_t phys_addr;
	struct jz4740_dma_desc *next;

	uint32_t transfer_shift;
};
#define JZ4740_DMA_DESC_SIZE (sizeof(uint32_t) * 4)

struct jz4740_dma_chan {
	unsigned int id;
	void *dev;
	const char *name;

	enum jz4740_dma_flags flags;
	uint32_t transfer_shift;

	jz4740_dma_complete_callback_t complete_cb;

	struct jz4740_dma_desc *desc;

	unsigned used:1;
};

#define JZ4740_DMA_CHANNEL(_id) { .id = _id }

struct jz4740_dma_chan jz4740_dma_channels[] = {
	JZ4740_DMA_CHANNEL(0),
	JZ4740_DMA_CHANNEL(1),
	JZ4740_DMA_CHANNEL(2),
	JZ4740_DMA_CHANNEL(3),
	JZ4740_DMA_CHANNEL(4),
	JZ4740_DMA_CHANNEL(5),
};

struct jz4740_dma_chan *jz4740_dma_request(void *dev, const char *name)
{
	unsigned int i;
	struct jz4740_dma_chan *dma = NULL;

	spin_lock(&jz4740_dma_lock);

	for (i = 0; i < ARRAY_SIZE(jz4740_dma_channels); ++i) {
		if (!jz4740_dma_channels[i].used) {
			dma = &jz4740_dma_channels[i];
			dma->used = 1;
			break;
		}
	}

	spin_unlock(&jz4740_dma_lock);

	if (!dma)
		return NULL;

	dma->dev = dev;
	dma->name = name;

	return dma;
}
EXPORT_SYMBOL_GPL(jz4740_dma_request);

static uint32_t jz4740_dma_config_to_cmd(const struct jz4740_dma_config *config)
{
	uint32_t cmd;

	cmd = config->flags << JZ_DMA_CMD_FLAGS_OFFSET;
	cmd |= config->src_width << JZ_DMA_CMD_SRC_WIDTH_OFFSET;
	cmd |= config->dst_width << JZ_DMA_CMD_DST_WIDTH_OFFSET;
	cmd |= config->transfer_size << JZ_DMA_CMD_TRANSFER_SIZE_OFFSET;
	cmd |= config->mode << JZ_DMA_CMD_MODE_OFFSET;
	cmd |= JZ_DMA_CMD_TRANSFER_IRQ_ENABLE;

	return cmd;
}

static uint32_t jz4740_dma_config_shift(const struct jz4740_dma_config *config)
{
	unsigned int shift;

	switch (config->transfer_size) {
	case JZ4740_DMA_TRANSFER_SIZE_2BYTE:
		shift = 1;
		break;
	case JZ4740_DMA_TRANSFER_SIZE_4BYTE:
		shift = 2;
		break;
	case JZ4740_DMA_TRANSFER_SIZE_16BYTE:
		shift = 4;
		break;
	case JZ4740_DMA_TRANSFER_SIZE_32BYTE:
		shift = 5;
		break;
	default:
		shift = 0;
		break;
	}

	return shift;
}

void jz4740_dma_configure(struct jz4740_dma_chan *dma,
	const struct jz4740_dma_config *config)
{
	uint32_t cmd;

	dma->transfer_shift = jz4740_dma_config_shift(config);
	cmd = jz4740_dma_config_to_cmd(config);

	jz4740_dma_write(JZ_REG_DMA_CMD(dma->id), cmd);
	jz4740_dma_write(JZ_REG_DMA_STATUS_CTRL(dma->id), 0);
	jz4740_dma_write(JZ_REG_DMA_REQ_TYPE(dma->id), config->request_type);
}
EXPORT_SYMBOL_GPL(jz4740_dma_configure);

void jz4740_dma_desc_configure(struct jz4740_dma_desc *desc,
	const struct jz4740_dma_config *config)
{
	desc->transfer_shift = jz4740_dma_config_shift(config);
	desc->data[0] = jz4740_dma_config_to_cmd(config);
}
EXPORT_SYMBOL_GPL(jz4740_dma_desc_configure);

void jz4740_dma_set_src_addr(struct jz4740_dma_chan *dma, dma_addr_t src)
{
	jz4740_dma_write(JZ_REG_DMA_SRC_ADDR(dma->id), src);
}
EXPORT_SYMBOL_GPL(jz4740_dma_set_src_addr);

void jz4740_dma_set_dst_addr(struct jz4740_dma_chan *dma, dma_addr_t dst)
{
	jz4740_dma_write(JZ_REG_DMA_DST_ADDR(dma->id), dst);
}
EXPORT_SYMBOL_GPL(jz4740_dma_set_dst_addr);

void jz4740_dma_set_transfer_count(struct jz4740_dma_chan *dma, uint32_t count)
{
	count >>= dma->transfer_shift;
	jz4740_dma_write(JZ_REG_DMA_TRANSFER_COUNT(dma->id), count);
}
EXPORT_SYMBOL_GPL(jz4740_dma_set_transfer_count);

void jz4740_dma_set_complete_cb(struct jz4740_dma_chan *dma,
	jz4740_dma_complete_callback_t cb)
{
	dma->complete_cb = cb;
}
EXPORT_SYMBOL_GPL(jz4740_dma_set_complete_cb);

void jz4740_dma_free(struct jz4740_dma_chan *dma)
{
	dma->dev = NULL;
	dma->complete_cb = NULL;
	dma->used = 0;
}
EXPORT_SYMBOL_GPL(jz4740_dma_free);

void jz4740_dma_enable(struct jz4740_dma_chan *dma)
{
	uint32_t ctrl;

	ctrl = JZ_DMA_STATUS_CTRL_ENABLE;

	if (!dma->desc)
		ctrl |= JZ_DMA_STATUS_CTRL_NO_DESC;

	jz4740_dma_write_mask(JZ_REG_DMA_STATUS_CTRL(dma->id),
			ctrl,
			JZ_DMA_STATUS_CTRL_HALT | JZ_DMA_STATUS_CTRL_NO_DESC |
			JZ_DMA_STATUS_CTRL_ENABLE);

	jz4740_dma_write_mask(JZ_REG_DMA_CTRL,
			JZ_DMA_CTRL_ENABLE,
			JZ_DMA_CTRL_HALT | JZ_DMA_CTRL_ENABLE);

	if (dma->desc) {
		jz4740_dma_write(JZ_REG_DMA_DESC_ADDR(dma->id), dma->desc->phys_addr);
		jz4740_dma_write(JZ_REG_DMA_DOORBELL, BIT(dma->id));
	}
}
EXPORT_SYMBOL_GPL(jz4740_dma_enable);

void jz4740_dma_disable(struct jz4740_dma_chan *dma)
{
	jz4740_dma_write_mask(JZ_REG_DMA_STATUS_CTRL(dma->id), 0,
			JZ_DMA_STATUS_CTRL_ENABLE);
}
EXPORT_SYMBOL_GPL(jz4740_dma_disable);

uint32_t jz4740_dma_get_residue(const struct jz4740_dma_chan *dma)
{
	uint32_t residue;
	residue = jz4740_dma_read(JZ_REG_DMA_TRANSFER_COUNT(dma->id));
	return residue << dma->transfer_shift;
}
EXPORT_SYMBOL_GPL(jz4740_dma_get_residue);

static void jz4740_dma_chan_irq(struct jz4740_dma_chan *dma)
{
	uint32_t status;

	status = jz4740_dma_read(JZ_REG_DMA_STATUS_CTRL(dma->id));

	jz4740_dma_write_mask(JZ_REG_DMA_STATUS_CTRL(dma->id), 0,
		JZ_DMA_STATUS_CTRL_ENABLE | JZ_DMA_STATUS_CTRL_TRANSFER_DONE);

	if (dma->complete_cb)
		dma->complete_cb(dma, 0, dma->dev);
}

static irqreturn_t jz4740_dma_irq(int irq, void *dev_id)
{
	uint32_t irq_status;
	unsigned int i;

	irq_status = readl(jz4740_dma_base + JZ_REG_DMA_IRQ);

	for (i = 0; i < 6; ++i) {
		if (irq_status & (1 << i))
			jz4740_dma_chan_irq(&jz4740_dma_channels[i]);
	}

	return IRQ_HANDLED;
}

void jz4740_dma_chan_set_desc(struct jz4740_dma_chan *chan,
	struct jz4740_dma_desc *desc)
{
	chan->desc = desc;
}
EXPORT_SYMBOL_GPL(jz4740_dma_chan_set_desc);

struct jz4740_dma_desc *jz4740_dma_desc_alloc(void)
{
	struct jz4740_dma_desc *desc;
	desc = kzalloc(sizeof(*desc), GFP_KERNEL);

	if (!desc)
		return NULL;

	desc->data = dma_pool_alloc(jz4740_dma_desc_pool, GFP_KERNEL,
		&desc->phys_addr);

	if (!desc->data) {
		kfree(desc);
		return NULL;
	}

	return desc;
}
EXPORT_SYMBOL_GPL(jz4740_dma_desc_alloc);

void jz4740_dma_desc_free(struct jz4740_dma_desc *desc)
{
	dma_pool_free(jz4740_dma_desc_pool, desc->data, desc->phys_addr);
	kfree(desc);
}
EXPORT_SYMBOL_GPL(jz4740_dma_desc_free);

void jz4740_dma_desc_set_src_addr(struct jz4740_dma_desc *desc, dma_addr_t src)
{
	desc->data[1] = src;
}
EXPORT_SYMBOL_GPL(jz4740_dma_desc_set_src_addr);

void jz4740_dma_desc_set_dst_addr(struct jz4740_dma_desc *desc, dma_addr_t dst)
{
	desc->data[2] = dst;
}
EXPORT_SYMBOL_GPL(jz4740_dma_desc_set_dst_addr);

int jz4740_dma_desc_set_transfer_count(struct jz4740_dma_desc *desc,
	uint32_t count)
{
	count >>= desc->transfer_shift;

	if (count > 0xffffff)
		return -EINVAL;

	desc->data[3] &= ~0xffffff;
	desc->data[3] |= count;

	return 0;
}
EXPORT_SYMBOL_GPL(jz4740_dma_desc_set_transfer_count);

int jz4740_dma_desc_set_next(struct jz4740_dma_desc *desc,
	struct jz4740_dma_desc *next)
{
	if (!next) {
		desc->next = NULL;
		desc->data[3] &= 0xff000000;
		desc->data[0] &= ~JZ_DMA_CMD_LINK_ENABLE;

		return 0;
	}

	if ((desc->phys_addr & 0xfffff000) != (next->phys_addr & 0xfffff000))
		return -EINVAL;

	desc->next = next;

	desc->data[3] &= 0xff000000;
	desc->data[3] |= (next->phys_addr & 0xff0) << 20;

	desc->data[0] |= JZ_DMA_CMD_LINK_ENABLE;

	return 0;
}
EXPORT_SYMBOL_GPL(jz4740_dma_desc_set_next);

struct jz4740_dma_sg *jz4740_dma_sg_alloc(struct scatterlist *sgl,
	size_t sg_len, enum dma_data_direction direction,
	struct jz4740_dma_config *config, dma_addr_t device_addr)
{
	struct scatterlist *sg;
	struct jz4740_dma_desc *descs;
	struct jz4740_dma_desc *prev = NULL;
	int i;

	descs = kzalloc(sizeof(*descs) * sg_len, GFP_KERNEL);

	if (!descs)
		return descs;

	for_each_sg(sgl, sg, sg_len, i) {
		descs[i].data = dma_pool_alloc(jz4740_dma_desc_pool, GFP_KERNEL,
			&descs[i].phys_addr);

		if (!descs[i].data)
			goto err;

		jz4740_dma_desc_configure(&descs[i], config);
		if (direction == DMA_TO_DEVICE) {
			jz4740_dma_desc_set_src_addr(&descs[i], sg_phys(sg));
			jz4740_dma_desc_set_dst_addr(&descs[i], device_addr);
		} else {
			jz4740_dma_desc_set_src_addr(&descs[i], device_addr);
			jz4740_dma_desc_set_src_addr(&descs[i], sg_phys(sg));
		}
		jz4740_dma_desc_set_transfer_count(&descs[i], sg_dma_len(sg));

		if (prev)
			jz4740_dma_desc_set_next(&descs[i], prev);
		prev = &descs[i];
	}

	return (struct jz4740_dma_sg *)descs;

err:
	kfree(descs);
	for (--i; i >= 0; --i)
		dma_pool_free(jz4740_dma_desc_pool, descs[i].data, descs[i].phys_addr);

	return NULL;
}
EXPORT_SYMBOL_GPL(jz4740_dma_sg_alloc);

static struct jz4740_dma_desc *jz4740_dma_sg_get_desc(struct jz4740_dma_sg *sg)
{
	return (struct jz4740_dma_desc*)sg;
}

void jz4740_dma_sg_free(struct jz4740_dma_sg *sg)
{
	struct jz4740_dma_desc *desc = jz4740_dma_sg_get_desc(sg);
	while (desc) {
		dma_pool_free(jz4740_dma_desc_pool, desc->data, desc->phys_addr);
		desc = desc->next;
	}
	kfree(sg);
}
EXPORT_SYMBOL_GPL(jz4740_dma_sg_free);

static int jz4740_dma_init(void)
{
	unsigned int ret;

	jz4740_dma_base = ioremap(JZ4740_DMAC_BASE_ADDR, 0x400);

	if (!jz4740_dma_base)
		return -EBUSY;

	spin_lock_init(&jz4740_dma_lock);

	ret = request_irq(JZ4740_IRQ_DMAC, jz4740_dma_irq, 0, "DMA", NULL);

	if (ret)
		printk(KERN_ERR "JZ4740 DMA: Failed to request irq: %d\n", ret);

	jz4740_dma_desc_pool = dma_pool_create("jz4740 dma descriptors", NULL,
		JZ4740_DMA_DESC_SIZE, JZ4740_DMA_DESC_SIZE, 0);

	return ret;
}
arch_initcall(jz4740_dma_init);
