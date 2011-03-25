/*
 *  Copyright (c) 2011 Peter Zotov <whitequark@whitequark.org>
 *  JZ4750 SoC NAND controller driver
 *
 *  Based on JZ4740 NAND controller driver,
 *    (C) 2009-2010, Lars-Peter Clausen <lars@metafoo.de>
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

#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <linux/gpio.h>

#include <jz4750_nand.h>

#define JZ_REG_NAND_CTRL	0x50

#define JZ_NAND_CTRL_ENABLE_CHIP(x) BIT((x) << 1)
#define JZ_NAND_CTRL_ASSERT_CHIP(x) BIT(((x) << 1) + 1)

#define JZ_NAND_MEM_ADDR_OFFSET	0x800000
#define JZ_NAND_MEM_CMD_OFFSET	0x400000

#define JZ_REG_BCH_CR		0x00
#define JZ_REG_BCH_CR_SET	0x04
#define JZ_REG_BCH_CR_CLR	0x08
#define JZ_REG_BCH_CNT		0x0C
#define JZ_REG_BCH_DR		0x10
#define JZ_REG_BCH_PAR(i)	(0x14 + (i))
#define JZ_REG_BCH_STATUS	0x6C
#define JZ_REG_BCH_STATUS_SET	0x74
#define JZ_REG_BCH_STATUS_CLR	0x78
#define JZ_REG_BCH_ERR(i)	(0x3C + ((i) << 2))

#define JZ_BCH_ENABLE		BIT(0)
#define JZ_BCH_RESET		BIT(1)
#define JZ_BCH_8BIT		BIT(3)
#define JZ_BCH_ENCODE		BIT(2)
#define JZ_BCH_USE_DMA		BIT(4)

#define JZ_BCH_STATUS_ERROR		BIT(0)
#define JZ_BCH_STATUS_UNCOR		BIT(1)
#define JZ_BCH_STATUS_ENC_FINISH	BIT(2)
#define JZ_BCH_STATUS_DEC_FINISH	BIT(3)
#define JZ_BCH_STATUS_ALL_F		BIT(4)
#define JZ_BCH_STATUS_ALL_0		BIT(5)
#define JZ_BCH_STATUS_ERR_COUNT		0xf0000000

struct jz_nand {
	struct mtd_info mtd;
	struct nand_chip chip;
	void __iomem *base;
	struct resource *mem;

	void __iomem *bank_base;
	struct resource *bank_mem;

	void __iomem *bch_base;
	struct resource *bch_mem;

	struct jz_nand_platform_data *pdata;
	bool is_reading;

	struct clk* clk_bch;
};

static inline struct jz_nand *mtd_to_jz_nand(struct mtd_info *mtd)
{
	return container_of(mtd, struct jz_nand, mtd);
}

static void jz_nand_cmd_ctrl(struct mtd_info *mtd, int dat, unsigned int ctrl)
{
	struct jz_nand *nand = mtd_to_jz_nand(mtd);
	struct nand_chip *chip = mtd->priv;
	uint32_t reg;

	if (ctrl & NAND_CTRL_CHANGE) {
		BUG_ON((ctrl & NAND_ALE) && (ctrl & NAND_CLE));

		if (ctrl & NAND_ALE)
			chip->IO_ADDR_W = nand->bank_base + JZ_NAND_MEM_ADDR_OFFSET;
		else if (ctrl & NAND_CLE)
			chip->IO_ADDR_W = nand->bank_base + JZ_NAND_MEM_CMD_OFFSET;
		else
			chip->IO_ADDR_W = nand->bank_base;

		reg = readl(nand->base + JZ_REG_NAND_CTRL);
		if (ctrl & NAND_NCE)
			reg |= JZ_NAND_CTRL_ASSERT_CHIP(0);
		else
			reg &= ~JZ_NAND_CTRL_ASSERT_CHIP(0);
		writel(reg, nand->base + JZ_REG_NAND_CTRL);
	}

	if (dat != NAND_CMD_NONE)
		writeb(dat, chip->IO_ADDR_W);
}

static int jz_nand_dev_ready(struct mtd_info *mtd)
{
	struct jz_nand *nand = mtd_to_jz_nand(mtd);
	return gpio_get_value_cansleep(nand->pdata->busy_gpio);
}

static void jz_nand_hwctl(struct mtd_info *mtd, int mode)
{
	struct jz_nand *nand = mtd_to_jz_nand(mtd);

	/* While BCH_CR register is defined as RW in manual, it is
	 * not true. It can only be modified through BCH_CR_SET and
	 * BCH_CR_CLR registers.
	 */
	if (nand->pdata->bch_8bit)
		writel(JZ_BCH_8BIT, nand->bch_base + JZ_REG_BCH_CR_SET);
	else
		writel(JZ_BCH_8BIT, nand->bch_base + JZ_REG_BCH_CR_CLR);

	switch (mode) {
	case NAND_ECC_READ:
		writel(JZ_BCH_ENCODE, nand->bch_base + JZ_REG_BCH_CR_CLR);
		nand->is_reading = true;
		break;
	case NAND_ECC_WRITE:
		writel(JZ_BCH_ENCODE, nand->bch_base + JZ_REG_BCH_CR_SET);
		nand->is_reading = false;
		break;
	default:
		break;
	}
}

static int jz_nand_calculate_ecc_bch(struct mtd_info *mtd, const uint8_t *dat,
	uint8_t *ecc_code)
{
	struct jz_nand *nand = mtd_to_jz_nand(mtd);
	uint32_t status;
	int i;
	unsigned int timeout = 1000;

	int size  = nand->chip.ecc.size,
	    bytes = nand->chip.ecc.bytes;

	return 0;

	if (nand->is_reading)
		return 0;

	writel(JZ_BCH_ENABLE | JZ_BCH_RESET, nand->bch_base + JZ_REG_BCH_CR_SET);

	writel((size << 1) & 0xffff, nand->bch_base + JZ_REG_BCH_CNT);

	for(i = 0; i < size; i++)
		writeb(dat[i], nand->bch_base + JZ_REG_BCH_DR);

	do {
		status = readl(nand->bch_base + JZ_REG_BCH_STATUS);
	} while (!(status & JZ_BCH_STATUS_ENC_FINISH) && --timeout);

	writel(JZ_BCH_ENABLE, nand->bch_base + JZ_REG_BCH_CR_CLR);

	if (timeout == 0)
		return -1;

	for(i = 0; i < bytes; i++)
		ecc_code[i] = readb(nand->bch_base + JZ_REG_BCH_PAR(i));

	return 0;
}

static int jz_nand_correct_ecc_bch(struct mtd_info *mtd, uint8_t *dat,
	uint8_t *read_ecc, uint8_t *calc_ecc)
{
	struct jz_nand *nand = mtd_to_jz_nand(mtd);
	int i, error_count;
	uint32_t status, error_regs[4];
	uint16_t *error_indexes, index;
	unsigned int timeout = 1000;

	int size  = nand->chip.ecc.size,
	    bytes = nand->chip.ecc.bytes;

	return 0;

	writel(JZ_BCH_ENABLE | JZ_BCH_RESET, nand->bch_base + JZ_REG_BCH_CR_SET);

	/* Clear status bits */
	writel(0xff,          nand->bch_base + JZ_REG_BCH_STATUS_CLR);

	writel((size + bytes) << 17, nand->bch_base + JZ_REG_BCH_CNT);

	for (i = 0; i < size; i++)
		writeb(dat[i], nand->bch_base + JZ_REG_BCH_DR);

	for (i = 0; i < bytes; i++)
		writeb(read_ecc[i], nand->bch_base + JZ_REG_BCH_DR);

	do {
		status = readl(nand->bch_base + JZ_REG_BCH_STATUS);
	} while (!(status & JZ_BCH_STATUS_DEC_FINISH) && --timeout);

	writel(JZ_BCH_ENABLE, nand->bch_base + JZ_REG_BCH_CR_CLR);

	if (timeout == 0)
		return -1;

	if (status & JZ_BCH_STATUS_ERROR) {
		if (status & JZ_BCH_STATUS_UNCOR)
			return -1;

		for(i = 0; i < 4; i++)
			error_regs[i] = readl(nand->bch_base + JZ_REG_BCH_ERR(i));

		/* All Ingenic processors are little-endian */
		error_indexes = (uint16_t*) error_regs;

		error_count = (status & JZ_BCH_STATUS_ERR_COUNT) >> 28;

		for (i = 0; i < error_count; i++) {
			index = error_indexes[i];

			if (index < size)
				dat[index >> 3] ^= (1 << (index & 0x7));
		}

		return error_count;
	}

	return 0;
}

#ifdef CONFIG_MTD_CMDLINE_PARTS
static const char *part_probes[] = {"cmdline", NULL};
#endif

static int jz_nand_ioremap_resource(struct platform_device *pdev,
	const char *name, struct resource **res, void __iomem **base)
{
	int ret;

	*res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!*res) {
		dev_err(&pdev->dev, "Failed to get platform %s memory\n", name);
		ret = -ENXIO;
		goto err;
	}

	*res = request_mem_region((*res)->start, resource_size(*res),
				pdev->name);
	if (!*res) {
		dev_err(&pdev->dev, "Failed to request %s memory region\n", name);
		ret = -EBUSY;
		goto err;
	}

	*base = ioremap((*res)->start, resource_size(*res));
	if (!*base) {
		dev_err(&pdev->dev, "Failed to ioremap %s memory region\n", name);
		ret = -EBUSY;
		goto err_release_mem;
	}

	return 0;

err_release_mem:
	release_mem_region((*res)->start, resource_size(*res));
err:
	*res = NULL;
	*base = NULL;
	return ret;
}

static int __devinit jz_nand_probe(struct platform_device *pdev)
{
	int ret;
	struct jz_nand *nand;
	struct nand_chip *chip;
	struct mtd_info *mtd;
	struct jz_nand_platform_data *pdata = pdev->dev.platform_data;
#ifdef CONFIG_MTD_PARTITIONS
	struct mtd_partition *partition_info;
	int num_partitions = 0;
#endif

	nand = kzalloc(sizeof(*nand), GFP_KERNEL);
	if (!nand) {
		dev_err(&pdev->dev, "Failed to allocate device structure.\n");
		return -ENOMEM;
	}

	ret = jz_nand_ioremap_resource(pdev, "mmio", &nand->mem, &nand->base);
	if (ret)
		goto err_free;

	ret = jz_nand_ioremap_resource(pdev, "bank", &nand->bank_mem,
			&nand->bank_base);
	if (ret)
		goto err_iounmap_mmio;

	ret = jz_nand_ioremap_resource(pdev, "bch", &nand->bch_mem,
			&nand->bch_base);
	if (ret)
		goto err_iounmap_bank;

	if (pdata && gpio_is_valid(pdata->busy_gpio)) {
		ret = gpio_request(pdata->busy_gpio, "NAND busy pin");
		if (ret) {
			dev_err(&pdev->dev,
				"Failed to request busy gpio %d: %d\n",
				pdata->busy_gpio, ret);
			goto err_iounmap_bch;
		}
	}

	nand->clk_bch = clk_get(NULL, "bch");
	if(IS_ERR(nand->clk_bch)) {
		ret = PTR_ERR(nand->clk_bch);
		dev_err(&pdev->dev, "Failed to request BCH clock: %d\n",
			ret);
		goto err_gpio_free;
	}

	mtd		= &nand->mtd;
	chip		= &nand->chip;
	mtd->priv	= chip;
	mtd->owner	= THIS_MODULE;
	mtd->name	= "jz4750-nand";

	chip->ecc.hwctl		= jz_nand_hwctl;
	chip->ecc.calculate	= jz_nand_calculate_ecc_bch;
	chip->ecc.correct	= jz_nand_correct_ecc_bch;
	chip->ecc.mode		= NAND_ECC_HW;
	chip->ecc.size		= 512;
	chip->ecc.bytes		= 13;

	if (pdata)
		chip->ecc.layout = pdata->ecc_layout;

	chip->chip_delay = 50;
	chip->cmd_ctrl = jz_nand_cmd_ctrl;

	if (pdata && gpio_is_valid(pdata->busy_gpio))
		chip->dev_ready = jz_nand_dev_ready;

	chip->IO_ADDR_R = nand->bank_base;
	chip->IO_ADDR_W = nand->bank_base;

	nand->pdata = pdata;
	platform_set_drvdata(pdev, nand);

	writel(JZ_NAND_CTRL_ENABLE_CHIP(0), nand->base + JZ_REG_NAND_CTRL);

	ret = nand_scan_ident(mtd, 1, NULL);
	if (ret) {
		dev_err(&pdev->dev,  "Failed to scan nand\n");
		goto err_clk_put;
	}

	if (pdata && pdata->ident_callback) {
		pdata->ident_callback(pdev, chip, &pdata->partitions,
					&pdata->num_partitions);
	}

	ret = nand_scan_tail(mtd);
	if (ret) {
		dev_err(&pdev->dev,  "Failed to scan nand\n");
		goto err_gpio_free;
	}

#ifdef CONFIG_MTD_PARTITIONS
#ifdef CONFIG_MTD_CMDLINE_PARTS
	num_partitions = parse_mtd_partitions(mtd, part_probes,
						&partition_info, 0);
#endif
	if (num_partitions <= 0 && pdata) {
		num_partitions = pdata->num_partitions;
		partition_info = pdata->partitions;
	}

	if (num_partitions > 0)
		ret = add_mtd_partitions(mtd, partition_info, num_partitions);
	else
#endif
	ret = add_mtd_device(mtd);

	if (ret) {
		dev_err(&pdev->dev, "Failed to add mtd device\n");
		goto err_nand_release;
	}

	clk_enable(nand->clk_bch);

	dev_info(&pdev->dev, "Successfully registered JZ4750 NAND driver\n");

	return 0;

err_nand_release:
	nand_release(&nand->mtd);
err_clk_put:
	clk_put(nand->clk_bch);
err_gpio_free:
	platform_set_drvdata(pdev, NULL);
	gpio_free(pdata->busy_gpio);
err_iounmap_bch:
	iounmap(nand->bch_base);
	release_mem_region(nand->bch_mem->start, resource_size(nand->bch_mem));
err_iounmap_bank:
	iounmap(nand->bank_base);
	release_mem_region(nand->bank_mem->start, resource_size(nand->bank_mem));
err_iounmap_mmio:
	iounmap(nand->base);
	release_mem_region(nand->mem->start, resource_size(nand->mem));
err_free:
	kfree(nand);
	return ret;
}

static int __devexit jz_nand_remove(struct platform_device *pdev)
{
	struct jz_nand *nand = platform_get_drvdata(pdev);

	nand_release(&nand->mtd);

	clk_disable(nand->clk_bch);
	clk_put(nand->clk_bch);

	/* Deassert and disable all chips */
	writel(0, nand->base + JZ_REG_NAND_CTRL);

	iounmap(nand->bch_base);
	release_mem_region(nand->bch_mem->start, resource_size(nand->bch_mem));
	iounmap(nand->bank_base);
	release_mem_region(nand->bank_mem->start, resource_size(nand->bank_mem));
	iounmap(nand->base);
	release_mem_region(nand->mem->start, resource_size(nand->mem));

	platform_set_drvdata(pdev, NULL);
	kfree(nand);

	return 0;
}

static struct platform_driver jz_nand_driver = {
	.probe = jz_nand_probe,
	.remove = __devexit_p(jz_nand_remove),
	.driver = {
		.name = "jz4750-nand",
		.owner = THIS_MODULE,
	},
};

static int __init jz_nand_init(void)
{
	return platform_driver_register(&jz_nand_driver);
}
module_init(jz_nand_init);

static void __exit jz_nand_exit(void)
{
	platform_driver_unregister(&jz_nand_driver);
}
module_exit(jz_nand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter Zotov <whitequark@whitequark.org>");
MODULE_DESCRIPTION("NAND controller driver for JZ4750 SoC");
MODULE_ALIAS("platform:jz4750-nand");
