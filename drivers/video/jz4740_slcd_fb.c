/*
 *  Copyright (C) 2009-2010, Lars-Peter Clausen <lars@metafoo.de>
 *  Copyright (C) 2010, Maarten ter Huurne <maarten@treewalker.org>
 *		JZ4720/JZ4740 SoC LCD framebuffer driver
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>

#include <linux/clk.h>
#include <linux/delay.h>

#include <linux/console.h>
#include <linux/fb.h>

#include <linux/dma-mapping.h>

#include <asm/mach-jz4740/dma.h>
#include <asm/mach-jz4740/gpio.h>
#include <asm/mach-jz4740/jz4740_fb.h>

#include "jz4740_lcd.h"
#include "jz4740_slcd.h"

struct jzfb_framedesc {
	uint32_t next;
	uint32_t addr;
	uint32_t id;
	uint32_t cmd;
} __attribute__((packed));

static struct fb_fix_screeninfo jzfb_fix __devinitdata = {
	.id =		"JZ4740 SLCD FB",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.xpanstep =	0,
	.ypanstep =	1,
	.ywrapstep =	0,
	.accel =	FB_ACCEL_NONE,
};

const static struct jz_gpio_bulk_request jz_slcd_ctrl_pins[] = {
	JZ_GPIO_BULK_PIN(LCD_PCLK),
	JZ_GPIO_BULK_PIN(SLCD_RS),
	JZ_GPIO_BULK_PIN(SLCD_CS),
};

const static struct jz_gpio_bulk_request jz_slcd_data_pins[] = {
	JZ_GPIO_BULK_PIN(LCD_DATA0),
	JZ_GPIO_BULK_PIN(LCD_DATA1),
	JZ_GPIO_BULK_PIN(LCD_DATA2),
	JZ_GPIO_BULK_PIN(LCD_DATA3),
	JZ_GPIO_BULK_PIN(LCD_DATA4),
	JZ_GPIO_BULK_PIN(LCD_DATA5),
	JZ_GPIO_BULK_PIN(LCD_DATA6),
	JZ_GPIO_BULK_PIN(LCD_DATA7),
	JZ_GPIO_BULK_PIN(LCD_DATA8),
	JZ_GPIO_BULK_PIN(LCD_DATA9),
	JZ_GPIO_BULK_PIN(LCD_DATA10),
	JZ_GPIO_BULK_PIN(LCD_DATA11),
	JZ_GPIO_BULK_PIN(LCD_DATA12),
	JZ_GPIO_BULK_PIN(LCD_DATA13),
	JZ_GPIO_BULK_PIN(LCD_DATA14),
	JZ_GPIO_BULK_PIN(LCD_DATA15),
	JZ_GPIO_BULK_PIN(LCD_DATA16),
	JZ_GPIO_BULK_PIN(LCD_DATA17),
};

static unsigned int jzfb_num_ctrl_pins(struct jzfb *jzfb)
{
	return ARRAY_SIZE(jz_slcd_ctrl_pins);
}

static unsigned int jzfb_num_data_pins(struct jzfb *jzfb)
{
	switch (jzfb->pdata->lcd_type) {
	case JZ_LCD_TYPE_SMART_PARALLEL_8_BIT:
		return 8;
	case JZ_LCD_TYPE_SMART_PARALLEL_16_BIT:
		return 16;
	case JZ_LCD_TYPE_SMART_PARALLEL_18_BIT:
		return 18;
	default:
		return 0;
	}
}

static void jzfb_free_gpio_pins(struct jzfb *jzfb)
{
	jz_gpio_bulk_free(jz_slcd_ctrl_pins, jzfb_num_ctrl_pins(jzfb));
	if (jzfb->pdata->lcd_type & (1 << 6)) {
		/* serial */
		jz_gpio_bulk_free(&jz_slcd_data_pins[15], 1);
	} else {
		/* parallel */
		jz_gpio_bulk_free(jz_slcd_data_pins,
				  jzfb_num_data_pins(jzfb));
	}
}

static int jzfb_setcolreg(unsigned regno, unsigned red, unsigned green,
			unsigned blue, unsigned transp, struct fb_info *fb)
{
	if (regno >= 16)
		return -EINVAL;

	red   = (red   * ((1 << fb->var.red.length  ) - 1)) / ((1 << 16) - 1);
	green = (green * ((1 << fb->var.green.length) - 1)) / ((1 << 16) - 1);
	blue  = (blue  * ((1 << fb->var.blue.length ) - 1)) / ((1 << 16) - 1);

	((uint32_t *)fb->pseudo_palette)[regno] =
		(red   << fb->var.red.offset  ) |
		(green << fb->var.green.offset) |
		(blue  << fb->var.blue.offset );

	return 0;
}

static int jzfb_get_controller_bpp(struct jzfb *jzfb)
{
	switch (jzfb->pdata->bpp) {
	case 18:
	case 24:
		return 32;
	case 15:
		return 16;
	default:
		return jzfb->pdata->bpp;
	}
}

static struct fb_videomode *jzfb_get_mode(struct jzfb *jzfb, struct fb_var_screeninfo *var)
{
	size_t i;
	struct fb_videomode *mode = jzfb->pdata->modes;

	for (i = 0; i < jzfb->pdata->num_modes; ++i, ++mode) {
		if (mode->xres == var->xres && mode->yres == var->yres)
			return mode;
	}

	return NULL;
}

static int jzfb_check_var(struct fb_var_screeninfo *var, struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;
	struct fb_videomode *mode;

	if (var->bits_per_pixel != jzfb_get_controller_bpp(jzfb) &&
		var->bits_per_pixel != jzfb->pdata->bpp)
		return -EINVAL;

	mode = jzfb_get_mode(jzfb, var);
	if (mode == NULL)
		return -EINVAL;

	fb_videomode_to_var(var, mode);

	/* Reserve space for double buffering. */
	var->yres_virtual = var->yres * 2;

	switch (jzfb->pdata->bpp) {
	case 8:
		break;
	case 15:
		var->red.offset = 10;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 5;
		var->blue.offset = 0;
		var->blue.length = 5;
		break;
	case 16:
		var->red.offset = 11;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 6;
		var->blue.offset = 0;
		var->blue.length = 5;
		break;
	case 18:
		var->red.offset = 16;
		var->red.length = 6;
		var->green.offset = 8;
		var->green.length = 6;
		var->blue.offset = 0;
		var->blue.length = 6;
		var->bits_per_pixel = 32;
		break;
	case 32:
	case 24:
		var->transp.offset = 24;
		var->transp.length = 8;
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->bits_per_pixel = 32;
		break;
	default:
		break;
	}

	return 0;
}

static void jzfb_disable_dma(struct jzfb *jzfb)
{
	jz4740_dma_disable(jzfb->dma);
	while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
	writeb(readb(jzfb->base + JZ_REG_SLCD_CTRL) & ~SLCD_CTRL_DMA_EN,
		jzfb->base + JZ_REG_SLCD_CTRL);
}

static struct jz4740_dma_config jzfb_slcd_dma_config = {
	.src_width = JZ4740_DMA_WIDTH_32BIT,
	.dst_width = JZ4740_DMA_WIDTH_16BIT,
	.transfer_size = JZ4740_DMA_TRANSFER_SIZE_16BYTE,
	.request_type = JZ4740_DMA_TYPE_SLCD,
	.flags = JZ4740_DMA_SRC_AUTOINC,
	.mode = JZ4740_DMA_MODE_BLOCK,
};

static void jzfb_upload_frame_dma(struct jzfb *jzfb)
{
	struct fb_info *fb = jzfb->fb;
	struct fb_videomode *mode = fb->mode;
	__u32 offset = fb->fix.line_length * fb->var.yoffset;
	__u32 size = fb->fix.line_length * mode->yres;

	/* Ensure that the data to be uploaded is in memory. */
	dma_cache_sync(fb->device, jzfb->vidmem + offset, size,
		       DMA_TO_DEVICE);

	jz4740_dma_set_src_addr(jzfb->dma, jzfb->vidmem_phys + offset);
	jz4740_dma_set_dst_addr(jzfb->dma,
				CPHYSADDR(jzfb->base + JZ_REG_SLCD_FIFO));
	jz4740_dma_set_transfer_count(jzfb->dma, size);

	while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
	writeb(readb(jzfb->base + JZ_REG_SLCD_CTRL) | SLCD_CTRL_DMA_EN,
		jzfb->base + JZ_REG_SLCD_CTRL);
	jz4740_dma_enable(jzfb->dma);
}

static void jzfb_upload_frame_cpu(struct jzfb *jzfb)
{
	const int num_pixels = jzfb->fb->mode->xres * jzfb->fb->mode->yres;
	uint16_t *p = jzfb->vidmem;
	int i;

	jzfb_disable_dma(jzfb);
	for (i = 0; i < num_pixels; i++) {
		uint16_t rgb = *p++;
		while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
		writel(SLCD_DATA_RS_DATA | rgb, jzfb->base + JZ_REG_SLCD_DATA);
	}
}

static void jzfb_refresh_work(struct work_struct *work)
{
	struct jzfb *jzfb = container_of(work, struct jzfb, refresh_work.work);

	mutex_lock(&jzfb->lock);
	if (jzfb->is_enabled) {
		if (1) {
			jzfb_upload_frame_dma(jzfb);
			/* The DMA complete callback will reschedule. */
		} else {
			jzfb_upload_frame_cpu(jzfb);
			schedule_delayed_work(&jzfb->refresh_work, HZ / 10);
		}
	}
	mutex_unlock(&jzfb->lock);
}

static void jzfb_refresh_work_complete(
		struct jz4740_dma_chan *dma, int res, void *dev)
{
	struct jzfb *jzfb = dev_get_drvdata(dev);
	// TODO: Stick to refresh rate in mode description.
	int interval = HZ / 60;

	schedule_delayed_work(&jzfb->refresh_work, interval);
}

static int jzfb_set_par(struct fb_info *info)
{
	struct jzfb *jzfb = info->par;
	struct fb_var_screeninfo *var = &info->var;
	struct fb_videomode *mode;
	uint16_t slcd_cfg;

	mode = jzfb_get_mode(jzfb, var);
	if (mode == NULL)
		return -EINVAL;

	info->mode = mode;

	slcd_cfg = SLCD_CFG_BURST_8_WORD;
	/* command size */
	slcd_cfg |= (jzfb->pdata->lcd_type & 3) << SLCD_CFG_CWIDTH_BIT;
	/* data size */
	if (jzfb->pdata->lcd_type & (1 << 6)) {
		/* serial */
		unsigned int num_bits;
		switch (jzfb->pdata->lcd_type) {
		case JZ_LCD_TYPE_SMART_SERIAL_8_BIT:
			slcd_cfg |= SLCD_CFG_DWIDTH_8_x1;
			num_bits = 8;
			break;
		case JZ_LCD_TYPE_SMART_SERIAL_16_BIT:
			slcd_cfg |= SLCD_CFG_DWIDTH_16;
			num_bits = 16;
			break;
		case JZ_LCD_TYPE_SMART_SERIAL_18_BIT:
			slcd_cfg |= SLCD_CFG_DWIDTH_18;
			num_bits = 18;
			break;
		default:
			num_bits = 0;
			break;
		}
		if (num_bits != jzfb->pdata->bpp) {
			dev_err(&jzfb->pdev->dev,
				"Data size (%d) does not match bpp (%d)\n",
				num_bits, jzfb->pdata->bpp);
		}
		slcd_cfg |= SLCD_CFG_TYPE_SERIAL;
	} else {
		/* parallel */
		switch (jzfb->pdata->bpp) {
		case 8:
			slcd_cfg |= SLCD_CFG_DWIDTH_8_x1;
			break;
		case 15:
		case 16:
			switch (jzfb->pdata->lcd_type) {
			case JZ_LCD_TYPE_SMART_PARALLEL_8_BIT:
				slcd_cfg |= SLCD_CFG_DWIDTH_8_x2;
				break;
			default:
				slcd_cfg |= SLCD_CFG_DWIDTH_16;
				break;
			}
			break;
		case 18:
			switch (jzfb->pdata->lcd_type) {
			case JZ_LCD_TYPE_SMART_PARALLEL_8_BIT:
				slcd_cfg |= SLCD_CFG_DWIDTH_8_x3;
				break;
			case JZ_LCD_TYPE_SMART_PARALLEL_16_BIT:
				slcd_cfg |= SLCD_CFG_DWIDTH_9_x2;
				break;
			case JZ_LCD_TYPE_SMART_PARALLEL_18_BIT:
				slcd_cfg |= SLCD_CFG_DWIDTH_18;
				break;
			default:
				break;
			}
			break;
		case 24:
			slcd_cfg |= SLCD_CFG_DWIDTH_8_x3;
			break;
		default:
			dev_err(&jzfb->pdev->dev,
				"Unsupported value for bpp: %d\n",
				jzfb->pdata->bpp);
		}
		slcd_cfg |= SLCD_CFG_TYPE_PARALLEL;
	}
	if (!jzfb->pdata->chip_select_active_low)
		slcd_cfg |= SLCD_CFG_CS_ACTIVE_HIGH;
	if (!jzfb->pdata->register_select_active_low)
		slcd_cfg |= SLCD_CFG_RS_CMD_HIGH;
	if (!jzfb->pdata->pixclk_falling_edge)
		slcd_cfg |= SLCD_CFG_CLK_ACTIVE_RISING;

#if 0
	// TODO(MtH): Compute rate from refresh or vice versa.
	if (mode->pixclock) {
		rate = PICOS2KHZ(mode->pixclock) * 1000;
		mode->refresh = rate / vt / ht;
	} else {
		if (jzfb->pdata->lcd_type == JZ_LCD_TYPE_8BIT_SERIAL)
			rate = mode->refresh * (vt + 2 * mode->xres) * ht;
		else
			rate = mode->refresh * vt * ht;

		mode->pixclock = KHZ2PICOS(rate / 1000);
	}
#endif

	mutex_lock(&jzfb->lock);
	if (!jzfb->is_enabled)
		clk_enable(jzfb->ldclk);

	// TODO(MtH): We should not change config while DMA might be running.
	writew(slcd_cfg, jzfb->base + JZ_REG_SLCD_CFG);

	if (!jzfb->is_enabled)
		clk_disable(jzfb->ldclk);
	mutex_unlock(&jzfb->lock);

	// TODO(MtH): Use maximum transfer speed that panel can handle.
	//            ILI9325 can do 10 MHz.
	clk_set_rate(jzfb->lpclk, 12000000);
	clk_set_rate(jzfb->ldclk, 42000000);

	return 0;
}

static void jzfb_enable(struct jzfb *jzfb)
{
	uint32_t ctrl;

	clk_enable(jzfb->ldclk);

	jz_gpio_bulk_resume(jz_slcd_ctrl_pins, jzfb_num_ctrl_pins(jzfb));
	if (jzfb->pdata->lcd_type & (1 << 6)) {
		/* serial */
		jz_gpio_bulk_resume(&jz_slcd_data_pins[15], 1);
	} else {
		/* parallel */
		jz_gpio_bulk_resume(jz_slcd_data_pins,
				    jzfb_num_data_pins(jzfb));
	}
	jzfb_disable_dma(jzfb);
	jzfb->panel->enable(jzfb);

	ctrl = readl(jzfb->base + JZ_REG_LCD_CTRL);
	ctrl |= JZ_LCD_CTRL_ENABLE;
	ctrl &= ~JZ_LCD_CTRL_DISABLE;
	writel(ctrl, jzfb->base + JZ_REG_LCD_CTRL);

	schedule_delayed_work(&jzfb->refresh_work, 0);
}

static void jzfb_disable(struct jzfb *jzfb)
{
	/* It is safe but wasteful to call refresh_work() while disabled. */
	cancel_delayed_work(&jzfb->refresh_work);

	/* Abort any DMA transfer that might be in progress and allow direct
	   writes to the panel. */
	jzfb_disable_dma(jzfb);

	jzfb->panel->disable(jzfb);
	jz_gpio_bulk_suspend(jz_slcd_ctrl_pins, jzfb_num_ctrl_pins(jzfb));
	if (jzfb->pdata->lcd_type & (1 << 6)) {
		/* serial */
		jz_gpio_bulk_suspend(&jz_slcd_data_pins[15], 1);
	} else {
		/* parallel */
		jz_gpio_bulk_suspend(jz_slcd_data_pins,
				     jzfb_num_data_pins(jzfb));
	}

	clk_disable(jzfb->ldclk);
}

static int jzfb_blank(int blank_mode, struct fb_info *info)
{
	struct jzfb *jzfb = info->par;
	int ret = 0;
	int new_enabled = (blank_mode == FB_BLANK_UNBLANK);

	mutex_lock(&jzfb->lock);
	if (new_enabled) {
		if (!jzfb->is_enabled)
			jzfb_enable(jzfb);
	} else {
		if (jzfb->is_enabled) {
			/* No sleep in TV-out mode. */
			if (readl(jzfb->base + JZ_REG_LCD_CFG) & JZ_LCD_CFG_SLCD)
				jzfb_disable(jzfb);
			else
				ret = -EBUSY;
		}
	}
	if (!ret)
		jzfb->is_enabled = new_enabled;
	mutex_unlock(&jzfb->lock);

	return ret;
}

static int jzfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct jzfb *jzfb = info->par;

	info->var.yoffset = var->yoffset;
	/* update frame start address for TV-out mode */
	jzfb->framedesc->addr = jzfb->vidmem_phys
	                      + info->fix.line_length * var->yoffset;

	return 0;
}

static int jzfb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	const unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	const unsigned long size = vma->vm_end - vma->vm_start;

	if (offset + size > info->fix.smem_len)
		return -EINVAL;

	if (remap_pfn_range(vma, vma->vm_start,
			    (info->fix.smem_start + offset) >> PAGE_SHIFT,
			    size, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static int jzfb_alloc_devmem(struct jzfb *jzfb)
{
	int max_framesize = 0;
	struct fb_videomode *mode = jzfb->pdata->modes;
	void *page;
	int i;

	for (i = 0; i < jzfb->pdata->num_modes; ++mode, ++i) {
		if (max_framesize < mode->xres * mode->yres)
			max_framesize = mode->xres * mode->yres;
	}

	max_framesize *= jzfb_get_controller_bpp(jzfb) >> 3;

	jzfb->framedesc = dma_alloc_coherent(&jzfb->pdev->dev,
				    sizeof(*jzfb->framedesc),
				    &jzfb->framedesc_phys, GFP_KERNEL);

	if (!jzfb->framedesc)
		return -ENOMEM;

	/* reserve memory for two frames to allow double buffering */
	jzfb->vidmem_size = PAGE_ALIGN(max_framesize * 2);
	jzfb->vidmem = dma_alloc_coherent(&jzfb->pdev->dev,
						jzfb->vidmem_size,
						&jzfb->vidmem_phys, GFP_KERNEL);

	if (!jzfb->vidmem)
		goto err_free_framedesc;

	for (page = jzfb->vidmem;
		 page < jzfb->vidmem + PAGE_ALIGN(jzfb->vidmem_size);
		 page += PAGE_SIZE) {
		SetPageReserved(virt_to_page(page));
	}

	jzfb->framedesc->next = jzfb->framedesc_phys;
	jzfb->framedesc->addr = jzfb->vidmem_phys;
	jzfb->framedesc->id = 0xdeafbead;
	jzfb->framedesc->cmd = 0;
	jzfb->framedesc->cmd |= max_framesize / 4;

	return 0;

err_free_framedesc:
	dma_free_coherent(&jzfb->pdev->dev, sizeof(*jzfb->framedesc),
				jzfb->framedesc, jzfb->framedesc_phys);
	return -ENOMEM;
}

static void jzfb_free_devmem(struct jzfb *jzfb)
{
	dma_free_coherent(&jzfb->pdev->dev, jzfb->vidmem_size,
				jzfb->vidmem, jzfb->vidmem_phys);
	dma_free_coherent(&jzfb->pdev->dev, sizeof(*jzfb->framedesc),
				jzfb->framedesc, jzfb->framedesc_phys);
}

#include "jz4740_lcd.h"

#define FBIOA320TVOUT 0x46F0
#define FB_A320TV_OFF 0
#define FB_A320TV_NTSC 1
#define FB_A320TV_PAL 2

static void jzfb_tv_out(struct jzfb *jzfb, unsigned int mode)
{
	int blank = jzfb->is_enabled ? FB_BLANK_UNBLANK : FB_BLANK_POWERDOWN;
	struct fb_event event = {
		.info = jzfb->fb,
		.data = &blank,
	};

	printk("A320 TV out: %d\n", mode);

	if (mode != FB_A320TV_OFF) {
		cancel_delayed_work(&jzfb->refresh_work);
		/* Abort any DMA transfer that might be in progress and
		   allow direct writes to the panel.  */
		jzfb_disable_dma(jzfb);
		jzfb->panel->disable(jzfb);

		/* set up LCD controller for TV output */

		writel(JZ_LCD_CFG_HSYNC_ACTIVE_LOW |
		       JZ_LCD_CFG_VSYNC_ACTIVE_LOW,
		       jzfb->base + JZ_REG_LCD_CFG);

		/* V-Sync pulse end position */
		writel(10, jzfb->base + JZ_REG_LCD_VSYNC);

		if (mode == FB_A320TV_PAL) {
			/* PAL */
			/* H-Sync pulse start position */
			writel(125, jzfb->base + JZ_REG_LCD_HSYNC);
			/* virtual area size */
			writel(0x036c0112, jzfb->base + JZ_REG_LCD_VAT);
			/* horizontal start/end point */
			writel(0x02240364, jzfb->base + JZ_REG_LCD_DAH);
			/* vertical start/end point */
			writel(0x1b010b, jzfb->base + JZ_REG_LCD_DAV);
		}
		else {
			/* NTSC */
			writel(0x3c, jzfb->base + JZ_REG_LCD_HSYNC);
			writel(0x02e00110, jzfb->base + JZ_REG_LCD_VAT);
			writel(0x019902d9, jzfb->base + JZ_REG_LCD_DAH);
			writel(0x1d010d, jzfb->base + JZ_REG_LCD_DAV);
		}
		writel(0, jzfb->base + JZ_REG_LCD_PS);
		writel(0, jzfb->base + JZ_REG_LCD_CLS);
		writel(0, jzfb->base + JZ_REG_LCD_SPL);
		writel(0, jzfb->base + JZ_REG_LCD_REV);
		/* reset status register */
		writel(0, jzfb->base + JZ_REG_LCD_STATE);

		/* tell LCDC about the frame descriptor address */
		writel(jzfb->framedesc_phys, jzfb->base + JZ_REG_LCD_DA0);

		writel(JZ_LCD_CTRL_BURST_16 | JZ_LCD_CTRL_ENABLE |
		       JZ_LCD_CTRL_BPP_15_16,
		       jzfb->base + JZ_REG_LCD_CTRL);
	}
	else {
		/* disable LCD controller and re-enable SLCD */
		writel(JZ_LCD_CFG_SLCD, jzfb->base + JZ_REG_LCD_CFG);
		jzfb->panel->enable(jzfb);
		schedule_delayed_work(&jzfb->refresh_work, 0);
	}

	/* reaffirm the current blanking state, to trigger a backlight update */
	fb_notifier_call_chain(FB_EVENT_BLANK, &event);
}

static int jzfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct jzfb *jzfb = info->par;
	switch (cmd) {
		case FBIOA320TVOUT:
			/* No TV-out mode while sleeping. */
			if (!jzfb->is_enabled)
				return -EBUSY;

			jzfb_tv_out(jzfb, arg);
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

static struct fb_ops jzfb_ops = {
	.owner			= THIS_MODULE,
	.fb_check_var 		= jzfb_check_var,
	.fb_set_par 		= jzfb_set_par,
	.fb_setcolreg		= jzfb_setcolreg,
	.fb_blank		= jzfb_blank,
	.fb_pan_display		= jzfb_pan_display,
	.fb_fillrect		= sys_fillrect,
	.fb_copyarea		= sys_copyarea,
	.fb_imageblit		= sys_imageblit,
	.fb_ioctl		= jzfb_ioctl,
	.fb_mmap		= jzfb_mmap,
};

static int __devinit jzfb_probe(struct platform_device *pdev)
{
	int ret;
	struct jzfb *jzfb;
	struct fb_info *fb;
	struct jz4740_fb_platform_data *pdata = pdev->dev.platform_data;
	struct resource *mem;

	if (!pdata) {
		dev_err(&pdev->dev, "Missing platform data\n");
		return -ENOENT;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!mem) {
		dev_err(&pdev->dev, "Failed to get register memory resource\n");
		return -ENOENT;
	}

	mem = request_mem_region(mem->start, resource_size(mem), pdev->name);

	if (!mem) {
		dev_err(&pdev->dev, "Failed to request register memory region\n");
		return -EBUSY;
	}

	fb = framebuffer_alloc(sizeof(struct jzfb), &pdev->dev);

	if (!fb) {
		dev_err(&pdev->dev, "Failed to allocate framebuffer device\n");
		ret = -ENOMEM;
		goto err_release_mem_region;
	}

	fb->fbops = &jzfb_ops;
	fb->flags = FBINFO_DEFAULT;

	jzfb = fb->par;
	jzfb->pdev = pdev;
	jzfb->pdata = pdata;
	jzfb->mem = mem;

	jzfb->dma = jz4740_dma_request(&pdev->dev, dev_name(&pdev->dev));
	if (!jzfb->dma) {
		dev_err(&pdev->dev, "Failed to get DMA channel\n");
		ret = -EBUSY;
		goto err_framebuffer_release;
	}
	jz4740_dma_configure(jzfb->dma, &jzfb_slcd_dma_config);
	jz4740_dma_set_complete_cb(jzfb->dma, &jzfb_refresh_work_complete);

	jzfb->ldclk = clk_get(&pdev->dev, "lcd");
	if (IS_ERR(jzfb->ldclk)) {
		ret = PTR_ERR(jzfb->ldclk);
		dev_err(&pdev->dev, "Failed to get lcd clock: %d\n", ret);
		goto err_free_dma;
	}

	jzfb->lpclk = clk_get(&pdev->dev, "lcd_pclk");
	if (IS_ERR(jzfb->lpclk)) {
		ret = PTR_ERR(jzfb->lpclk);
		dev_err(&pdev->dev, "Failed to get lcd pixel clock: %d\n", ret);
		goto err_put_ldclk;
	}

	jzfb->base = ioremap(mem->start, resource_size(mem));

	if (!jzfb->base) {
		dev_err(&pdev->dev, "Failed to ioremap register memory region\n");
		ret = -EBUSY;
		goto err_put_lpclk;
	}

	platform_set_drvdata(pdev, jzfb);

	fb_videomode_to_modelist(pdata->modes, pdata->num_modes,
				 &fb->modelist);
	fb->mode = pdata->modes;

	fb_videomode_to_var(&fb->var, fb->mode);
	fb->var.bits_per_pixel = pdata->bpp;
	jzfb_check_var(&fb->var, fb);

	ret = jzfb_alloc_devmem(jzfb);
	if (ret) {
		dev_err(&pdev->dev, "Failed to allocate video memory\n");
		goto err_iounmap;
	}

	fb->fix = jzfb_fix;
	fb->fix.line_length = fb->var.bits_per_pixel * fb->var.xres / 8;
	fb->fix.mmio_start = mem->start;
	fb->fix.mmio_len = resource_size(mem);
	fb->fix.smem_start = jzfb->vidmem_phys;
	fb->fix.smem_len =  fb->fix.line_length * fb->var.yres_virtual;
	fb->screen_base = jzfb->vidmem;
	fb->pseudo_palette = jzfb->pseudo_palette;

	fb_alloc_cmap(&fb->cmap, 256, 0);

	mutex_init(&jzfb->lock);

	clk_enable(jzfb->ldclk);
	jzfb->is_enabled = 1;

	writel(JZ_LCD_CFG_SLCD, jzfb->base + JZ_REG_LCD_CFG);
	writeb(0, jzfb->base + JZ_REG_SLCD_CTRL);

	jzfb_set_par(fb);

	jz_gpio_bulk_request(jz_slcd_ctrl_pins, jzfb_num_ctrl_pins(jzfb));
	if (jzfb->pdata->lcd_type & (1 << 6)) {
		/* serial */
		jz_gpio_bulk_request(&jz_slcd_data_pins[15], 1);
	} else {
		/* parallel */
		jz_gpio_bulk_request(jz_slcd_data_pins,
				     jzfb_num_data_pins(jzfb));
	}

	jzfb->panel = jz_slcd_panels_probe(jzfb);
	if (!jzfb->panel) {
		dev_err(&pdev->dev, "Failed to find panel driver\n");
		ret = -ENOENT;
		goto err_free_devmem;
	}
	jzfb_disable_dma(jzfb);
	jzfb->panel->init(jzfb);
	jzfb->panel->enable(jzfb);

	ret = register_framebuffer(fb);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register framebuffer: %d\n", ret);
		goto err_free_panel;
	}

	jzfb->fb = fb;
	fb_prepare_logo(jzfb->fb, 0);
	fb_show_logo(jzfb->fb, 0);

	INIT_DELAYED_WORK(&jzfb->refresh_work, jzfb_refresh_work);
	schedule_delayed_work(&jzfb->refresh_work, 0);

	return 0;

err_free_panel:
	jzfb->panel->exit(jzfb);
err_free_devmem:
	jzfb_free_gpio_pins(jzfb);

	fb_dealloc_cmap(&fb->cmap);
	jzfb_free_devmem(jzfb);
err_iounmap:
	iounmap(jzfb->base);
err_put_lpclk:
	clk_put(jzfb->lpclk);
err_put_ldclk:
	clk_put(jzfb->ldclk);
err_free_dma:
	jz4740_dma_free(jzfb->dma);
err_framebuffer_release:
	framebuffer_release(fb);
err_release_mem_region:
	release_mem_region(mem->start, resource_size(mem));
	return ret;
}

static int __devexit jzfb_remove(struct platform_device *pdev)
{
	struct jzfb *jzfb = platform_get_drvdata(pdev);

	jzfb_blank(FB_BLANK_POWERDOWN, jzfb->fb);

	/* Blanking will prevent future refreshes from behind scheduled.
	   Now wait for a possible refresh in progress to finish. */
	cancel_delayed_work_sync(&jzfb->refresh_work);

	jzfb->panel->exit(jzfb);

	jzfb_free_gpio_pins(jzfb);

	jz4740_dma_free(jzfb->dma);

	iounmap(jzfb->base);
	release_mem_region(jzfb->mem->start, resource_size(jzfb->mem));

	fb_dealloc_cmap(&jzfb->fb->cmap);
	jzfb_free_devmem(jzfb);

	platform_set_drvdata(pdev, NULL);

	clk_put(jzfb->lpclk);
	clk_put(jzfb->ldclk);

	framebuffer_release(jzfb->fb);

	return 0;
}

#ifdef CONFIG_PM

static int jzfb_suspend(struct device *dev)
{
	struct jzfb *jzfb = dev_get_drvdata(dev);

	console_lock();
	fb_set_suspend(jzfb->fb, 1);
	console_unlock();

	mutex_lock(&jzfb->lock);
	if (jzfb->is_enabled)
		jzfb_disable(jzfb);
	mutex_unlock(&jzfb->lock);

	return 0;
}

static int jzfb_resume(struct device *dev)
{
	struct jzfb *jzfb = dev_get_drvdata(dev);
	clk_enable(jzfb->ldclk);

	mutex_lock(&jzfb->lock);
	if (jzfb->is_enabled)
		jzfb_enable(jzfb);
	mutex_unlock(&jzfb->lock);

	console_lock();
	fb_set_suspend(jzfb->fb, 0);
	console_unlock();

	return 0;
}

static const struct dev_pm_ops jzfb_pm_ops = {
	.suspend	= jzfb_suspend,
	.resume		= jzfb_resume,
	.poweroff	= jzfb_suspend,
	.restore	= jzfb_resume,
};

#define JZFB_PM_OPS (&jzfb_pm_ops)

#else
#define JZFB_PM_OPS NULL
#endif

static struct platform_driver jzfb_driver = {
	.probe		= jzfb_probe,
	.remove		= __devexit_p(jzfb_remove),
	.driver = {
		.name	= "jz4740-fb",
		.pm	= JZFB_PM_OPS,
	},
};

static int __init jzfb_init(void)
{
	return platform_driver_register(&jzfb_driver);
}
module_init(jzfb_init);

static void __exit jzfb_exit(void)
{
	platform_driver_unregister(&jzfb_driver);
}
module_exit(jzfb_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>, Maarten ter Huurne <maarten@treewalker.org>");
MODULE_DESCRIPTION("JZ4740 SoC SLCD framebuffer driver");
MODULE_ALIAS("platform:jz4740-fb");
