/*
 *  PDC FB kernel driver
 *
 *  Copyright (C) 2018  Digital Media Professionals Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/stddef.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/fb.h>
#include "pdc.h"

#define USE_DEVTREE
#ifndef USE_DEVTREE
static unsigned int reg_prop[] = { 0x43c10000, 0x100 };
static int irq_prop = 50;
#endif

#define REG_ADDR(PA, OF) ((void __iomem *)(PA) + OF)
#define ALLOC_SIZE() (width * height * ((bpp == 32) ? 4 : 3) * 2)
#define FB_DEV_NAME "dmp_fb"
#define FB_NUM_SUBDEV 1

#define PALETTE_ENTRIES_NO	16
static int RED_SHIFT = 16;
static int GREEN_SHIFT = 8;
static int BLUE_SHIFT = 0;

// module arguments: these can be modified on module loading:
// e.g: insmod **_km.ko width=1280 height=720
static int width = 1280;
static int height = 720;
static int pol = 4;
static int bpp = 24;
module_param(width, int, 0644);
module_param(height, int, 0644);
module_param(pol, int, 0644);
module_param(bpp, int, 0644);
MODULE_PARM_DESC(width, "frame buffer width");
MODULE_PARM_DESC(height, "frame buffer height");
MODULE_PARM_DESC(pol, "sync polarity");
MODULE_PARM_DESC(bpp, "specify bits-per-pixel (default=24)");

struct fb_subdev {
	int init_done;
	spinlock_t int_lock;
	wait_queue_head_t wait_queue;
	int wait_status;
	int pan_display;
	int irq;
	phys_addr_t bar_physical;
	size_t reg_size;
	void *bar_logical;

	dma_addr_t fb_pa;
	void *fb_la;
	struct fb_info info;
	u32 pseudo_palette[PALETTE_ENTRIES_NO];
};

struct fb_dev {
	struct device *dev;
	struct fb_subdev subdev[FB_NUM_SUBDEV];
};

static irqreturn_t handle_int(int irq, void *p)
{
	unsigned int rd_data;
	dma_addr_t next_fba;
	struct fb_subdev *subdev = (struct fb_subdev *)p;

	rd_data = ioread32(REG_ADDR(subdev->bar_logical, PDC_REG_STATUS));
	if (rd_data & 0x8000)
		pr_info(FB_DEV_NAME "WARNING, VINT w/ UINT!\n");

	// for VINT version: clr VINT|UNIT (bits [17],[18])
	iowrite32(0x060000, REG_ADDR(subdev->bar_logical, PDC_REG_SWAP));

	if (subdev->pan_display) {
		next_fba = subdev->fb_pa +
			   (subdev->info.var.yoffset * subdev->info.var.xres *
			    (subdev->info.var.bits_per_pixel >> 3));

		iowrite32(0x8 | 1, REG_ADDR(subdev->bar_logical, 0x94));
		iowrite32(next_fba,
			  REG_ADDR(subdev->bar_logical, PDC_REG_FBADDR));
		// wait for FB addr update consume:
		rd_data = 1;
		while (rd_data & 1)
			rd_data = ioread32(REG_ADDR(subdev->bar_logical, 0x94));
		subdev->pan_display = 0;
	}

	spin_lock(&subdev->int_lock);
	if (subdev->wait_status == 1) { // user waiting for int/swap
		subdev->wait_status = 2;
		wake_up_interruptible(&subdev->wait_queue);
	}
	spin_unlock(&subdev->int_lock);

	return IRQ_HANDLED;
}

static void wait_int(struct fb_subdev *subdev)
{
	unsigned long irq_save = 0;
	spin_lock_irqsave(&subdev->int_lock, irq_save);
	subdev->wait_status = 1;
	spin_unlock_irqrestore(&subdev->int_lock, irq_save);

	if (!wait_event_interruptible(subdev->wait_queue,
				      (subdev->wait_status & 2))) {
		spin_lock_irqsave(&subdev->int_lock, irq_save);
		subdev->wait_status = 0;
		spin_unlock_irqrestore(&subdev->int_lock, irq_save);
	}
}

static int dvfb_setcolreg(unsigned int regno, unsigned int red,
			  unsigned int green, unsigned int blue,
			  unsigned int transp, struct fb_info *info)
{
	u32 *palette = info->pseudo_palette;

	if (regno >= PALETTE_ENTRIES_NO)
		return -EINVAL;

	if (info->var.grayscale) {
		// Convert color to grayscale.
		// grayscale = 0.30*R + 0.59*G + 0.11*B
		blue = (red * 77 + green * 151 + blue * 28 + 127) >> 8;
		green = blue;
		red = green;
	}

	// Only handle 8 bits of each color
	red >>= 8;
	green >>= 8;
	blue >>= 8;
	palette[regno] = (red << RED_SHIFT) | (green << GREEN_SHIFT) |
			 (blue << BLUE_SHIFT);

	return 0;
}

static int dvfb_blank(int blank, struct fb_info *info)
{
	struct fb_subdev *subdev = container_of(info, struct fb_subdev, info);

	switch (blank) {
	case FB_BLANK_UNBLANK:
		/* turn on display */
		pdc_start(subdev->bar_logical);
		break;

	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		/* turn off display */
		pdc_stop(subdev->bar_logical);
		break;
	default:
		break;
	}

	return 0;
}

static int dvfb_pan_display(struct fb_var_screeninfo *var,
			    struct fb_info *info)
{
	struct fb_subdev *subdev = container_of(info, struct fb_subdev, info);

	if ((var->xoffset != 0) ||
	    (var->yoffset + info->var.yres > info->var.yres_virtual))
		return -EINVAL;
	
	info->var.yoffset = var->yoffset;
	subdev->pan_display = 1;
	return 0;
}

static int dvfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	unsigned int ret = 0;
	struct fb_subdev *subdev = container_of(info, struct fb_subdev, info);

	switch (cmd) {
	case FBIO_WAITFORVSYNC: {
		wait_int(subdev);
		ret = 0;
		break;
	}
	default:
		break;
	}

	return ret;
}

static const struct fb_fix_screeninfo dvfb_fix = {
	.id =		FB_DEV_NAME,
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.xpanstep =	0,
	.ypanstep =	1,
	.ywrapstep =	0, 
	.accel =	FB_ACCEL_NONE,
};

static const struct fb_var_screeninfo dvfb_var = {
	.red =		{ 0, 8, 0 },
	.green =	{ 0, 8, 0 },
	.blue =		{ 0, 8, 0 },
	.transp =	{ 0, 0, 0 },
	.activate =	FB_ACTIVATE_NOW
};

static struct fb_ops dvfb_ops = {
	.owner			= THIS_MODULE,
	.fb_setcolreg		= dvfb_setcolreg,
	.fb_blank		= dvfb_blank,
	.fb_pan_display		= dvfb_pan_display,
	.fb_fillrect		= cfb_fillrect,
	.fb_copyarea		= cfb_copyarea,
	.fb_imageblit		= cfb_imageblit,
	.fb_ioctl		= dvfb_ioctl,
};

static int allocate_fb(struct fb_dev *fb_dev)
{
	size_t alloc_size = ALLOC_SIZE();

	if (dma_set_mask_and_coherent(fb_dev->dev, DMA_BIT_MASK(32))) {
		dev_err(fb_dev->dev, "No suitable DMA available.\n");
		return -ENOMEM;
	}

	fb_dev->subdev[0].fb_la = dma_alloc_coherent(
		fb_dev->dev, alloc_size, &fb_dev->subdev[0].fb_pa, GFP_KERNEL);
	if (!fb_dev->subdev[0].fb_la) {
		dev_err(fb_dev->dev, "Can not allocate frame buffer memory.\n");
		return -ENOMEM;
	}
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	if ((unsigned long long)fb_dev->subdev[0].fb_pa + alloc_size >
	    4294967296ull) {
		dma_free_coherent(
			fb_dev->dev, alloc_size, fb_dev->subdev[0].fb_la,
			fb_dev->subdev[0].fb_pa);
		dev_err(fb_dev->dev,
			"dma_addr=%llu alloc_size=%llu lies in high memory\n",
			(unsigned long long)fb_dev->subdev[0].fb_pa,
			(unsigned long long)alloc_size);
		return -ENOMEM;
	}
#endif
	pr_info(FB_DEV_NAME ": frame buffer mem allocated at PA: 0x%08x\n",
		(unsigned int)fb_dev->subdev[0].fb_pa);

	return 0;
}

static void free_fb(struct fb_dev *fb_dev)
{
	size_t alloc_size = ALLOC_SIZE();
	dma_free_coherent(fb_dev->dev, alloc_size, fb_dev->subdev[0].fb_la,
			  fb_dev->subdev[0].fb_pa);
}

static int pdc_init(struct fb_dev *fb_dev)
{
	struct fb_subdev *subdev = &fb_dev->subdev[0];
	int pdc_dim[5];
	unsigned int fb_pa[2];
	int ret;

	pdc_dim[0] = width;
	pdc_dim[1] = height;
	pdc_dim[2] = width;
	pdc_dim[3] = height;
	pdc_dim[4] = (pol & 0xffff) | ((bpp == 32) ? 0x40000 : 0x30000);

	if (0 != allocate_fb(fb_dev)) {
		dev_err(fb_dev->dev, "Failed to allocate FB buffer.\n");
		return -ENOMEM;
	}
	fb_pa[0] = subdev->fb_pa & 0xffffffff;
	fb_pa[1] = fb_pa[0] + (width * height * ((bpp == 32) ? 4 : 3));

	pdc_config(subdev->bar_logical, pdc_dim, fb_pa);
	pdc_start(subdev->bar_logical);

	// set channel offset
	if (bpp == 32) {
		RED_SHIFT = 24;
		GREEN_SHIFT = 16;
		BLUE_SHIFT = 8;
	}
	// fill in fb_info
	subdev->info.device = fb_dev->dev;
	subdev->info.screen_base = (void __iomem *)subdev->fb_la;
	subdev->info.fbops = &dvfb_ops;
	subdev->info.pseudo_palette = subdev->pseudo_palette;
	subdev->info.flags = FBINFO_DEFAULT | FBINFO_HWACCEL_YPAN;

	subdev->info.fix = dvfb_fix;
	subdev->info.fix.smem_start = subdev->fb_pa;
	subdev->info.fix.smem_len = ALLOC_SIZE();
	subdev->info.fix.line_length = width * ((bpp == 32) ? 4 : 3);

	subdev->info.var = dvfb_var;
	subdev->info.var.xres = width;
	subdev->info.var.yres = height;
	subdev->info.var.xres_virtual = width;
	subdev->info.var.yres_virtual = height * 2;
	subdev->info.var.xoffset = 0;
	subdev->info.var.yoffset = 0;
	subdev->info.var.red.offset = RED_SHIFT;
	subdev->info.var.green.offset = GREEN_SHIFT;
	subdev->info.var.blue.offset = BLUE_SHIFT;
	subdev->info.var.bits_per_pixel = bpp;
	subdev->info.var.grayscale = 0;

	// allocate color map
	ret = fb_alloc_cmap(&subdev->info.cmap, PALETTE_ENTRIES_NO, 0);
	if (ret) {
		dev_err(fb_dev->dev, "Fail to allocate colormap (%d entries)\n",
			PALETTE_ENTRIES_NO);
		free_fb(fb_dev);
		return ret;
	}
	
	// register new frame buffer
	ret = register_framebuffer(&subdev->info);
	if (ret) {
		dev_err(fb_dev->dev, "Could not register frame buffer\n");
		fb_dealloc_cmap(&subdev->info.cmap);
		free_fb(fb_dev);
		return ret;
	}

	return 0;
}

static int dvfb_probe(struct platform_device *pdev,
		      struct device_node *dev_node)
{
	struct fb_dev *fb_dev;
	u32 reg_base;
	u32 reg_size;
	int i, ret, irq;

	fb_dev = devm_kzalloc(&pdev->dev, sizeof(struct fb_dev), GFP_KERNEL);
	if (!fb_dev) {
		dev_err(&pdev->dev, "Failed to allocate device data.\n");
		return -ENOMEM;
	}
	fb_dev->dev = &(pdev->dev);
	platform_set_drvdata(pdev, fb_dev);

#ifdef USE_DEVTREE
	{
		struct device_node *parent_node;
		u32 addr_cells;
		unsigned int reg_index;
		parent_node = of_get_parent(dev_node);
		if (parent_node) {
			of_property_read_u32(parent_node, "#address-cells",
					     &addr_cells);
			reg_index = addr_cells - 1;
			of_node_put(parent_node);
			of_property_read_u32_index(dev_node, "reg", reg_index,
						   &reg_base);
			reg_index += addr_cells;
			of_property_read_u32_index(dev_node, "reg", reg_index,
						   &reg_size);
		} else {
			of_property_read_u32_index(dev_node, "reg", 0,
						   &reg_base);
			of_property_read_u32_index(dev_node, "reg", 1,
						   &reg_size);
		}
	}
#else
	reg_base = reg_prop[0];
	reg_size = reg_prop[1];
#endif
	pr_info(FB_DEV_NAME ": reg base=0x%08x size=0x%08x\n", reg_base,
		reg_size);

	fb_dev->subdev[0].wait_status = 0;
	fb_dev->subdev[0].bar_physical = reg_base;
	fb_dev->subdev[0].reg_size = reg_size;
	fb_dev->subdev[0].bar_logical = ioremap_nocache(reg_base, reg_size);
	if (!fb_dev->subdev[0].bar_logical) {
		dev_err(&pdev->dev, "ioremap_nocache failed.\n");
		ret = -EBUSY;
		goto fail_ioremap;
	}

	for (i = 0; i < FB_NUM_SUBDEV; i++) {
#ifdef USE_DEVTREE
		irq = of_irq_get(dev_node, 0);
#else
		irq = irq_prop;
#endif
		fb_dev->subdev[i].irq = irq;
		ret = request_irq(irq, handle_int, IRQF_SHARED, FB_DEV_NAME,
				  &fb_dev->subdev[i]);
		if (ret != 0) {
			dev_err(&pdev->dev, "request_irq failed %d (%d).\n",
				irq, i);
			goto fail_device_init;
		}

		init_waitqueue_head(&(fb_dev->subdev[i].wait_queue));
		spin_lock_init(&(fb_dev->subdev[i].int_lock));
		fb_dev->subdev[i].init_done = 1;
	}

	// pdc specific initialization:
	ret = pdc_init(fb_dev);
	if (ret) {
		dev_err(&pdev->dev, "pdc_init failed.\n");
		goto fail_device_init;
	}

	return 0;

fail_device_init:
	for (i = 0; i < FB_NUM_SUBDEV; i++) {
		if (fb_dev->subdev[i].init_done) {
			free_irq(fb_dev->subdev[i].irq, &(fb_dev->subdev[i]));
			fb_dev->subdev[i].init_done = 0;
		}
	}
	iounmap(fb_dev->subdev[0].bar_logical);
fail_ioremap:
	kfree(fb_dev);
	return ret;
}

static int dev_probe(struct platform_device *pdev)
{
	int err = 0;
	struct device_node *dev_node = NULL;

#ifdef USE_DEVTREE
	dev_node = pdev->dev.of_node;
#endif

	err = dvfb_probe(pdev, dev_node);
	if (0 != err) {
		dev_err(&pdev->dev, "dvfb_probe failed.\n");
		return err;
	}

	return 0;
}

static int dev_remove(struct platform_device *pdev)
{
	struct fb_dev *fb_dev;
	int i;

	fb_dev = platform_get_drvdata(pdev);

	if (fb_dev) {
		unregister_framebuffer(&fb_dev->subdev[0].info);
		fb_dealloc_cmap(&fb_dev->subdev[0].info.cmap);
		free_fb(fb_dev);

		for (i = 0; i < FB_NUM_SUBDEV; i++) {
			if (fb_dev->subdev[i].init_done) {
				free_irq(fb_dev->subdev[i].irq,
					 &(fb_dev->subdev[i]));
				fb_dev->subdev[i].init_done = 0;
			}
			if (fb_dev->subdev[i].bar_logical) {
				iounmap(fb_dev->subdev[i].bar_logical);
			}
		}

		platform_set_drvdata(pdev, NULL);
	}

	return 0;
}

#ifdef USE_DEVTREE
static const struct of_device_id fb_of_ids[] = {
	{ .compatible = "dmp,pdc" },
	{ }
};
#endif

static struct platform_driver fb_platform_driver = {
	.probe = dev_probe,
	.remove = dev_remove,
	.driver =
		{
			.name = FB_DEV_NAME,
			.owner = THIS_MODULE,
#ifdef USE_DEVTREE
			.of_match_table = fb_of_ids,
#endif
		},
};

#ifndef USE_DEVTREE
static void dev_release(struct device *dev)
{
}

static struct platform_device fb_platform_device = {
	.name = FB_DEV_NAME,
	.id = -1,
	.num_resources = 0,
	.resource = NULL,
	.dev =
		{
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &fb_platform_device.dev.coherent_dma_mask,
			.release = dev_release,
		},
};
#endif

static int __init drm_init(void)
{
	int ret;
	ret = platform_driver_register(&fb_platform_driver);
	if (ret)
		return ret;

#ifndef USE_DEVTREE
	ret = platform_device_register(&fb_platform_device);
	if (ret) {
		platform_driver_unregister(&fb_platform_driver);
		return ret;
	}
#endif

	return ret;
}

static void __exit drm_exit(void)
{
#ifndef USE_DEVTREE
	platform_device_unregister(&fb_platform_device);
#endif
	platform_driver_unregister(&fb_platform_driver);
}

module_init(drm_init);
module_exit(drm_exit);
MODULE_DESCRIPTION("simplistic frame buffer");
MODULE_AUTHOR("Digital Media Professionals Inc.");
MODULE_VERSION("7.0.20181214");
MODULE_LICENSE("GPL");
