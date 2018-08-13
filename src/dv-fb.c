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
#include <linux/cdev.h>
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
#include <uapi/linux/fb.h>
#include "pdc.h"

#define REG_ADDR(PA, OF) ((void __iomem *)(PA) + OF)
#define FB_DEV_NAME "dmp_fb"
#define FB_NUM_SUBDEV 1

// module arguments: these can be modified on module loading:
// e.g: insmod **_km.ko width=1280 height=720
static int width = 1280;
static int height = 720;
static int pol = 0;
static int bpp = 32;
static int fbn = 0;
module_param(width, int, 0644);
module_param(height, int, 0644);
module_param(pol, int, 0644);
module_param(bpp, int, 0644);
module_param(fbn, int, 0644);
MODULE_PARM_DESC(width, "frame buffer width");
MODULE_PARM_DESC(height, "frame buffer height");
MODULE_PARM_DESC(pol, "sync polarity");
MODULE_PARM_DESC(bpp, "specify bits-per-pixel (default=32)");
MODULE_PARM_DESC(fbn, "device node num to create (/dev/fbN (default = fb0)");

struct fb_subdev {
	int init_done;
	spinlock_t int_lock;
	wait_queue_head_t wait_queue;
	int wait_status;
	int irq;
	phys_addr_t bar_physical;
	size_t reg_size;
	void *bar_logical;

	dma_addr_t fb_pa;
	void *fb_la;
	struct fb_var_screeninfo fb_sci;
};

struct fb_dev {
	struct device *dev;
	dev_t devt;
	struct cdev cdev;
	struct fb_subdev subdev[FB_NUM_SUBDEV];
};

static struct class *fb_class = NULL;

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

	spin_lock(&subdev->int_lock);
	if (subdev->wait_status == 1) { // user waiting for int/swap
		next_fba = subdev->fb_pa +
			   (subdev->fb_sci.yoffset * subdev->fb_sci.xres *
			    (subdev->fb_sci.bits_per_pixel >> 3));

		iowrite32(0x8 | 1, REG_ADDR(subdev->bar_logical, 0x94));
		iowrite32(next_fba,
			  REG_ADDR(subdev->bar_logical, PDC_REG_FBADDR));
		// wait for FB addr update consume:
		rd_data = 1;
		while (rd_data & 1)
			rd_data = ioread32(REG_ADDR(subdev->bar_logical, 0x94));
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

static int fb_open(struct inode *inode, struct file *file)
{
	struct fb_dev *fb_dev =
		container_of(inode->i_cdev, struct fb_dev, cdev);
	file->private_data = &fb_dev->subdev[iminor(inode)];
	return 0;
}

static int fb_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long fb_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned int ret = 0;
	struct fb_subdev *subdev = file->private_data;
	unsigned int __user *ptr = (unsigned int __user *)arg;
	struct fb_var_screeninfo fb_sci;

	switch (cmd) {
	case FBIOGET_VSCREENINFO: {
		ret = copy_to_user(ptr, &(subdev->fb_sci),
				   sizeof(struct fb_var_screeninfo));
		break;
	}
	case FBIOPAN_DISPLAY: { // assume that caller has reset yoffset
		ret = copy_from_user(&fb_sci, ptr,
				     sizeof(struct fb_var_screeninfo));
		if (ret == 0)
			subdev->fb_sci.yoffset = fb_sci.yoffset;
		break;
	}
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

static int fb_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct fb_subdev *subdev = file->private_data;
	unsigned long map_size = vma->vm_end - vma->vm_start;
	if (map_size != width * height * 2 * ((bpp == 32) ? 4 : 3))
		return -EINVAL;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	return io_remap_pfn_range(vma, vma->vm_start,
				  (subdev->fb_pa >> PAGE_SHIFT), map_size,
				  vma->vm_page_prot);
}

static struct file_operations fb_file_ops = {
	.owner = THIS_MODULE,
	.open = fb_open,
	.release = fb_release,
	.unlocked_ioctl = fb_ioctl,
	.mmap = fb_mmap,
};

static int fb_allocate_buffer(struct fb_dev *fb_dev)
{
	size_t alloc_size = width * height * ((bpp == 32) ? 4 : 3) * 2;

	if (dma_set_mask_and_coherent(fb_dev->dev, DMA_BIT_MASK(32))) {
		dev_err(fb_dev->dev, "No suitable DMA available.\n");
		return -ENOMEM;
	}

	fb_dev->subdev[0].fb_la = dma_alloc_coherent(
		fb_dev->dev, alloc_size, &fb_dev->subdev[0].fb_pa, GFP_KERNEL);
	if (!fb_dev->subdev[0].fb_la) {
		dev_err(fb_dev->dev, "Can not allocate frame buffer memory.\n");
		return 1;
	}

	pr_info(FB_DEV_NAME ": frame buffer mem allocated at PA: 0x%08x\n",
		fb_dev->subdev[0].fb_pa);

	return 0;
}

static int pdc_init(struct fb_dev *fb_dev)
{
	struct fb_subdev *subdev = &fb_dev->subdev[0];
	int pdc_dim[5];
	dma_addr_t fb_pa[2];

	pdc_dim[0] = width;
	pdc_dim[1] = height;
	pdc_dim[2] = width;
	pdc_dim[3] = height;
	pdc_dim[4] = (pol & 0xffff) | ((bpp == 32) ? 0x40000 : 0x30000);

	// fill in supported areas of fb_screen_info:
	subdev->fb_sci.xres = width;
	subdev->fb_sci.yres = height;
	subdev->fb_sci.xres_virtual = width;
	subdev->fb_sci.yres_virtual = height * 2;
	subdev->fb_sci.xoffset = 0;
	subdev->fb_sci.yoffset = 0;
	subdev->fb_sci.bits_per_pixel = bpp;
	subdev->fb_sci.grayscale = 0;

	if (0 != fb_allocate_buffer(fb_dev)) {
		dev_err(fb_dev->dev, "Failed to allocate FB buffer.\n");
		return -ENOMEM;
	}
	fb_pa[0] = subdev->fb_pa;
	fb_pa[1] = fb_pa[0] + (width * height * ((bpp == 32) ? 4 : 3));

	pdc_config(subdev->bar_logical, pdc_dim, fb_pa);
	pdc_start(subdev->bar_logical);

	return 0;
}

static int fb_probe(struct platform_device *pdev, struct device_node *dev_node)
{
	struct fb_dev *fb_dev;
	const unsigned int *prop = NULL;
	phys_addr_t reg_base;
	size_t reg_size;
	int i, ret, pbytes, dev_major, irq;

	fb_dev = kzalloc(sizeof(struct fb_dev), GFP_KERNEL);
	if (!fb_dev) {
		dev_err(&pdev->dev, "Failed to allocate device data.\n");
		return -ENOMEM;
	}
	fb_dev->dev = &(pdev->dev);
	platform_set_drvdata(pdev, fb_dev);

	prop = of_get_property(dev_node, "reg", &pbytes);
	if ((prop == NULL) || (pbytes < 8)) {
		dev_err(&pdev->dev, "reg property not found\n");
		ret = -ENODEV;
		goto fail_get_property;
	}
	// note that device-tree property data is big-endian:
	reg_base = be32_to_cpup(prop);
	reg_size = be32_to_cpup(prop + 1);
	pr_info(FB_DEV_NAME ": reg base=0x%08x size=0x%08x (prop=%d bytes)\n",
		reg_base, reg_size, pbytes);

	fb_dev->subdev[0].wait_status = 0;
	fb_dev->subdev[0].bar_physical = reg_base;
	fb_dev->subdev[0].reg_size = reg_size;
	fb_dev->subdev[0].bar_logical = ioremap_nocache(reg_base, reg_size);
	if (!fb_dev->subdev[0].bar_logical) {
		dev_err(&pdev->dev, "ioremap_nocache failed.\n");
		ret = -EBUSY;
		goto fail_ioremap;
	}

	// create & register character device(s):
	ret = alloc_chrdev_region(&fb_dev->devt, 0, FB_NUM_SUBDEV, FB_DEV_NAME);
	if (ret) {
		dev_err(&pdev->dev, "alloc_chrdev_region failed.\n");
		goto fail_alloc_chrdev_region;
	}

	fb_class = class_create(THIS_MODULE, FB_DEV_NAME);
	if (IS_ERR(fb_class)) {
		dev_err(&pdev->dev, "class_create failed.\n");
		ret = PTR_ERR(fb_class);
		goto fail_class_create;
	}

	dev_major = MAJOR(fb_dev->devt);

	cdev_init(&fb_dev->cdev, &fb_file_ops);
	ret = cdev_add(&fb_dev->cdev, fb_dev->devt, FB_NUM_SUBDEV);
	if (ret) {
		dev_err(&pdev->dev, "cdev_add failed.\n");
		goto fail_cdev_add;
	}

	for (i = 0; i < FB_NUM_SUBDEV; i++) {
		struct device *dev;
		dev = device_create(fb_class, NULL, MKDEV(dev_major, i), fb_dev,
				    "fb%d", fbn + i);
		if (IS_ERR(dev)) {
			dev_err(&pdev->dev, "device_create fail\n");
			ret = PTR_ERR(dev);
			goto fail_device_init;
		}

		irq = of_irq_get(dev_node, 0);
		fb_dev->subdev[i].irq = irq;
		ret = request_irq(irq, handle_int, IRQF_SHARED, FB_DEV_NAME,
				  &fb_dev->subdev[i]);
		if (ret != 0) {
			dev_err(&pdev->dev, "request_irq failed %d (%d).\n",
				irq, i);
			device_destroy(fb_class, MKDEV(dev_major, i));
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
			device_destroy(fb_class, MKDEV(dev_major, i));
			fb_dev->subdev[i].init_done = 0;
		}
	}
	cdev_del(&fb_dev->cdev);
fail_cdev_add:
	class_destroy(fb_class);
fail_class_create:
	unregister_chrdev_region(fb_dev->devt, FB_NUM_SUBDEV);
fail_alloc_chrdev_region:
	iounmap(fb_dev->subdev[0].bar_logical);
fail_ioremap:
fail_get_property:
	kfree(fb_dev);
	return ret;
}

static int dev_probe(struct platform_device *pdev)
{
	int err = 0;
	struct device_node *dev_node;

	dev_node = of_find_compatible_node(NULL, NULL, "DMP_fb,DMP_fb");
	if (dev_node == NULL) {
		dev_err(&pdev->dev, "No compatible node found.\n");
		return -ENODEV;
	} else {
		of_node_put(dev_node);
	}

	err = fb_probe(pdev, dev_node);
	if (0 != err) {
		dev_err(&pdev->dev, "fb_probe failed.\n");
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
		unsigned int driver_major = MAJOR(fb_dev->devt);
		size_t alloc_size = width * height * ((bpp == 32) ? 4 : 3) * 2;

		dma_free_coherent(&pdev->dev, alloc_size,
				  fb_dev->subdev[0].fb_la,
				  fb_dev->subdev[0].fb_pa);

		for (i = 0; i < FB_NUM_SUBDEV; i++) {
			if (fb_dev->subdev[i].init_done) {
				free_irq(fb_dev->subdev[i].irq,
					 &(fb_dev->subdev[i]));
				device_destroy(fb_class,
					       MKDEV(driver_major, i));
				fb_dev->subdev[i].init_done = 0;
			}
		}

		cdev_del(&fb_dev->cdev);
		class_destroy(fb_class);
		unregister_chrdev_region(fb_dev->devt, FB_NUM_SUBDEV);
		for (i = 0; i < FB_NUM_SUBDEV; i++) {
			if (fb_dev->subdev[i].bar_logical) {
				iounmap(fb_dev->subdev[i].bar_logical);
			}
		}

		platform_set_drvdata(pdev, NULL);
		kfree(fb_dev);
	}

	return 0;
}

static struct platform_driver fb_platform_driver = {
	.probe = dev_probe,
	.remove = dev_remove,
	.driver =
		{
			.name = FB_DEV_NAME,
			.owner = THIS_MODULE,
		},
};

static void dev_release(struct device *dev)
{
}

static u64 drm_dma_mask;
static struct platform_device fb_platform_device = {
	.name = FB_DEV_NAME,
	.id = -1,
	.num_resources = 0,
	.resource = NULL,
	.dev =
		{
			.dma_mask = &drm_dma_mask,
			.release = dev_release,
		},
};

static int __init drm_init(void)
{
	int ret;
	ret = platform_driver_register(&fb_platform_driver);
	if (ret)
		return ret;

	ret = platform_device_register(&fb_platform_device);
	if (ret)
		platform_driver_unregister(&fb_platform_driver);

	return ret;
}

static void __exit drm_exit(void)
{
	platform_device_unregister(&fb_platform_device);
	platform_driver_unregister(&fb_platform_driver);
}

module_init(drm_init);
module_exit(drm_exit);
MODULE_DESCRIPTION("simplistic frame buffer");
MODULE_AUTHOR("Digital Media Professionals Inc.");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
