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

#include <linux/printk.h>
#include <linux/io.h>
#include "pdc.h"
//#define PDC_PRINT_CFG

/*** pommard PDC driver re-use ***/
typedef struct lcdc_open_mini {
	unsigned int image_height;
	unsigned int image_width;
	unsigned int fb0_addr;
	unsigned int fb1_addr;
	unsigned int fb_stride;
	unsigned int write_back;
	unsigned int fbw_addr;
	unsigned int fb_format;
} lcdc_open_t;

typedef struct display_info_mini {
	unsigned int xres;
	unsigned int yres;
	unsigned int xres_virtual;
	unsigned int yres_virtual;
	unsigned int xoffset;
	unsigned int yoffset;
	unsigned int left_margin;
	unsigned int right_margin;
	unsigned int upper_margin;
	unsigned int lower_margin;
	unsigned int hsync_len;
	unsigned int vsync_len;
	unsigned int hsync_pol;
	unsigned int vsync_pol;
	unsigned int reverse;
} display_info_t;

typedef struct pdc_context_mini {
	int ch;
	lcdc_open_t *open;
	display_info_t *disp_info;
} pdc_context_t;

#define LCDC_FORMAT_RGBA8888 0
#define LCDC_FORMAT_RGB888 4
#define LCDC_FORMAT_ARGB8888 5 // note: incorrect for new PDC
#define LCDC_FORMAT_ABGR8888 6 // note: incorrect for new PDC

#define REG_H_COUNT_SIZE_MINUS_1 0x0000
#define REG_H_ADDR_TIME_START 0x0004
#define REG_H_RIGHT_BORDER 0x0008
#define REG_H_BLANK 0x000C
#define REG_H_SYNC 0x0010
#define REG_H_BACK_PORCH 0x0014
#define REG_H_LEFT_BORDER 0x0018
#define REG_H_INT_TIMING 0x001C
#define REG_H_DMA_TIMING 0x0020
#define REG_V_COUNT_SIZE_MINUS_1 0x0024
#define REG_V_ADDR_TIME_START 0x0028
#define REG_V_BOTTOM_BORDER 0x002C
#define REG_V_BLANK 0x0030
#define REG_V_SYNC 0x0034
#define REG_V_BACK_PORCH 0x0038
#define REG_V_TOP_BORDER 0x003C
#define REG_V_INT_TIMING 0x0040
#define REG_V_INC_H_VALUE 0x0044
#define REG_SIGNAL_POL 0x0048
#define REG_BORDER_COLOR 0x004C
#define REG_H_STATUS 0x0050
#define REG_V_STATUS 0x0054
#define REG_OUT_SIZE 0x005C
#define REG_PIC_BORDER_H 0x0060
#define REG_PIC_BORDER_V 0x0064
#define REG_FB0_ADDR 0x0068
#define REG_FB1_ADDR 0x006C
#define REG_MODE 0x0070
#define REG_START 0x0074
#define REG_SWAP 0x0078
#define REG_STATUS 0x007C
#define REG_RAM_ADDR 0x0080
#define REG_RAM_DATA 0x0084
#define REG_FB_SIZE 0x0090
#define REG_REG_UPDATE 0x009C
#define REG_BLOCK_MODE 0x00A0
#define REG_V_DMA_TIMING 0x00A4
#define REG_LINE_BUF_CONTROL 0x00A8
#define REG_Y_SCALE_FACTOR 0x00AC
#define REG_X_SCALE_FACTOR 0x00B0
#define REG_GLOBAL_EN 0x00C0
#define REG_GLOBAL_START 0x00C4
#define REG_GLOBAL_SWAP 0x00C8
#define REG_GLOBAL_RD_START 0x00CC
#define REG_FB_W 0x00D4
#define REG_LONG_LINEBUF 0x00E0

#define REG_ADDR(PA, OF) ((void __iomem *)(PA) + OF)
#define pio_write32(A, B)                                                      \
	iowrite32(B, REG_ADDR(pdc_addr, A));                                   \
	rwLog[l++] = A;                                                        \
	rwLog[l++] = B
#define pio_read32(A) ioread32(REG_ADDR(pdc_addr, A))

static void lcdc_set_parameters(pdc_context_t *ctx, void *pdc_addr)
{
	display_info_t *info;
	unsigned int val, val2;
	unsigned int rwLog[100];
	int l = 0;

	info = ctx->disp_info;

	/* NOTE  : the top/bottom/left/right border is to Zero.
               if these parameter is required, add member to display_info_t. */

	/* NOTE2 : the timing parameter of "display_info_t" is correponded to
               Linux FB. refer to "Documentation/fb/framebuffer.txt" for detail.
               the basic image is as following:
	           H : backporch(left margin) -> L_border=0  -> active image -> R_border=0 -> frontportch(right margin) -> hsync
	           V : backporch(upper margin)-> T_border=0  -> active image -> B_border=0 -> frontporch(lower_margin)  -> vsync   */

	/* total count */
	val = info->left_margin + info->xres + info->right_margin +
	      info->hsync_len;
	val2 = info->upper_margin + info->yres + info->lower_margin +
	       info->vsync_len;
	pio_write32(REG_H_COUNT_SIZE_MINUS_1, val - (info->reverse ? 1 : 1));
	pio_write32(REG_V_COUNT_SIZE_MINUS_1, val2 - (info->reverse ? 1 : 0));

	/* active start */
	if (info->reverse) {
		val = info->left_margin + info->right_margin + info->hsync_len;
		val2 = info->upper_margin + info->lower_margin +
		       info->vsync_len;
	} else {
		val = info->left_margin;
		val2 = info->upper_margin;
	}
	pio_write32(REG_H_ADDR_TIME_START, val);
	pio_write32(REG_V_ADDR_TIME_START, val2);

	/* border -- assumes to Zero, refer also to above NOTE */
	/* -- Right and Bottom : same as blank start, so set after */
	/* -- Left and Top : same as active start */
	pio_write32(REG_H_LEFT_BORDER, val);
	pio_write32(REG_V_TOP_BORDER, val2);

	/* blank start -- this means "front-porch start" */
	if (info->reverse) {
		val = info->left_margin + info->xres + info->right_margin +
		      info->hsync_len;
		val2 = info->upper_margin + info->yres + info->lower_margin +
		       info->vsync_len;
		pio_write32(REG_H_BLANK, 0);
		pio_write32(REG_H_RIGHT_BORDER, val - 1);
		pio_write32(REG_V_BLANK, 0);
		pio_write32(REG_V_BOTTOM_BORDER, val2 - 1);
	} else {
		val = info->left_margin + info->xres;
		val2 = info->upper_margin + info->yres;
		pio_write32(REG_H_BLANK, val);
		pio_write32(REG_H_RIGHT_BORDER, val);
		pio_write32(REG_V_BLANK, val2);
		pio_write32(REG_V_BOTTOM_BORDER, val2);
	}

	/* sync start */
	if (info->reverse) {
		val = info->right_margin;
		val2 = info->lower_margin;
	} else {
		val = info->left_margin + info->xres + info->right_margin;
		val2 = info->upper_margin + info->yres + info->lower_margin;
	}
	pio_write32(REG_H_SYNC, val);
	pio_write32(REG_V_SYNC, val2);

	/* interrupt timing -- same as sync */
	if (info->reverse) {
		val = 1 << 16;
		val2 = 1 << 16;
	} else {
		val = ((val + info->hsync_len) << 16) | val;
		val2 = ((val2 + info->vsync_len) << 16) | val2;
	}
	pio_write32(REG_H_INT_TIMING, val);
	pio_write32(REG_V_INT_TIMING, val2);

	/* DMA timing -- during 1 clock (for H) */
	val = 1 << 16;
	pio_write32(REG_H_DMA_TIMING, val);
	pio_write32(REG_V_DMA_TIMING, 10);

	/* back-porch start */
	if (info->reverse) {
		val = info->right_margin + info->hsync_len;
		val2 = info->lower_margin + info->vsync_len;
	} else {
		val = info->left_margin + info->xres + info->right_margin +
		      info->hsync_len;
		val2 = info->upper_margin + info->yres + info->lower_margin +
		       info->vsync_len;
	}
	pio_write32(REG_H_BACK_PORCH, val);
	pio_write32(REG_V_BACK_PORCH, val2);

	/* V increment timing -- set Hsync timing */
	if (info->reverse)
		val = 0;
	else
		val = info->left_margin + info->xres + info->right_margin;
	pio_write32(REG_V_INC_H_VALUE, val);

	/* signal polarity */
	val = (info->vsync_pol << 4) | info->hsync_pol;
	pio_write32(REG_SIGNAL_POL, val);

	/* Input image size */
	val = (ctx->open->image_height << 16) | ctx->open->image_width;
	pio_write32(REG_OUT_SIZE, val);

	/* pixel border */
	val = pio_read32(REG_H_ADDR_TIME_START); /* start */
	val2 = val + info->xres - (info->reverse ? 1 : 0); /* end   */
	if (info->xres != info->xres_virtual) {
		val = val + info->xoffset;
		val2 = val + info->xres_virtual;
	}
	pio_write32(REG_PIC_BORDER_H, (val2 << 16) | val);

	val = pio_read32(REG_V_ADDR_TIME_START); /* start */
	val2 = val + info->yres - (info->reverse ? 1 : 0); /* end   */
	if (info->yres != info->yres_virtual) {
		val = val + info->yoffset;
		val2 = val + info->yres_virtual;
	}
	pio_write32(REG_PIC_BORDER_V, (val2 << 16) | val);

	/* FB address, stride */
	pio_write32(REG_FB0_ADDR, ctx->open->fb0_addr);
	//pio_write32(REG_FB1_ADDR, ctx->open->fb1_addr);   /* unused in new PDC */
	pio_write32(REG_FB_SIZE, ctx->open->fb_stride);
	if (ctx->open->write_back && ctx->ch == 3)
		pio_write32(REG_FB_W, ctx->open->fbw_addr);

	/* Mode -- format, burst length, req interval */
	val = ctx->open->fb_format;
	if (val >=
	    LCDC_FORMAT_ARGB8888) /* need to swap -- set to other register */
		val = LCDC_FORMAT_RGBA8888;

	//val2 = 0x00080200; //(8 << 16) | (3 << 8) | (1 << 7) | val;
	val2 = ctx->open->fb_format;
	pio_write32(REG_MODE, val2);

#if 0
    /* Gamma table */
    lcdc_init_gamma_table(ctx);

    /* Block Mode */
    val = (ctx->open->scaling_filter << 2) | (ctx->open->block_format << 1);
    /* -- RGBA byte order */
    if (ctx->open->fb_format == LCDC_FORMAT_ARGB8888)
        val |= (1 << 18);
    else if (ctx->open->fb_format == LCDC_FORMAT_ABGR8888)
        val |= (3 << 18);
    /* -- Dither */
    if (ctx->open->dither)
        val |= ((1 << 26) | (1 << 25));

    if (ctx->open->write_back && ctx->ch == 3)
        val |= (1 << 8);

    pio_write32(REG_BLOCK_MODE, val);

    /* Scaling factor */
    if (info->xres_virtual != ctx->open->image_width) {
        /* int */
        val  = ctx->open->image_width / info->xres_virtual;
        /* flac */
        val2 = ((ctx->open->image_width % info->xres_virtual) << 20) /
                                                         info->xres_virtual;
        pio_write32(REG_X_SCALE_FACTOR, (val << 20) | val2);
    }
    if (info->yres_virtual != ctx->open->image_height) {
        /* int */
        val  = ctx->open->image_height / info->yres_virtual;
        /* flac */
        val2 = ((ctx->open->image_height % info->yres_virtual) << 20) /
                                                         info->yres_virtual;
        pio_write32(REG_Y_SCALE_FACTOR, (val << 20) | val2);
    }
#endif
#ifdef PDC_PRINT_CFG
	for (val = 0; val < l; val += 2) {
		printk(KERN_INFO "\treg: %04x_%08x\n", rwLog[val],
		       rwLog[val + 1]);
	}
#endif
}
#undef pio_write32
#undef pio_read32

static unsigned int IMAGE_WIDTH;
static unsigned int IMAGE_HEIGHT;
static unsigned int LEFT_MARGIN;
static unsigned int RIGHT_MARGIN;
static unsigned int HSYNC_LENGTH;
static unsigned int UPPER_MARGIN;
static unsigned int LOWER_MARGIN;
static unsigned int VSYNC_LENGTH;
static unsigned int SVAL;

#define DISP_VESAVGA 0
#define DISP_VESAXGA 1
#define DISP_SMALLDISP 2
#define DISP_CEA861FHD 3
#define DISP_VESA4K 4
#define DISP_FHD17 5
#define DISP_VESAXGA_RB 6
#define DISP_720p 7

void displaySetup(int dispType)
{
	if (dispType == DISP_VESAVGA) {
		IMAGE_WIDTH = 640;
		IMAGE_HEIGHT = 480;
		LEFT_MARGIN = 48; /* same as back porch (H) */
		RIGHT_MARGIN = 16; /* same as front porch (H) */
		HSYNC_LENGTH = 96;
		UPPER_MARGIN = 33; /* same as back porch (V) */
		LOWER_MARGIN = 10; /* same as front porch (V) */
		VSYNC_LENGTH = 2;
	} else if (dispType == DISP_VESAXGA) {
		IMAGE_WIDTH = 1024;
		IMAGE_HEIGHT = 768;
		LEFT_MARGIN = 160; /* same as back porch (H) */
		RIGHT_MARGIN = 24; /* same as front porch (H) */
		HSYNC_LENGTH = 136;
		UPPER_MARGIN = 29; /* same as back porch (V) */
		LOWER_MARGIN = 3; /* same as front porch (V) */
		VSYNC_LENGTH = 6;
	} else if (dispType == DISP_VESAXGA_RB) {
		IMAGE_WIDTH = 1024;
		IMAGE_HEIGHT = 768;
		LEFT_MARGIN = 40; /* same as back porch (H) */
		RIGHT_MARGIN = 8; /* same as front porch (H) */
		HSYNC_LENGTH = 42;
		UPPER_MARGIN = 6; /* same as back porch (V) */
		LOWER_MARGIN = 9; /* same as front porch (V) */
		VSYNC_LENGTH = 8;
	} else if (dispType == DISP_720p) {
		IMAGE_WIDTH = 1280;
		IMAGE_HEIGHT = 720;
		LEFT_MARGIN = 216; /* same as back porch (H) */
		RIGHT_MARGIN = 72; /* same as front porch (H) */
		HSYNC_LENGTH = 80;
		UPPER_MARGIN = 22; /* same as back porch (V) */
		LOWER_MARGIN = 3; /* same as front porch (V) */
		VSYNC_LENGTH = 5;
	} else if (dispType == DISP_SMALLDISP) {
		IMAGE_WIDTH = 800;
		IMAGE_HEIGHT = 480;
		LEFT_MARGIN = 96; /* same as back porch (H) */
		RIGHT_MARGIN = 24; /* same as front porch (H) */
		HSYNC_LENGTH = 72;
		UPPER_MARGIN = 7; /* same as back porch (V) */
		LOWER_MARGIN = 3; /* same as front porch (V) */
		VSYNC_LENGTH = 10;
	} else if (dispType == DISP_CEA861FHD) {
		IMAGE_WIDTH = 1920;
		IMAGE_HEIGHT = 1080;
		LEFT_MARGIN = 148; /* same as back porch (H) */
		RIGHT_MARGIN = 88; /* same as front porch (H) */
		HSYNC_LENGTH = 44;
		UPPER_MARGIN = 36; /* same as back porch (V) */
		LOWER_MARGIN = 4; /* same as front porch (V) */
		VSYNC_LENGTH = 5;
	} else if (dispType == DISP_VESA4K) {
		IMAGE_WIDTH = 3840;
		IMAGE_HEIGHT = 2160;
		LEFT_MARGIN = 560; /* same as back porch (H) */
		RIGHT_MARGIN = 176; /* same as front porch (H) */
		HSYNC_LENGTH = 88;
		UPPER_MARGIN = 90; /* same as back porch (V) */
		LOWER_MARGIN = 8; /* same as front porch (V) */
		VSYNC_LENGTH = 10;
	} else if (dispType == DISP_FHD17) {
		IMAGE_WIDTH = 1920;
		IMAGE_HEIGHT = 1080;
		LEFT_MARGIN = 124; /* same as back porch (H) */
		RIGHT_MARGIN = 68; /* same as front porch (H) */
		HSYNC_LENGTH = 44;
		UPPER_MARGIN = 36; /* same as back porch (V) */
		LOWER_MARGIN = 4; /* same as front porch (V) */
		VSYNC_LENGTH = 5;
	} else {
		IMAGE_WIDTH = 1920;
		IMAGE_HEIGHT = 1080;
		LEFT_MARGIN = 280; /* same as back porch (H) */ // onlap = 148
		RIGHT_MARGIN = 88; /* same as front porch (H) */ // onlap = 88
		HSYNC_LENGTH = 44; // onlap = 32
		UPPER_MARGIN = 45; /* same as back porch (V) */ //onlap 31
		LOWER_MARGIN = 4; /* same as front porch (V) */ //onlap 2
		VSYNC_LENGTH = 5; // default 5
	}
}

void pdc_config(void *pdc_addr, int *dims, unsigned int *fbPA)
{
	unsigned int tmp;
	int width, height, pol, bpp, dispType;
	lcdc_open_t Lopen;
	display_info_t Ldisp;
	pdc_context_t Lctx;

	width = dims[0];
	height = dims[1];
	//srcW=dims[2];   // not supported!
	//srcH=dims[3];   // not supported!
	pol = dims[4] & 7;
	SVAL = (dims[4] >> 4) & 0xf;
	bpp = (dims[4] >> 16) & 0xf; // max=4

	if ((width == 1024) && (height == 768) && (pol & 8))
		dispType = DISP_VESAXGA_RB;
	else if ((width == 1024) && (height == 768))
		dispType = DISP_VESAXGA;
	else if ((width == 1280) && (height == 720))
		dispType = DISP_720p;
	else
		dispType = DISP_VESAVGA; // TODO!

	//printk(KERN_INFO "\tPDC: allocated FB0@ VA=0x%08x PA=0x%08x\n",
	//	 (unsigned int)__va(fbPA[0]), fbPA[0]);
	//printk(KERN_INFO "\tPDC: allocated FB1@ VA=0x%08x PA=0x%08x\n",
	//	 (unsigned int)__va(fbPA[1]), fbPA[1]);
	printk(KERN_INFO "\tPDC: start reg config...\n");

	displaySetup(dispType);
	Lopen.fb_stride = IMAGE_WIDTH * bpp;
	Lopen.fb_format =
		(bpp == 4) ? LCDC_FORMAT_RGBA8888 : LCDC_FORMAT_RGB888;
	Lopen.image_width = IMAGE_WIDTH;
	Lopen.image_height = IMAGE_HEIGHT;
	Lopen.fb0_addr = fbPA[0];
	Lopen.fb1_addr = 0; /* unused in new PDC */
	Lopen.fbw_addr = 0;
	Lopen.write_back = 0;
	Ldisp.xres = IMAGE_WIDTH;
	Ldisp.yres = IMAGE_HEIGHT;
	Ldisp.xres_virtual = Ldisp.xres; /* same -- it means no scaling */
	Ldisp.yres_virtual = Ldisp.yres;
	Ldisp.xoffset = 0;
	Ldisp.yoffset = 0;
	Ldisp.left_margin = LEFT_MARGIN;
	Ldisp.right_margin = RIGHT_MARGIN;
	Ldisp.upper_margin = UPPER_MARGIN;
	Ldisp.lower_margin = LOWER_MARGIN;
	Ldisp.hsync_len = HSYNC_LENGTH;
	Ldisp.vsync_len = VSYNC_LENGTH;
	Ldisp.hsync_pol = (pol & 1) ? 1 : 0;
	Ldisp.vsync_pol = (pol & 2) ? 1 : 0;
	Ldisp.reverse = (pol & 4) ? 1 : 0;
	Lctx.open = &Lopen;
	Lctx.disp_info = &Ldisp;
	Lctx.ch = 0;
	lcdc_set_parameters(&Lctx, pdc_addr);

	/* note: we are currently using h/w with no gamma table */
#if 0
  iowrite32(0, REG_ADDR(pdc_addr,REG_RAM_ADDR));
  for(tmp=0; tmp<256; tmp++) {
    iowrite32((tmp<<16)|(tmp<<8)|tmp, pdc_addr+REG_RAM_DATA);
  }
#endif

	printk(KERN_INFO "\tPDC: reg config done\n");
	//sanity check:
	tmp = ioread32(REG_ADDR(pdc_addr, REG_OUT_SIZE));
	printk(KERN_INFO "\tPDC: reg read sanity check: DIM=0x%08x\n", tmp);
	tmp = ioread32(REG_ADDR(pdc_addr, REG_FB0_ADDR));
	printk(KERN_INFO "\tPDC: reg read sanity check: FB0=0x%08x\n", tmp);
}

void pdc_start(void *pdc_addr)
{
	//iowrite32(0x100003, pdc_addr+0x94);
	iowrite32(SVAL | 0x1, REG_ADDR(pdc_addr, REG_START));
}

void pdc_stop(void *pdc_addr)
{
	iowrite32(0, REG_ADDR(pdc_addr, REG_START));
}
