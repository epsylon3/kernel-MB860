/*
 * drivers/video/tegra-fb.c
 *
 * Dumb framebuffer driver for NVIDIA Tegra SoCs
 *
 * Copyright (C) 2009 - 2010 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <asm/cacheflush.h>
#include <mach/iomap.h>
#include <mach/nvrm_linux.h>
#include <mach/iomap.h>
#include "nvcommon.h"
#include "nvos.h"
#include "nvcolor.h"
#include "nvbootargs.h"
#include "nvrm_module.h"
#include "nvrm_memmgr.h"
#include "nvrm_power.h"
#include "nvrm_ioctls.h"
#ifdef CONFIG_MACH_MOT
#include <asm/bootinfo.h>
#endif

#define DISPLAY_BASE    (TEGRA_DISPLAY_BASE)
//#define DISPLAY_BASE    (0x54200000)
//#define DISPLAY_BASE    (TEGRA_DISPLAY2_BASE)

#include <video/tegrafb.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <mach/nvhost.h>
#include <mach/nvmap.h>

#undef HAVE_NVMAP_CLIENT
#define ENABLE_HDMI 0

#define LCD_WIDTH  540
#define LCD_HEIGHT 960

#define LCD_BYTES  2 /* 2 x 8 bits = 16bpp */

#if 0
static struct fb_info tegra_fb_def = {
	.fix = {
		.id		= "nvtegrafb",
		.type		= FB_TYPE_PACKED_PIXELS,
		.visual		= FB_VISUAL_TRUECOLOR,
		.xpanstep	= 0,
		.ypanstep	= 0,
		.accel		= FB_ACCEL_NONE,
		.line_length	= LCD_WIDTH * LCD_BYTES,
	},

	// these values are just defaults. they will be over-written with the
	// correct values from the boot args.
	.var = {
		.xres		= LCD_WIDTH,
		.yres		= LCD_HEIGHT,
		.xres_virtual	= LCD_WIDTH,
		.yres_virtual	= LCD_HEIGHT,
		.bits_per_pixel	= LCD_BYTES * 8,
		.red		= {.offset=11, .length=5, .msb_right=0},
		.green		= {5, 6, 0},
		.blue		= {0, 5, 0},
		.transp		= {0, 0, 0},
		.activate	= FB_ACTIVATE_NOW,
		.height		= -1,
		.width		= -1,
		.pixclock	= 24500,
		.left_margin	= 0,
		.right_margin	= 0,
		.upper_margin	= 0,
		.lower_margin	= 0,
		.hsync_len	= 0,
		.vsync_len	= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};
#endif

struct tegra_fb_info {
	struct tegra_dc_win	*win;
	struct nvhost_device	*ndev;
	struct fb_info		*info;
	bool			valid;

	struct resource		*fb_mem;

	int			xres;
	int			yres;

	atomic_t		in_use;
#ifdef HAVE_NVMAP_CLIENT
	struct nvmap_client	*user_nvmap;
	struct nvmap_client	*fb_nvmap;
#else
	void *user_nvmap;
	void *fb_nvmap;
#endif
	struct workqueue_struct	*flip_wq;
};

struct tegra_fb_flip_win {
	struct tegra_fb_windowattr	attr;
	struct nvmap_handle_ref		*handle;
	dma_addr_t			phys_addr;
};

struct tegra_fb_flip_data {
	struct work_struct		work;
	struct tegra_fb_info		*fb;
	struct tegra_fb_flip_win	win[TEGRA_FB_FLIP_N_WINDOWS];
	u32				syncpt_max;
};

static unsigned long s_fb_addr;
static unsigned long s_fb_size;
static unsigned long s_fb_width;
static unsigned long s_fb_height;
static int s_fb_Bpp;
static NvRmMemHandle s_fb_hMem;
static unsigned long *s_fb_regs;
static unsigned short s_use_tearing_effect;
static NvU32 s_power_id = -1ul;
static NvBool tegra_fb_power_on( void );
static void tegra_fb_trigger_frame( void );
static void tegra_fb_power_off( void );

#define REGW( reg, val ) \
	do { \
		writel( (val), s_fb_regs + (reg) ); \
		wmb(); \
	} while( 0 )

/* palette array used by the fbcon */
u32 pseudo_palette[16];

/* fb_ops kernel interface */

int tegra_fb_open(struct fb_info *info, int user);
int tegra_fb_release(struct fb_info *info, int user);
int tegra_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info);
int tegra_fb_set_par(struct fb_info *info);
int tegra_fb_setcolreg(unsigned regno, unsigned red, unsigned green,
	unsigned blue, unsigned transp, struct fb_info *info);
int tegra_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info);
int tegra_fb_blank(int blank, struct fb_info *info);
void tegra_fb_fillrect(struct fb_info *info, const struct fb_fillrect *rect);
void tegra_fb_copyarea(struct fb_info *info, const struct fb_copyarea *region);
void tegra_fb_imageblit(struct fb_info *info, const struct fb_image *image);
int tegra_fb_cursor(struct fb_info *info, struct fb_cursor *cursor);
int tegra_fb_sync(struct fb_info *info);

static struct fb_ops tegra_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_open	= tegra_fb_open,
	.fb_release	= tegra_fb_release,
	.fb_check_var	= tegra_fb_check_var,
	.fb_set_par	= tegra_fb_set_par,
	.fb_setcolreg	= tegra_fb_setcolreg,
	.fb_pan_display = tegra_fb_pan_display,
	.fb_blank	= tegra_fb_blank,
	.fb_fillrect	= tegra_fb_fillrect,
	.fb_copyarea	= tegra_fb_copyarea,
	.fb_imageblit	= tegra_fb_imageblit,
	.fb_cursor	= tegra_fb_cursor,
	.fb_sync	= tegra_fb_sync,
};

int tegra_fb_open(struct fb_info *info, int user)
{
	struct tegra_fb_info *tegra_fb = info->par;

#ifdef CONFIG_MACH_MOT
    extern int MotorolaBootFBArgGet(unsigned int *arg);
    unsigned int allow_open = 0;

    MotorolaBootFBArgGet(&allow_open);

    allow_open |= PU_REASON_CHARGER == bi_powerup_reason(); // allow charge-only mode to open
    if (!allow_open)
		return -1;
#endif

	if (atomic_xchg(&tegra_fb->in_use, 1))
		return -EBUSY;

	tegra_fb->user_nvmap = NULL;

	return 0;
}

int tegra_fb_release(struct fb_info *info, int user)
{
	struct tegra_fb_info *tegra_fb = info->par;
	struct fb_var_screeninfo *var = &info->var;

	flush_workqueue(tegra_fb->flip_wq);

	if (tegra_fb->win->cur_handle) {
#ifdef HAVE_NVMAP_CLIENT
		nvmap_unpin(tegra_fb->fb_nvmap, tegra_fb->win->cur_handle);
		nvmap_free(tegra_fb->fb_nvmap, tegra_fb->win->cur_handle);
#endif
		tegra_fb->win->cur_handle = NULL;

		tegra_fb->win->x = 0;
		tegra_fb->win->y = 0;
		tegra_fb->win->w = var->xres;
		tegra_fb->win->h = var->yres;
		tegra_fb->win->out_x = 0;
		tegra_fb->win->out_y = 0;
		tegra_fb->win->out_w = var->xres;
		tegra_fb->win->out_h = var->yres;
		tegra_fb->win->flags = TEGRA_WIN_FLAG_ENABLED;
	}

	if (tegra_fb->user_nvmap) {
#ifdef HAVE_NVMAP_CLIENT
		nvmap_client_put(tegra_fb->user_nvmap);
#endif
		tegra_fb->user_nvmap = NULL;
	}

	WARN_ON(!atomic_xchg(&tegra_fb->in_use, 0));

	return 0;
}

int tegra_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	if ((var->yres * var->xres * var->bits_per_pixel / 8 * 2) >
	    info->screen_size)
		return -EINVAL;

	/* double yres_virtual to allow double buffering through pan_display */
	var->yres_virtual = var->yres * 2;

	return 0;
}

int tegra_fb_set_par(struct fb_info *info)
{
	struct tegra_fb_info *tegra_fb = info->par;
	struct fb_var_screeninfo *var = &info->var;

	if (var->bits_per_pixel) {
		/* we only support RGB ordering for now */
		switch (var->bits_per_pixel) {
		case 32:
			var->red.offset = 0;
			var->red.length = 8;
			var->green.offset = 8;
			var->green.length = 8;
			var->blue.offset = 16;
			var->blue.length = 8;
			var->transp.offset = 24;
			var->transp.length = 8;
			tegra_fb->win->fmt = TEGRA_WIN_FMT_R8G8B8A8;
			break;
		case 16:
			var->red.offset = 11;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 6;
			var->blue.offset = 0;
			var->blue.length = 5;
			tegra_fb->win->fmt = TEGRA_WIN_FMT_B5G6R5;
			break;

		default:
			return -EINVAL;
		}
		info->fix.line_length = var->xres * var->bits_per_pixel / 8;
		tegra_fb->win->stride = info->fix.line_length;
		tegra_fb->win->stride_uv = 0;
		tegra_fb->win->offset_u = 0;
		tegra_fb->win->offset_v = 0;
	}

	if (var->pixclock) {
		struct tegra_dc_mode mode;

		info->mode = (struct fb_videomode *)
			fb_find_best_mode(var, &info->modelist);
		if (!info->mode) {
			dev_warn(&tegra_fb->ndev->dev, "can't match video mode\n");
			return -EINVAL;
		}

		mode.pclk = PICOS2KHZ(info->mode->pixclock) * 1000;
		mode.h_ref_to_sync = 1;
		mode.v_ref_to_sync = 1;
		mode.h_sync_width = info->mode->hsync_len;
		mode.v_sync_width = info->mode->vsync_len;
		mode.h_back_porch = info->mode->left_margin;
		mode.v_back_porch = info->mode->upper_margin;
		mode.h_active = info->mode->xres;
		mode.v_active = info->mode->yres;
		mode.h_front_porch = info->mode->right_margin;
		mode.v_front_porch = info->mode->lower_margin;

		mode.flags = 0;

		if (!(info->mode->sync & FB_SYNC_HOR_HIGH_ACT))
			mode.flags |= TEGRA_DC_MODE_FLAG_NEG_H_SYNC;

		if (!(info->mode->sync & FB_SYNC_VERT_HIGH_ACT))
			mode.flags |= TEGRA_DC_MODE_FLAG_NEG_V_SYNC;

		tegra_dc_set_mode(tegra_fb->win->dc, &mode);

		tegra_fb->win->w = info->mode->xres;
		tegra_fb->win->h = info->mode->yres;
		tegra_fb->win->out_w = info->mode->xres;
		tegra_fb->win->out_h = info->mode->yres;
	}
	tegra_fb_trigger_frame();
	return 0;
}

int tegra_fb_setcolreg(unsigned regno, unsigned red, unsigned green,
	unsigned blue, unsigned transp, struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;

	if ((info->fix.visual == FB_VISUAL_TRUECOLOR) ||
	    (info->fix.visual == FB_VISUAL_DIRECTCOLOR)) {
		u32 v;

		if( regno >= 16 ) {
			return -EINVAL;
		}

		v = (red << var->red.offset) | (green << var->green.offset) |
			(blue << var->blue.offset);

		((u32 *)info->pseudo_palette)[regno] = v;
	}

	return 0;
}

static NvBool tegra_fb_power_register( void )
{
	if( s_power_id != -1ul )
	{
		return NV_TRUE;
	}

	if( NvRmPowerRegister( s_hRmGlobal, 0, &s_power_id ) != NvSuccess )
	{
		printk( "nvtegrafb: unable to load power manager\n" );
		return NV_FALSE;
	}

	return NV_TRUE;
}

static NvBool tegra_fb_power_on( void )
{
	if( NvRmPowerVoltageControl( s_hRmGlobal,
		NVRM_MODULE_ID( NvRmModuleID_GraphicsHost, 0 ),
		s_power_id, NvRmVoltsUnspecified, NvRmVoltsUnspecified,
		NULL, 0, NULL ) != NvSuccess )
	{
		printk( "nvtegrafb: unable to enable graphics host power\n" );
		return NV_FALSE;
	}

	if( NvRmPowerVoltageControl( s_hRmGlobal,
		NVRM_MODULE_ID( NvRmModuleID_Display, 0 ),
		s_power_id, NvRmVoltsUnspecified, NvRmVoltsUnspecified,
		NULL, 0, NULL ) != NvSuccess )
	{
		printk( "nvtegrafb: unable to enable display power\n" );
		return NV_FALSE;
	}

#if ENABLE_HDMI
	if( NvRmPowerVoltageControl( s_hRmGlobal,
		NVRM_MODULE_ID( NvRmModuleID_Hdmi, 0 ),
		s_power_id, NvRmVoltsUnspecified, NvRmVoltsUnspecified,
		NULL, 0, NULL ) != NvSuccess )
	{
		printk( "nvtegrafb: unable to enable HDMI\n" );
	}
#endif

	NvRmPowerModuleClockControl( s_hRmGlobal, NvRmModuleID_GraphicsHost,
		s_power_id, NV_TRUE );

	return NV_TRUE;
}

static void tegra_fb_power_off( void )
{
	// this will most likely not actually disable power to the display,
	// but will make it such that the power reference count is correct
	NvRmPowerVoltageControl( s_hRmGlobal,
		NVRM_MODULE_ID( NvRmModuleID_GraphicsHost, 0 ),
		s_power_id, NvRmVoltsOff, NvRmVoltsOff,
		NULL, 0, NULL );

	NvRmPowerVoltageControl( s_hRmGlobal,
		NVRM_MODULE_ID( NvRmModuleID_Display, 0 ),
		s_power_id, NvRmVoltsOff, NvRmVoltsOff,
		NULL, 0, NULL );

#if ENABLE_HDMI
	NvRmPowerVoltageControl( s_hRmGlobal,
		NVRM_MODULE_ID( NvRmModuleID_Hdmi, 0 ),
		s_power_id, NvRmVoltsOff, NvRmVoltsOff,
		NULL, 0, NULL );
#endif

	NvRmPowerModuleClockControl( s_hRmGlobal, NvRmModuleID_GraphicsHost,
		s_power_id, NV_FALSE );
}

static void tegra_fb_trigger_frame( void )
{
	if( !s_use_tearing_effect )
	{
		return;
	}

	if( !tegra_fb_power_on() )
	{
		return;
	}

	// state control: write the host trigger bit (24) along with a general
	// activation request (bit 0)
	REGW( 0x41, (1 << 24) | 1 );

	tegra_fb_power_off();
}

int tegra_fb_blank(int blank, struct fb_info *info)
{
	struct tegra_fb_info *tegra_fb = info->par;

	switch (blank) {
	case FB_BLANK_UNBLANK:
		dev_dbg(&tegra_fb->ndev->dev, "unblank\n");
		tegra_dc_enable(tegra_fb->win->dc);
		return 0;

	case FB_BLANK_POWERDOWN:
		dev_dbg(&tegra_fb->ndev->dev, "blank\n");
		flush_workqueue(tegra_fb->flip_wq);
		tegra_dc_disable(tegra_fb->win->dc);
		return 0;

	default:
		return -ENOTTY;
	}
}

void tegra_fb_suspend(struct tegra_fb_info *tegra_fb)
{
	flush_workqueue(tegra_fb->flip_wq);
}
/*
int tegra_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	u32 addr;

	if( !tegra_fb_power_on() ) {
		return -EINVAL;
	}

	info->var.xoffset = var->xoffset;
	info->var.yoffset = var->yoffset;

	addr = s_fb_addr + (var->yoffset * tegra_fb_def.fix.line_length ) +
		(var->xoffset * s_fb_Bpp );

	// window header - select Window A
	REGW( 0x42, (1 << 4) );
	// window surface base address
	REGW( 0x800, addr );
	// state control - general update - Window A
	REGW( 0x41, (1 << 8) | (1 << 9) );
	// state control - general activate - Window A
	REGW( 0x41, (1 << 0) | (1 << 1) );

	tegra_fb_trigger_frame();
	tegra_fb_power_off();

	return 0;
}
*/
int tegra_fb_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	struct tegra_fb_info *tegra_fb = info->par;
	char __iomem *flush_start;
	char __iomem *flush_end;
	u32 addr;

	if( !tegra_fb_power_on() ) {
		return -EINVAL;
	}

	if (!tegra_fb->win->cur_handle) {
		flush_start = info->screen_base + (var->yoffset * info->fix.line_length);
		flush_end = flush_start + (var->yres * info->fix.line_length);

		info->var.xoffset = var->xoffset;
		info->var.yoffset = var->yoffset;

		addr = info->fix.smem_start + (var->yoffset * info->fix.line_length) +
			(var->xoffset * (var->bits_per_pixel/8));

		tegra_fb->win->phys_addr = addr;
		/* TODO: update virt_addr */

		tegra_dc_update_windows(&tegra_fb->win, 1);
		tegra_dc_sync_windows(&tegra_fb->win, 1);
	}

	tegra_fb_trigger_frame();
	tegra_fb_power_off();

	return 0;
}

void tegra_fb_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	cfb_fillrect(info, rect);
	tegra_fb_trigger_frame();
}

void tegra_fb_copyarea(struct fb_info *info, const struct fb_copyarea *region)
{
	cfb_copyarea(info, region);
	tegra_fb_trigger_frame();
}

void tegra_fb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	cfb_imageblit(info, image);
	tegra_fb_trigger_frame();
}

int tegra_fb_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	return 0;
}

int tegra_fb_sync(struct fb_info *info)
{
	return 0;
}

#ifdef CONFIG_MACH_MOT
static ssize_t tegra_fb_show_screen_size(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
    unsigned int scrSize;
    extern int MotorolaBootDispArgGet(unsigned int *arg);

    if ( MotorolaBootDispArgGet(&scrSize) != 0 )
        scrSize = 0;

    return sprintf(buf, "%d\n", scrSize);
}

static DEVICE_ATTR(screen_size, 0444, tegra_fb_show_screen_size, NULL);
#endif

void tegra_fb_update_monspecs(struct tegra_fb_info *fb_info,
			      struct fb_monspecs *specs,
			      bool (*mode_filter)(struct fb_videomode *mode))
{
	struct fb_event event;
	struct fb_modelist *m;
	int i;

	mutex_lock(&fb_info->info->lock);
	fb_destroy_modedb(fb_info->info->monspecs.modedb);

	fb_destroy_modelist(&fb_info->info->modelist);

	if (specs == NULL) {
		struct tegra_dc_mode mode;
		memset(&fb_info->info->monspecs, 0x0,
		       sizeof(fb_info->info->monspecs));
		memset(&mode, 0x0, sizeof(mode));
		tegra_dc_set_mode(fb_info->win->dc, &mode);
		mutex_unlock(&fb_info->info->lock);
		return;
	}

	memcpy(&fb_info->info->monspecs, specs,
	       sizeof(fb_info->info->monspecs));

	for (i = 0; i < specs->modedb_len; i++) {
		if (mode_filter) {
			if (mode_filter(&specs->modedb[i]))
				fb_add_videomode(&specs->modedb[i],
						 &fb_info->info->modelist);
		} else {
			fb_add_videomode(&specs->modedb[i],
					 &fb_info->info->modelist);
		}
	}

	if (list_empty(&fb_info->info->modelist)) {
		struct tegra_dc_mode mode;
		memset(&fb_info->info->var, 0x0, sizeof(fb_info->info->var));
		memset(&mode, 0x0, sizeof(mode));
		tegra_dc_set_mode(fb_info->win->dc, &mode);
	} else {
		/* in case the first mode was not matched */
		m = list_first_entry(&fb_info->info->modelist, struct fb_modelist, list);
		m->mode.flag |= FB_MODE_IS_FIRST;
		fb_info->info->mode = (struct fb_videomode *)
			fb_find_best_display(specs, &fb_info->info->modelist);

		memset(&fb_info->info->var, 0x0,
		       sizeof(fb_info->info->var));
		fb_videomode_to_var(&fb_info->info->var, fb_info->info->mode);
		tegra_fb_set_par(fb_info->info);
	}

	event.info = fb_info->info;
	fb_notifier_call_chain(FB_EVENT_NEW_MODELIST, &event);
	mutex_unlock(&fb_info->info->lock);
}

struct tegra_fb_info *tegra_fb_register(struct nvhost_device *ndev,
					struct tegra_dc *dc,
					struct tegra_fb_data *fb_data,
					struct resource *fb_mem)
{
	struct tegra_dc_win *win;
	struct fb_info *info;
	struct tegra_fb_info *tegra_fb;
	void __iomem *fb_base = NULL;
	unsigned long fb_size = 0;
	unsigned long fb_phys = 0;
	int ret = 0;

	pr_notice("%s: dc=%p, fb_data=%p, fb_mem=%p\n", __func__, dc, fb_data, fb_mem);
#ifdef CONFIG_MACH_MOT
	NvError e;
	NvBootArgsFramebuffer boot_fb;

	e = NvOsBootArgGet(NvBootArgKey_Framebuffer, &boot_fb, sizeof(boot_fb));
	if (e != NvSuccess || !boot_fb.MemHandleKey) {
		pr_warning("nvtegrafb: bootargs not found\n");
		ret = -1;
		goto err;
	}

	e = NvRmMemHandleClaimPreservedHandle(s_hRmGlobal, boot_fb.MemHandleKey,
		&s_fb_hMem );
	if (e != NvSuccess) {
		pr_err("tegra_fb_register: Unable to reserve bootup framebuffer memory.\n");
		ret = -1;
		goto err;
	}


	s_fb_width = boot_fb.Width;
	s_fb_height = boot_fb.Height * boot_fb.NumSurfaces;
	s_fb_size = boot_fb.Size;
	s_fb_addr = NvRmMemPin(s_fb_hMem);
	s_fb_Bpp = NV_COLOR_GET_BPP(boot_fb.ColorFormat) >> 3;
#endif
	tegra_fb_power_register();

	s_fb_regs = ioremap_nocache( DISPLAY_BASE, 256 * 1024 );


	// need to poke a trigger register if the tearing effect signal is used
	if( boot_fb.Flags & NVBOOTARG_FB_FLAG_TEARING_EFFECT )
	{
		s_use_tearing_effect = 1;
	}

	win = tegra_dc_get_window(dc, fb_data->win);
	if (!win) {
		dev_err(&ndev->dev, "dc does not have a window at index %d\n",
			fb_data->win);
		return ERR_PTR(-ENOENT);
	}

	info = framebuffer_alloc(sizeof(struct tegra_fb_info), &ndev->dev);
	if (!info) {
		ret = -ENOMEM;
		goto err;
	}
	//memcpy (&info->var, &tegra_fb_def.var, sizeof(struct fb_var_screeninfo));

	tegra_fb = info->par;
	tegra_fb->win = win;
	tegra_fb->ndev = ndev;
	tegra_fb->fb_mem = fb_mem;
	tegra_fb->xres = fb_data->xres;
	tegra_fb->yres = fb_data->yres;

#ifdef HAVE_NVMAP_CLIENT
	tegra_fb->fb_nvmap = nvmap_create_client(nvmap_dev, "nvtegrafb");
	if (!tegra_fb->fb_nvmap) {
		dev_err(&ndev->dev, "couldn't create nvmap client\n");
		ret = -ENOMEM;
		goto err_free;
	}
#endif

	atomic_set(&tegra_fb->in_use, 0);

	tegra_fb->flip_wq = create_singlethread_workqueue(dev_name(&ndev->dev));
	if (!tegra_fb->flip_wq) {
		dev_err(&ndev->dev, "couldn't create flip work-queue\n");
		ret = -ENOMEM;
		goto err_delete_wq;
	}

	if (fb_mem) {
		fb_size = resource_size(fb_mem);
		fb_phys = fb_mem->start;
		fb_base = ioremap_nocache(fb_phys, fb_size);
	} else {
		fb_size = s_fb_size;
		fb_phys = s_fb_addr;
		fb_base = ioremap_nocache(DISPLAY_BASE, 256 * 1024 );
	}
	if (!fb_base) {
		dev_err(&ndev->dev, "fb can't be mapped phys=0x%lx iobase=0x%p size=%lu\n",
		                    fb_phys, fb_base, fb_size);
		ret = -EBUSY;
		goto err_put_client;
	}
	s_fb_regs = fb_base;
	tegra_fb->valid = true;

	info->fbops = &tegra_fb_ops;
	info->pseudo_palette = pseudo_palette;
	info->screen_base = fb_base;
	info->screen_size = fb_size;

	strlcpy(info->fix.id, "tegrafb", sizeof(info->fix.id));
	info->fix.type		= FB_TYPE_PACKED_PIXELS;
	info->fix.visual	= FB_VISUAL_TRUECOLOR;
	info->fix.xpanstep	= 1;
	info->fix.ypanstep	= 1;
	info->fix.accel		= FB_ACCEL_NONE;
	info->fix.smem_start	= fb_phys;
	info->fix.smem_len	= fb_size;

	info->var.xres			= fb_data->xres;
	info->var.yres			= fb_data->yres;
	info->var.xres_virtual		= fb_data->xres;
	info->var.yres_virtual		= fb_data->yres * 2;
	info->var.bits_per_pixel	= fb_data->bits_per_pixel;
	info->var.activate		= FB_ACTIVATE_VBL;
	info->var.height		= tegra_dc_get_out_height(dc);
	info->var.width			= tegra_dc_get_out_width(dc);
	info->var.pixclock		= 0;
	info->var.left_margin		= 0;
	info->var.right_margin		= 0;
	info->var.upper_margin		= 0;
	info->var.lower_margin		= 0;
	info->var.hsync_len		= 0;
	info->var.vsync_len		= 0;
	info->var.vmode			= FB_VMODE_NONINTERLACED;

	win->x = 0;
	win->y = 0;
	win->w = fb_data->xres;
	win->h = fb_data->yres;
	/* TODO: set to output res dc */
	win->out_x = 0;
	win->out_y = 0;
	win->out_w = fb_data->xres;
	win->out_h = fb_data->yres;
	win->z = 0;
	win->phys_addr = fb_phys;
	win->virt_addr = fb_base;
	win->offset_u = 0;
	win->offset_v = 0;
	win->stride = fb_data->xres * fb_data->bits_per_pixel / 8;
	win->stride_uv = 0;
	win->flags = TEGRA_WIN_FLAG_ENABLED;

	if (fb_mem)
		tegra_fb_set_par(info);

	if (register_framebuffer(info)) {
		dev_err(&ndev->dev, "failed to register framebuffer\n");
		ret = -ENODEV;
		goto err_iounmap_fb;
	}

	tegra_fb->info = info;

	dev_info(&ndev->dev, "probed\n");

	if (fb_data->flags & TEGRA_FB_FLIP_ON_PROBE) {
		tegra_dc_update_windows(&tegra_fb->win, 1);
		tegra_dc_sync_windows(&tegra_fb->win, 1);
	}

#ifdef CONFIG_MACH_MOT
	/* create a sysfs file to advertise screen info */
	device_create_file(&ndev->dev, &dev_attr_screen_size);
#endif

	return tegra_fb;

err_iounmap_fb:
	iounmap(fb_base);
err_put_client:
#ifdef HAVE_NVMAP_CLIENT
	nvmap_client_put(tegra_fb->fb_nvmap);
#endif
err_delete_wq:
	destroy_workqueue(tegra_fb->flip_wq);
err_free:
	framebuffer_release(info);
err:
	pr_err("%s: returned error %d\n", __func__, ret);
	return ERR_PTR(ret);
}

void tegra_fb_unregister(struct tegra_fb_info *fb_info)
{
	struct fb_info *info = fb_info->info;

	pr_err("%s\n", __func__);

#ifdef HAVE_NVMAP_CLIENT
	if (fb_info->win->cur_handle) {
		nvmap_unpin(fb_info->fb_nvmap, fb_info->win->cur_handle);
		nvmap_free(fb_info->fb_nvmap, fb_info->win->cur_handle);
	}

	if (fb_info->fb_nvmap)
		nvmap_client_put(fb_info->fb_nvmap);
#endif

	tegra_fb_power_off();
	NvRmPowerUnRegister( s_hRmGlobal, s_power_id );

	unregister_framebuffer(info);

	flush_workqueue(fb_info->flip_wq);
	destroy_workqueue(fb_info->flip_wq);

	iounmap(info->screen_base);
	framebuffer_release(info);
}

/* Old moto/nvdia stuff
static int tegra_plat_probe( struct platform_device *d )
{
	NvError e;
	NvBootArgsFramebuffer boot_fb;

	e = NvOsBootArgGet(NvBootArgKey_Framebuffer, &boot_fb, sizeof(boot_fb));
	if (e != NvSuccess || !boot_fb.MemHandleKey) {
		pr_warning("nvtegrafb: bootargs not found\n");
		return -1;
	}

	e = NvRmMemHandleClaimPreservedHandle(s_hRmGlobal, boot_fb.MemHandleKey,
		&s_fb_hMem );
	if (e != NvSuccess) {
		pr_err("nvtegrafb: Unable to query bootup framebuffer memory.\n");
		return -1;
	}

	tegra_fb_power_register();

	s_fb_width = boot_fb.Width;
	s_fb_height = boot_fb.Height * boot_fb.NumSurfaces;
	s_fb_size = boot_fb.Size;
	s_fb_addr = NvRmMemPin(s_fb_hMem);
	s_fb_Bpp = NV_COLOR_GET_BPP(boot_fb.ColorFormat) >> 3;
	s_fb_regs = ioremap_nocache( DISPLAY_BASE, 256 * 1024 );

	// need to poke a trigger register if the tearing effect signal is used
	if( boot_fb.Flags & NVBOOTARG_FB_FLAG_TEARING_EFFECT )
	{
		s_use_tearing_effect = 1;
	}

	tegra_fb_def.fix.smem_start = s_fb_addr;
	tegra_fb_def.fix.smem_len = s_fb_size;
	tegra_fb_def.fix.line_length = boot_fb.Pitch;

	tegra_fb_def.fbops = &tegra_fb_ops;

	tegra_fb_def.screen_size = s_fb_size;
	tegra_fb_def.pseudo_palette = pseudo_palette;
	tegra_fb_def.screen_base = ioremap_nocache(s_fb_addr, s_fb_size);

	tegra_fb_def.var.xres = boot_fb.Width;
	tegra_fb_def.var.yres = boot_fb.Height;
	tegra_fb_def.var.xres_virtual = s_fb_width;
	tegra_fb_def.var.yres_virtual = s_fb_height;

	if (boot_fb.ColorFormat == NvColorFormat_A8R8G8B8)
	{
		pr_info("nvtegrafb: using %dx%d BGRA_8888 format\n",
			boot_fb.Width, boot_fb.Height);
		tegra_fb_def.var.bits_per_pixel = 32;
		tegra_fb_def.var.transp.offset = 24;
		tegra_fb_def.var.transp.length = 8;
		tegra_fb_def.var.red.offset = 16;
		tegra_fb_def.var.red.length = 8;
		tegra_fb_def.var.green.offset = 8;
		tegra_fb_def.var.green.length = 8;
		tegra_fb_def.var.blue.offset = 0;
		tegra_fb_def.var.blue.length = 8;
	} else {
		tegra_fb_info.var.bits_per_pixel = s_fb_Bpp * 8;
		pr_info("nvtegrafb: using %dx%d %dbpp format\n",
			boot_fb.Width, boot_fb.Height, s_fb_Bpp * 8);
	}

	if( tegra_fb_def.screen_base == 0 ) {
		pr_err("nvtegrafb: framebuffer map failure\n");
		NvRmMemHandleFree(s_fb_hMem);
		s_fb_hMem = NULL;
		return -1;
	}
	if( boot_fb.NumSurfaces > 1 ) {
		tegra_fb_def.fix.ypanstep = 1;
	}

	pr_info("nvtegrafb: base address: %x physical: %x\n",
		(unsigned int)tegra_fb_def.screen_base,
		(unsigned int)s_fb_addr );

	register_framebuffer(&tegra_fb_def);

#ifdef CONFIG_MACH_MOT
	// create a sysfs file to advertise screen info
	device_create_file(&d->dev, &dev_attr_screen_size);
#endif

	return 0;
}

struct platform_driver tegra_platform_driver =
{
	.probe	= tegra_plat_probe,
	.driver	= {
		.name = "nvtegrafb",
		.owner = THIS_MODULE,
	},
};

static struct platform_device tegra_fb_device =
{
	.name		= "nvtegrafb",
	.id		= -1,
	.num_resources	= 0,
	.resource	= 0,
	.dev		= {
		.platform_data = NULL,
	},
};

static int __init tegra_fb_init(void)
{
	int e;
	e = platform_driver_register(&tegra_platform_driver);
	if (e) {
		printk("nvtegrafb: platform_driver_register failed\n");
		return e;
	}

	e = platform_device_register(&tegra_fb_device);
	if (e) {
		printk("nvtegrafb: platform_device_register failed\n");
	}

	return e;
}

static void __exit tegra_exit( void )
{
	tegra_fb_power_off();

	NvRmPowerUnRegister( s_hRmGlobal, s_power_id );

	unregister_framebuffer(&tegra_fb_def);
}
module_init(tegra_fb_init);
module_exit(tegra_exit);
*/
