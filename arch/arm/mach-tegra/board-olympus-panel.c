/*
 * arch/arm/mach-tegra/board-olympus-panel.c
 *
 * Copyright (C) 2012 Atrix-Dev-Team
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
#include <linux/earlysuspend.h>
#include <linux/gpio.h>
#include <linux/resource.h>
#include <linux/platform_device.h>
#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <mach/nvhost.h>
#include <mach/tegra_fb.h>

#include "board.h"
#include "gpio-names.h"

/* to clean later : */
#include <mach/nvrm_linux.h>
#include "nvcommon.h"
#include "nvos.h"
#include "nvcolor.h"
#include "nvbootargs.h"
#include "nvrm_module.h"
#include "nvrm_memmgr.h"
#include "nvrm_power.h"
#include "nvrm_ioctls.h"

/*
 HDMI plugged screens on :
 gpio-25  (nvrm_gpio       PD1 ) in  hi irq-217 (default)
 gpio-32  (backlight_en    PE0 ) out hi backlight_en
 gpio-35  (nvrm_gpio       PE3 ) out hi
 gpio-46  (nvrm_gpio       PF6 ) out hi
 gpio-47  (nvrm_gpio       PF7 ) out hi key_backlight_en
 gpio-111 (nvrm_gpio       PN7 ) in  hi irq-303 (default) << IRQ HDMI IN
 gpio-174 (nvrm_gpio       PV6 ) out hi usb ?

 HDMI plugged screens off :
 gpio-25  (nvrm_gpio           ) in  hi irq-217 (default)
 gpio-32  (backlight_en        ) out lo
 gpio-35  (nvrm_gpio           ) out lo
 gpio-46  (nvrm_gpio           ) out lo
 gpio-47  (nvrm_gpio           ) out lo
 gpio-174 (nvrm_gpio           ) out hi
*/
#define olympus_backlight_en TEGRA_GPIO_PE0

#define HDMI_INPUT TEGRA_GPIO_PN7 // ok sure

//#define HDMI_5V_EN TEGRA_GPIO_PE3 // no special bugs
//#define HDMI_5V_EN TEGRA_GPIO_PF6 // reboot
//#define HDMI_5V_EN TEGRA_GPIO_PV6 // break usb

//to put in iomap.h
#define TEGRA_HDMI_BASE         0x54280000
#define TEGRA_HDMI_SIZE         SZ_256K

#define USE_LCD_FRAMEBUFFER

/* Framebuffer(s) */
#ifdef USE_LCD_FRAMEBUFFER
static struct resource fb_lcd_resource[] = {
	[0] = {
		.name   = "irq",
		.start  = INT_DISPLAY_GENERAL,
		.end    = INT_DISPLAY_GENERAL,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.name   = "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.name   = "fbmem",
//		.start	= 0x1c03a000,
//		.end	= 0x1c03a000 + 0x500000 - 1,
		.start	= 0x1fddc000,
		.end	= 0x1fddc000 + 0x500000 - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_fb_lcd_data olympus_lcd_platform_data = {
	.lcd_xres	= 540,
	.lcd_yres	= 960,
	.fb_xres	= 540,
	.fb_yres	= 960,
	.bits_per_pixel	= 16,
};

static struct platform_device olympus_fb_device = {
	.name 		= "tegrafb",
	.id		= 0,
	.resource	= fb_lcd_resource,
	.num_resources 	= ARRAY_SIZE(fb_lcd_resource),
	.dev = {
		.platform_data = &olympus_lcd_platform_data,
	},
};
#endif

/* HDMI nvhost framebuffer device */
static int olympus_hdmi_init(void) {

#ifdef HDMI_5V_EN
	tegra_gpio_enable(HDMI_5V_EN);
	gpio_request(HDMI_5V_EN, "hdmi_5v_en");
	gpio_direction_output(HDMI_5V_EN, 1);
#endif

#if 0
// let gpios config handled by nvrm "hal"
	tegra_gpio_enable(HDMI_INPUT);
	gpio_request(HDMI_INPUT, "nvrm_gpio");
	gpio_direction_input(HDMI_INPUT);
#endif

	return 0;
}

#ifdef HDMI_5V_EN
static int olympus_hdmi_enable(void) {
	gpio_set_value(HDMI_5V_EN, 1);
	return 0;
}
static int olympus_hdmi_disable(void) {
	gpio_set_value(HDMI_5V_EN, 0);
	return 0;
}
#endif

static struct resource olympus_disp_hdmi_resources[] = {
	{
		.name   = "irq",
		.start  = INT_DISPLAY_B_GENERAL,
		.end    = INT_DISPLAY_B_GENERAL,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "regs",
		.start  = TEGRA_DISPLAY2_BASE,
		.end    = TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE-1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "fbmem",
		.flags  = IORESOURCE_MEM,
		.start	= 0x1c03a000,
		.end	= 0x1c03a000 + 0x500000 - 1,
	},
	{
		.name   = "hdmi_regs",
		.start  = TEGRA_HDMI_BASE,
		.end    = TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE-1,
		.flags  = IORESOURCE_MEM,
	},
};

// used for hdmi ??
static struct tegra_dc_mode ventana_panel_modes[] = {
	{
		.pclk = 62200000,
		.h_ref_to_sync = 11,
		.v_ref_to_sync = 1,
		.h_sync_width = 58,
		.v_sync_width = 4,
		.h_back_porch = 58,
		.v_back_porch = 4,
		.h_active = 1366,
		.v_active = 768,
		.h_front_porch = 58,
		.v_front_porch = 4,
	},
};

static struct tegra_dc_out olympus_disp_hdmi_out = {
	.type = TEGRA_DC_OUT_HDMI,
	.flags = TEGRA_DC_OUT_HOTPLUG_HIGH,
	.dcc_bus = 1,
	.hotplug_gpio = HDMI_INPUT,
	.align = TEGRA_DC_ALIGN_MSB,
	.order = TEGRA_DC_ORDER_RED_BLUE,

	.modes = ventana_panel_modes,
	.n_modes = ARRAY_SIZE(ventana_panel_modes),

#ifdef HDMI_5V_EN
	.enable  = olympus_hdmi_enable,
	.disable = olympus_hdmi_disable,
#endif
};

static struct tegra_fb_data olympus_disp_hdmi_fb_data = {
	.win		= 0,
	.xres       = 1920,
	.yres       = 1080,
	.bits_per_pixel = 4,
};

static struct tegra_dc_platform_data olympus_disp_hdmi_pdata = {
	.flags		= 0,
	.emc_clk_rate	= ULONG_MAX,
	.default_out	= &olympus_disp_hdmi_out,
	.fb		= &olympus_disp_hdmi_fb_data,
};

static struct nvhost_device olympus_disp_hdmi_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= olympus_disp_hdmi_resources,
	.num_resources	= ARRAY_SIZE(olympus_disp_hdmi_resources),
	.dev = {
		.platform_data = &olympus_disp_hdmi_pdata,
	},
};

#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend olympus_panel_early_suspender;
static void olympus_panel_early_suspend(struct early_suspend *h) {
	if (num_registered_fb > 1)
		fb_blank(registered_fb[1], FB_BLANK_POWERDOWN);
	if (num_registered_fb > 0)
		fb_blank(registered_fb[0], FB_BLANK_POWERDOWN);
}
static void olympus_panel_late_resume(struct early_suspend *h) {
	if (num_registered_fb > 1)
		fb_blank(registered_fb[1], FB_BLANK_UNBLANK);
	if (num_registered_fb > 0)
		fb_blank(registered_fb[0], FB_BLANK_UNBLANK);
}
#endif

int __init olympus_panel_init(void) {
	int ret = 0;
	struct resource *res;

	NvError e;
	NvBootArgsFramebuffer boot_fb;
	NvRmMemHandle fb_hMem;

	olympus_hdmi_init();

	e = NvOsBootArgGet(NvBootArgKey_Framebuffer, &boot_fb, sizeof(boot_fb));
	if (e != NvSuccess || !boot_fb.MemHandleKey) {
		pr_warning("%s: Framebuffer bootargs not found\n", __func__);
	}
	else {
		tegra_bootloader_fb_size  = boot_fb.Size; //~2MB
		pr_info("%s: %dx%d with %d surfaces\n", __func__, boot_fb.Width, 
		        boot_fb.Height, boot_fb.NumSurfaces);

//		tegra_bootloader_fb_start = (unsigned long) NvRmMemGetAddress(boot_fb.MemHandleKey, 0);

		e = NvRmMemHandleClaimPreservedHandle(s_hRmGlobal, boot_fb.MemHandleKey,
			&fb_hMem );
		if (e != NvSuccess) {
			pr_err("%s: Unable to query bootup framebuffer memory.\n", __func__);
		}
		else
			tegra_bootloader_fb_start = NvRmMemPin(fb_hMem);
/**/
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	olympus_panel_early_suspender.suspend = olympus_panel_early_suspend;
	olympus_panel_early_suspender.resume = olympus_panel_late_resume;
	olympus_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&olympus_panel_early_suspender);
#endif
	if (!tegra_fb_start) {
		tegra_fb_start = tegra_bootloader_fb_start;
		tegra_fb_size = tegra_bootloader_fb_size;
	}
	res = nvhost_get_resource_byname(&olympus_disp_hdmi_device, IORESOURCE_MEM, "fbmem");
	if (!res->start && tegra_fb2_start) {
		res->start = tegra_fb2_start;
		res->end = tegra_fb2_start + tegra_fb2_size - 1;
	}
	if (res->start && !tegra_fb2_start) {
		tegra_fb2_start = res->start;
		tegra_fb2_size = res->end - res->start;
	}
	pr_warning("%s: start=0x%x end=%x\n", __func__, res->start, res->end);

/*	if (tegra_fb2_size == 0xFFFFFFFF) {
		tegra_fb2_start = tegra_fb_start + 0x400000;
		tegra_fb2_size = boot_fb.Size;
	}*/

/*
	tegra_dump_reserved_memory();
	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,
	                       min(tegra_fb_size, tegra_bootloader_fb_size));
*/
	ret = nvhost_device_register(&olympus_disp_hdmi_device);

	tegra_dump_reserved_memory();

#ifdef USE_LCD_FRAMEBUFFER
	//ret = nvhost_device_register(&olympus_disp_lcd_device);
	//pr_info("nvhost_device_register fb0: ret=%d\n", ret);
	ret = platform_device_register(&olympus_fb_device);
	pr_info("platform_device_register fb0: ret=%d\n", ret);
#endif

	return ret;
}
