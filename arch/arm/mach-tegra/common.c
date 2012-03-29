/*
 * arch/arm/mach-tegra/board-harmony.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@android.com>
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

#include <linux/init.h>
#include <linux/io.h>
#include <linux/cpu.h>
#include <linux/highmem.h>
#include <linux/nvmap.h>

#include <asm/hardware/cache-l2x0.h>
#include <asm/cacheflush.h>
#include <asm/outercache.h>
#include <asm/page.h>
#include <asm/system.h>

#include <mach/iomap.h>
#include <mach/dma.h>
#include <mach/fuse.h>

#ifdef CONFIG_MFD_CPCAP
#include <linux/spi/cpcap.h>
#include <mach/gpio.h>
#include "gpio-names.h"
#endif

#include "board.h"

#define APB_MISC_HIDREV		0x804
#define FUSE_VISIBILITY_REG_OFFSET		0x48
#define FUSE_VISIBILITY_BIT_POS		28
#define FUSE_SPARE_BIT_18_REG_OFFSET		0x248
#define FUSE_SPARE_BIT_19_REG_OFFSET		0x24c

unsigned long tegra_bootloader_fb_start=0;
unsigned long tegra_bootloader_fb_size=0;
unsigned long tegra_fb_start=0;
unsigned long tegra_fb_size=0;
unsigned long tegra_fb2_start=0;
unsigned long tegra_fb2_size=0;
unsigned long tegra_carveout_start=0;
unsigned long tegra_carveout_size=0;
unsigned long tegra_lp0_vec_start=0;
unsigned long tegra_lp0_vec_size=0;
unsigned long tegra_grhost_aperture=0;

bool tegra_chip_compare(u32 chip, u32 major_rev, u32 minor_rev)
{
	void __iomem *misc = IO_ADDRESS(TEGRA_APB_MISC_BASE);
	u32 val = readl(misc + APB_MISC_HIDREV);
	u32 id = (val>>8) & 0xff;
	u32 minor = (val>>16) & 0xf;
	u32 major = (val>>4) & 0xf;

	return (chip==id) &&
		(minor_rev==minor || minor_rev==TEGRA_ALL_REVS) &&
		(major_rev==major || major_rev==TEGRA_ALL_REVS);
}

bool tegra_is_ap20_a03p(void)
{
	if (tegra_is_ap20_a03()) {
		void __iomem *clk = IO_ADDRESS(TEGRA_CLK_RESET_BASE);
		void __iomem *fuse = IO_ADDRESS(TEGRA_FUSE_BASE);
		u32 clk_val = readl(clk + FUSE_VISIBILITY_REG_OFFSET);
		u32 fuse_18_val = 0;
		u32 fuse_19_val = 0;

		clk_val |= (1 << FUSE_VISIBILITY_BIT_POS);
		writel(clk_val, (clk + FUSE_VISIBILITY_REG_OFFSET));
		fuse_18_val = readl(fuse + FUSE_SPARE_BIT_18_REG_OFFSET);
		fuse_19_val = readl(fuse + FUSE_SPARE_BIT_19_REG_OFFSET);
		clk_val &= ~(1 << FUSE_VISIBILITY_BIT_POS);
		writel(clk_val, (clk + FUSE_VISIBILITY_REG_OFFSET));
		return (((fuse_18_val|fuse_19_val)&1)? true:false);
	}
	else {
		return false;
	}
}

#ifdef CONFIG_DMABOUNCE
int dma_needs_bounce(struct device *dev, dma_addr_t addr, size_t size)
{
	return 0;
}
#endif

void tegra_machine_restart(char mode, const char *cmd)
{
#ifdef CONFIG_MFD_CPCAP
	/* Disable powercut detection before restart */
/* NVSSW-992: FIXME: this can cause scheduling to happen via the SPI driver, which
   won't work if we are atomic (panic).
	cpcap_disable_powercut(); */

	/* Assert SYSRSTRTB input to CPCAP to force cold restart */
	gpio_request(TEGRA_GPIO_PZ2, "sysrstrtb");
	gpio_set_value(TEGRA_GPIO_PZ2, 0);
	gpio_direction_output(TEGRA_GPIO_PZ2, 0);
#else
	disable_nonboot_cpus();
	flush_cache_all();
	outer_shutdown();
	arm_machine_restart(mode, cmd);
#endif
}

void __init tegra_init_cache(void)
{
#ifdef CONFIG_CACHE_L2X0
	void __iomem	*p = IO_ADDRESS(TEGRA_ARM_PERIF_BASE) + 0x3000;
	unsigned	pl310_auxctrl = 0x7C480001;
	unsigned	reg;

	writel(0x331, p + L2X0_TAG_LATENCY_CTRL);
	writel(0x441, p + L2X0_DATA_LATENCY_CTRL);
	writel(7, p + L2X0_PREFETCH_OFFSET);
	writel(2, p + L2X0_PWR_CTRL);

	l2x0_init(p, pl310_auxctrl, 0x8200c3fe);

#ifdef CONFIG_CPU_V7
#define PL310_FLZE	(1<<0)
	asm volatile ("mrc p15, 0, %0, c1, c0, 1" : "=r" (reg) : : "cc");
	/* enable Full Line of Zeros */
	if (pl310_auxctrl & PL310_FLZE)
		reg |= 1 << 3;
	asm volatile ("mcr p15, 0, %0, c1, c0, 1" : : "r" (reg) : "cc");
#endif
#endif
}

void __init tegra_common_init(void)
{
#ifdef CONFIG_CPU_V7
	/* enable dynamic clock gating */
	unsigned int reg;
	asm volatile ("mrc p15, 0, %0, c15, c0, 0" : "=r" (reg) : : "cc");
	reg |= 1;
	asm volatile ("mcr p15, 0, %0, c15, c0, 0" : : "r" (reg) : "cc");
#endif

	nvmap_add_carveout_heap(TEGRA_IRAM_BASE, TEGRA_IRAM_SIZE,
				"iram", NVMEM_HEAP_CARVEOUT_IRAM);
	tegra_init_clock();
	tegra_init_cache();
	tegra_init_fuse_cache();
	tegra_dma_init();
	tegra_mc_init();
	arm_pm_restart = tegra_machine_restart;
}

static int __init tegra_bootloader_fb_arg(char *options)
{
	char *p = options;
	tegra_bootloader_fb_size = memparse(p, &p);
	if (*p == '@')
		tegra_bootloader_fb_start = memparse(p+1, &p);
	pr_info("Found tegra_fbmem: %08lx@%08lx\n",
		tegra_bootloader_fb_size, tegra_bootloader_fb_start);
	return 0;
}
early_param("tegra_fbmem", tegra_bootloader_fb_arg);

static int __init tegra_lp0_vec_arg(char *options)
{
        char *p = options;
 
        tegra_lp0_vec_size = memparse(p, &p);
        if (*p == '@')
                tegra_lp0_vec_start = memparse(p+1, &p);
 
        return 0;
}
early_param("lp0_vec", tegra_lp0_vec_arg);
/*
 * Convert a page to/from a physical address
 */
#define phys_to_page(phys)  (pfn_to_page(__phys_to_pfn(phys)))

/*
 * Due to conflicting restrictions on the placement of the framebuffer,
 * the bootloader is likely to leave the framebuffer pointed at a location
 * in memory that is outside the grhost aperture.  This function will move
 * the framebuffer contents from a physical address that is anywher (lowmem,
 * highmem, or outside the memory map) to a physical address that is outside
 * the memory map.
 */
void tegra_move_framebuffer(unsigned long to, unsigned long from,
	unsigned long size)
{
	struct page *page;
	void __iomem *to_io;
	void *from_virt;
	unsigned long i;
	BUG_ON(PAGE_ALIGN((unsigned long)to) != (unsigned long)to);
	BUG_ON(PAGE_ALIGN(from) != from);
	BUG_ON(PAGE_ALIGN(size) != size);
	to_io = ioremap(to, size);
	if (!to_io) {
		pr_err("%s: Failed to map target framebuffer\n", __func__);
		return;
	}
	pr_info("%s: %08lx %08lx %08lx %p", __func__, to, from, size, to_io);
	if (pfn_valid(page_to_pfn(phys_to_page(from)))) {
		for (i = 0 ; i < size; i += PAGE_SIZE) {
			page = phys_to_page(from + i);
			from_virt = kmap(page);
			memcpy_toio(to_io + i, from_virt, PAGE_SIZE);
			kunmap(page);
		}
	} else {
		void __iomem *from_io = ioremap(from, size);
		if (!from_io) {
			pr_err("%s: Failed to map source framebuffer\n",
				__func__);
			goto out;
		}
		for (i = 0; i < size; i+= 4)
			writel(readl(from_io + i), to_io + i);
		iounmap(from_io);
	}
out:
	iounmap(to_io);
}

#ifdef CONFIG_HAVE_MEMBLOCK
void __init tegra_reserve(unsigned long carveout_size, unsigned long fb_size,
	unsigned long fb2_size)
{
	if (tegra_lp0_vec_size)
		if (memblock_reserve(tegra_lp0_vec_start, tegra_lp0_vec_size))
			pr_err("Failed to reserve lp0_vec %08lx@%08lx\n",
				tegra_lp0_vec_size, tegra_lp0_vec_start);


	tegra_carveout_start = memblock_end_of_DRAM() - carveout_size;
	if (memblock_remove(tegra_carveout_start, carveout_size))
		pr_err("Failed to remove carveout %08lx@%08lx from memory "
			"map\n",
			tegra_carveout_start, carveout_size);
	else
		tegra_carveout_size = carveout_size;

	tegra_fb2_start = memblock_end_of_DRAM() - fb2_size;
	if (memblock_remove(tegra_fb2_start, fb2_size))
		pr_err("Failed to remove second framebuffer %08lx@%08lx from "
			"memory map\n",
			tegra_fb2_start, fb2_size);
	else
		tegra_fb2_size = fb2_size;

	tegra_fb_start = memblock_end_of_DRAM() - fb_size;
	if (memblock_remove(tegra_fb_start, fb_size))
		pr_err("Failed to remove framebuffer %08lx@%08lx from memory "
			"map\n",
			tegra_fb_start, fb_size);
	else
		tegra_fb_size = fb_size;

	if (tegra_fb_size)
		tegra_grhost_aperture = tegra_fb_start;

	if (tegra_fb2_size && tegra_fb2_start < tegra_grhost_aperture)
		tegra_grhost_aperture = tegra_fb2_start;

	if (tegra_carveout_size && tegra_carveout_start < tegra_grhost_aperture)
		tegra_grhost_aperture = tegra_carveout_start;

	/*
	 * TODO: We should copy the bootloader's framebuffer to the framebuffer
	 * allocated above, and then free this one.
	 */
	if (tegra_bootloader_fb_size)
		if (memblock_reserve(tegra_bootloader_fb_start,
				tegra_bootloader_fb_size))
			pr_err("Failed to reserve lp0_vec %08lx@%08lx\n",
				tegra_lp0_vec_size, tegra_lp0_vec_start);

	tegra_dump_reserved_memory();
}
#endif

void __init tegra_dump_reserved_memory(void)
{
	pr_info("Tegra reserved memory:\n"
		"LP0:                    %08lx - %08lx\n"
		"Bootloader framebuffer: %08lx - %08lx\n"
		"Framebuffer:            %08lx - %08lx\n"
		"2nd Framebuffer:         %08lx - %08lx\n"
		"Carveout:               %08lx - %08lx\n",
		tegra_lp0_vec_start,
		tegra_lp0_vec_start + tegra_lp0_vec_size - 1,
		tegra_bootloader_fb_start,
		tegra_bootloader_fb_start + tegra_bootloader_fb_size - 1,
		tegra_fb_start,
		tegra_fb_start + tegra_fb_size - 1,
		tegra_fb2_start,
		tegra_fb2_start + tegra_fb2_size - 1,
		tegra_carveout_start,
		tegra_carveout_start + tegra_carveout_size - 1);
}