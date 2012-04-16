ifeq ($(TARGET_BOARD_PLATFORM),tegra)

# Adaptation from Defy repo config by Tanguy Pruvot
#
#####################################################################
#
# How to use:
#
# This makefile is invoked from kernel/Android.mk. You may build
# the kernel and modules by using the "kernel" target in repo root :
#
# source build/envsetup.sh
# choosecombo 1 1 olympus eng
# make kernel
#
# It is also invoked automatically as part of the default target.
#
######################################################################
#set -x

ROOTDIR := $(shell pwd -P)/

ifneq ($(strip $(TOPDIR)),)
    ROOTDIR := $(TOPDIR)
endif

ifeq ($(KERNEL_CROSS_COMPILE),)
	KERNEL_CROSS_COMPILE=$(ROOTDIR)$(TARGET_TOOLS_PREFIX)
endif

ifeq ($(TARGET_PRODUCT),)
	TARGET_PRODUCT := generic
endif

ifeq ($(PRODUCT_OUT),)
	PRODUCT_OUT := out/target/product/$(TARGET_PRODUCT)
endif

ifeq ($(TARGET_OUT),)
	TARGET_OUT := $(PRODUCT_OUT)/system
endif

ifeq ($(HOST_PREBUILT_TAG),)
	HOST_PREBUILT_TAG := linux-x86
endif

ifeq ($(TARGET_BUILD_VARIANT),)
	TARGET_BUILD_VARIANT := user
endif

ifeq ($(HOST_OUT_EXECUTABLES),)
	HOST_OUT_EXECUTABLES := out/host/$(HOST_PREBUILT_TAG)/bin
endif

ifeq ($(DEPMOD),)
	DEPMOD := $(shell which depmod 2> /dev/null || echo $(HOST_OUT_EXECUTABLES)/depmod$(HOST_EXECUTABLE_SUFFIX))
endif

###############################################################################
# Adjust Settings here if required, or in your BoardConfig.mk
###############################################################################

ifeq ($(KERNEL_SRC_DIR),)
    KERNEL_SRC_DIR := $(ROOTDIR)kernel/moto/olympus
endif

# Default board defconfig (without defconfig suffix)
ifeq ($(BLD_CONF),)
    BLD_CONF=tegra_olympus_gb_defconfig
endif

###############################################################################

KERNEL_CROSS_COMPILE   := $(ROOTDIR)prebuilt/$(HOST_PREBUILT_TAG)/toolchain/arm-eabi-4.4.3/bin/arm-eabi-

KERNEL_BUILD_DIR       := $(ROOTDIR)$(PRODUCT_OUT)/obj/kernel_intermediates/build
TARGET_PREBUILT_KERNEL ?= $(KERNEL_BUILD_DIR)/arch/arm/boot/zImage

KERNEL_WARN_FILTER := $(KERNEL_SRC_DIR)/scripts/gcc_warn_filter.cfg
KERNEL_ERR_LOG     := $(KERNEL_BUILD_DIR)/.kbld_err_log.txt
KMOD_ERR_LOG       := $(KERNEL_BUILD_DIR)/.kmod_err_log.txt
KERNEL_FFLAG       := $(KERNEL_BUILD_DIR)/.filter_ok.txt

DEFCONFIGSRC                := ${KERNEL_SRC_DIR}/arch/arm/configs
LJAPDEFCONFIGSRC            := ${DEFCONFIGSRC}/ext_config
PRODUCT_SPECIFIC_DEFCONFIGS := $(DEFCONFIGSRC)/tegra_olympus_android_defconfig

_TARGET_DEFCONFIG           := __ext_olympus_defconfig
TARGET_DEFCONFIG            := $(DEFCONFIGSRC)/$(_TARGET_DEFCONFIG)

MOTO_MOD_INSTALL := $(TARGET_OUT)/lib/modules

WLAN_DHD_PATH := $(ROOTDIR)vendor/bcm/wlan/osrc/open-src/src/dhd/linux

# Disabled, this is made to force proper syntax commits (spaces etc)
# GIT_HOOKS_DIR := $(KERNEL_SRC_DIR)/.git/hooks
# inst_hook: $(GIT_HOOKS_DIR)/pre-commit $(GIT_HOOKS_DIR)/checkpatch.pl
inst_hook:

$(GIT_HOOKS_DIR)/pre-commit: $(KERNEL_SRC_DIR)/scripts/pre-commit
	@-cp -f $< $@
	@-chmod ugo+x $@

$(GIT_HOOKS_DIR)/checkpatch.pl: $(KERNEL_SRC_DIR)/scripts/checkpatch.pl
	@-cp -f $< $@
	@-chmod ugo+x $@

ifneq ($(BLD_CONF),)
    PRODUCT_SPECIFIC_DEFCONFIGS := $(DEFCONFIGSRC)/$(BLD_CONF)
endif

ifneq ($(PRODUCT),)
    PRODUCT_SPECIFIC_DEFCONFIGS += ${LJAPDEFCONFIGSRC}/product/${PRODUCT}.config
endif

#Turn on kernel engineering build as default when TARGET_BUILD_VARIANT is eng, to disable it, add ENG_BLD=0 in build command
ifeq ($(TARGET_BUILD_VARIANT), user)
    ENG_BLD := 0
else
    ENG_BLD := 1
endif

ENG_BLD = 0

ifeq ($(ENG_BLD),1)
    PRODUCT_SPECIFIC_DEFCONFIGS += ${LJAPDEFCONFIGSRC}/eng_bld.config
else
    PRODUCT_SPECIFIC_DEFCONFIGS += ${LJAPDEFCONFIGSRC}/user_bld.config
endif

ifeq ($(TEST_DRV_CER),1)
    ifeq ($(TEST_COVERAGE),)
            TEST_COVERAGE=1
    endif
    ifeq ($(TEST_KMEMLEAK),)
            TEST_KMEMLEAK=1
    endif
    ifeq ($(TEST_FAULTINJECT),)
            TEST_FAULTINJECT=1
    endif
endif

# Optional features
ifeq ($(TEST_COVERAGE),1)
    PRODUCT_SPECIFIC_DEFCONFIGS += ${LJAPDEFCONFIGSRC}/feature/coverage.config
endif
ifeq ($(TEST_KMEMLEAK),1)
    PRODUCT_SPECIFIC_DEFCONFIGS += ${LJAPDEFCONFIGSRC}/feature/kmemleak.config
endif
ifeq ($(TEST_FAULTINJECT),1)
    PRODUCT_SPECIFIC_DEFCONFIGS += ${LJAPDEFCONFIGSRC}/feature/faultinject.config
endif
ifeq ($(TEST_MUDFLAP),1)
    PRODUCT_SPECIFIC_DEFCONFIGS += ${LJAPDEFCONFIGSRC}/feature/mudflap.config
endif

#
# make kernel output directory structure
#---------------------------------------
$(KERNEL_BUILD_DIR):
	mkdir -p $(KERNEL_BUILD_DIR)

#
# make combined defconfig file
#---------------------------------------
$(TARGET_DEFCONFIG): FORCE $(PRODUCT_SPECIFIC_DEFCONFIGS)
	( perl -le 'print "# This file was automatically generated from:\n#\t" . join("\n#\t", @ARGV) . "\n"' $(PRODUCT_SPECIFIC_DEFCONFIGS) && cat $(PRODUCT_SPECIFIC_DEFCONFIGS) ) > $(TARGET_DEFCONFIG) || ( rm -f $@ && false )

.PHONY: FORCE
FORCE:

#
# make kernel configuration
#--------------------------
# We need to remove include/config if a make was run in kernel source tree
CONFIG_OUT := $(KERNEL_BUILD_DIR)/.config
kernel_config: $(CONFIG_OUT)
$(CONFIG_OUT): $(TARGET_DEFCONFIG) $(KERNEL_FFLAG) inst_hook | $(KERNEL_BUILD_DIR)
	@echo -e ${CL_PFX}"kernel_config"${CL_RST}
	@rm -rf $(KERNEL_SRC_DIR)/include/config
	@echo DEPMOD: $(DEPMOD)
	$(MAKE) -j1 -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
		CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
		O=$(KERNEL_BUILD_DIR) \
		KBUILD_DEFCONFIG=$(_TARGET_DEFCONFIG) \
		defconfig modules_prepare

#
# clean if warn filter changed
#-----------------------------
$(KERNEL_FFLAG): $(KERNEL_WARN_FILTER) | $(KERNEL_BUILD_DIR)
ifeq ($(KERNEL_CHECK_GCC_WARNINGS),1)
	@echo "Gcc warning filter changed, clean build will be enforced\n"
	$(MAKE) -j1 -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
                 CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
                 O=$(KERNEL_BUILD_DIR) clean
endif
	@touch $(KERNEL_FFLAG)

## fail building if there are unfiltered warnings
## bypass if TEST_MUDFLAP is 1
# $(1): input log file
ifeq ($(TEST_MUDFLAP),1)
define kernel-check-gcc-warnings
endef
else
define kernel-check-gcc-warnings
	@if [ -e $1 ]; then \
		(cat $1 | \
			$(KERNEL_SRC_DIR)/scripts/chk_gcc_warn.pl $(KERNEL_SRC_DIR) \
				$(KERNEL_WARN_FILTER)) \
			|| ((rm -f $1) && false); fi
endef
endif

#
# build kernel and internal kernel modules
# ========================================
# We need to check warning no matter if build passed, failed or interuptted
.PHONY: kernel
kernel: $(CONFIG_OUT)
	@echo -e ${CL_PFX}"kernel"${CL_RST}
	$(call kernel-check-gcc-warnings, $(KERNEL_ERR_LOG))
	$(MAKE) -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
		CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) O=$(KERNEL_BUILD_DIR) \
		zImage 2>&1 | tee $(KERNEL_ERR_LOG)
	$(call kernel-check-gcc-warnings, $(KERNEL_ERR_LOG))

#
# make kernel modules
#--------------------------
# We need to check warning no matter if build passed, failed or interuptted
.PHONY: kernel_modules
kernel_modules: $(CONFIG_OUT) | $(DEPMOD)
	@echo -e ${CL_PFX}"kernel_modules"${CL_RST}
	$(call kernel-check-gcc-warnings, $(KMOD_ERR_LOG))
	$(MAKE) -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
		CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) O=$(KERNEL_BUILD_DIR) \
		DEPMOD=$(DEPMOD) INSTALL_MOD_PATH=$(KERNEL_BUILD_DIR) \
		modules 2>&1 | tee $(KMOD_ERR_LOG)
	$(call kernel-check-gcc-warnings, $(KMOD_ERR_LOG))

# To build modules (.ko) in specific folder
# It is useful for build specific module with extra options
# (e.g. TEST_DRV_CER)
kernel_dir:
	@echo -e ${CL_PFX}"kernel_dir"${CL_RST}
	$(MAKE) -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
		CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
		O=$(KERNEL_BUILD_DIR) $(DIR_TO_BLD)

#NOTE: "strip" MUST be done for generated .ko files!!!
.PHONY: kernel_modules_install
kernel_modules_install: kernel_modules | $(DEPMOD)
	@echo -e ${CL_PFX}"kernel_modules_install"${CL_RST}
	$(MAKE) -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
		CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
		O=$(KERNEL_BUILD_DIR) \
		DEPMOD=$(DEPMOD) \
		INSTALL_MOD_PATH=$(KERNEL_BUILD_DIR) \
		modules_install

kernel_clean:
	@echo -e ${CL_PFX}"kernel_clean"${CL_RST}
	@rm -f $(PRODUCT_OUT)/kernel
	@rm -f $(PRODUCT_OUT)/boot.img
	@rm -f $(PRODUCT_OUT)/recovery.img
	$(MAKE) -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
		CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
		O=$(KERNEL_BUILD_DIR) mrproper
	@rm -f $(KERNEL_BUILD_DIR)/.*.txt
	@rm -rf $(KERNEL_SRC_DIR)/include/config
	@rm -f $(TARGET_DEFCONFIG)
	@rm -f $(MOTO_MOD_INSTALL)/*.ko
	@rm -rf $(KERNEL_BUILD_DIR)

#
#----------------------------------------------------------------------------
# To use "make ext_modules" to buld external kernel modules
#----------------------------------------------------------------------------
# build external kernel modules
#
# NOTE: "strip" MUST be done for generated .ko files!!!
# =============================
.PHONY: ext_kernel_modules
ext_kernel_modules: wlan_dhd

ext_kernel_modules_clean: wlan_dhd_clean

# wlan driver module
#-------------------
#API_MAKE = env -u MAKECMDGOALS make PREFIX=$(KERNEL_BUILD_DIR) \

API_MAKE = make PREFIX=$(KERNEL_BUILD_DIR) \
		CROSS=$(KERNEL_CROSS_COMPILE) \
		CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
		KERNEL_DIR=$(KERNEL_BUILD_DIR) \
		PROJROOT=$(PROJROOT) \
		LINUXSRCDIR=$(KERNEL_SRC_DIR) \
		LINUXBUILDDIR=$(KERNEL_BUILD_DIR) \

.PHONY: wlan_dhd
wlan_dhd: $(CONFIG_OUT)
	@echo -e ${CL_PFX}"wlan_dhd"${CL_RST}
	$(API_MAKE) -C $(WLAN_DHD_PATH)

wlan_dhd_clean:
	$(API_MAKE) -C $(WLAN_DHD_PATH) clean

#
# The below rules are for the Android build system
#-------------------------------------------------
ifneq ($(DO_NOT_REBUILD_THE_KERNEL),1)
.PHONY: $(TARGET_PREBUILT_KERNEL)
endif

$(TARGET_PREBUILT_KERNEL): kernel

device/motorola/olympus/kernel: $(TARGET_PREBUILT_KERNEL)
	cp $(KERNEL_BUILD_DIR)/arch/arm/boot/zImage $(ANDROID_BUILD_TOP)/device/motorola/olympus/kernel

$(INSTALLED_KERNEL_TARGET): $(TARGET_PREBUILT_KERNEL) | $(ACP)
	$(transform-prebuilt-to-target)

#
# install kernel modules into system image
#-----------------------------------------
# dummy.ko is used for system image dependency
TARGET_DUMMY_MODULE := $(MOTO_MOD_INSTALL)/dummy.ko
ALL_PREBUILT += $(TARGET_DUMMY_MODULE)
$(TARGET_DUMMY_MODULE): kernel ext_kernel_modules kernel_modules_install
	@echo -e ${CL_PFX}"Install kernel and modules..."${CL_RST}
	$(API_MAKE) -C $(WLAN_DHD_PATH)
	mkdir -p $(MOTO_MOD_INSTALL)
	rm -f $(MOTO_MOD_INSTALL)/dummy.ko
	find $(KERNEL_BUILD_DIR)/lib/modules -name "*.ko" -exec cp -f {} \
		$(MOTO_MOD_INSTALL) \; || true
	cp $(WLAN_DHD_PATH)/dhd.ko $(MOTO_MOD_INSTALL)
	$(KERNEL_CROSS_COMPILE)strip --strip-debug $(MOTO_MOD_INSTALL)/*.ko
	cp $(KERNEL_BUILD_DIR)/arch/arm/boot/zImage $(PRODUCT_OUT)/kernel
	touch $(MOTO_MOD_INSTALL)/dummy.ko

ROOTDIR :=

endif # tegra
