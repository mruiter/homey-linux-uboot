/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the SolidRun mx6 based boards
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __MX6HOMEY_CONFIG_H
#define __MX6HOMEY_CONFIG_H

#define CONFIG_BOOTDELAY	0

#include "mx6_common.h"

#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_MMC_SUPPORT
#define CONFIG_SPL_FAT_SUPPORT

#define CONFIG_CMD_SPL
/*#define CONFIG_SPL_OS_BOOT
#define CONFIG_SPL_OF_LIBFDT
#define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTOR -1
#define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTORS -1
#define CONFIG_SYS_MMCSD_RAW_MODE_KERNEL_SECTOR -1
#define CONFIG_SPL_FS_LOAD_KERNEL_NAME "uImage"
#define CONFIG_SPL_FS_LOAD_ARGS_NAME   "fastboot.fdt" 
#define CONFIG_SYS_SPL_ARGS_ADDR       0x18000000 */

#include "imx6_spl.h"

#define CONFIG_IMX_THERMAL

#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_UART

#define CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC3_BASE_ADDR
#define CONFIG_FAT_WRITE

/* USB */
#define CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2
#define CONFIG_USB_KEYBOARD
#define CONFIG_SYS_USB_EVENT_POLL
#define CONFIG_SYS_DEVICE_NULLDEV
#define CONFIG_SILENT_CONSOLE
#define CONFIG_PREBOOT \
    "setenv stdin  nulldev; " \
    "setenv stdout nulldev; " \
    "setenv stderr serial; "

/* Command definition */

#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONFIG_CONSOLE_DEV	"ttymxc0"
#define CONFIG_MMCROOT		"/dev/mmcblk0p2"
#define CONFIG_SYS_FSL_USDHC_NUM	1
#define CONFIG_SYS_MMC_ENV_DEV		0	/* SDHC3 */

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define CONFIG_EXTRA_ENV_SETTINGS \
	"image=uImage\0" \
    "ramdisk_file=busybox.rd\0" \
    "ramdisk_addr_r=0x19000000\0" \
	"fdtfile=undefined\0" \
	"fdt_addr_r=0x18000000\0" \
	"ramdisk_force_addr=0x18000010\0" \
	"ramdisk_force_ref_addr=0x18000020\0" \
	"console=" CONFIG_CONSOLE_DEV "\0" \
	"baudrate=115200n8\0" \
	"bootm_size=0x10000000\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"mmcpart=1\0" \
	"mmcroot=" CONFIG_MMCROOT " rootwait rw\0" \
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr_r} ${fdtfile}\0" \
	"loadramdisk=fatload mmc ${mmcdev}:${mmcpart} ${ramdisk_addr_r} ${ramdisk_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
        "if run loadfdt; then " \
    	    "if test $bootmode = ramdisk && run loadramdisk; then " \
    	        "echo Loaded ramdisk.; " \
                "setenv bootargs rdinit=/sbin/init rootwait console=tty0; " \
            "else " \
                "setenv bootargs console=tty0 root=${mmcroot} rootfstype=ext4 " \
                        "rd.dm=0 rd.luks=0 rd.lvm=0 raid=noautodetect; " \
    		    "setenv ramdisk_addr_r -; " \
                /*"if spl export fdt ${loadaddr} - ${fdt_addr_r}; then " */\
                /*    "fatwrite mmc ${mmcdev}:${mmcpart} 0x1FFE7000 " CONFIG_SPL_FS_LOAD_ARGS_NAME " 0x800; fi; " */\
            "fi; " \
			"bootm ${loadaddr} ${ramdisk_addr_r} ${fdt_addr_r}; " \
		"else " \
			"echo WARN: Cannot load the DT; " \
		"fi; \0" \
	"findfdt="\
		"if test $board_rev = MX6Q ; then " \
			"setenv fdtfile imx6q-homey.dtb; fi; " \
		"if test $board_rev = MX6DL ; then " \
			"setenv fdtfile imx6dl-homey.dtb; fi; " \
		"if test $fdtfile = undefined; then " \
			"echo WARNING: Could not determine dtb to use; fi; \0"

#define CONFIG_BOOTCOMMAND \
    "mmc dev ${mmcdev}; " \
    "if test $bootmode = linux || test $bootmode = ramdisk; then " \
        "mmc read ${fdt_addr_r} 0 1; " \
        "mw.l ${ramdisk_force_ref_addr} 1 1; " \
        "if cmp.l ${ramdisk_force_addr} ${ramdisk_force_ref_addr} 1; then " \
            "setenv bootmode ramdisk; " \
            "echo Entering forced ramdisk boot...; " \
        "fi; " \
	    "run findfdt; " \
        "if mmc rescan; then " \
    		"if run loadimage; then " \
    			"run mmcboot; " \
    		"else echo Unable to boot.; fi; " \
        "else echo Unable to boot.; fi; " \
    "else if test $bootmode = flash_and_reset; then " \
        "fuse prog -y 0 6 0x10; " \
        "mw.l 20D8040 0x3060; " \
        "mw.b 20D8047 0x10; " \
        "mw.l ${ramdisk_force_addr} 1 1; " \
        "mmc write ${fdt_addr_r} 0 0xA800; " \
        "echo erasing possible update...; " \
        "mmc erase 543000 4000; " \
        "echo possible POR incoming...; " \
        "gpio clear 52; " \
        "sleep 1; " \
        "gpio input 52; " \
        "sleep 2; " \
        "echo resetting usom...; " \
        "reset; " \
    "else echo Unable to boot.; fi; " \
    "fi; "

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS           1
#define CONFIG_SYS_SDRAM_BASE          MMDC0_ARB_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Environment organization */
#define CONFIG_ENV_SIZE			(8 * 1024)
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_OFFSET		(8 * 64 * 1024)

#endif                         /* __MX6CUBOXI_CONFIG_H */
