/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6LYTX_COMMON_CONFIG_H
#define __MX6LYTX_COMMON_CONFIG_H

#include "mx6_common.h"

#define CONFIG_IMX_THERMAL

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_MXC_UART

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR      USDHC3_BASE_ADDR

#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		MII10
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		1

#define CONFIG_PHYLIB
#define CONFIG_PHY_ATHEROS

#include "../../board/lytx/mx6lytx/lcflex_bootloader_version.h" /* defines LCFLEX_BOOTLOADER_VERSION */

#ifdef CONFIG_CMD_SF
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS		0
#define CONFIG_SF_DEFAULT_CS		0
#define CONFIG_SF_DEFAULT_SPEED		20000000
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#endif

#ifdef CONFIG_SUPPORT_EMMC_BOOT
#define EMMC_ENV \
	"emmcdev=2\0" \
	"update_emmc_firmware=" \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"if ${get_cmd} ${update_sd_firmware_filename}; then " \
			"if mmc dev ${emmcdev} 0; then "	\
				"setexpr fw_sz ${filesize} / 0x200; " \
				"setexpr fw_sz ${fw_sz} + 1; "	\
				"mmc write ${loadaddr} 0x2 ${fw_sz}; " \
			"fi; "	\
		"fi\0"
#else
#define EMMC_ENV ""
#endif

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

#define CONFIG_PREBOOT ""

#define CONFIG_EXTRA_ENV_SETTINGS \
	"DEFAULT_CS=1\0" \
        "DEFAULT_KERNEL=1\0" \
        "DEFAULT_RFS=/dev/mmcblk0p3\0" \
        "autoload=no\0" \
	"baudrate="__stringify(CONFIG_BAUDRATE)"\0" \
        "boot_counter=0 \0" \
        "boot_fdt=no\0" \
        "bootargs=console=ttymxc2,115200 root=/dev/mmcblk0p3 rootwait rw \0" \
        "bootcmd=run findtest; run check_test_cs; run load_kernel; run load_dtb; run setbootargs; run check_for_first_boot; run do_boot \0" \
        "bootdelay=3\0" \
	"first_boot_check=0\0" \
	"test_cs=undefined\0" \
	"check_for_first_boot=if test ${first_boot_check} = 0; then setenv first_boot_check 1; saveenv; fi\0" \
	"check_test_cs=if test ${test_cs} = yes; then run check_cs; fi\0" \
	"check_cs=if test ${kernel} = 1; then run test_cs1; else run test_cs2; fi\0" \
	"cs=1\0" \
        "default1=setexpr boot_counter ${boot_counter} + 1; saveenv\0" \
        "do_boot=bootm ${kernel_address} - ${dtb_address}\0" \
	"dtb_address=0x18000000\0" \
	"dtb_file=lcflex.dtb\0" \
	"ethact=FEC\0" \
	"ethprime=FEC\0" \
        "kernel1=1\0" \
        "kernel2=2\0" \
	"kernel_address=0x12000000\0" \
        "kernel_file=uImage\0" \
	"linuxconsole=ttymxc2,115200\0" \
	"load_dtb=fatload mmc ${mmcdev}:${kernel} ${dtb_address} ${dtb_file}\0" \
	"load_kernel=fatload mmc ${mmcdev}:${kernel} ${kernel_address} ${kernel_file}\0" \
	"loadaddr=0x12000000\0" \
	"max_boot_attempts=2\0" \
        "mmcdev=0\0" \
	"netmask=255.0.0.0\0" \
        "rfs1=/dev/mmcblk0p3\0" \
        "rfs2=/dev/mmcblk0p5\0" \
        "serverip=192.168.20.119\0" \
	"setbootargs=setenv bootargs console=${linuxconsole} root=${rfs} rootfstype=ext4 ro rootwait \0" \
	"setwdog=setenv wdogTimeout ${wdogTimeout};saveenv\0" \
	"switch1=setenv kernel ${kernel1}; setenv rfs ${rfs1}; setenv cs 1; setenv boot_counter 0; setenv cs2_status BAD; echo Using k1 and rfs1;saveenv\0" \
	"switch2=setenv kernel ${kernel2}; setenv rfs ${rfs2}; setenv cs 2; setenv boot_counter 0; setenv cs1_status BAD; echo Using k2 and rfs2;saveenv\0" \
	"test_cs1=echo Current codeset 1 is ${cs1_status}; if test ${cs1_status} = FAIL; then run switch_cs2; fi\0" \
	"test_cs2=echo Current codeset 2 is ${cs2_status}; if test ${cs2_status} = FAIL; then run switch_cs1; fi\0" \
 	"mmcautodetect=yes\0" \
        "rfs=/dev/mmcblk0p3\0" \
        "kernel=1\0" \
        "cs=1\0" \
        "cs1_status=GOOD\0" \
        "cs2_status=GOOD\0" \
        "wdogTimeout=61\0" \
	"findtest="\
		"if test $test_cs = undefined; then " \
			"if test $board_name = LYTX && test $lytx_rev = SF1; then " \
				"setenv test_cs yes; fi; " \
			"if test $board_name = LYTX && test $lytx_rev = SF64; then " \
				"setenv test_cs yes; fi; " \
			"if test $board_name = LYTX && test $lytx_rev = AVM; then " \
				"setenv test_cs avm; fi; " \
			"if test $board_name = LYTX && test $lytx_rev = DVM; then " \
				"fi; " \
			"if test $board_name = LYTX && test $lytx_rev = Tamarin; then " \
				"setenv test_cs yes; fi; " \
			"if test $board_name = LYTX && test $lytx_rev = Argus; then " \
				"fi; " \
			"if test $board_name = LYTX && test $lytx_rev = ArgusMV; then " \
				"fi; " \
		"fi;\0" \

#define CONFIG_BOOTCOMMAND \
				"run bootcmd" \


#define CONFIG_ARP_TIMEOUT     200UL

#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END         0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS           1
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Environment organization */
#define CONFIG_ENV_SIZE			(8 * 1024)

#define CONFIG_ENV_IS_IN_MMC

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(6 * 64 * 1024)
/*
#define CONFIG_ENV_OFFSET		(768 * 1024)
*/
#endif

#ifdef CONFIG_MX6DL
#define CONFIG_IPUV3_CLK 198000000
#else
#define CONFIG_IPUV3_CLK 264000000
#endif


#endif                         /* __MX6LYTX_COMMON_CONFIG_H */
