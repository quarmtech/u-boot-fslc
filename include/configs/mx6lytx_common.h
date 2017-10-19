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
#if 0
#define CONFIG_EXTRA_ENV_SETTINGS \
	"script=boot.scr\0" \
	"image=uimage\0" \
	"fdt_file=undefined\0" \
	"fdt_addr=0x18000000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"console=" CONSOLE_DEV "\0" \
	"dfuspi=dfu 0 sf 0:0:10000000:0\0" \
	"dfu_alt_info_spl=spl raw 0x400\0" \
	"dfu_alt_info_img=u-boot raw 0x10000\0" \
	"dfu_alt_info=spl raw 0x400\0" \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"mmcpart=1\0" \
	"finduuid=part uuid mmc ${mmcdev}:2 uuid\0" \
	"update_sd_firmware=" \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"if mmc dev ${mmcdev}; then "	\
			"if ${get_cmd} ${update_sd_firmware_filename}; then " \
				"setexpr fw_sz ${filesize} / 0x200; " \
				"setexpr fw_sz ${fw_sz} + 1; "	\
				"mmc write ${loadaddr} 0x2 ${fw_sz}; " \
			"fi; "	\
		"fi\0" \
	EMMC_ENV	  \
	"mmcargs=setenv bootargs console=${console},${baudrate} " \
		"root=PARTUUID=${uuid} rootwait rw\0" \
	"loadbootscript=" \
		"fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source\0" \
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run finduuid; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootm ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootm; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootm; " \
		"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} " \
		"root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
	"netboot=echo Booting from net ...; " \
		"run netargs; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${image}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
		"findfdt="\
			"if test $fdt_file = undefined; then " \
				"if test $board_name = SABREAUTO && test $board_rev = MX6QP; then " \
					"setenv fdt_file imx6qp-sabreauto.dtb; fi; " \
				"if test $board_name = SABREAUTO && test $board_rev = MX6Q; then " \
					"setenv fdt_file imx6q-sabreauto.dtb; fi; " \
				"if test $board_name = SABREAUTO && test $board_rev = MX6DL; then " \
					"setenv fdt_file imx6dl-sabreauto.dtb; fi; " \
				"if test $board_name = LYTX && test $board_rev = MX6QP; then " \
					"setenv fdt_file imx6qp-sabresd.dtb; fi; " \
				"if test $board_name = LYTX && test $board_rev = MX6Q; then " \
					"setenv fdt_file avm.dtb; fi; " \
				"if test $board_name = LYTX && test $board_rev = MX6DL; then " \
					"setenv fdt_file lcflex.dtb; fi; " \
				"if test $fdt_file = undefined; then " \
					"echo WARNING: Could not determine dtb to use; fi; " \
			"fi;\0" \

#define CONFIG_BOOTCOMMAND \
	"run findfdt;" \
	"mmc dev ${mmcdev};" \
	"if mmc rescan; then " \
		"if run loadbootscript; then " \
		"run bootscript; " \
		"else " \
			"if run loadimage; then " \
				"run mmcboot; " \
			"else run netboot; " \
			"fi; " \
		"fi; " \
	"else run netboot; fi"
#endif
#define CONFIG_EXTRA_ENV_SETTINGS \
	"DEFAULT_CS=1\0" \
        "DEFAULT_KERNEL=1\0" \
        "DEFAULT_RFS=/dev/mmcblk0p3\0" \
        "autoload=no\0" \
	"baudrate="__stringify(CONFIG_BAUDRATE)"\0" \
        "boot_counter=0 \0" \
        "boot_fdt=no\0" \
        "bootargs=console=ttymxc2,115200 root=/dev/mmcblk0p3 rootwait rw \0" \
        "bootcmd=run finddtb; run check_test_cs; run load_kernel; run load_dtb; run setbootargs; run check_for_first_boot; run do_boot \0" \
        "bootdelay=3\0" \
	"first_boot_check=0\0" \
	"test_cs=undefined\0" \
	"check_for_first_boot=if test ${first_boot_check} = 0; then setenv first_boot_check 1; saveenv; fi\0" \
	"check_test_cs=if test ${test_cs} =lcflex; then run check_cs; fi\0" \
	"check_cs=if test ${kernel} = 1; then run test_cs1; else run test_cs2; fi\0" \
	"cs=1\0" \
        "default1=setexpr boot_counter ${boot_counter} + 1; saveenv\0" \
        "do_boot=bootm ${kernel_address} - ${dtb_address}\0" \
	"dtb_address=0x18000000\0" \
	"dtb_file=undefined\0" \
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
	"finddtb="\
		"if test $fdtb_file = undefined; then " \
			"if test $board_name = LYTX && test $lytx_rev = SF1; then " \
				"setenv dtb_file lcflex.dtb; setenv test_cs lcflex; fi; " \
			"if test $board_name = LYTX && test $lytx_rev = SF64; then " \
				"setenv dtb_file imx6q-sabreauto.dtb; fi; " \
			"if test $board_name = LYTX && test $lytx_rev = AVM; then " \
				"setenv dtb_file avm.dtb; setenv test_cs avm; fi; " \
			"if test $board_name = LYTX && test $lytx_rev = DVM; then " \
				"setenv dtb_file imx6qp-sabresd.dtb; fi; " \
			"if test $board_name = LYTX && test $lytx_rev = Tamarin; then " \
				"setenv dtb_file avm.dtb; setenv test_cs avm; fi; " \
			"if test $board_name = LYTX && test $lytx_rev = Argus; then " \
				"setenv dtb_file lcflex.dtb; setenv test_cs lcflex; fi; " \
				"if test $board_name = LYTX && test $lytx_rev = ArgusMV; then " \
					"setenv dtb_file lcflex.dtb; setenv test_cs lcflex; fi; " \
			"if test $dtb_file = undefined; then " \
				"echo WARNING: Could not determine dtb to use; fi; " \
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
#define CONFIG_ENV_OFFSET		(768 * 1024)
#endif

#ifdef CONFIG_MX6DL
#define CONFIG_IPUV3_CLK 198000000
#else
#define CONFIG_IPUV3_CLK 264000000
#endif


#endif                         /* __MX6LYTX_COMMON_CONFIG_H */
