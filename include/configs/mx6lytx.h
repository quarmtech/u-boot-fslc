/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6LYTX_CONFIG_H
#define __MX6LYTX_CONFIG_H

#ifdef CONFIG_SPL
#include "imx6_spl.h"
#endif

#define CONFIG_MACH_TYPE	3980
#define CONFIG_MXC_UART_BASE	UART3_BASE
#define CONSOLE_DEV		"ttymxc2"

#define CONFIG_SUPPORT_EMMC_BOOT /* eMMC specific */

#include "mx6lytx_common.h"

/* Falcon Mode */
#define CONFIG_SPL_FS_LOAD_ARGS_NAME	"args"
#define CONFIG_SPL_FS_LOAD_KERNEL_NAME	"uImage"
#define CONFIG_CMD_SPL
#define CONFIG_SYS_SPL_ARGS_ADDR       0x18000000
#define CONFIG_CMD_SPL_WRITE_SIZE      (128 * SZ_1K)

/* Falcon Mode - MMC support: args@1MB kernel@2MB */
#define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTOR  0x800   /* 1MB */
#define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTORS (CONFIG_CMD_SPL_WRITE_SIZE / 512)
#define CONFIG_SYS_MMCSD_RAW_MODE_KERNEL_SECTOR        0x1000  /* 2MB */

#define CONFIG_SYS_FSL_USDHC_NUM	1
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_SYS_MMC_ENV_DEV		0	/* SDHC3 */
#endif


/* I2C Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		  100000


/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR	0x08

/* Fuses */
#define CONFIG_MXC_OCOTP

#endif                         /* __MX6LYTX_CONFIG_H */
