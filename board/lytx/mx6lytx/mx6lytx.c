/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <linux/errno.h>
#include <asm/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/video.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "pfuze.h"
#include <usb.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define I2C_PMIC	0

#define I2C_PAD MUX_PAD_CTRL(I2C_PAD_CTRL)

/*
#define DISP0_PWR_EN	IMX_GPIO_NR(1, 21)

#define KEY_VOL_UP	IMX_GPIO_NR(1, 4)
*/
#define IMX6_ISL_RST IMX_GPIO_NR(3, 8)
#define IMX6_ISL_PD	IMX_GPIO_NR(3, 8)

#define ANATOP_PLL_LOCK             	0x80000000
#define ANATOP_PLL_PWDN_MASK        	0x00001000
#define ANATOP_PLL_BYPASS_MASK      	0x00010000
#define ANATOP_FEC_PLL_ENABLE_MASK  	0x00002000

static int setup_fec(void)
{
        u32 reg = 0;
        /* Select ENET CLK source   */
        reg =  readl(IOMUXC_BASE_ADDR + 0x4);
        reg |= (0x1 << 21);
        writel(reg, IOMUXC_BASE_ADDR + 0x4);


        /* Configure and Start the ENET PLL */
        /* diviseur => 25 MHz */
        writel (BP_ANADIG_PLL_ENET_DIV_SELECT, ANATOP_BASE_ADDR + 0xe0);
        writel (0x0, ANATOP_BASE_ADDR + 0xe0);

        /* clear bypass et powerdown */
        writel (BM_ANADIG_PLL_ENET_BYPASS |  BM_ANADIG_PLL_ENET_POWERDOWN, ANATOP_BASE_ADDR + 0xe0);
        /* enable */
        writel (BM_ANADIG_PLL_ENET_ENABLE, ANATOP_BASE_ADDR + 0xe0);

        /* Release PHY reset at least 30ms after the clock setup*/
        mdelay (50);
     	gpio_set_value(IMX_GPIO_NR(3, 16), 1);
        return 0;
}

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

static iomux_v3_cfg_t const uart3_pads[] = {
	IOMUX_PADS(PAD_SD4_CMD__UART3_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_CLK__UART3_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

static iomux_v3_cfg_t const enet_pads[] = {
	IOMUX_PADS(PAD_KEY_ROW1__ENET_COL	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_COL3__ENET_CRS	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_MDC__ENET_MDC	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_MDIO__ENET_MDIO	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_GPIO_18__ENET_RX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_RXD0__ENET_RX_DATA0	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_RXD1__ENET_RX_DATA1	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_COL2__ENET_RX_DATA2	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_COL0__ENET_RX_DATA3	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_CRS_DV__ENET_RX_EN	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_RX_ER__ENET_RX_ER	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_TXD0__ENET_TX_DATA0	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_TXD1__ENET_TX_DATA1	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_ROW2__ENET_TX_DATA2	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_ROW0__ENET_TX_DATA3	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_TX_EN__ENET_TX_EN	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_GPIO_19__ENET_TX_ER	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA3__GPIO3_IO03	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA2__GPIO3_IO02	| MUX_PAD_CTRL(ENET_PAD_CTRL)),
	IOMUX_PADS(PAD_GPIO_16__ENET_REF_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL | PAD_CTL_SRE_FAST)),
	/* AR8031 PHY Reset */
	IOMUX_PADS(PAD_EIM_D16__GPIO3_IO16	| MUX_PAD_CTRL(NO_PAD_CTRL)),
};
/* Uncomment when adding wdog
static iomux_v3_cfg_t const wdog_pads[] = {
	IOMUX_PADS(PAD_SD1_DAT2__WDOG1_RESET_B_DEB | MUX_PAD_CTRL(NO_PAD_CTRL)),
};
*/

static void setup_iomux_enet(void)
{
	SETUP_IOMUX_PADS(enet_pads);

	/* Reset AR8031 PHY */
	gpio_direction_output(IMX_GPIO_NR(3, 16) , 0);
	#if 0
	mdelay(10);
	gpio_set_value(IMX_GPIO_NR(3, 16), 1);
	#endif
	udelay(100);
}

#if 0
static iomux_v3_cfg_t const usdhc2_pads[] = {
	IOMUX_PADS(PAD_SD2_CLK__SD2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_CMD__SD2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT0__SD2_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT1__SD2_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT2__SD2_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT3__SD2_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D4__SD2_DATA4	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D5__SD2_DATA5	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D6__SD2_DATA6	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D7__SD2_DATA7	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D2__GPIO2_IO02	| MUX_PAD_CTRL(NO_PAD_CTRL)), /* CD */
};
#endif

static iomux_v3_cfg_t const usdhc3_pads[] = {
	IOMUX_PADS(PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT4__SD3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT5__SD3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT6__SD3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT7__SD3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};

#if 0
static iomux_v3_cfg_t const usdhc4_pads[] = {
	IOMUX_PADS(PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};

static iomux_v3_cfg_t const ecspi1_pads[] = {
	IOMUX_PADS(PAD_KEY_COL0__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_COL1__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_ROW0__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_ROW1__GPIO4_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static iomux_v3_cfg_t const rgb_pads[] = {
	IOMUX_PADS(PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN15__IPU1_DI0_PIN15 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN2__IPU1_DI0_PIN02 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN3__IPU1_DI0_PIN03 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN4__IPU1_DI0_PIN04 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT0__IPU1_DISP0_DATA00 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT1__IPU1_DISP0_DATA01 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT2__IPU1_DISP0_DATA02 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT3__IPU1_DISP0_DATA03 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT4__IPU1_DISP0_DATA04 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT5__IPU1_DISP0_DATA05 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT6__IPU1_DISP0_DATA06 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT7__IPU1_DISP0_DATA07 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT8__IPU1_DISP0_DATA08 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT9__IPU1_DISP0_DATA09 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT10__IPU1_DISP0_DATA10 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT11__IPU1_DISP0_DATA11 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT12__IPU1_DISP0_DATA12 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT13__IPU1_DISP0_DATA13 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT14__IPU1_DISP0_DATA14 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT15__IPU1_DISP0_DATA15 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT16__IPU1_DISP0_DATA16 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT17__IPU1_DISP0_DATA17 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT18__IPU1_DISP0_DATA18 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT19__IPU1_DISP0_DATA19 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT20__IPU1_DISP0_DATA20 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT21__IPU1_DISP0_DATA21 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT22__IPU1_DISP0_DATA22 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT23__IPU1_DISP0_DATA23 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static iomux_v3_cfg_t const bl_pads[] = {
	IOMUX_PADS(PAD_SD1_DAT3__GPIO1_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static void enable_backlight(void)
{
	SETUP_IOMUX_PADS(bl_pads);
	gpio_direction_output(DISP0_PWR_EN, 1);
}

static void enable_rgb(struct display_info_t const *dev)
{
	SETUP_IOMUX_PADS(rgb_pads);
	enable_backlight();
}

static void enable_lvds(struct display_info_t const *dev)
{
	enable_backlight();
}
#endif



#if 0
static void setup_spi(void)
{
	SETUP_IOMUX_PADS(ecspi1_pads);
}

iomux_v3_cfg_t const pcie_pads[] = {
	IOMUX_PADS(PAD_EIM_D19__GPIO3_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL)),	/* POWER */
	IOMUX_PADS(PAD_GPIO_17__GPIO7_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL)),	/* RESET */
};

static void setup_pcie(void)
{
	SETUP_IOMUX_PADS(pcie_pads);
}

iomux_v3_cfg_t const di0_pads[] = {
	IOMUX_PADS(PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK),	/* DISP0_CLK */
	IOMUX_PADS(PAD_DI0_PIN2__IPU1_DI0_PIN02),		/* DISP0_HSYNC */
	IOMUX_PADS(PAD_DI0_PIN3__IPU1_DI0_PIN03),		/* DISP0_VSYNC */
};
#endif

static void setup_iomux_uart(void)
{
	SETUP_IOMUX_PADS(uart3_pads);
}

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[1] = {
	{USDHC3_BASE_ADDR},
};
static iomux_v3_cfg_t const power_pads[] = {
	IOMUX_PADS(PAD_EIM_DA8__GPIO3_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA9__GPIO3_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};
static void setup_iomux_power(void)
{
	SETUP_IOMUX_PADS(power_pads);
	gpio_direction_output(IMX6_ISL_RST, 1);
	gpio_direction_output(IMX6_ISL_PD, 1);
}

#if 0
#define USDHC2_CD_GPIO	IMX_GPIO_NR(2, 2)
#define USDHC3_CD_GPIO	IMX_GPIO_NR(2, 0)
#endif

int board_mmc_get_env_dev(int devno)
{
	return devno - 2;
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC3_BASE_ADDR:
		ret = 1;
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
/*
	#ifndef CONFIG_SPL_BUILD
	int ret;
	int i;
	printf("Not using SPL Build\n");

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc0                    SD2
	 * mmc1                    SD3
	 * mmc2                    eMMC
	 */
	/*for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			SETUP_IOMUX_PADS(usdhc3_pads);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the ?board (%d)\n",
			       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

	return 0;
#else */
	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	unsigned reg = readl(&psrc->sbmr1) >> 11;
	/*
	 * Upon reading BOOT_CFG register the following map is done:
	 * Bit 11 and 12 of BOOT_CFG register can determine the current
	 * mmc port
	 * 0x1                  SD1
	 * 0x2                  SD2
	 * 0x3                  SD4
	 */
	 printf("Using SPL Build\n");

	switch (reg & 0x3) {
	case 0x2:
		SETUP_IOMUX_PADS(usdhc3_pads);
		usdhc_cfg[0].esdhc_base = USDHC3_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	}

	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
/*#endif*/
}
#endif

static int ar8031_phy_fixup(struct phy_device *phydev)
{
	unsigned short val;

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);

	val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);

	/* introduce tx clock delay */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x5);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	val |= 0x0100;
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	ar8031_phy_fixup(phydev);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}




/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_eth_init(bd_t *bis)
{
	setup_iomux_enet();
	setup_fec();

	return cpu_eth_init(bis);
}


int board_early_init_f(void)
{
	setup_iomux_uart();
/* BBP Need to add WDOG*/
	setup_iomux_power();
	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
#if 0
	if (is_mx6dq() || is_mx6dqp())
	{
		setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6q_i2c_pad_info1);
		setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6q_i2c_pad_info3);
	}
	else
	{
		setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6dl_i2c_pad_info1);
		setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6dl_i2c_pad_info3);
	}
#endif
	return 0;
}

int power_init_board(void)
{
	struct pmic *p;
	unsigned int reg;
	int ret;

	p = pfuze_common_init(I2C_PMIC);
	if (!p)
		return -ENODEV;

	ret = pfuze_mode_init(p, APS_PFM);
	if (ret < 0)
		return ret;

	/* Increase VGEN3 from 2.5 to 2.8V */
	pmic_reg_read(p, PFUZE100_VGEN3VOL, &reg);
	reg &= ~LDO_VOL_MASK;
	reg |= LDOB_2_80V;
	pmic_reg_write(p, PFUZE100_VGEN3VOL, reg);

	/* Increase VGEN5 from 2.8 to 3V */
	pmic_reg_read(p, PFUZE100_VGEN5VOL, &reg);
	reg &= ~LDO_VOL_MASK;
	reg |= LDOB_3_00V;
	pmic_reg_write(p, PFUZE100_VGEN5VOL, reg);

	return 0;
}


#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd2",	 MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{"sd3",	 MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	/* 8 bit bus width */
	{"emmc", MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
	#if 0
	static char dig_pot_reg_val_array[6]={0x64,0x5a,0x50,0x46,0x3c,0x32};
	unsigned char read_val;
	unsigned char write_val;
	unsigned int loop_var;
#endif

	#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
	#endif
	/* BBP Copied code from Lytx */
	#if 0
	i2c_read(0x2f,5,1,&read_val,1);
	write_val=0;
	if(read_val>(dig_pot_reg_val_array[0]))
	{
		write_val=dig_pot_reg_val_array[0];
	}

	if(write_val==0)
	{
		for(loop_var=0; ((loop_var < 5) && (write_val == 0));loop_var++)
		{
			if(read_val >= (dig_pot_reg_val_array[loop_var]+((dig_pot_reg_val_array[0]- dig_pot_reg_val_array[1])/2)))
			{
				write_val= dig_pot_reg_val_array[loop_var];
			}
		}

		if(write_val == 0)
		{
			write_val = dig_pot_reg_val_array[5];
		}

	}

	i2c_write(0x2f,2,1,&write_val,1);
	i2c_write(0x2f,1,1,&write_val,1);

	i2c_read(0x2f,5,1,&read_val,1);

#ifdef MFGTOOL_UBOOT
	fuse_prog(0, 5, 0x5060);
	fuse_prog(0, 6, 0x10);

	int val1;
	int val2;
	fuse_sense(0, 5, &val1);
	fuse_sense(0, 6, &val2);
	printf("OTP flashed !!\n");
#else
		#define WDOG_ENABLE_FUSE_MASK     0x200000

		int fuse_val;
		/* Burn fuse to enable the 90 sec serial downloader watchdog timer at
		* boot if it hasn't yet been enabled. */
		fuse_sense(0, 6, &fuse_val);
		if ( (fuse_val & WDOG_ENABLE_FUSE_MASK) == 0 )
		{
			fuse_prog(0, 6, WDOG_ENABLE_FUSE_MASK);
		}

#endif
#endif
/* End of copy */
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	setenv("board_name", "LYTX");

	if (is_mx6dqp())
		setenv("board_rev", "MX6QP");
	else if (is_mx6dq())
		setenv("board_rev", "MX6Q");
	else if (is_mx6sdl())
		setenv("board_rev", "MX6DL");
#endif

	int lytx_brd;
	i2c_set_bus_num(2);
	lytx_brd = i2c_reg_read(0x53, 14);

switch(lytx_brd){
	case 0:
		printf("Lytx Board SF1\n");
		setenv("lytx_rev", "SF1");
		break;
	case 1:
		printf("Lytx Board SF64\n");
		setenv("lytx_rev", "SF64");
		break;
	case 2:
		printf("Lytx Board AVM\n");
		setenv("lytx_rev", "AVM");
		break;
	case 3:
		printf("Lytx Board DVM\n");
		setenv("lytx_rev", "DVM");
		break;
	case 4:
		printf("Lytx Board Tamarin\n");
		setenv("lytx_rev", "Tamarin");
		break;
	case 5:
		printf("Lytx Board Argus\n");
		setenv("lytx_rev", "Argus");
		break;
	case 6:
		printf("Lytx Board ArgusMV\n");
		setenv("lytx_rev", "ArgusMV");
		break;
	default:
		printf("Lytx Board Default SF1\n");
		setenv("lytx_rev", "SF1");
		break;

}

	return 0;
}

int checkboard(void)
{
	puts("Board: MX6-Lytx\n");
	return 0;
}

#ifdef CONFIG_SPL_BUILD
#include <asm/arch/mx6-ddr.h>
#include <spl.h>
#include <libfdt.h>
static struct i2c_pads_info mx6q_i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6Q_PAD_CSI0_DAT9__I2C1_SCL | I2C_PAD,
		.gpio_mode = MX6Q_PAD_CSI0_DAT9__GPIO5_IO27 | I2C_PAD,
		.gp = IMX_GPIO_NR(5, 27)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_CSI0_DAT8__I2C1_SDA | I2C_PAD,
		.gpio_mode = MX6Q_PAD_CSI0_DAT8__GPIO5_IO26 | I2C_PAD,
		.gp = IMX_GPIO_NR(5, 26)
	}
};

static struct i2c_pads_info mx6dl_i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6DL_PAD_CSI0_DAT9__I2C1_SCL | I2C_PAD,
		.gpio_mode = MX6DL_PAD_CSI0_DAT9__GPIO5_IO27 | I2C_PAD,
		.gp = IMX_GPIO_NR(5, 27)
	},
	.sda = {
		.i2c_mode = MX6DL_PAD_CSI0_DAT8__I2C1_SDA | I2C_PAD,
		.gpio_mode = MX6DL_PAD_CSI0_DAT8__GPIO5_IO26 | I2C_PAD,
		.gp = IMX_GPIO_NR(5, 26)
	}
};
static struct i2c_pads_info mx6q_i2c_pad_info3 = {
	.scl = {
		.i2c_mode = MX6Q_PAD_GPIO_5__I2C3_SCL | I2C_PAD,
		.gpio_mode = MX6Q_PAD_GPIO_5__GPIO1_IO05 | I2C_PAD,
		.gp = IMX_GPIO_NR(1, 5)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_GPIO_6__I2C3_SDA | I2C_PAD,
		.gpio_mode = MX6Q_PAD_GPIO_6__GPIO1_IO06 | I2C_PAD,
		.gp = IMX_GPIO_NR(1, 6)
	}
};
static struct i2c_pads_info mx6dl_i2c_pad_info3 = {
	.scl = {
		.i2c_mode = MX6DL_PAD_GPIO_5__I2C3_SCL | I2C_PAD,
		.gpio_mode = MX6DL_PAD_GPIO_5__GPIO1_IO05 | I2C_PAD,
		.gp = IMX_GPIO_NR(1, 5)
	},
	.sda = {
		.i2c_mode = MX6DL_PAD_GPIO_6__I2C3_SDA | I2C_PAD,
		.gpio_mode = MX6DL_PAD_GPIO_6__GPIO1_IO06 | I2C_PAD,
		.gp = IMX_GPIO_NR(1, 6)
	}
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0x00C03F3F, &ccm->CCGR0);
	writel(0x0030FC03, &ccm->CCGR1);
	writel(0x0FFFC000, &ccm->CCGR2);
	writel(0x3FF00000, &ccm->CCGR3);
	writel(0x00FFF300, &ccm->CCGR4);
	writel(0x0F0000C3, &ccm->CCGR5);
	writel(0x000003FF, &ccm->CCGR6);
}

static void gpr_init(void)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	/* enable AXI cache for VDOA/VPU/IPU */
	writel(0xF00000CF, &iomux->gpr[4]);
	if (is_mx6dqp()) {
		/* set IPU AXI-id1 Qos=0x1 AXI-id0/2/3 Qos=0x7 */
		writel(0x007F007F, &iomux->gpr[6]);
		writel(0x007F007F, &iomux->gpr[7]);
	} else {
		/* set IPU AXI-id0 Qos=0xf(bypass) AXI-id1 Qos=0x7 */
		writel(0x007F007F, &iomux->gpr[6]);
		writel(0x007F007F, &iomux->gpr[7]);
	}
}

static int avm_dcd_table[] = {
	0x020e0798, 0x000C0000,
	0x020e0758, 0x00000000,
	0x020e0588, 0x00000028,
	0x020e0594, 0x00000028,
	0x020e056c, 0x00000028,
	0x020e0578, 0x00000028,
	0x020e074c, 0x00000028,
	0x020e057c, 0x00000028,
	0x020e058c, 0x00000000,
	0x020e059c, 0x00000028,
	0x020e05a0, 0x00000028,
	0x020e078c, 0x00000028,
	0x020e0750, 0x00020000,
	0x020e05a8, 0x00000028,
	0x020e05b0, 0x00000028,
	0x020e0524, 0x00000028,
	0x020e051c, 0x00000028,
	0x020e0518, 0x00000028,
	0x020e050c, 0x00000028,
	0x020e05b8, 0x00000028,
	0x020e05c0, 0x00000028,
	0x020e0774, 0x00020000,
	0x020e0784, 0x00000028,
	0x020e0788, 0x00000028,
	0x020e0794, 0x00000028,
	0x020e079c, 0x00000028,
	0x020e07a0, 0x00000028,
	0x020e07a4, 0x00000028,
	0x020e07a8, 0x00000028,
	0x020e0748, 0x00000028,
	0x020e05ac, 0x00000028,
	0x020e05b4, 0x00000028,
	0x020e0528, 0x00000028,
	0x020e0520, 0x00000028,
	0x020e0514, 0x00000028,
	0x020e0510, 0x00000028,
	0x020e05bc, 0x00000028,
	0x020e05c4, 0x00000028,
	0x021b001c, 0x00008000,
	0x021b0800, 0xa1390003,
	0x021b080c, 0x0018001c,
	0x021b0810, 0x0029001F,
	0x021b480c, 0x000c001c,
	0x021b4810, 0x00060016,
	0x021b083c, 0x032c0334,
	0x021b0840, 0x03200324,
	0x021b483c, 0x03340344,
	0x021b4840, 0x03200270,
	0x021b0848, 0x3630302c,
	0x021b4848, 0x2e2c2638,
	0x021b0850, 0x3c38423e,
	0x021b4850, 0x402e4a40,
	0x021b081c, 0x33333333,
	0x021b0820, 0x33333333,
	0x021b0824, 0x33333333,
	0x021b0828, 0x33333333,
	0x021b481c, 0x33333333,
	0x021b4820, 0x33333333,
	0x021b4824, 0x33333333,
	0x021b4828, 0x33333333,
	0x021b08c0, 0x24911492,
	0x021b48c0, 0x24911492,
	0x021b08b8, 0x00000800,
	0x021b48b8, 0x00000800,
	0x021b0004, 0x00020036,
	0x021b0008, 0x09444040,
	0x021b000c, 0x3a3f78f5,
	0x021b0010, 0xFF538F64,
	0x021b0014, 0x01FF00DB,
	0x021b0018, 0x00011740,
	0x021b001c, 0x00008000,
	0x021b002c, 0x000026d2,
	0x021b0030, 0x003f1023,
	0x021b0040, 0x00000017,
	0x021b0000, 0x821A0000,
	0x021b001c, 0x02088032,
	0x021b001c, 0x00008033,
	0x021b001c, 0x00048031,
	0x021b001c, 0x19408030,
	0x021b001c, 0x04008040,
	0x021b0020, 0x00007800,
	0x021b0818, 0x00022227,
	0x021b4818, 0x00022227,
	0x021b0004, 0x00025576,
	0x021b0404, 0x00011006,
	0x021b001c, 0x00000000,
};

static int mx6qp_dcd_table[] = {
	0x020e0798, 0x000c0000,
	0x020e0758, 0x00000000,
	0x020e0588, 0x00000030,
	0x020e0594, 0x00000030,
	0x020e056c, 0x00000030,
	0x020e0578, 0x00000030,
	0x020e074c, 0x00000030,
	0x020e057c, 0x00000030,
	0x020e058c, 0x00000000,
	0x020e059c, 0x00000030,
	0x020e05a0, 0x00000030,
	0x020e078c, 0x00000030,
	0x020e0750, 0x00020000,
	0x020e05a8, 0x00000030,
	0x020e05b0, 0x00000030,
	0x020e0524, 0x00000030,
	0x020e051c, 0x00000030,
	0x020e0518, 0x00000030,
	0x020e050c, 0x00000030,
	0x020e05b8, 0x00000030,
	0x020e05c0, 0x00000030,
	0x020e0774, 0x00020000,
	0x020e0784, 0x00000030,
	0x020e0788, 0x00000030,
	0x020e0794, 0x00000030,
	0x020e079c, 0x00000030,
	0x020e07a0, 0x00000030,
	0x020e07a4, 0x00000030,
	0x020e07a8, 0x00000030,
	0x020e0748, 0x00000030,
	0x020e05ac, 0x00000030,
	0x020e05b4, 0x00000030,
	0x020e0528, 0x00000030,
	0x020e0520, 0x00000030,
	0x020e0514, 0x00000030,
	0x020e0510, 0x00000030,
	0x020e05bc, 0x00000030,
	0x020e05c4, 0x00000030,
	0x021b0800, 0xa1390003,
	0x021b080c, 0x001b001e,
	0x021b0810, 0x002e0029,
	0x021b480c, 0x001b002a,
	0x021b4810, 0x0019002c,
	0x021b083c, 0x43240334,
	0x021b0840, 0x0324031a,
	0x021b483c, 0x43340344,
	0x021b4840, 0x03280276,
	0x021b0848, 0x44383A3E,
	0x021b4848, 0x3C3C3846,
	0x021b0850, 0x2e303230,
	0x021b4850, 0x38283E34,
	0x021b081c, 0x33333333,
	0x021b0820, 0x33333333,
	0x021b0824, 0x33333333,
	0x021b0828, 0x33333333,
	0x021b481c, 0x33333333,
	0x021b4820, 0x33333333,
	0x021b4824, 0x33333333,
	0x021b4828, 0x33333333,
	0x021b08c0, 0x24912249,
	0x021b48c0, 0x24914289,
	0x021b08b8, 0x00000800,
	0x021b48b8, 0x00000800,
	0x021b0004, 0x00020036,
	0x021b0008, 0x24444040,
	0x021b000c, 0x555A7955,
	0x021b0010, 0xFF320F64,
	0x021b0014, 0x01ff00db,
	0x021b0018, 0x00001740,
	0x021b001c, 0x00008000,
	0x021b002c, 0x000026d2,
	0x021b0030, 0x005A1023,
	0x021b0040, 0x00000027,
	0x021b0400, 0x14420000,
	0x021b0000, 0x831A0000,
	0x021b0890, 0x00400C58,
	0x00bb0008, 0x00000000,
	0x00bb000c, 0x2891E41A,
	0x00bb0038, 0x00000564,
	0x00bb0014, 0x00000040,
	0x00bb0028, 0x00000020,
	0x00bb002c, 0x00000020,
	0x021b001c, 0x04088032,
	0x021b001c, 0x00008033,
	0x021b001c, 0x00048031,
	0x021b001c, 0x09408030,
	0x021b001c, 0x04008040,
	0x021b0020, 0x00005800,
	0x021b0818, 0x00011117,
	0x021b4818, 0x00011117,
	0x021b0004, 0x00025576,
	0x021b0404, 0x00011006,
	0x021b001c, 0x00000000,
};

static int sf1_dcd_table[] = {
	//0x020c4068, 0xffffffff,
	//0x020c406c, 0xffffffff,
	//0x020c4070, 0xffffffff,
	//0x020c4074, 0xffffffff,
	//0x020c4078, 0xffffffff,
	//0x020c407c, 0xffffffff,
	//0x020c4080, 0xffffffff,
	//0x020c4084, 0xffffffff,
	0x020e0774, 0x000C0000,
	0x020e0754, 0x00000000,
	0x020e04ac, 0x00000028,
	0x020e04b0, 0x00000028,
	0x020e0464, 0x00000028,
	0x020e0490, 0x00000028,
	0x020e074c, 0x00000028,
	0x020e0494, 0x00000028,
	0x020e04a0, 0x00000000,
	0x020e04b4, 0x00000028,
	0x020e04b8, 0x00000028,
	0x020e076c, 0x00000028,
	0x020e0750, 0x00020000,
	0x020e04bc, 0x00000028,
	0x020e04c0, 0x00000028,
	0x020e04c4, 0x00000028,
	0x020e04c8, 0x00000028,
	0x020e04cc, 0x00000028,
	0x020e04d0, 0x00000028,
	0x020e04d4, 0x00000028,
	0x020e04d8, 0x00000028,
	0x020e0760, 0x00020000,
	0x020e0764, 0x00000028,
	0x020e0770, 0x00000028,
	0x020e0778, 0x00000028,
	0x020e077c, 0x00000028,
	0x020e0780, 0x00000028,
	0x020e0784, 0x00000028,
	0x020e078c, 0x00000028,
	0x020e0748, 0x00000028,
	0x020e0470, 0x00000028,
	0x020e0474, 0x00000028,
	0x020e0478, 0x00000028,
	0x020e047c, 0x00000028,
	0x020e0480, 0x00000028,
	0x020e0484, 0x00000028,
	0x020e0488, 0x00000028,
	0x020e048c, 0x00000028,
	0x021b001c, 0x00008000,
	0x021b0800, 0xA1390003,
	0x021b080c, 0x001F001F,
	0x021b0810, 0x001F001F,
	0x021b480c, 0x001F001F,
	0x021b4810, 0x001F001F,
	0x021b083c, 0x422C022C,
	0x021b083c, 0x02180220,
	0x021b0840, 0x0207017E,
	0x021b483c, 0x4201020C,
	0x021b4840, 0x01660172,
	0x021b0848, 0x484A4E4C,
	0x021b4848, 0x40404040,
	0x021b0850, 0x3A363432,
	0x021b4850, 0x40404040,
	0x021b081c, 0x33333333,
	0x021b0820, 0x33333333,
	0x021b0824, 0x33333333,
	0x021b0828, 0x33333333,
	0x021b481c, 0x33333333,
	0x021b4820, 0x33333333,
	0x021b4824, 0x33333333,
	0x021b4828, 0x33333333,
	0x021b08b8, 0x00000800,
	0x021b48b8, 0x00000800,
	0x021b0004, 0x0002002D,
	0x021b0008, 0x00333040,
	0x021b000c, 0x3F435313,
	0x021b0010, 0xB66E8B63,
	0x021b0014, 0x01FF00DB,
	0x021b0018, 0x00011740,
	0x021b001c, 0x00008000,
	0x021b002c, 0x000026D2,
	0x021b0030, 0x00431023,
	0x021b0040, 0x00000017,
	0x021b0000, 0x83190000,
	0x021b001c, 0x02808032,
	0x021b001c, 0x00008033,
	0x021b001c, 0x00048031,
	0x021b001c, 0x15208030,
	0x021b001c, 0x04008040,
	0x021b0020, 0x00007800,
	0x021b0818, 0x00022227,
	0x021b4818, 0x00022227,
	0x021b0004, 0x0002556D,
	0x021b0404, 0x00011006,
	0x021b001c, 0x00000000,

};

static void ddr_init(int *table, int size)
{
	int i;

	for (i = 0; i < size / 2 ; i++)
		writel(table[2 * i + 1], table[2 * i]);
}

static void spl_dram_init(int ddr_rev)
{
	switch(ddr_rev){
		case 0:
			ddr_init(sf1_dcd_table, ARRAY_SIZE(sf1_dcd_table));
			break;
		case 1:
			ddr_init(sf1_dcd_table, ARRAY_SIZE(sf1_dcd_table));
			break;
		case 2:
			ddr_init(avm_dcd_table, ARRAY_SIZE(avm_dcd_table));
			break;
		case 3:
			ddr_init(sf1_dcd_table, ARRAY_SIZE(sf1_dcd_table));
			break;
		case 4:
			ddr_init(sf1_dcd_table, ARRAY_SIZE(sf1_dcd_table));
			break;
		case 5:
			ddr_init(sf1_dcd_table, ARRAY_SIZE(sf1_dcd_table));
			break;
		case 6:
			ddr_init(sf1_dcd_table, ARRAY_SIZE(sf1_dcd_table));
			break;
		default:
			ddr_init(sf1_dcd_table, ARRAY_SIZE(sf1_dcd_table));
			break;

	}
}

void board_init_f(ulong dummy)
{
	printf("board_init_f\n");

	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();
	gpr_init();

	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	if (is_mx6dq() || is_mx6dqp())
	{
		setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6q_i2c_pad_info1);
		setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6q_i2c_pad_info3);
	}
	else
	{
		setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6dl_i2c_pad_info1);
		setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &mx6dl_i2c_pad_info3);
	}

	int lbrd;
	i2c_set_bus_num(2);
	lbrd = i2c_reg_read(0x53, 14);

	/* DDR initialization */
	spl_dram_init(lbrd);


	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();
	printf("SPL EEPROM %i\n", lbrd);

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}
#endif
