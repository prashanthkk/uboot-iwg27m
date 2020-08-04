/*
 * Copyright 2020 iWave System Technologies Pvt Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <malloc.h>
#include <errno.h>
#include <netdev.h>
#include <fsl_ifc.h>
#include <fdt_support.h>
#include <linux/libfdt.h>
#include <environment.h>
#include <fsl_esdhc.h>
#include <i2c.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch/sci/sci.h>
#include <asm/arch/imx8-pins.h>
#include <dm.h>
#include <imx8_hsio.h>
#include <usb.h>
#include <asm/arch/iomux.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/video.h>
#include <asm/arch/video_common.h>
#include <power-domain.h>
#include "../common/tcpc.h"
#include <cdns3-uboot.h>
#include <asm/arch/lpcg.h>
#include <bootm.h>

DECLARE_GLOBAL_DATA_PTR;

int bom_rev, pcb_rev;

#define ENET_INPUT_PAD_CTRL	((SC_PAD_CONFIG_OD_IN << PADRING_CONFIG_SHIFT) | (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) \
						| (SC_PAD_28FDSOI_DSE_18V_10MA << PADRING_DSE_SHIFT) | (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

#define ENET_NORMAL_PAD_CTRL	((SC_PAD_CONFIG_NORMAL << PADRING_CONFIG_SHIFT) | (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) \
						| (SC_PAD_28FDSOI_DSE_18V_10MA << PADRING_DSE_SHIFT) | (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))


#define GPIO_PAD_CTRL	((SC_PAD_CONFIG_NORMAL << PADRING_CONFIG_SHIFT) | (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) \
						| (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT) | (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

#define GPIO_PAD_CFG_CTRL	((SC_PAD_CONFIG_NORMAL << PADRING_CONFIG_SHIFT) | (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) \
						| (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT) | (SC_PAD_28FDSOI_PS_NONE << PADRING_PULL_SHIFT))

#define UART_PAD_CTRL	((SC_PAD_CONFIG_OUT_IN << PADRING_CONFIG_SHIFT) | (SC_PAD_ISO_OFF << PADRING_LPCONFIG_SHIFT) \
						| (SC_PAD_28FDSOI_DSE_DV_HIGH << PADRING_DSE_SHIFT) | (SC_PAD_28FDSOI_PS_PU << PADRING_PULL_SHIFT))

static iomux_cfg_t uart4_pads[] = {
	SC_P_M40_GPIO0_00 | MUX_PAD_CTRL(UART_PAD_CTRL),
        SC_P_M40_GPIO0_01 | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx8_iomux_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
}

#define LCD_RST IMX_GPIO_NR(0, 3)
static iomux_cfg_t lcd_rst_pads[] = {
	SC_P_SIM0_PD | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

void lcd_reset(void)
{
	imx8_iomux_setup_multiple_pads(lcd_rst_pads, ARRAY_SIZE(lcd_rst_pads));
	gpio_request(LCD_RST, "LCD-GPIO");
	gpio_direction_output(LCD_RST,1);
	mdelay(20);
	gpio_direction_output(LCD_RST,0);
}

#define PDN_EN IMX_GPIO_NR(4, 7)
#define CORE_PDN IMX_GPIO_NR(2, 30)

static iomux_cfg_t wifi_pads[] = {
	SC_P_USDHC1_RESET_B | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),
        SC_P_ESAI0_TX4_RX1  | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static void wifi_pwr_seq(void)
{
        imx8_iomux_setup_multiple_pads(wifi_pads, ARRAY_SIZE(wifi_pads));
        gpio_request(PDN_EN, "pdn_en_gpio");
        gpio_request(CORE_PDN, "core_pdn_gpio");
	gpio_direction_output(PDN_EN,1);
	mdelay(5);
	gpio_direction_output(CORE_PDN,1);
}

int board_early_init_f(void)
{
	int ret;

	/* When start u-boot in XEN VM, directly return */
	if (IS_ENABLED(CONFIG_XEN)) {
		writel(0xF53535F5, (void __iomem *)0x80000000);
		return 0;
	}

	/* Set UART4 clock root to 80 MHz */
	sc_pm_clock_rate_t rate = 80000000;

	/* Power up UART4 */
	ret = sc_pm_set_resource_power_mode(-1, SC_R_UART_4, SC_PM_PW_MODE_ON);
	if (ret)
		return ret;

	ret = sc_pm_set_clock_rate(-1, SC_R_UART_4, 2, &rate);
	if (ret)
		return ret;

	/* Enable UART4 clock root */
	ret = sc_pm_clock_enable(-1, SC_R_UART_4, 2, true, false);
	if (ret)
		return ret;

	lpcg_all_clock_on(LPUART_4_LPCG);

	setup_iomux_uart();

/* Dual bootloader feature will require CAAM access, but JR0 and JR1 will be
 * assigned to seco for imx8, use JR3 instead.
 */
#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_DUAL_BOOTLOADER)
	sc_pm_set_resource_power_mode(-1, SC_R_CAAM_JR3, SC_PM_PW_MODE_ON);
	sc_pm_set_resource_power_mode(-1, SC_R_CAAM_JR3_OUT, SC_PM_PW_MODE_ON);
#endif

	return 0;
}

#if IS_ENABLED(CONFIG_FEC_MXC)
#include <miiphy.h>

#ifndef CONFIG_DM_ETH
static iomux_cfg_t pad_enet1[] = {
	SC_P_ENET1_RGMII_RX_CTL | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET1_RGMII_RXD0 | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET1_RGMII_RXD1 | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET1_RGMII_RXD2 | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET1_RGMII_RXD3 | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET1_RGMII_RXC | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET1_RGMII_TX_CTL | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET1_RGMII_TXD0 | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET1_RGMII_TXD1 | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET1_RGMII_TXD2 | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET1_RGMII_TXD3 | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET1_RGMII_TXC | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET1_REFCLK_125M_25M | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),

	SC_P_ENET1_MDC | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET1_MDIO | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_MIPI_DSI1_I2C0_SCL | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),
        SC_P_MIPI_DSI0_I2C0_SCL | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),
        SC_P_MIPI_DSI0_I2C0_SDA | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static iomux_cfg_t pad_enet0[] = {
	SC_P_ENET0_RGMII_RX_CTL | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET0_RGMII_RXD0 | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET0_RGMII_RXD1 | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET0_RGMII_RXD2 | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET0_RGMII_RXD3 | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET0_RGMII_RXC | MUX_PAD_CTRL(ENET_INPUT_PAD_CTRL),
	SC_P_ENET0_RGMII_TX_CTL | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET0_RGMII_TXD0 | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET0_RGMII_TXD1 | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET0_RGMII_TXD2 | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET0_RGMII_TXD3 | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET0_RGMII_TXC | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),

	SC_P_ENET0_MDC | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET0_MDIO | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
	SC_P_ENET0_REFCLK_125M_25M | MUX_PAD_CTRL(ENET_NORMAL_PAD_CTRL),
        SC_P_PCIE_CTRL1_WAKE_B | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),
        SC_P_PCIE_CTRL1_CLKREQ_B | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),
        SC_P_PCIE_CTRL0_CLKREQ_B | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static void setup_iomux_fec(void)
{
	if (0 == CONFIG_FEC_ENET_DEV)
		imx8_iomux_setup_multiple_pads(pad_enet0, ARRAY_SIZE(pad_enet0));
	else
		imx8_iomux_setup_multiple_pads(pad_enet1, ARRAY_SIZE(pad_enet1));
}

int board_eth_init(bd_t *bis)
{
	int ret;
	struct power_domain pd;

	printf("[%s] %d\n", __func__, __LINE__);

	if (CONFIG_FEC_ENET_DEV) {
		if (!power_domain_lookup_name("conn_enet1", &pd))
			power_domain_on(&pd);
	} else {
		if (!power_domain_lookup_name("conn_enet0", &pd))
			power_domain_on(&pd);
	}

	setup_iomux_fec();

	ret = fecmxc_initialize_multi(bis, CONFIG_FEC_ENET_DEV,
		CONFIG_FEC_MXC_PHYADDR, IMX_FEC_BASE);
	if (ret)
		printf("FEC1 MXC: %s:failed\n", __func__);

	return ret;
}
#endif

int board_phy_config(struct phy_device *phydev)
{
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x00);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x82ee);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}
#endif

#define OTG_PWR IMX_GPIO_NR(4, 3)
static iomux_cfg_t otg_pwr_pads[] = {
	SC_P_USB_SS3_TC0 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static void usb_otg_pwr_enable(void)
{
	imx8_iomux_setup_multiple_pads(otg_pwr_pads, ARRAY_SIZE(otg_pwr_pads));
	gpio_request(OTG_PWR, "OTG-GPIO");
	gpio_direction_output(OTG_PWR,1);
}

#define HUB_RST IMX_GPIO_NR(1, 21)

static iomux_cfg_t hub_pwr_pads[] = {
	SC_P_MIPI_DSI1_I2C0_SDA | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static void usb_hub_pwr_enable(void)
{
	imx8_iomux_setup_multiple_pads(hub_pwr_pads, ARRAY_SIZE(hub_pwr_pads));
	gpio_request(HUB_RST, "usb_hub_reset_gpio");
	gpio_direction_output(HUB_RST,1);
}

#define BCONFIG_0 IMX_GPIO_NR(1, 5)
#define BCONFIG_1 IMX_GPIO_NR(1, 13)
#define BCONFIG_2 IMX_GPIO_NR(1, 12)
#define BCONFIG_3 IMX_GPIO_NR(1, 11)
#define BCONFIG_4 IMX_GPIO_NR(0, 6)
#define BCONFIG_5 IMX_GPIO_NR(0, 7)
#define BCONFIG_6 IMX_GPIO_NR(0, 11)

int board_config_pads[] = {
	BCONFIG_0,
	BCONFIG_1,
	BCONFIG_2,
	BCONFIG_3,
	BCONFIG_4,
	BCONFIG_5,
	BCONFIG_6,
};

static iomux_cfg_t board_cfg[] = {
	SC_P_LVDS0_GPIO01 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CFG_CTRL),
	SC_P_LVDS1_I2C0_SDA | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CFG_CTRL),
	SC_P_LVDS1_I2C0_SCL | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CFG_CTRL),
	SC_P_LVDS1_GPIO01 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CFG_CTRL),
	SC_P_M40_I2C0_SCL | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CFG_CTRL),
	SC_P_M40_I2C0_SDA | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CFG_CTRL),
	SC_P_M41_I2C0_SDA | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CFG_CTRL),
};

void get_board_info(void)
{
	int i;

	imx8_iomux_setup_multiple_pads(board_cfg, ARRAY_SIZE(board_cfg));

	for (i=0;i<ARRAY_SIZE(board_config_pads);i++) {
		if(i<=3) {
			gpio_request(board_config_pads[i], "SOM-Revision-GPIO");
			gpio_direction_input(board_config_pads[i]);
			pcb_rev |= (gpio_get_value(board_config_pads[i]) << i);
		} else {
			gpio_request(board_config_pads[i], "SOM-Revision-GPIO");
			gpio_direction_input(board_config_pads[i]);
			bom_rev |= (gpio_get_value(board_config_pads[i]) << (i-4));
		}
	}
}

static void print_board_info(void)
{
	printf ("\n");
	printf ("Board Info:\n");
	printf ("\tBSP Version     : %s\n", BSP_VERSION);
	printf ("\tSOM Version     : iW-PRFHZ-AP-01-R%x.%x\n",bom_rev+1,pcb_rev);
	printf ("\n");
}

int checkboard(void)
{
	puts("Board: iMX8QM IWG27M\n");

	print_bootinfo();

	return 0;
}

#ifdef CONFIG_FSL_HSIO

#define PCIE_PAD_CTRL	((SC_PAD_CONFIG_OD_IN << PADRING_CONFIG_SHIFT))
static iomux_cfg_t board_pcie_pins[] = {
	SC_P_PCIE_CTRL0_WAKE_B | MUX_MODE_ALT(3) | MUX_PAD_CTRL(PCIE_PAD_CTRL),
	SC_P_PCIE_CTRL0_PERST_B | MUX_MODE_ALT(3) | MUX_PAD_CTRL(PCIE_PAD_CTRL),
};

static void imx8qm_hsio_initialize(void)
{
	struct power_domain pd;
	int ret;

	if (!power_domain_lookup_name("hsio_sata0", &pd)) {
		ret = power_domain_on(&pd);
		if (ret)
			printf("hsio_sata0 Power up failed! (error = %d)\n", ret);
	}

	if (!power_domain_lookup_name("hsio_pcie0", &pd)) {
		ret = power_domain_on(&pd);
		if (ret)
			printf("hsio_pcie0 Power up failed! (error = %d)\n", ret);
	}

	if (!power_domain_lookup_name("hsio_pcie1", &pd)) {
		ret = power_domain_on(&pd);
		if (ret)
			printf("hsio_pcie1 Power up failed! (error = %d)\n", ret);
	}

	if (!power_domain_lookup_name("hsio_gpio", &pd)) {
		ret = power_domain_on(&pd);
		if (ret)
			 printf("hsio_gpio Power up failed! (error = %d)\n", ret);
	}

	lpcg_all_clock_on(HSIO_PCIE_X2_LPCG);
	lpcg_all_clock_on(HSIO_PCIE_X1_LPCG);
	lpcg_all_clock_on(HSIO_PHY_X2_LPCG);
	lpcg_all_clock_on(HSIO_PHY_X1_LPCG);
	lpcg_all_clock_on(HSIO_PCIE_X2_CRR2_LPCG);
	lpcg_all_clock_on(HSIO_PCIE_X1_CRR3_LPCG);
	lpcg_all_clock_on(HSIO_MISC_LPCG);
	lpcg_all_clock_on(HSIO_GPIO_LPCG);

	imx8_iomux_setup_multiple_pads(board_pcie_pins, ARRAY_SIZE(board_pcie_pins));
}

void pci_init_board(void)
{
	/* test the 1 lane mode of the PCIe A controller */
	mx8qm_pcie_init();
}
#endif

#ifdef CONFIG_USB

#ifdef CONFIG_USB_TCPC
#define USB_TYPEC_SEL IMX_GPIO_NR(4, 6)
#define USB_TYPEC_EN IMX_GPIO_NR(4, 19)

static iomux_cfg_t ss_mux_gpio[] = {
	SC_P_USB_SS3_TC3 | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	SC_P_QSPI1A_SS0_B | MUX_MODE_ALT(3) | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

struct tcpc_port port;
struct tcpc_port_config port_config = {
	.i2c_bus = 1,
	.addr = 0x22,
	.port_type = TYPEC_PORT_DFP,
};

void ss_mux_select(enum typec_cc_polarity pol)
{
	if (pol == TYPEC_POLARITY_CC1)
		gpio_direction_output(USB_TYPEC_SEL, 0);
	else
		gpio_direction_output(USB_TYPEC_SEL, 1);
}

static void setup_typec(void)
{
	imx8_iomux_setup_multiple_pads(ss_mux_gpio, ARRAY_SIZE(ss_mux_gpio));
	gpio_request(USB_TYPEC_SEL, "typec_sel");
	gpio_request(USB_TYPEC_EN, "typec_en");

	gpio_direction_output(USB_TYPEC_EN, 1);

	tcpc_init(&port, port_config, &ss_mux_select);
}
#endif

int board_usb_init(int index, enum usb_init_type init)
{
	int ret = 0;

	if (index == 1) {
		if (init == USB_INIT_HOST) {
#ifdef CONFIG_USB_TCPC
			ret = tcpc_setup_dfp_mode(&port);
#endif
#ifdef CONFIG_USB_CDNS3_GADGET
		} else {
#ifdef CONFIG_USB_TCPC
			ret = tcpc_setup_ufp_mode(&port);
			printf("%d setufp mode %d\n", index, ret);
#endif
#endif
		}
	}

	return ret;

}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	int ret = 0;

	if (index == 1) {
		if (init == USB_INIT_HOST) {
#ifdef CONFIG_USB_TCPC
			ret = tcpc_disable_src_vbus(&port);
#endif
		}
	}

	return ret;
}
#endif

void iwg27m_fdt_update(void *fdt)
{
	/* IWG27M: Select LCD/HDMI based on boot argument */
	if (!strcmp("hdmi", env_get("disp"))) {
		fdt_setprop_string(fdt, fdt_path_offset(fdt, "/bus@56220000/dphy@56228300"), "status", "disabled");
		fdt_setprop_string(fdt, fdt_path_offset(fdt, "/bus@56220000/dsi_host@56228000"), "status", "disabled");

		fdt_setprop_string(fdt, fdt_path_offset(fdt, "/bus@56260000/hdmi@56268000"), "status", "okay");
		fdt_setprop_string(fdt, fdt_path_offset(fdt, "/bus@56260000/irqsteer@56260000"), "status", "okay");
		fdt_setprop_string(fdt, fdt_path_offset(fdt, "/sound-hdmi-tx"), "status", "okay");
	}
	else if (!strcmp("lcd", env_get("disp"))) {
		fdt_setprop_string(fdt, fdt_path_offset(fdt, "/bus@56220000/dphy@56228300"), "status", "okay");
		fdt_setprop_string(fdt, fdt_path_offset(fdt, "/bus@56220000/dsi_host@56228000"), "status", "okay");

		fdt_setprop_string(fdt, fdt_path_offset(fdt, "/bus@56260000/hdmi@56268000"), "status", "disabled");
		fdt_setprop_string(fdt, fdt_path_offset(fdt, "/bus@56260000/irqsteer@56260000"), "status", "disabled");
		fdt_setprop_string(fdt, fdt_path_offset(fdt, "/sound-hdmi-tx"), "status", "disabled");
	}

	if (bom_rev == 3) {
		/* IWG27M: Enable Voltage switching only for C1 PMIC supported R4.x board */
		fdt_delprop(fdt, fdt_path_offset(fdt, "/bus@5b000000/mmc@5b020000"), "no-1-8-v");
		/* IWg27M: Enable EDID functionality only for R4.x Board */	
		fdt_delprop(fdt, fdt_path_offset(fdt, "/bus@56260000/hdmi@56268000"), "fsl,no_edid");
	}
}

int board_init(void)
{

#ifdef CONFIG_FSL_HSIO
	imx8qm_hsio_initialize();
#endif

#if defined(CONFIG_USB) && defined(CONFIG_USB_TCPC)
	setup_typec();
#endif

	return 0;
}

void board_quiesce_devices(void)
{
	const char *power_on_devices[] = {
		"dma0_chan21",
	};

	if (IS_ENABLED(CONFIG_XEN)) {
		/* Clear magic number to let xen know uboot is over */
		writel(0x0, (void __iomem *)0x80000000);
		return;
	}

	power_off_pd_devices(power_on_devices, ARRAY_SIZE(power_on_devices));
}

void detail_board_ddr_info(void)
{
	puts("\nDDR    ");
}

/*
 * Board specific reset that is system reset.
 */
void reset_cpu(ulong addr)
{
	/* TODO */
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, bd_t *bd)
{
	return 0;
}
#endif

extern uint32_t _end_ofs;
int board_late_init(void)
{
	print_board_info();
	/* IWG27M: WIFI: Correcting WIFI Power On Sequence */
	wifi_pwr_seq();
	usb_otg_pwr_enable();
	usb_hub_pwr_enable();

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "iW-RainboW-G27M-i.MX8QM/QP SMARC");
	env_set("board_rev", "iW-PRFHZ-AP-01-R1.X");
#endif

	env_set("sec_boot", "no");
#ifdef CONFIG_AHAB_BOOT
	env_set("sec_boot", "yes");
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	/* IWG27M: Updating iMX8QM/QP FDT file based on CPU */
	if(is_imx8qp())
		env_set("fdt_file","imx8qp-iwg27m.dtb");
	else
		env_set("fdt_file","imx8qm-iwg27m.dtb");

	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
int is_recovery_key_pressing(void)
{
	return 0; /*TODO*/
}
#endif /*CONFIG_FSL_FASTBOOT*/

#if defined(CONFIG_VIDEO_IMXDPUV1)
struct display_info_t const displays[] = {
};
size_t display_count = ARRAY_SIZE(displays);

#endif /* CONFIG_VIDEO_IMXDPUV1 */
