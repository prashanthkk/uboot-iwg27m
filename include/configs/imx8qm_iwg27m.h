/*
 * Copyright (C) 2020 iWave System Technologies Pvt Ltd.
 */

#ifndef __IMX8QM_IWG27M_H
#define __IMX8QM_IWG27M_H

#include <linux/sizes.h>
#include <asm/arch/imx-regs.h>
#include "imx_env.h"

#define BSP_VERSION		 	"iW-PRFHZ-SC-01-R3.0-REL1.0a-Linux5.4.3"

#ifdef CONFIG_SPL_BUILD
#define CONFIG_PARSE_CONTAINER
#define CONFIG_SPL_TEXT_BASE				0x100000
#define CONFIG_SPL_MAX_SIZE				(192 * 1024)
#define CONFIG_SYS_MONITOR_LEN				(1024 * 1024)
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_USE_SECTOR
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR                0x1040 /* (flash.bin_offset + 2Mb)/sector_size */
#define CONFIG_SYS_SPI_U_BOOT_OFFS 0x200000

/*
 * 0x08081000 - 0x08180FFF is for m4_0 xip image,
 * 0x08181000 - 0x008280FFF is for m4_1 xip image
  * So 3rd container image may start from 0x8281000
 */
#define CONFIG_SYS_UBOOT_BASE 0x08281000
#define CONFIG_SYS_MMCSD_FS_BOOT_PARTITION             0

#define CONFIG_SPL_LDSCRIPT		"arch/arm/cpu/armv8/u-boot-spl.lds"
/*
 * The memory layout on stack:  DATA section save + gd + early malloc
 * the idea is re-use the early malloc (CONFIG_SYS_MALLOC_F_LEN) with
 * CONFIG_SYS_SPL_MALLOC_START
 */
#define CONFIG_SPL_STACK		0x013fff0
#define CONFIG_SPL_BSS_START_ADDR      0x00130000
#define CONFIG_SPL_BSS_MAX_SIZE		0x1000	/* 4 KB */
#define CONFIG_SYS_SPL_MALLOC_START	0x82200000
#define CONFIG_SYS_SPL_MALLOC_SIZE     0x80000	/* 512 KB */
#define CONFIG_SERIAL_LPUART_BASE	0x5a0a0000
#define CONFIG_SYS_ICACHE_OFF
#define CONFIG_SYS_DCACHE_OFF
#define CONFIG_MALLOC_F_ADDR		0x00138000

#define CONFIG_SPL_RAW_IMAGE_ARM_TRUSTED_FIRMWARE

#define CONFIG_SPL_ABORT_ON_RAW_IMAGE /* For RAW image gives a error info not panic */

#define CONFIG_OF_EMBED
#endif

#define CONFIG_REMAKE_ELF

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_ARCH_MISC_INIT

#define CONFIG_CMD_READ

/* Flat Device Tree Definitions */
#define CONFIG_OF_BOARD_SETUP

#undef CONFIG_CMD_EXPORTENV
#undef CONFIG_CMD_IMPORTENV
#undef CONFIG_CMD_IMLS

#undef CONFIG_CMD_CRC32
#undef CONFIG_BOOTM_NETBSD

#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR       0
#define USDHC1_BASE_ADDR                0x5B010000
#define USDHC2_BASE_ADDR                0x5B020000
#define USDHC3_BASE_ADDR                0x5B030000
#define CONFIG_SUPPORT_EMMC_BOOT	/* eMMC specific */

#define CONFIG_ENV_OVERWRITE

#define CONFIG_PCIE_IMX
#define CONFIG_CMD_PCI
#define CONFIG_PCI_SCAN_SHOW
/*#define CONFIG_IMX_PCIEB*/

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

#define CONFIG_FEC_XCV_TYPE             RGMII
#define FEC_QUIRK_ENET_MAC

/* ENET0 connects AR8031 on CPU board, ENET1 connects to base board */
#define CONFIG_FEC_ENET_DEV 0

#if (CONFIG_FEC_ENET_DEV == 0)
#define IMX_FEC_BASE			0x5B040000
#define CONFIG_FEC_MXC_PHYADDR          0x4
#define CONFIG_ETHPRIME                 "eth0"
#define CONFIG_FEC_MXC_MDIO_BASE	0x5B040000
#elif (CONFIG_FEC_ENET_DEV == 1)
#define IMX_FEC_BASE			0x5B050000
#define CONFIG_FEC_MXC_PHYADDR          0x5
#define CONFIG_ETHPRIME                 "eth1"
#define CONFIG_FEC_MXC_MDIO_BASE	0x5B050000
#endif

#ifdef CONFIG_AHAB_BOOT
#define AHAB_ENV "sec_boot=yes\0"
#else
#define AHAB_ENV "sec_boot=no\0"
#endif

/* Boot M4 */
#define M4_BOOT_ENV \
	"m4_0_image=m4_0.bin\0" \
	"m4_1_image=m4_1.bin\0" \
	"loadm4image_0=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${m4_0_image}\0" \
	"loadm4image_1=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${m4_1_image}\0" \
	"m4boot_0=run loadm4image_0; dcache flush; bootaux ${loadaddr} 0\0" \
	"m4boot_1=run loadm4image_1; dcache flush; bootaux ${loadaddr} 1\0" \

#define CONFIG_MFG_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS_DEFAULT \
	"initrd_addr=0x83100000\0" \
	"initrd_high=0xffffffffffffffff\0" \
	"emmc_dev=0\0" \
	"sd_dev=1\0" \

/* Initial environment variables */
#define CONFIG_EXTRA_ENV_SETTINGS		\
	CONFIG_MFG_ENV_SETTINGS \
	M4_BOOT_ENV \
	AHAB_ENV \
	"image=Image\0" \
	"panel=NULL\0" \
	"ethaddr=00:01:02:03:04:05\0" \
	"eth1addr=1A:2B:3C:4D:5E:6F\0" \
	"initrd_addr=0x83100000\0"		\
	"initrd_high=0xffffffffffffffff\0" \
	"console=ttyLP4\0" \
	"earlycon=lpuart32,0x5a0a0000\0" \
	"fdt_addr=0x83000000\0"			\
	"fdt_high=0xffffffffffffffff\0"		\
	"cntr_addr=0x98000000\0"			\
	"cntr_file=os_cntr_signed.bin\0" \
	"boot_fdt=try\0" \
	"disp=lcd\0" \
	"fdt_file=imx8qm-iwg27m.dtb\0" \
	"mmcpart=" __stringify(CONFIG_SYS_MMC_IMG_LOAD_PART) "\0" \
	"mmcautodetect=no\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} earlycon root=${mmcroot}\0 " \
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"hdp_addr=0x9c000000\0" \
	"hdprx_addr=0x9c800000\0" \
	"hdp_file=hdmitxfw.bin\0" \
	"hdprx_file=hdmirxfw.bin\0" \
	"loadhdp=fatload mmc ${mmcdev}:${mmcpart} ${hdp_addr} ${hdp_file}\0" \
	"loadhdprx=fatload mmc ${mmcdev}:${mmcpart} ${hdprx_addr} ${hdprx_file}\0" \
	"boot_os=booti ${loadaddr} - ${fdt_addr};\0" \
	"loadcntr=fatload mmc ${mmcdev}:${mmcpart} ${cntr_addr} ${cntr_file}\0" \
	"auth_os=auth_cntr ${cntr_addr}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"if run loadhdp; then; hdp load ${hdp_addr}; fi;" \
		"if run loadhdprx; then; hdprx load ${hdprx_addr}; fi;" \
		"run mmcargs; " \
		"if test ${sec_boot} = yes; then " \
			"if run auth_os; then " \
				"run boot_os; " \
			"else " \
				"echo ERR: failed to authenticate; " \
			"fi; " \
		"else " \
			"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
				"if run loadfdt; then " \
					"run boot_os; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"else " \
				"echo wait for boot; " \
			"fi;" \
		"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} earlycon " \
		"root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
	"netboot=echo Booting from net ...; " \
		"run netargs;  " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"if ${get_cmd} ${hdp_addr} ${hdp_file}; then; hdp load ${hdp_addr}; fi;" \
		"if ${get_cmd} ${hdprx_addr} ${hdprx_file}; then; hdprx load ${hdprx_addr}; fi;" \
		"if test ${sec_boot} = yes; then " \
			"${get_cmd} ${cntr_addr} ${cntr_file}; " \
			"if run auth_os; then " \
				"run boot_os; " \
			"else " \
				"echo ERR: failed to authenticate; " \
			"fi; " \
		"else " \
			"${get_cmd} ${loadaddr} ${image}; " \
			"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
				"if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
					"run boot_os; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"else " \
				"booti; " \
			"fi;" \
		"fi;\0"

#define CONFIG_BOOTCOMMAND \
	   "mmc dev ${mmcdev}; if mmc rescan; then " \
	   "if test ${sec_boot} = yes; then " \
			"if run loadcntr; then " \
				"run mmcboot; " \
			"else run netboot; " \
			"fi; " \
		"else " \
			"if run loadimage; then " \
				"run mmcboot; " \
			"else run netboot; " \
			"fi; " \
		"fi; " \
	"else booti ${loadaddr} - ${fdt_addr}; fi"

#define CONFIG_EXTRA_ENV_SETTINGS_EMMC \
	CONFIG_EXTRA_ENV_SETTINGS \
	"mmcdev=0\0" \
        "mmcroot=/dev/mmcblk0p2 rootwait rw\0" \

#define CONFIG_EXTRA_ENV_SETTINGS_SSD \
	CONFIG_EXTRA_ENV_SETTINGS \
	"mmcdev=1\0" \
	"mmcroot=/dev/mmcblk1p2 rootwait rw\0" \

#define CONFIG_EXTRA_ENV_SETTINGS_MSD \
	CONFIG_EXTRA_ENV_SETTINGS \
	"mmcdev=2\0" \
	"mmcroot=/dev/mmcblk2p2 rootwait rw\0" \

/* Link Definitions */
#define CONFIG_LOADADDR			0x80280000

#define CONFIG_SYS_LOAD_ADDR           CONFIG_LOADADDR

#define CONFIG_SYS_INIT_SP_ADDR         0x80200000

/* Default environment is in EMMC */
#define CONFIG_ENV_SIZE			0x2000

#ifdef CONFIG_QSPI_BOOT
#define CONFIG_ENV_OFFSET       (4 * 1024 * 1024)
#define CONFIG_ENV_SECT_SIZE	(128 * 1024)
#define CONFIG_ENV_SPI_BUS	CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS	CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE	CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ	CONFIG_SF_DEFAULT_SPEED
#else
#define CONFIG_ENV_OFFSET       (64 * SZ_64K)
#define CONFIG_SYS_MMC_ENV_PART		0	/* user area */
#endif

#define CONFIG_SYS_MMC_IMG_LOAD_PART	1

#define CONFIG_SYS_MMC_ENV_DEV		0   /* USDHC1 */
#define CONFIG_SYS_FSL_USDHC_NUM	3

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		((CONFIG_ENV_SIZE + (32*1024)) * 1024)

#define CONFIG_SYS_SDRAM_BASE		0x80000000
#define CONFIG_NR_DRAM_BANKS		4
#define PHYS_SDRAM_1			0x80000000
#define PHYS_SDRAM_2			0x880000000
#define PHYS_SDRAM_1_SIZE		0x80000000	/* 2 GB */
#if defined(CONFIG_SDRAM_SIZE_2GB)
#define PHYS_SDRAM_2_SIZE		0x00000000	/* 0 GB */
#elif defined(CONFIG_SDRAM_SIZE_4GB)
#define PHYS_SDRAM_2_SIZE		0x80000000	/* 2 GB */
#elif defined(CONFIG_SDRAM_SIZE_8GB)
#define PHYS_SDRAM_2_SIZE		0x180000000	/* 6 GB */
#endif

#define CONFIG_SYS_MEMTEST_START    0xA0000000
#define CONFIG_SYS_MEMTEST_END      (CONFIG_SYS_MEMTEST_START + (PHYS_SDRAM_1_SIZE >> 2))

/* Serial */
#ifdef CONFIG_SYS_PROMPT
#undef CONFIG_SYS_PROMPT
#endif
#define CONFIG_SYS_PROMPT              "iWave-G27 > "

#define CONFIG_BAUDRATE			115200

/* Monitor Command Prompt */
#define CONFIG_SYS_PROMPT_HUSH_PS2     "> "
#define CONFIG_SYS_CBSIZE              2048
#define CONFIG_SYS_MAXARGS             64
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
					sizeof(CONFIG_SYS_PROMPT) + 16)

/* Generic Timer Definitions */
#define COUNTER_FREQUENCY		8000000	/* 8MHz */

#define CONFIG_IMX_SMMU

/* MT35XU512ABA1G12 has only one Die, so QSPI0 B won't work */
#ifdef CONFIG_FSL_FSPI
#define FSL_FSPI_FLASH_SIZE		SZ_64M
#define FSL_FSPI_FLASH_NUM		1
#define FSPI0_BASE_ADDR			0x5d120000
#define FSPI0_AMBA_BASE			0
#define CONFIG_SYS_FSL_FSPI_AHB
#endif

#define CONFIG_SERIAL_TAG

/* USB Config */
#ifndef CONFIG_SPL_BUILD
#define CONFIG_CMD_USB
#define CONFIG_USB_STORAGE
#define CONFIG_USBD_HS

#define CONFIG_CMD_USB_MASS_STORAGE
#define CONFIG_USB_GADGET_MASS_STORAGE
#define CONFIG_USB_FUNCTION_MASS_STORAGE

#endif

#define CONFIG_USB_MAX_CONTROLLER_COUNT 2

/* USB OTG controller configs */
#ifdef CONFIG_USB_EHCI_HCD
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#endif

/* Framebuffer */
#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_IMXDPUV1
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IMX_VIDEO_SKIP
#endif

#define CONFIG_OF_SYSTEM_SETUP

#if defined(CONFIG_TARGET_IMX8QM_IWG27M_UUU)
#include "imx8qm_iwg27m_uuu.h"
#endif

#endif /* __IMX8QM_IWG27M_H */
