/*
 * Copyright 2009 Sascha Hauer, <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */

/*
 * This machine is known as:
 *  - i.MX25 3-Stack Development System
 *  - i.MX25 Platform Development Kit (i.MX25 PDK)
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/input/matrix_keypad.h>
#include <linux/usb/otg.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/memory.h>
#include <asm/mach/map.h>
#include <mach/common.h>
#include <mach/mx25.h>
#include <mach/iomux-mx25.h>
#include <linux/leds_pwm.h>
#include <linux/dm9000.h>
#include <mach/iomux-v3.h>

#include "devices-imx25.h"

static const struct imxuart_platform_data uart_pdata __initconst = {
	.flags = IMXUART_HAVE_RTSCTS,
};

static const struct imxuart_platform_data uart_nortscts_pdata __initconst = {
	.flags = 0,
};

static iomux_v3_cfg_t mx25dh_pads[] = {
	MX25_PAD_FEC_MDC__FEC_MDC,
	MX25_PAD_FEC_MDIO__FEC_MDIO,
	MX25_PAD_FEC_TDATA0__FEC_TDATA0,
	MX25_PAD_FEC_TDATA1__FEC_TDATA1,
	MX25_PAD_FEC_TX_EN__FEC_TX_EN,
	MX25_PAD_FEC_RDATA0__FEC_RDATA0,
	MX25_PAD_FEC_RDATA1__FEC_RDATA1,
	MX25_PAD_FEC_RX_DV__FEC_RX_DV,
	MX25_PAD_FEC_TX_CLK__FEC_TX_CLK,

	/* LCD */
	MX25_PAD_LD0__LD0,
	MX25_PAD_LD1__LD1,
	MX25_PAD_LD2__LD2,
	MX25_PAD_LD3__LD3,
	MX25_PAD_LD4__LD4,
	MX25_PAD_LD5__LD5,
	MX25_PAD_LD6__LD6,
	MX25_PAD_LD7__LD7,
	MX25_PAD_LD8__LD8,
	MX25_PAD_LD9__LD9,
	MX25_PAD_LD10__LD10,
	MX25_PAD_LD11__LD11,
	MX25_PAD_LD12__LD12,
	MX25_PAD_LD13__LD13,
	MX25_PAD_LD14__LD14,
	MX25_PAD_LD15__LD15,
	//MX25_PAD_GPIO_E__LD16,
	//MX25_PAD_GPIO_F__LD17,
	MX25_PAD_HSYNC__HSYNC,
	MX25_PAD_VSYNC__VSYNC,
	MX25_PAD_LSCLK__LSCLK,
	MX25_PAD_OE_ACD__OE_ACD,
	MX25_PAD_CONTRAST__CONTRAST,

	/* SD1 */
	MX25_PAD_SD1_CMD__SD1_CMD,
	MX25_PAD_SD1_CLK__SD1_CLK,
	MX25_PAD_SD1_DATA0__SD1_DATA0,
	MX25_PAD_SD1_DATA1__SD1_DATA1,
	MX25_PAD_SD1_DATA2__SD1_DATA2,
	MX25_PAD_SD1_DATA3__SD1_DATA3,
	MX25_PAD_EB1__GPIO_2_13,  /* carddetect microSD */
	MX25_PAD_CLKO__GPIO_2_21, /* carddetect SD */
	MX25_PAD_UPLL_BYPCLK__GPIO_3_16, /* SD_CARD_SEL */

	/* USB Power */
	MX25_PAD_EXT_ARMCLK__GPIO_3_15,
//	MX25_PAD_CS5__GPIO_3_21,  /* look at GPIOs section */

	/* CAN */
	MX25_PAD_GPIO_A__CAN1_TX,
	MX25_PAD_GPIO_B__CAN1_RX,

	/* I2C */
	MX25_PAD_I2C1_CLK__I2C1_CLK,
	MX25_PAD_I2C1_DAT__I2C1_DAT,
	MX25_PAD_GPIO_C__I2C2_CLK,
	MX25_PAD_GPIO_D__I2C2_DAT,

	/* SPI 1 */
	MX25_PAD_CSPI1_MISO__CSPI1_MISO,
	MX25_PAD_CSPI1_MOSI__CSPI1_MOSI,
	MX25_PAD_CSPI1_RDY__CSPI1_RDY,
	MX25_PAD_CSPI1_SCLK__CSPI1_SCLK,
	MX25_PAD_CSPI1_SS0__GPIO_1_16,
	MX25_PAD_CSPI1_SS1__GPIO_1_17,

	/* SPI 3 */
	MX25_PAD_CSI_D2__CSPI3_MOSI,
	MX25_PAD_CSI_D3__CSPI3_MISO,
	MX25_PAD_CSI_D4__CSPI3_SCLK,
	MX25_PAD_CSI_D6__GPIO_1_31,

	/* Second Ethernet */
	MX25_PAD_OE__OE,
	MX25_PAD_CS0__CS0,

	/* GPIOs */
	MX25_PAD_EB0__GPIO_2_12,	/* GPIO A */
	MX25_PAD_CS5__GPIO_3_21,	/* GPIO B */
	MX25_PAD_A18__GPIO_2_4,		/* GPIO C */
	MX25_PAD_A19__GPIO_2_5,		/* GPIO D */
	MX25_PAD_A20__GPIO_2_6,		/* GPIO E */
	MX25_PAD_A21__GPIO_2_7,		/* GPIO F */
	MX25_PAD_A22__GPIO_2_8,		/* GPIO G */
	MX25_PAD_A23__GPIO_2_9,		/* GPIO H */
	MX25_PAD_VSTBY_REQ__GPIO_3_17,	/* GPIO I */

	/* Additional GPIOs - Camera Interface */
	MX25_PAD_CSI_HSYNC__GPIO_1_10,	/* GPIO J | CAM_HSYNC */
	MX25_PAD_CSI_PIXCLK__GPIO_1_11,	/* GPIO K | CAM_PIXCLK */
	MX25_PAD_CSI_MCLK__GPIO_1_8,	/* GPIO L | CAM_MCLK */
	MX25_PAD_CSI_VSYNC__GPIO_1_9,	/* GPIO M | CAM_VSYNC */
	MX25_PAD_CSI_D9__GPIO_4_21,	/* GPIO N | CAM_D9 */
	MX25_PAD_CSI_D8__GPIO_1_7,	/* GPIO O | CAM_D8 */
	MX25_PAD_CSI_D7__GPIO_1_6,	/* GPIO P | CAM_D7 */
	//MX25_PAD_CSI_D6__GPIO_1_31, 	/* GPIO Q | CAM_D6 | SPI3_SS0 */
	MX25_PAD_CSI_D5__GPIO_1_30,	/* GPIO R | CAM_D5 */
	//MX25_PAD_CSI_D4__GPIO_1_29,	/* GPIO S | CAM_D4 | SPI3_SCLK */
	//MX25_PAD_CSI_D3__GPIO_1_28,	/* GPIO T | CAM_D3 | SPI3_MISO */
	//MX25_PAD_CSI_D2__GPIO_1_27,	/* GPIO U | CAM_D2 | SPI3_MOSI */
	// Not connected		/* GPIO V | CAM_D1 */
	// Not connected		/* GPIO W | CAM_D0 */

	/* UART STD */
	MX25_PAD_ECB__UART5_TXD_MUX,
	MX25_PAD_LBA__UART5_RXD_MUX,

	/* UART BT */
	MX25_PAD_KPP_ROW0__UART3_RXD_MUX,
	MX25_PAD_KPP_ROW1__UART3_TXD_MUX,
	//MX25_PAD_LD0__GPIO_2_15,	/* TODO in software: CTS as GPIO */
	//MX25_PAD_A24__GPIO_2_10,	/* TODO in software: RTS as GPIO */

	/* int highest pri */
	MX25_PAD_RTCK__GPIO_3_14,
};

static const struct fec_platform_data mx25_fec_pdata __initconst = {
	.phy    = PHY_INTERFACE_MODE_RMII,
};

/*
 * NAND Partitions
 */
static struct mtd_partition __initdata dh_nand_partition[] = {
        {
                .name   = "u-boot",
                .offset = 0,
                .size   = 0x000C0000,
        },
        {
                .name   = "u-boot-env1",
                .offset = MTDPART_OFS_NXTBLK,
                .size   = 0x00060000,
        },
        {
                .name   = "u-boot-env2",
                .offset = MTDPART_OFS_NXTBLK,
                .size   = 0x00060000,
        },
        {
                .name   = "splash",
                .offset = MTDPART_OFS_NXTBLK,
                .size   = 0x000C0000,
        },
        {
                .name   = "dh-settings",
                .offset = MTDPART_OFS_NXTBLK,
                .size   = 0x00040000,
        },
        {
                .name   = "kernel",
                .offset = MTDPART_OFS_NXTBLK,
                .size   = 0x00380000,
        },
        {
                .name   = "rootfs",
                .offset = MTDPART_OFS_NXTBLK,
                .size   = MTDPART_SIZ_FULL,
        },
};

static const struct mxc_nand_platform_data
mx25dh_nand_board_info __initconst = {
	.width		= 1,
	.hw_ecc		= 1,
	.flash_bbt	= 1,

	.parts		= dh_nand_partition,
	.nr_parts	= ARRAY_SIZE(dh_nand_partition),
};



static struct imx_fb_videomode mx25dh_modes[] = {
	{
		.mode	= {
			.name		= "u-boot",
			.refresh	= 60,
			.xres		= 800,
			.yres		= 480,
			.pixclock	= KHZ2PICOS(22000),
			.left_margin	= 86,
			.right_margin	= 42,
			.upper_margin	= 33,
			.lower_margin	= 10,
			.hsync_len	= 64,
			.vsync_len	= 2,
		},
		.bpp	= 16,
		.pcr	= 0xfac00082,
	},
};

static struct imx_fb_platform_data mx25dh_fb_pdata __initdata = {
	.mode		= mx25dh_modes,
	.num_modes	= ARRAY_SIZE(mx25dh_modes),
	.pwmr		= 0x00A903FF,
	.lscr1		= 0x00000000,
	.dmacr		= 0x00020010,
};

static const uint32_t mx25dh_keymap[] = {
	KEY(0, 0, KEY_UP),
	KEY(0, 1, KEY_DOWN),
	KEY(0, 2, KEY_VOLUMEDOWN),
	KEY(0, 3, KEY_HOME),
	KEY(1, 0, KEY_RIGHT),
	KEY(1, 1, KEY_LEFT),
	KEY(1, 2, KEY_ENTER),
	KEY(1, 3, KEY_VOLUMEUP),
	KEY(2, 0, KEY_F6),
	KEY(2, 1, KEY_F8),
	KEY(2, 2, KEY_F9),
	KEY(2, 3, KEY_F10),
	KEY(3, 0, KEY_F1),
	KEY(3, 1, KEY_F2),
	KEY(3, 2, KEY_F3),
	KEY(3, 3, KEY_POWER),
};

static const struct matrix_keymap_data mx25dh_keymap_data __initconst = {
	.keymap		= mx25dh_keymap,
	.keymap_size	= ARRAY_SIZE(mx25dh_keymap),
};


static int dhevm_cpuimx25_usbh2_init(struct platform_device *pdev)
{
	return mx25_initialize_usb_hw(pdev->id, MXC_EHCI_INTERFACE_SINGLE_UNI |
			MXC_EHCI_INTERNAL_PHY | MXC_EHCI_IPPUE_DOWN);
}

static const struct mxc_usbh_platform_data usbh2_pdata __initconst = {
	.init	= dhevm_cpuimx25_usbh2_init,
	.portsc	= MXC_EHCI_MODE_SERIAL,
};

static const struct fsl_usb2_platform_data otg_device_pdata __initconst = {
	.operating_mode = FSL_USB2_DR_DEVICE,
	.phy_mode       = FSL_USB2_PHY_UTMI,
};

static int dhevm_cpuimx25_otg_init(struct platform_device *pdev)
{
	return mx25_initialize_usb_hw(pdev->id, MXC_EHCI_INTERFACE_DIFF_UNI);
}

static const struct mxc_usbh_platform_data otg_pdata __initconst = {
	.init	= dhevm_cpuimx25_otg_init,
	.portsc	= MXC_EHCI_MODE_UTMI,
};

/*
 * handle arbitration loss on i2c1
 */

static iomux_v3_cfg_t mx25_pads_i2c1_func[] = {
	MX25_PAD_I2C1_CLK__I2C1_CLK,
	MX25_PAD_I2C1_DAT__I2C1_DAT,
};

static iomux_v3_cfg_t mx25_pads_i2c1_gpios[] = {
	MX25_PAD_I2C1_CLK__GPIO_1_12, /* GPIO_PORTA | 12 */
	MX25_PAD_I2C1_DAT__GPIO_1_13, /* GPIO_PORTA | 13 */
};

static int i2c1_arbloss(void)
{
        int index;
        printk("handle arbitration loss on i2c1!\n");
        
        /* configure i2c pads to gpios */
        mxc_iomux_v3_setup_multiple_pads(mx25_pads_i2c1_gpios,
			ARRAY_SIZE(mx25_pads_i2c1_gpios));
        
        /* aquire and setup gpios */
        gpio_request(GPIO_PORTA | 12, "i2c1_imx" );
	gpio_direction_output(GPIO_PORTA | 12, 1);
		
	gpio_request(GPIO_PORTA | 13, "i2c1_imx" );
	gpio_direction_input(GPIO_PORTA | 13 );
	
        /* toggle scl until sda line goes to high */
        for (index = 0; index < 8; index++) {
                if (gpio_get_value(GPIO_PORTA | 13)) {
                        printk("i2c1 sda is free!\n");
                        break;
                }
                
                /* toggle scl pin */
                udelay(10);
                gpio_set_value(GPIO_PORTA | 12, 0);
                udelay(10);
                gpio_set_value(GPIO_PORTA | 12, 1);
        }
                
        if (!gpio_get_value(GPIO_PORTA | 13)) {
              printk("failed to free line i2c1 sda !!!\n");
        }        
        
        /* release gpios */
        gpio_free(GPIO_PORTA | 12);
        gpio_free(GPIO_PORTA | 13);
        
        /* reset i2c pads to i2c mode */
        mxc_iomux_v3_setup_multiple_pads(mx25_pads_i2c1_func,
			ARRAY_SIZE(mx25_pads_i2c1_func));
          
        return 0;      
}

static const struct imxi2c_platform_data dh_evm_i2c1_data __initconst = {
	.bitrate = 100000,
        .i2c_arbloss = i2c1_arbloss,
};

/*
 * handle arbitration loss on i2c2
 */
 
static iomux_v3_cfg_t mx25_pads_i2c2_func[] = {
	MX25_PAD_GPIO_C__I2C2_CLK,
	MX25_PAD_GPIO_D__I2C2_DAT,
};

static iomux_v3_cfg_t mx25_pads_i2c2_gpios[] = {
	MX25_PAD_GPIO_C__GPIO_C, /* GPIO_PORTA | 2 */
	MX25_PAD_GPIO_D__GPIO_D, /* GPIO_PORTA | 3 */
};

static int i2c2_arbloss(void)
{
        int index;
        printk("handle arbitration loss on i2c2!\n");
                
        /* configure i2c pads to gpios */
        mxc_iomux_v3_setup_multiple_pads(mx25_pads_i2c2_gpios,
			ARRAY_SIZE(mx25_pads_i2c2_gpios));
        
        /* aquire and setup gpios */
        gpio_request(GPIO_PORTA | 2, "i2c2_imx" );
	gpio_direction_output(GPIO_PORTA | 2, 1);
		
	gpio_request(GPIO_PORTA | 3, "i2c2_imx" );
	gpio_direction_input(GPIO_PORTA | 3 );
        
        /* toggle scl until sda line goes to high */
        for (index = 0; index < 8; index++) {
                if (gpio_get_value(GPIO_PORTA | 3)) {
                        printk("i2c2 sda is free!\n");
                        break;
                }
                
                /* toggle scl pin */
                udelay(10);
                gpio_set_value(GPIO_PORTA | 2, 0);
                udelay(10);
                gpio_set_value(GPIO_PORTA | 2, 1);
        }
        
        if (!gpio_get_value(GPIO_PORTA | 3)) {
              printk("failed to free line i2c2 sda !!!\n");
        }        
        
        /* release gpios */
        gpio_free(GPIO_PORTA | 2);
        gpio_free(GPIO_PORTA | 3);
        
        /* reset i2c pads to i2c mode */
        mxc_iomux_v3_setup_multiple_pads(mx25_pads_i2c2_func,
			ARRAY_SIZE(mx25_pads_i2c2_func));
        
        return 0;  
}

static const struct imxi2c_platform_data dh_evm_i2c2_data __initconst = {
	.bitrate = 100000,
        .i2c_arbloss = i2c2_arbloss,
};

static struct i2c_board_info dh_evm_i2c_devices1[] = {
	{
		I2C_BOARD_INFO("rtc-ds1307", 0x6f),
		.type	= "mcp7941x",
	},
};

/* DH-Touch (Single-Touch) - configuration of Infrafit IWRK */
#ifdef CONFIG_TOUCHSCREEN_TMG120_TS
#define TMG120_TS_IRQGPIO	IRQ_GPIOB(9) 

static struct i2c_board_info dh_evm_i2c_devices2[] = {
	{
		I2C_BOARD_INFO("tmg120_ts", 0x20),
		.type= "tmg120_ts",
		.irq = TMG120_TS_IRQGPIO,
	},
};
#endif

static int mx25dh_spi0_cs[] = {GPIO_PORTA | 16};

static const struct spi_imx_master mx25dh_spi0_data __initconst = {
	.chipselect	= mx25dh_spi0_cs,
	.num_chipselect = ARRAY_SIZE(mx25dh_spi0_cs),
};

static int mx25dh_spi2_cs[] = {GPIO_PORTA | 31};

static const struct spi_imx_master mx25dh_spi2_data __initconst = {
	.chipselect	= mx25dh_spi2_cs,
	.num_chipselect = ARRAY_SIZE(mx25dh_spi2_cs),
};

static const struct flash_platform_data mx25dh_spi_slave_data = {
	.type		= "m25p32",
};

static struct spi_board_info __initdata mx25dh_spi0_board_info[] = {
	{
		.modalias	= "spidev",
//		.platform_data	= &mx25dh_spi_slave_data,
		.irq		= -1,
		.max_speed_hz	= 50000000,
		.bus_num	= 1,
		.chip_select	= 0,
	},
	{
		.modalias	= "spidev",
//		.platform_data	= &mx25dh_spi_slave_data,
		.irq		= -1,
		.max_speed_hz	= 50000000,
		.bus_num	= 2,
		.chip_select	= 0,
	},
};

static struct led_pwm mx25dh_pwm_leds[] = {
	{
		.name		= "pwm3:out",
		.pwm_id		= 3,
		.max_brightness	= 255,
		.pwm_period_ns	= 1000000,
	},
};

static struct led_pwm_platform_data mx25dh_pwm_data = {
	.num_leds	= ARRAY_SIZE(mx25dh_pwm_leds),
	.leds		= mx25dh_pwm_leds,
};

static struct platform_device mx25dh_leds_pwm = {
	.name	= "leds_pwm",
	.id	= -1,
	.dev	= {
		.platform_data = &mx25dh_pwm_data,
	},
};



static int otg_mode_host=1;

static int __init dheva_cpuimx25_otg_mode(char *options)
{
	if (!strcmp(options, "host"))
		otg_mode_host = 1;
	else if (!strcmp(options, "device"))
		otg_mode_host = 0;
	else
		pr_info("otg_mode neither \"host\" nor \"device\". "
			"Defaulting to device\n");
	return 0;
}
__setup("otg_mode=", dheva_cpuimx25_otg_mode);

static int otg_power_pin=0;

static int __init dheva_cpuimx25_otg_power_pin(char *options)
{
	if (!strcmp(options, "100"))
		otg_power_pin = 1;
	else if (!strcmp(options, "200"))
		otg_power_pin = 1;
	else {
		otg_power_pin = 0;
		pr_info("No otg power pin. \n");
	}
	return 0;
}
__setup("dheva01_version=", dheva_cpuimx25_otg_power_pin);

static int backlight_invert=0;

static int __init dheva_cpuimx25_bl_invert(char *options)
{
	if (!strcmp(options, "normal"))
		backlight_invert = 0;
	else if (!strcmp(options, "inverted"))
		backlight_invert = 1;
	else
		pr_info("pwm_pol neither \"normal\" nor \"invert\". "
			"Defaulting to normal\n");
	return 0;
}
__setup("pwm_pol=", dheva_cpuimx25_bl_invert);

static int spi_cs_variant=0;

static int __init dheva_cpuimx25_spi_variant(char *options)
{
	if (!strcmp(options, "default"))
		spi_cs_variant = 0;
	else if (!strcmp(options, "variant"))
		spi_cs_variant = 1;
	else
		pr_info("spi_variant neither \"default\" nor \"variant\". "
			"Defaulting to default\n");
	return 0;
}
__setup("spi_variant=", dheva_cpuimx25_spi_variant);

/* onboard microSDcard */
#define GPIO_SD1CD      IMX_GPIO_NR(2, 13)

static struct esdhc_platform_data sd1_pdata = {
         .cd_gpio = GPIO_SD1CD,
         .cd_type = ESDHC_CD_GPIO,
         .wp_type = ESDHC_WP_NONE,
};

/* offboard SDcard */
#define GPIO_SD2CD      IMX_GPIO_NR(2, 21)

static struct esdhc_platform_data sd2_pdata = {
         .cd_gpio = GPIO_SD2CD,
         .cd_type = ESDHC_CD_GPIO,
         .wp_type = ESDHC_WP_NONE,
};

/* dhcom specific sdcard initialization */
static void dhcom_add_sdhci_esdhc_imx(void)
{
	/* setup SD_CARD_SEL gpio */
	gpio_request(GPIO_PORTC | 16, "sdhci-esdhc-imx" );

	if (gpio_get_value(GPIO_PORTC | 16))
	{
		gpio_direction_output(GPIO_PORTC | 16, 1); /* select offboard SD interface */

		/* work with offboard SD-interface carddetect */
		imx25_add_sdhci_esdhc_imx(0, &sd2_pdata);
		printk("DHCOM: work with SD card interface!\n");
	}
	else
	{
		gpio_direction_output(GPIO_PORTC | 16, 0); /* select onboard microSD */

		/* work with onboard microSD carddetect */
		imx25_add_sdhci_esdhc_imx(0, &sd1_pdata);
		printk("DHCOM: work with microSD card!\n");
	}
}


static void __init mx25dh_init(void)
{
	imx25_soc_init();

	mxc_iomux_v3_setup_multiple_pads(mx25dh_pads,
			ARRAY_SIZE(mx25dh_pads));

	imx25_add_imx_uart1(&uart_pdata);
	imx25_add_imx_uart2(&uart_nortscts_pdata);
	imx25_add_imx_uart4(&uart_nortscts_pdata);

	/* OTG Power Pin only if argument was set */
	if (otg_power_pin) {
		if (otg_mode_host) {
			gpio_request(GPIO_PORTC | 21, "ehci-imx" );
			gpio_direction_output(GPIO_PORTC | 21, 1);
			
		} else {
			gpio_request(GPIO_PORTC | 21, "fsl-usb2-udc" );
			gpio_direction_output(GPIO_PORTC | 21, 0);
		}
	}

	if (otg_mode_host) 
		imx25_add_mxc_ehci_otg(&otg_pdata);
	else 
		imx25_add_fsl_usb2_udc(&otg_device_pdata);
	
	gpio_request(GPIO_PORTC | 15, "ehci-imx");
	gpio_direction_output(GPIO_PORTC | 15, 0);

	imx25_add_flexcan0(NULL);

	imx25_add_mxc_ehci_hs(&usbh2_pdata);
	imx25_add_mxc_nand(&mx25dh_nand_board_info);
	//imx25_add_imxdi_rtc(NULL);

	if (backlight_invert)
		mx25dh_fb_pdata.backlight_inverted = 1;

	imx25_add_imx_fb(&mx25dh_fb_pdata);

	imx25_add_imx2_wdt(NULL);

	i2c_register_board_info(1, dh_evm_i2c_devices1,	ARRAY_SIZE(dh_evm_i2c_devices1));

/* Register DH-Touch (Single-Touch) */
#ifdef CONFIG_TOUCHSCREEN_TMG120_TS
	i2c_register_board_info(2, dh_evm_i2c_devices2,	ARRAY_SIZE(dh_evm_i2c_devices2));
#endif
	imx25_add_imx_i2c1(&dh_evm_i2c1_data);
	imx25_add_imx_i2c2(&dh_evm_i2c2_data);

/* Work with onboard resistive Touch */
#ifdef CONFIG_TOUCHSCREEN_IMX_ADC
	imx25_add_mxc_tsc();
#endif

	imx25_add_pwm_mxc0();
	imx25_add_pwm_mxc1();
	imx25_add_pwm_mxc2();
	imx25_add_pwm_mxc3();


#if defined(CONFIG_SPI_IMX) || defined(CONFIG_SPI_IMX_MODULE)
	/* SPI_CS0 init */
	if (spi_cs_variant) {
		/* GPIO A 17 is the CS */
		mx25dh_spi0_cs[0] = GPIO_PORTA | 17;

		/* GPIO A 16 is not used from linux but
		 * lets set it to high, in case somthing is
		 * connected
		*/
		gpio_request(GPIO_PORTA | 16, "spi_imx" );
		gpio_direction_output(GPIO_PORTA | 16, 1);
	} else {
		/* GPIO A 16 is the CS */
		mx25dh_spi0_cs[0] = GPIO_PORTA | 16;

		/* GPIO A 17 is not used from linux but
		 * lets set it to high, in case somthing is
		 * connected
		*/
		gpio_request(GPIO_PORTA | 17, "spi_imx" );
		gpio_direction_output(GPIO_PORTA | 17, 1);
	}

	imx25_add_spi_imx1(&mx25dh_spi0_data);
	imx25_add_spi_imx2(&mx25dh_spi2_data);
	spi_register_board_info(mx25dh_spi0_board_info,
			ARRAY_SIZE(mx25dh_spi0_board_info));
#endif

	imx25_add_fec(&mx25_fec_pdata);
	//imx25_add_imx_keypad(&mx25dh_keymap_data);

	/* dhcom specific sdcard initialization */
	dhcom_add_sdhci_esdhc_imx();
	//imx25_add_sdhci_esdhc_imx(0, NULL);


	platform_device_register(&mx25dh_leds_pwm);
}

static void __init mx25dh_timer_init(void)
{
	mx25_clocks_init();
}

static struct sys_timer mx25dh_timer = {
	.init   = mx25dh_timer_init,
};

MACHINE_START(MX25_DHEVM, "dh-electronics DHeva01")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.atag_offset = 0x100,
	.map_io         = mx25_map_io,
	.init_early = imx25_init_early,
	.init_irq       = mx25_init_irq,
	.init_machine   = mx25dh_init,
	.handle_irq = imx25_handle_irq,
	.timer          = &mx25dh_timer,
MACHINE_END

