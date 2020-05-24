// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Base driver for Analog Devices ADP5520/ADP5501 MFD PMICs
 * LCD Backlight: drivers/video/backlight/adp5520_bl
 * LEDs		: drivers/led/leds-adp5520
 * GPIO		: drivers/gpio/adp5520-gpio (ADP5520 only)
 * Keys		: drivers/input/keyboard/adp5520-keys (ADP5520 only)
 *
 * Copyright 2009 Analog Devices Inc.
 *
 * Author: Michael Hennerich <michael.hennerich@analog.com>
 *
 * Derived from da903x:
 * Copyright (C) 2008 Compulab, Ltd.
 *	Mike Rapoport <mike@compulab.co.il>
 *
 * Copyright (C) 2006-2008 Marvell International Ltd.
 *	Eric Miao <eric.miao@marvell.com>
 */

#include <linux/mfd/ad5940.h>
#include <linux/mfd/core.h>
#include <linux/module.h>

static const struct mfd_cell ad5940_devs[] = {
        {
                .name = "ad5940-adc",
                .of_compatible = "adi,ad5940-adc",
        },
        {
                .name = "ad5940-pinctrl",
                .of_compatible = "adi,ad5940-pinctrl",
        },
        {
                .name = "ad5940-intc",
                .of_compatible = "adi,ad5940-intc",
        },
};

static const u32 ad5940_powerup_setting[][2] = {
	{ 0x0908, 0x02c9 },
	{ 0x0c08, 0x206c },
	{ 0x21f0, 0x0010 },
	{ 0x0410, 0x02c9 },
	{ 0x0a28, 0x0009 },
	{ 0x238c, 0x0104 },
	{ 0x0a04, 0x4859 },
	{ 0x0a04, 0xf27b },
	{ 0x0a00, 0x8009 },
	{ 0x22f0, 0x0000 },
	{ 0x2230, 0xde87a5af },
	{ 0x2250, 0x103f },
	{ 0x22b0, 0x203c },
	{ 0x2230, 0xde87a5a0 },
};

static int ad5940_probe(struct spi_device *spi)
{
        struct ad5940 *ad5940;
	u32 chip_id;
        int ret;
	u8 i;

        ad5940 = devm_kzalloc(&spi->dev, sizeof(*ad5940), GFP_KERNEL);
        if (!ad5940)
                return -ENOMEM;

        ad5940->spi = spi;

        spi_set_drvdata(spi, ad5940);

	ret = ad5940_read_reg(spi, AD5940_AFECON_CHIPID, &chip_id);
	if (ret < 0)
		return ret;
	if (chip_id != AD5940_CHIPID) {
		dev_err(&spi->dev, "Wrong chip ID with 0x%x.\n", chip_id);
		return -ENXIO;
	}

	for (i = 0; i < ARRAY_SIZE(ad5940_powerup_setting); i++)
		ad5940_write_reg(spi, ad5940_powerup_setting[i][0],
				ad5940_powerup_setting[i][1]);

	ret = ad5940_write_reg(spi, AD5940_REG_PMBW,
			       AD5940_PMBW_LP | AD5940_PMBW_BW250);
	if (ret < 0)
		return ret;

        ret = devm_mfd_add_devices(&spi->dev, PLATFORM_DEVID_NONE,
                        ad5940_devs, ARRAY_SIZE(ad5940_devs),
                        NULL, 0, NULL);
        if (ret < 0)
                dev_err(&spi->dev, "Devices registration failed.");

        return ret;
}

static const struct of_device_id ad5940_dt_match[] = {
	{ .compatible = "adi,ad5940" },
	{},
};
MODULE_DEVICE_TABLE(of, ad5940_spi_ids);

static struct spi_driver ad5940_driver = {
        .driver = {
                .name   = "ad5940",
                .of_match_table = ad5940_dt_match,
        },
        .probe          = ad5940_probe,
};
module_spi_driver(ad5940_driver);

MODULE_AUTHOR("Song Qiang <songqiang1304521@gmail.com>");
MODULE_DESCRIPTION("Analog Devices AD5940 analog front end driver");
MODULE_LICENSE("GPL");
