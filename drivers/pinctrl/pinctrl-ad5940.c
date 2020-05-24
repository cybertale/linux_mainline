
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinmux.h>

#include <linux/mfd/ad5940.h>

#include "core.h"
#include "pinctrl-utils.h"

#define AD5940_REG_GP0CON	0x00000000
#define AD5940_REG_GP0OEN	0x00000004
#define AD5940_REG_GP0PE	0x00000008
#define AD5940_REG_GP0IEN	0x0000000C
#define AD5940_REG_GP0IN	0x00000010
#define AD5940_REG_GP0OUT	0x00000014
#define AD5940_REG_GP0SET	0x00000018
#define AD5940_REG_GP0CLR	0x0000001C
#define AD5940_REG_GP0TGL	0x00000020

struct ad5940_pinctrl {
	struct device *dev;
	struct pinctrl_dev *pinctrl;
	struct pinctrl_gpio_range range;
	struct gpio_chip chip;
	struct spi_device *spi;
};

struct ad5940_function_group {
	const char *name;
	const char * const *group;
};

struct ad5940_pin_group {
	const char *name;
	const unsigned int *pins;
};

static const struct pinctrl_pin_desc ad5940_gpio_pins[] = {
	PINCTRL_PIN(0, "gpio0"),
	PINCTRL_PIN(1, "gpio1"),
	PINCTRL_PIN(2, "gpio2"),
	PINCTRL_PIN(3, "gpio3"),
	PINCTRL_PIN(4, "gpio4"),
	PINCTRL_PIN(5, "gpio5"),
	PINCTRL_PIN(6, "gpio6"),
	PINCTRL_PIN(7, "gpio7"),
};

static const unsigned int gpio0_pins[] = { 0 };
static const unsigned int gpio1_pins[] = { 1 };
static const unsigned int gpio2_pins[] = { 2 };
static const unsigned int gpio3_pins[] = { 3 };
static const unsigned int gpio4_pins[] = { 4 };
static const unsigned int gpio5_pins[] = { 5 };
static const unsigned int gpio6_pins[] = { 6 };
static const unsigned int gpio7_pins[] = { 7 };

static struct ad5940_pin_group ad5940_pin_groups[] = {
	{
		.name = "gpio0",
		.pins = gpio0_pins,
	},
	{
		.name = "gpio1",
		.pins = gpio1_pins,
	},
	{
		.name = "gpio2",
		.pins = gpio2_pins,
	},
	{
		.name = "gpio3",
		.pins = gpio3_pins,
	},
	{
		.name = "gpio4",
		.pins = gpio4_pins,
	},
	{
		.name = "gpio5",
		.pins = gpio5_pins,
	},
	{
		.name = "gpio6",
		.pins = gpio6_pins,
	},
	{
		.name = "gpio7",
		.pins = gpio7_pins,
	},
};

static const char * const gpiogrp0[] = { "gpio0" };
static const char * const gpiogrp1[] = { "gpio1" };
static const char * const gpiogrp2[] = { "gpio2" };
static const char * const gpiogrp3[] = { "gpio3" };
static const char * const gpiogrp4[] = { "gpio4" };
static const char * const gpiogrp5[] = { "gpio5" };
static const char * const gpiogrp6[] = { "gpio6" };
static const char * const gpiogrp7[] = { "gpio7" };

// Here one pin group has only one pin, and since core functions index groups with numerical index, we can use pin index instead.
static const struct ad5940_function_group ad5940_functions[] = {
	{	.name = "int0out",	.group = gpiogrp0,	},
	{	.name = "seq0trig",	.group = gpiogrp0,	},
	{	.name = "sync0out",	.group = gpiogrp0,	},
	{	.name = "gpio0",	.group = gpiogrp0,	},
	{	.name = "gpio1",	.group = gpiogrp1,	},
	{	.name = "seq1trig",	.group = gpiogrp1,	},
	{	.name = "sync1out",	.group = gpiogrp1,	},
	{	.name = "deepsleep",	.group = gpiogrp1,	},
	{	.name = "por",		.group = gpiogrp2,	},
	{	.name = "seq2trig",	.group = gpiogrp2,	},
	{	.name = "sync2out",	.group = gpiogrp2,	},
	{	.name = "extclk",	.group = gpiogrp2,	},
	{	.name = "gpio3",	.group = gpiogrp3,	},
	{	.name = "seq3trig",	.group = gpiogrp3,	},
	{	.name = "sync3out",	.group = gpiogrp3,	},
	{	.name = "int0out",	.group = gpiogrp3,	},
	{	.name = "gpio4",	.group = gpiogrp4,	},
	{	.name = "seq0trig",	.group = gpiogrp4,	},
	{	.name = "sync4out",	.group = gpiogrp4,	},
	{	.name = "int1out",	.group = gpiogrp4,	},
	{	.name = "gpio5",	.group = gpiogrp5,	},
	{	.name = "seq1trig",	.group = gpiogrp5,	},
	{	.name = "sync5out",	.group = gpiogrp5,	},
	{	.name = "extclk",	.group = gpiogrp5,	},
	{	.name = "gpio6",	.group = gpiogrp6,	},
	{	.name = "seq2trig",	.group = gpiogrp6,	},
	{	.name = "sync6out",	.group = gpiogrp6,	},
	{	.name = "int0out",	.group = gpiogrp6,	},
	{	.name = "gpio7",	.group = gpiogrp7,	},
	{	.name = "seq3trig",	.group = gpiogrp7,	},
	{	.name = "sync7out",	.group = gpiogrp7,	},
	{	.name = "int1out",	.group = gpiogrp7,	},
};

static int ad5940_get_group_pins(struct pinctrl_dev *pinctrl_dev,
				unsigned int group,
				const unsigned int **pins,
				unsigned int *num_pins)
{
	*pins = (unsigned int *)ad5940_pin_groups[group].pins;
	*num_pins = 1;
	return 0;
}

static int ad5940_get_groups_count(struct pinctrl_dev *pinctrl_dev)
{
	return ARRAY_SIZE(ad5940_pin_groups);
}

static const char *ad5940_get_group_name(struct pinctrl_dev *pinctrl_dev,
				unsigned int group)
{
	return ad5940_pin_groups[group].name;
}

static const struct pinctrl_ops ad5940_pinctrl_ops = {
	.get_group_pins		= ad5940_get_group_pins,
	.get_groups_count	= ad5940_get_groups_count,
	.get_group_name		= ad5940_get_group_name,
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_all,
	.dt_free_map		= pinctrl_utils_free_map,
};

static int ad5940_pmx_get_functions_count(struct pinctrl_dev *pinctrl_dev)
{
	return ARRAY_SIZE(ad5940_functions);
}

static const char *ad5940_pmx_get_function_name(struct pinctrl_dev *pinctrl_dev, unsigned int function)
{
	return ad5940_functions[function].name;
}

static int ad5940_pmx_get_function_group(struct pinctrl_dev *pinctrl_dev,
					unsigned int function,
					const char *const **groups,
					unsigned int *const num_groups)
{
	*groups = ad5940_functions[function].group;
	*num_groups = 1;
	return 0;
}

static int ad5940_pmx_set_mux(struct pinctrl_dev *pinctrl_dev,
				unsigned int function,
				unsigned int group)
{
	struct ad5940_pinctrl *ad5940_pinctrl = pinctrl_dev_get_drvdata(pinctrl_dev);

	//Group is pin index here.
	return ad5940_write_reg_mask(ad5940_pinctrl->spi, AD5940_REG_GP0CON,
			0x3 << group * 2, (function - group * 4) << group * 2);

}

static const struct pinmux_ops ad5940_pmx_ops = {
	.get_functions_count	= ad5940_pmx_get_functions_count,
	.get_function_name	= ad5940_pmx_get_function_name,
	.get_function_groups	= ad5940_pmx_get_function_group,
	.set_mux		= ad5940_pmx_set_mux,
};

static int ad5940_pinconf_get(struct pinctrl_dev *pinctrl_dev,
				unsigned pin,
				unsigned long *config)
{
	struct ad5940_pinctrl *ad5940_pinctrl = pinctrl_dev_get_drvdata(pinctrl_dev);
	u32 tmp;
	int ret;
	
	*config = 0;
	if (pin == 2)
		return -ENOTSUPP;

	ret = ad5940_read_reg(ad5940_pinctrl->spi, AD5940_REG_GP0PE, &tmp);
	if (ret < 0)
		return ret;
	// pin starts at 0?????????
	if (tmp & 1 << pin)
		*config |= AD5940_GPIO_PULLUP;

	return 0;
}

static int ad5940_pinconf_set(struct pinctrl_dev *pinctrl_dev,
				unsigned pin,
				unsigned long *configs,
				unsigned num_configs)
{
	struct ad5940_pinctrl *ad5940_pinctrl = pinctrl_dev_get_drvdata(pinctrl_dev);
	int i, ret, tmp;

	for (i = 0; i < num_configs; i++) {
		if (configs[i] & AD5940_GPIO_PULLUP)
			tmp = 1;
		else
			tmp = 0;

		ret = ad5940_write_reg_mask(ad5940_pinctrl->spi, AD5940_REG_GP0PE, 1 << pin, tmp << pin);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static const struct pinconf_ops ad5940_pinconf_ops = {
	.pin_config_get		= ad5940_pinconf_get,
	.pin_config_set		= ad5940_pinconf_set,
};

static int ad5940_pinctrl_get_value(struct gpio_chip *chip, unsigned offset)
{
	struct ad5940_pinctrl *ad5940_pinctrl = gpiochip_get_data(chip);
	u32 tmp;
	int ret;

	ret = ad5940_read_reg(ad5940_pinctrl->spi, AD5940_REG_GP0IN, &tmp);
	if (ret < 0)
		return ret;

	return !!(tmp & (1 << offset));
}

static void ad5940_pinctrl_set_value(struct gpio_chip *chip, unsigned offset, int val)
{
	struct ad5940_pinctrl *ad5940_pinctrl = gpiochip_get_data(chip);
	int tmp;

	if (val)
		tmp = 1;
	else
		tmp = 0;
	ad5940_write_reg_mask(ad5940_pinctrl->spi, AD5940_REG_GP0OUT, 1 << offset, tmp << offset);
}

static int ad5940_pinctrl_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct ad5940_pinctrl *ad5940_pinctrl = gpiochip_get_data(chip);
	int ret;

	ret = ad5940_write_reg_mask(ad5940_pinctrl->spi, AD5940_REG_GP0IEN, 1 << offset, 1 << offset);
	if (ret < 0)
		return ret;
	ret = ad5940_write_reg_mask(ad5940_pinctrl->spi, AD5940_REG_GP0OEN, 1 << offset, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad5940_pinctrl_direction_output(struct gpio_chip *chip, unsigned offset, int val)
{
	struct ad5940_pinctrl *ad5940_pinctrl = gpiochip_get_data(chip);
	int ret;

	ret = ad5940_write_reg_mask(ad5940_pinctrl->spi, AD5940_REG_GP0OEN, 1 << offset, 1 << offset);
	if (ret < 0)
		return ret;
	ret = ad5940_write_reg_mask(ad5940_pinctrl->spi, AD5940_REG_GP0IEN, 1 << offset, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad5940_pinctrl_chip_register(struct ad5940_pinctrl *ad5940_pinctrl)
{
	int ret;
	struct gpio_chip *chip = &ad5940_pinctrl->chip;

	chip->direction_input = ad5940_pinctrl_direction_input;
	chip->direction_output = ad5940_pinctrl_direction_output;
	chip->set = ad5940_pinctrl_set_value;
	chip->get = ad5940_pinctrl_get_value;
	chip->base = -1;
	chip->ngpio = 8;
	chip->label = "AD5940-gpio";
	chip->owner = THIS_MODULE;

	// All lines started as input.
	ad5940_write_reg(ad5940_pinctrl->spi, AD5940_REG_GP0IEN, 0x00FF);
	ad5940_write_reg(ad5940_pinctrl->spi, AD5940_REG_GP0OEN, 0x0000);

	ret = devm_gpiochip_add_data(ad5940_pinctrl->dev, chip, ad5940_pinctrl);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad5940_pinctrl_probe(struct platform_device *pdev)
{
	struct ad5940_pinctrl *ad5940_pinctrl;
	struct pinctrl_desc *pinctrl_desc;
	struct ad5940 *ad5940 = dev_get_drvdata(pdev->dev.parent);

	ad5940_pinctrl = devm_kzalloc(&pdev->dev, sizeof(*ad5940_pinctrl), GFP_KERNEL);
	if (!ad5940_pinctrl)
		return -ENOMEM;

	ad5940_pinctrl->dev = &pdev->dev;
	ad5940_pinctrl->spi = ad5940->spi;

	pinctrl_desc = devm_kzalloc(&pdev->dev, sizeof(*pinctrl_desc), GFP_KERNEL);
	if (!pinctrl_desc)
		return -ENOMEM;

	ad5940_pinctrl_chip_register(ad5940_pinctrl);

	pinctrl_desc->owner = THIS_MODULE;
	pinctrl_desc->pctlops = &ad5940_pinctrl_ops;
	pinctrl_desc->pmxops = &ad5940_pmx_ops;
	pinctrl_desc->confops = &ad5940_pinconf_ops;
	pinctrl_desc->name = "ad5940-pinctrl";
	pinctrl_desc->pins = ad5940_gpio_pins;
	pinctrl_desc->npins = ARRAY_SIZE(ad5940_gpio_pins);

	platform_set_drvdata(pdev, ad5940_pinctrl);

	ad5940_pinctrl->pinctrl = devm_pinctrl_register(&pdev->dev, pinctrl_desc, ad5940_pinctrl);
	if (IS_ERR(ad5940_pinctrl->pinctrl))
		return PTR_ERR(ad5940_pinctrl->pinctrl);

	ad5940_pinctrl->range.name = "gpio";
	ad5940_pinctrl->range.npins = 8;
	ad5940_pinctrl->range.gc = &ad5940_pinctrl->chip;

	pinctrl_add_gpio_range(ad5940_pinctrl->pinctrl, &ad5940_pinctrl->range);

	return 0;
}

static struct platform_driver ad5940_pinctrl_driver = {
	.driver = {
		.name   = "ad5940-pinctrl",
	},
	.probe          = ad5940_pinctrl_probe,
};

module_platform_driver(ad5940_pinctrl_driver);

MODULE_AUTHOR("Song Qiang <songqiang1304521@gmail.com>");
MODULE_DESCRIPTION("AD5940 pinctrl driver");
MODULE_LICENSE("GPL");