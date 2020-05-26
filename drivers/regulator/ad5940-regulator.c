
#include <linux/mfd/ad5940.h>

#include <linux/of.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>

struct ad5940_regulator {
	struct device *dev;
	struct spi_device *spi;

	int count;
};

#define AD5940_REG_BUFSECCON		0x00002180
#define AD5940_ENABLE_V1P8THERMST	BIT(8)
#define AD5940_ENABLE_V1P1LPADCDISCHG	BIT(6)
#define AD5940_ENABLE_V1P1LPADC		BIT(5)
#define AD5940_ENABLE_V1P1HSADC		BIT(4)
#define AD5940_ENABLE_V1P8HSADCDISCHG	BIT(3)
#define AD5940_ENABLE_V1P8LPADC		BIT(2)
#define AD5940_ENABLE_V1P8HSADCLIMIT	BIT(1)
#define AD5940_ENABLE_V1P8HSADC		BIT(0)

static int ad5940_regulator_enable(struct regulator_dev *rdev)
{
	struct ad5940_regulator *regulator = rdev_get_drvdata(rdev);

	return ad5940_write_reg_mask(regulator->spi, rdev->desc->enable_reg,
			rdev->desc->enable_mask, rdev->desc->enable_mask);
}

static int ad5940_regulator_disable(struct regulator_dev *rdev)
{
	struct ad5940_regulator *regulator = rdev_get_drvdata(rdev);

	return ad5940_write_reg_mask(regulator->spi, rdev->desc->enable_reg,
			rdev->desc->enable_mask, 0);
}

static int ad5940_is_regulator_enabled(struct regulator_dev *rdev)
{
	struct ad5940_regulator *regulator = rdev_get_drvdata(rdev);
	u32 tmp;
	int ret;

	ret = ad5940_read_reg(regulator->spi, rdev->desc->enable_reg, &tmp);
	if (ret < 0)
		return ret;
	dev_info(regulator->dev, "mask: %x", rdev->desc->enable_mask);
	return (tmp & rdev->desc->enable_mask) == rdev->desc->enable_mask;
}

static const struct regulator_ops ad5940_regulator_ops = {
	.is_enabled	= ad5940_is_regulator_enabled,
	.enable		= ad5940_regulator_enable,
	.disable	= ad5940_regulator_disable,
};

#define AD5940_REGULATOR(_name, _id, _of_match, _em, _type)	\
	{						\
		.name		= _name,		\
		.id		= _id,			\
		.of_match	= of_match_ptr(_of_match),	\
		.owner		= THIS_MODULE,		\
		.type		= _type,		\
		.enable_reg	= AD5940_REG_BUFSECCON,	\
		.enable_mask	= _em,			\
		.ops		= &ad5940_regulator_ops,\
		.n_voltages	= 1,			\
	}

enum ad5940_regulator_id {
	AD5940_V1P8HSADC,
	AD5940_V1P8HSADCLIMIT,
	AD5940_V1P8LPADC,
	AD5940_V1P8HSADCDISCHG,
	AD5940_V1P1HSADC,
	AD5940_V1P1LPADC,
	AD5940_V1P1LPADCDISCHG,
	AD5940_V1P8THERMST,
};

static struct regulator_desc ad5940_regulators[] = {
	AD5940_REGULATOR("HSADCREFHIGH", AD5940_V1P8HSADC, "hsadc_ref",
			 AD5940_ENABLE_V1P8HSADC, REGULATOR_VOLTAGE),
	AD5940_REGULATOR("HSADCREFHIGH", AD5940_V1P8HSADCLIMIT, "hsadc_ref",
			 AD5940_ENABLE_V1P8HSADCLIMIT, REGULATOR_CURRENT),
	AD5940_REGULATOR("LPADCREGHIGH", AD5940_V1P8LPADC, "lpadc_ref",
			 AD5940_ENABLE_V1P8LPADC, REGULATOR_VOLTAGE),
	AD5940_REGULATOR("HSADCDISCHG", AD5940_V1P8HSADCDISCHG, "hsadc_discharge",
			 AD5940_ENABLE_V1P8HSADCDISCHG, REGULATOR_VOLTAGE),
	AD5940_REGULATOR("HSADCCOM", AD5940_V1P1HSADC, "hsadc_com",
			 AD5940_ENABLE_V1P1HSADC, REGULATOR_VOLTAGE),
	AD5940_REGULATOR("LPADCCOM", AD5940_V1P1LPADC, "lpadc_com",
			 AD5940_ENABLE_V1P1LPADC, REGULATOR_VOLTAGE),
	AD5940_REGULATOR("LPADCDISCHG", AD5940_V1P1LPADCDISCHG, "lpadc_discharge",
			 AD5940_ENABLE_V1P1LPADCDISCHG, REGULATOR_VOLTAGE),
	AD5940_REGULATOR("EXTOUT", AD5940_V1P8THERMST, "refout",
			 AD5940_ENABLE_V1P8THERMST, REGULATOR_VOLTAGE),
};

static struct of_regulator_match ad5940_regulator_matches[] = {
	{ .name = "hsadc_ref" 		},
	{ .name = "hsadc_limit" 	},
	{ .name = "lpadc_ref" 		},
	{ .name = "hsadc_discharge" 	},
	{ .name = "hsadc_com" 		},
	{ .name = "lpadc_com" 		},
	{ .name = "lpadc_discharge" 	},
	{ .name = "ref_out" 		},
};

static int ad5940_regulator_probe(struct platform_device *pdev)
{
	struct ad5940 *ad5940 = dev_get_drvdata(pdev->dev.parent);
	struct ad5940_regulator *regulator;
	struct regulator_dev *rdev;
	struct regulator_config config = { 0 };
	struct device_node *node;
	int ret, i;
	
	regulator = devm_kzalloc(&pdev->dev, sizeof(*regulator), GFP_KERNEL);
	if (!regulator)
		return -ENOMEM;

	regulator->dev = &pdev->dev;
	regulator->spi = ad5940->spi;

	platform_set_drvdata(pdev, regulator);

	node = of_get_child_by_name(pdev->dev.parent->of_node, "regulators");
	if (IS_ERR(node)) {
		dev_err(&pdev->dev, "regulator dt node not found!");
		return PTR_ERR(node);
	}
	ret = of_regulator_match(&pdev->dev, node, ad5940_regulator_matches,
				 ARRAY_SIZE(ad5940_regulator_matches));
	if (ret < 0) {
		dev_err(&pdev->dev, "Error parsing regulator init data: %d\n",
			ret);
		return ret;
	}

	config.dev = &pdev->dev;
	for (i = 0; i < ARRAY_SIZE(ad5940_regulators); i++) {
		config.init_data = ad5940_regulator_matches[i].init_data;
		config.driver_data = regulator;
		config.of_node = ad5940_regulator_matches[i].of_node;
		ad5940_regulators[i].fixed_uV = config.init_data->constraints.min_uV;
		rdev = devm_regulator_register(&pdev->dev, &ad5940_regulators[i], &config);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev, "regulator %s register failed.", ad5940_regulators[i].name);
			return PTR_ERR(rdev);
		}
	}

	return 0;
}

static struct platform_driver ad5940_regulator_driver = {
	.driver = {
		.name   = "ad5940-regulator",
	},
	.probe          = ad5940_regulator_probe,
};

module_platform_driver(ad5940_regulator_driver);

MODULE_AUTHOR("Song Qiang <songqiang1304521@gmail.com>");
MODULE_DESCRIPTION("AD5940 regulator driver");
MODULE_LICENSE("GPL");