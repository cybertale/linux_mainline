
#include <linux/iio/iio.h>
#include <linux/mfd/ad5940.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>

#define AD5940_REG_HSDACDAT		0x00002048

struct ad5940_dac {
	struct spi_device *spi;
	struct iio_dev *iio;
	int vref;
};

//Ext info to control gain.

static int ad5940_dac_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct ad5940_dac *dac = iio_priv(indio_dev);
	int ret;
	u32 tmp;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = ad5940_read_reg(dac->spi, AD5940_REG_HSDACDAT, &tmp);
		if (ret < 0)
			return ret;
		*val = tmp;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 1000;
		*val2 = 12;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int ad5940_dac_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{
	struct ad5940_dac *dac = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val < 0x200 || val > 0xe00)
			return -EINVAL;
		return ad5940_write_reg(dac->spi, AD5940_REG_HSDACDAT, val);
	default:
		return -EINVAL;
	}
}

static const struct iio_info ad5940_dac_iio_info = {
	.write_raw = ad5940_dac_write_raw,
	.read_raw = ad5940_dac_read_raw,
};

static const struct iio_chan_spec ad5940_dac_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.output = 1,
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_SCALE),
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 32,
		},
	},
};

static int ad5940_dac_probe(struct platform_device *pdev)
{
	struct ad5940 *ad5940 = dev_get_drvdata(pdev->dev.parent);
	struct ad5940_dac *dac;
	struct iio_dev *indio_dev;
	int ret;
	
	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*dac));
	if (!indio_dev)
		return -ENOMEM;

	dac = iio_priv(indio_dev);
	dac->spi = ad5940->spi;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &ad5940_dac_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->channels = ad5940_dac_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad5940_dac_channels);

	ret = ad5940_write_reg_mask(dac->spi, AD5940_REG_AFECON, AD5940_AFECON_DACEN_MSK,
				AD5940_AFECON_DACEN);
	if (ret < 0)
		return ret;

	// HSDAC is always using the internal reference.
	dac->vref = AD5940_INTERNAL_VREF;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret < 0)
		return ret;

	return 0;
}

static struct platform_driver ad5940_dac_driver = {
	.driver = {
		.name   = "ad5940-dac",
	},
	.probe          = ad5940_dac_probe,
};

module_platform_driver(ad5940_dac_driver);

MODULE_AUTHOR("Song Qiang <songqiang1304521@gmail.com>");
MODULE_DESCRIPTION("AD5940 DAC driver");
MODULE_LICENSE("GPL");