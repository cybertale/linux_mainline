// SPDX-License-Identifier: GPL
/*
 * AD5940 SPI ADC driver
 *
 * Copyright (C) 2020 Song Qiang <songqiang1304521@gmail.com>
 * 
 * TODO: to trigger only once, will need to diable adc conversion before clear irq, IRQ will trigger twice.
 */

#include <linux/bsearch.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/mfd/ad5940.h>

#include <linux/iio/iio.h>

#define AD5940_REG_ADCCON		0x000021A8
#define AD5940_ADCCON_PGA_MSK		GENMASK(18, 16)
#define AD5940_ADCCON_PGA_1		FIELD_PREP(AD5940_ADCCON_PGA_MSK, 0)
#define AD5940_ADCCON_PGA_1P5		FIELD_PREP(AD5940_ADCCON_PGA_MSK, 1)
#define AD5940_ADCCON_PGA_2		FIELD_PREP(AD5940_ADCCON_PGA_MSK, 2)
#define AD5940_ADCCON_PGA_4		FIELD_PREP(AD5940_ADCCON_PGA_MSK, 3)
#define AD5940_ADCCON_PGA_9		FIELD_PREP(AD5940_ADCCON_PGA_MSK, 4)
#define AD5940_ADCCON_MUX_MSK		(GENMASK(12, 8) | GENMASK(5, 0))

#define AD5940_REG_REPEATADCCNV		0x000021F0

#define AD5940_CHANNEL_AINP_MSK		GENMASK(5, 0)
#define AD5940_CHANNEL_AINP(x)		FIELD_PREP(AD5940_CHANNEL_AINP_MSK, x)
#define AD5940_CHANNEL_AINN_MSK		GENMASK(12, 8)
#define AD5940_CHANNEL_AINN(x)		FIELD_PREP(AD5940_CHANNEL_AINN_MSK, x)

#define AD5940_CHANNEL_NAME		0

struct ad5940_channel_config {
	u32 ain;
	const char *channel_name;
};

struct ad5940_state {
	struct spi_device *spi;
	struct ad5940 *ad5940;
	/*
	 * This lock is for protecting the current sequence of SPI r/w commands
	 * will not be interrupted by other r/w sequences.
	 */
	struct mutex lock;
	struct completion measuring_done;
	u32 conversion_time;

	u8 n_input;
	u8 p_input;

	struct regulator *vref;
	int vref_mv;

	int num_channels;
	struct ad5940_channel_config *channel_config;

	int irq;
};

static ssize_t ad5940_read_info(struct iio_dev *indio_dev,
				uintptr_t private,
				const struct iio_chan_spec *chan,
				char *buf)
{
	struct ad5940_state *st = iio_priv(indio_dev);

	switch ((u32)private) {
	case AD5940_CHANNEL_NAME:
		return sprintf(buf, "%s\n",
			st->channel_config[chan->address].channel_name);
	default:
		return -EINVAL;
	}
}

static const struct iio_chan_spec_ext_info ad4590_ext_info[] = {
	{
		.name = "name",
		.read = ad5940_read_info,
		.private = AD5940_CHANNEL_NAME,
		.shared = IIO_SEPARATE,
	},
	{ },
};

static const struct iio_chan_spec ad5940_channel_template = {
	.type = IIO_VOLTAGE,
	.differential = 1,
	.indexed = 1,
	.ext_info = ad4590_ext_info,
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
};

/*
 * IRQ can sleep in interrupt controller of AD5940 and
 * stopping of conversion can not be threaded, or it will
 * keep generating interrupts.
 */
static irqreturn_t ad5940_irq_handler(int irq, void *private)
{
	struct ad5940_state *st = private;
	int ret;

	ret = ad5940_write_reg_mask(st->spi, AD5940_REG_AFECON, AD5940_AFECON_ADCCONVEN, 0);
	if (ret < 0)
		dev_err(&st->spi->dev, "adc conversion stop failed.");

	complete(&st->measuring_done);
	return IRQ_HANDLED;
}

static int ad5940_read(struct ad5940_state *st, u32 mux, int *val)
{
	int ret;
	u32 tmp;

	mutex_lock(&st->lock);
	ret = ad5940_write_reg(st->spi, AD5940_REG_REPEATADCCNV, 0x10);
	if (ret < 0)
		goto unlock_return;

	ret = ad5940_write_reg_mask(st->spi, AD5940_REG_ADCCON, AD5940_ADCCON_MUX_MSK,
				    mux);
	if (ret < 0)
		goto unlock_return;

	reinit_completion(&st->measuring_done);
	// enable_irq(st->irq);
	ret = ad5940_write_reg_mask(st->spi, AD5940_REG_AFECON,
				    AD5940_AFECON_ADCCONVEN_MSK,
				    AD5940_AFECON_ADCCONVEN);
	if (ret < 0)
		goto unlock_return;
	/*
	 * The highest conversion time (1/800000s, which is 1.25us)
	 * plus the SPI read time.
	 */
	ret = wait_for_completion_timeout(&st->measuring_done,
					  msecs_to_jiffies(100));
	if (!ret) {
		ret = -ETIMEDOUT;
		ad5940_write_reg_mask(st->spi, AD5940_REG_AFECON, AD5940_AFECON_ADCCONVEN, 0);
		goto unlock_return;
	}

	ad5940_read_reg(st->spi, AD5940_REG_ADCDAT, &tmp);
	*val = tmp;
	ret = 0;

unlock_return:
	mutex_unlock(&st->lock);
	return ret;
}

static int ad5940_read_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val, int *val2, long info)
{
	struct ad5940_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		dev_info(&indio_dev->dev, "mux %d, %d", chan->channel, chan->channel2);
		ret = ad5940_read(st, chan->channel | chan->channel2, val);
		if (ret != 0)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = st->vref_mv;
		*val2 = 16;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static const struct iio_info ad5940_info = {
	.read_raw = &ad5940_read_raw,
};

int cmp_u8(const void *a, const void *b)
{
	return (*(u8 *)a - *(u8 *)b);
}

static int ad5940_check_channel_indexes(struct device *dev, u32 *ain)
{
	const u8 channel_p[] = {
		0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 18, 19,
		20, 22, 23, 24, 25, 26, 31, 33, 35, 36
	};
	const u8 channel_n[] = {
		0, 1, 2, 4, 5, 6, 7, 10, 11, 12, 14, 16, 17, 20
	};
	u8 *index;

	index = (u8 *) bsearch(&ain[0], channel_p, ARRAY_SIZE(channel_p),
				sizeof(u8), cmp_u8);
	if (!index) {
		dev_err(dev, "Positive input index not found.\n");
		return -EINVAL;
	}

	index = (u8 *) bsearch(&ain[1], channel_n, ARRAY_SIZE(channel_n),
				sizeof(u8), cmp_u8);
	if (!index) {
		dev_err(dev, "negtive input index not found.\n");
		return -EINVAL;
	}

	return 0;
}

static int ad5940_of_parse_channel_config(struct iio_dev *indio_dev,
					  struct device_node *np)
{
	struct ad5940_state *st = iio_priv(indio_dev);
	struct iio_chan_spec *chan;
	struct device_node *child;
	u32 channel, ain[2];
	int ret;

	st->num_channels = of_get_available_child_count(np);
	if (!st->num_channels) {
		dev_err(indio_dev->dev.parent, "no channel children\n");
		return -ENODEV;
	}

	chan = devm_kcalloc(indio_dev->dev.parent, st->num_channels,
			    sizeof(*chan), GFP_KERNEL);
	if (!chan)
		return -ENOMEM;

	st->channel_config = devm_kcalloc(indio_dev->dev.parent,
					  st->num_channels,
					  sizeof(*st->channel_config),
					  GFP_KERNEL);
	if (!st->channel_config)
		return -ENOMEM;

	indio_dev->channels = chan;
	indio_dev->num_channels = st->num_channels;

	for_each_available_child_of_node(np, child) {
		ret = of_property_read_u32(child, "reg", &channel);
		if (ret)
			goto err;

		ret = of_property_read_u32_array(child, "diff-channels",
						 ain, 2);
		if (ret)
			goto err;

		ret = of_property_read_string(child, "channel-name",
				&st->channel_config[channel].channel_name);
		if (ret)
			st->channel_config[channel].channel_name = "none-name";

		ret = ad5940_check_channel_indexes(indio_dev->dev.parent, ain);
		if (ret) {
			dev_err(indio_dev->dev.parent,
				"some input channel index does not exist: %d, %d, %d",
				channel, ain[0], ain[1]);
			goto err;
		}

		st->channel_config[channel].ain = AD5940_CHANNEL_AINP(ain[0]) |
						  AD5940_CHANNEL_AINN(ain[1]);

		*chan = ad5940_channel_template;
		chan->address = channel;
		chan->scan_index = channel;
		chan->channel = ain[0];
		chan->channel2 = ain[1];

		chan++;
	}

	return 0;
err:
	of_node_put(child);

	return ret;
}

// static int ad5940_config_polarity(struct ad5940_state *st, u32 polarity)
// {
// 	u32 val;

// 	if (polarity == IRQF_TRIGGER_RISING)
// 		val = AD5940_INTCPOL_POS;
// 	else
// 		val = AD5940_INTCPOL_NEG;

// 	return ad5940_write_reg_mask(st->spi, AD5940_REG_INTCPOL,
// 				     AD5940_INTCPOL_MSK, val);
// }

static int ad5940_setup(struct ad5940_state *st)
{
	int ret;

	ret = ad5940_write_reg_mask(st->spi, AD5940_REG_ADCCON,
				    AD5940_ADCCON_PGA_MSK,
				    AD5940_ADCCON_PGA_1);
	if (ret < 0)
		return ret;

	return ad5940_write_reg_mask(st->spi, AD5940_REG_AFECON,
				     AD5940_AFECON_ADCEN_MSK,
			             AD5940_AFECON_ADCEN);
}

static void ad5940_regulator_disable(void *data)
{
	struct ad5940_state *st = data;

	regulator_disable(st->vref);
}

static int ad5940_adc_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct ad5940_state *st;
	struct ad5940 *ad5940 = dev_get_drvdata(pdev->dev.parent);
	int vref_uv = 0;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->ad5940 = ad5940;
	st->spi = ad5940->spi;
	init_completion(&st->measuring_done);

	st->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!st->irq) {
		dev_err(&pdev->dev, "read irq from dt failed.");
		return st->irq;
	}

	ret = devm_request_irq(&pdev->dev, st->irq,
					ad5940_irq_handler,
					0, dev_name(&pdev->dev),
					st);
	if (ret < 0) {
		dev_err(&pdev->dev, "irq request failed.");
		return ret;
	}

	st->vref = devm_regulator_get_optional(&pdev->dev, "vref");
	if (!IS_ERR(st->vref)) {
		ret = regulator_enable(st->vref);
		if (ret) {
			dev_err(&pdev->dev, "Failed to enbale specified vref supply.\n");
			return ret;
		}

		ret = devm_add_action_or_reset(&pdev->dev,
				ad5940_regulator_disable, st);
		if (ret) {
			regulator_disable(st->vref);
			return ret;
		}

		ret = regulator_get_voltage(st->vref);
		if (ret < 0)
			return ret;

		vref_uv = ret;
	}

	if (vref_uv)
		st->vref_mv = vref_uv / 1000;
	else
		st->vref_mv = 1820;

	indio_dev->dev.parent = &st->spi->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad5940_info;

	ret = ad5940_of_parse_channel_config(indio_dev, pdev->dev.of_node);
	if (ret < 0)
		return ret;

	ret = ad5940_setup(st);
	if (ret)
		return ret;

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static struct platform_driver ad5940_adc_driver = {
	.driver = {
		.name = "ad5940-adc",
	},
	.probe = ad5940_adc_probe,
};
module_platform_driver(ad5940_adc_driver);

MODULE_AUTHOR("Song Qiang <songqiang1304521@gmail.com>");
MODULE_DESCRIPTION("Analog Device AD5940 ADC driver");
MODULE_LICENSE("GPL");