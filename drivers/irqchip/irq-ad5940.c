

//TODO: Therre are two intc, now hac

#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>

#include <linux/mfd/ad5940.h>

#define AD5940_INT_NUM		24
#define AD5940_INT_NUM_ALL	32

#define AD5940_REG_INTCSEL0		0x00003008
#define AD5940_REG_INTCSEL1		0x0000300C
#define AD5940_REG_INTCFLAG0		0x00003010
#define AD5940_REG_INTCPOL		0x00003000	/* Interrupt Polarity. */
#define AD5940_INTCPOL_MSK		BIT(0)
#define AD5940_INTCPOL_POS		FIELD_PREP(AD5940_INTCPOL_MSK, 1)
#define AD5940_INTCPOL_NEG		FIELD_PREP(AD5940_INTCPOL_MSK, 0)
#define AD5940_REG_INTCLR		0x00003004

struct ad5940_intc {
	struct device *dev;
	struct spi_device *spi;
	int parent_irq;
	struct irq_domain *domain;
	struct work_struct ad5940_isr_work;
};

static u32 ad5940_translate_local_irq(u32 index)
{
	if (index == 23)
		return 31;
	else if (index == 22)
		return 29;
	else if (index >= 17)
		return index + 6;
	else if (index >= 14)
		return index + 1;
	else
		return index;
}

static void ad5940_irq_ack(struct irq_data *data)
{
	struct ad5940_intc *intc = irq_data_get_irq_chip_data(data);

	ad5940_write_reg(intc->spi, AD5940_REG_INTCLR,
		1 << ad5940_translate_local_irq(data->hwirq));
}

static void ad5940_irq_mask(struct irq_data *data)
{
	struct ad5940_intc *intc = irq_data_get_irq_chip_data(data);

	ad5940_write_reg_mask(intc->spi, AD5940_REG_INTCSEL0,
		1 << ad5940_translate_local_irq(data->hwirq), 0);
}

static void ad5940_irq_unmask(struct irq_data *data)
{
	struct ad5940_intc *intc = irq_data_get_irq_chip_data(data);
	u32 tmp = ad5940_translate_local_irq(data->hwirq);

	ad5940_write_reg_mask(intc->spi, AD5940_REG_INTCSEL0, 1 << tmp, 1 << tmp);
}

static struct irq_chip ad5940_irq_chip = {
	.name		= "ad5940-irq",
	.irq_ack	= ad5940_irq_ack,
	.irq_mask	= ad5940_irq_mask,
	.irq_unmask	= ad5940_irq_unmask,
};

static irqreturn_t ad5940_thread_irq_handler(int parent_irq, void *data)
{

	struct ad5940_intc *intc = data;
	unsigned long bitmap;
	unsigned long bit;
	unsigned int irq;
	u32 tmp;
	int ret;

	ret = ad5940_read_reg(intc->spi, AD5940_REG_INTCFLAG0, &tmp);
	if (ret < 0) {
		dev_err(intc->dev, "FLAG read error in irq with %d", ret);
		return IRQ_HANDLED;
	}

	bitmap = tmp;
	for_each_set_bit(bit, &bitmap, AD5940_INT_NUM_ALL) {
		irq = irq_find_mapping(intc->domain, bit);
		local_irq_disable();
		generic_handle_irq(irq);
		local_irq_enable();
	}

	return IRQ_HANDLED;
}

static irqreturn_t ad5940_irq_handler(int irq, void *private)
{
	return IRQ_WAKE_THREAD;
}

static int ad5940_irq_map(struct irq_domain *h, unsigned int virq,
			irq_hw_number_t hw)
{
	irq_set_chip_and_handler(virq, &ad5940_irq_chip, handle_edge_irq);
	irq_set_chip_data(virq, h->host_data);

	return 0;
}

static const struct irq_domain_ops ad5940_irq_ops = {
	.map = ad5940_irq_map,
};

static int ad5940_irq_probe(struct platform_device *pdev)
{
	struct ad5940 *ad5940 = dev_get_drvdata(pdev->dev.parent);
	struct ad5940_intc *intc;
	int ret;
	
	intc = devm_kzalloc(&pdev->dev, sizeof(*intc), GFP_KERNEL);
	if (!intc)
		return -ENOMEM;

	intc->dev = &pdev->dev;
	intc->spi = ad5940->spi;

	intc->parent_irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!intc->parent_irq) {
		dev_err(&pdev->dev, "irq parse from dt failed.");
		return -ENODEV;
	}

	ad5940_write_reg(intc->spi, AD5940_REG_INTCSEL0, 0x0);

	ret = devm_request_threaded_irq(&pdev->dev, intc->parent_irq, ad5940_irq_handler,
				ad5940_thread_irq_handler, IRQ_TYPE_EDGE_FALLING | IRQF_ONESHOT, "ad5940-intc", intc);
	if (ret < 0) {
		dev_err(&pdev->dev, "irq request failed");
		return ret;
	}

	intc->domain = irq_domain_add_linear(pdev->dev.of_node, AD5940_INT_NUM, &ad5940_irq_ops, intc);
	if (!intc->domain) {
		return -ENOMEM;
	}
	intc->domain->name = "ad5940-irq-domain";

	dev_info(&pdev->dev, "interrupt controller registered.");

	return 0;
}

static struct platform_driver ad5940_intc_driver = {
	.driver = {
		.name   = "ad5940-intc",
	},
	.probe          = ad5940_irq_probe,
};

module_platform_driver(ad5940_intc_driver);

MODULE_AUTHOR("Song Qiang <songqiang1304521@gmail.com>");
MODULE_DESCRIPTION("AD5940 interrupt controller driver");
MODULE_LICENSE("GPL");
