
#include <linux/bitfield.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

struct ad5940_adc_platform_data {

};

struct ad5940_gpio_platform_data {

};

struct ad5940 {
        struct spi_device *spi;
};

#define AD5940_SPICMD_SETADDR		0x20      /* set the register address that is going to operate. */
#define AD5940_SPICMD_READREG		0x6d      /* command to read register */
#define AD5940_SPICMD_WRITEREG		0x2d      /* command to write register */
#define AD5940_SPICMD_READFIFO		0x5f      /* command to read FIFO */

#define AD5940_CHANNEL_AINP_MSK		GENMASK(5, 0)
#define AD5940_CHANNEL_AINP(x)		FIELD_PREP(AD5940_CHANNEL_AINP_MSK, x)
#define AD5940_CHANNEL_AINN_MSK		GENMASK(12, 8)
#define AD5940_CHANNEL_AINN(x)		FIELD_PREP(AD5940_CHANNEL_AINN_MSK, x)

#define AD5940_CHANNEL_NAME		0

#define AD5940_REG_AFECON		0x00002000	/* AFE Configuration Register. */
#define AD5940_AFECON_ADCCONVEN_MSK	BIT(8)		/* ADC Conversion Enable. */
#define AD5940_AFECON_ADCCONVEN		FIELD_PREP(AD5940_AFECON_ADCCONVEN_MSK, 1)
#define AD5940_AFECON_ADCEN_MSK		BIT(7)		/* ADC Power Enable. */
#define AD5940_AFECON_ADCEN		FIELD_PREP(AD5940_AFECON_ADCEN_MSK, 1)

#define AD5940_REG_ADCDAT		0x00002074	/* ADC Raw Result. */
#define AD5940_REG_DFTREAL		0x00002078	/* DFT Result, Real Part. */
#define AD5940_REG_DFTIMAG		0x0000207C	/* DFT Result, Imaginary Part. */
#define AD5940_REG_SINC2DAT		0x00002080	/* Supply Rejection Filter Result. */
#define AD5940_REG_TEMPSENSDAT		0x00002084	/* Temperature Sensor Result. */
#define AD5940_REG_STATSMEAN		0x000021C8	/* Statistics Mean Output. */
#define AD5940_REG_STATSVAR		0x000021C0	/* Variance Output. */

#define AD5940_REG_INTCSEL0		0x00003008
#define AD5940_REG_INTCSEL1		0x0000300C
#define AD5940_REG_INTCFLAG0		0x00003010
#define AD5940_REG_INTCPOL		0x00003000	/* Interrupt Polarity. */
#define AD5940_INTCPOL_MSK		BIT(0)
#define AD5940_INTCPOL_POS		FIELD_PREP(AD5940_INTCPOL_MSK, 1)
#define AD5940_INTCPOL_NEG		FIELD_PREP(AD5940_INTCPOL_MSK, 0)
#define AD5940_REG_INTCLR		0x00003004

#define AD5940_REG_GP0CON		0x00000000
#define AD5940_REG_GP0OEN		0x00000004

#define AD5940_REG_PMBW			0x000022F0	/* Power and Bandwidth Management. */
#define	AD5940_PMBW_SYSHS_MSK		BIT(0)
#define	AD5940_PMBW_HP			FIELD_PREP(AD5940_PMBW_SYSHS_MSK, 1)
#define	AD5940_PMBW_LP			FIELD_PREP(AD5940_PMBW_SYSHS_MSK, 0)
#define AD5940_PMBW_SYSBW_MSK		GENMASK(3, 2)
#define	AD5940_PMBW_BWNA		FIELD_PREP(AD5940_PMBW_SYSBW_MSK, 0)
#define	AD5940_PMBW_BW50		FIELD_PREP(AD5940_PMBW_SYSBW_MSK, 1)
#define	AD5940_PMBW_BW100		FIELD_PREP(AD5940_PMBW_SYSBW_MSK, 2)
#define	AD5940_PMBW_BW250		FIELD_PREP(AD5940_PMBW_SYSBW_MSK, 3)

#define AD5940_REG_ADCCON		0x000021A8
#define AD5940_ADCCON_PGA_MSK		GENMASK(18, 16)
#define AD5940_ADCCON_PGA_1		FIELD_PREP(AD5940_ADCCON_PGA_MSK, 0)
#define AD5940_ADCCON_PGA_1P5		FIELD_PREP(AD5940_ADCCON_PGA_MSK, 1)
#define AD5940_ADCCON_PGA_2		FIELD_PREP(AD5940_ADCCON_PGA_MSK, 2)
#define AD5940_ADCCON_PGA_4		FIELD_PREP(AD5940_ADCCON_PGA_MSK, 3)
#define AD5940_ADCCON_PGA_9		FIELD_PREP(AD5940_ADCCON_PGA_MSK, 4)
#define AD5940_ADCCON_MUX_MSK		(GENMASK(12, 8) | GENMASK(5, 0))

#define AD5940_AFECON_CHIPID		0x00000404            /*  AFECON Chip Identification */
#define AD5940_AFECON_CLKCON0		0x00000408            /*  AFECON Clock Divider Configuration */
#define AD5940_AFECON_CLKEN1		0x00000410            /*  AFECON Clock Gate Enable */
#define AD5940_AFECON_CLKSEL		0x00000414            /*  AFECON Clock Select */
#define AD5940_AFECON_CLKCON0KEY	0x00000420            /*  AFECON Enable Clock Division to 8Mhz,4Mhz and 2Mhz */
#define AD5940_AFECON_SWRSTCON		0x00000424            /*  AFECON Software Reset */
#define AD5940_AFECON_TRIGSEQ		0x00000430            /*  AFECON Trigger Sequence */

#define AD5940_CHIPID			0x5502

#define AD5940_GPIO_PULLUP		0x00000001

static int ad5940_set_addr(struct spi_device *spi, u16 addr)
{
	u8 tx_buf[3];

	tx_buf[0] = AD5940_SPICMD_SETADDR;
	tx_buf[1] = (addr >> 8) & 0xff;
	tx_buf[2] = addr & 0xff;

	return spi_write(spi, tx_buf, 3);
}

static int ad5940_read_reg(struct spi_device *spi, u16 addr, u32 *data)
{
	u8 tx_buf[2];
	u8 rx_buf[4];
	int rx_len;
	int ret;

	tx_buf[0] = AD5940_SPICMD_READREG;
	tx_buf[1] = 0;

	ret = ad5940_set_addr(spi, addr);
	if (ret < 0)
		return ret;

	if ((addr >= 0x1000) && (addr <= 0x3014))
		rx_len = 4;
	else
		rx_len = 2;

	ret = spi_write_then_read(spi, tx_buf, 2, rx_buf, rx_len);
	if (ret < 0)
		return ret;
	
	if (rx_len == 2)
		*data = (rx_buf[0] << 8) | rx_buf[1];
	else
		*data = (rx_buf[0] << 24) | (rx_buf[1] << 16) |
			(rx_buf[2] << 8) | rx_buf[3];

	dev_info(&spi->dev, "read %x got %x", addr, *data);
	return 0;
}

static int ad5940_write_reg(struct spi_device *spi, u16 addr, u32 data)
{
	u8 tx_buf[5];
	int tx_len;
	int ret;

	tx_buf[0] = AD5940_SPICMD_WRITEREG;
	if ((addr >= 0x1000) && (addr <= 0x3014)) {
		tx_buf[1] = (data >> 24) & 0xff;
		tx_buf[2] = (data >> 16) & 0xff;
		tx_buf[3] = (data >> 8) & 0xff;
		tx_buf[4] = data & 0xff;
		tx_len = 5;
	} else {
		tx_buf[1] = (data >> 8) & 0xff;
		tx_buf[2] = data & 0xff;
		tx_len = 3;
	}

	ret = ad5940_set_addr(spi, addr);
	if (ret < 0)
		return ret;

	dev_info(&spi->dev, "write %x %x", addr, data);
	return spi_write(spi, tx_buf, tx_len);
}

static int ad5940_write_reg_mask(struct spi_device *spi, u16 addr,
				 u32 mask, u32 data)
{
	u32 temp;
	int ret;

	ret = ad5940_read_reg(spi, addr, &temp);
	if (ret < 0)
		return ret;

	temp &= ~mask;
	temp |= data;

	return ad5940_write_reg(spi, addr, temp);
}
