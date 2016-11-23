/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2013 Qtechnology/AS
 * ricardo.ribalda@gmail.com
 *
 */

#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>

#define DRIVER_NAME "spi-qtec-lpc"

#define QTEC_LPC_NUMCS 7
#define QTEC_LPC_BPW 8
#define QTEC_LPC_MODE SPI_MODE_3
#define QTEC_LPC_SPEED 33000000

#define QTEC_LPC_ID0_VAL 0x07
#define QTEC_LPC_ID0_MASK 0x37
#define QTEC_LPC_ID1_VAL 0xec

//Register definitions
#define QTEC_LPC_IOLEN 2
#define QTEC_LPC_REGA 0
#define QTEC_LPC_REGB 1
#define QTEC_LPC_CS QTEC_LPC_REGA
#define QTEC_LPC_RESET QTEC_LPC_REGA
#define QTEC_LPC_ID0 QTEC_LPC_REGA
#define QTEC_LPC_MOSI QTEC_LPC_REGB
#define QTEC_LPC_MISO QTEC_LPC_REGB
#define QTEC_LPC_ID1 QTEC_LPC_REGB

#define OFLOW BIT(7)
#define BUSY BIT(6)

struct qtec_lpc_spi {
	struct spi_master *spi_master;
	unsigned long portbase;
};

static void qtec_lpc_spi_write(struct qtec_lpc_spi *iospi, unsigned int reg, uint8_t value){
	dev_dbg(&iospi->spi_master->dev,"W %lX %.2X",iospi->portbase+reg,value);
	outb(value,iospi->portbase+reg);
}

static uint8_t qtec_lpc_spi_read(struct qtec_lpc_spi *iospi, unsigned int reg){
	uint8_t ret;
	ret=inb(iospi->portbase+reg);
	dev_dbg(&iospi->spi_master->dev,"R %lX %.2X",iospi->portbase+reg,ret);

	return ret;
}

static int qtec_lpc_spi_setup(struct spi_device *spi)
{
	if (spi->bits_per_word!=QTEC_LPC_BPW){
		dev_err(&spi->dev, "Invalid bpw %d. Only %d supported\n",spi->bits_per_word,QTEC_LPC_BPW);
		return -EINVAL;
	}

	/* Warning Mode should be set in the bitstream!*/
	if (spi->chip_select>=QTEC_LPC_NUMCS){
		dev_err(&spi->dev, "Invalid chip select %d. Only up to %d supported\n",spi->chip_select,QTEC_LPC_NUMCS);
		return -EINVAL;
	}

	if (spi->max_speed_hz<QTEC_LPC_SPEED){
		dev_err(&spi->dev, "Invalid max speed %d. SPI speed is fixed to %d\n",spi->max_speed_hz,QTEC_LPC_SPEED);
		return -EINVAL;
	}

	return 0;
}

static int qtec_lpc_spi_transfer_one_message(struct spi_master *master,struct spi_message *m){
	struct qtec_lpc_spi *qtec_lpc_spi=spi_master_get_devdata(master);
	int cs;
	struct spi_transfer *t, *tmp_t;
	bool cs_enabled = false;

	cs = m->spi->chip_select;
	m->actual_length=0;
	list_for_each_entry_safe(t, tmp_t, &m->transfers, transfer_list) {
		unsigned int p=0;
		const uint8_t *tx=t->tx_buf;
		uint8_t *rx=t->rx_buf;

		//set cs
		if (!cs_enabled){
			qtec_lpc_spi_write(qtec_lpc_spi,QTEC_LPC_CS,~((1<<(cs+1))|1));
			cs_enabled = true;
		}

		while (p<t->len){
			if (tx)
				qtec_lpc_spi_write(qtec_lpc_spi,QTEC_LPC_MOSI,tx[p]);
			else
				qtec_lpc_spi_write(qtec_lpc_spi,QTEC_LPC_MOSI,0);

			while (qtec_lpc_spi_read(qtec_lpc_spi,QTEC_LPC_REGA) & BUSY);

			if (rx)
				rx[p]=qtec_lpc_spi_read(qtec_lpc_spi,QTEC_LPC_MOSI);
			p++;
		}
		m->actual_length+=t->len;

		if (t->cs_change){
			qtec_lpc_spi_write(qtec_lpc_spi,QTEC_LPC_CS,0xfe);
			cs_enabled = false;
		}

	}
	//clear cs
	if (cs_enabled)
		qtec_lpc_spi_write(qtec_lpc_spi,QTEC_LPC_CS,0xfe);

	//Send 8 clks after cs
	qtec_lpc_spi_write(qtec_lpc_spi,QTEC_LPC_MOSI,0);

	if (qtec_lpc_spi_read(qtec_lpc_spi,QTEC_LPC_REGA) & OFLOW){
		dev_err(&master->dev, "Transfer overflow! Please increase the SPI core clock speed! Transfer failed!\n");
		qtec_lpc_spi_write(qtec_lpc_spi,QTEC_LPC_RESET,0xff);
		qtec_lpc_spi_write(qtec_lpc_spi,QTEC_LPC_RESET,0xfe);
	}

	m->status=0;
	spi_finalize_current_message(master);

	return 0;
}

static int qtec_lpc_spi_probe(struct platform_device *pdev){
	struct resource *res_port,*res_bus;
	struct spi_master *master;
	struct qtec_lpc_spi *p;
	int err;

	master = spi_alloc_master(&pdev->dev, sizeof(struct qtec_lpc_spi));
	if (!master)
		return -ENOMEM;
	p = spi_master_get_devdata(master);
	platform_set_drvdata(pdev, p);

	//Probe the hw
	res_port = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (!res_port){
		dev_err(&pdev->dev, "found no io resource\n");
		err= -ENXIO;
		goto fail;
	}
	res_bus = platform_get_resource(pdev, IORESOURCE_BUS, 0);
	if (!res_bus)
		master->bus_num = -1;
	else
		master->bus_num = res_bus->start;

	//Request io resource
	if (!devm_request_region(&pdev->dev,res_port->start,QTEC_LPC_IOLEN,DRIVER_NAME)){
		dev_err(&pdev->dev, "Unable to request io region 0x%lx (%d)\n",(long)res_port->start,QTEC_LPC_IOLEN);
		err= -EBUSY;
		goto fail;
	}

	p->portbase=res_port->start;
	p->spi_master=master;

	master->num_chipselect = QTEC_LPC_NUMCS;
	master->mode_bits = SPI_CPHA |
			    SPI_CPOL |
			    SPI_CS_HIGH |
			    SPI_LSB_FIRST |
			    SPI_3WIRE;

	master->setup = qtec_lpc_spi_setup;
	master->transfer_one_message = qtec_lpc_spi_transfer_one_message;

	//Probe
	if ((qtec_lpc_spi_read(p,QTEC_LPC_ID0) & QTEC_LPC_ID0_MASK) != QTEC_LPC_ID0_VAL){
		dev_err(&pdev->dev, "Invalid device id0 0x%.2x\n",qtec_lpc_spi_read(p,QTEC_LPC_ID0));
		err=-EIO;
		goto fail;
	}

	//Reset device
	qtec_lpc_spi_write(p,QTEC_LPC_RESET,0xff);
	qtec_lpc_spi_write(p,QTEC_LPC_RESET,0xfe);
	if (QTEC_LPC_ID1_VAL!= qtec_lpc_spi_read(p,QTEC_LPC_ID1)){
		dev_err(&pdev->dev, "Invalid device id1 0x%.2x\n",qtec_lpc_spi_read(p,QTEC_LPC_ID1));
		err=-EIO;
		goto fail;
	}

	err = spi_register_master(master);
	if (err) {
		dev_err(&pdev->dev, "register master failed: %d\n", err);
		goto fail;
	}

	dev_info(&pdev->dev, "QTEC_LPC SPI driver at 0x%lx\n",p->portbase);

	return 0;
fail:
	spi_master_put(master);
	return err;

}

static int qtec_lpc_spi_remove(struct platform_device *pdev){
	struct qtec_lpc_spi *p = platform_get_drvdata(pdev);

	//Setting the core on reset
	qtec_lpc_spi_write(p,QTEC_LPC_RESET,0xff);
	spi_unregister_master(p->spi_master);

	return 0;
}

static struct platform_driver qtec_lpc_spi_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
	},
	.probe		= qtec_lpc_spi_probe,
	.remove		= qtec_lpc_spi_remove,
};

module_platform_driver(qtec_lpc_spi_driver);

static struct platform_device_id qtec_lpc_spi_ids[] = {
	{
		.name           = DRIVER_NAME,
	},
	{ }
};

MODULE_DEVICE_TABLE(platform, qtec_lpc_spi_ids);

MODULE_DESCRIPTION("Qtec LCP SPI bus driver");
MODULE_AUTHOR("Ricardo Ribalda");
MODULE_LICENSE("GPL");
