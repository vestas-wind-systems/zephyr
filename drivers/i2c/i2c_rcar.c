/*
 * Copyright (c) 2021 IoT.bzh
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rcar_gen3_i2c

#include <errno.h>
#include <device.h>
#include <devicetree.h>
#include <soc.h>
#include <drivers/i2c.h>
#include <drivers/clock_control.h>
#include <drivers/clock_control/rcar_clock_control.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(i2c_rcar);
#include "i2c-priv.h"

struct i2c_rcar_cfg {
	uint32_t reg_addr;
	char *clock_controller;
	struct rcar_cpg_clk mod_clk;
	struct rcar_cpg_clk bus_clk;
	uint32_t bitrate;
};

struct i2c_rcar_data {
	uint32_t clk_rate;
};

/* Registers */
#define RCAR_I2C_ICSCR			0x00 /* slave ctrl */
#define RCAR_I2C_ICMCR			0x04 /* master ctrl */
#define RCAR_I2C_ICMCR_MDBS		BIT(7) /* non-fifo mode switch */
#define RCAR_I2C_ICMCR_FSCL		BIT(6) /* override SCL pin */
#define RCAR_I2C_ICMCR_FSDA		BIT(5) /* override SDA pin */
#define RCAR_I2C_ICMCR_OBPC		BIT(4) /* override pins */
#define RCAR_I2C_ICMCR_MIE		BIT(3) /* master if enable */
#define RCAR_I2C_ICMCR_TSBE		BIT(2)
#define RCAR_I2C_ICMCR_FSB		BIT(1) /* force stop bit */
#define RCAR_I2C_ICMCR_ESG		BIT(0) /* enable start bit gen */
#define RCAR_I2C_ICSSR			0x08 /* slave status */
#define RCAR_I2C_ICMSR			0x0c /* master status */
#define RCAR_I2C_ICMSR_MASK		0x7f
#define RCAR_I2C_ICMSR_MNR		BIT(6) /* Nack */
#define RCAR_I2C_ICMSR_MAL		BIT(5) /* Arbitration lost */
#define RCAR_I2C_ICMSR_MST		BIT(4) /* Stop */
#define RCAR_I2C_ICMSR_MDE		BIT(3)
#define RCAR_I2C_ICMSR_MDT		BIT(2)
#define RCAR_I2C_ICMSR_MDR		BIT(1)
#define RCAR_I2C_ICMSR_MAT		BIT(0)
#define RCAR_I2C_ICSIER		0x10 /* slave irq enable */
#define RCAR_I2C_ICMIER		0x14 /* master irq enable */
#define RCAR_I2C_ICCCR			0x18 /* clock dividers */
#define RCAR_I2C_ICCCR_SCGD_OFF	3
#define RCAR_I2C_ICSAR			0x1c /* slave address */
#define RCAR_I2C_ICMAR			0x20 /* master address */
#define RCAR_I2C_ICRXD_ICTXD		0x24 /* data port */
#define RCAR_I2C_ICFBSCR		0x38 /*First Bit Setup Cycle (Gen3).*/
#define RCAR_I2C_ICFBSCR_TCYC17	0x0f /* 17*Tcyc */

/* Recommended settings from official documentation */
#define RCAR_I2C_ICCCR_CDF_100_KHZ	6
#define RCAR_I2C_ICCCR_CDF_400_KHZ	6
#define RCAR_I2C_ICCCR_SCGD_100_KHZ	21
#define RCAR_I2C_ICCCR_SCGD_400_KHZ	3

/* Helper macros for I2C */
#define DEV_I2C_CFG(dev) \
	((const struct i2c_rcar_cfg *)(dev)->config)
#define DEV_I2C_DATA(dev) \
	((struct i2c_rcar_data *)(dev)->data)

static uint32_t i2c_rcar_read(const struct i2c_rcar_cfg *config,
				  uint32_t offs)
{
	return sys_read32(config->reg_addr + offs);
}

static void i2c_rcar_write(const struct i2c_rcar_cfg *config,
			       uint32_t offs, uint32_t value)
{
	sys_write32(value, config->reg_addr + offs);
}

static int rcar_i2c_finish(const struct device *dev)
{
	const struct i2c_rcar_cfg *config = DEV_I2C_CFG(dev);
	uint16_t timeout = 0;

	/* Wait for the end of the transmission*/
	while (!(i2c_rcar_read(config, RCAR_I2C_ICMSR) & RCAR_I2C_ICMSR_MST) && (timeout<10)){
		k_busy_wait(USEC_PER_MSEC);
		timeout++;
	}
	if (timeout == 10) {
		return -ETIMEDOUT;
	}

	i2c_rcar_write(config, RCAR_I2C_ICSSR, 0);
	i2c_rcar_write(config, RCAR_I2C_ICMSR, 0);
	i2c_rcar_write(config, RCAR_I2C_ICMCR, 0);

	return 0;
}

static int i2c_rcar_set_addr(const struct device *dev,
                uint8_t chip, uint8_t read)
{
	const struct i2c_rcar_cfg *config = DEV_I2C_CFG(dev);
	/* mask to define*/
	uint32_t mask = RCAR_I2C_ICMSR_MAT |
		   (read ? RCAR_I2C_ICMSR_MDR : RCAR_I2C_ICMSR_MDE);
	uint16_t timeout = 0;

	i2c_rcar_write(config, RCAR_I2C_ICMIER, 0);
	i2c_rcar_write(config, RCAR_I2C_ICMCR, RCAR_I2C_ICMCR_MDBS);
	i2c_rcar_write(config, RCAR_I2C_ICMSR, 0);

	/* Wait for the bus to be available */
	while ((i2c_rcar_read(config, RCAR_I2C_ICMCR) & RCAR_I2C_ICMCR_FSDA) && (timeout<2)){
		k_busy_wait(USEC_PER_MSEC);
		timeout++;
	}
	if (timeout == 2) {
		return -ETIMEDOUT;
	}

	i2c_rcar_write(config, RCAR_I2C_ICMAR, (chip << 1) | read);
	/* Reset */
	i2c_rcar_write(config, RCAR_I2C_ICMCR, RCAR_I2C_ICMCR_MDBS | RCAR_I2C_ICMCR_MIE | RCAR_I2C_ICMCR_ESG);
	/* Clear Status */
	i2c_rcar_write(config, RCAR_I2C_ICMSR, 0);

	timeout = 0;
	while (!(i2c_rcar_read(config, RCAR_I2C_ICMSR) & mask) && (timeout<100)){
		k_busy_wait(USEC_PER_MSEC);
		timeout++;
	}
	if (timeout == 100) {
		return -ETIMEDOUT;
	}

	/* Check NAK */
	if(i2c_rcar_read(config, RCAR_I2C_ICMSR) & RCAR_I2C_ICMSR_MNR)
		return -EIO;

	return 0;
}

static int i2c_rcar_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_rcar_cfg *config = DEV_I2C_CFG(dev);
	uint8_t cdf, scgd;

	if (!(I2C_MODE_MASTER & dev_config)) {
		return -EINVAL;
	}

	if (I2C_ADDR_10_BITS & dev_config) {
		return -EINVAL;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		/* Setting ICCCR to recommended value for 100 kHz bus */
		cdf = RCAR_I2C_ICCCR_CDF_100_KHZ;
		scgd = RCAR_I2C_ICCCR_SCGD_100_KHZ;
		break;
	case I2C_SPEED_FAST:
		/* Setting ICCCR to recommended value for 400 kHz bus */
		cdf = RCAR_I2C_ICCCR_CDF_400_KHZ;
		scgd = RCAR_I2C_ICCCR_SCGD_400_KHZ;
		break;
	default:
		return -EINVAL;
	}

	i2c_rcar_write(config, RCAR_I2C_ICCCR, (scgd << RCAR_I2C_ICCCR_SCGD_OFF) | cdf);

	/* reset slave mode */
	i2c_rcar_write(config, RCAR_I2C_ICSIER, 0);
	i2c_rcar_write(config, RCAR_I2C_ICSAR, 0);
	i2c_rcar_write(config, RCAR_I2C_ICSCR, 0);
	i2c_rcar_write(config, RCAR_I2C_ICSSR, 0);

	/* reset master mode */
	i2c_rcar_write(config, RCAR_I2C_ICMIER, 0);
	i2c_rcar_write(config, RCAR_I2C_ICMCR, 0);
	i2c_rcar_write(config, RCAR_I2C_ICMSR, 0);
	i2c_rcar_write(config, RCAR_I2C_ICMAR, 0);

	return 0;
}

static int i2c_rcar_transfer_msg(const struct device *dev, struct i2c_msg *msg){
	const struct i2c_rcar_cfg *config = DEV_I2C_CFG(dev);
	uint32_t icmcr = RCAR_I2C_ICMCR_MDBS | RCAR_I2C_ICMCR_MIE;
	uint32_t i;
	uint16_t timeout = 0;

	if ((msg->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ){
		for (i = 0; i < msg->len; i++) {
			if (msg->len - 1 == i) {
				icmcr |= RCAR_I2C_ICMCR_FSB;
			}

			i2c_rcar_write(config, RCAR_I2C_ICMCR, icmcr);
			i2c_rcar_write(config, RCAR_I2C_ICMSR, (uint32_t)~RCAR_I2C_ICMSR_MDR);

			/* Wait for a data to be received*/
			while (!(i2c_rcar_read(config, RCAR_I2C_ICMSR) & RCAR_I2C_ICMSR_MDR) && (timeout<100)){
				k_busy_wait(USEC_PER_MSEC);
				timeout++;
			}
			if (timeout == 100) {
				return -ETIMEDOUT;
			}

			msg->buf[i] = i2c_rcar_read(config, RCAR_I2C_ICRXD_ICTXD) & 0xff;
		}

		i2c_rcar_write(config, RCAR_I2C_ICMSR, (uint32_t)~RCAR_I2C_ICMSR_MDR);
	}
	else {
		for (i = 0; i < msg->len; i++) {
			i2c_rcar_write(config, RCAR_I2C_ICRXD_ICTXD, msg->buf[i]);
			i2c_rcar_write(config, RCAR_I2C_ICMCR, icmcr);
			i2c_rcar_write(config, RCAR_I2C_ICMSR, (uint32_t)~RCAR_I2C_ICMSR_MDE);

			while (!(i2c_rcar_read(config, RCAR_I2C_ICMSR) & RCAR_I2C_ICMSR_MDE) && (timeout<100)){
				k_busy_wait(USEC_PER_MSEC);
				timeout++;
			}
			if (timeout == 100) {
				return -ETIMEDOUT;
			}
		}

		i2c_rcar_write(config, RCAR_I2C_ICMSR, (uint32_t)~RCAR_I2C_ICMSR_MDE);
		icmcr |= RCAR_I2C_ICMCR_FSB;
		i2c_rcar_write(config, RCAR_I2C_ICMCR, icmcr);
	}

	return rcar_i2c_finish(dev);
}

static int i2c_rcar_transfer(const struct device *dev,
			   struct i2c_msg *msgs, uint8_t num_msgs,
			   uint16_t addr)
{
	const struct i2c_rcar_cfg *config = DEV_I2C_CFG(dev);
	uint16_t timeout = 0;
	int ret;

	if (!num_msgs) {
		return 0;
	}

	/* Wait for the bus to be available */
	while ((i2c_rcar_read(config, RCAR_I2C_ICMCR) & RCAR_I2C_ICMCR_FSDA) && (timeout<10)){
		k_busy_wait(USEC_PER_MSEC);
		timeout++;
	}
	if (timeout == 10) {
		return -ETIMEDOUT;
	}

	do {
		/* Send address after any Start condition */
		if (i2c_rcar_set_addr(dev, addr, !!(msgs->flags & I2C_MSG_READ))) {
			return -EIO; /* No ACK received */
		}

		/* Transfer data */
		if (msgs->len) {
			ret = i2c_rcar_transfer_msg(dev, msgs);
			if (ret) {
				return ret;
			}
		}

		/* Next message */
		msgs++;
		num_msgs--;
	} while (num_msgs);

	/* Complete without error */
	return 0;
}

static int i2c_rcar_init(const struct device *dev)
{
	const struct i2c_rcar_cfg *config = DEV_I2C_CFG(dev);
	struct i2c_rcar_data *data = DEV_I2C_DATA(dev);
	const struct device *clk;
	uint32_t bitrate_cfg;
	int ret;

	if (config->clock_controller) {
		clk = device_get_binding(config->clock_controller);
		if (!clk) return -ENODEV;


		ret = clock_control_on(clk,
				(clock_control_subsys_t *) &config->mod_clk);

		if (ret < 0) {
			return ret;
		}


		ret = clock_control_get_rate(clk,
				(clock_control_subsys_t *) &config->bus_clk,
				&data->clk_rate);

		if (ret < 0) {
			return ret;
		}
	}

	bitrate_cfg = i2c_map_dt_bitrate(config->bitrate);

	return i2c_rcar_configure(dev, I2C_MODE_MASTER | bitrate_cfg);
}

static const struct i2c_driver_api i2c_rcar_driver_api = {
	.configure = i2c_rcar_configure,
	.transfer = i2c_rcar_transfer,
};

/* Device Instantiation */
#define I2C_RCAR_INIT(n)						      \
	static const struct i2c_rcar_cfg i2c_rcar_cfg_##n = {		      \
		.reg_addr = DT_INST_REG_ADDR(n),			      \
		.clock_controller = DT_INST_CLOCKS_LABEL(n),		      \
		.bitrate = DT_INST_PROP(n, clock_frequency),                  \
		.mod_clk.module =					      \
			DT_INST_CLOCKS_CELL_BY_IDX(n, 0, module),	      \
		.mod_clk.domain =					      \
			DT_INST_CLOCKS_CELL_BY_IDX(n, 0, domain),	      \
		.bus_clk.module =					      \
			DT_INST_CLOCKS_CELL_BY_IDX(n, 1, module),	      \
		.bus_clk.domain =					      \
			DT_INST_CLOCKS_CELL_BY_IDX(n, 1, domain),	      \
	};								      \
									      \
	static struct i2c_rcar_data i2c_rcar_data_##n;		              \
								              \
	DEVICE_DT_INST_DEFINE(n,					      \
			    i2c_rcar_init,				      \
			    device_pm_control_nop,			      \
			    &i2c_rcar_data_##n,			              \
			    &i2c_rcar_cfg_##n,				      \
			    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,  \
			    &i2c_rcar_driver_api			      \
			    );						      \

DT_INST_FOREACH_STATUS_OKAY(I2C_RCAR_INIT)
