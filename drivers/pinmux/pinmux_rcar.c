/*
 * Copyright (c) 2020 IoT.bzh
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rcar_pinmux

#include <errno.h>
#include <device.h>
#include <drivers/pinmux.h>
#include <soc.h>

#define PINMUX_RCAR_PMMR 0x0
#define PINMUX_RCAR_GPSR 0x100
#define PINMUX_RCAR_IPSR 0x200

struct pinmux_rcar_config {
	uint32_t reg_addr;
};

/* Any write to IPSR or GPSR must be precede to a write to PMMR with 
 * the inverse value.
 */
static void pinmux_rcar_write(const struct pinmux_rcar_config *config,
			      uint32_t offs, uint32_t val)
{
	sys_write32(~val, config->reg_addr + PINMUX_RCAR_PMMR);
	sys_write32(val, config->reg_addr + offs);
}

/* Set the pin either in gpio or peripheral */
void pinmux_rcar_set_gpsr(const struct device *dev,
		       uint8_t gpsr, uint8_t pos, bool peripheral)
{
	const struct pinmux_rcar_config *config = dev->config;
	uint32_t val = sys_read32(config->reg_addr + PINMUX_RCAR_GPSR +
				  gpsr * sizeof(uint32_t));
	if (peripheral)
		val |= BIT(pos);
	else
		val &= ~BIT(pos);
	pinmux_rcar_write(config, PINMUX_RCAR_GPSR + gpsr * sizeof(uint32_t),
			  val);
}

void pinmux_rcar_set_ipsr(const struct device *dev,
				 uint8_t ipsr, uint8_t pos, uint32_t val)
{
	const struct pinmux_rcar_config *config = dev->config;
	uint32_t reg = sys_read32(config->reg_addr + PINMUX_RCAR_IPSR +
				  ipsr * sizeof(uint32_t));
	reg &= ~(0xF << pos);
	reg |= (val << pos);
	pinmux_rcar_write(config, PINMUX_RCAR_IPSR + ipsr * sizeof(uint32_t),
			  reg);
}
	
static int pinmux_rcar_set(const struct device *dev, uint32_t pin,
			     uint32_t func)
{
	return -ENOTSUP;
}

static int pinmux_rcar_get(const struct device *dev, uint32_t pin,
			     uint32_t *func)
{
	return -ENOTSUP;
}
	
static int pinmux_rcar_pullup(const struct device *dev, uint32_t pin,
				uint8_t func)
{
	return -ENOTSUP;
}

static int pinmux_rcar_input(const struct device *dev, uint32_t pin,
			       uint8_t func)
{
	return -ENOTSUP;
}

static int pinmux_rcar_init(const struct device *dev)
{
	return 0;
}

static const struct pinmux_driver_api pinmux_rcar_driver_api = {
	.set = pinmux_rcar_set,
	.get = pinmux_rcar_get,
	.pullup = pinmux_rcar_pullup,
	.input = pinmux_rcar_input,
};

static const struct pinmux_rcar_config pinmux_rcar_0_config = { 
		.reg_addr = DT_INST_REG_ADDR(0),	       
};							       

DEVICE_AND_API_INIT(pinmux_rcar, DT_INST_LABEL(0),
			    &pinmux_rcar_init,
			    NULL, &pinmux_rcar_0_config,
			    PRE_KERNEL_1,
			    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
			    &pinmux_rcar_driver_api);
