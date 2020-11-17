/*
 * Copyright (c) 2020 IoT.bzh
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rcar_cpg_mssr
#include <errno.h>
#include <soc.h>
#include <drivers/clock_control.h>

struct rcar_mssr_config {
	uint32_t base_address;
};

#define DEV_CFG(dev)  ((struct rcar_mssr_config *)(dev->config))

static const uint16_t rmstpsr[] = {
	0x110, 0x114, 0x118, 0x11c, 0x120, 0x124, 0x128, 0x12c,
	0x980, 0x984, 0x988, 0x98c,
};

#define	RMSTPSR(i)	rmstpsr[i]

/*
 * Software Reset Register offsets
 */
static const uint16_t srcr[] = {
	0x0A0, 0x0A8, 0x0B0, 0x0B8, 0x0BC, 0x0C4, 0x1C8, 0x1CC,
	0x920, 0x924, 0x928, 0x92C,
};

#define	SRCR(i)		srcr[i]

/* Software Reset Clearing Register offsets */
#define	SRSTCLR(i)	(0x940 + (i) * 4)

#define CPGWPR 0x0900

static void cpg_write(uint32_t addr, uint32_t val)
{
	sys_write32(~val, CPGWPR);
	sys_write32(val, addr);
	/* Wait for at least one cycle of the RCLK clock (@ ca. 32 kHz) */
	k_sleep(K_USEC(35));
}

static void cpg_reset(const struct rcar_mssr_config *config,
		      uint32_t reg, uint32_t bit)
{
	uint32_t addr = config->base_address + SRCR(reg);
	cpg_write(addr, BIT(bit));

	addr = config->base_address + SRSTCLR(reg);
	cpg_write(addr, BIT(bit));
}

static int cpg_rmstp_clock_endisable(const struct device *dev,
			     clock_control_subsys_t sub_system, bool enable)
{
	const struct rcar_mssr_config *config = DEV_CFG(dev);
	uint32_t index = POINTER_TO_UINT(sub_system);
	uint32_t reg = index / 100;
	uint32_t bit = index % 100;
	uint32_t bitmask = BIT(bit);
	uint32_t reg_val;

	int key = irq_lock();
	reg_val = sys_read32(config->base_address + rmstpsr[reg]);
	if (enable)
		reg_val &= ~bitmask;
	else
		reg_val |= bitmask;

	sys_write32(reg_val, config->base_address + rmstpsr[reg]);
	if (!enable)
		cpg_reset(config, reg, bit);
	irq_unlock(key);

	return 0;
}

static int mssr_blocking_start(const struct device *dev,
			       clock_control_subsys_t sys)
{
	return cpg_rmstp_clock_endisable(dev, sys, true);
}

static int mssr_stop(const struct device *dev,
		     clock_control_subsys_t sys)
{
	return cpg_rmstp_clock_endisable(dev, sys, false);
}

static int rcar_mssr_init(const struct device *dev)
{
	return 0;
}

static const struct clock_control_driver_api rcar_mssr_api = {
	.on = mssr_blocking_start,
	.off = mssr_stop,
};

#define RCAR_MSSR_INIT(inst)						\
	static struct rcar_mssr_config rcar_mssr##inst##_config = {	\
		.base_address = DT_INST_REG_ADDR(inst)			\
	};								\
									\
	DEVICE_AND_API_INIT(rcar_mssr##inst, DT_INST_LABEL(inst),	\
			    &rcar_mssr_init,				\
			    NULL, &rcar_mssr##inst##_config,		\
			    PRE_KERNEL_1,				\
			    CONFIG_KERNEL_INIT_PRIORITY_OBJECTS,	\
			    &rcar_mssr_api);

DT_INST_FOREACH_STATUS_OKAY(RCAR_MSSR_INIT)
