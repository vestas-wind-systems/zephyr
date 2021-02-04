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

static const uint32_t rmstpsr[] = {
	0x110, 0x114, 0x118, 0x11c, 0x120, 0x124, 0x128, 0x12c,
	0x980, 0x984, 0x988, 0x98c,
};

#define	RMSTPSR(i)	rmstpsr[i]

static int cpg_rmstp_clock_endisable(const struct device *dev,
			     clock_control_subsys_t sub_system, bool enable)
{
	struct rcar_mssr_config *config = DEV_CFG(dev);
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
