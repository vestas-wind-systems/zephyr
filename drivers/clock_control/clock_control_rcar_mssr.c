/*
 * Copyright (c) 2020 IoT.bzh
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rcar_cpg_mssr
#include <errno.h>
#include <soc.h>
#include <drivers/clock_control.h>
#include <drivers/clock_control/rcar_clock_control.h>
#include <dt-bindings/clock/renesas-cpg.h>

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
/* CAN-FD Clock Frequency Control Register */
#define CANFDCKCR 0x244

/* Clock stop bit */
#define CANFDCKCR_CKSTP BIT(8)

/* On H3,M3,E3 parent clock of CANFD has 800MHz rate */
#define CANFDCKCR_PARENT_CLK_RATE 800000000
#define CANFDCKCR_DIVIDER_MASK 0x1FF

#define S3D4_CLK_RATE 66600000

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

static int cpg_core_clock_endisable(const struct device *dev,
				    uint32_t module, uint32_t rate, bool enable)
{
	const struct rcar_mssr_config *config = DEV_CFG(dev);
	uint32_t divider;
	int ret;

	/* Only support CANFD core clock at the moment */
	if (module != CPG_CORE_CLK_CANFD)
		return -EINVAL;
	int key = irq_lock();

	uint32_t addr = config->base_address + CANFDCKCR;
	if (enable) {
		if (CANFDCKCR_PARENT_CLK_RATE % rate) {
			__ASSERT(true, "Can not generate %u from CANFD parent clock", rate);
			ret = -EINVAL;
			goto unlock;
		}
		divider = (CANFDCKCR_PARENT_CLK_RATE / rate) - 1;
		if (divider > CANFDCKCR_DIVIDER_MASK) {
			__ASSERT(true, "Can not genearate %u from CANFD parent clock", rate);
			ret = -EINVAL;
			goto unlock;
		}
		cpg_write(addr, divider);
	}
	else
		cpg_write(addr, CANFDCKCR_CKSTP);

unlock:
	irq_unlock(key);
	return 0;
}

static int cpg_rmstp_clock_endisable(const struct device *dev,
			     uint32_t module, bool enable)
{
	const struct rcar_mssr_config *config = DEV_CFG(dev);
	uint32_t reg = module / 100;
	uint32_t bit = module % 100;
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
	struct rcar_cpg_clk *clk = (struct rcar_cpg_clk *) sys;

	if (clk->domain == CPG_MOD)
		return cpg_rmstp_clock_endisable(dev, clk->module, true);

	if (clk->domain == CPG_CORE)
		return cpg_core_clock_endisable(dev, clk->module, clk->rate, true);

	return -EINVAL;
}

static int mssr_stop(const struct device *dev,
		     clock_control_subsys_t sys)
{
	struct rcar_cpg_clk *clk = (struct rcar_cpg_clk *) sys;

	if (clk->domain == CPG_MOD)
		return cpg_rmstp_clock_endisable(dev, clk->module, false);

	if (clk->domain == CPG_CORE)
		return cpg_core_clock_endisable(dev, clk->module, false, 0);

	return -EINVAL;
}

static int cpg_get_rate(const struct device *dev,
				 clock_control_subsys_t sys,
				 uint32_t *rate)
{
	const struct rcar_mssr_config *config = DEV_CFG(dev);
	struct rcar_cpg_clk *clk = (struct rcar_cpg_clk *) sys;
	uint32_t val;
	int ret = 0;

	if (clk->domain != CPG_CORE) {
		return -ENOTSUP;
	}

	switch (clk->module) {
	case CPG_CORE_CLK_CANFD:
		val = sys_read32(config->base_address + CANFDCKCR);
		if (val & CANFDCKCR_CKSTP)
			*rate = 0;
		else {
			val &= CANFDCKCR_DIVIDER_MASK;
			*rate = CANFDCKCR_PARENT_CLK_RATE / (val + 1);
		}
		break;
	case CPG_CORE_CLK_S3D4:
		*rate = S3D4_CLK_RATE;
		break;
	default:
		ret = -ENOTSUP;
	}

	return ret;
}

static int rcar_mssr_init(const struct device *dev)
{
	return 0;
}

static const struct clock_control_driver_api rcar_mssr_api = {
	.on = mssr_blocking_start,
	.off = mssr_stop,
	.get_rate = cpg_get_rate,
};

#define RCAR_MSSR_INIT(inst)						\
	static struct rcar_mssr_config rcar_mssr##inst##_config = {	\
		.base_address = DT_INST_REG_ADDR(inst)			\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst,					\
			    &rcar_mssr_init,				\
			    device_pm_control_nop,			\
			    NULL, &rcar_mssr##inst##_config,		\
			    PRE_KERNEL_1,				\
			    CONFIG_KERNEL_INIT_PRIORITY_OBJECTS,	\
			    &rcar_mssr_api);

DT_INST_FOREACH_STATUS_OKAY(RCAR_MSSR_INIT)
