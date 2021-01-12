/*
 * Copyright (c) 2021 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kinetis_lpit

#include <drivers/counter.h>
#include <fsl_lpit.h>

struct mcux_lpit_config {
	void (*irq_config_func)(const struct device *dev);
};

struct mcux_lpit_data {

};

static int mcux_lpit_start(const struct device *dev)
{
	/* TODO */

	return 0;
}

static int mcux_lpit_stop(const struct device *dev)
{
	/* TODO */

	return 0;
}

static int mcux_lpit_get_value(const struct device *dev, uint32_t *ticks)
{
	/* TODO */

	return 0;
}

static int mcux_lpit_set_top_value(const struct device *dev,
				    const struct counter_top_cfg *cfg)
{
	/* TODO */

	return 0;
}

static uint32_t mcux_lpit_get_pending_int(const struct device *dev)
{
	/* TODO */

	return 0;
}

static uint32_t mcux_lpit_get_top_value(const struct device *dev)
{
	/* TODO */

	return 0;
}

static uint32_t mcux_lpit_get_max_relative_alarm(const struct device *dev)
{
	/* TODO */

	return 0;
}

static void mcux_lpit_isr(const struct device *dev)
{

	/* TODO */
}

static int mcux_lpit_init(const struct device *dev)
{
	/* TODO */

	return 0;
}

static const struct counter_driver_api mcux_lpit_driver_api = {
	.start = mcux_lpit_start,
	.stop = mcux_lpit_stop,
	.get_value = mcux_lpit_get_value,
	.set_top_value = mcux_lpit_set_top_value,
	.get_pending_int = mcux_lpit_get_pending_int,
	.get_top_value = mcux_lpit_get_top_value,
	.get_max_relative_alarm = mcux_lpit_get_max_relative_alarm,
};

#define MCUX_LPIT_IRQ_CODE(n, idx)					\
	do {								\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, idx, irq),		\
			    DT_INST_IRQ_BY_IDX(n, idx, priority),	\
			    mcux_lpit_isr,				\
			    DEVICE_DT_INST_GET(n), 0);			\
		irq_enable(DT_INST_IRQ_BY_IDX(n, idx, irq));		\
	} while (0)

#define MCUX_LPIT_IRQ(n, idx) \
	COND_CODE_1(DT_INST_IRQ_HAS_IDX(n, idx),			\
		    (MCUX_LPIT_IRQ_CODE(n, idx)), ())

#define MCUX_LPIT_INIT(n)						\
	static void mcux_lpit_irq_config_##n(const struct device *dev); \
									\
	static struct mcux_lpit_data mcux_lpit_data_##n;		\
									\
	static const struct mcux_lpit_config mcux_lpit_config_##n = {	\
		.irq_config_func = mcux_lpit_irq_config_##n,		\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, &mcux_lpit_init,			\
			      device_pm_control_nop,			\
			      &mcux_lpit_data_##n,			\
			      &mcux_lpit_config_##n, POST_KERNEL,	\
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			      &mcux_lpit_driver_api);			\
									\
	static void mcux_lpit_irq_config_##n(const struct device *dev)	\
	{								\
		MCUX_LPIT_IRQ(n, 0);					\
		MCUX_LPIT_IRQ(n, 1);					\
		MCUX_LPIT_IRQ(n, 2);					\
		MCUX_LPIT_IRQ(n, 3);					\
	}

DT_INST_FOREACH_STATUS_OKAY(MCUX_LPIT_INIT)
