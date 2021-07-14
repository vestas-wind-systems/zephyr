/*
 * Copyright (c) 2021 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT lm77

#include <device.h>
#include <drivers/sensor.h>
#include <drivers/i2c.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(lm77, CONFIG_SENSOR_LOG_LEVEL);

/* LM77 registers */
#define LM77_REG_TEMP   0x00U
#define LM77_REG_CONFIG 0x01U
#define LM77_REG_THYST  0x02U
#define LM77_REG_TCRIT  0x03U
#define LM77_REG_TLOW   0x04U
#define LM77_REG_THIGH  0x05U

struct lm77_config {
	const struct device *i2c;
	uint16_t addr;
};

struct lm77_data {
	const struct sensor_trigger *trigger;
	sensor_trigger_handler_t handler;
	uint16_t temp;
};

static int lm77_attr_set(const struct device *dev, enum sensor_channel chan,
			 enum sensor_attribute attr, const struct sensor_value *val)
{
	/* TODO: SENSOR_ATTR_LOWER_THRESH, SENSOR_ATTR_UPPER_THRESH, THYST?, TCRIT? */
}

static int lm77_attr_get(const struct device *dev, enum sensor_channel chan,
			 enum sensor_attribute attr, struct sensor_value *val)
{
	/* TODO: SENSOR_ATTR_LOWER_THRESH, SENSOR_ATTR_UPPER_THRESH, THYST?, TCRIT? */
}

static int lm77_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			    sensor_trigger_handler_t handler)
{
	/* TODO: SENSOR_TRIG_THRESHOLD */
}

static int lm77_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct lm77_config *config = dev->config;
	struct lm77_data *data = dev->data;
	int err;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	/* TODO: read temperature */

	return 0;
}

static int lm77_channel_get(const struct device *dev, enum sensor_channel chan,
			    struct sensor_value *val)
{
	const struct lm77_config *config = dev->config;
	struct lm77_data *data = dev->data;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	/* TODO: fill in value */

	return 0;
}

static const struct sensor_driver_api lm77_driver_api = {
	.attr_set = lm77_attr_set,
	.attr_get = lm77_attr_get,
	.trigger_set = lm77_trigger_set,
	.sample_fetch = lm77_sample_fetch,
	.channel_get = lm77_channel_get,
};

static int lm77_init(const struct device *dev)
{
	const struct lm77_config *config = dev->config;
	struct lm77_data *data = dev->data;
	int err;
	int i;

	if (!device_is_ready(config->i2c)) {
		LOG_ERR("I2c bus not ready");
		return -EINVAL;
	}

	/* TODO: support INT gpio */

	return 0;
}

/* TODO: shutdown support? */
#define LM77_INIT(n)							\
	static struct lm77_data lm77_data_##n;				\
									\
	static const struct lm77_config lm77_config_##n = {		\
		.i2c = DEVICE_DT_GET(DT_BUS(DT_DRV_INST(n))),		\
		.addr = DT_INST_REG_ADDR(n),				\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst, lm77_init,				\
			      device_pm_control_nop,			\
			      &lm77_data_##n,				\
			      &lm77_config_##n, POST_KERNEL,		\
			      CONFIG_SENSOR_INIT_PRIORITY,		\
			      &lm77_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LM77_INIT)
