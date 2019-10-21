/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/adc.h>
#include <stdio.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(main);

static double lm94022_millivolt_to_celcius(double millivolt)
{
	/*
	 * Approximate transfer function @ 20 degC according to
	 * LM94022 datasheet:
	 *
	 * V = (-5.5 mV/degC) * T + 1035 mV
	 */

	return (millivolt - 1035.0) / -5.5;
}

void main(void)
{
	struct device *lmp90100;
	double vcold, vhot;
	double tcold, thot;
	s32_t buffer[2];
	int err;
	int i;
	const struct adc_channel_cfg ch_cfg[2] = {
		{
			.channel_id = 0,
			.differential = 1,
			.input_positive = 5,
			.input_negative = 7,
			.reference = ADC_REF_EXTERNAL0,
			.gain = ADC_GAIN_1,
			.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		},
		{
			.channel_id = 1,
			.differential = 1,
			.input_positive = 4,
			.input_negative = 3,
			.reference = ADC_REF_EXTERNAL0,
			.gain = ADC_GAIN_1,
			.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		}
	};
	const struct adc_sequence seq = {
		.options = NULL,
		.channels = BIT(1) | BIT(0),
		.buffer = buffer,
		.buffer_size = sizeof(buffer),
		.resolution = 24,
		.oversampling = 0,
		.calibrate = 0
	};

	lmp90100 = device_get_binding(DT_INST_0_TI_LMP90100_LABEL);
	if (!lmp90100) {
		LOG_ERR("LMP90100 device not found");
		return;
	}

	for (i = 0; i < ARRAY_SIZE(ch_cfg); i++) {
		err = adc_channel_setup(lmp90100, &ch_cfg[i]);
		if (err) {
			LOG_ERR("failed to setup ADC channel %d (err %d)", i,
				err);
			return;
		}
	}

	while (true) {
		err = adc_read(lmp90100, &seq);
		if (err) {
			LOG_ERR("failed to read ADC (err %d)", err);
			return;
		}

		LOG_INF("ADC channel 0 = %d (0x%08x)", buffer[0], buffer[0]);
		LOG_INF("ADC channel 1 = %d (0x%08x)", buffer[1], buffer[1]);

		vcold = (buffer[0] / 8388608.0) * 4100 * 1.0;
		printf("Vcold = %.02f mV\n", vcold);

		tcold = lm94022_millivolt_to_celcius(vcold);
		printf("Tcold = %.02f degC\n", tcold);

		vhot = (buffer[1] / 8388608.0) * 4100 * 1.0;
		printf("Vhot = %.02f mV\n", vhot);

		k_sleep(1000);
	}
}
