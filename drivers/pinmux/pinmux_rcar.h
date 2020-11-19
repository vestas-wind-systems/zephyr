/*
 * Copyright (c) 2020 IoT.bzh
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_PINMUX_RCAR_H_
#define ZEPHYR_DRIVERS_PINMUX_RCAR_H_

struct device *dev;

void pinmux_rcar_set_gpsr(const struct device *dev,
			  uint8_t gpsr, uint8_t pos, bool peripheral);

void pinmux_rcar_set_ipsr(const struct device *dev,
			  uint8_t ipsr, uint8_t pos, uint32_t val);
#endif /* ZEPHYR_DRIVERS_PINMUX_RCAR_H_ */
