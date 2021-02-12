/*
 * Copyright (c) 2020 IoT.bzh
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_PINMUX_RCAR_H_
#define ZEPHYR_DRIVERS_PINMUX_RCAR_H_
#include <soc.h>

void pinmux_rcar_set_pingroup(const struct rcar_pin *pins, size_t num_pins);

#endif /* ZEPHYR_DRIVERS_PINMUX_RCAR_H_ */
