/* Copyright 2021 IoT.bzh
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <soc.h>
#include <pinmux/pinmux_rcar.h>

static int rcar_ebisu_pinmux_init(const struct device *dev)
{
	 ARG_UNUSED(dev);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(can0), okay) && CONFIG_CAN
	 pinmux_rcar_set_pingroup(can0_data, ARRAY_SIZE(can0_data));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(scif2), okay) && CONFIG_SERIAL
	/* SCIF2 = UBOOT OUTPUT */
	 pinmux_rcar_set_pingroup(scif2_data_a, ARRAY_SIZE(scif2_data_a));
#endif

	 return 0;
}

SYS_INIT(rcar_ebisu_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
