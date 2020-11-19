/* Copyright 2020 IoT.bzh
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <drivers/pinmux.h>
#include <pinmux/pinmux_rcar.h>

 static int rcar_ebisu_pinmux_init(const struct device *dev)
 {
	 ARG_UNUSED(dev);

	 const struct device *pfc =
		 device_get_binding(DT_LABEL(DT_NODELABEL(pfc)));
	 __ASSERT(pfc, "Fail to get pincontrol device");

#if DT_NODE_HAS_STATUS(DT_NODELABEL(can0), okay) && CONFIG_CAN
	 /* CAN0_TX */
	 pinmux_rcar_set_gpsr(pfc, 0, 12, true);
	 pinmux_rcar_set_ipsr(pfc, 7, 4, 3);

	 /* CAN0_RX */
	 pinmux_rcar_set_gpsr(pfc, 0, 13, true);
	 pinmux_rcar_set_ipsr(pfc, 7, 8, 3);
#endif
	 return 0;
 }

SYS_INIT(rcar_ebisu_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
