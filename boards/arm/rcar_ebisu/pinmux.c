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
	 /* Make compiler happy if pfc is not used */
	 (void) pfc;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(can0), okay) && CONFIG_CAN
	 /* CAN0_TX */
	 pinmux_rcar_set_gpsr(pfc, 0, 12, true);
	 pinmux_rcar_set_ipsr(pfc, 7, 4, 3);

	 /* CAN0_RX */
	 pinmux_rcar_set_gpsr(pfc, 0, 13, true);
	 pinmux_rcar_set_ipsr(pfc, 7, 8, 3);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(scif2), okay) && CONFIG_SERIAL
	/* SCIF2 = UBOOT OUTPUT */
	/* SCIF2_RX */
	pinmux_rcar_set_gpsr(pfc, 5, 9, true);
	pinmux_rcar_set_ipsr(pfc, 12, 12, 0);

	/* SCIF2_TX */
	pinmux_rcar_set_gpsr(pfc, 5, 8, true);
	pinmux_rcar_set_ipsr(pfc, 12, 8, 0);
#endif

	 return 0;
 }

SYS_INIT(rcar_ebisu_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
