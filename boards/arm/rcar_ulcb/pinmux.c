/* Copyright 2020 IoT.bzh
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <drivers/pinmux.h>
#include <pinmux/pinmux_rcar.h>

 static int rcar_ulcb_pinmux_init(const struct device *dev)
 {
	 ARG_UNUSED(dev);

	 const struct device *pfc =
		 device_get_binding(DT_LABEL(DT_NODELABEL(pfc)));
	 __ASSERT(pfc, "Fail to get pincontrol device");

#if DT_NODE_HAS_STATUS(DT_NODELABEL(can0), okay) && CONFIG_CAN
	/* CAN0_TX */
	pinmux_rcar_set_gpsr(pfc, 1, 23, true);
	pinmux_rcar_set_ipsr(pfc, 4, 24, 8);

	/* CAN0_RX */
	pinmux_rcar_set_gpsr(pfc, 1, 24, true);
	pinmux_rcar_set_ipsr(pfc, 4, 28, 8);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(scif1), okay) && CONFIG_SERIAL
	/* SCIF1 = SECONDARY SERIAL PORT */
	/* SCIF1_RX */
	pinmux_rcar_set_gpsr(pfc, 5, 5, true);
	pinmux_rcar_set_ipsr(pfc, 12, 12, 0);

	/* SCIF1_TX */
	pinmux_rcar_set_gpsr(pfc, 5, 6, true);
	pinmux_rcar_set_ipsr(pfc, 12, 16, 0);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(scif2), okay) && CONFIG_SERIAL
	/* SCIF2 = UBOOT OUTPUT */
	/* SCIF2_RX */
	pinmux_rcar_set_gpsr(pfc, 5, 11, true);
	pinmux_rcar_set_ipsr(pfc, 13, 4, 0);

	/* SCIF2_TX */
	pinmux_rcar_set_gpsr(pfc, 5, 10, true);
	pinmux_rcar_set_ipsr(pfc, 13, 0, 0);
#endif

	return 0;
 }

SYS_INIT(rcar_ulcb_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
