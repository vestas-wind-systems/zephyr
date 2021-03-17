/*
 * Copyright (c) 2020 IoT.bzh
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RENESAS_CPG_MSSR_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RENESAS_CPG_MSSR_H_

#define CPG_CORE			0	/* Core Clock */
#define CPG_MOD				1	/* Module Clock */

#define CPG_CORE_CLK_CANFD		0 /* CANFD clock*/
#define CPG_CORE_CLK_S3D4		1 /* SCIF clock */
#define CPG_CORE_CLK_S3D2		2 /* I2C0,1,2 clock */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RENESAS_CPG_MSSR_H_ */
