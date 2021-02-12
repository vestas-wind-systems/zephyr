/*
 * Copyright (c) 2021 IoT.bzh
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _PFC_RCAR_R8A77990_
#define _PFC_RCAR_R8A77990_
#include "pfc-rcar.h"

static const struct rcar_pin can0_data[] = {
	/* TX, RX */
	{7, 4, 3, 0, 12, true}, {7, 8, 3, 0, 13, true},
};

static const struct rcar_pin scif2_data_a[] = {
	/* TX, RX */
	{12, 8, 0, 5, 8, true}, {12, 12, 0, 5, 9, true},
};

#endif /* _PFC_RCAR_R8A77990_ */
