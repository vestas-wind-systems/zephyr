/*
 * Copyright (c) 2021 IoT.bzh
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _PFC_RCAR_R8A77951_
#define _PFC_RCAR_R8A77951_
#include "pfc-rcar.h"

static const struct rcar_pin can0_data_a[] = {
	/* TX, RX */
	{4, 24, 8, 1, 23, true}, {4, 28, 8, 1, 24, true},
};

static const struct rcar_pin scif1_data_a[] = {
	/* TX, RX */
	{12, 16, 0, 5, 6, true}, {12, 12, 0, 5, 5, true},
};

static const struct rcar_pin scif2_data_a[] = {
	/* TX, RX */
	{13, 0, 0, 5, 10, true}, {13, 4, 0, 5, 11, true},
};

#endif /* _PFC_RCAR_R8A77951_ */
