/*
 * Copyright (c) 2021 IoT.bzh
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef _PFC_RCAR__H_
#define _PFC_RCAR__H_
#include <stdint.h>
#include <stdbool.h>

struct rcar_pin {
	/* select pin function */
	uint8_t ipsr_bank;      /* bank number 0 - 18 */
	uint8_t ipsr_shift;     /* bit shift 0 - 28 */
	uint8_t ipsr_val;       /* choice from 0x0 to 0xF */
	/* Select gpio or function */
	uint8_t gpsr_bank;      /* bank number 0 - 7 */
	uint8_t gpsr_num;       /* pin index < 32 */
	bool gpsr_val;          /* gpio:false, peripheral:true */
};

#endif /* _PFC_RCAR__H_ */
