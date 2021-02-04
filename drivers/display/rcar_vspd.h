#pragma once

/*
 * Copyright (c) 2020 Thierry Bultel <thierry.bultel@iot.bzh>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>

typedef enum {
	RCAR_VSPD0,	/* It's VSPDL on H3 */
	RCAR_VSPD1,
	RCAR_VSPD2,
} vspd_channel_t;


typedef void (*vspd_start_api)(const struct device *dev);
typedef void (*vspd_stop_api)(const struct device *dev);
typedef void * (*vspd_get_framebuffer_api)(const struct device *dev, vspd_channel_t ch);
typedef void (*vspd_get_resolution_api)(const struct device *dev, vspd_channel_t ch, uint32_t *width, uint32_t *height);


/**
 * @brief Display driver API
 * API which a display driver should expose
 */
struct vspd_driver_api {
	vspd_start_api start;
	vspd_stop_api stop;
    vspd_get_framebuffer_api get_framebuffer;
	vspd_get_resolution_api get_resolution;
};