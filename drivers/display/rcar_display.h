#pragma once

#include <stdint.h>

typedef struct display_timing {
	uint32_t hFront;
	uint32_t hActive;
	uint32_t hBack;
	uint32_t hSync;
	uint32_t vFront;
	uint32_t vActive;
	uint32_t vBack;
	uint32_t vSync;
} rcar_display_timing_t;

typedef enum {
	DISPLAY_LVDS0,
	DISPLAY_LVDS1,
	DISPLAY_HDMI0,
	DISPLAY_HDMI1,
} rcar_display_path_t;

