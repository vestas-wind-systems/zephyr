/*
 * Copyright (c) 2022-2023 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB UCAN Device Class public header
 */

#ifndef ZEPHYR_INCLUDE_USB_CLASS_UCAN_H_
#define ZEPHYR_INCLUDE_USB_CLASS_UCAN_H_

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/byteorder.h>

#define UCAN_MODE_LOOPBACK    BIT(0)
#define UCAN_MODE_SILENT      BIT(1)
#define UCAN_MODE_3_SAMPLES   BIT(2)
#define UCAN_MODE_ONE_SHOT    BIT(3)
#define UCAN_MODE_BERR_REPORT BIT(4)

#define UCAN_MAX_DLC 8

/* UCAN USB EP OUT message types */
#define UCAN_OUT_TX 0x02U

/* UCAN USB EP IN message types */
#define UCAN_IN_TX_COMPLETE 0x01U
#define UCAN_IN_RX          0x02U

struct ucan_device_info {
	uint32_t freq;
	uint8_t  tx_fifo;
	uint8_t  sjw_max;
	uint8_t  tseg1_min;
	uint8_t  tseg1_max;
	uint8_t  tseg2_min;
	uint8_t  tseg2_max;
	uint16_t brp_inc;
	uint32_t brp_min;
	uint32_t brp_max;
	uint16_t ctrlmodes;
	uint16_t hwfilter;
	uint16_t rxmboxes;
} __packed;

struct ucan_can_msg {
	uint32_t id;
	union {
		uint8_t data[UCAN_MAX_DLC];
		uint8_t dlc;
	};
} __packed;

/* UCAN USB EP OUT payload */
struct ucan_message_out {
	uint16_t len;
	uint8_t type;
	uint8_t subtype;
	struct ucan_can_msg can_msg;
} __packed __aligned(4);

/* UCAN USB EP IN payload */
struct ucan_message_in {
	uint16_t len;
	uint8_t type;
	uint8_t subtype;
	union {
		struct ucan_can_msg can_msg;
		/* struct ucan_tx_complete_entry_t can_tx_complete_msg[0]; ?? */
	};
} __packed __aligned(4);

struct ucan_bittiming {
	uint32_t tq;
	uint16_t brp;
	uint16_t sample_point;
	uint8_t prop_seg;
	uint8_t phase_seg1;
	uint8_t phase_seg2;
	uint8_t sjw;
} __packed;

typedef int (*ucan_cmd_start_cb_t)(const struct device *dev, uint16_t mode, void *user_data);

typedef int (*ucan_cmd_stop_cb_t)(const struct device *dev, void *user_data);

typedef int (*ucan_cmd_reset_cb_t)(const struct device *dev, void *user_data);

typedef int (*ucan_cmd_set_bittiming_cb_t)(const struct device *dev,
					   const struct ucan_bittiming *timing,
					   void *user_data);

typedef int (*ucan_cmd_restart_cb_t)(const struct device *dev, void *user_data);

typedef int (*ucan_msg_out_cb_t)(const struct device *dev, struct ucan_message_out *msg,
				 void *user_data);

struct ucan_ops {
	ucan_cmd_start_cb_t start;
	ucan_cmd_stop_cb_t stop;
	ucan_cmd_reset_cb_t reset;
	ucan_cmd_set_bittiming_cb_t set_bittiming;
	ucan_cmd_restart_cb_t restart;
	ucan_msg_out_cb_t msg_out;
};

void ucan_set_fw_string(uint8_t *fw_string, size_t len);

void ucan_register(const struct device *dev, const struct ucan_device_info *info,
		   const struct ucan_ops *ops, void *user_data);

int ucan_write(const struct device *dev, const struct ucan_message_in *msg);

#endif /* ZEPHYR_INCLUDE_USB_CLASS_UCAN_H_ */
