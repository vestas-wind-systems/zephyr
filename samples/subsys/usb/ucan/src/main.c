/*
 * Copyright (c) 2022-2023 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/class/ucan.h>
#include <zephyr/usb/usb_device.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

struct context {
	const struct device *ucan;
	const struct device *can;
	struct k_msgq *msgq;
	struct k_poll_event events[1];
	struct k_work_poll work;
	int filter_id_std;
	int filter_id_ext;
	bool started;
};

static void can_tx_callback(const struct device *dev, int error, void *user_data)
{
	/* struct context *ctx = CONTAINER_OF(dev, struct context, can); */
	uint8_t subtype = POINTER_TO_UINT(user_data);

	LOG_INF("CAN frame (UCAN ID 0x%02x) tx result: %d", subtype, error);
	/* TODO: notify UCAN of transmission result */
}

static void can_rx_work_handler(struct k_work *work)
{
	struct k_work_poll *pwork = CONTAINER_OF(work, struct k_work_poll, work);
	struct context *ctx = CONTAINER_OF(pwork, struct context, work);
	struct ucan_message_in msg;
	struct can_frame frame;
	int err;

	while (k_msgq_get(ctx->msgq, &frame, K_NO_WAIT) == 0) {
		/* TODO: pass received frame to UCAN */
		/* msg.len = sizeof(msg); */
		/* msg.type = UCAN_IN_RX; */
		/* msg.subtype = 0; */

		/* msg.can_msg.id = 0x010; */
		/* msg.can_msg.dlc = frame.dlc; */
		/* memcpy(&msg.can_msg.data, frame.data, can_dlc_to_bytes(frame.dlc)); */
		/* dlc for rtr ? */

		err = ucan_write(ctx->ucan, &msg);
		if (err != 0) {
			LOG_ERR("failed to write CAN frame to UCAN (err %d)", err);
			/* TODO: notify UCAN of error? */
		}
	}

	err = k_work_poll_submit(&ctx->work, ctx->events,
				 ARRAY_SIZE(ctx->events), K_FOREVER);
	if (err != 0) {
		LOG_ERR("failed to resubmit CAN rx msgq polling (err %d)", err);
		/* TODO: notify UCAN of error? */
	}
}

static int ucan_cmd_start(const struct device *ucan, uint16_t mode, void *user_data)
{
	struct context *ctx = user_data;
	struct can_filter filter = { 0 };
	can_mode_t can_mode = CAN_MODE_NORMAL;
	int err;

	LOG_INF("start");

	if ((mode & UCAN_MODE_LOOPBACK) != 0) {
		can_mode |= CAN_MODE_LOOPBACK;
	}

	if ((mode & UCAN_MODE_SILENT) != 0) {
		can_mode |= CAN_MODE_LISTENONLY;
	}

	if ((mode & UCAN_MODE_ONE_SHOT) != 0) {
		can_mode |= CAN_MODE_ONE_SHOT;
	}

	if ((mode & UCAN_MODE_3_SAMPLES) != 0) {
		can_mode |= CAN_MODE_3_SAMPLES;
	}

	/* TODO: berr report mode */
	/* TODO: manual bus recovery */

	err = can_set_mode(ctx->can, can_mode);
	if (err != 0) {
		LOG_ERR("failed set CAN controller mode 0x%08x (err %d)", can_mode, err);
		return err;
	}

	/* Standard CAN ID filter */
	err = can_add_rx_filter_msgq(ctx->can, ctx->msgq, &filter);
	if (err < 0) {
		LOG_ERR("failed attach std rx msgq (err %d)", err);
		return err;
	}

	ctx->filter_id_std = err;

	/* Extended CAN ID filter */
	filter.flags = CAN_FILTER_IDE;
	err = can_add_rx_filter_msgq(ctx->can, ctx->msgq, &filter);
	if (err < 0) {
		LOG_ERR("failed attach ext rx msgq (err %d)", err);
		return err;
	}

	ctx->filter_id_ext = err;

	err = can_start(ctx->can);
	if (err != 0) {
		LOG_ERR("failed to start CAN controller (err %d)", err);
		return err;
	}

	ctx->started = true;

	return 0;
}

static int ucan_cmd_stop(const struct device *ucan, void *user_data)
{
	struct context *ctx = user_data;
	int err;

	if (ctx->filter_id_std >= 0) {
		can_remove_rx_filter(ctx->can, ctx->filter_id_std);
		ctx->filter_id_std = -1;
	}

	if (ctx->filter_id_ext >= 0) {
		can_remove_rx_filter(ctx->can, ctx->filter_id_ext);
		ctx->filter_id_ext = -1;
	}

	if (ctx->started) {
		err = can_stop(ctx->can);
		if (err != 0) {
			LOG_ERR("failed to stop CAN controller (err %d)", err);
			return err;
		}

		ctx->started = false;
	}

	return 0;
}

static int ucan_cmd_reset(const struct device *dev, void *user_data)
{
	LOG_INF("reset");

	return ucan_cmd_stop(dev, user_data);
}

static int ucan_cmd_set_bittiming(const struct device *ucan, const struct ucan_bittiming *timing,
				  void *user_data)
{
	struct context *ctx = user_data;
	struct can_timing can_timing;
	int err;

	can_timing.sjw = timing->sjw;
	can_timing.prop_seg = timing->prop_seg;
	can_timing.phase_seg1 = timing->phase_seg1;
	can_timing.phase_seg2 = timing->phase_seg2;
	can_timing.prescaler = timing->brp;

	err = can_set_timing(ctx->can, &can_timing);
	if (err) {
		LOG_ERR("failed to set CAN controller timing (err %d)", err);
	}

	return err;
}

static int ucan_cmd_restart(const struct device *ucan, void *user_data)
{
	LOG_INF("restart");

	/* TODO: initiate can_recover() in a workq/thread */

	return 0;
}

static int ucan_msg_out(const struct device *ucan, struct ucan_message_out *msg,
			void *user_data)
{
	struct context *ctx = user_data;
	struct can_frame frame;
	int err;

	LOG_INF("ucan_msg_out(): len = %u, type = %u, subtype = %u",
		msg->len, msg->type, msg->subtype);

	if (msg->type == UCAN_OUT_TX) {
		LOG_ERR("unsupported UCAN OUT message type (%u)", msg->type);
		/* TODO: send UCAN error frame? stall EP? */
		return -ENOTSUP;
	};

	LOG_INF("queuing CAN frame (UCAN ID 0x%02x) for transmission", msg->subtype);
	err = can_send(ctx->can, &frame, K_NO_WAIT, can_tx_callback,
		       UINT_TO_POINTER(msg->subtype));
	if (err < 0) {
		LOG_ERR("failed to queue CAN frame (UCAN ID 0x%02x) for transmission (err %d)",
			msg->subtype, err);
		/* TODO: send UCAN error frame, stall EP? */
		return -EIO;
	}

	return 0;
}

static const struct ucan_ops ucan_ops = {
	.start = ucan_cmd_start,
	.stop = ucan_cmd_stop,
	.reset = ucan_cmd_reset,
	.set_bittiming = ucan_cmd_set_bittiming,
	.restart = ucan_cmd_restart,
	.msg_out = ucan_msg_out,
};

static int ucan_device_info_init(struct ucan_device_info *info, const struct device *can)
{
	const struct can_timing *tmin = can_get_timing_min(can);
	const struct can_timing *tmax = can_get_timing_max(can);
	uint32_t clk_rate;
	can_mode_t cap;
	int err;

	err = can_get_core_clock(can, &clk_rate);
	if (err != 0) {
		return err;
	}

	err = can_get_capabilities(can, &cap);
	if (err != 0) {
		return err;
	}

	info->freq = clk_rate;
	info->tx_fifo = CONFIG_SAMPLE_CAN_TX_QUEUE_SIZE;
	info->sjw_max = tmax->sjw;
	info->tseg1_min = tmin->phase_seg1;
	info->tseg1_max = tmax->phase_seg1;
	info->tseg2_min = tmin->phase_seg2;
	info->tseg2_max = tmax->phase_seg2;
	info->brp_inc = 1;
	info->brp_min = tmin->prescaler;
	info->brp_max = tmax->prescaler;
	info->ctrlmodes = UCAN_MODE_BERR_REPORT;
	info->hwfilter = 0; /* Not yet used by Linux kernel driver */
	info->rxmboxes = 0; /* Not yet used by Linux kernel driver */

	if ((cap & CAN_MODE_LOOPBACK) != 0) {
		info->ctrlmodes |= UCAN_MODE_LOOPBACK;
	}

	if ((cap & CAN_MODE_LISTENONLY) != 0) {
		info->ctrlmodes |= UCAN_MODE_SILENT;
	}

	if ((cap & CAN_MODE_ONE_SHOT) != 0) {
		info->ctrlmodes |= UCAN_MODE_ONE_SHOT;
	}

	if ((cap & CAN_MODE_3_SAMPLES) != 0) {
		info->ctrlmodes |= UCAN_MODE_3_SAMPLES;
	}

	return 0;
}

CAN_MSGQ_DEFINE(rx_msgq, CONFIG_SAMPLE_CAN_RX_QUEUE_SIZE);

static struct context ctx = {
	.ucan = DEVICE_DT_GET(DT_NODELABEL(ucan0)),
	.can = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus)),
	.msgq = &rx_msgq,
	.events = {
		K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
					 K_POLL_MODE_NOTIFY_ONLY,
					 &rx_msgq)
	},
	.filter_id_std = -1,
	.filter_id_ext = -1,
};

int main(void)
{
	struct ucan_device_info ucan_info;
	int err;

	LOG_INF("Initializing UCAN \"%s\" on CAN device \"%s\"",
		ctx.ucan->name, ctx.can->name);

	if (!device_is_ready(ctx.ucan)) {
		LOG_ERR("UCAN USB device not ready");
		return 0;
	}

	if (!device_is_ready(ctx.can)) {
		LOG_ERR("CAN controller device not ready");
		return 0;
	}

	k_work_poll_init(&ctx.work, can_rx_work_handler);

	err = k_work_poll_submit(&ctx.work, ctx.events, ARRAY_SIZE(ctx.events), K_FOREVER);
	if (err != 0) {
		LOG_ERR("Failed to submit CAN rx msgq polling (err %d)", err);
		return 0;
	}

	err = ucan_device_info_init(&ucan_info, ctx.can);
	if (err != 0) {
		LOG_ERR("failed to initialize UCAN device info (err %d)", err);
		return 0;
	}

	ucan_register(ctx.ucan, &ucan_info, &ucan_ops, (void *)&ctx);

	err = usb_enable(NULL);
	if (err != 0) {
		LOG_ERR("failed to enable USB (err %d)", err);
		return 0;
	}
}
