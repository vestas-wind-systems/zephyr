/*
 * Copyright (c) 2022-2023 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/ucan.h>

#include <usb_descriptor.h>

LOG_MODULE_REGISTER(ucan, CONFIG_USB_DEVICE_UCAN_LOG_LEVEL);

#define DT_DRV_COMPAT zephyr_ucan

/* UCAN protocol version */
#define UCAN_PROTOCOL_VERSION 3U

/* UCAN USB endpoint indexes */
#define UCAN_IN_EP_IDX  0U
#define UCAN_OUT_EP_IDX 1U

/* UCAN USB device commands */
#define UCAN_DEVICE_GET_FW_STRING  0x00U

/* UCAN USB interface commands */
#define UCAN_COMMAND_START         0x00U
#define UCAN_COMMAND_STOP          0x01U
#define UCAN_COMMAND_SLEEP         0x02U
#define UCAN_COMMAND_WAKEUP        0x03U
#define UCAN_COMMAND_RESET         0x04U
#define UCAN_COMMAND_GET           0x05U
#define UCAN_COMMAND_FILTER        0x06U
#define UCAN_COMMAND_SET_BITTIMING 0x07U
#define UCAN_COMMAND_RESTART       0x08U

/* UCAN_COMMAND_GET subcommands */
#define UCAN_SUBCOMMAND_GET_INFO             0x00U
#define UCAN_SUBCOMMAND_GET_PROTOCOL_VERSION 0x01U

struct ucan_config {
	struct usb_if_descriptor if0;
	struct usb_ep_descriptor if0_in_ep;
	struct usb_ep_descriptor if0_out_ep;
} __packed;

struct ucan_data {
	struct usb_dev_data common;
	struct ucan_device_info device_info;
	const struct ucan_ops *ops;
	void *user_data;
	bool started;
};

static const uint32_t ucan_protocol_version = UCAN_PROTOCOL_VERSION;
static sys_slist_t ucan_data_devlist;

#ifdef CONFIG_USB_DEVICE_UCAN_FW_STRING_SUPPORT
#define UCAN_DEFAULT_FIRMWARE_STRING STRINGIFY(BUILD_VERSION)
/* Firmware string is shared by all UCAN interfaces on the device */
static uint8_t *ucan_fw_string = UCAN_DEFAULT_FIRMWARE_STRING;
static size_t ucan_fw_string_len = sizeof(UCAN_DEFAULT_FIRMWARE_STRING);
#endif /* CONFIG_USB_DEVICE_UCAN_FW_STRING_SUPPORT */

static void ucan_interface_config(struct usb_desc_header *head, uint8_t bInterfaceNumber)
{
	struct usb_if_descriptor *if_desc = (struct usb_if_descriptor *)head;

	LOG_DBG("bInterfaceNumber = %u", bInterfaceNumber);
	if_desc->bInterfaceNumber = bInterfaceNumber;
}

static void ucan_status_callback(struct usb_cfg_data *cfg, enum usb_dc_status_code status,
				     const uint8_t *param)
{
	struct usb_dev_data *common;
	struct ucan_data *data;

	common = usb_get_dev_data_by_cfg(&ucan_data_devlist, cfg);
	if (common == NULL) {
		LOG_ERR("device data not found for cfg %p", cfg);
		return;
	}

	data = CONTAINER_OF(common, struct ucan_data, common);

	switch (status) {
	case USB_DC_ERROR:
		LOG_INF("USB device error");
		break;
	case USB_DC_RESET:
		LOG_INF("USB device reset");
		break;
	case USB_DC_CONNECTED:
		LOG_INF("USB device connected");
		break;
	case USB_DC_CONFIGURED:
		LOG_INF("USB device configured");
		break;
	case USB_DC_DISCONNECTED:
		LOG_INF("USB device disconnected");
		break;
	case USB_DC_SUSPEND:
		LOG_INF("USB device suspend");
		break;
	case USB_DC_RESUME:
		LOG_INF("USB device resume");
		break;
	case USB_DC_INTERFACE:
		LOG_INF("USB device interface selected");
		break;
	case USB_DC_SET_HALT:
		LOG_INF("USB device set halt");
		break;
	case USB_DC_CLEAR_HALT:
		LOG_INF("USB device clear halt");
		break;
	case USB_DC_SOF:
		LOG_INF("USB device start-of-frame");
		break;
	case USB_DC_UNKNOWN:
		__fallthrough;
	default:
		LOG_ERR("USB device unknown state");
		break;
	}
}

static int ucan_cmd_start(const struct device *dev, int32_t tlen, uint8_t *tdata)
{
	struct ucan_data *data = dev->data;
	ucan_cmd_start_cb_t callback = data->ops->start;
	uint16_t mode;
	int err = -ENOTSUP;

	if (tlen != sizeof(mode)) {
		LOG_ERR("invalid length for start command (len %d)", tlen);
		return -EINVAL;
	}

	mode = sys_get_le16(tdata);

	LOG_INF("UCAN start, mode = 0x%04x", mode);

	if (callback != NULL) {
		err = callback(dev, mode, data->user_data);
		if (err == 0) {
			data->started = true;
		}
	}

	return err;
}

static int ucan_cmd_stop(const struct device *dev)
{
	struct ucan_data *data = dev->data;
	ucan_cmd_stop_cb_t callback = data->ops->stop;
	int err = -ENOTSUP;

	LOG_INF("UCAN stop");

	if (callback != NULL) {
		err = callback(dev, data->user_data);
		if (err == 0) {
			data->started = false;
		}
	}

	return err;
}

static int ucan_cmd_sleep(const struct device *dev)
{
	ARG_UNUSED(dev);

	LOG_INF("UCAN sleep command not supported");

	return -ENOTSUP;
}

static int ucan_cmd_wakeup(const struct device *dev)
{
	ARG_UNUSED(dev);

	LOG_INF("UCAN wakeup command not supported");

	return -ENOTSUP;
}

static int ucan_cmd_reset(const struct device *dev)
{
	struct ucan_data *data = dev->data;
	ucan_cmd_reset_cb_t callback = data->ops->reset;
	int err = -ENOTSUP;

	LOG_INF("UCAN reset");

	if (callback != NULL) {
		err = callback(dev, data->user_data);
		if (err == 0) {
			data->started = false;
		}
	}

	return 0;
}

static int ucan_cmd_filter(const struct device *dev)
{
	ARG_UNUSED(dev);

	LOG_INF("UCAN filter command not supported");

	return -ENOTSUP;
}

static int ucan_cmd_set_bittiming(const struct device *dev, int32_t tlen, uint8_t *tdata)
{
	struct ucan_data *data = dev->data;
	ucan_cmd_set_bittiming_cb_t callback = data->ops->set_bittiming;
	struct ucan_bittiming timing;
	int err = -ENOTSUP;

	if (tlen != sizeof(timing)) {
		LOG_ERR("invalid length for set bittiming command (len %d)", tlen);
		return -EINVAL;
	}

	if (data->started == true) {
		LOG_ERR("cannot set bittiming while started");
		return -EINVAL;
	}

	memcpy(&timing, tdata, sizeof(timing));
	timing.tq = sys_le32_to_cpu(timing.tq);
	timing.brp = sys_le16_to_cpu(timing.brp);
	timing.sample_point = sys_le16_to_cpu(timing.sample_point);

	LOG_DBG("tq = %u, brp = %u, sample_point = %u, prop_seg = %u, "
		"phase_seq1 = %u, phase_seq2 = %u, sjw = %u",
		timing.tq, timing.brp, timing.sample_point,
		timing.prop_seg, timing.phase_seg1, timing.phase_seg2, timing.sjw);

	if (callback != NULL) {
		err = callback(dev, &timing, data->user_data);
	}

	return err;
}

static int ucan_cmd_restart(const struct device *dev)
{
	struct ucan_data *data = dev->data;
	ucan_cmd_restart_cb_t callback = data->ops->restart;
	int err = -ENOTSUP;

	LOG_INF("UCAN restart");

	if (callback != NULL) {
		err = callback(dev, data->user_data);
	}

	return err;
}

static int ucan_cmd_get_fw_string(const struct device *dev, int32_t *tlen, uint8_t **tdata)
{
#ifdef CONFIG_USB_DEVICE_UCAN_FW_STRING_SUPPORT
	if (ucan_fw_string != NULL && ucan_fw_string_len != 0) {
		*tlen = ucan_fw_string_len;
		*tdata = ucan_fw_string;
		return 0;
	}
#endif /* CONFIG_USB_DEVICE_UCAN_FW_STRING_SUPPORT */

	return -ENOTSUP;
}

static int ucan_cmd_get_info(const struct device *dev, int32_t *tlen, uint8_t **tdata)
{
	struct ucan_data *data = dev->data;

	*tlen = sizeof(data->device_info);
	*tdata = (uint8_t *)(&data->device_info);

	return 0;
}

static int ucan_cmd_get_protocol_version(int32_t *tlen, uint8_t **tdata)
{
	*tlen = sizeof(ucan_protocol_version);
	*tdata = (uint8_t *)&(ucan_protocol_version);

	return 0;
}

static int ucan_vendor_request_handler(struct usb_setup_packet *setup,
					   int32_t *tlen, uint8_t **tdata)
{
	struct usb_dev_data *common;
	const struct device *dev;

	common = usb_get_dev_data_by_iface(&ucan_data_devlist, (uint8_t)setup->wIndex);
	if (common == NULL) {
		LOG_ERR("device data not found for interface %u", setup->wIndex);
		return -ENODEV;
	}

	dev = common->dev;

	switch (setup->RequestType.recipient) {
	case USB_REQTYPE_RECIPIENT_DEVICE:
		if (usb_reqtype_is_to_host(setup)) {
			/* Device to host */
			switch (setup->bRequest) {
			case UCAN_DEVICE_GET_FW_STRING:
				return ucan_cmd_get_fw_string(dev, tlen, tdata);
			default:
				break;
			}
		}
		break;
	case USB_REQTYPE_RECIPIENT_INTERFACE:
		if (usb_reqtype_is_to_host(setup)) {
			/* Interface to host */
			switch (setup->bRequest) {
			case UCAN_COMMAND_GET:
				switch (setup->wValue) {
				case UCAN_SUBCOMMAND_GET_INFO:
					return ucan_cmd_get_info(dev, tlen, tdata);
				case UCAN_SUBCOMMAND_GET_PROTOCOL_VERSION:
					return ucan_cmd_get_protocol_version(tlen, tdata);
				default:
					break;
				}
				break;
			default:
				break;
			}
		} else {
			/* Host to interface */
			switch (setup->bRequest) {
			case UCAN_COMMAND_START:
				return ucan_cmd_start(dev, *tlen, *tdata);
			case UCAN_COMMAND_STOP:
				return ucan_cmd_stop(dev);
			case UCAN_COMMAND_SLEEP:
				return ucan_cmd_sleep(dev);
			case UCAN_COMMAND_WAKEUP:
				return ucan_cmd_wakeup(dev);
			case UCAN_COMMAND_RESET:
				return ucan_cmd_reset(dev);
			case UCAN_COMMAND_FILTER:
				return ucan_cmd_filter(dev);
			case UCAN_COMMAND_SET_BITTIMING:
				return ucan_cmd_set_bittiming(dev, *tlen, *tdata);
			case UCAN_COMMAND_RESTART:
				return ucan_cmd_restart(dev);
			default:
				break;
			};
		}
		break;
	default:
		break;
	}

	LOG_ERR("UCAN device bmRequestType 0x%02x bRequest 0x%02x not supported",
		setup->bmRequestType, setup->bRequest);

	return -ENOTSUP;
}

static void ucan_ep_in_callback(uint8_t ep, enum usb_dc_ep_cb_status_code status)
{
	struct ucan_data *data;
	struct usb_dev_data *common;

	__ASSERT(status == USB_DC_EP_DATA_IN, "invalid EP IN status");

	common = usb_get_dev_data_by_ep(&ucan_data_devlist, ep);
	if (common == NULL) {
		LOG_ERR("device data not found for endpoint %u", ep);
		return;
	}

	data = CONTAINER_OF(common, struct ucan_data, common);

	/* TODO */
	LOG_INF("ucan_usb_ep_in_callback()");
}

static void ucan_ep_out_callback(uint8_t ep, enum usb_dc_ep_cb_status_code status)
{
	struct ucan_data *data;
	ucan_msg_out_cb_t callback;
	struct usb_dev_data *common;
	struct ucan_message_out msg;
	uint32_t nbytes;
	int err;

	__ASSERT(status == USB_DC_EP_DATA_OUT, "invalid EP OUT status");

	common = usb_get_dev_data_by_ep(&ucan_data_devlist, ep);
	if (common == NULL) {
		LOG_ERR("device data not found for endpoint %u", ep);
		/* TODO: stall ep? */
		return;
	}

	data = CONTAINER_OF(common, struct ucan_data, common);
	callback = data->ops->msg_out;

	err = usb_read(ep, (uint8_t *)&msg, sizeof(msg), &nbytes);
	if (err != 0) {
		LOG_ERR("failed to read OUT EP data (err %d)", err);
		/* TODO: stall ep? */
		return;
	}

	if (nbytes < offsetof(struct ucan_message_out, can_msg.id)) {
		LOG_ERR("too few bytes received (nbytes = %u)", nbytes);
		/* TODO: stall ep? */
		return;
	}

	msg.len = sys_le16_to_cpu(msg.len);
	msg.can_msg.id = sys_le32_to_cpu(msg.can_msg.id);

	if (msg.len != nbytes) {
		LOG_ERR("message length mismatch (%u != %u)", msg.len, nbytes);
		/* TODO: stall ep? */
		return;
	}

	if (callback != NULL) {
		err = callback(common->dev, &msg, data->user_data);
		if (err < 0) {
			LOG_ERR("OUT EP callback failed (err %d)", err);
			err = usb_ep_set_stall(ep);
			if (err < 0) {
				LOG_ERR("failed to stall OUT EP %d (err %d)", ep, err);
			}
			return;
		}
	}
}

void ucan_set_fw_string(uint8_t *fw_string, size_t len)
{
#ifdef CONFIG_USB_DEVICE_UCAN_FW_STRING_SUPPORT
	ucan_fw_string = fw_string;
	ucan_fw_string_len = len;
#endif /* CONFIG_USB_DEVICE_UCAN_FW_STRING_SUPPORT */
}

int ucan_write(const struct device *dev, const struct ucan_message_in *msg)
{
	struct usb_cfg_data *cfg = (void *)dev->config;
	uint8_t ep = cfg->endpoint[UCAN_IN_EP_IDX].ep_addr;

	return usb_write(ep, (uint8_t *)&msg, msg->len, NULL);
}

void ucan_register(const struct device *dev, const struct ucan_device_info *info,
		       const struct ucan_ops *ops, void *user_data)
{
	struct ucan_data *data = dev->data;

	__ASSERT_NO_MSG(ops != NULL);
	__ASSERT_NO_MSG(info != NULL);

	data->ops = ops;
	data->user_data = user_data;

	/* Make a copy and do byteswapping to prepare for transmission */
	memcpy(&data->device_info, info, sizeof(data->device_info));
	data->device_info.freq = sys_cpu_to_le32(data->device_info.freq);
	data->device_info.brp_inc = sys_cpu_to_le16(data->device_info.brp_inc);
	data->device_info.brp_min = sys_cpu_to_le16(data->device_info.brp_min);
	data->device_info.brp_max = sys_cpu_to_le16(data->device_info.brp_max);
	data->device_info.ctrlmodes = sys_cpu_to_le16(data->device_info.ctrlmodes);
	data->device_info.hwfilter = sys_cpu_to_le16(data->device_info.hwfilter);
	data->device_info.rxmboxes = sys_cpu_to_le16(data->device_info.rxmboxes);

	sys_slist_append(&ucan_data_devlist, &data->common.node);
}

static int ucan_init(const struct device *dev)
{
	struct ucan_data *data = dev->data;

	data->common.dev = dev;

	return 0;
}

#define INITIALIZER_IF							\
	{								\
		.bLength = sizeof(struct usb_if_descriptor),		\
		.bDescriptorType = USB_DESC_INTERFACE,			\
		.bInterfaceNumber = 0,					\
		.bAlternateSetting = 0,					\
		.bNumEndpoints = 2,					\
		.bInterfaceClass = USB_BCC_VENDOR,			\
		.bInterfaceSubClass = 0,				\
		.bInterfaceProtocol = 0,				\
		.iInterface = 0,					\
	}

#define INITIALIZER_IF_EP(addr, mps)					\
	{								\
		.bLength = sizeof(struct usb_ep_descriptor),		\
		.bDescriptorType = USB_DESC_ENDPOINT,			\
		.bEndpointAddress = addr,				\
		.bmAttributes = USB_DC_EP_BULK,				\
		.wMaxPacketSize = mps,					\
		.bInterval = 0x00,					\
	}

#define UCAN_CFG_AND_DATA_DEFINE(x)					\
	USBD_CLASS_DESCR_DEFINE(primary, 0)				\
	struct ucan_config ucan_cfg_##x = {				\
		.if0 = INITIALIZER_IF,					\
		/* TODO: support for multiple IN messages? */		\
		.if0_in_ep = INITIALIZER_IF_EP(AUTO_EP_IN,		\
			sizeof(struct ucan_message_in)),		\
		.if0_out_ep = INITIALIZER_IF_EP(AUTO_EP_OUT,		\
			sizeof(struct ucan_message_out)), 		\
	};								\
									\
	static struct usb_ep_cfg_data ucan_ep_data_##x[] = {	\
		{							\
			.ep_cb = ucan_ep_in_callback,		\
			.ep_addr = AUTO_EP_IN,				\
		},							\
		{							\
			.ep_cb = ucan_ep_out_callback,		\
			.ep_addr = AUTO_EP_OUT,				\
		},							\
	};								\
									\
	USBD_DEFINE_CFG_DATA(ucan_config_##x) = {			\
		.usb_device_description = NULL,				\
		.interface_config = ucan_interface_config,		\
		.interface_descriptor = &ucan_cfg_##x.if0,		\
		.cb_usb_status = ucan_status_callback,		\
		.interface = {						\
			.class_handler = NULL,				\
			.custom_handler = NULL,				\
			.vendor_handler = ucan_vendor_request_handler, \
		},							\
		.num_endpoints = ARRAY_SIZE(ucan_ep_data_##x),	\
		.endpoint = ucan_ep_data_##x,			\
	};								\
									\
	static struct ucan_data ucan_data_##x = {		\
		.started = false,					\
	};

#define DUMMY_API (const void *)1

#define UCAN_DEVICE_DEFINE(inst)					\
	BUILD_ASSERT(DT_INST_ON_BUS(inst, usb),				\
		     "node " DT_NODE_PATH(DT_DRV_INST(inst))		\
		     " is not assigned to a USB device controller");	\
	UCAN_CFG_AND_DATA_DEFINE(inst)				\
									\
	DEVICE_DT_INST_DEFINE(inst, ucan_init, NULL,		\
		&ucan_data_##inst, &ucan_config_##inst,		\
		POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,	\
		DUMMY_API);

DT_INST_FOREACH_STATUS_OKAY(UCAN_DEVICE_DEFINE);
