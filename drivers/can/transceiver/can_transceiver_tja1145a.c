/*
 * Copyright (c) 2024 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/can/transceiver.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>

LOG_MODULE_REGISTER(can_transceiver_tja1145a, CONFIG_CAN_LOG_LEVEL);

#define DT_DRV_COMPAT nxp_tja1145a

/*
 * The register definitions correspond to those found in the NXP TJA1145A datasheet, revision 2
 * 23 September 2020.
 */

/* Mode control register */
#define TJA1145A_MODE_CONTROL    0x01
#define TJA1145A_MODE_CONTROL_MC GENMASK(2, 0)

/* Main status register */
#define TJA1145A_MAIN_STATUS      0x03
#define TJA1145A_MAIN_STATUS_FSMS BIT(7)
#define TJA1145A_MAIN_STATUS_OTWS BIT(6)
#define TJA1145A_MAIN_STATUS_NMS  BIT(5)

/* System even enable register */
#define TJA1145A_SYSTEM_EVENT_ENABLE       0x04
#define TJA1145A_SYSTEM_EVENT_ENABLE_OTWE  BIT(2)
#define TJA1145A_SYSTEM_EVENT_ENABLE_SPIFE BIT(3)

/* Memory 0 to 3 registers */
#define TJA1145A_MEMORY_0 0x06
#define TJA1145A_MEMORY_1 0x07
#define TJA1145A_MEMORY_2 0x08
#define TJA1145A_MEMORY_3 0x09

/* Lock control register */
#define TJA1145A_LOCK_CONTROL      0x0a
#define TJA1145A_LOCK_CONTROL_LK6C BIT(6)
#define TJA1145A_LOCK_CONTROL_LK5C BIT(5)
#define TJA1145A_LOCK_CONTROL_LK4C BIT(4)
#define TJA1145A_LOCK_CONTROL_LK3C BIT(3)
#define TJA1145A_LOCK_CONTROL_LK2C BIT(2)
#define TJA1145A_LOCK_CONTROL_LK1C BIT(1)
#define TJA1145A_LOCK_CONTROL_LK0C BIT(0)

/* CAN control register */
#define TJA1145A_CAN_CONTROL       0x20
#define TJA1145A_CAN_CONTROL_CFDC  BIT(6)
#define TJA1145A_CAN_CONTROL_PNCOK BIT(5)
#define TJA1145A_CAN_CONTROL_CPNC  BIT(4)
#define TJA1145A_CAN_CONTROL_CMC   GENMASK(1, 0)

/* Transceiver status register */
#define TJA1145A_TRANSCEIVER_STATUS        0x22
#define TJA1145A_TRANSCEIVER_STATUS_CTS    BIT(7)
#define TJA1145A_TRANSCEIVER_STATUS_CPNERR BIT(6)
#define TJA1145A_TRANSCEIVER_STATUS_CPNS   BIT(5)
#define TJA1145A_TRANSCEIVER_STATUS_COSCS  BIT(4)
#define TJA1145A_TRANSCEIVER_STATUS_CBSS   BIT(3)
#define TJA1145A_TRANSCEIVER_STATUS_VCS    BIT(1)
#define TJA1145A_TRANSCEIVER_STATUS_CFS    BIT(0)

/* Transceiver event enable register */
#define TJA1145A_TRANSCEIVER_EVENT_ENABLE      0x23
#define TJA1145A_TRANSCEIVER_EVENT_ENABLE_CBSE BIT(4)
#define TJA1145A_TRANSCEIVER_EVENT_ENABLE_CFE  BIT(1)
#define TJA1145A_TRANSCEIVER_EVENT_ENABLE_CWE  BIT(0)

/* Data rate register */
#define TJA1145A_DATA_RATE               0x26
#define TJA1145A_DATA_RATE_CDR           GENMASK(2, 0)
#define TJA1145A_DATA_RATE_CDR_50KBITS   FIELD_PREP(TJA1145A_DATA_RATE_CDR, 0U)
#define TJA1145A_DATA_RATE_CDR_100KBITS  FIELD_PREP(TJA1145A_DATA_RATE_CDR, 1U)
#define TJA1145A_DATA_RATE_CDR_125KBITS  FIELD_PREP(TJA1145A_DATA_RATE_CDR, 2U)
#define TJA1145A_DATA_RATE_CDR_250KBITS  FIELD_PREP(TJA1145A_DATA_RATE_CDR, 3U)
#define TJA1145A_DATA_RATE_CDR_500KBITS  FIELD_PREP(TJA1145A_DATA_RATE_CDR, 5U)
#define TJA1145A_DATA_RATE_CDR_1000KBITS FIELD_PREP(TJA1145A_DATA_RATE_CDR, 7U)

/* Identifier 0 to 3 registers */
#define TJA1145A_IDENTIFIER_0 0x27
#define TJA1145A_IDENTIFIER_1 0x28
#define TJA1145A_IDENTIFIER_2 0x29
#define TJA1145A_IDENTIFIER_3 0x2a

/* Mask 0 to 3 registers */
#define TJA1145A_MASK_0 0x2b
#define TJA1145A_MASK_1 0x2c
#define TJA1145A_MASK_2 0x2d
#define TJA1145A_MASK_3 0x2e

/* Frame control register */
#define TJA1145A_FRAME_CONTROL      0x2f
#define TJA1145A_FRAME_CONTROL_IDE  BIT(7)
#define TJA1145A_FRAME_CONTROL_PNDM BIT(6)
#define TJA1145A_FRAME_CONTROL_DLC  GENMASK(3, 0)

/* WAKE pin status register */
#define TJA1145A_WAKE_PIN_STATUS      0x4b
#define TJA1145A_WAKE_PIN_STATUS_WPVS BIT(1)

/* WAKE pin enable register */
#define TJA1145A_WAKE_PIN_ENABLE      0x4c
#define TJA1145A_WAKE_PIN_ENABLE_WPRE BIT(0)
#define TJA1145A_WAKE_PIN_ENABLE_WPFE BIT(1)

/* Event capture status register */
#define TJA1145A_EVENT_CAPTURE_STATUS      0x60
#define TJA1145A_EVENT_CAPTURE_STATUS_WPE  BIT(3)
#define TJA1145A_EVENT_CAPTURE_STATUS_TRXE BIT(2)
#define TJA1145A_EVENT_CAPTURE_STATUS_SYSE BIT(0)

/* System event status register */
#define TJA1145A_SYSTEM_EVENT_STATUS      0x61
#define TJA1145A_SYSTEM_EVENT_STATUS_PO   BIT(4)
#define TJA1145A_SYSTEM_EVENT_STATUS_OTW  BIT(2)
#define TJA1145A_SYSTEM_EVENT_STATUS_SPIF BIT(1)

/* Transceiver event status register */
#define TJA1145A_TRANSCEIVER_EVENT_STATUS       0x63
#define TJA1145A_TRANSCEIVER_EVENT_STATUS_PNFDE BIT(5)
#define TJA1145A_TRANSCEIVER_EVENT_STATUS_CBS   BIT(4)
#define TJA1145A_TRANSCEIVER_EVENT_STATUS_CF    BIT(1)
#define TJA1145A_TRANSCEIVER_EVENT_STATUS_CW    BIT(0)

/* WAKE pin event status */
#define TJA1145A_WAKE_PIN_EVENT_STATUS     0x064
#define TJA1145A_WAKE_PIN_EVENT_STATUS_WPR BIT(1)
#define TJA1145A_WAKE_PIN_EVENT_STATUS_WPF BIT(0)

/* Data mask 0 to 7 registers */
#define TJA1145A_DATA_MASK_0 0x68
#define TJA1145A_DATA_MASK_1 0x69
#define TJA1145A_DATA_MASK_2 0x6a
#define TJA1145A_DATA_MASK_3 0x6b
#define TJA1145A_DATA_MASK_4 0x6c
#define TJA1145A_DATA_MASK_5 0x6d
#define TJA1145A_DATA_MASK_6 0x6e
#define TJA1145A_DATA_MASK_7 0x6f

/* Identification register */
#define TJA1145A_IDENTIFICATION             0x7e
#define TJA1145A_IDENTIFICATION_TJA1145AT   0x70U /* TJA1145AT, TJA1145ATK */
#define TJA1145A_IDENTIFICATION_TJA1145ATFD 0x74U /* TJA1145AT/FD, TJA1145ATK/FD */

/* Maximum valid address */
#define TJA1145A_ADDR_MAX 0x7f

struct tja1145a_config {
	struct spi_dt_spec spi;
};

struct tja1145a_data {
};

static int tja1145a_read(const struct device *dev, off_t addr, uint8_t *val, size_t len)
{
	const struct tja1145a_config *config = dev->config;
	uint8_t id = (addr << 1U) | 1U;
	const struct spi_buf tx_bufs[] = {
		{.buf = &id, .len = sizeof(id)},
	};
	const struct spi_buf rx_bufs[] = {
		{.buf = NULL, .len = sizeof(id)},
		{.buf = val, .len = len},
	};
	const struct spi_buf_set tx = {.buffers = tx_bufs, .count = ARRAY_SIZE(tx_bufs)};
	const struct spi_buf_set rx = {.buffers = rx_bufs, .count = ARRAY_SIZE(rx_bufs)};

	__ASSERT_NO_MSG(len == 1U || len == 2U || len == 3U);
	__ASSERT_NO_MSG(addr <= TJA1145A_ADDR_MAX);
	__ASSERT_NO_MSG(addr + len <= TJA1145A_ADDR_MAX);

	return spi_transceive_dt(&config->spi, &tx, &rx);
}

static inline int tja1145a_read8(const struct device *dev, off_t addr, uint8_t *val)
{
	return tja1145a_read(dev, addr, val, sizeof(*val));
}

static int tja1145a_write(const struct device *dev, off_t addr, uint8_t *val, size_t len)
{
	const struct tja1145a_config *config = dev->config;
	uint8_t id = addr << 1U;
	const struct spi_buf tx_bufs[] = {
		{.buf = &id, .len = sizeof(id)},
		{.buf = val, .len = len},
	};
	const struct spi_buf_set tx = {.buffers = tx_bufs, .count = ARRAY_SIZE(tx_bufs)};

	__ASSERT_NO_MSG(len == 1U || len == 2U || len == 3U);
	__ASSERT_NO_MSG(addr <= TJA1145A_ADDR_MAX);
	__ASSERT_NO_MSG(addr + len <= TJA1145A_ADDR_MAX);

	return spi_write_dt(&config->spi, &tx);
}

static inline int tja1145a_write8(const struct device *dev, off_t addr, uint8_t val)
{
	return tja1145a_write(dev, addr, &val, sizeof(val));
}

static int tja1145a_enable(const struct device *dev)
{
	return 0;
}

static int tja1145a_disable(const struct device *dev)
{
	return 0;
}

static int tja1145a_init(const struct device *dev)
{
	const struct tja1145a_config *config = dev->config;
	uint8_t reg;
	int err;

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	/*
	 * Initialization sequence according to "Application Hints - High speed CAN transceiver for
	 * partial networking TJA1145A", revision 01.01 — 22 October 2020 (AH1903).
	 */

	/* Check device identification register */
	err = tja1145a_read8(dev, TJA1145A_IDENTIFICATION, &reg);
	if (err < 0) {
		LOG_ERR("failed to read device ID register (err %d)", err);
		return -ENODEV;
	}

	switch (reg) {
	case TJA1145A_IDENTIFICATION_TJA1145AT:
		LOG_DBG("found device ID: TJA1145AT*");
		break;
	case TJA1145A_IDENTIFICATION_TJA1145ATFD:
		LOG_DBG("found device ID: TJA1145AT*/FD");
		break;
	default:
		LOG_ERR("unsupported device ID: 0x%02x", reg);
		return -ENODEV;
	}

	/* TODO: Check reason for entering sleep mode (put this in PM routine as well?), restore */

	/* TODO: Setup event handling thread, if needed */

	/* TODO: read and clear events */
	err = tja1145a_read8(dev, TJA1145A_SYSTEM_EVENT_STATUS, &reg);
	if (err < 0) {
		LOG_ERR("failed to read system event status register (err %d)", err);
		return -ENODEV;
	}
	LOG_DBG("system event status: 0x%02x", reg);

	err = tja1145a_read8(dev, TJA1145A_TRANSCEIVER_EVENT_STATUS, &reg);
	if (err < 0) {
		LOG_ERR("failed to read transceiver event status register (err %d)", err);
		return -ENODEV;
	}
	LOG_DBG("transceiver event status: 0x%02x", reg);

	err = tja1145a_read8(dev, TJA1145A_WAKE_PIN_EVENT_STATUS, &reg);
	if (err < 0) {
		LOG_ERR("failed to read WAKE pin event status register (err %d)", err);
		return -ENODEV;
	}
	LOG_DBG("WAKE pin event status: 0x%02x", reg);

	err = tja1145a_write8(dev, TJA1145A_WAKE_PIN_EVENT_STATUS, 0x3U);
	if (err < 0) {
		LOG_ERR("failed to write WAKE pin event status register (err %d)", err);
		return -ENODEV;
	}

	err = tja1145a_read8(dev, TJA1145A_WAKE_PIN_EVENT_STATUS, &reg);
	if (err < 0) {
		LOG_ERR("failed to read WAKE pin event status register (err %d)", err);
		return -ENODEV;
	}
	LOG_DBG("WAKE pin event status: 0x%02x", reg);

	/* TODO: initialize TJA1145A registers, if needed (mode, events, wake pin) */

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int tja1145a_pm_action(const struct device *dev, enum pm_device_action action)
{
	/* TODO: check k_can_yield()? */

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		/* TODO */
		break;

	case PM_DEVICE_ACTION_RESUME:
		/* TODO */
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static const struct can_transceiver_driver_api tja1145a_driver_api = {
	.enable = tja1145a_enable,
	.disable = tja1145a_disable,
};

#define TJA1145A_INIT(inst)                                                                        \
	static const struct tja1145a_config tja1145a_config_##inst = {                             \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_MODE_CPHA | SPI_WORD_SET(8), 0),             \
	};                                                                                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(inst, tja1145a_pm_action);                                        \
                                                                                                   \
	static struct tja1145a_data tja1145a_data_##inst;                                          \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, tja1145a_init, PM_DEVICE_DT_INST_GET(inst),                    \
			      &tja1145a_data_##inst, &tja1145a_config_##inst, POST_KERNEL,         \
			      CONFIG_CAN_TRANSCEIVER_INIT_PRIORITY, &tja1145a_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TJA1145A_INIT)
