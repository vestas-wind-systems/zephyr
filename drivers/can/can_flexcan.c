/*
 * Copyright (c) 2019-2024 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Base driver compatible */
#define DT_DRV_COMPAT nxp_flexcan

/* CAN FD extension compatible */
#define CAN_FLEXCAN_FD_DRV_COMPAT nxp_flexcan_fd

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/transceiver.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>

LOG_MODULE_REGISTER(can_flexcan, CONFIG_CAN_LOG_LEVEL);

/* TODO: Errata: */
/* 5641: Skip MB0 */
/* 5829: Skip MB0 */
/* 6032: TX issue */
/* 8341: Freeze mode issue */
/* 9595: Freeze mode issue */

/* Module Configuration Register */
#define CAN_FLEXCAN_MCR         0x0000
#define CAN_FLEXCAN_MCR_MDIS    BIT(31)
#define CAN_FLEXCAN_MCR_FRZ     BIT(30)
#define CAN_FLEXCAN_MCR_RFEN    BIT(29)
#define CAN_FLEXCAN_MCR_HALT    BIT(28)
#define CAN_FLEXCAN_MCR_NOTRDY  BIT(27)
#define CAN_FLEXCAN_MCR_WAKMSK  BIT(26)
#define CAN_FLEXCAN_MCR_SOFTRST BIT(25)
#define CAN_FLEXCAN_MCR_FRZACK  BIT(24)
#define CAN_FLEXCAN_MCR_SUPV    BIT(23)
#define CAN_FLEXCAN_MCR_SLFWAK  BIT(22)
#define CAN_FLEXCAN_MCR_WRNEN   BIT(21)
#define CAN_FLEXCAN_MCR_LPMACK  BIT(20)
#define CAN_FLEXCAN_MCR_WAKSRC  BIT(19)
#define CAN_FLEXCAN_MCR_SRXDIS  BIT(17)
#define CAN_FLEXCAN_MCR_IRMQ    BIT(16)
#define CAN_FLEXCAN_MCR_DMA     BIT(15)
#define CAN_FLEXCAN_MCR_LPRIOEN BIT(13)
#define CAN_FLEXCAN_MCR_AEN     BIT(12)
#define CAN_FLEXCAN_MCR_FDEN    BIT(11)
#define CAN_FLEXCAN_MCR_IDAM    GENMASK(9, 8)
#define CAN_FLEXCAN_MCR_MAXMB   GENMASK(6, 0)

/* Control 1 Register */
#define CAN_FLEXCAN_CTRL1         0x0004
#define CAN_FLEXCAN_CTRL1_BOFFMSK BIT(15)
#define CAN_FLEXCAN_CTRL1_ERRMSK  BIT(14)
#define CAN_FLEXCAN_CTRL1_CLKSRC  BIT(13)
#define CAN_FLEXCAN_CTRL1_LPB     BIT(12)
#define CAN_FLEXCAN_CTRL1_TWRNMSK BIT(11)
#define CAN_FLEXCAN_CTRL1_RWRNMSK BIT(10)
#define CAN_FLEXCAN_CTRL1_SMP     BIT(7)
#define CAN_FLEXCAN_CTRL1_BOFFREC BIT(6)
#define CAN_FLEXCAN_CTRL1_TSYN    BIT(5)
#define CAN_FLEXCAN_CTRL1_LBUF    BIT(4)
#define CAN_FLEXCAN_CTRL1_LOM     BIT(3)

/* Free Running Timer */
#define CAN_FLEXCAN_TIMER       0x0008
#define CAN_FLEXCAN_TIMER_VALUE GENMASK(15, 0)

/* Rx Mailboxes Global Mask Register (Legacy) */
#define CAN_FLEXCAN_RXMGMASK 0x0010

/* Rx 14 Mask Register (Legacy) */
#define CAN_FLEXCAN_RX14MASk 0x0014

/* Rx 15 Mask Register (Legacy) */
#define CAN_FLEXCAN_RX15MASK 0x0018

/* Error Counter (*_FAST fields only present in FlexCAN FD) */
#define CAN_FLEXCAN_ECR               0x001c
#define CAN_FLEXCAN_ECR_RXERRCNT_FAST GENMASK(31, 24)
#define CAN_FLEXCAN_ECR_TXERRCNT_FAST GENMASK(23, 16)
#define CAN_FLEXCAN_ECR_RXERRCNT      GENMASK(15, 8)
#define CAN_FLEXCAN_ECR_TXERRCNT      GENMASK(7, 0)

/* Error and Status 1 Register (*_FAST fields only present in FlexCAN FD) */
#define CAN_FLEXCAN_ESR1              0x0020
#define CAN_FLEXCAN_ESR1_BIT1ERR_FAST BIT(31)
#define CAN_FLEXCAN_ESR1_BIT0ERR_FAST BIT(30)
#define CAN_FLEXCAN_ESR1_CRCERR_FAST  BIT(28)
#define CAN_FLEXCAN_ESR1_FRMERR_FAST  BIT(27)
#define CAN_FLEXCAN_ESR1_STFERR_FAST  BIT(26)
#define CAN_FLEXCAN_ESR1_ERROVR       BIT(21)
#define CAN_FLEXCAN_ESR1_ERRINT_FAST  BIT(20)
#define CAN_FLEXCAN_ESR1_BOFFDONEINT  BIT(19)
#define CAN_FLEXCAN_ESR1_SYNCH        BIT(18)
#define CAN_FLEXCAN_ESR1_TWRNINT      BIT(17)
#define CAN_FLEXCAN_ESR1_RWRNINT      BIT(16)
#define CAN_FLEXCAN_ESR1_BIT1ERR      BIT(15)
#define CAN_FLEXCAN_ESR1_BIT0ERR      BIT(14)
#define CAN_FLEXCAN_ESR1_ACKERR       BIT(13)
#define CAN_FLEXCAN_ESR1_CRCERR       BIT(12)
#define CAN_FLEXCAN_ESR1_FRMERR       BIT(11)
#define CAN_FLEXCAN_ESR1_STFERR       BIT(10)
#define CAN_FLEXCAN_ESR1_TXWRN        BIT(9)
#define CAN_FLEXCAN_ESR1_RXWRN        BIT(8)
#define CAN_FLEXCAN_ESR1_IDLE         BIT(7)
#define CAN_FLEXCAN_ESR1_TX           BIT(6)
#define CAN_FLEXCAN_ESR1_FLTCONF      GENMASK(5, 4)
#define CAN_FLEXCAN_ESR1_RX           BIT(3)
#define CAN_FLEXCAN_ESR1_BOFFINT      BIT(2)
#define CAN_FLEXCAN_ESR1_ERRINT       BIT(1)
#define CAN_FLEXCAN_ESR1_WAKINT       BIT(0)

/* Interrupt Masks 2 Register (BUF63TO32M) */
#define CAN_FLEXCAN_IMASK2 0x0024

/* Interrupt Masks 1 Register (BUF31TO0M) */
#define CAN_FLEXCAN_IMASK1 0x0028

/* Interrupt Flags 2 Register (BUF63TO32I) */
#define CAN_FLEXCAN_IFLAG2 0x002c

/* Interrupt Flags 1 Register (BUF31TO0I) */
#define CAN_FLEXCAN_IFLAG1 0x0030

/* Control 2 Register (*_FAST fields only present in FlexCAN FD) */
#define CAN_FLEXCAN_CTRL2             0x0034
#define CAN_FLEXCAN_CTRL2_ERRMSK_FAST BIT(31)
#define CAN_FLEXCAN_CTRL2_BOFFDONEMSK BIT(30)
#define CAN_FLEXCAN_CTRL2_RFFN        GENMASK(27, 24)
#define CAN_FLEXCAN_CTRL2_TASD        GENMASK(23, 19)
#define CAN_FLEXCAN_CTRL2_MRP         BIT(18)
#define CAN_FLEXCAN_CTRL2_RRS         BIT(17)
#define CAN_FLEXCAN_CTRL2_EACEN       BIT(16)
#define CAN_FLEXCAN_CTRL2_TIMER_SRC   BIT(15)
#define CAN_FLEXCAN_CTRL2_PREXCEN     BIT(14)
#define CAN_FLEXCAN_CTRL2_ISOCANFDEN  BIT(12)
#define CAN_FLEXCAN_CTRL2_EDFLTDIS    BIT(11)

/* Error and Status 2 Register */
#define CAN_FLEXCAN_ESR2      0x0038
#define CAN_FLEXCAN_ESR2_LPTM GENMASK(22, 16)
#define CAN_FLEXCAN_ESR2_VPS  BIT(14)
#define CAN_FLEXCAN_ESR2_IMB  BIT(13)

/* CRC Register */
#define CAN_FLEXCAN_CRCR       0x0044
#define CAN_FLEXCAN_CRCR_MBCRC GENMASK(22, 16)
#define CAN_FLEXCAN_CRCR_TXCRC GENMASK(14, 0)

/* Rx FIFO Global Mask Register */
#define CAN_FLEXCAN_RXFGMASK 0x0048

/* Rx FIFO Information Register */
#define CAN_FLEXCAN_RXFIR       0x004c
#define CAN_FLEXCAN_RXFIR_IDHIT GENMASK(8, 0)

/* CAN Bit Timing Register */
#define CAN_FLEXCAN_CBT          0x0050
#define CAN_FLEXCAN_CBT_BTF      BIT(31)
#define CAN_FLEXCAN_CBT_EPRESDIV GENMASK(30, 21)
#define CAN_FLEXCAN_CBT_ERJW     GENMASK(20, 16)
#define CAN_FLEXCAN_CBT_EPROPSEG GENMASK(15, 10)
#define CAN_FLEXCAN_CBT_EPSEG1   GENMASK(9, 5)
#define CAN_FLEXCAN_CBT_EPSEG2   GENMASK(4, 0)

/* Interrupt Masks 3 Register (BUF95TO64M) */
#define CAN_FLEXCAN_IMASK3 0x006c

/* Interrupt Flags 3 Register (BUF95TO64I) */
#define CAN_FLEXCAN_IFLAGS3 0x0074

/* Message Buffers (MBs) base address */
#define CAN_FLEXCAN_MB_BASE 0x0080

/* Rx Individual Mask Registers base (RXIMR0 up to RXIMR95) */
#define CAN_FLEXCAN_RXIMR_BASE 0x0880

/* CAN FD Control Register (FlexCAN FD only) */
#define CAN_FLEXCAN_FDCTRL         0x0c00
#define CAN_FLEXCAN_FDCTRL_FDRATE  BIT(31)
#define CAN_FLEXCAN_FDCTRL_MBDSR2  GENMASK(23, 22)
#define CAN_FLEXCAN_FDCTRL_MBDSR1  GENMASK(20, 19)
#define CAN_FLEXCAN_FDCTRL_MBDSR0  GENMASK(17, 16)
#define CAN_FLEXCAN_FDCTRL_TDCEN   BIT(15)
#define CAN_FLEXCAN_FDCTRL_TDCFAIL BIT(14)
#define CAN_FLEXCAN_FDCTRL_TDCOFF  GENMASK(12, 8)
#define CAN_FLEXCAN_FDCTRL_TDCVAL  GENMASK(5, 0)

/* CAN FD Bit Timing Register (FlexCAN FD only) */
#define CAN_FLEXCAN_FDCBT          0x0c04
#define CAN_FLEXCAN_FDCBT_FPRESDIV GENMASK(29, 20)
#define CAN_FLEXCAN_FDCBT_FRJW     GENMASK(18, 16)
#define CAN_FLEXCAN_FDCBT_FPROPSEG GENMASK(14, 10)
#define CAN_FLEXCAN_FDCBT_FPSEG1   GENMASK(7, 5)
#define CAN_FLEXCAN_FDCBT_FPSEG2   GENMASK(2, 0)

/* CAN FD CRC Register (FlexCAN FD only) */
#define CAN_FLEXCAN_FDCRC 0x0c08
#define CAN_FLEXCAN_FDCRC_FD_MBCRC GENMASK(30, 24)
#define CAN_FLEXCAN_FDCRC_FD_TXCRC GENMASK(20, 0)

/* TODO: ECC registers (present in e.g. S32K3xx) */
/* TODO: Enhanced registers (present in e.g. S32K3xx) */

struct can_flexcan_config {
	const struct can_driver_config common;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	int clk_source;
	mm_reg_t base;
	void (*irq_config_func)(const struct device *dev);
#ifdef CONFIG_CAN_FLEXCAN_FD
	bool flexcan_fd;
#endif /* CONFIG_CAN_FLEXCAN_FD */
	const struct pinctrl_dev_config *pincfg;
};

struct can_flexcan_data {
	struct can_driver_data common;
};

static inline void can_flexcan_write_reg(const struct device *dev, uint16_t reg, uint32_t val)
{
	const struct can_flexcan_config *config = dev->config;

	sys_write32(val, config->base + reg);
}

static inline uint32_t can_flexcan_read_reg(const struct device *dev, uint16_t reg)
{
	const struct can_flexcan_config *config = dev->config;

	return sys_read32(config->base + reg);
}

static uint32_t can_flexcan_read_esr1(const struct device *dev)
{
	uint32_t esr1;

	esr1 = can_flexcan_read_reg(dev, CAN_FLEXCAN_ESR1);

	if ((esr1 & CAN_FLEXCAN_ESR1_BIT1ERR) != 0U) {
		CAN_STATS_BIT1_ERROR_INC(dev);
	}

	if ((esr1 & CAN_FLEXCAN_ESR1_BIT0ERR) != 0U) {
		CAN_STATS_BIT0_ERROR_INC(dev);
	}

	if ((esr1 & CAN_FLEXCAN_ESR1_ACKERR) != 0U) {
		CAN_STATS_ACK_ERROR_INC(dev);
	}

	if ((esr1 & CAN_FLEXCAN_ESR1_CRCERR) != 0U) {
		CAN_STATS_CRC_ERROR_INC(dev);
	}

	if ((esr1 & CAN_FLEXCAN_ESR1_FRMERR) != 0U) {
		CAN_STATS_FORM_ERROR_INC(dev);
	}

	if ((esr1 & CAN_FLEXCAN_ESR1_STFERR) != 0U) {
		CAN_STATS_STUFF_ERROR_INC(dev);
	}

	return esr1;
}

static int can_flexcan_enter_freeze_mode(const struct device *dev, uint32_t timeout_us)
{
	uint32_t mcr;

	/* TODO: handle errata 8341 and 9595 */

	/* TODO: locking? */

	mcr = can_flexcan_read_reg(dev, CAN_FLEXCAN_MCR);
	mcr |= CAN_FLEXCAN_MCR_FRZ;
	can_flexcan_write_reg(dev, CAN_FLEXCAN_MCR, mcr);

	mcr |= CAN_FLEXCAN_MCR_HALT;
	can_flexcan_write_reg(dev, CAN_FLEXCAN_MCR, mcr);

	if (!WAIT_FOR((can_flexcan_read_reg(dev, CAN_FLEXCAN_MCR) & CAN_FLEXCAN_MCR_FRZACK) != 0U,
		      timeout_us, k_busy_wait(1U))) {
		LOG_WRN("timeout on entering freeze mode (timeout %u us)", timeout_us);
		return -EAGAIN;
	}

	return 0;
}

static int can_flexcan_leave_freeze_mode(const struct device *dev, uint32_t timeout_us)
{
	uint32_t mcr;

	/* TODO: locking? */

	mcr = can_flexcan_read_reg(dev, CAN_FLEXCAN_MCR);
	mcr &= ~(CAN_FLEXCAN_MCR_HALT);
	can_flexcan_write_reg(dev, CAN_FLEXCAN_MCR, mcr);

	mcr &= ~(CAN_FLEXCAN_MCR_FRZ);
	can_flexcan_write_reg(dev, CAN_FLEXCAN_MCR, mcr);

	if (!WAIT_FOR((can_flexcan_read_reg(dev, CAN_FLEXCAN_MCR) & CAN_FLEXCAN_MCR_FRZACK) == 0U,
		      timeout_us, k_busy_wait(1U))) {
		LOG_WRN("timeout on leaving freeze mode (timeout %u us)", timeout_us);
		return -EAGAIN;
	}

	return 0;

}

static int can_flexcan_get_core_clock(const struct device *dev, uint32_t *rate)
{
	const struct can_flexcan_config *config = dev->config;

	return clock_control_get_rate(config->clock_dev, config->clock_subsys, rate);
}

static int can_flexcan_get_max_filters(const struct device *dev, bool ide)
{
	ARG_UNUSED(ide);

	/* TODO: return max MB minus tx MBs */
	return 10U;
}

static int can_flexcan_set_timing(const struct device *dev, const struct can_timing *timing)
{
	struct can_flexcan_data *data = dev->data;
	uint32_t cbt;

	if (!timing) {
		return -EINVAL;
	}

	if (data->common.started) {
		return -EBUSY;
	}

	cbt = CAN_FLEXCAN_CBT_BTF |
		FIELD_PREP(CAN_FLEXCAN_CBT_EPRESDIV, timing->prescaler - 1U) |
		FIELD_PREP(CAN_FLEXCAN_CBT_ERJW, timing->sjw - 1U) |
		FIELD_PREP(CAN_FLEXCAN_CBT_EPROPSEG, timing->prop_seg - 1U) |
		FIELD_PREP(CAN_FLEXCAN_CBT_EPSEG1, timing->phase_seg1 - 1U) |
		FIELD_PREP(CAN_FLEXCAN_CBT_EPSEG2, timing->phase_seg2 - 1U);

	can_flexcan_write_reg(dev, CAN_FLEXCAN_CBT, cbt);

	return 0;
}

#ifdef CONFIG_CAN_FLEXCAN_FD
static int can_flexcan_set_timing_data(const struct device *dev,
				       const struct can_timing *timing_data)
{
	struct can_flexcan_data *data = dev->data;
	uint32_t fdcbt;

	if (!timing_data) {
		return -EINVAL;
	}

	if (data->common.started) {
		return -EBUSY;
	}

	fdcbt = FIELD_PREP(CAN_FLEXCAN_FDCBT_FPRESDIV, timing_data->prescaler - 1U) |
		FIELD_PREP(CAN_FLEXCAN_FDCBT_FRJW, timing_data->sjw - 1U) |
		FIELD_PREP(CAN_FLEXCAN_FDCBT_FPROPSEG, timing_data->prop_seg - 1U) |
		FIELD_PREP(CAN_FLEXCAN_FDCBT_FPSEG1, timing_data->phase_seg1 - 1U) |
		FIELD_PREP(CAN_FLEXCAN_FDCBT_FPSEG2, timing_data->phase_seg2 - 1U);

	can_flexcan_write_reg(dev, CAN_FLEXCAN_FDCBT, fdcbt);

	/* TODO: set TDCOFF in FDCTRL with locking */

	return 0;
}
#endif /* CONFIG_CAN_FLEXCAN_FD */

static int can_flexcan_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	const struct can_flexcan_config *config = dev->config;

	*cap = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_3_SAMPLES;

	if (IS_ENABLED(CONFIG_CAN_MANUAL_RECOVERY_MODE)) {
		*cap |= CAN_MODE_MANUAL_RECOVERY;
	}

	if (UTIL_AND(IS_ENABLED(CONFIG_CAN_FLEXCAN_FD), config->flexcan_fd)) {
		*cap |= CAN_MODE_FD;
	}

	return 0;
}

static int can_flexcan_start(const struct device *dev)
{
	const struct can_flexcan_config *config = dev->config;
	struct can_flexcan_data *data = dev->data;
	int err;

	if (data->common.started) {
		return -EALREADY;
	}

	if (config->common.phy != NULL) {
		err = can_transceiver_enable(config->common.phy, data->common.mode);
		if (err != 0) {
			LOG_ERR("failed to enable CAN transceiver (err %d)", err);
			return err;
		}
	}

	/* Reset statistics and clear error counters */
	CAN_STATS_RESET(dev);

	/* Clear error counters */
	can_flexcan_write_reg(dev, CAN_FLEXCAN_ECR, 0U);

	/* TODO: Exit freeze mode */
	/* TODO: default timeout? */
	/* err = can_flexcan_leave_freeze_mode(dev, 0U); */
	if (err != 0) {
		return -EIO;
	}

	data->common.started = true;

	return 0;
}

static int can_flexcan_stop(const struct device *dev)
{
	const struct can_flexcan_config *config = dev->config;
	struct can_flexcan_data *data = dev->data;
	int err;

	if (!data->common.started) {
		return -EALREADY;
	}

	data->common.started = false;

	/* TODO: Abort any pending TX frames before entering freeze mode */
	/* TODO: enter freeze mode */
	/* TODO: default timeout? */
	/* err = can_flexcan_leave_freeze_mode(dev, 0U); */
	if (err != 0) {
		return -EIO;
	}

	if (config->common.phy != NULL) {
		err = can_transceiver_disable(config->common.phy);
		if (err != 0) {
			LOG_ERR("failed to disable CAN transceiver (err %d)", err);
			return err;
		}
	}

	return 0;
}

static int can_flexcan_set_mode(const struct device *dev, can_mode_t mode)
{
	can_mode_t supported = CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_3_SAMPLES;
	const struct can_flexcan_config *config = dev->config;
	struct can_flexcan_data *data = dev->data;
	uint32_t ctrl1;
	uint32_t mcr;

	if (data->common.started) {
		return -EBUSY;
	}

	if (IS_ENABLED(CONFIG_CAN_MANUAL_RECOVERY_MODE)) {
		supported |= CAN_MODE_MANUAL_RECOVERY;
	}

	if (UTIL_AND(IS_ENABLED(CONFIG_CAN_FLEXCAN_FD), config->flexcan_fd)) {
		supported |= CAN_MODE_FD;
	}

	if ((mode & ~(supported)) != 0) {
		LOG_ERR("unsupported mode: 0x%08x", mode);
		return -ENOTSUP;
	}

	if ((mode & CAN_MODE_FD) != 0 && (mode & CAN_MODE_3_SAMPLES) != 0) {
		LOG_ERR("triple samling is not supported in CAN FD mode");
		return -ENOTSUP;
	}

	/* TODO: locking */

	ctrl1 = can_flexcan_read_reg(dev, CAN_FLEXCAN_CTRL1);
	mcr = can_flexcan_read_reg(dev, CAN_FLEXCAN_MCR);

	if ((mode & CAN_MODE_LOOPBACK) != 0) {
		/* Enable loopback and self-reception */
		ctrl1 |= CAN_FLEXCAN_CTRL1_LPB;
		mcr &= ~(CAN_FLEXCAN_MCR_SRXDIS);
	} else {
		/* Disable loopback and self-reception */
		ctrl1 &= ~(CAN_FLEXCAN_CTRL1_LPB);
		mcr |= CAN_FLEXCAN_MCR_SRXDIS;
	}

	if ((mode & CAN_MODE_LISTENONLY) != 0) {
		/* Enable listen-only mode */
		ctrl1 |= CAN_FLEXCAN_CTRL1_LOM;
	} else {
		/* Disable listen-only mode */
		ctrl1 &= ~(CAN_FLEXCAN_CTRL1_LOM);
	}

	if ((mode & CAN_MODE_3_SAMPLES) != 0) {
		/* Enable triple sampling mode */
		ctrl1 |= CAN_FLEXCAN_CTRL1_SMP;
	} else {
		/* Disable triple sampling mode */
		ctrl1 &= ~(CAN_FLEXCAN_CTRL1_SMP);
	}

	if (IS_ENABLED(CONFIG_CAN_MANUAL_RECOVERY_MODE)) {
		if ((mode & CAN_MODE_MANUAL_RECOVERY) != 0) {
			/* Disable auto-recovery from bus-off */
			ctrl1 |= CAN_FLEXCAN_CTRL1_BOFFREC;
		} else {
			/* Enable auto-recovery from bus-off */
			ctrl1 &= ~(CAN_FLEXCAN_CTRL1_BOFFREC);
		}
	}

	if (UTIL_AND(IS_ENABLED(CONFIG_CAN_FLEXCAN_FD), config->flexcan_fd)) {
		if ((mode & CAN_MODE_FD) != 0) {
			/* TODO: enable TDC if not in loopback */

			/* Enable CAN FD mode */
			mcr |= CAN_FLEXCAN_MCR_FDEN;
		} else {
			/* Disable CAN FD mode */
			mcr &= ~(CAN_FLEXCAN_MCR_FDEN);
		}
	}

	can_flexcan_write_reg(dev, CAN_FLEXCAN_CTRL1, ctrl1);
	can_flexcan_write_reg(dev, CAN_FLEXCAN_MCR, mcr);

	data->common.mode = mode;

	return 0;
}

static int can_flexcan_get_state(const struct device *dev, enum can_state *state,
				 struct can_bus_err_cnt *err_cnt)
{
	struct can_flexcan_data *data = dev->data;
	uint32_t esr1;
	uint32_t ecr;

	if (state != NULL) {
		if (!data->common.started) {
			*state = CAN_STATE_STOPPED;
		} else {
			esr1 = can_flexcan_read_esr1(dev);

			/* if ((status_flags & CAN_ESR1_FLTCONF(2)) != 0U) { */
			/* 	*state = CAN_STATE_BUS_OFF; */
			/* } else if ((status_flags & CAN_ESR1_FLTCONF(1)) != 0U) { */
			/* 	*state = CAN_STATE_ERROR_PASSIVE; */
			/* } else if ((status_flags & */
			/* 	(kFLEXCAN_TxErrorWarningFlag | kFLEXCAN_RxErrorWarningFlag)) != 0) { */
			/* 	*state = CAN_STATE_ERROR_WARNING; */
			/* } else { */
			/* 	*state = CAN_STATE_ERROR_ACTIVE; */
			/* } */
		}
	}

	if (err_cnt != NULL) {
		ecr = can_flexcan_read_reg(dev, CAN_FLEXCAN_ECR);
		err_cnt->rx_err_cnt = FIELD_GET(CAN_FLEXCAN_ECR_RXERRCNT, ecr);
		err_cnt->tx_err_cnt = FIELD_GET(CAN_FLEXCAN_ECR_TXERRCNT, ecr);
	}

	return 0;
}

static int can_flexcan_send(const struct device *dev, const struct can_frame *frame,
			    k_timeout_t timeout, can_tx_callback_t callback, void *user_data)
{
	struct can_flexcan_data *data = dev->data;
	uint8_t max_dlc = CAN_MAX_DLC;
	enum can_state state;

	if (UTIL_AND(IS_ENABLED(CONFIG_CAN_FLEXCAN_FD),
		     ((data->common.mode & CAN_MODE_FD) != 0U))) {
		if ((frame->flags &
		     ~(CAN_FRAME_IDE | CAN_FRAME_RTR | CAN_FRAME_FDF | CAN_FRAME_BRS)) != 0) {
			LOG_ERR("unsupported CAN frame flags 0x%02x", frame->flags);
			return -ENOTSUP;
		}

		if ((frame->flags & CAN_FRAME_FDF) != 0) {
			max_dlc = CANFD_MAX_DLC;
		}
	} else {
		if ((frame->flags & ~(CAN_FRAME_IDE | CAN_FRAME_RTR)) != 0) {
			LOG_ERR("unsupported CAN frame flags 0x%02x", frame->flags);
			return -ENOTSUP;
		}
	}

	if (frame->dlc > max_dlc) {
		LOG_ERR("DLC of %d exceeds maximum (%d)", frame->dlc, max_dlc);
		return -EINVAL;
	}

	if (!data->common.started) {
		return -ENETDOWN;
	}

	(void)can_flexcan_get_state(dev, &state, NULL);
	if (state == CAN_STATE_BUS_OFF) {
		LOG_DBG("failed to send frame, bus-off");
		return -ENETUNREACH;
	}

	/* TODO: aquire TX MB within timeout */

	/* TODO: queue TX frame frame */

	return 0;
}

static int can_flexcan_add_rx_filter(const struct device *dev, can_rx_callback_t callback,
				     void *user_data, const struct can_filter *filter)
{
	struct can_flexcan_data *data = dev->data;

	__ASSERT_NO_MSG(callback);

	if ((filter->flags & ~(CAN_FILTER_IDE)) != 0) {
		LOG_ERR("unsupported CAN filter flags 0x%02x", filter->flags);
		return -ENOTSUP;
	}

	/* TODO: Find and allocate RX message buffer */

	/* TODO: Configure RX MB */
	/* TODO: The indidual RX mask registers can only be written in freeze mode */
	/* config->base->RXIMR[ALLOC_IDX_TO_RXMB_IDX(alloc)] = mask; */

	/* data->rx_cbs[alloc].arg = user_data; */
	/* data->rx_cbs[alloc].function = callback; */

	if (data->common.started) {
		/* TODO: exit freeze mode again */
	}

	/* TODO: return ID */
	return 0;
}

static void can_flexcan_set_state_change_callback(const struct device *dev,
						  can_state_change_callback_t callback,
						  void *user_data)
{
	struct can_flexcan_data *data = dev->data;

	data->common.state_change_cb = callback;
	data->common.state_change_cb_user_data = user_data;
}

#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
static int can_flexcan_recover(const struct device *dev, k_timeout_t timeout)
{
	const struct can_flexcan_config *config = dev->config;
	struct can_flexcan_data *data = dev->data;
	enum can_state state;
	uint64_t start_time;
	int ret = 0;

	if (!data->common.started) {
		return -ENETDOWN;
	}

	if ((data->common.mode & CAN_MODE_MANUAL_RECOVERY) == 0U) {
		return -ENOTSUP;
	}

	(void)can_flexcan_get_state(dev, &state, NULL);
	if (state != CAN_STATE_BUS_OFF) {
		return 0;
	}

	start_time = k_uptime_ticks();
	/* TODO: initiate bus-off recovery */
	/* config->base->CTRL1 &= ~CAN_CTRL1_BOFFREC_MASK; */

	/* TODO: use WAIT_FOR()? */
	if (!K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
		(void)can_flexcan_get_state(dev, &state, NULL);

		while (state == CAN_STATE_BUS_OFF) {
			if (!K_TIMEOUT_EQ(timeout, K_FOREVER) &&
			    k_uptime_ticks() - start_time >= timeout.ticks) {
				ret = -EAGAIN;
			}

			(void)can_flexcan_get_state(dev, &state, NULL);
		}
	}

	/* TODO: What does this do? /
	/* config->base->CTRL1 |= CAN_CTRL1_BOFFREC_MASK; */

	return ret;
}
#endif /* CONFIG_CAN_MANUAL_RECOVERY_MODE */

static void can_flexcan_remove_rx_filter(const struct device *dev, int filter_id)
{
/* 	struct can_flexcan_data *data = dev->data; */

/* 	if (filter_id < 0 || filter_id >= CAN_FLEXCAN_MAX_RX) { */
/* 		LOG_ERR("filter ID %d out of bounds", filter_id); */
/* 		return; */
/* 	} */

/* 	k_mutex_lock(&data->rx_mutex, K_FOREVER); */

/* 	if (atomic_test_and_clear_bit(data->rx_allocs, filter_id)) { */
/* #ifdef CONFIG_CAN_FLEXCAN_FD */
/* 		const struct can_flexcan_config *config = dev->config; */

/* 		/\* Stop FlexCAN FD MBs unless already in stopped mode *\/ */
/* 		if (!config->flexcan_fd || data->common.started) { */
/* #endif /\* CONFIG_CAN_FLEXCAN_FD *\/ */
/* 			can_flexcan_mb_stop(dev, filter_id); */
/* #ifdef CONFIG_CAN_FLEXCAN_FD */
/* 		} */
/* #endif /\* CONFIG_CAN_FLEXCAN_FD *\/ */

/* 		data->rx_cbs[filter_id].function = NULL; */
/* 		data->rx_cbs[filter_id].arg = NULL; */
/* 	} else { */
/* 		LOG_WRN("Filter ID %d already detached", filter_id); */
/* 	} */

/* 	k_mutex_unlock(&data->rx_mutex); */
}

static inline void can_flexcan_transfer_error_status(const struct device *dev, uint64_t error)
{
	const struct can_flexcan_config *config = dev->config;
	struct can_flexcan_data *data = dev->data;
	const can_state_change_callback_t cb = data->common.state_change_cb;
	void *cb_data = data->common.state_change_cb_user_data;
	can_tx_callback_t function;
	void *arg;
	int alloc;
	enum can_state state;
	struct can_bus_err_cnt err_cnt;

	/* if ((error & kFLEXCAN_Bit0Error) != 0U) { */
	/* 	CAN_STATS_BIT0_ERROR_INC(dev); */
	/* } */

	/* if ((error & kFLEXCAN_Bit1Error) != 0U) { */
	/* 	CAN_STATS_BIT1_ERROR_INC(dev); */
	/* } */

	/* if ((error & kFLEXCAN_AckError) != 0U) { */
	/* 	CAN_STATS_ACK_ERROR_INC(dev); */
	/* } */

	/* if ((error & kFLEXCAN_StuffingError) != 0U) { */
	/* 	CAN_STATS_STUFF_ERROR_INC(dev); */
	/* } */

	/* if ((error & kFLEXCAN_FormError) != 0U) { */
	/* 	CAN_STATS_FORM_ERROR_INC(dev); */
	/* } */

	/* if ((error & kFLEXCAN_CrcError) != 0U) { */
	/* 	CAN_STATS_CRC_ERROR_INC(dev); */
	/* } */

	/* (void)can_flexcan_get_state(dev, &state, &err_cnt); */
	/* if (data->state != state) { */
	/* 	data->state = state; */

	/* 	if (cb != NULL) { */
	/* 		cb(dev, state, err_cnt, cb_data); */
	/* 	} */
	/* } */

/* 	if (state == CAN_STATE_BUS_OFF) { */
/* 		/\* Abort any pending TX frames in case of bus-off *\/ */
/* 		for (alloc = 0; alloc < CAN_FLEXCAN_MAX_TX; alloc++) { */
/* 			/\* Copy callback function and argument before clearing bit *\/ */
/* 			function = data->tx_cbs[alloc].function; */
/* 			arg = data->tx_cbs[alloc].arg; */

/* 			if (atomic_test_and_clear_bit(data->tx_allocs, alloc)) { */
/* #ifdef CONFIG_CAN_FLEXCAN_FD */
/* 				if ((data->common.mode & CAN_MODE_FD) != 0U) { */
/* 					/\* FLEXCAN_TransferFDAbortSend(config->base, &data->handle, */
/* 					 *\/ */
/* 					/\* 			    ALLOC_IDX_TO_TXMB_IDX(alloc)); */
/* 					 *\/ */
/* 				} else { */
/* #endif                                  /\* CONFIG_CAN_FLEXCAN_FD *\/ */
/* 					/\* FLEXCAN_TransferAbortSend(config->base, &data->handle, *\/ */
/* 					/\* 			  ALLOC_IDX_TO_TXMB_IDX(alloc)); *\/ */
/* #ifdef CONFIG_CAN_FLEXCAN_FD */
/* 				} */
/* #endif /\* CONFIG_CAN_FLEXCAN_FD *\/ */

/* 				function(dev, -ENETUNREACH, arg); */
/* 				k_sem_give(&data->tx_allocs_sem); */
/* 			} */
/* 		} */
/* 	} */
}

static inline void can_flexcan_transfer_tx_idle(const struct device *dev, uint32_t mb)
{
	/* alloc = TX_MBIDX_TO_ALLOC_IDX(mb); */

	/* Copy callback function and argument before clearing bit */
	/* function = data->tx_cbs[alloc].function; */
	/* arg = data->tx_cbs[alloc].arg; */

	/* if (atomic_test_and_clear_bit(data->tx_allocs, alloc)) { */
	/* 	function(dev, 0, arg); */
	/* 	k_sem_give(&data->tx_allocs_sem); */
	/* } */
}

static inline void can_flexcan_transfer_rx_idle(const struct device *dev, uint32_t mb)
{
	const struct can_flexcan_config *config = dev->config;
	struct can_flexcan_data *data = dev->data;
	/* flexcan_mb_transfer_t xfer; */
	struct can_frame frame;
	/* status_t status = kStatus_Fail; */

	/* alloc = RX_MBIDX_TO_ALLOC_IDX(mb); */
	/* function = data->rx_cbs[alloc].function; */
	/* arg = data->rx_cbs[alloc].arg; */

	/* if (atomic_test_bit(data->rx_allocs, alloc)) { */
/* #ifdef CONFIG_CAN_FLEXCAN_FD */
/* 		if ((data->common.mode & CAN_MODE_FD) != 0U) { */
			/* can_flexcan_fd_to_can_frame(&data->rx_cbs[alloc].frame.fd, &frame); */
		/* } else { */
/* #endif                  /\* CONFIG_CAN_FLEXCAN_FD *\/ */
			/* can_flexcan_to_can_frame(&data->rx_cbs[alloc].frame.classic, &frame); */
/* #ifdef CONFIG_CAN_FLEXCAN_FD */
		/* } */
/* #endif /\* CONFIG_CAN_FLEXCAN_FD *\/ */
		/* function(dev, &frame, arg); */

		/* Setup RX message buffer to receive next message */
		/* xfer.mbIdx = mb; */
/* #ifdef CONFIG_CAN_FLEXCAN_FD */
/* 		if ((data->common.mode & CAN_MODE_FD) != 0U) { */
			/* xfer.framefd = &data->rx_cbs[alloc].frame.fd; */
			/* status = FLEXCAN_TransferFDReceiveNonBlocking(config->base, */
			/* 					      &data->handle, */
			/* 					      &xfer); */
		/* } else { */
/* #endif                  /\* CONFIG_CAN_FLEXCAN_FD *\/ */
			/* xfer.frame = &data->rx_cbs[alloc].frame.classic; */
			/* status = FLEXCAN_TransferReceiveNonBlocking(config->base, */
			/* 					    &data->handle, */
			/* 					    &xfer); */
/* #ifdef CONFIG_CAN_FLEXCAN_FD */
/* 		} */
/* #endif /\* CONFIG_CAN_FLEXCAN_FD *\/ */

		/* if (status != kStatus_Success) { */
		/* 	LOG_ERR("Failed to restart rx for filter id %d " */
		/* 		"(err = %d)", alloc, status); */
		/* } */
	/* } */
}

/* static FLEXCAN_CALLBACK(can_flexcan_transfer_callback) */
/* { */
/* 	struct can_flexcan_data *data = (struct can_flexcan_data *)userData; */
/* 	const struct can_flexcan_config *config = data->dev->config; */
/* 	/\* */
/* 	 * The result field can either be a MB index (which is limited to 32 bit */
/* 	 * value) or a status flags value, which is 32 bit on some platforms but */
/* 	 * 64 on others. To decouple the remaining functions from this, the */
/* 	 * result field is always promoted to uint64_t. */
/* 	 *\/ */
/* 	uint32_t mb = (uint32_t)result; */
/* 	uint64_t status_flags = result; */

/* 	ARG_UNUSED(base); */

/* 	switch (status) { */
/* 	case kStatus_FLEXCAN_UnHandled: */
/* 		/\* Not all fault confinement state changes are handled by the HAL *\/ */
/* 		__fallthrough; */
/* 	case kStatus_FLEXCAN_ErrorStatus: */
/* 		can_flexcan_transfer_error_status(data->dev, status_flags); */
/* 		break; */
/* 	case kStatus_FLEXCAN_TxSwitchToRx: */
/* #ifdef CONFIG_CAN_FLEXCAN_FD */
/* 		if ((data->common.mode & CAN_MODE_FD) != 0U) { */
/* 			FLEXCAN_TransferFDAbortReceive(config->base, &data->handle, mb); */
/* 		} else { */
/* #endif /\* CONFIG_CAN_FLEXCAN_FD *\/ */
/* 			FLEXCAN_TransferAbortReceive(config->base, &data->handle, mb); */
/* #ifdef CONFIG_CAN_FLEXCAN_FD */
/* 		} */
/* #endif /\* CONFIG_CAN_FLEXCAN_FD *\/ */
/* 		__fallthrough; */
/* 	case kStatus_FLEXCAN_TxIdle: */
/* 		can_flexcan_transfer_tx_idle(data->dev, mb); */
/* 		break; */
/* 	case kStatus_FLEXCAN_RxOverflow: */
/* 		CAN_STATS_RX_OVERRUN_INC(data->dev); */
/* 		__fallthrough; */
/* 	case kStatus_Fail: */
/* 		/\* If reading an RX MB failed mark it as idle to be reprocessed. *\/ */
/* 		__fallthrough; */
/* 	case kStatus_FLEXCAN_RxIdle: */
/* 		can_flexcan_transfer_rx_idle(data->dev, mb); */
/* 		break; */
/* 	default: */
/* 		LOG_WRN("Unhandled status 0x%08x (result = 0x%016llx)", */
/* 			status, status_flags); */
/* 	} */
/* } */

static void can_flexcan_isr(const struct device *dev)
{
	const struct can_flexcan_config *config = dev->config;
	struct can_flexcan_data *data = dev->data;

	/* FLEXCAN_TransferHandleIRQ(config->base, &data->handle); */
}

static int can_flexcan_init(const struct device *dev)
{
	const struct can_flexcan_config *config = dev->config;
	struct can_flexcan_data *data = dev->data;
	struct can_timing timing = {0};
	int err;

	if (config->common.phy != NULL) {
		if (!device_is_ready(config->common.phy)) {
			LOG_ERR("CAN transceiver not ready");
			return -ENODEV;
		}
	}

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("clock device not ready");
		return -ENODEV;
	}

	err = clock_control_on(config->clock_dev, config->clock_subsys);
	if (err != 0) {
		LOG_ERR("failed to enable clock (err %d)", err);
		return -ENODEV;
	}

	/* TODO: reset flexcan */

	/* TODO: enter freeze mode */

	/* TODO: do initialization */

	/* TODO: MCR */
	/* TODO: CTRL1 */
	/* TODO: FDCTRL */

	/* Enable auto-recovery from bus-off */
	/* config->base->CTRL1 &= ~(CAN_CTRL1_BOFFREC_MASK); */

	err = can_calc_timing(dev, &timing, config->common.bus_speed, config->common.sample_point);
	if (err == -EINVAL) {
		LOG_ERR("unable to calculate timing for bus-speed %u, sample-point %u",
			config->common.bus_speed, config->common.sample_point);
		return -ENODEV;
	}

	err = can_set_timing(dev, &timing);
	if (err != 0) {
		LOG_ERR("failed to set timing (err %d)", err);
		return -ENODEV;
	}

	if (UTIL_AND(IS_ENABLED(CONFIG_CAN_FLEXCAN_FD), config->flexcan_fd)) {
		err = can_calc_timing_data(dev, &timing, config->common.bus_speed_data,
					   config->common.sample_point_data);
		if (err == -EINVAL) {
			LOG_ERR("unable to calculate timing for bus-speed-data %u, "
				"sample-point-data %u", config->common.bus_speed_data,
				config->common.sample_point_data);
			return -ENODEV;
		}

		err = can_set_timing_data(dev, &timing);
		if (err != 0) {
			LOG_ERR("failed to set data phase timing (err %d)", err);
			return -ENODEV;
		}
	}

	/* TODO: initialize all MB control and status words */

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}

	/* FLEXCAN_GetDefaultConfig(&flexcan_config); */
	/* flexcan_config.maxMbNum = CAN_FLEXCAN_MAX_MB; */
	/* flexcan_config.clkSrc = config->clk_source; */
	/* flexcan_config.baudRate = clock_freq / */
	/*       (1U + data->timing.prop_seg + data->timing.phase_seg1 + */
	/*        data->timing.phase_seg2) / data->timing.prescaler; */

	/* flexcan_config.enableIndividMask = true; */
	/* flexcan_config.enableLoopBack = false; */
	/* flexcan_config.disableSelfReception = true; */
	/* Initialize in listen-only mode since FLEXCAN_{FD}Init() exits freeze mode */
	/* flexcan_config.enableListenOnlyMode = true; */

	/* flexcan_config.timingConfig.rJumpwidth = data->timing.sjw - 1U; */
	/* flexcan_config.timingConfig.propSeg = data->timing.prop_seg - 1U; */
	/* flexcan_config.timingConfig.phaseSeg1 = data->timing.phase_seg1 - 1U; */
	/* flexcan_config.timingConfig.phaseSeg2 = data->timing.phase_seg2 - 1U; */

#ifdef CONFIG_CAN_FLEXCAN_FD
	if (config->flexcan_fd) {
		/* flexcan_config.timingConfig.frJumpwidth = data->timing_data.sjw - 1U; */
		/* flexcan_config.timingConfig.fpropSeg = data->timing_data.prop_seg; */
		/* flexcan_config.timingConfig.fphaseSeg1 = data->timing_data.phase_seg1 - 1U; */
		/* flexcan_config.timingConfig.fphaseSeg2 = data->timing_data.phase_seg2 - 1U; */

		/* FLEXCAN_FDInit(config->base, &flexcan_config, clock_freq, kFLEXCAN_64BperMB,
		 * true); */
	} else {
#endif          /* CONFIG_CAN_FLEXCAN_FD */
		/* FLEXCAN_Init(config->base, &flexcan_config, clock_freq); */
#ifdef CONFIG_CAN_FLEXCAN_FD
	}
#endif /* CONFIG_CAN_FLEXCAN_FD */

	/* FLEXCAN_TransferCreateHandle(config->base, &data->handle, */
	/* 			     can_flexcan_transfer_callback, data); */

	/* Manually enter freeze mode, set normal mode, and clear error counters */
	/* FLEXCAN_EnterFreezeMode(config->base); */
	/* (void)can_flexcan_set_mode(dev, CAN_MODE_NORMAL); */
	/* config->base->ECR &= ~(CAN_ECR_TXERRCNT_MASK | CAN_ECR_RXERRCNT_MASK); */

	/* TODO: configure interrupts */

	config->irq_config_func(dev);

	/* (void)can_flexcan_get_state(dev, &data->state, NULL); */

	return 0;
}

__maybe_unused static const struct can_driver_api can_flexcan_driver_api = {
	.get_capabilities = can_flexcan_get_capabilities,
	.start = can_flexcan_start,
	.stop = can_flexcan_stop,
	.set_mode = can_flexcan_set_mode,
	.set_timing = can_flexcan_set_timing,
	.send = can_flexcan_send,
	.add_rx_filter = can_flexcan_add_rx_filter,
	.remove_rx_filter = can_flexcan_remove_rx_filter,
	.get_state = can_flexcan_get_state,
#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
	.recover = can_flexcan_recover,
#endif /* CONFIG_CAN_MANUAL_RECOVERY_MODE */
	.set_state_change_callback = can_flexcan_set_state_change_callback,
	.get_core_clock = can_flexcan_get_core_clock,
	.get_max_filters = can_flexcan_get_max_filters,
	/*
	 * FlexCAN timing limits are specified in the CAN Bit Timing Register (CBT) field
	 * descriptions" table in the SoC reference manual.
	 *
	 * Note that the values here are the "physical" timing limits, whereas the register field
	 * limits are physical values minus 1 (which is handled via CBT register assignments
	 * assignments elsewhere in this driver).
	 */
	.timing_min = {
		.sjw = 1,
		.prop_seg = 1,
		.phase_seg1 = 1,
		.phase_seg2 = 2,
		.prescaler = 1
	},
	.timing_max = {
		.sjw = 32,
		.prop_seg = 64,
		.phase_seg1 = 32,
		.phase_seg2 = 32,
		.prescaler = 1024
	}
};

#ifdef CONFIG_CAN_FLEXCAN_FD
static const struct can_driver_api can_flexcan_fd_driver_api = {
	.get_capabilities = can_flexcan_get_capabilities,
	.start = can_flexcan_start,
	.stop = can_flexcan_stop,
	.set_mode = can_flexcan_set_mode,
	.set_timing = can_flexcan_set_timing,
	.set_timing_data = can_flexcan_set_timing_data,
	.send = can_flexcan_send,
	.add_rx_filter = can_flexcan_add_rx_filter,
	.remove_rx_filter = can_flexcan_remove_rx_filter,
	.get_state = can_flexcan_get_state,
#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
	.recover = can_flexcan_recover,
#endif /* CONFIG_CAN_MANUAL_RECOVERY_MODE */
	.set_state_change_callback = can_flexcan_set_state_change_callback,
	.get_core_clock = can_flexcan_get_core_clock,
	.get_max_filters = can_flexcan_get_max_filters,
	/*
	 * FlexCAN timing limits are specified in the CAN Bit Timing Register (CBT) and CAN FD Bit
	 * Timing Register (FDCBT) field descriptions" table in the SoC reference manual.
	 *
	 * Note that the values here are the "physical" timing limits, whereas the register field
	 * limits are physical values minus 1 (which is handled via CBT/FDCBT register assignments
	 * assignments elsewhere in this driver).
	 */
	.timing_min = {
		.sjw = 1,
		.prop_seg = 1,
		.phase_seg1 = 1,
		.phase_seg2 = 2,
		.prescaler = 1
	},
	.timing_max = {
		.sjw = 32,
		.prop_seg = 64,
		.phase_seg1 = 32,
		.phase_seg2 = 32,
		.prescaler = 1024
	},
	.timing_data_min = {
		.sjw = 1,
		.prop_seg = 1,
		.phase_seg1 = 1,
		.phase_seg2 = 2,
		.prescaler = 1
	},
	.timing_data_max = {
		.sjw = 8,
		.prop_seg = 32,
		.phase_seg1 = 8,
		.phase_seg2 = 8,
		.prescaler = 1024
	},
};
#endif /* CONFIG_CAN_FLEXCAN_FD */

#define CAN_FLEXCAN_IRQ_CONFIG(node_id, prop, idx)                                                 \
	do {                                                                                       \
		IRQ_CONNECT(DT_IRQ_BY_IDX(node_id, idx, irq),                                      \
			    DT_IRQ_BY_IDX(node_id, idx, priority),                                 \
			    can_flexcan_isr,                                                       \
			    DEVICE_DT_GET(node_id),                                                \
			    0);                                                                    \
		irq_enable(DT_IRQ_BY_IDX(node_id, idx, irq));                                      \
	} while (false);

#ifdef CONFIG_CAN_FLEXCAN_FD
#define CAN_FLEXCAN_MAX_BITRATE(inst)                                                              \
	COND_CODE_1(DT_NODE_HAS_COMPAT(DT_DRV_INST(inst), CAN_FLEXCAN_FD_DRV_COMPAT), (8000000),   \
		    (1000000))
#else /* CONFIG_CAN_FLEXCAN_FD */
#define CAN_FLEXCAN_MAX_BITRATE(id) 1000000
#endif /* !CONFIG_CAN_FLEXCAN_FD */

#ifdef CONFIG_CAN_FLEXCAN_FD
#define CAN_FLEXCAN_DRIVER_API(inst)                                                               \
	COND_CODE_1(DT_NODE_HAS_COMPAT(DT_DRV_INST(inst), CAN_FLEXCAN_FD_DRV_COMPAT),              \
		    (can_flexcan_fd_driver_api), (can_flexcan_driver_api))
#else /* CONFIG_CAN_FLEXCAN_FD */
#define CAN_FLEXCAN_DRIVER_API(inst) can_flexcan_driver_api
#endif /* !CONFIG_CAN_FLEXCAN_FD */

#define CAN_FLEXCAN_DEVICE_INIT(inst)                                                              \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
	static void can_flexcan_irq_config_##inst(const struct device *dev)                        \
	{                                                                                          \
		DT_INST_FOREACH_PROP_ELEM(inst, interrupt_names, CAN_FLEXCAN_IRQ_CONFIG);          \
	}                                                                                          \
                                                                                                   \
	static const struct can_flexcan_config can_flexcan_config_##inst = {                       \
		.common = CAN_DT_DRIVER_CONFIG_INST_GET(inst, 0, CAN_FLEXCAN_MAX_BITRATE(inst)),   \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                             \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, name),           \
		.clk_source = DT_INST_PROP(inst, clk_source),                                      \
		.base = (mm_reg_t)DT_INST_REG_ADDR(inst),                                          \
		.irq_config_func = can_flexcan_irq_config_##inst,                                  \
		IF_ENABLED(CONFIG_CAN_FLEXCAN_FD,                                                  \
			(.flexcan_fd = DT_NODE_HAS_COMPAT(DT_DRV_INST(inst),                       \
							  CAN_FLEXCAN_FD_DRV_COMPAT),))            \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
	};                                                                                         \
                                                                                                   \
	static struct can_flexcan_data can_flexcan_data_##inst;                                    \
                                                                                                   \
	CAN_DEVICE_DT_INST_DEFINE(inst, can_flexcan_init, NULL, &can_flexcan_data_##inst,          \
				  &can_flexcan_config_##inst, POST_KERNEL,                         \
				  CONFIG_CAN_INIT_PRIORITY, &CAN_FLEXCAN_DRIVER_API(inst));

DT_INST_FOREACH_STATUS_OKAY(CAN_FLEXCAN_DEVICE_INIT)
