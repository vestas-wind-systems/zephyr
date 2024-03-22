/*
 * Copyright (c) 2019-2026 Vestas Wind Systems A/S
 * Copyright 2025-2026 NXP
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
#include <zephyr/spinlock.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/sys_io.h>

LOG_MODULE_REGISTER(can_flexcan, CONFIG_CAN_LOG_LEVEL);

/* TODO: Errata: */
/* git -C ../modules/hal/nxp/ grep -h -o -e 'FSL_FEATURE_FLEXCAN_HAS_ERRATA_[0-9]*' | sort -n | uniq
 */
/* 050443: RX MB read issue */
/* 052403: RX MB read issue */
/* 5641:   Skip MB0 */
/* 5829:   Skip MB0 */
/* 6032:   TX issue */
/* 8341:   Freeze mode issue */
/* 9595:   Freeze mode issue */

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
#define CAN_FLEXCAN_CTRL1          0x0004
#define CAN_FLEXCAN_CTRL1_PRESDIV  GENMASK(31, 24)
#define CAN_FLEXCAN_CTRL1_RJW      GENMASK(23, 22)
#define CAN_FLEXCAN_CTRL1_PSEG1    GENMASK(21, 19)
#define CAN_FLEXCAN_CTRL1_PSEG2    GENMASK(18, 16)
#define CAN_FLEXCAN_CTRL1_BOFFMSK  BIT(15)
#define CAN_FLEXCAN_CTRL1_ERRMSK   BIT(14)
#define CAN_FLEXCAN_CTRL1_CLKSRC   BIT(13)
#define CAN_FLEXCAN_CTRL1_LPB      BIT(12)
#define CAN_FLEXCAN_CTRL1_TWRNMSK  BIT(11)
#define CAN_FLEXCAN_CTRL1_RWRNMSK  BIT(10)
#define CAN_FLEXCAN_CTRL1_SMP      BIT(7)
#define CAN_FLEXCAN_CTRL1_BOFFREC  BIT(6)
#define CAN_FLEXCAN_CTRL1_TSYN     BIT(5)
#define CAN_FLEXCAN_CTRL1_LBUF     BIT(4)
#define CAN_FLEXCAN_CTRL1_LOM      BIT(3)
#define CAN_FLEXCAN_CTRL1_PROP_SEG GENMASK(2, 0)

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

/* Error and Status 1 Register (ESR1) (*_FAST fields only present in FlexCAN FD) */
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

/* Error and Status 1 Register (ESR1) Fault Confinement (FLTCONF) State codes */
#define CAN_FLEXCAN_ESR1_FLTCONF_ERROR_ACTIVE  0U
#define CAN_FLEXCAN_ESR1_FLTCONF_ERROR_PASSIVE 1U
#define CAN_FLEXCAN_ESR1_FLTCONF_BUS_OFF_BIT   BIT(1)

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

/* CAN Bit Timing Register (not present in all instances) */
#define CAN_FLEXCAN_CBT          0x0050
#define CAN_FLEXCAN_CBT_BTF      BIT(31)
#define CAN_FLEXCAN_CBT_EPRESDIV GENMASK(30, 21)
#define CAN_FLEXCAN_CBT_ERJW     GENMASK(20, 16)
#define CAN_FLEXCAN_CBT_EPROPSEG GENMASK(15, 10)
#define CAN_FLEXCAN_CBT_EPSEG1   GENMASK(9, 5)
#define CAN_FLEXCAN_CBT_EPSEG2   GENMASK(4, 0)

/* Interrupt Masks 4 Register (BUF127TO96M) */
#define CAN_FLEXCAN_IMASK4 0x0068

/* Interrupt Masks 3 Register (BUF95TO64M) */
#define CAN_FLEXCAN_IMASK3 0x006c

/* Interrupt Flags 4 Register (BUF127TO96M) */
#define CAN_FLEXCAN_IFLAG4 0x0070

/* Interrupt Flags 3 Register (BUF95TO64I) */
#define CAN_FLEXCAN_IFLAG3 0x0074

/* Message Buffers (MBs) base address */
#define CAN_FLEXCAN_MB_BASE 0x0080

/* Message Buffer (MB) Control and Status (CS) register offset */
#define CAN_FLEXCAN_MB_CS           0x0000
#define CAN_FLEXCAN_MB_CS_EDL       BIT(31)
#define CAN_FLEXCAN_MB_CS_BRS       BIT(30)
#define CAN_FLEXCAN_MB_CS_ESI       BIT(29)
#define CAN_FLEXCAN_MB_CS_CODE      GENMASK(27, 24)
#define CAN_FLEXCAN_MB_CS_SRR       BIT(22)
#define CAN_FLEXCAN_MB_CS_IDE       BIT(21)
#define CAN_FLEXCAN_MB_CS_RTR       BIT(20)
#define CAN_FLEXCAN_MB_CS_DLC       GENMASK(19, 16)
#define CAN_FLEXCAN_MB_CS_TIMESTAMP GENMASK(15, 0)

/* Message Buffer (MB) Control and Status (CS) codes */
#define CAN_FLEXCAN_MB_CS_CODE_RX_INACTIVE  0U
#define CAN_FLEXCAN_MB_CS_CODE_RX_BUSY_FLAG 1U
#define CAN_FLEXCAN_MB_CS_CODE_RX_FULL      2U
#define CAN_FLEXCAN_MB_CS_CODE_RX_EMPTY     4U
#define CAN_FLEXCAN_MB_CS_CODE_TX_INACTIVE  8U
#define CAN_FLEXCAN_MB_CS_CODE_TX_ABORT     9U
#define CAN_FLEXCAN_MB_CS_CODE_RX_RANSWER   10U
#define CAN_FLEXCAN_MB_CS_CODE_TX_DATA      12U /* MB RTR = 0 */
#define CAN_FLEXCAN_MB_CS_CODE_TX_REMOTE    12U /* MB RTR = 1 */
#define CAN_FLEXCAN_MB_CS_CODE_TX_TANSWER   14U

/* Message Buffer (MB) ID register offset */
#define CAN_FLEXCAN_MB_ID        0x0004
#define CAN_FLEXCAN_MB_ID_PRIO   GENMASK(31, 29)
#define CAN_FLEXCAN_MB_ID_STD_ID GENMASK(28, 18)
#define CAN_FLEXCAN_MB_ID_EXT_ID GENMASK(28, 0)

/* Message Buffer (MB) data offset */
#define CAN_FLEXCAN_MB_DATA 0x0008

/* Rx Individual Mask Registers base (RXIMRn) */
#define CAN_FLEXCAN_RXIMR_BASE    0x0880

/* CAN FD Control Register (FlexCAN FD only) */
#define CAN_FLEXCAN_FDCTRL         0x0c00
#define CAN_FLEXCAN_FDCTRL_FDRATE  BIT(31)
#define CAN_FLEXCAN_FDCTRL_MBDSR3  GENMASK(26, 25)
#define CAN_FLEXCAN_FDCTRL_MBDSR2  GENMASK(23, 22)
#define CAN_FLEXCAN_FDCTRL_MBDSR1  GENMASK(20, 19)
#define CAN_FLEXCAN_FDCTRL_MBDSR0  GENMASK(17, 16)
#define CAN_FLEXCAN_FDCTRL_TDCEN   BIT(15)
#define CAN_FLEXCAN_FDCTRL_TDCFAIL BIT(14)
#define CAN_FLEXCAN_FDCTRL_TDCOFF  GENMASK(12, 8)
#define CAN_FLEXCAN_FDCTRL_TDCVAL  GENMASK(5, 0)

/* CAN FD Control Register Message Buffer Data Sizes */
#define CAN_FLEXCAN_FDCTRL_MBDS_8BYTES  0u
#define CAN_FLEXCAN_FDCTRL_MBDS_16BYTES 1u
#define CAN_FLEXCAN_FDCTRL_MBDS_32BYTES 2u
#define CAN_FLEXCAN_FDCTRL_MBDS_64BYTES 3u

/* CAN FD Bit Timing Register (FlexCAN FD only) */
#define CAN_FLEXCAN_FDCBT          0x0c04
#define CAN_FLEXCAN_FDCBT_FPRESDIV GENMASK(29, 20)
#define CAN_FLEXCAN_FDCBT_FRJW     GENMASK(18, 16)
#define CAN_FLEXCAN_FDCBT_FPROPSEG GENMASK(14, 10)
#define CAN_FLEXCAN_FDCBT_FPSEG1   GENMASK(7, 5)
#define CAN_FLEXCAN_FDCBT_FPSEG2   GENMASK(2, 0)

/* CAN FD CRC Register (FlexCAN FD only) */
#define CAN_FLEXCAN_FDCRC          0x0c08
#define CAN_FLEXCAN_FDCRC_FD_MBCRC GENMASK(30, 24)
#define CAN_FLEXCAN_FDCRC_FD_TXCRC GENMASK(20, 0)

/* TODO: support ECC registers (present in e.g. S32K3xx)? */
/* TODO: support Enhanced registers (present in e.g. S32K3xx)? */
/* TODO: support enhanced bit timing registers */

/* Common fields for all FlexCAN RX masks (RXMGMASK, RXIMRn, ...) */
#define CAN_FLEXCAN_RX_MASK_RTR    BIT(31)
#define CAN_FLEXCAN_RX_MASK_IDE    BIT(30)
#define CAN_FLEXCAN_RX_MASK_STD_ID GENMASK(28, 18)
#define CAN_FLEXCAN_RX_MASK_EXT_ID GENMASK(28, 0)

/* Mode change timeout */
#define CAN_FLEXCAN_TIMEOUT_US 1000U

struct can_flexcan_mb_header {
	uint32_t cs;
	uint32_t id;
};

struct can_flexcan_rx_callback {
	can_rx_callback_t callback;
	void *user_data;
};

struct can_flexcan_tx_callback {
	can_tx_callback_t callback;
	void *user_data;
};

struct can_flexcan_config {
	const struct can_driver_config common;
	DEVICE_MMIO_NAMED_ROM(reg_base);
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	uint32_t clk_source;
	uint32_t number_of_mb;
	void (*irq_config_func)(const struct device *dev);
	const struct pinctrl_dev_config *pincfg;
	struct can_flexcan_rx_callback *rx_cbs;
	struct can_flexcan_tx_callback *tx_cbs;
	uint8_t num_rx_cbs;
	uint8_t num_tx_cbs;
	uint8_t maxmb;
	uint8_t max_filters;
#ifdef CONFIG_CAN_FLEXCAN_FD
	bool flexcan_fd;
#endif /* CONFIG_CAN_FLEXCAN_FD */
};

struct can_flexcan_data {
	struct can_driver_data common;
	DEVICE_MMIO_NAMED_RAM(reg_base);
	struct k_spinlock lock;
	struct k_sem rx_sem;
	struct k_sem tx_sem;
	enum can_state state;
};

/* MMIO helpers */
#define DEV_CFG(_dev)  ((const struct can_flexcan_config *)(_dev)->config)
#define DEV_DATA(_dev) ((struct can_flexcan_data *)(_dev)->data)

/* Forward declarations */
static int can_flexcan_get_state(const struct device *dev, enum can_state *state,
				 struct can_bus_err_cnt *err_cnt);

static inline void can_flexcan_write_reg(const struct device *dev, uint16_t reg, uint32_t val)
{
	sys_write32(val, DEVICE_MMIO_NAMED_GET(dev, reg_base) + reg);
}

static inline uint32_t can_flexcan_read_reg(const struct device *dev, uint16_t reg)
{
	return sys_read32(DEVICE_MMIO_NAMED_GET(dev, reg_base) + reg);
}

/* TODO: perhaps just fold this into get_state()? */
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

static void can_flexcan_write_rximr(const struct device *dev, uint8_t idx, uint32_t rximr)
{
	__maybe_unused const struct can_flexcan_config *config = dev->config;
	mem_addr_t rximr_base;

	__ASSERT_NO_MSG(idx < config->number_of_mb);

	rximr_base = DEVICE_MMIO_NAMED_GET(dev, reg_base) + CAN_FLEXCAN_RXIMR_BASE +
		(idx * sizeof(uint32_t));
	sys_write32(rximr, rximr_base);
}

static mem_addr_t can_flexcan_get_mb_base(const struct device *dev, uint8_t mbidx)
{
	__maybe_unused const struct can_flexcan_config *config = dev->config;
	mem_addr_t mb_base;

	__ASSERT_NO_MSG(mbidx < config->number_of_mb);

	/* TODO: add support for 64 byte CAN FD MBs */
	mb_base = DEVICE_MMIO_NAMED_GET(dev, reg_base) + CAN_FLEXCAN_MB_BASE + (mbidx * 16U);

	return mb_base;
}

static void can_flexcan_write_mb(const struct device *dev, uint8_t mbidx,
				 const struct can_flexcan_mb_header *mbhdr, const uint8_t *data,
				 size_t len)
{
	mem_addr_t mb_base = can_flexcan_get_mb_base(dev, mbidx);

	if (mbhdr != NULL) {
		sys_write32(mbhdr->cs, mb_base + CAN_FLEXCAN_MB_CS);
		sys_write32(mbhdr->id, mb_base + CAN_FLEXCAN_MB_ID);
	}

	if (data != NULL) {
		/* TODO: assert on len and MB size */
		for (size_t i = 0U; i < len; i++) {
			sys_write8(data[i], mb_base + CAN_FLEXCAN_MB_DATA + i);
		}
	}
}

static void can_flexcan_read_mb(const struct device *dev, uint8_t mbidx,
				struct can_flexcan_mb_header *mbhdr, uint8_t *data, size_t len)
{
	mem_addr_t mb_base = can_flexcan_get_mb_base(dev, mbidx);

	if (mbhdr != NULL) {
		mbhdr->cs = sys_read32(mb_base + CAN_FLEXCAN_MB_CS);
		mbhdr->id = sys_read32(mb_base + CAN_FLEXCAN_MB_ID);
	}

	if (data != NULL) {
		/* TODO: assert on len and MB size */
		for (size_t i = 0U; i < len; i++) {
			data[i] = sys_read8(mb_base + CAN_FLEXCAN_MB_DATA + i);
		}
	}
}

static void can_flexcan_mb_irq_enable(const struct device *dev, uint8_t mbidx, bool enable)
{
	static const uint16_t imasks[] = {CAN_FLEXCAN_IMASK1, CAN_FLEXCAN_IMASK2,
					  CAN_FLEXCAN_IMASK3, CAN_FLEXCAN_IMASK4};
	__maybe_unused const struct can_flexcan_config *config = dev->config;
	struct can_flexcan_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t imask_idx;
	uint32_t imask_bit;
	uint32_t imask;

	__ASSERT_NO_MSG(mbidx < config->number_of_mb);

	imask_idx = mbidx / NUM_BITS(uint32_t);
	imask_bit = mbidx % NUM_BITS(uint32_t);

	__ASSERT_NO_MSG(imask_idx < ARRAY_SIZE(imasks));

	key = k_spin_lock(&data->lock);

	imask = can_flexcan_read_reg(dev, imasks[imask_idx]);

	if (enable) {
		imask |= BIT(imask_bit);
	} else {
		imask &= ~BIT(imask_bit);
	}

	can_flexcan_write_reg(dev, imasks[imask_idx], imask);

	k_spin_unlock(&data->lock, key);
}

static void can_flexcan_mb_irq_ack(const struct device *dev, uint8_t mbidx)
{
	static const uint16_t iflags[] = {CAN_FLEXCAN_IFLAG1, CAN_FLEXCAN_IFLAG2,
					  CAN_FLEXCAN_IFLAG3, CAN_FLEXCAN_IFLAG4};
	__maybe_unused const struct can_flexcan_config *config = dev->config;
	uint32_t iflag_idx;
	uint32_t iflag_bit;

	__ASSERT_NO_MSG(mbidx < config->number_of_mb);

	iflag_idx = mbidx / NUM_BITS(uint32_t);
	iflag_bit = mbidx % NUM_BITS(uint32_t);

	__ASSERT_NO_MSG(iflag_idx < ARRAY_SIZE(iflags));

	can_flexcan_write_reg(dev, iflags[iflag_idx], BIT(iflag_bit));
}

static int can_flexcan_exit_low_power_mode(const struct device *dev, uint32_t timeout_us)
{
	struct can_flexcan_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t mcr;

	key = k_spin_lock(&data->lock);

	mcr = can_flexcan_read_reg(dev, CAN_FLEXCAN_MCR);
	mcr &= ~(CAN_FLEXCAN_MCR_MDIS);
	can_flexcan_write_reg(dev, CAN_FLEXCAN_MCR, mcr);

	k_spin_unlock(&data->lock, key);

	if (!WAIT_FOR((can_flexcan_read_reg(dev, CAN_FLEXCAN_MCR) & CAN_FLEXCAN_MCR_LPMACK) == 0U,
		      timeout_us, k_busy_wait(1U))) {
		LOG_WRN("timeout on exiting low-power mode (timeout %u us)", timeout_us);
		return -EAGAIN;
	}

	return 0;
}

static int can_flexcan_soft_reset(const struct device *dev, uint32_t timeout_us)
{
	struct can_flexcan_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t mcr;

	key = k_spin_lock(&data->lock);

	mcr = can_flexcan_read_reg(dev, CAN_FLEXCAN_MCR);
	mcr |= CAN_FLEXCAN_MCR_SOFTRST;
	can_flexcan_write_reg(dev, CAN_FLEXCAN_MCR, mcr);

	k_spin_unlock(&data->lock, key);

	if (!WAIT_FOR((can_flexcan_read_reg(dev, CAN_FLEXCAN_MCR) & CAN_FLEXCAN_MCR_SOFTRST) == 0U,
		      timeout_us, k_busy_wait(1U))) {
		LOG_WRN("timeout on soft-resetting (timeout %u us)", timeout_us);
		return -EAGAIN;
	}

	return 0;
}

static int can_flexcan_enter_freeze_mode(const struct device *dev, uint32_t timeout_us)
{
	struct can_flexcan_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t mcr;

	/* TODO: handle errata 8341 and 9595 */

	key = k_spin_lock(&data->lock);

	mcr = can_flexcan_read_reg(dev, CAN_FLEXCAN_MCR);
	mcr |= CAN_FLEXCAN_MCR_FRZ;
	can_flexcan_write_reg(dev, CAN_FLEXCAN_MCR, mcr);

	mcr |= CAN_FLEXCAN_MCR_HALT;
	can_flexcan_write_reg(dev, CAN_FLEXCAN_MCR, mcr);

	k_spin_unlock(&data->lock, key);

	if (!WAIT_FOR((can_flexcan_read_reg(dev, CAN_FLEXCAN_MCR) & CAN_FLEXCAN_MCR_FRZACK) != 0U,
		      timeout_us, k_busy_wait(1U))) {
		LOG_WRN("timeout on entering freeze mode (timeout %u us)", timeout_us);
		return -EAGAIN;
	}

	return 0;
}

static int can_flexcan_exit_freeze_mode(const struct device *dev, uint32_t timeout_us)
{
	struct can_flexcan_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t mcr;

	key = k_spin_lock(&data->lock);

	mcr = can_flexcan_read_reg(dev, CAN_FLEXCAN_MCR);
	mcr &= ~(CAN_FLEXCAN_MCR_HALT);
	can_flexcan_write_reg(dev, CAN_FLEXCAN_MCR, mcr);

	mcr &= ~(CAN_FLEXCAN_MCR_FRZ);
	can_flexcan_write_reg(dev, CAN_FLEXCAN_MCR, mcr);

	k_spin_unlock(&data->lock, key);

	if (!WAIT_FOR((can_flexcan_read_reg(dev, CAN_FLEXCAN_MCR) & CAN_FLEXCAN_MCR_FRZACK) == 0U,
		      timeout_us, k_busy_wait(1U))) {
		LOG_WRN("timeout on exiting freeze mode (timeout %u us)", timeout_us);
		return -EAGAIN;
	}

	return 0;
}

static int can_flexcan_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	__maybe_unused const struct can_flexcan_config *config = dev->config;

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
	can_flexcan_write_reg(dev, CAN_FLEXCAN_ECR, 0U);

	err = can_flexcan_exit_freeze_mode(dev, CAN_FLEXCAN_TIMEOUT_US);
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

	/* TODO: Abort any pending TX frames before entering freeze mode */

	err = can_flexcan_enter_freeze_mode(dev, CAN_FLEXCAN_TIMEOUT_US);
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

	data->common.started = false;

	return 0;
}

static int can_flexcan_set_mode(const struct device *dev, can_mode_t mode)
{
	__maybe_unused const struct can_flexcan_config *config = dev->config;
	struct can_flexcan_data *data = dev->data;
	k_spinlock_key_t key;
	can_mode_t supported;
	uint32_t ctrl1;
	uint32_t mcr;

	if (data->common.started) {
		return -EBUSY;
	}

	(void)can_flexcan_get_capabilities(dev, &supported);

	if ((mode & ~(supported)) != 0) {
		LOG_ERR("unsupported mode: 0x%08x", mode);
		return -ENOTSUP;
	}

	if ((mode & CAN_MODE_FD) != 0 && (mode & CAN_MODE_3_SAMPLES) != 0) {
		LOG_ERR("triple sampling is not supported in CAN FD mode");
		return -ENOTSUP;
	}

	key = k_spin_lock(&data->lock);

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
		uint32_t fdctrl;

		if ((mode & CAN_MODE_FD) != 0) {
			/* Enable CAN FD mode */
			mcr |= CAN_FLEXCAN_MCR_FDEN;

			fdctrl = can_flexcan_read_reg(dev, CAN_FLEXCAN_FDCTRL);

			/* Transceiver Delay Compensation (TDC) must be disabled in loopback mode */
			if ((mode & CAN_MODE_LOOPBACK) != 0) {
				fdctrl &= ~(CAN_FLEXCAN_FDCTRL_TDCEN);
			} else {
				fdctrl |= CAN_FLEXCAN_FDCTRL_TDCEN;
			}

			can_flexcan_write_reg(dev, CAN_FLEXCAN_FDCTRL, fdctrl);
		} else {
			/* Disable CAN FD mode */
			mcr &= ~(CAN_FLEXCAN_MCR_FDEN);
		}
	}

	can_flexcan_write_reg(dev, CAN_FLEXCAN_CTRL1, ctrl1);
	can_flexcan_write_reg(dev, CAN_FLEXCAN_MCR, mcr);

	data->common.mode = mode;

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int can_flexcan_set_timing(const struct device *dev, const struct can_timing *timing)
{
	__maybe_unused const struct can_flexcan_config *config = dev->config;
	struct can_flexcan_data *data = dev->data;

	if (!timing) {
		return -EINVAL;
	}

	if (data->common.started) {
		return -EBUSY;
	}

	if (UTIL_AND(IS_ENABLED(CONFIG_CAN_FLEXCAN_FD), config->flexcan_fd)) {
		/* Use the CAN Bit Timing (CBT) register for CAN FD capable instances */
		uint32_t cbt;

		cbt = CAN_FLEXCAN_CBT_BTF |
		      FIELD_PREP(CAN_FLEXCAN_CBT_EPRESDIV, timing->prescaler - 1U) |
		      FIELD_PREP(CAN_FLEXCAN_CBT_ERJW, timing->sjw - 1U) |
		      FIELD_PREP(CAN_FLEXCAN_CBT_EPROPSEG, timing->prop_seg - 1U) |
		      FIELD_PREP(CAN_FLEXCAN_CBT_EPSEG1, timing->phase_seg1 - 1U) |
		      FIELD_PREP(CAN_FLEXCAN_CBT_EPSEG2, timing->phase_seg2 - 1U);

		can_flexcan_write_reg(dev, CAN_FLEXCAN_CBT, cbt);
	} else {
		/* Use the CTRL1 register for non-CAN FD capable instances */
		k_spinlock_key_t key;
		uint32_t ctrl1;

		key = k_spin_lock(&data->lock);

		ctrl1 = can_flexcan_read_reg(dev, CAN_FLEXCAN_CTRL1);

		ctrl1 &= ~(CAN_FLEXCAN_CTRL1_PRESDIV | CAN_FLEXCAN_CTRL1_RJW |
			   CAN_FLEXCAN_CTRL1_PSEG1 | CAN_FLEXCAN_CTRL1_PSEG2);

		ctrl1 |= FIELD_PREP(CAN_FLEXCAN_CTRL1_PRESDIV, timing->prescaler - 1U) |
			 FIELD_PREP(CAN_FLEXCAN_CTRL1_RJW, timing->sjw - 1U) |
			 FIELD_PREP(CAN_FLEXCAN_CTRL1_PSEG1, timing->phase_seg1 - 1U) |
			 FIELD_PREP(CAN_FLEXCAN_CTRL1_PSEG2, timing->phase_seg2 - 1U) |
			 FIELD_PREP(CAN_FLEXCAN_CTRL1_PROP_SEG, timing->prop_seg - 1U);

		can_flexcan_write_reg(dev, CAN_FLEXCAN_CTRL1, ctrl1);

		k_spin_unlock(&data->lock, key);
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

	if (k_sem_take(&data->tx_sem, timeout) != 0) {
		return -EAGAIN;
	}

	/* TODO: locate available TX MB */

	/* TODO: store callback and user data */

	/* TODO: enqueue TX frame */

	/* TODO: enable TX MB IRQ */

	return 0;
}

static int can_flexcan_add_rx_filter(const struct device *dev, can_rx_callback_t callback,
				     void *user_data, const struct can_filter *filter)
{
	const struct can_flexcan_config *config = dev->config;
	struct can_flexcan_data *data = dev->data;
	struct can_flexcan_mb_header mbhdr = {0};
	uint32_t rximr = CAN_FLEXCAN_RX_MASK_IDE;
	int filter_id = -ENOSPC;
	uint8_t mbidx;
	int err;

	__ASSERT_NO_MSG(callback);

	if ((filter->flags & ~(CAN_FILTER_IDE)) != 0) {
		LOG_ERR("unsupported CAN filter flags 0x%02x", filter->flags);
		return -ENOTSUP;
	}

	mbhdr.cs = FIELD_PREP(CAN_FLEXCAN_MB_CS_CODE, CAN_FLEXCAN_MB_CS_CODE_RX_EMPTY);

	if (!IS_ENABLED(CONFIG_CAN_ACCEPT_RTR)) {
		rximr |= CAN_FLEXCAN_RX_MASK_RTR;
	}

	if ((filter->flags & CAN_FILTER_IDE) != 0U) {
		mbhdr.cs |= CAN_FLEXCAN_MB_CS_IDE;
		mbhdr.id |= FIELD_PREP(CAN_FLEXCAN_MB_ID_EXT_ID, filter->id);
		rximr |= FIELD_PREP(CAN_FLEXCAN_RX_MASK_EXT_ID, filter->mask);
	} else {
		mbhdr.id |= FIELD_PREP(CAN_FLEXCAN_MB_ID_STD_ID, filter->id);
		rximr |= FIELD_PREP(CAN_FLEXCAN_RX_MASK_STD_ID, filter->mask);
	}

	k_sem_take(&data->rx_sem, K_FOREVER);

	for (int i = 0U; i < config->num_rx_cbs; i++) {
		if (config->rx_cbs[i].callback == NULL) {
			filter_id = i;
			break;
		}
	}

	if (filter_id == -ENOSPC) {
		LOG_WRN("no free RX filters");
		goto unlock;
	}

	/* TODO: map between filter_id and mbidx */
	mbidx = filter_id;

	/*
	 * The Initialize Receive Individual Mask Registers (RXIMRn) can only be written in freeze
	 * mode.
	 */
	err = can_flexcan_enter_freeze_mode(dev, CAN_FLEXCAN_TIMEOUT_US);
	if (err != 0) {
		filter_id = -EIO;
		goto unlock;
	}

	can_flexcan_write_mb(dev, mbidx, &mbhdr, NULL, 0U);
	can_flexcan_write_rximr(dev, mbidx, rximr);

	config->rx_cbs[filter_id].user_data = user_data;
	config->rx_cbs[filter_id].callback = callback;

	/* TODO: clear iflag? */
	can_flexcan_mb_irq_enable(dev, mbidx, true);

	if (data->common.started) {
		err = can_flexcan_exit_freeze_mode(dev, CAN_FLEXCAN_TIMEOUT_US);
		if (err != 0) {
			filter_id = -EIO;
			config->rx_cbs[filter_id].user_data = NULL;
			config->rx_cbs[filter_id].callback = NULL;
			goto unlock;
		}
	}

unlock:
	k_sem_give(&data->rx_sem);

	return filter_id;
}

static void can_flexcan_remove_rx_filter(const struct device *dev, int filter_id)
{
	const struct can_flexcan_config *config = dev->config;
	struct can_flexcan_data *data = dev->data;
	struct can_flexcan_mb_header mbhdr = {0};
	uint8_t mbidx;

	if (filter_id < 0 || filter_id >= config->num_rx_cbs) {
		LOG_ERR("filter ID %d out of bounds", filter_id);
		return;
	}

	/* TODO: map between filter_id and mbidx */
	mbidx = filter_id;

	k_sem_take(&data->rx_sem, K_FOREVER);

	config->rx_cbs[filter_id].user_data = NULL;
	config->rx_cbs[filter_id].callback = NULL;

	can_flexcan_mb_irq_enable(dev, mbidx, false);

	/* TODO: wait for MB to become non-busy */

	mbhdr.cs = FIELD_PREP(CAN_FLEXCAN_MB_CS_CODE, CAN_FLEXCAN_MB_CS_CODE_RX_INACTIVE);
	mbhdr.id = 0U;

	can_flexcan_write_mb(dev, mbidx, &mbhdr, NULL, 0U);

	k_sem_give(&data->rx_sem);
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

	/* TODO: What does this do? */
	/* config->base->CTRL1 |= CAN_CTRL1_BOFFREC_MASK; */

	return ret;
}
#endif /* CONFIG_CAN_MANUAL_RECOVERY_MODE */

static int can_flexcan_get_state(const struct device *dev, enum can_state *state,
				 struct can_bus_err_cnt *err_cnt)
{
	struct can_flexcan_data *data = dev->data;
	uint32_t fltconf;
	uint32_t esr1;
	uint32_t ecr;

	if (err_cnt != NULL) {
		ecr = can_flexcan_read_reg(dev, CAN_FLEXCAN_ECR);
		err_cnt->rx_err_cnt = FIELD_GET(CAN_FLEXCAN_ECR_RXERRCNT, ecr);
		err_cnt->tx_err_cnt = FIELD_GET(CAN_FLEXCAN_ECR_TXERRCNT, ecr);
	}

	if (state != NULL) {
		if (!data->common.started) {
			*state = CAN_STATE_STOPPED;
			return 0;
		}

		esr1 = can_flexcan_read_esr1(dev);
		fltconf = FIELD_GET(CAN_FLEXCAN_ESR1_FLTCONF, esr1);

		if ((fltconf & CAN_FLEXCAN_ESR1_FLTCONF_BUS_OFF_BIT) != 0U) {
			*state = CAN_STATE_BUS_OFF;
		} else if (fltconf == CAN_FLEXCAN_ESR1_FLTCONF_ERROR_PASSIVE) {
			*state = CAN_STATE_ERROR_PASSIVE;
		} else if ((esr1 & (CAN_FLEXCAN_ESR1_TXWRN | CAN_FLEXCAN_ESR1_RXWRN)) != 0U) {
			*state = CAN_STATE_ERROR_WARNING;
		} else {
			*state = CAN_STATE_ERROR_ACTIVE;
		}
	}

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

static int can_flexcan_get_core_clock(const struct device *dev, uint32_t *rate)
{
	const struct can_flexcan_config *config = dev->config;

	return clock_control_get_rate(config->clock_dev, config->clock_subsys, rate);
}

static int can_flexcan_get_max_filters(const struct device *dev, bool ide)
{
	const struct can_flexcan_config *config = dev->config;

	ARG_UNUSED(ide);

	return config->max_filters;
}

#ifdef CONFIG_CAN_FLEXCAN_FD
static int can_flexcan_set_timing_data(const struct device *dev,
				       const struct can_timing *timing_data)
{
	const uint32_t tdcoff_max = FIELD_GET(CAN_FLEXCAN_FDCTRL_TDCOFF, CAN_FLEXCAN_FDCTRL_TDCOFF);
	struct can_flexcan_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t fdctrl;
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

	key = k_spin_lock(&data->lock);

	fdctrl = can_flexcan_read_reg(dev, CAN_FLEXCAN_FDCTRL);

	fdctrl &= ~(CAN_FDCTRL_TDCOFF_MASK);
	fdctrl |= FIELD_PREP(CAN_FDCTRL_TDCOFF_MASK, CAN_CALC_TDCO((timing_data), 1U, tdcoff_max));

	can_flexcan_write_reg(dev, CAN_FLEXCAN_FDCTRL, fdctrl);

	k_spin_unlock(&data->lock, key);

	return 0;
}
#endif /* CONFIG_CAN_FLEXCAN_FD */

static void can_flexcan_handle_mb_irq(const struct device *dev, uint8_t mbidx)
{
	struct can_flexcan_mb_header mbhdr;

	/* TODO: RX: check that the MB is not busy */
	can_flexcan_read_mb(dev, mbidx, &mbhdr, NULL, 0U);

	/* TODO: RX: read MB data, if any */

	can_flexcan_mb_irq_ack(dev, mbidx);

	/* TODO: RX: Unlock the MB by reading the free-running timer */
	(void)can_flexcan_read_reg(dev, CAN_FLEXCAN_TIMER);

	/* TODO: process MB */

	LOG_INF("mbidx = %d, cs = 0x%08x, id = 0x%08x", mbidx, mbhdr.cs, mbhdr.id);
}

static void can_flexcan_isr(const struct device *dev)
{
	static const uint16_t iflags[] = {CAN_FLEXCAN_IFLAG1, CAN_FLEXCAN_IFLAG2,
					  CAN_FLEXCAN_IFLAG3, CAN_FLEXCAN_IFLAG4};
	const struct can_flexcan_config *config = dev->config;
	const uint32_t iflag_max = config->maxmb / NUM_BITS(uint32_t);
	uint32_t iflag;
	uint32_t esr1;
	uint8_t mbidx;
	int idx = 0;

	/* TODO: mask with IFLAGn */
	do {
		__ASSERT_NO_MSG(idx < ARRAY_SIZE(iflags));

		iflag = can_flexcan_read_reg(dev, iflags[idx]);

		if (iflag != 0U) {
			/* TODO: optimize this to only iterate the bits needed? */
			for (uint8_t bit = 0U; bit < NUM_BITS(uint32_t); bit++) {
				if ((iflag & BIT(bit)) != 0U) {
					mbidx = idx * NUM_BITS(uint32_t) + bit;
					can_flexcan_handle_mb_irq(dev, mbidx);
				}
			}
		}
	} while (++idx <= iflag_max);

	/* TODO: just call get_state here, which will read ESR1 and update stats */
	/* TODO: check state against last stored state and call callback if changed */
	/* TODO: if state is bus-off, abort all pending TX MBs */
	esr1 = can_flexcan_read_esr1(dev);
	can_flexcan_write_reg(dev, CAN_FLEXCAN_ESR1, esr1);

	LOG_INF("esr1 = 0x%08x", esr1);
}

static int can_flexcan_init(const struct device *dev)
{
	const struct can_flexcan_config *config = dev->config;
	struct can_flexcan_data *data = dev->data;
	struct can_flexcan_mb_header mbhdr = {0};
	struct can_timing timing = {0};
	uint32_t ctrl1;
	uint32_t mcr;
	int err;

	LOG_DBG("%s: maxmb=%d, num_rx_cbs=%d, num_tx_cbs=%d", dev->name, config->maxmb,
		config->num_rx_cbs, config->num_tx_cbs);

	DEVICE_MMIO_NAMED_MAP(dev, reg_base, K_MEM_CACHE_NONE | K_MEM_DIRECT_MAP);

	k_sem_init(&data->rx_sem, 1U, 1U);
	k_sem_init(&data->tx_sem, config->num_tx_cbs, config->num_tx_cbs);

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

	err = clock_control_configure(config->clock_dev, config->clock_subsys, NULL);
	if (err != 0) {
		if (err != -ENOSYS) {
			LOG_ERR("failed to configure clock (err %d)", err);
			return -ENODEV;
		}

		LOG_WRN("clock configuration not implemented, using defaults");
	}

	err = clock_control_on(config->clock_dev, config->clock_subsys);
	if (err != 0) {
		LOG_ERR("failed to enable clock (err %d)", err);
		return -ENODEV;
	}

	err = can_flexcan_exit_low_power_mode(dev, CAN_FLEXCAN_TIMEOUT_US);
	if (err != 0) {
		return -ENODEV;
	}

	err = can_flexcan_soft_reset(dev, CAN_FLEXCAN_TIMEOUT_US);
	if (err != 0) {
		return -ENODEV;
	}

	/* TODO: reset any other registers here? */

	/* Module Configuration Register (MCR) */
	mcr = can_flexcan_read_reg(dev, CAN_FLEXCAN_MCR);
	mcr &= ~(CAN_FLEXCAN_MCR_MAXMB);
	mcr |= CAN_FLEXCAN_MCR_WRNEN | CAN_FLEXCAN_MCR_SRXDIS | CAN_FLEXCAN_MCR_IRMQ |
	       CAN_FLEXCAN_MCR_AEN | FIELD_PREP(CAN_FLEXCAN_MCR_MAXMB, config->maxmb);
	can_flexcan_write_reg(dev, CAN_FLEXCAN_MCR, mcr);

	/* Control 1 Register (CTRL1) */
	ctrl1 = can_flexcan_read_reg(dev, CAN_FLEXCAN_CTRL1);
	ctrl1 &= ~(CAN_CTRL1_BOFFREC_MASK);
	ctrl1 |= CAN_FLEXCAN_CTRL1_BOFFMSK | CAN_FLEXCAN_CTRL1_ERRMSK | CAN_FLEXCAN_CTRL1_TWRNMSK |
		 CAN_FLEXCAN_CTRL1_RWRNMSK;

	if (config->clk_source != 0U) {
		ctrl1 |= CAN_FLEXCAN_CTRL1_CLKSRC;
	}

	can_flexcan_write_reg(dev, CAN_FLEXCAN_CTRL1, ctrl1);

	/* CAN classic/arbitration phase timing */
	err = can_calc_timing(dev, &timing, config->common.bitrate, config->common.sample_point);
	if (err < 0) {
		LOG_ERR("failed to calculate timing for bitrate %u, sample-point %u (err %d)",
			config->common.bitrate, config->common.sample_point, err);
		return -ENODEV;
	}

	err = can_set_timing(dev, &timing);
	if (err != 0) {
		LOG_ERR("failed to set timing (err %d)", err);
		return -ENODEV;
	}

	if (UTIL_AND(IS_ENABLED(CONFIG_CAN_FLEXCAN_FD), config->flexcan_fd)) {
		static const uint32_t mbdrs[] = {
			CAN_FLEXCAN_FDCTRL_MBDSR0, CAN_FLEXCAN_FDCTRL_MBDSR1,
			CAN_FLEXCAN_FDCTRL_MBDSR2, CAN_FLEXCAN_FDCTRL_MBDSR3};
		/* CAN FD Control Register (FDCTRL) */
		uint32_t fdctrl = CAN_FLEXCAN_FDCTRL_FDRATE;
		int i = 0;

		do {
			/* Configure Message Buffer RAM block 0 up to 3 as 7 x 64-byte MBs */
			__ASSERT_NO_MSG(i < ARRAY_SIZE(mbdrs));
			fdctrl |= FIELD_PREP(mbdrs[i], CAN_FLEXCAN_FDCTRL_MBDS_64BYTES);
		} while (++i < config->number_of_mb / 32U);

		can_flexcan_write_reg(dev, CAN_FLEXCAN_FDCTRL, fdctrl);

		/* Data phase timing */
		err = can_calc_timing_data(dev, &timing, config->common.bitrate_data,
					   config->common.sample_point_data);
		if (err < 0) {
			LOG_ERR("failed to calculate data phase timing for bitrate-data %u, "
				"sample-point-data %u (err %d)",
				config->common.bitrate_data, config->common.sample_point_data, err);
			return -ENODEV;
		}

		err = can_set_timing_data(dev, &timing);
		if (err != 0) {
			LOG_ERR("failed to set data phase timing (err %d)", err);
			return -ENODEV;
		}
	}

	for (uint8_t i = 0; i < config->number_of_mb; i++) {
		/* Initialize MB control and status (CS) word */
		can_flexcan_write_mb(dev, i, &mbhdr, NULL, 0U);
	}

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}

	config->irq_config_func(dev);

	(void)can_flexcan_get_state(dev, &data->state, NULL);

	return 0;
}

static DEVICE_API(can, can_flexcan_driver_api) __maybe_unused = {
	.get_capabilities = can_flexcan_get_capabilities,
	.start = can_flexcan_start,
	.stop = can_flexcan_stop,
	.set_mode = can_flexcan_set_mode,
	.set_timing = can_flexcan_set_timing,
	.send = can_flexcan_send,
	.add_rx_filter = can_flexcan_add_rx_filter,
	.remove_rx_filter = can_flexcan_remove_rx_filter,
#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
	.recover = can_flexcan_recover,
#endif /* CONFIG_CAN_MANUAL_RECOVERY_MODE */
	.get_state = can_flexcan_get_state,
	.set_state_change_callback = can_flexcan_set_state_change_callback,
	.get_core_clock = can_flexcan_get_core_clock,
	.get_max_filters = can_flexcan_get_max_filters,
	/*
	 * FlexCAN timing limits are specified in the Control 1 (CTRL1) field description table in
	 * the SoC reference manual.
	 *
	 * Note that the values here are the "physical" timing limits, whereas the register field
	 * limits are physical values minus 1 (which is handled via CTRL1 register assignments
	 * elsewhere in this driver).
	 */
	.timing_min = {
		.sjw = 1,
		.prop_seg = 1,
		.phase_seg1 = 1,
		.phase_seg2 = 2,
		.prescaler = 1,
	},
	.timing_max = {
		.sjw = 4,
		.prop_seg = 8,
		.phase_seg1 = 8,
		.phase_seg2 = 8,
		.prescaler = 256,
	},
};

#ifdef CONFIG_CAN_FLEXCAN_FD
static DEVICE_API(can, can_flexcan_fd_driver_api) = {
	.get_capabilities = can_flexcan_get_capabilities,
	.start = can_flexcan_start,
	.stop = can_flexcan_stop,
	.set_mode = can_flexcan_set_mode,
	.set_timing = can_flexcan_set_timing,
	.send = can_flexcan_send,
	.add_rx_filter = can_flexcan_add_rx_filter,
	.remove_rx_filter = can_flexcan_remove_rx_filter,
#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
	.recover = can_flexcan_recover,
#endif /* CONFIG_CAN_MANUAL_RECOVERY_MODE */
	.get_state = can_flexcan_get_state,
	.set_state_change_callback = can_flexcan_set_state_change_callback,
	.get_core_clock = can_flexcan_get_core_clock,
	.get_max_filters = can_flexcan_get_max_filters,
	.set_timing_data = can_flexcan_set_timing_data,
	/*
	 * FlexCAN timing limits are specified in the CAN Bit Timing Register (CBT) and CAN FD Bit
	 * Timing Register (FDCBT) field description tables in the SoC reference manual.
	 *
	 * Note that the values here are the "physical" timing limits, whereas the register field
	 * limits are physical values minus 1 (which is handled via CBT/FDCBT register assignments
	 * elsewhere in this driver).
	 */
	.timing_min = {
		.sjw = 1,
		.prop_seg = 1,
		.phase_seg1 = 1,
		.phase_seg2 = 2,
		.prescaler = 1,
	},
	.timing_max = {
		.sjw = 32,
		.prop_seg = 64,
		.phase_seg1 = 32,
		.phase_seg2 = 32,
		.prescaler = 1024,
	},
	.timing_data_min = {
		.sjw = 1,
		.prop_seg = 1,
		.phase_seg1 = 1,
		.phase_seg2 = 2,
		.prescaler = 1,
	},
	.timing_data_max = {
		.sjw = 8,
		.prop_seg = 32,
		.phase_seg1 = 8,
		.phase_seg2 = 8,
		.prescaler = 1024,
	},
};
#endif /* CONFIG_CAN_FLEXCAN_FD */

#ifdef CONFIG_CAN_FLEXCAN_FD
/* Each 512-byte RAM region can contain up to 32 x 8-byte MBs or 7 x 64-byte MBs (CAN FD). */
#define CAN_FLEXCAN_USABLE_MBS(inst)                                                               \
	COND_CODE_1(DT_NODE_HAS_COMPAT(DT_DRV_INST(inst), CAN_FLEXCAN_FD_DRV_COMPAT),              \
		    ((DT_INST_PROP(inst, number_of_mb) * 7U) / 32U),                               \
		    (DT_INST_PROP(inst, number_of_mb)))
#define CAN_FLEXCAN_MAX_BITRATE(inst)                                                              \
	COND_CODE_1(DT_NODE_HAS_COMPAT(DT_DRV_INST(inst), CAN_FLEXCAN_FD_DRV_COMPAT), (8000000),   \
		    (1000000))
#define CAN_FLEXCAN_DRIVER_API(inst)                                                               \
	COND_CODE_1(DT_NODE_HAS_COMPAT(DT_DRV_INST(inst), CAN_FLEXCAN_FD_DRV_COMPAT),              \
		    (can_flexcan_fd_driver_api), (can_flexcan_driver_api))
#else /* CONFIG_CAN_FLEXCAN_FD */
#define CAN_FLEXCAN_USABLE_MBS(inst) DT_INST_PROP(inst, number_of_mb)
#define CAN_FLEXCAN_MAX_BITRATE(id)  1000000
#define CAN_FLEXCAN_DRIVER_API(inst) can_flexcan_driver_api
#endif /* !CONFIG_CAN_FLEXCAN_FD */

/*
 * RX message buffers (filters) take up the first N message buffers. The rest are available for
 * TX use.
 */
#define CAN_FLEXCAN_MAX_FILTERS(inst) \
	DT_INST_PROP_OR(inst, max_filters, CONFIG_CAN_FLEXCAN_MAX_FILTERS)
#define CAN_FLEXCAN_NUM_RX_CBS(inst) (CAN_FLEXCAN_MAX_FILTERS(inst))
#define CAN_FLEXCAN_NUM_TX_CBS(inst) (CAN_FLEXCAN_USABLE_MBS(inst) - CAN_FLEXCAN_NUM_RX_CBS(inst))

#define CAN_FLEXCAN_CHECK_MAX_FILTERS(inst)                                                        \
	BUILD_ASSERT(CAN_FLEXCAN_USABLE_MBS(inst) > CAN_FLEXCAN_NUM_RX_CBS(inst),                  \
		     "FlexCAN instance " STRINGIFY(inst) " has too few usable MBs ("               \
		     STRINGIFY(CAN_FLEXCAN_USABLE_MBS(inst))                                       \
		     ") for the requested number of RX filters "                                   \
		     "(" STRINGIFY(CAN_FLEXCAN_NUM_RX_CBS(inst)) ")")

#define CAN_FLEXCAN_IRQ_CONFIG(node_id, prop, idx)                                                 \
	do {                                                                                       \
		IRQ_CONNECT(DT_IRQ_BY_IDX(node_id, idx, irq),                                      \
			    DT_IRQ_BY_IDX(node_id, idx, priority), can_flexcan_isr,                \
			    DEVICE_DT_GET(node_id), 0);                                            \
		irq_enable(DT_IRQ_BY_IDX(node_id, idx, irq));                                      \
	} while (false);

#define CAN_FLEXCAN_CLK_SOURCE(inst)      DT_INST_PROP(inst, clk_source)
#define CAN_FLEXCAN_CLK_SOURCE_NAME(inst) CONCAT(clksrc, CAN_FLEXCAN_CLK_SOURCE(inst))

#define CAN_FLEXCAN_CLOCKS_FROM_CLK_SOURCE(inst)                                                   \
	.clock_dev = DEVICE_DT_GET(                                                                \
		DT_INST_CLOCKS_CTLR_BY_NAME(inst, CAN_FLEXCAN_CLK_SOURCE_NAME(inst))),             \
	.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(                       \
		inst, CAN_FLEXCAN_CLK_SOURCE_NAME(inst), name),                                    \
	.clk_source = CAN_FLEXCAN_CLK_SOURCE(inst)

#define CAN_FLEXCAN_CLOCKS_NO_CLK_SOURCE(inst)                                                     \
	.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                                     \
	.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, name),                   \
	.clk_source = 0U

#define CAN_FLEXCAN_CLOCKS(inst)                                                                   \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, clk_source),                                       \
		(CAN_FLEXCAN_CLOCKS_FROM_CLK_SOURCE(inst)),                                        \
		(CAN_FLEXCAN_CLOCKS_NO_CLK_SOURCE(inst)))

#define CAN_FLEXCAN_CHECK_CLK_SOURCE(inst)                                                         \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, clk_source),                                        \
		(BUILD_ASSERT(DT_INST_CLOCKS_HAS_NAME(inst, CAN_FLEXCAN_CLK_SOURCE_NAME(inst)),    \
			"FlexCAN instance " STRINGIFY(inst) " clk-source without named clock")))

#define CAN_FLEXCAN_DEVICE_INIT(inst)                                                              \
	CAN_FLEXCAN_CHECK_MAX_FILTERS(inst);                                                       \
	CAN_FLEXCAN_CHECK_CLK_SOURCE(inst);                                                        \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
	static void can_flexcan_irq_config_##inst(const struct device *dev)                        \
	{                                                                                          \
		DT_INST_FOREACH_PROP_ELEM(inst, interrupt_names, CAN_FLEXCAN_IRQ_CONFIG);          \
	}                                                                                          \
                                                                                                   \
	static struct can_flexcan_rx_callback                                                      \
		can_flexcan_rx_cbs_##inst[CAN_FLEXCAN_NUM_RX_CBS(inst)];                           \
	static struct can_flexcan_tx_callback                                                      \
		can_flexcan_tx_cbs_##inst[CAN_FLEXCAN_NUM_TX_CBS(inst)];                           \
                                                                                                   \
	static const struct can_flexcan_config can_flexcan_config_##inst = {                       \
		.common = CAN_DT_DRIVER_CONFIG_INST_GET(inst, 0, CAN_FLEXCAN_MAX_BITRATE(inst)),   \
		DEVICE_MMIO_NAMED_ROM_INIT(reg_base, DT_DRV_INST(inst)),                           \
		CAN_FLEXCAN_CLOCKS(inst),                                                          \
		.number_of_mb = DT_INST_PROP(inst, number_of_mb),                                  \
		.irq_config_func = can_flexcan_irq_config_##inst,                                  \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
		.rx_cbs = can_flexcan_rx_cbs_##inst,                                               \
		.tx_cbs = can_flexcan_tx_cbs_##inst,                                               \
		.num_rx_cbs = CAN_FLEXCAN_NUM_RX_CBS(inst),                                        \
		.num_tx_cbs = CAN_FLEXCAN_NUM_TX_CBS(inst),                                        \
		.maxmb = CAN_FLEXCAN_NUM_RX_CBS(inst) + CAN_FLEXCAN_NUM_TX_CBS(inst) - 1U,         \
		.max_filters = CAN_FLEXCAN_MAX_FILTERS(id),                                        \
		IF_ENABLED(CONFIG_CAN_FLEXCAN_FD,                                                  \
		(.flexcan_fd = DT_NODE_HAS_COMPAT(DT_DRV_INST(inst), CAN_FLEXCAN_FD_DRV_COMPAT),)) \
	};                                                                                         \
                                                                                                   \
	static struct can_flexcan_data can_flexcan_data_##inst;                                    \
                                                                                                   \
	CAN_DEVICE_DT_INST_DEFINE(inst, can_flexcan_init, NULL, &can_flexcan_data_##inst,          \
				  &can_flexcan_config_##inst, POST_KERNEL,                         \
				  CONFIG_CAN_INIT_PRIORITY, &CAN_FLEXCAN_DRIVER_API(inst));

DT_INST_FOREACH_STATUS_OKAY(CAN_FLEXCAN_DEVICE_INIT)
