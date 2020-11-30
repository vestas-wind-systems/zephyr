/*
 * Copyright (c) 2020 IoT.bzh
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rcar_gen3_can

#include <kernel.h>
#include <errno.h>
#include <drivers/can.h>
#include <drivers/clock_control.h>
#include <drivers/clock_control/rcar_clock_control.h>

#include <logging/log.h>
LOG_MODULE_DECLARE(can_driver, CONFIG_CAN_LOG_LEVEL);

#include "can_utils.h"

static K_KERNEL_STACK_DEFINE(can_rcar_thread_stack,
			     CONFIG_CAN_RCAR_INT_THREAD_STACK_SIZE);

typedef void (*init_func_t)(const struct device *dev);

struct can_rcar_cfg {
	uint32_t reg_addr;
	int reg_size;
	init_func_t init_func;
	char *clock_controller;
	struct rcar_cpg_clk mod_clk;
	struct rcar_cpg_clk bus_clk;
	uint32_t bus_speed;
	uint8_t sjw;
	uint8_t prop_seg;
	uint8_t phase_seg1;
	uint8_t phase_seg2;
	size_t int_thread_stack_size;
	int int_thread_priority;
};

struct can_rcar_data {
	struct k_mutex inst_mutex;
	can_tx_callback_t tx_callback;
	void *tx_callback_arg;
	struct k_sem tx_int_sem;
	uint32_t error_flags;

	const struct device *dev;
	struct k_thread int_thread;
	k_thread_stack_t *int_thread_stack;
	struct k_sem rx_int_sem;
	struct k_mutex rx_mutex;
	can_rx_callback_t rx_callback[CONFIG_CAN_RCAR_MAX_FILTER];
	void *rx_callback_arg[CONFIG_CAN_RCAR_MAX_FILTER];
	struct zcan_filter filter[CONFIG_CAN_RCAR_MAX_FILTER];

	enum can_state state;
};

#define DEV_CAN_CFG(dev)						\
	((const struct can_rcar_cfg *)(dev)->config)

#define DEV_CAN_DATA(dev) ((struct can_rcar_data *const)(dev)->data)

#define	RCAR_CAN_CTLR		0x0840	/* Control Register */
/* Control Register bits */
#define RCAR_CAN_CTLR_BOM	(3 << 11) /* Bus-Off Recovery Mode Bits */
#define RCAR_CAN_CTLR_BOM_ENT	(1 << 11) /* Entry to halt mode */
					/* at bus-off entry */
#define RCAR_CAN_CTLR_SLPM	(1 << 10)

#define RCAR_CAN_CTLR_CANM_HALT	(1 << 9)
#define RCAR_CAN_CTLR_CANM_RESET (1 << 8)
#define RCAR_CAN_CTLR_CANM_MASK (3 << 8)

#define RCAR_CAN_CTLR_MLM	(1 << 3) /* Message Lost Mode Select */
#define RCAR_CAN_CTLR_IDFM	(3 << 1) /* ID Format Mode Select Bits */
#define RCAR_CAN_CTLR_IDFM_MIXED (1 << 2) /* Mixed ID mode */
#define RCAR_CAN_CTLR_MBM	(1 << 0) /* Mailbox Mode select */

#define	RCAR_CAN_MKR0		0x0430	/* Mask Register 0 */
#define	RCAR_CAN_MKR1		0x0434	/* Mask Register 1 */
#define	RCAR_CAN_MKR2		0x0400	/* Mask Register 2 */
#define	RCAR_CAN_MKR3		0x0404	/* Mask Register 3 */
#define	RCAR_CAN_MKR4		0x0408	/* Mask Register 4 */
#define	RCAR_CAN_MKR5		0x040C	/* Mask Register 5 */
#define	RCAR_CAN_MKR6		0x0410	/* Mask Register 6 */
#define	RCAR_CAN_MKR7		0x0414	/* Mask Register 7 */
#define	RCAR_CAN_MKR8		0x0418	/* Mask Register 8 */
#define	RCAR_CAN_MKR9		0x041C	/* Mask Register 9 */
#define	RCAR_CAN_FIDCR0		0x0420	/* FIFO Received ID Compare Register 0 */
#define	RCAR_CAN_FIDCR1		0x0424	/* FIFO Received ID Compare Register 1 */
/* FIFO Received ID Compare Registers 0 and 1 bits */
#define RCAR_CAN_FIDCR_IDE	(1 << 31) /* ID Extension Bit */
#define RCAR_CAN_FIDCR_RTR	(1 << 30) /* Remote Transmission Request Bit */

#define	RCAR_CAN_MKIVLR0	0x0438	/* Mask Invalid Register 0 */
#define	RCAR_CAN_MKIVLR1	0x0428	/* Mask Invalid Register 1 */

#define	RCAR_CAN_MIER0		0x043C	/* Mailbox Interrupt Enable Register 0 */
#define	RCAR_CAN_MIER1		0x042C	/* Mailbox Interrupt Enable Register 1 */
#define RCAR_CAN_MIER1_RXFIE	(1 << 28) /* Receive  FIFO Interrupt Enable */
#define RCAR_CAN_MIER1_TXFIE	(1 << 24) /* Transmit FIFO Interrupt Enable */



#define	RCAR_CAN_STR		0x0842	/* Status Register */
#define RCAR_CAN_STR_RSTST	(1 << 8) /* Reset Status Bit */
#define RCAR_CAN_STR_HLTST	(1 << 9) /* Halt Status Bit */
#define MAX_STR_READS		0x100

#define	RCAR_CAN_BCR		0x0844	/* Bit Configuration Register */

#define	RCAR_CAN_CLKR		0x0847	/* Clock Select Register */
#define	RCAR_CAN_CLKR_EXT_CLOCK	0x3     /* External input clock */
#define	RCAR_CAN_CLKR_CLKP2	0x1
#define	RCAR_CAN_CLKR_CLKP1	0x0

#define	RCAR_CAN_EIER		0x084C	/* Error Interrupt Enable Register */

#define	RCAR_CAN_IER		0x0860	/* Interrupt Enable Register */
#define RCAR_CAN_IER_ERSIE	(1 << 5) /* Error (ERS) Interrupt Enable Bit */
#define RCAR_CAN_IER_RXFIE	(1 << 4) /* Reception FIFO Interrupt Enable Bit */
#define RCAR_CAN_IER_TXFIE	(1 << 3) /* Transmission FIFO Interrupt Enable Bit */

#define	RCAR_CAN_ISR		0x0861	/* Interrupt Status Register */
#define RCAR_CAN_ISR_ERSF	(1 << 5) /* Error (ERS) Interrupt Status Bit */
#define RCAR_CAN_ISR_RXFF	(1 << 4) /* Reception FIFO Interrupt */
					/* Status Bit */
#define RCAR_CAN_ISR_TXFF	(1 << 3) /* Transmission FIFO Interrupt */
					/* Status Bit */

#define RCAR_CAN_RFCR           0x0848	/* Receive FIFO Control Register */
#define RCAR_CAN_RFCR_RFE	(1 << 0) /* Receive FIFO Enable */
#define RCAR_CAN_RFCR_RFEST	(1 << 7) /* Receive FIFO Empty Flag */

#define	RCAR_CAN_RFPCR		0x0849	/* Receive FIFO Pointer Control Register */

#define	RCAR_CAN_TFCR		0x084A	/* Transmit FIFO Control Register */
#define RCAR_CAN_TFCR_TFE	(1 << 0)/* Transmit FIFO Enable */
#define RCAR_CAN_TFCR_TFUST	(7 << 1)/* Transmit FIFO Unsent Message */
					/* Number Status Bits */
#define RCAR_CAN_TFCR_TFUST_SHIFT 1	/* Offset of Transmit FIFO Unsent */
					/* Message Number Status Bits */

#define	RCAR_CAN_TFPCR		0x084B	/* Transmit FIFO Pointer Control Register */

/* Error Code Store Register*/
#define	RCAR_CAN_ECSR		0x0850	/* Error Code Store Register */
#define RCAR_CAN_ECSR_EDPM	(1 << 7) /* Error Display Mode Select Bit */
#define RCAR_CAN_ECSR_ADEF	(1 << 6) /* ACK Delimiter Error Flag */
#define RCAR_CAN_ECSR_BE0F	(1 << 5) /* Bit Error (dominant) Flag */
#define RCAR_CAN_ECSR_BE1F	(1 << 4) /* Bit Error (recessive) Flag */
#define RCAR_CAN_ECSR_CEF	(1 << 3) /* CRC Error Flag */
#define RCAR_CAN_ECSR_AEF	(1 << 2) /* ACK Error Flag */
#define RCAR_CAN_ECSR_FEF	(1 << 1) /* Form Error Flag */
#define RCAR_CAN_ECSR_SEF	(1 << 0) /* Stuff Error Flag */

#define	RCAR_CAN_TCR		0x0858	/* Test Control Register */
#define RCAR_CAN_TCR_TSTE        (1 << 0) /* Test Mode Enable Bit*/
#define RCAR_CAN_TCR_LISTEN_ONLY (1 << 1)
#define RCAR_CAN_TCR_INT_LOOP    (3 << 1) /* Internal loopback*/

/* Error Interrupt Factor Judge Register bits */
#define RCAR_CAN_EIFR           0x084D
#define RCAR_CAN_EIFR_BLIF	(1 << 7) /* Bus Lock Detect Flag */
#define RCAR_CAN_EIFR_OLIF	(1 << 6) /* Overload Frame Transmission */
					 /* Detect Flag */
#define RCAR_CAN_EIFR_ORIF	(1 << 5) /* Receive Overrun Detect Flag */
#define RCAR_CAN_EIFR_BORIF	(1 << 4) /* Bus-Off Recovery Detect Flag */
#define RCAR_CAN_EIFR_BOEIF	(1 << 3) /* Bus-Off Entry Detect Flag */
#define RCAR_CAN_EIFR_EPIF	(1 << 2) /* Error Passive Detect Flag */
#define RCAR_CAN_EIFR_EWIF	(1 << 1) /* Error Warning Detect Flag */
#define RCAR_CAN_EIFR_BEIF	(1 << 0) /* Bus Error Detect Flag */

#define RCAR_CAN_RECR           0x084D   /* Receive Error Count Register */
#define RCAR_CAN_TECR           0x084F   /* Transmit Error Count Register*/

/* Mailbox configuration:
 * mailbox 60 - 63 - Rx FIFO mailboxes
 * mailbox 56 - 59 - Tx FIFO mailboxes
 * non-FIFO mailboxes are not used
 */
#define RCAR_CAN_N_MBX		64 /* Number of mailboxes in non-FIFO mode */
#define RCAR_CAN_RX_FIFO_MBX	60 /* Mailbox - window to Rx FIFO */
#define RCAR_CAN_TX_FIFO_MBX	56 /* Mailbox - window to Tx FIFO */
#define	RCAR_CAN_MB_56		0x0380	/* Mailbox Register 56 */
#define	RCAR_CAN_MB_60		0x03C0	/* Mailbox Register 60 */
/* DLC must be accessed as a 16 bit register */
#define	RCAR_CAN_MB_DLC_OFFSET	0x4	/* Mailbox offset to data length code */
#define	RCAR_CAN_MB_DATA_OFFSET	0x6	/* Mailbox offset to data section up to 8 bytes*/
#define	RCAR_CAN_MB_TSH_OFFSET	0x14	/* Mailbox offset to timestamp upper byte */
#define	RCAR_CAN_MB_TSL_OFFSET	0x14	/* Mailbox offset to timestamp lower byte */
#define RCAR_CAN_FIFO_DEPTH	4
#define RCAR_CAN_MB_SID_SHIFT   18
#define RCAR_CAN_MB_RTR         (1 << 30)
#define RCAR_CAN_MB_IDE         (1 << 31)
#define RCAR_CAN_MB_SID_MASK    0x1FFC0000
#define RCAR_CAN_MB_EID_MASK    0x1FFFFFFF


static inline uint16_t can_rcar_read16(const struct can_rcar_cfg *config, uint32_t offs)
{
	return sys_read16(config->reg_addr + offs);
}

static inline void can_rcar_write16(const struct can_rcar_cfg *config, uint32_t offs,
				   uint16_t value)
{
	sys_write16(value, config->reg_addr + offs);
}

static void can_rcar_tx_done(const struct device *dev)
{
	struct can_rcar_data *data = DEV_CAN_DATA(dev);

	/* we may keep track of in-flight messages there.
	 * ATM: consider that we are sending messages one by one.
	 */
	if (data->tx_callback) {
		data->tx_callback(CAN_TX_OK, data->tx_callback_arg);
	} else {
		data->error_flags = CAN_TX_OK;
		k_sem_give(&data->tx_int_sem);
	}
}

static void can_rcar_error(const struct device *dev)
{
	const struct can_rcar_cfg *config = DEV_CAN_CFG(dev);
	struct can_rcar_data *data = DEV_CAN_DATA(dev);

	/* We may check there if we failed to transmit a message.
	 * In this case the caller should be notified with an error_status.
	 */
	uint8_t eifr, ecsr, txerr = 0, rxerr = 0;

	eifr = sys_read8(config->reg_addr + RCAR_CAN_EIFR);
	if (eifr & (RCAR_CAN_EIFR_EWIF | RCAR_CAN_EIFR_EPIF)) {
		txerr = sys_read8(config->reg_addr + RCAR_CAN_TECR);
		rxerr = sys_read8(config->reg_addr + RCAR_CAN_RECR);
	}

	if (eifr & RCAR_CAN_EIFR_BEIF) {

		LOG_ERR("Bus error interrupt:\n");
		ecsr = sys_read8(config->reg_addr + RCAR_CAN_ECSR);
		if (ecsr & RCAR_CAN_ECSR_ADEF) {
			LOG_ERR("ACK Delimiter Error\n");
			sys_write8(~RCAR_CAN_ECSR_ADEF, config->reg_addr + RCAR_CAN_ECSR);
		}
		if (ecsr & RCAR_CAN_ECSR_BE0F) {
			LOG_ERR("Bit Error (dominant)\n");
			sys_write8(~RCAR_CAN_ECSR_BE0F, config->reg_addr + RCAR_CAN_ECSR);
		}
		if (ecsr & RCAR_CAN_ECSR_BE1F) {
			LOG_ERR("Bit Error (recessive)\n");
			sys_write8(~RCAR_CAN_ECSR_BE1F, config->reg_addr + RCAR_CAN_ECSR);
		}
		if (ecsr & RCAR_CAN_ECSR_CEF) {
			LOG_ERR("CRC Error\n");
			sys_write8(~RCAR_CAN_ECSR_CEF, config->reg_addr + RCAR_CAN_ECSR);
		}
		if (ecsr & RCAR_CAN_ECSR_AEF) {
			LOG_ERR("ACK Error\n");
			sys_write8(~RCAR_CAN_ECSR_AEF, config->reg_addr + RCAR_CAN_ECSR);
		}
		if (ecsr & RCAR_CAN_ECSR_FEF) {
			LOG_ERR("Form Error\n");
			sys_write8(~RCAR_CAN_ECSR_FEF, config->reg_addr + RCAR_CAN_ECSR);
		}
		if (ecsr & RCAR_CAN_ECSR_SEF) {
			LOG_ERR("Stuff Error\n");
			sys_write8(~RCAR_CAN_ECSR_SEF, config->reg_addr + RCAR_CAN_ECSR);
		}

		sys_write8(~RCAR_CAN_EIFR_BEIF, config->reg_addr + RCAR_CAN_EIFR);
	}
	if (eifr & RCAR_CAN_EIFR_EWIF) {
		LOG_ERR("Error warning interrupt\n");
		/* Clear interrupt condition */
		sys_write8(~RCAR_CAN_EIFR_EWIF, config->reg_addr + RCAR_CAN_EIFR);
	}
	if (eifr & RCAR_CAN_EIFR_EPIF) {
		LOG_ERR("Error passive interrupt\n");
		data->state = CAN_ERROR_PASSIVE;
		/* Clear interrupt condition */
		sys_write8(~RCAR_CAN_EIFR_EPIF, config->reg_addr + RCAR_CAN_EIFR);
	}
	if (eifr & RCAR_CAN_EIFR_BOEIF) {
		LOG_ERR("Bus-off entry interrupt\n");
		/* FIXME: if there is an in flight message there return an error */
		sys_write8(RCAR_CAN_IER_ERSIE, config->reg_addr + RCAR_CAN_IER);
		data->state = CAN_BUS_OFF;
		/* Clear interrupt condition */
		sys_write8(~RCAR_CAN_EIFR_BOEIF, config->reg_addr + RCAR_CAN_EIFR);
	}
	if (eifr & RCAR_CAN_EIFR_ORIF) {
		LOG_ERR("Receive overrun error interrupt\n");
		sys_write8(~RCAR_CAN_EIFR_ORIF, config->reg_addr + RCAR_CAN_EIFR);
	}
	if (eifr & RCAR_CAN_EIFR_OLIF) {
		LOG_ERR("Overload Frame Transmission error interrupt\n");
		sys_write8(~RCAR_CAN_EIFR_OLIF, config->reg_addr + RCAR_CAN_EIFR);
	}
	if (eifr & RCAR_CAN_EIFR_BLIF) {
		LOG_ERR("Bus lock detected interrupt\n");
		sys_write8(~RCAR_CAN_EIFR_BLIF, config->reg_addr + RCAR_CAN_EIFR);
	}
}

static void can_rcar_rx_filter(struct can_rcar_data *data,
			      const struct zcan_frame *msg)
{
	struct zcan_frame tmp_msg;
	uint8_t i;

	k_mutex_lock(&data->rx_mutex, K_FOREVER);
	for (i = 0; i < CONFIG_CAN_RCAR_MAX_FILTER; i++) {
		if (data->rx_callback[i] == NULL)
			continue;

		if (!can_utils_filter_match(msg,
					  &data->filter[i])) {
			continue; /* filter did not match */
		}
		/* Make a temporary copy in case the user modifies the message */
		tmp_msg = *msg;
		data->rx_callback[i](&tmp_msg, data->rx_callback_arg[i]);
	}
	k_mutex_unlock(&data->rx_mutex);
};

static void can_rcar_rx_handler(const struct device *dev)
{
	struct can_rcar_data *data = DEV_CAN_DATA(dev);
	const struct can_rcar_cfg *config = DEV_CAN_CFG(dev);
	struct zcan_frame msg;
	uint32_t val;
	int i;

	val = sys_read32(config->reg_addr + RCAR_CAN_MB_60);
	if (val & RCAR_CAN_MB_IDE) {
		msg.id_type = CAN_EXTENDED_IDENTIFIER;
		msg.ext_id = val & RCAR_CAN_MB_EID_MASK;
	} else {
		msg.id_type = CAN_STANDARD_IDENTIFIER;
		msg.std_id = (val & RCAR_CAN_MB_SID_MASK) >> RCAR_CAN_MB_SID_SHIFT;
	}

	if (val & RCAR_CAN_MB_RTR)
	{
		msg.rtr = CAN_REMOTEREQUEST;
	} else {
		msg.rtr = CAN_DATAFRAME;
	}

	msg.dlc = sys_read16(config->reg_addr
			    + RCAR_CAN_MB_60 + RCAR_CAN_MB_DLC_OFFSET) & 0xF;

	/* Be paranoid doc states that any value greater than 8
	 * should be considered as 8 bytes.
	 */
	if (msg.dlc > CAN_MAX_DLC)
		msg.dlc = CAN_MAX_DLC;

	for (i = 0; i < msg.dlc; i++)
		msg.data[i] = sys_read8(config->reg_addr
			   + RCAR_CAN_MB_60 + RCAR_CAN_MB_DATA_OFFSET + i);
#if defined(CONFIG_CAN_RX_TIMESTAMP)
	/* read upper byte */
	msg.timestamp = sys_read8(config->reg_addr +
				  RCAR_CAN_MB_60 + RCAR_CAN_MB_TSH_OFFSET);
	msg.timestamp = msg.timestamp << 16;
	/* and then read lower byte */
	msg.timestamp |= sys_read8(config->reg_addr +
				   RCAR_CAN_MB_60 + RCAR_CAN_MB_TSL_OFFSET);
#endif
	/* Increment CPU side pointer */
	sys_write8(0xff, config->reg_addr + RCAR_CAN_RFPCR);

	can_rcar_rx_filter(data, &msg);
}

static void can_rcar_rx_thread(const struct device *dev)
{
	const struct can_rcar_cfg *config = DEV_CAN_CFG(dev);
	struct can_rcar_data *data = DEV_CAN_DATA(dev);

	while (1) {
		k_sem_take(&data->rx_int_sem, K_FOREVER);
		/* while there is pending messages */
		while (!(sys_read8(config->reg_addr + RCAR_CAN_RFCR)
			 & RCAR_CAN_RFCR_RFEST))
			can_rcar_rx_handler(dev);
	}
};

static void can_rcar_isr(const struct device *dev)
{
	const struct can_rcar_cfg *config = DEV_CAN_CFG(dev);
	struct can_rcar_data *data = DEV_CAN_DATA(dev);

	uint8_t isr;

	isr = sys_read8(config->reg_addr + RCAR_CAN_ISR);
	if (isr & RCAR_CAN_ISR_ERSF) {
		/* Clear the Error interrupt */
		isr &= ~RCAR_CAN_ISR_ERSF;
		sys_write8(isr, config->reg_addr + RCAR_CAN_ISR);
		can_rcar_error(dev);
	}
	if (isr & RCAR_CAN_ISR_TXFF) {
		/* Clear the Tx interrupt */
		isr &= ~RCAR_CAN_ISR_TXFF;
		sys_write8(isr, config->reg_addr + RCAR_CAN_ISR);
		can_rcar_tx_done(dev);
	}
	if (isr & RCAR_CAN_ISR_RXFF) {
		/* Clear the Rx interrupt */
		isr &= ~RCAR_CAN_ISR_RXFF;
		sys_write8(isr, config->reg_addr + RCAR_CAN_ISR);
		/* wake up the rx thread */
		k_sem_give(&data->rx_int_sem);
	}
}

static int can_rcar_leave_sleep_mode(const struct can_rcar_cfg *config)
{
	uint16_t ctlr, str;
	int i;

	ctlr = can_rcar_read16(config, RCAR_CAN_CTLR);
	ctlr &= ~RCAR_CAN_CTLR_SLPM;
	can_rcar_write16(config, RCAR_CAN_CTLR, ctlr);
	for (i = 0; i < MAX_STR_READS; i++) {
		str = can_rcar_read16(config, RCAR_CAN_STR);
		if (!(str & RCAR_CAN_STR_SLPST))
			return 0;
	}
	return -ETIME;
}

static int can_rcar_enter_reset_mode(const struct can_rcar_cfg *config, bool force)
{
	uint16_t ctlr;
	int i;

	ctlr = can_rcar_read16(config, RCAR_CAN_CTLR);
	ctlr &= ~RCAR_CAN_CTLR_CANM_MASK;
	ctlr |= RCAR_CAN_CTLR_CANM_RESET;
	if (force)
		ctlr |= RCAR_CAN_CTLR_CANM_HALT;
	can_rcar_write16(config, RCAR_CAN_CTLR, ctlr);
	for (i = 0; i < MAX_STR_READS; i++) {
		if (can_rcar_read16(config, RCAR_CAN_STR) & RCAR_CAN_STR_RSTST)
			return 0;
	}
	return -ETIME;
}

static int can_rcar_enter_halt_mode(const struct can_rcar_cfg *config)
{
	uint16_t ctlr;
	int i;

	ctlr = can_rcar_read16(config, RCAR_CAN_CTLR);
	ctlr &= ~RCAR_CAN_CTLR_CANM_MASK;
	ctlr |= RCAR_CAN_CTLR_CANM_HALT;
	can_rcar_write16(config, RCAR_CAN_CTLR, ctlr);
	for (i = 0; i < MAX_STR_READS; i++) {
		if (can_rcar_read16(config, RCAR_CAN_STR) & RCAR_CAN_STR_HLTST)
			return 0;
	}

	return -ETIME;
}

static int can_rcar_enter_operation_mode(const struct can_rcar_cfg *config)
{
	uint16_t ctlr, str;
	int i;

	ctlr = can_rcar_read16(config, RCAR_CAN_CTLR);
	ctlr &= ~RCAR_CAN_CTLR_CANM_MASK;
	can_rcar_write16(config, RCAR_CAN_CTLR, ctlr);

	for (i = 0; i < MAX_STR_READS; i++) {
		str = can_rcar_read16(config, RCAR_CAN_STR);
		if (!(str & RCAR_CAN_CTLR_CANM_MASK))
			break;
	}

	if (i == MAX_STR_READS)
		return -ETIME;

	/* Enable Rx and Tx FIFO */
	sys_write8(RCAR_CAN_RFCR_RFE, config->reg_addr + RCAR_CAN_RFCR);
	sys_write8(RCAR_CAN_TFCR_TFE, config->reg_addr + RCAR_CAN_TFCR);

	return 0;
}

/* Bit Configuration Register settings */
#define RCAR_CAN_BCR_TSEG1(x)	(((x) & 0x0f) << 20)
#define RCAR_CAN_BCR_BPR(x)	(((x) & 0x3ff) << 8)
#define RCAR_CAN_BCR_SJW(x)	(((x) & 0x3) << 4)
#define RCAR_CAN_BCR_TSEG2(x)	((x) & 0x07)

static void can_rcar_set_bittiming(const struct can_rcar_cfg *config, uint32_t bitrate)
{
	uint16_t brp;
	uint32_t bcr;

	__ASSERT((config->phase_seg1 + config->prop_seg >= 4) &&
		 (config->phase_seg1 + config->prop_seg <= 16),
		 "4 <= phase-seg1 + prop-seg <= 16");

	__ASSERT((config->sjw >= 1) && (config->sjw <= 4),
		 "1 <= sjw <= 4");

	__ASSERT((config->phase_seg2 >= 2) && (config->phase_seg2 <= 8),
		 "2 <= phase-seg2 <= 8");

	const uint8_t bit_length = 1 + config->prop_seg + config->phase_seg1 +
		config->phase_seg2;

	__ASSERT((config->bus_clk.rate % (bit_length * bitrate)) == 0,
		 "Prescaler is not a natural number!");

	brp = config->bus_clk.rate / (bit_length * bitrate) - 1;
	__ASSERT((brp >= 0) && (brp <= 1023),
		 "0 <= prescaler division ratio (BRP) <= 1023");

	bcr = RCAR_CAN_BCR_TSEG1(config->phase_seg1 + config->prop_seg - 1) |
	      RCAR_CAN_BCR_BPR(brp) | RCAR_CAN_BCR_SJW(config->sjw - 1) |
	      RCAR_CAN_BCR_TSEG2(config->phase_seg2 - 1);
	/* Don't overwrite CLKR with 32-bit BCR access; CLKR has 8-bit access.
	 * All the registers are big-endian but they get byte-swapped on 32-bit
	 * read/write (but not on 8-bit, contrary to the manuals)...
	 */
	sys_write32((bcr << 8) | RCAR_CAN_CLKR_CLKP2, config->reg_addr + RCAR_CAN_BCR);
}

int can_rcar_runtime_configure(const struct device *dev, enum can_mode mode,
				uint32_t bitrate)
{
	const struct can_rcar_cfg *config = DEV_CAN_CFG(dev);
	struct can_rcar_data *data = DEV_CAN_DATA(dev);
	uint8_t tcr;
	int ret = 0;

	if (!bitrate)
		bitrate = config->bus_speed;

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	/* Changing bittiming should be done in reset mode */
	ret = can_rcar_enter_reset_mode(config, true);
	if (ret != 0)
		goto unlock;

	can_rcar_set_bittiming(config, bitrate);

	switch (mode) {
	case CAN_NORMAL_MODE:
		tcr = 0;
		break;
	/*Controller is not allowed to send dominant bits*/
	case CAN_SILENT_MODE:
		tcr = RCAR_CAN_TCR_LISTEN_ONLY | RCAR_CAN_TCR_TSTE;
		break;
	/*Controller is in loopback mode (receive own messages)*/
	case CAN_LOOPBACK_MODE:
		tcr = RCAR_CAN_TCR_INT_LOOP | RCAR_CAN_TCR_TSTE;
		break;
	/*Combination of loopback and silent*/
	case CAN_SILENT_LOOPBACK_MODE:
		ret = -ENOSYS;
		goto unlock;
	}

	/* Writting to TCR registers must be done in halt mode */
	ret = can_rcar_enter_halt_mode(config);
	if (ret)
		goto unlock;

	sys_write8(tcr, config->reg_addr + RCAR_CAN_TCR);
	/* Go back to operation mode */
	ret = can_rcar_enter_operation_mode(config);

unlock:
	k_mutex_unlock(&data->inst_mutex);
	return ret;
}

static void can_rcar_register_state_change_isr(const struct device *dev,
						can_state_change_isr_t isr)
{
}

static enum can_state can_rcar_get_state(const struct device *dev,
					  struct can_bus_err_cnt *err_cnt)
{
	return CAN_ERROR_ACTIVE;
}

#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
int can_rcar_recover(const struct device *dev, k_timeout_t timeout)
{
	return 0;
}
#endif /* CONFIG_CAN_AUTO_BUS_OFF_RECOVERY */

int can_rcar_send(const struct device *dev, const struct zcan_frame *msg,
		   k_timeout_t timeout, can_tx_callback_t callback,
		   void *callback_arg)
{
	const struct can_rcar_cfg *config = DEV_CAN_CFG(dev);
	struct can_rcar_data *data = DEV_CAN_DATA(dev);
	uint32_t identifier;
	int i;

	LOG_DBG("Sending %d bytes on %s. "
		    "Id: 0x%x, "
		    "ID type: %s, "
		    "Remote Frame: %s"
		    , msg->dlc, dev->name
		    , msg->id_type == CAN_STANDARD_IDENTIFIER ?
				      msg->std_id :  msg->ext_id
		    , msg->id_type == CAN_STANDARD_IDENTIFIER ?
		    "standard" : "extended"
		    , msg->rtr == CAN_DATAFRAME ? "no" : "yes");


	__ASSERT(msg->dlc == 0U || msg->data != NULL, "Dataptr is null");

	if (msg->dlc > CAN_MAX_DLC) {
		LOG_ERR("DLC of %d exceeds maximum (%d)", msg->dlc, CAN_MAX_DLC);
		return CAN_TX_EINVAL;
	}

	k_mutex_lock(&data->inst_mutex, K_FOREVER);
	data->tx_callback = callback;
	data->tx_callback_arg = callback_arg;
	k_sem_reset(&data->tx_int_sem);

	if (msg->id_type == CAN_STANDARD_IDENTIFIER) {
		identifier = msg->std_id << RCAR_CAN_MB_SID_SHIFT;
	} else {
		identifier = msg->ext_id | RCAR_CAN_MB_IDE;
	}

	if (msg->rtr == CAN_REMOTEREQUEST) {
		identifier |= RCAR_CAN_MB_RTR;
	}

	sys_write32(identifier, config->reg_addr + RCAR_CAN_MB_56);

	sys_write16(msg->dlc, config->reg_addr
			   + RCAR_CAN_MB_56 + RCAR_CAN_MB_DLC_OFFSET);

	for (i = 0; i < msg->dlc; i++)
		sys_write8(msg->data[i], config->reg_addr
			   + RCAR_CAN_MB_56 + RCAR_CAN_MB_DATA_OFFSET + i);

	/* Start Tx: increment the CPU-side pointer for the transmit FIFO
	 * to the next mailbox location
	 */
	sys_write8(0xff, config->reg_addr + RCAR_CAN_TFPCR);

	k_mutex_unlock(&data->inst_mutex);
	if (callback == NULL) {
		/* Does we should admit that we failed at some point ? */
		k_sem_take(&data->tx_int_sem, K_FOREVER);
		return data->error_flags;
	}

	return CAN_TX_OK;
}

static inline int can_rcar_attach(const struct device *dev,
				   can_rx_callback_t cb,
				   void *cb_arg,
				   const struct zcan_filter *filter)
{
	struct can_rcar_data *data = DEV_CAN_DATA(dev);
	int i;
	for (i = 0; i < CONFIG_CAN_RCAR_MAX_FILTER; i++)
		if (data->rx_callback[i] == NULL) {
			data->rx_callback[i] = cb;
			data->rx_callback_arg[i] = cb_arg;
			data->filter[i] = *filter;
			return i;
		}
	return CAN_NO_FREE_FILTER;
}

int can_rcar_attach_isr(const struct device *dev, can_rx_callback_t isr,
			 void *cb_arg,
			 const struct zcan_filter *filter)
{
	struct can_rcar_data *data = DEV_CAN_DATA(dev);
	int filter_nr;

	k_mutex_lock(&data->rx_mutex, K_FOREVER);
	filter_nr = can_rcar_attach(dev, isr, cb_arg, filter);
	k_mutex_unlock(&data->rx_mutex);
	return filter_nr;
}

void can_rcar_detach(const struct device *dev, int filter_nr)
{
	struct can_rcar_data *data = DEV_CAN_DATA(dev);
	if (filter_nr >= CONFIG_CAN_RCAR_MAX_FILTER)
		return;

	k_mutex_lock(&data->rx_mutex, K_FOREVER);
	data->rx_callback[filter_nr] = NULL;
	k_mutex_unlock(&data->rx_mutex);
}

static int can_rcar_init(const struct device *dev)
{
	const struct can_rcar_cfg *config = DEV_CAN_CFG(dev);
	struct can_rcar_data *data = DEV_CAN_DATA(dev);
	const struct device *clk;
	int ret;
	uint16_t ctlr;

	k_mutex_init(&data->inst_mutex);
	k_mutex_init(&data->rx_mutex);
	k_sem_init(&data->tx_int_sem, 0, 1);
	k_sem_init(&data->rx_int_sem, 0, 1);
	data->int_thread_stack = can_rcar_thread_stack;
	data->tx_callback = NULL;
	memset(data->rx_callback, 0, sizeof(data->rx_callback));
	data->dev = dev;

	if (config->clock_controller) {
		clk = device_get_binding(config->clock_controller);
		if (!clk) {
			return -ENODEV;
		}

		/* reset the registers */
		ret = clock_control_off(clk,
				(clock_control_subsys_t *) &config->mod_clk);
		if (ret < 0) {
			return ret;
		}

		ret = clock_control_on(clk,
				(clock_control_subsys_t *) &config->mod_clk);
		if (ret < 0) {
			return ret;
		}

		ret = clock_control_on(clk,
				(clock_control_subsys_t *) &config->bus_clk);
		if (ret < 0) {
			return ret;
		}
	}

	ret = can_rcar_enter_reset_mode(config, false);
	__ASSERT(!ret, "Fail to set CAN controller to reset mode");
	if (ret)
		return ret;

	ret = can_rcar_leave_sleep_mode(config);
	__ASSERT(!ret, "Fail to leave CAN controller from sleep mode");
	if (ret)
		return ret;

	can_rcar_set_bittiming(config, config->bus_speed);

	ctlr = can_rcar_read16(config, RCAR_CAN_CTLR);
	ctlr |= RCAR_CAN_CTLR_IDFM_MIXED; /* Select mixed ID mode */
	ctlr |= RCAR_CAN_CTLR_BOM_ENT;	/* Entry to halt mode automatically */
					/* at bus-off */
	ctlr |= RCAR_CAN_CTLR_MBM;	/* Select FIFO mailbox mode */
	ctlr |= RCAR_CAN_CTLR_MLM;	/* Overrun mode */
	ctlr &= ~RCAR_CAN_CTLR_SLPM; /* Clear CAN Sleep mode */
	can_rcar_write16(config, RCAR_CAN_CTLR, ctlr);

	/* Accept all SID and EID */
	sys_write32(0, config->reg_addr + RCAR_CAN_MKR8);
	sys_write32(0, config->reg_addr + RCAR_CAN_MKR9);
	/* In FIFO mailbox mode, write "0" to bits 24 to 31 */
	sys_write32(0, config->reg_addr + RCAR_CAN_MKIVLR0);
	sys_write32(0, config->reg_addr + RCAR_CAN_MKIVLR1);
	/* Accept standard and extended ID frames, but not
	 * remote frame.
	 */
	sys_write32(0, config->reg_addr + RCAR_CAN_FIDCR0);
	sys_write32(RCAR_CAN_FIDCR_IDE,
		    config->reg_addr + RCAR_CAN_FIDCR1);

	/* Enable and configure FIFO mailbox interrupts Rx and Tx */
	sys_write32(RCAR_CAN_MIER1_RXFIE | RCAR_CAN_MIER1_TXFIE,
			config->reg_addr + RCAR_CAN_MIER1);

	sys_write8(RCAR_CAN_IER_ERSIE | RCAR_CAN_IER_RXFIE | RCAR_CAN_IER_TXFIE,
			config->reg_addr + RCAR_CAN_IER);

	/* Accumulate error codes */
	sys_write8(RCAR_CAN_ECSR_EDPM, config->reg_addr + RCAR_CAN_ECSR);

	/* Enable interrupts for all type of errors */
	sys_write8(0xFF, config->reg_addr + RCAR_CAN_EIER);

	/* start interrupt thread to handle rx frames */
	k_thread_create(&data->int_thread, data->int_thread_stack,
			config->int_thread_stack_size,
			(k_thread_entry_t) can_rcar_rx_thread, (void *)dev,
			NULL, NULL, K_PRIO_COOP(config->int_thread_priority),
			0, K_NO_WAIT);

	/* Go to operation mode */
	ret = can_rcar_enter_operation_mode(config);
	__ASSERT(!ret, "Fail to set CAN controller to operation mode");
	if (ret)
		return ret;

	config->init_func(dev);
	return 0;
}

static const struct can_driver_api can_rcar_driver_api = {
	.configure = can_rcar_runtime_configure,
	.send = can_rcar_send,
	.attach_isr = can_rcar_attach_isr,
	.detach = can_rcar_detach,
	.get_state = can_rcar_get_state,
#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
	.recover = can_rcar_recover,
#endif
	.register_state_change_isr = can_rcar_register_state_change_isr
};

/* Device Instantiation */
#define CAN_RCAR_INIT(n) \
	static void can_rcar_##n##_init(const struct device *dev);	\
	static const struct can_rcar_cfg can_rcar_cfg_##n = {         \
		.reg_addr = DT_INST_REG_ADDR(n), \
		.reg_size = DT_INST_REG_SIZE(n), \
		.init_func = can_rcar_##n##_init, \
		.clock_controller = DT_INST_CLOCKS_LABEL(n), \
		.mod_clk.module = \
		DT_INST_CLOCKS_CELL_BY_IDX(n, 0, module), \
		.mod_clk.domain = \
		DT_INST_CLOCKS_CELL_BY_IDX(n, 0, domain), \
		.bus_clk.module = \
		DT_INST_CLOCKS_CELL_BY_IDX(n, 1, module), \
		.bus_clk.domain = \
		DT_INST_CLOCKS_CELL_BY_IDX(n, 1, domain), \
		.bus_clk.rate = 40000000,		  \
		.bus_speed = DT_INST_PROP(n, bus_speed), \
		.sjw = DT_INST_PROP(n, sjw), \
		.prop_seg = DT_INST_PROP(n, prop_seg), \
		.phase_seg1 = DT_INST_PROP(n, phase_seg1), \
		.phase_seg2 = DT_INST_PROP(n, phase_seg2), \
		.int_thread_stack_size = CONFIG_CAN_RCAR_INT_THREAD_STACK_SIZE, \
		.int_thread_priority = CONFIG_CAN_RCAR_INT_THREAD_PRIO, \
	};					       \
	static struct can_rcar_data can_rcar_data_##n; \
\
	DEVICE_AND_API_INIT(rcar_can_##n, \
			    DT_INST_LABEL(n), \
			    can_rcar_init, \
			    &can_rcar_data_##n, \
			    &can_rcar_cfg_##n, \
			    POST_KERNEL, \
			    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, \
			    &can_rcar_driver_api \
			   ); \
	static void can_rcar_##n##_init(const struct device *dev)	\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    0,						\
			    can_rcar_isr,				\
			    DEVICE_GET(rcar_can_##n), 0);		\
									\
		irq_enable(DT_INST_IRQN(n));				\
	}


DT_INST_FOREACH_STATUS_OKAY(CAN_RCAR_INIT)
