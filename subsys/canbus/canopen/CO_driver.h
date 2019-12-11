/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CO_DRIVER_H
#define CO_DRIVER_H

/*
 * Zephyr RTOS CAN driver interface and configuration for CANopenNode
 * CANopen protocol stack.
 *
 * See CANopenNode/stack/drvTemplate/CO_driver.h for API description.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr.h>
#include <zephyr/types.h>
#include <device.h>
#include <toolchain.h>

/* Use static variables instead of calloc() */
#define CO_USE_GLOBALS

/* Use Zephyr provided crc16 implementation */
#define CO_USE_OWN_CRC16

/* Use SDO buffer size from Kconfig */
#define CO_SDO_BUFFER_SIZE CONFIG_CANOPEN_SDO_BUFFER_SIZE

/* Use trace buffer size from Kconfig */
#define CO_TRACE_BUFFER_SIZE_FIXED CONFIG_CANOPEN_TRACE_BUFFER_SIZE

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define CO_LITTLE_ENDIAN
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define CO_BIG_ENDIAN
#else
#error "Unsupported endianness"
#endif

typedef bool          bool_t;
typedef float         float32_t;
typedef long double   float64_t;
typedef char          char_t;
typedef unsigned char oChar_t;
typedef unsigned char domain_t;

typedef enum{
	CO_ERROR_NO               =  0,
	CO_ERROR_ILLEGAL_ARGUMENT = -1,
	CO_ERROR_OUT_OF_MEMORY    = -2,
	CO_ERROR_TIMEOUT          = -3,
	CO_ERROR_ILLEGAL_BAUDRATE = -4,
	CO_ERROR_RX_OVERFLOW      = -5,
	CO_ERROR_RX_PDO_OVERFLOW  = -6,
	CO_ERROR_RX_MSG_LENGTH    = -7,
	CO_ERROR_RX_PDO_LENGTH    = -8,
	CO_ERROR_TX_OVERFLOW      = -9,
	CO_ERROR_TX_PDO_WINDOW    = -10,
	CO_ERROR_TX_UNCONFIGURED  = -11,
	CO_ERROR_PARAMETERS       = -12,
	CO_ERROR_DATA_CORRUPT     = -13,
	CO_ERROR_CRC              = -14
} CO_ReturnError_t;

typedef struct {
	u8_t DLC;
	u8_t data[8];
} CO_CANrxMsg_t;

typedef void (*CO_CANrxBufferCallback_t)(void *object,
					 const CO_CANrxMsg_t *message);

typedef struct {
	u16_t ident;
	int filter_id;
	void *object;
	CO_CANrxBufferCallback_t pFunct;
} CO_CANrx_t;

typedef struct{
	u16_t ident;
	bool_t rtr;
	u8_t DLC;
	u8_t data[8];
	volatile bool_t bufferFull;
	volatile bool_t syncFlag;
} CO_CANtx_t;

typedef struct {
	bool_t configured;
	struct device *dev;
	CO_CANrx_t *rx_array;
	u16_t rx_size;
	CO_CANtx_t *tx_array;
	u16_t tx_size;
	volatile bool_t CANnormal;
	volatile bool_t first_tx_msg;
	u32_t errors;
	void *em;
} CO_CANmodule_t;

void canopen_send_lock(void);
void canopen_send_unlock(void);
#define CO_LOCK_CAN_SEND()   canopen_send_lock()
#define CO_UNLOCK_CAN_SEND() canopen_send_unlock()

void canopen_emcy_lock(void);
void canopen_emcy_unlock(void);
#define CO_LOCK_EMCY()   canopen_emcy_lock()
#define CO_UNLOCK_EMCY() canopen_emcy_unlock()

void canopen_od_lock(void);
void canopen_od_unlock(void);
#define CO_LOCK_OD()   canopen_od_lock()
#define CO_UNLOCK_OD() canopen_od_unlock()

void CO_CANsetConfigurationMode(s32_t CANbaseAddress);

void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule);

CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t *CANmodule,
				   s32_t CANbaseAddress,
				   CO_CANrx_t rxArray[], u16_t rxSize,
				   CO_CANtx_t txArray[], u16_t txSize,
				   u16_t CANbitRate);

void CO_CANmodule_disable(CO_CANmodule_t *CANmodule);

CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t *CANmodule, u16_t index,
				    u16_t ident, u16_t mask, bool_t rtr,
				    void *object,
				    CO_CANrxBufferCallback_t pFunct);

CO_CANtx_t *CO_CANtxBufferInit(CO_CANmodule_t *CANmodule, u16_t index,
			       u16_t ident, bool_t rtr, u8_t noOfBytes,
			       bool_t syncFlag);

CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer);

void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule);

void CO_CANverifyErrors(CO_CANmodule_t *CANmodule);

#ifdef __cplusplus
}
#endif

#endif /* CO_DRIVER_H */
