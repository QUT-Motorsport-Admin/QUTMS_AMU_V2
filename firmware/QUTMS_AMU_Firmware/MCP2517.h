/*****************************************************************************
* @file    firmware/QUTMS_HVBoard_Firmware/MCP2517.h
* @author  Zoe Goodward
* @version V1.0.0
* @date    21/11/2019 5:50:22 PM
* @brief   This file..
*****************************************************************************/

#ifndef MCP2517_H_
#define MCP2517_H_

#include "MCP2517_defines.h"

// Transmit Channels
#define MCP2517_TX_FIFO MCP2517_FIFO_CH1
// Receive Channels
#define MCP2517_RX_FIFO MCP2517_FIFO_CH2

#define MCP2517_MAX_MSG_SIZE 21

// CAN IDs
typedef enum {
	CAN_RECEIVE_ID_INV = 0x0C000000, /**< CAN Bus Identifier for Inverters 	1100000000000000000000000000 */
	CAN_RECEIVE_ID_AMU = 0x0A000000, /**< CAN Bus Identifier for AMU 		1010000000000000000000000000 */
	CAN_RECEIVE_ID_PDM = 0x09000000, /**< CAN Bus Identifier for PDM 		1001000000000000000000000000 */
	CAN_RECEIVE_ID_SHUTDOWN = 0x08800000, /**< CAN Bus for Shutdown 		1000100000000000000000000000 */
	CAN_RECEIVE_ID_WHEEL = 0x08400000, /**< CAN Bus for Steering Wheel 		1000010000000000000000000000 */
	CAN_RECEIVE_ID_BMS = 0x08200000, /**< CAN Bus for BMS					1000001000000000000000000000 */
	CAN_RECEIVE_ID_HV = 0x08100000, /**< CAN Bus for HV 					1000000100000000000000000000 */
} CAN_RECEIVE_ADDRESS; // When sent from CC to other boards

typedef enum {
	CAN_SEND_ID_INV = 0x04000000, /**< CAN Bus Identifier for Inverters 	0100000000000000000000000000 */
	CAN_SEND_ID_AMU = 0x02000000, /**< CAN Bus Identifier for AMU 		0010000000000000000000000000 */
	CAN_SEND_ID_PDM = 0x01000000, /**< CAN Bus Identifier for PDM 		0001000000000000000000000000 */
	CAN_SEND_ID_SHUTDOWN = 0x0800000, /**< CAN Bus for Shutdown 		0000100000000000000000000000 */
	CAN_SEND_ID_WHEEL = 0x0400000, /**< CAN Bus for Steering Wheel 		0000010000000000000000000000 */
	CAN_SEND_ID_BMS = 0x0200000, /**< CAN Bus for BMS					0000001000000000000000000000 */
	CAN_SEND_ID_HV = 0x0100000, /**< CAN Bus for HV 					0000000100000000000000000000 */
} CAN_SEND_ADDRESS; // When sent from other boards to CC


void MCP2517_init();

void enterWordInBufferAtIndex (const uint32_t value, uint8_t buff[], const uint8_t index);
uint32_t wordFromBufferAtIndex (uint8_t buff[], const uint8_t index);

void MCP2517_writeReg8(const uint16_t regAddr, const uint8_t value);
void MCP2517_writeReg32(const uint16_t regAddr, const uint32_t value);
uint8_t MCP2517_readReg8(const uint16_t regAddr);
uint32_t MCP2517_readReg32(const uint16_t regAddr);

void MCP2517_assertCS();
void MCP2517_deassertCS();

void MCP2517_reset();
void MCP2517_setMode(MCP2517_OPERATION_MODE opMode);
uint8_t MCP2517_getMode();
void MCP2517_testRAM();

uint8_t MCP2517_receiveFifoStatus(MCP2517_FIFO_CHANNEL channel, MCP2517_RX_FIFO_STATUS *flags);
uint8_t MCP2517_transmitFifoStatus(MCP2517_FIFO_CHANNEL channel, MCP2517_TX_FIFO_STATUS *flags);

void MCP2517_recieveMessage(CAN_RECEIVE_ADDRESS *receiveID, uint8_t *numDataBytes, uint8_t *data);
void MCP2517_readMsgReceive(CAN_RECEIVE_ADDRESS *receiveID, uint8_t *numDataBytes, uint8_t *data, MCP2517_RX_MSG_OBJ *rxObj);

uint8_t MCP2517_transmitMessage(CAN_SEND_ADDRESS canMessageID, uint8_t numDataBytes, uint8_t *messageData);
void MCP2517_loadMsgTXFifo(MCP2517_TX_MSG_OBJ *txObj, uint8_t *payload, uint8_t numDataBytes);

#endif /* MCP2517_H_ */