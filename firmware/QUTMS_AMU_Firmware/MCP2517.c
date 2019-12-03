/*****************************************************************************
* @file    firmware/QUTMS_HVBoard_Firmware/MCP2517.c
* @author  Zoe Goodward
* @version V1.0.0
* @date
* @brief
*****************************************************************************/
// 5:39PM 21/11/2019

#define F_CPU 16000000UL /* CPU clock in Hertz */

#include <util/delay.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdbool.h>

#include "input.h"
#include "UART.h"
#include "spi.h"
#include "MCP2517.h"
#include "MCP2517_reg.h"
#include "MCP2517_defines.h"


// *****************************************************************************
// Helper Functions
// *****************************************************************************
inline void enterWordInBufferAtIndex (const uint32_t value, uint8_t buff[], const uint8_t index) {
	buff[index + 0] = (uint8_t) value;
	buff[index + 1] = (uint8_t) (value >> 8);
	buff[index + 2] = (uint8_t) (value >> 16);
	buff[index + 3] = (uint8_t) (value >> 24);
}

inline uint32_t wordFromBufferAtIndex (uint8_t buff[], const uint8_t index) {
	uint32_t result = (uint32_t) buff[index + 0];
	result |= ((uint32_t) buff[index + 1]) << 8;
	result |= ((uint32_t) buff[index + 2]) << 16;
	result |= ((uint32_t) buff[index + 3]) << 24;
	return result;
}


// *****************************************************************************
// SPI Write Functions
// *****************************************************************************
inline void MCP2517_writeReg8(const uint16_t regAddr, const uint8_t value) {
	uint8_t buff[3] = {0};

	buff[0] = (uint8_t) ((MCP2517_INSTRUCTION_WRITE << 4) + ((regAddr >> 8) & 0xF));
	buff[1] = (uint8_t) (regAddr & 0xFF);
	buff[2] = value;
	
	MCP2517_assertCS();
	spi_transfer_buffer(buff, 3);
	MCP2517_deassertCS();
}

inline void MCP2517_writeReg32(const uint16_t regAddr, const uint32_t value) {
	uint8_t buff[6] = {0};
	
	buff[0] = (uint8_t) ((MCP2517_INSTRUCTION_WRITE << 4) + ((regAddr >> 8) & 0xF));
	buff[1] = (uint8_t) (regAddr & 0xFF);
	
	enterWordInBufferAtIndex(value, buff, 2);
	
	MCP2517_assertCS();
	spi_transfer_buffer(buff, 6);
	MCP2517_deassertCS();
}


// *****************************************************************************
// SPI Read Functions
// *****************************************************************************
inline uint8_t MCP2517_readReg8(const uint16_t regAddr) {
	uint8_t buff[3] = {0};

	buff[0] = (uint8_t) ((MCP2517_INSTRUCTION_READ << 4) + ((regAddr >> 8) & 0xF));
	buff[1] = (uint8_t) (regAddr & 0xFF);
	buff[2] = 0x00;
	
	MCP2517_assertCS();
	spi_transfer_buffer(buff, 3);
	MCP2517_deassertCS();
	
	return buff[2];
}

// change to uint8_t
inline uint32_t MCP2517_readReg32(const uint16_t regAddr) {
	uint8_t buff[6] = {0};

	buff[0] = (uint8_t) ((MCP2517_INSTRUCTION_READ << 4) + ((regAddr >> 8) & 0xF));
	buff[1] = (uint8_t) (regAddr & 0xFF);
	buff[2] = 0x00;

	MCP2517_assertCS();
	spi_transfer_buffer(buff, 6);
	MCP2517_deassertCS();

	const uint32_t result = wordFromBufferAtIndex(buff, 2); // change to uint8_t

	return result;
}

// *****************************************************************************
// Assert the MCP2517 CS pin
// *****************************************************************************
void MCP2517_assertCS() {
	CAN_CS_PORT &= ~(1 << CAN_CS);
}

void MCP2517_deassertCS() {
	CAN_CS_PORT |= (1 << CAN_CS);
}


// *****************************************************************************
// MCP2517 Configuration Functions
// *****************************************************************************
// Software reset
void MCP2517_reset() {
	uint8_t buff[2] = {0};
	
	buff[0] = (uint8_t) ((MCP2517_INSTRUCTION_WRITE << 4) + ((MCP2517_REG_ADDR_C1CON >> 8) & 0xF));
	buff[1] = (uint8_t) (MCP2517_REG_ADDR_C1CON & 0xFF);
	
	MCP2517_assertCS();
	spi_transfer_buffer(buff, 2);
	MCP2517_deassertCS();
}

// Set operation mode
void MCP2517_setMode(MCP2517_OPERATION_MODE opMode) {
	// Set mode and abort all transactions
	MCP2517_writeReg8(MCP2517_REG_ADDR_C1CON + 3, opMode | (1 << 3));
}

// Read current operation mode
uint8_t MCP2517_getMode() {
	return (MCP2517_readReg8(MCP2517_REG_ADDR_C1CON + 2) >> 5) & 0x07;
}

// Test that RAM is reading and writing correctly - !!!!
void MCP2517_testRAM() {
	for (uint32_t i = 1 ; i != 0; i <<= 1) {
		MCP2517_writeReg32(MCP2517_RAM_ADDR_START, i);
		const uint32_t readBackValue = MCP2517_readReg32(0x400);
		if (readBackValue != i) {
			uart0_transmit(MCP2517_RAM_ERROR); // Error code
		}
	}
}

// Initalise the MCP2517 chip
void MCP2517_init() {
	//cli(); // Disable interrupts while configuring
	
	// Request configuration mode
	MCP2517_setMode(MCP2517_CONFIGURATION_MODE);
	// Wait 2ms for chip to change modes
	_delay_ms(2);
	// Check that chip is now in config mode
	uint8_t mode = MCP2517_getMode();
	if(mode != MCP2517_CONFIGURATION_MODE) {
		//LED_A_ON;
		uart0_transmit(MCP2517_MODE_SELECT_ERROR);
	}
	//
	// Perform software reset
	MCP2517_reset();
	// Let the chip change modes
	_delay_ms(2);
	// Check that chip has flipped
	mode = MCP2517_getMode();
	if(mode != MCP2517_CONFIGURATION_MODE) {
		//LED_B_ON;
		uart0_transmit(MCP2517_MODE_SELECT_ERROR);
	}
	
	// Configure the Bit Time registers: 250K/2M, 80% sample point
	uint32_t data = 0x00; // BRP = 0
	data <<= 8;
	data |= 0x3E; // TSEG1 = 62
	data <<= 8;
	data |= 0x0F; // TSEG2 = 15
	data <<= 8;
	data |= 0x0F; // SJW = 15
	MCP2517_writeReg32(MCP2517_REG_ADDR_C1NBTCFG, data);
	
	// FIFO 1: Transmit FIFO; 5 messages, 64 byte maximum payload, low priority
	uint8_t d = 0xEF; // FiFo Size: 7, Pay Load Size: 64 bytes
	MCP2517_writeReg8(MCP2517_REG_ADDR_C1FIFOCON + (MCP2517_TX_FIFO * MCP2517_C1FIFO_OFFSET) + 3, d);
	d = 1 << 7; // FIFO is a TX FIFO
	MCP2517_writeReg8(MCP2517_REG_ADDR_C1FIFOCON + (MCP2517_TX_FIFO * MCP2517_C1FIFO_OFFSET), d);
	d = 0x03; // Unlimited retransmission attempts
	d <<= 5;
	d |= 0x00; // Lowest message priority
	MCP2517_writeReg8(MCP2517_REG_ADDR_C1FIFOCON + (MCP2517_TX_FIFO * MCP2517_C1FIFO_OFFSET) + 2, d);
	
	// FIFO 2: Receive FIFO
	d = 0b00000011; // Payload size: 8, FiFo size: 4
	MCP2517_writeReg8(MCP2517_REG_ADDR_C1FIFOCON + (MCP2517_RX_FIFO * MCP2517_C1FIFO_OFFSET) + 3, d);
	// Enable interrupt for FIFO
	//d = 1; // FIFO not Empty
	//MCP2517_writeReg8(MCP2517_REG_ADDR_C1FIFOCON + (SHUTDOWN_RX_FIFO * MCP2517_C1FIFO_OFFSET), d);
	
	// Link FIFO and Filter
	d = 1 << 7; // Filter is enabled
	d |= 0x02; // Message matching filter is stored in FIFO2
	MCP2517_writeReg8(MCP2517_REG_ADDR_C1FLTCON + MCP2517_FILTER0, d);
	
	// Enable ECC
	d = 0x01;
	MCP2517_writeReg8(MCP2517_REG_ADDR_ECCCON, d);
	
	// Configuration Done: Select CAN 2.0B Mode - For testing use external loopback
	MCP2517_setMode(MCP2517_CLASSIC_MODE);
	_delay_ms(2);
	mode = MCP2517_getMode();
	if(mode != MCP2517_CLASSIC_MODE) {
		//LED_A_ON;
		//uart0_transmit(MCP2517_MODE_SELECT_ERROR);
	}
	
	//// Initialise and test RAM
	////MCP2517_testRAM();
	//
	//// Configure the osC1llator and CLKO pin
	////uint8_t osc = 0x00; // Divide by 1
	////MCP2517_writeReg8(MCP2517_REG_ADDR_OSC, osc);
	//
	//// Speed up SPI post reset
	////spi_init(0,0);
	//
	//// Configure the I/O pins: use INT
	//uint8_t io = 0x03; // PM0 and PM1 are GPIO pins
	//MCP2517_writeReg8(MCP2517_REG_ADDR_IOCON, io);
	////MCP2517_readReg8(MCP2517_REG_ADDR_IOCON);
	//
	//// Configure TXQ
	////d = 0x00; // TXQ disabled
	////MCP2517_writeReg8(MCP2517_REG_ADDR_C1TXQCON + 3, d);
	//
	//// Configure the CAN Control register:
	////d = 0x00; // Disable TXQ and don't save TEF
	////MCP2517_writeReg8(MCP2517_REG_ADDR_C1CON + 2, d);
	//
	//
	//
	//// Setup RX Filter:
	//uint32_t data = 0x01; // EXIDE - enable extended identifiers & SID11
	//data <<= 6;
	//data |= MCP2517_ID_PDM; // EID =
	//data <<= 18;
	//data |= MCP2517_ID_PDM >> 18; // SID =
	//MCP2517_writeReg32(MCP2517_REG_ADDR_C1FLTOBJ + (MCP2517_FILTER0 * C1FILTER_OFFSET), data);
	//
	//// Setup RX Mask
	//data = 0x01; // MIDE - match only message types that correspond to EXIDE bit in filter
	
	//
	//// Activate interrupts
	//d = (1 << 1); // Receive FIFO interrupt enable
	//d |= (1 << 0); // Transmit FIFO interrupt enable
	//MCP2517_writeReg8(MCP2517_REG_ADDR_C1INT + 2, d);
	//MCP2517_writeReg8(MCP2517_REG_ADDR_C1INT + 3, 0x00);
	////
	//
	//
	//
	////sei(); // Re-enable interrupts
}



// *****************************************************************************
// MCP2517 FIFO Status Functions
// *****************************************************************************
uint8_t MCP2517_receiveFifoStatus(MCP2517_FIFO_CHANNEL channel, MCP2517_RX_FIFO_STATUS *flags) {
	uint8_t status = MCP2517_readReg8(MCP2517_REG_ADDR_C1FIFOSTA + (channel * MCP2517_C1FIFO_OFFSET));
	
	// Update channel status
	*flags = (MCP2517_RX_FIFO_STATUS) ((status) & MCP2517_RX_FIFO_ALL_STATUS);
	
	return status;
}


uint8_t MCP2517_transmitFifoStatus(MCP2517_FIFO_CHANNEL channel, MCP2517_TX_FIFO_STATUS *flags) {
	uint8_t status = MCP2517_readReg8(MCP2517_REG_ADDR_C1FIFOSTA + (channel * MCP2517_C1FIFO_OFFSET));
	
	// Update channel status
	*flags = (MCP2517_TX_FIFO_STATUS) ((status) & MCP2517_TX_FIFO_ALL_STATUS);
	
	return status;
}


// *****************************************************************************
// MCP2517 Receive Functions
// *****************************************************************************
void MCP2517_recieveMessage(CAN_RECEIVE_ADDRESS *receiveID, uint8_t *numDataBytes, uint8_t *data) {
	MCP2517_RX_MSG_OBJ rxObj;
	MCP2517_RX_FIFO_STATUS rxFlags;
	
	// Check that FIFO is not empty
	MCP2517_receiveFifoStatus(MCP2517_RX_FIFO, &rxFlags);
	
	if (rxFlags & MCP2517_RX_FIFO_NOT_EMPTY_STATUS) {
		///LED_B_ON;
		// Read message
		MCP2517_readMsgReceive(receiveID, numDataBytes, data, &rxObj);
		_delay_ms(100);
		//LED_B_OFF;
	}
}

void MCP2517_readMsgReceive(CAN_RECEIVE_ADDRESS *receiveID, uint8_t *numDataBytes, uint8_t *data, MCP2517_RX_MSG_OBJ *rxObj) {
	
	uint8_t buff[MCP2517_MAX_MSG_SIZE] = {0}; // Max size of transmit message
	
	// Write instruction
	const uint16_t regAddr = MCP2517_RAM_ADDR_START + MCP2517_readReg32(MCP2517_REG_ADDR_C1FIFOUA + (MCP2517_RX_FIFO * MCP2517_C1FIFO_OFFSET));
	buff[0] = (uint8_t) ((MCP2517_INSTRUCTION_READ << 4) + ((regAddr >> 8) & 0xF));
	buff[1] = (uint8_t) (regAddr & 0xFF);
	
	// SPI Transfer
	MCP2517_assertCS();
	spi_transfer_buffer(buff, sizeof(buff));
	MCP2517_deassertCS();
	
	// Get frame ID and Control bits
	rxObj->MCP2517_word[0] = 0;
	rxObj->MCP2517_word[1] = 0;
	
	rxObj->MCP2517_byte[0] = buff[2];
	rxObj->MCP2517_byte[1] = buff[3];
	rxObj->MCP2517_byte[2] = buff[4];
	rxObj->MCP2517_byte[3] = buff[5];
	
	rxObj->MCP2517_byte[4] = buff[6];
	rxObj->MCP2517_byte[5] = buff[7];
	rxObj->MCP2517_byte[6] = buff[8];
	rxObj->MCP2517_byte[7] = buff[9];
	
	// Get message ID
	*receiveID = (uint32_t) (rxObj->MCP2517_bF.MCP2517_id.MCP2517_SID);
	
	// Get the number of data bytes (size of payload)
	*numDataBytes = (uint8_t) (rxObj->MCP2517_byte[4] & 0xF);

	rxObj->MCP2517_word[2] = 0;
	
	for(uint8_t i = 0; i < 8; i++) {
		data[i] = buff[i + 10];
	}
	
	// Increment FIFO buffer - set UNIC bit - Update channel
	const uint8_t d = 1 << 0;
	MCP2517_writeReg8(MCP2517_REG_ADDR_C1FIFOCON + (MCP2517_RX_FIFO * MCP2517_C1FIFO_OFFSET), d);
}

// *****************************************************************************
// MCP2517 Transmit Functions
// *****************************************************************************
uint8_t MCP2517_transmitMessage(CAN_SEND_ADDRESS canMessageID, uint8_t numDataBytes, uint8_t *messageData) {
	
	// Check if numDataBytes > 8
	if (numDataBytes > 8) {
		uart0_transmit(MCP2517_MESSAGE_SIZE_ERROR);
		return MCP2517_MESSAGE_SIZE_ERROR;
	}
	
	// Create transmit message object
	MCP2517_TX_MSG_OBJ txObj;
	
	// Set ID and CTRL bits to 0
	txObj.MCP2517_word[0] = 0;
	txObj.MCP2517_word[1] = 0;
	// Configure ID bits
	txObj.MCP2517_bF.MCP2517_id.MCP2517_SID = canMessageID >> 18; // Base ID
	txObj.MCP2517_bF.MCP2517_id.MCP2517_EID = canMessageID; // Extended ID
	// Configure CTRL bits
	txObj.MCP2517_bF.MCP2517_ctrl.MCP2517_FDF = 0; // CAN 2.B frame
	txObj.MCP2517_bF.MCP2517_ctrl.MCP2517_BRS = 1; // Switch data bit rate
	txObj.MCP2517_bF.MCP2517_ctrl.MCP2517_IDE = 1; // Extended format frame
	txObj.MCP2517_bF.MCP2517_ctrl.MCP2517_RTR = 0; // Not a remote frame request
	txObj.MCP2517_bF.MCP2517_ctrl.MCP2517_DLC = numDataBytes; // Data length code

	// Check that FIFO is not full
	MCP2517_TX_FIFO_STATUS txFlags;

	MCP2517_transmitFifoStatus(MCP2517_TX_FIFO, &txFlags);
	
	// If not full proceed to append FIFO to buffer and transmit
	if (txFlags & MCP2517_TX_FIFO_NOT_FULL_STATUS) {
		//LED_A_ON;
		MCP2517_loadMsgTXFifo(&txObj, messageData, numDataBytes);
		_delay_ms(100);
		//LED_A_OFF;
	}
	
	return MCP2517_NO_ERROR;
}


void MCP2517_loadMsgTXFifo(MCP2517_TX_MSG_OBJ *txObj, uint8_t *payload, uint8_t numDataBytes) {
	
	uint8_t buff[MCP2517_MAX_MSG_SIZE] = {0}; // Max number of transmit bytes
	
	// Write only multiples of 4 to RAM
	uint8_t i;
	uint16_t n = 0;
	uint8_t j = 0;

	if (numDataBytes % 4) { // Largest value may be 3, lowest 0 - 1
		// Add bytes
		n = 4 - (numDataBytes % 4);
		i = numDataBytes + 8;
		for (j = 0; j < n; j++) {
			buff[i + 8 + j] = 0;
		}
	}
	
	// Write instruction
	const uint16_t regAddr = MCP2517_RAM_ADDR_START + MCP2517_readReg32(MCP2517_REG_ADDR_C1FIFOUA + (MCP2517_TX_FIFO * MCP2517_C1FIFO_OFFSET));
	buff[0] = (uint8_t) ((MCP2517_INSTRUCTION_WRITE << 4) + ((regAddr >> 8) & 0xF));
	buff[1] = (uint8_t) (regAddr & 0xFF);
	
	// Add TX Message Object's ID bits to buffer
	buff[2] = txObj->MCP2517_byte[0];
	buff[3] = txObj->MCP2517_byte[1];
	buff[4] = txObj->MCP2517_byte[2];
	buff[5] = txObj->MCP2517_byte[3];
	// Add TX Message Object's ID Control bits to buffer
	buff[6] = txObj->MCP2517_byte[4];
	buff[7] = txObj->MCP2517_byte[5];
	buff[8] = txObj->MCP2517_byte[6];
	buff[9] = txObj->MCP2517_byte[7];
	
	// Loop through and add the payload data bytes to the buffer
	for (i = 0; i < numDataBytes; i++) {
		buff[i + 10] = payload[i];
	}

	// Send CAN packet via SPI
	MCP2517_assertCS();
	spi_transfer_buffer(buff, sizeof(buff));
	MCP2517_deassertCS();
	
	// Increment FIFO and send message
	const uint8_t d = (1 << 0) | (1 << 1); // Set UINC, TXREQ bit
	MCP2517_writeReg8(MCP2517_REG_ADDR_C1FIFOCON + (MCP2517_TX_FIFO * MCP2517_C1FIFO_OFFSET) + 1, d);
}

// RESET CHIP IF GOES TO RESTRICTED ACCESS MODE