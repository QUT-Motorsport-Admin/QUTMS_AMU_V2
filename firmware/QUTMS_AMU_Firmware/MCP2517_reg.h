/*
 * QUTMS_HVBoard_Firmware.c
 *
 * Created: 24/11/2019 11:50:53 AM
 * Author : Zoe Goodward
 */

#ifndef MCP2517_reg_H_
#define MCP2517_reg_H_

// *****************************************************************************
// SPI Instruction Commands
// *****************************************************************************
#define MCP2517_INSTRUCTION_RESET			0x00
#define MCP2517_INSTRUCTION_READ			0x03
#define MCP2517_INSTRUCTION_READ_CRC		0x0B
#define MCP2517_INSTRUCTION_WRITE			0x02
#define MCP2517_INSTRUCTION_WRITE_CRC		0x0A
#define MCP2517_INSTRUCTION_WRITE_SAFE		0x0C

// *****************************************************************************
// MCP2517FD Specific Register Addresses
// *****************************************************************************
#define MCP2517_REG_ADDR_OSC        0xE00
#define MCP2517_REG_ADDR_IOCON      0xE04
#define MCP2517_REG_ADDR_CRC    	0xE08
#define MCP2517_REG_ADDR_ECCCON  	0xE0C
#define MCP2517_REG_ADDR_ECCSTA  	0xE10

// *****************************************************************************
// MCP2517 Register Addresses (in RAM)
// *****************************************************************************
// Configuration Registers
#define MCP2517_REG_ADDR_C1CON  	0x000 // CAN Control Register
#define MCP2517_REG_ADDR_C1NBTCFG	0x004 // Normal Bit Time Configuration Register
#define MCP2517_REG_ADDR_C1DBTCFG	0x008 // Data Bit Time Configuration Register
#define MCP2517_REG_ADDR_C1TDC  	0x00C // Transmitter Delay Compensation Register
#define MCP2517_REG_ADDR_C1TBC      0x010 // Time Base Counter Register
#define MCP2517_REG_ADDR_C1TSCON    0x014 // Time Stamp Control Register

// Interrupt and Status Registers
#define MCP2517_REG_ADDR_C1VEC      0x018 // Interrupt Code Register
#define MCP2517_REG_ADDR_C1INT      0x01C // Interrupt Register
#define MCP2517_REG_ADDR_C1RXIF     0x020 // Receive Interrupt Status Register
#define MCP2517_REG_ADDR_C1TXIF     0x024 // Transmit Interrupt Status Register
#define MCP2517_REG_ADDR_C1RXOVIF   0x028 // Receive Overflow Interrupt Status Register
#define MCP2517_REG_ADDR_C1TXATIF   0x02C // Transmit Attempt Interrupt Status Register
#define MCP2517_REG_ADDR_C1TXREQ    0x030 // Transmit Request Register

// Error and Diagnostic Registers
#define MCP2517_REG_ADDR_C1TREC     0x034 // Transmit/Receive Error Count Register
#define MCP2517_REG_ADDR_C1BDIAG0   0x038 // Bus Diagnostic Register 0
#define MCP2517_REG_ADDR_C1BDIAG1   0x03C // Bus Diagnostic Register 1

// FIFO Control and Status Registers
#define MCP2517_REG_ADDR_C1TEFCON   0x040 // Transmit Event FIFO Control Register
#define MCP2517_REG_ADDR_C1TEFSTA   0x044 // Transmit Event FIFO Status Register
#define MCP2517_REG_ADDR_C1TEFUA    0x048 // Transmit Event FIFO User Address Register
#define MCP2517_REG_ADDR_C1TXQCON	0x050 // Transmit Queue Control Register
#define MCP2517_REG_ADDR_C1TXQSTA	0x054 // Transmit Queue Status Register
#define MCP2517_REG_ADDR_C1TXQUA	0x058 // Transmit Queue User Address Register
#define MCP2517_REG_ADDR_C1FIFOCON  0x050 // FIFO Control Register (1 - 31)
#define MCP2517_REG_ADDR_C1FIFOSTA  0x054 // FIFO Status Register (1 - 31)
#define MCP2517_REG_ADDR_C1FIFOUA   0x058 // FIFO User Address Register (1 - 31)

// Helper Values
#define MCP2517_C1FIFO_OFFSET       0x0C // Difference between the start of each FIFO channel
#define MCP2517_C1FILTER_OFFSET     0x08 // Difference between the start of each filter

// Filter Configuration and Control Registers
#define MCP2517_REG_ADDR_C1FLTCON   (MCP2517_REG_ADDR_C1FIFOCON + (MCP2517_C1FIFO_OFFSET * MCP2517_FIFO_CHANNEL_TOTAL))
#define MCP2517_REG_ADDR_C1FLTOBJ   (MCP2517_REG_ADDR_C1FLTCON + MCP2517_FIFO_CHANNEL_TOTAL)
#define MCP2517_REG_ADDR_C1MASK     (MCP2517_REG_ADDR_C1FLTOBJ + 0x04)

// *****************************************************************************
// MCP2517 RAM Address
// *****************************************************************************
#define MCP2517_RAM_ADDR_START 0x400
#define MCP2517_RAM_SIZE       2048
#define MCP2517_RAM_ADDR_END   (MCP2517_RAM_ADDR_START + MCP2517_RAM_SIZE)

#endif // MCP2517_reg_H