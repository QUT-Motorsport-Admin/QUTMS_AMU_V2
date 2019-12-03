/*
 * QUTMS_HVBoard_Firmware.c
 *
 * Created: 18/09/2019 12:51:26 PM
 * Author : Zoe Goodward
 */

#define F_CPU 16000000UL /* CPU clock in Hertz */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "input.h"
#include "UART.h"
#include "spi.h"
#include "adc.h"
#include "MCP2517.h"
#include "MCP2517_reg.h"

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
uint8_t AMU_DATA[5] = {0};

/*============================================================================
Function:   AMU_init()
------------------------------------------------------------------------------
Purpose :   consolidates all the functions required for the initialisation of
			the AMU board
Input   :   none
Returns :   void
Notes   :
============================================================================*/
void AMU_init() 
{
	DDRB = 0b10110000; // MOSI and SCK and CAN_CS as output, SS output
	DDRA = 0b01010001; // LEDS
	// Set Pre-charge, Shutdown+ and Shutdown- as outputs
	DDRD = 0b00001110;
	
	CAN_CS_PORT |= (1 << CAN_CS); // CS high to turn off
	
	SHDN_NEG_OFF;
	SHDN_POS_OFF;
	PRE_CHARGE_OFF;
	
	//adc_init();
	//uart0_init(9600);
	spi_init(0,0); // 1,0
	MCP2517_init();
	sei(); // Enable interrupts
}

int main(void)
{
    AMU_init();
	
	uint8_t data[8] = {0};
	CAN_RECEIVE_ADDRESS receiveID;
	uint8_t numDataBytes;
	
    while(1) 
    {
		// Fan control pin is BMS alarm
		MCP2517_recieveMessage(&receiveID, &numDataBytes, data);
		if(receiveID == CAN_RECEIVE_ID_PDM >> 18) { // Use PDM CAN packet
			/* Byte 0 */
			if(CHECK_BIT(data[0], 6)) { // Shutdown -
				SHDN_NEG_ON;
			} else {
				SHDN_NEG_OFF;
			}
			if(CHECK_BIT(data[0], 7)) { // Shutdown +
				SHDN_POS_ON;
				LED_A_ON;
			} else {
				SHDN_POS_OFF;
				LED_A_OFF;
			}
			/* Byte 5 */
			if(CHECK_BIT(data[5], 0)) { // Pre-charge
				PRE_CHARGE_ON;
				LED_B_ON;
			} else {
				PRE_CHARGE_OFF;
				LED_B_OFF;
			}
			MCP2517_transmitMessage(CAN_SEND_ID_AMU, 5, AMU_DATA);
		} else {
			//
		}		
		receiveID = 0;		
    }
}


