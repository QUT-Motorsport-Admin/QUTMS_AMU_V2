/*****************************************************************************
* @file    firmware/QUTMS_SteeringWheel_Firmware/input.h
* @author  Zoe Goodward
* @version V1.0.0
* @date    2/08/2019 1:20:22 PM
* @brief   This file defines the outputs and inputs of the pins of the MCU
*****************************************************************************/

#ifndef INPUT_H_
#define INPUT_H_

/*	Pinout

	RX			PD0
	TX			PD1
	
*/

/* CAN */
#define CAN_CS_PORT		PORTA
#define CAN_CS			PINA0

/* LEDS */
#define LED_A_ON		PORTA |= 0b01000000
#define LED_A_OFF		PORTA &=~0b01000000
#define LED_B_ON		PORTA |= 0b00010000
#define LED_B_OFF		PORTA &=~0b00010000
#define LED_A_TOGGLE	PORTA ^= 0b01000000
#define LED_B_TOGGLE	PORTA ^= 0b00010000

/* Contactors and Precharge */
#define SHDN_NEG_ON		PORTD |= 0b00000010
#define SHDN_NEG_OFF	PORTD &=~0b00000010
#define PRE_CHARGE_ON	PORTD |= 0b00000100
#define PRE_CHARGE_OFF	PORTD &=~0b00000100
#define SHDN_POS_ON		PORTD |= 0b00001000
#define SHDN_POS_OFF	PORTD &=~0b00001000

#endif /* INPUT_H_ */