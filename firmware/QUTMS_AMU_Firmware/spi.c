/*****************************************************************************
* @file    firmware/QUTMS_HVBoard_Firmware/spi.c
* @author  Zoe Goodward
* @version V1.0.0
* @date    2/08/2019 1:25:34 PM
* @brief   This file...
*****************************************************************************/

/* INCLUDES */
#include <inttypes.h>
#include <stddef.h>
#include <avr/io.h>
#include "spi.h"
#include "input.h"
#include "UART.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  SPI Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void spi_init(uint8_t clkRate0, uint8_t clkRate1)
{
	SPCR = (SPI_INTERRUPT<<SPIE)
	|(1<<SPE)
	|(SPI_DATA_ORDER<<DORD)
	|(SPI_MSTR_MODE<<MSTR)
	|(SPI_CLK_POLARITY<<CPOL)
	|(SPI_CLK_PHASE<<CPHA)
	|(clkRate0<<SPR0)
	|(clkRate1<<SPR1);
	//set interrupt, enable SPI, MSB mode, set as master, low is idle, read on leading edge, Set to speed(fosc/4) SPR0=1, SPR1=0
	SPSR = (DBL_CLK<<SPI2X);	//set whether we want 2x speed or not (1=2x speed).
	SPDR = 0x00;		//ensure data register has nothing in it
}

//shifts out 8 bits of data
//  uint8_t data - the data to be shifted out
//  returns uint8_t - the data received during sending
// FULLY WORKING
uint8_t spi_send_byte(uint8_t data) {
	/* Start transmission */ // Transmission is started by writing data to the transmission register
	SPDR = data;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));
	/* Return any data that was shifted into this register upon transmission  */
	return SPDR;
}

// FULLY WORKING
void spi_transfer_buffer(uint8_t *buf, uint8_t count)
{
	if (count == 0) return;
	for(uint8_t i = 0; i < count; i++) {
		uint8_t out = *(buf + i);
		SPDR = out;
		while(!(SPSR & (1<<SPIF)));
		*(buf + i) = SPDR;
	}
}



void spi_disable(void) {
	SPDR = 0;
}