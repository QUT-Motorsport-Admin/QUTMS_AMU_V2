/*****************************************************************************
* @file    firmware/QUTMS_HVBoard_Firmware/spi.h
* @author  Zoe Goodward
* @version V1.0.0
* @date    2/08/2019 1:27:31 PM
* @brief   This file declares the variables and functions that are used for
*          SPI
*****************************************************************************/

#ifndef SPI_H_
#define SPI_H_

// Aliases for the different SPI chip pins used in the car
#if (defined(__AVR_ATmega1284P__))
 #define SPI_SCK_PIN	7 //PB7
 #define SPI_MOSI_PIN	5 //PB5
 #define SPI_MISO_PIN   6 //PB6
#elif (defined((__AVR_ATmega1280__))
 #define SPI_SCK_PIN	1 //PB1
 #define SPI_MOSI_PIN	2 //PB2
 #define SPI_MISO_PIN	3 //PB3
#elif (defined((__AVR_ATmega64M1__))
 #define SPI_SCK_PIN	7 //PB7
 #define SPI_MOSI_PIN	1 //PB1
 #define SPI_MISO_PIN	0 //PB0
#else
 #error Unknown processor
#endif

//64mM1, other SPI?

//////////////////////////////
/*		  CONFIGURE			*/

// SPI CLock Modes
/* SPI MODE 0:	CPOL = 0, CPHA = 0; Leading Edge: Sample (Rising), Trailing Edge: Setup (Falling) */
/* SPI MODE 1:	CPOL = 0, CPHA = 1; Leading Edge: Setup (Rising), Trailing Edge: Sample (Falling) */
/* SPI MODE 2:	CPOL = 1, CPHA = 0; Leading Edge: Sample (Falling), Trailing Edge: Setup (Rising) */
/* SPI MODE 3:	CPOL = 1, CPHA = 1; Leading Edge: Setup (Falling), Trailing Edge: Sample (Rising) */
#define SPI_MODE 0

// Clock Rate
/* SPI CLOCK RATE 0:	SCK Frequency: Fosc/4 */
/* SPI CLOCK RATE 1:	SCK Frequency: Fosc/16 */
/* SPI CLOCK RATE 2:	SCK Frequency: Fosc/64 */
/* SPI CLOCK RATE 3:	SCK Frequency: Fosc/128 */
/* SPI CLOCK RATE 4:	SCK Frequency: Fosc/2 */
/* SPI CLOCK RATE 5:	SCK Frequency: Fosc/8 */
/* SPI CLOCK RATE 6:	SCK Frequency: Fosc/32 */
/* SPI CLOCK RATE 7:	SCK Frequency: Fosc/64 */
#define SPI_CLK_RATE 0

#define SPI_MSTR 1

#define SPI_MSBFIRST 0

#define SPI_INT_ENABLE 0

/*		END OF CONFIG		*/
//////////////////////////////


#ifdef SPI_INT_ENABLE
 #if SPI_INT_ENABLE == 0
  #define SPI_INTERRUPT	0
 #elif SPI_INT_ENABLE == 1
  #define SPI_INTERRUPT	1
 #endif
#else 
 #error SPI: SPIE in SPCR not set
#endif

#ifdef SPI_MSBFIRST
 #if SPI_MSBFIRST == 0
  #define SPI_DATA_ORDER 0
 #elif SPI_MSBFIRST == 1
  #define SPI_DATA_ORDER 1
 #endif
#else
 #error SPI: DORD in SPCR not set
#endif


#ifdef SPI_MSTR
 #if SPI_MSTR == 0
  #define SPI_MSTR_MODE 0
 #elif SPI_MSTR == 1
  #define SPI_MSTR_MODE 1
 #endif
#else
 #error SPI: MSTR in SPCR not set
#endif

#ifdef SPI_MODE
 #if SPI_MODE == 0
  #define SPI_CLK_POLARITY	0
  #define SPI_CLK_PHASE		0
 #elif SPI_MODE == 1
  #define SPI_CLK_POLARITY	0
  #define SPI_CLK_PHASE		1
 #elif SPI_MODE == 2
  #define SPI_CLK_POLARITY	1
  #define SPI_CLK_PHASE		0
 #elif SPI_MODE == 3
  #define SPI_CLK_POLARITY	1
  #define SPI_CLK_PHASE		1
 #endif
#else
 #error SPI: Data mode (CPOL & CPHA) in SPCR not set
#endif

#ifdef SPI_CLK_RATE
 #if SPI_CLK_RATE == 0
  #define SPI_CLK_RATE0 0
  #define SPI_CLK_RATE1 0
  #define DBL_CLK		0
 #elif SPI_CLK_RATE == 1
  #define SPI_CLK_RATE0 1
  #define SPI_CLK_RATE1 0
  #define DBL_CLK		0
 #elif SPI_CLK_RATE == 2
  #define SPI_CLK_RATE0 0
  #define SPI_CLK_RATE1 1
  #define DBL_CLK		0
 #elif SPI_CLK_RATE == 3
  #define SPI_CLK_RATE0 1
  #define SPI_CLK_RATE1 1
  #define DBL_CLK		0
 #elif SPI_CLK_RATE == 4
  #define SPI_CLK_RATE0 0
  #define SPI_CLK_RATE1 0
  #define DBL_CLK		1
 #elif SPI_CLK_RATE == 5
  #define SPI_CLK_RATE0 1
  #define SPI_CLK_RATE1 0
  #define DBL_CLK		1
 #elif SPI_CLK_RATE == 6
  #define SPI_CLK_RATE0 0
  #define SPI_CLK_RATE1 1
  #define DBL_CLK		1
 #elif SPI_CLK_RATE == 7
  #define SPI_CLK_RATE0 1
  #define SPI_CLK_RATE1 1
  #define DBL_CLK		1
 #endif
#else
 #error SPI: Clock rate (SPR0 & SPR1) in SPCR not set
#endif

// Functions
void spi_init(uint8_t clkRate0, uint8_t clkRate1);
//void spi_init(void);
uint8_t spi_send_byte(uint8_t data);
void spi_transfer_buffer(uint8_t *buf, uint8_t count);
uint16_t spi_transfer_16(uint16_t data);

#endif /* SPI_H_ */