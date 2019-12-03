#ifndef UART_H_
#define UART_H_

void uart0_init(unsigned int baudRate);
void uart0_transmit(uint8_t data);
uint8_t uart0_receive(void);
void uart0_flush(void);


#endif /* UART_H_ */