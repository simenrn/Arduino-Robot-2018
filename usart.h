/************************************************************************/
// File:			usart.h
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Driver USART. Protected printf with mutex.
/************************************************************************/

#ifndef USART_H_
#define USART_H_

/************************************************************************/
//Initialize USART driver, note that RXD0/TXD0 (PD0/PD1) is used
// Note that the nRF51 dongle is limited to send 20 characters
// in each package
/************************************************************************/
void vUSART_init();

void vUSART_send(uint8_t *data, uint16_t len);
void vUSART_set_receive_callback(void(*cb)(uint8_t*, uint16_t));

#endif /* USART_H_ */

