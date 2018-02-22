/************************************************************************/
// File:			usart.c
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Driver USART. Protected printf with mutex.
/************************************************************************/
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include "usart.h"
#include "LED.h"
/* Baud rate set in defines */
#include "defines.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "buffer.h"

SemaphoreHandle_t xUartMutex;
void(*receive_callback)(uint8_t*, uint16_t);
void vFrameReaderTask( void *pvParameters );
void vUartSendTask(void *pvParamters);

TaskHandle_t frame_receiver = NULL;
buffer_t send_buffer;
uint8_t receive_buffer[100];
/* Set up baud prescale according to datasheet table 17-1 page 174 */
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16))) - 1)

/************************************************************************/
//Initialize USART driver, note that RXD0/TXD0 (PD0/PD1) is used
// Note that the nRF51 dongle is limited to send 20 characters
// in each package
/************************************************************************/
void vUSART_init(){
    /* Set baud rate, has to match nRF51 dongle! */
    UBRR2H = (unsigned char)(BAUD_PRESCALE>>8);
    UBRR2L = (unsigned char)BAUD_PRESCALE;
    
    /* RX/TX Complete, data register empty */
    UCSR2A = (1<<RXC2) | (1<<TXC2) | (1<<UDRE2);

    /* Enable reciever, transmitter, and recieve interrupt enable*/
    UCSR2B = (1<<RXEN2) | (1<<TXEN2) | (1<<RXCIE2);

    /* Set frame format: 8data, 1 stop bit, no parity */
    UCSR2C = (1<<UCSZ20) | (1<<UCSZ21);
    UCSR2C &= ~((1<<USBS2) & (1<<UPM21) & (1<<UPM20));

	uint8_t *buf = pvPortMalloc(100);
	buffer_init(&send_buffer, buf, 100);
	xTaskCreate(vFrameReaderTask, "FrameReader", 300, NULL, 4, NULL);
	xTaskCreate(vUartSendTask, "UartSendTask", 300, NULL, 4, NULL);
	
	xUartMutex = xSemaphoreCreateMutex();
}

void vUSART_send(uint8_t *data, uint16_t len) {
	xSemaphoreTake(xUartMutex, portMAX_DELAY);
	buffer_append(&send_buffer, data, len);
	xSemaphoreGive(xUartMutex);
}

void vUSART_set_receive_callback(void(*cb)(uint8_t*, uint16_t)) {
	receive_callback = cb;
}

void vFrameReaderTask( void *pvParameters ){
	frame_receiver = xTaskGetCurrentTaskHandle();
	uint32_t notification_value = 0;
	while(1){
		xTaskNotifyWait(0xFFFFFFFF, 0xFFFFFFFF, &notification_value, portMAX_DELAY);
		if(receive_callback != NULL) receive_callback(receive_buffer, (uint16_t)notification_value);
	}
}

void vUartSendTask(void *pvParamters) {
	uint8_t data[100];
	uint16_t num;
	uint16_t i;
	while(1){
		xSemaphoreTake(xUartMutex, portMAX_DELAY);
		num = buffer_remove_token(&send_buffer, data, 0x00, 100);
		xSemaphoreGive(xUartMutex);
		if(num>0) {
			for(i=0;i<num;i++) {
				while ( !( UCSR2A & (1<<UDRE2)) );
				UDR2 = data[i];
			}
		}
		vTaskDelay(10*portTICK_PERIOD_MS);
	}
}


ISR(USART2_RX_vect){
	static uint8_t input_buffer[100];
	static uint16_t input_index = 0;
	input_buffer[input_index++] = UDR2;
	if(input_buffer[input_index-1] == 0x00) {
		if(frame_receiver != NULL) {
			memcpy(receive_buffer, input_buffer, input_index);
			xTaskNotifyFromISR(frame_receiver, input_index, eSetValueWithoutOverwrite, NULL);
		}
		input_index = 0;
	}
	if(input_index > 100) input_index = 0; // Something went wrong, received too many bytes
}