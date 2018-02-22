#ifndef _NETWORK_H_
#define _NETWORK_H_

#include <stdint.h>

#define PROTOCOL_SIMPLE     0
#define PROTOCOL_ARQ        1

#define MAX_FRAME_SIZE      50 //Must be at least 8 to accomodate the six overhead bytes (receiver, sender, protocool, crc, 0x00 delimiter and one COBS overhead), plus at least two data bytes, must also be changed in the server dongle 'protocol.c' file, and in the server application
#define MAX_PAYLOAD_SIZE    MAX_FRAME_SIZE - 6 //

// Frame: [ RECEIVER | SENDER | PROTOCOL |        DATA           | CRC | 0x00 ]

void network_init(void);
void network_set_callback(uint8_t protocol, void (*cb)(uint8_t, uint8_t*, uint16_t));
uint8_t network_send(uint8_t remote_address, uint8_t protocol, uint8_t *data, uint16_t len);
uint8_t network_get_address(void);

#endif