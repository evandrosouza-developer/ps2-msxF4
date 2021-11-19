#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

struct ring
{
	uint8_t *data;
	uint16_t put_ptr;
	uint16_t get_ptr;
};


//Setup ring
void ring_init(struct ring *ring, uint8_t *buf);

//Setup serial subsystem
void serial_setup(void);

// Put a char (uint8_t) on serial buffer.
// It returns true if there was room to put this on USART TX buffer.
// It is a non blocking function
uint16_t ring_put_ch(struct ring *ring, uint8_t ch);

// Put a char (uint8_t) on serial buffer.
// It returns the number of chars, or 0 (zero) if there was no room to put this on USART TX buffer.
// It is a non blocking function
uint16_t serial_put_char(uint8_t);

// It returns the number of available chars in USART RX ring (0 or false if none).
// It is a non blocking function
uint16_t serial_available_get_char(void);

// If there is an available char in serial, it returns with an uint8_t.
// It is a non blocking function
uint8_t serial_get_char(void);

// Send a ASCIIZ string to serial (up to 127 chars).
// It is a non blocking function if there is room on TX Buffer
void usart_send_string(uint8_t*);

//Wait until the transmission is concluded
void wait_tx_ends(void);

/*Functions to convert strings*/
// Convert a two byte string pointed by i into a binary byte. 
uint8_t conv_2a_hex_to_uint8(uint8_t*, int16_t);

// Convert a word (32 bit binary) to into a 8 char string. 
void conv_uint32_to_8a_hex(uint32_t, uint8_t*);

// Convert a half-word (16 bit binary) to into a 4 char string. 
void conv_uint16_to_4a_hex(uint16_t, uint8_t*);

// Convert a byte (8 bit binary) to into a 2 char string. 
void conv_uint8_to_2a_hex(uint8_t, uint8_t*);

// Convert a word (32 bit binary) to into a 10 char string. 
void conv_uint32_to_dec(uint32_t, uint8_t*);

#ifdef __cplusplus
}
#endif
