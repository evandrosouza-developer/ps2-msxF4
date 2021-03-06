#include "serial.h"
#include "port_def.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

// See the inspiring file:
// https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f1/stm32-h103/usart_irq_printf/usart_irq_printf.c

#define SERIAL_RING_BUFFER_SIZE_POWER 9 //To use debug messages, increase this for at least 10
#define SERIAL_RING_BUFFER_SIZE (1 << SERIAL_RING_BUFFER_SIZE_POWER)

#define X_ON	17
#define X_OFF 19
#define X_OFF_TRIGGER (3*SERIAL_RING_BUFFER_SIZE/4)
#define X_ON_TRIGGER (SERIAL_RING_BUFFER_SIZE/2)

volatile bool enable_xon_xoff = true, xon_condition = true, xoff_condition = false, xonoff_sendnow = false;

struct ring tx_ring;
struct ring rx_ring;

uint8_t tx_ring_buffer[SERIAL_RING_BUFFER_SIZE];
uint8_t rx_ring_buffer[SERIAL_RING_BUFFER_SIZE];

//Prototypes:
uint16_t ring_tx_put_ch(uint8_t);
int _write(int, char*, int);
int _read(int, char*, int);
void isr_serial(void);


void ring_init(struct ring *ring, uint8_t *buf)
{
	ring->data = buf;
	ring->put_ptr = 0;
	ring->get_ptr = 0;
}


//Ready to be used outside this module.
// Setup serial used in main.
void serial_setup(void)
{
	// Enable clocks for GPIO port A (for GPIO_USART_PORT_TX in main) and USART_PORT.
 	rcc_periph_clock_enable(RCC_GPIOA);

#if USART_PORT == USART1
 	rcc_periph_clock_enable(RCC_USART1);
	// Enable the USART_PORT interrupt.
	nvic_enable_irq(NVIC_USART1_IRQ);
	// Setup GPIO pins for USART1 transmit
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9);
	// Setup GPIO pins for USART1 receive
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, GPIO10);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO10);
#endif

#if USART_PORT == USART2
 	rcc_periph_clock_enable(RCC_USART2);
	// Enable the USART_PORT interrupt.
	nvic_enable_irq(NVIC_USART2_IRQ);
	// Setup GPIO pins for USART2 transmit
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2);
	// Setup GPIO pins for USART2 receive
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, GPIO3);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO3);
#endif

	// Initialize input and output ring buffers.
	ring_init(&tx_ring, tx_ring_buffer);
	ring_init(&rx_ring, rx_ring_buffer);

	// Setup UART parameters.
	usart_set_baudrate(USART_PORT, 115200); // w/o OVER8... (16x oversampling)
	//USART_CR1(USART_PORT) |= USART_CR1_OVER8;	//With OVER8 (8x oversampling)
	//usart_set_baudrate(USART_PORT, 115200/2); // "/2" 'cause OVER8... (8x oversampling)
	usart_set_databits(USART_PORT, 8);
	usart_set_parity(USART_PORT, USART_PARITY_NONE);
	usart_set_stopbits(USART_PORT, USART_STOPBITS_1);
	usart_set_flow_control(USART_PORT, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART_PORT, USART_MODE_TX_RX);
	USART_CR3(USART_PORT) |= USART_CR3_ONEBIT;	//when the line is noise-free to increase the receiver???s tolerance to clock deviations

	// Enable USART_PORT Receive interrupt (only RX, because TX int will be activated only at first transmission).
	//USART_CR1(USART_PORT) |= USART_CR1_RXNEIE;
	usart_enable_rx_interrupt(USART_PORT);

	//Clear RX errors: IDLE (bit 4), ORE (3), Noise detected (2), FRE (1), Parity Error (0)
	// IDLE: Idle line detected: USART_SR_IDLE	(1 << 4)
	// ORE: Overrun error: USART_SR_ORE					(1 << 3)
	// NE: Noise error flag: USART_SR_NE				(1 << 2)
	// FE: Framing error: USART_SR_FE						(1 << 1)
	// PE: Parity error: USART_SR_PE						(1 << 0)
	//A read to the USART_SR register followed by a read to the USART_DR register
	if 	((USART_SR(USART_PORT) && 0B11111) != 0)	//(USART_SR_IDLE | USART_SR_ORE | USART_SR_NE | USART_SR_NE | USART_SR_PE)
	{
		uint32_t reg32 = USART_DR(USART_PORT);
		reg32 &= 0xFFFFFFFF;	//Only to avoid the warning of unused variable 'reg32'
	}

	// Enable the USART.
	usart_enable(USART_PORT);	//USART_CR1(USART_PORT) |= USART_CR1_UE;
	uint32_t reg32 = USART_CR1(USART_PORT);
	reg32 &= 0xFFFFFFFF;	//Only to avoid the warning of unused variable 'reg32'

	// Finally init X_ON/X_OFF flags.
	xon_condition = true;
	xoff_condition = false;
	xonoff_sendnow = false;
}


/*************************************************************************************************/
/****************************Ready to be used from outside of this module*************************/
/*************************************************************************************************/
//Ready to be used from outside of this module.
// It returns the number of available characters in USART_PORT RX ring:
// 0 (or false) to no available chars, or
// > 0 (or true) is the number of available chars
uint16_t serial_available_get_char(void)
{
	uint16_t result = (ring_avail_get_ch(&rx_ring));
	return result;
}


//Ready to be used from outside of this module.
// If there is an available char in serial, it returns with an uint8_t.
//You MUST use the above function "serial_available_get_char" BEFORE this one,
//in order to verify a valid available reading by this function.
uint8_t serial_get_char(void)
{
	return (uint8_t)ring_rx_get_ch();
}


//Ready to be used outside this module.
// Put a char (uint8_t) on serial buffer.
// It returns true if there was room to put this on USART_PORT TX buffer.
uint16_t serial_put_char(uint8_t ch)
{
	/* Put char in tx_ring. */
	uint16_t result = ring_tx_put_ch(ch);
	return result;
}


//Ready to be used outside this module.
// Put an ASCIIZ (uint8_t) string on serial buffer.
// No return  and no blocking function if there is enough space on USART_PORT TX buffer, otherwise,
// wait until buffer is filled.
void serial_send_string(uint8_t *string)
{
	uint16_t iter = 0;
	do
	{	//usart_send_blocking(usart, string[iter++]);
		while (!serial_put_char(string[iter]))
			__asm("nop");
		iter++;	//points to next char on the string
	}	while(string[iter] != 0 && iter < 128);
}


//Ready to be used outside this module.
// Wait until Transmission is finished.
void serial_wait_tx_ends(void)
{
	while ((USART_CR1(USART_PORT) & USART_CR1_TXEIE) != 0)
		__asm("NOP");			//Wait here while there is TX Interrupt enable
	while ((USART_SR(USART_PORT) & USART_SR_TC) == 0)
		__asm("NOP");			//Then wait here until the tx of the last char is completed
}


/*************************************************************************************************/
/*************************************Internal serial routines************************************/
/*************************************************************************************************/
//Used as an internal function.
//It is used to put a char in the ring buffer, both TX and RX.
//It returns number of chars are in the buffer, or 0 when there was no room to put this char.
uint16_t ring_put_ch(struct ring *ring, uint8_t ch)
{
	uint16_t i, i_next;
	i = ring->put_ptr;				//i is the original position
	//i_next is the next position of ring->put_ptr
	i_next = (i + 1) & (uint16_t)(SERIAL_RING_BUFFER_SIZE - 1); //if(i_next >= (uint16_t)SERIAL_RING_BUFFER_SIZE) i_next = 0;
	if(i_next != ring->get_ptr)
	{
		ring->data[i] = ch;			//saves in the put_ptr position 
		ring->put_ptr = i_next;	//now put_ptr points to the next position of i
		//Optimizing calculations inside the interrupt => The general formula is:
		//CharsInBuffer = (RING_BUFFER_SIZE - ring.get_ptr + ring.put_ptr) % RING_BUFFER_SIZE;
		//but SERIAL_RING_BUFFER_SIZE is a power of two, so the rest of the division is computed zeroing
		//the higher bits of the summed buffer lenght, so in the case of 256 (2**8), you have to keep only
		//the lowest 8 bits, or (SERIAL_RING_BUFFER_SIZE - 1).
		return (uint16_t)(SERIAL_RING_BUFFER_SIZE - ring->get_ptr + i_next) & (uint16_t)(SERIAL_RING_BUFFER_SIZE - 1);
	}
	else
	{
		//No room for more (Buffer full)
		return 0;
	}
}


//Used as an internal function.
//It is used to put a char in the ring TX buffer, as it will initiate TX if the first one is put on buffer.
//It returns number of chars are in the buffer of 0 when there was no room to add this char.
//Here XONOFF is processed
uint16_t ring_tx_put_ch(uint8_t ch)
{
	uint16_t qty_in_buffer;

	qty_in_buffer = ring_put_ch(&tx_ring, ch);
	if (qty_in_buffer == 1)
	{
		// Enable Transmit request for the first one
		usart_enable_tx_interrupt(USART_PORT);
	}

	return qty_in_buffer;
}


//Used as an internal function.
//Returns the number of chars available in the ring (both TX and RX) or 0 if none.
uint16_t ring_avail_get_ch(struct ring *ring)
{
	uint16_t output;
	output = (SERIAL_RING_BUFFER_SIZE - ring->get_ptr + ring->put_ptr) & (SERIAL_RING_BUFFER_SIZE - 1);
	return output;
}


//Used as an internal function.
//It returns char when it is available or 0xFFFF when no one is available
//Used on both TX and RX buffers.
uint16_t ring_get_ch(struct ring *ring, uint16_t *qty_in_buffer)
{
	uint16_t i = ring->get_ptr;
	if(i == ring->put_ptr)
		//No char in buffer
		return 0xFFFF;
	int16_t result = (int16_t)ring->data[i];
	*qty_in_buffer = (SERIAL_RING_BUFFER_SIZE - i + ring->put_ptr) & (SERIAL_RING_BUFFER_SIZE - 1);
	i++;
	ring->get_ptr = i & (uint16_t)(SERIAL_RING_BUFFER_SIZE - 1); //if(i >= (uint16_t)SERIAL_RING_BUFFER_SIZE)	i = 0;
	return result;
}


//Used as an internal function.
//Implemented X_ON/X_OFF protocol
//It returns char when it is available or -1 when no one is available
//Used only on RX buffer.
uint8_t ring_rx_get_ch(void)
{
	uint8_t ch;
	uint16_t qty_in_buffer;

	ch = ring_get_ch(&rx_ring, &qty_in_buffer);
	return ch;
}


/*************************************************************************************************/
/*************************************General Conversions*****************************************/
/*************************************************************************************************/
//Ready to be used outside this module.
// Convert a two byte string pointed by i into a binary byte. 
// It returns and no blocking function if there is enough space on USART_PORT TX buffer, otherwise,
uint8_t conv_2a_hex_to_uint8(uint8_t *instring, int16_t i)
{
	uint8_t binuint8, ch;

	ch = instring[i];// & 0x5F; //to capital letter
	binuint8 = (ch > ('A'-1) && ch < ('F'+1)) ?
						 (ch-('A'-10)) : (ch-'0'); //55 = "A"0x41-10; 48="0"0x30
	binuint8 <<= 4;
	ch = instring[i+1];// & 0x5F; //to capital letter
	binuint8 += (ch > ('A'-1) && ch < ('F'+1)) ?
							(ch-('A'-10)) : (ch-'0'); //55 = "A"0x41-10; 48="0"0x30
	return binuint8;
}


//Ready to be used outside this module.
// Convert a word (32 bit binary) to into a 8 char string. 
void conv_uint32_to_8a_hex(uint32_t value, uint8_t *outstring)
{
	uint8_t iter;

	/*end of string*/
	outstring += 8;
	*(outstring--) = 0;

	for(iter=0; iter<8; iter++)
	{
		*(outstring--) = (((value&0xf) > 0x9) ? (0x40 + ((value&0xf) - 0x9)) : (0x30 | (value&0xf)));
		value >>= 4;
	}
}


//Ready to be used outside this module.
// Convert a half-word (16 bit binary) to into a 4 char string. 
void conv_uint16_to_4a_hex(uint16_t value, uint8_t *outstring)
{
	uint8_t iter;

	/*end of string*/
	outstring += 4;
	*(outstring--) = 0;

	for(iter=0; iter<4; iter++)
	{
		*(outstring--) = (((value&0xf) > 0x9) ? (0x40 + ((value&0xf) - 0x9)) : (0x30 | (value&0xf)));
		value >>= 4;
	}
}


//Ready to be used outside this module.
// Convert a byte (8 bit binary) to into a 2 char string. 
void conv_uint8_to_2a_hex(uint8_t value, uint8_t *outstring)
{
	uint8_t iter;

	/*end of string*/
	outstring += 2;
	*(outstring--) = 0;

	for(iter=0; iter<2; iter++)
	{
		*(outstring--) = (((value&0xf) > 0x9) ? (0x40 + ((value&0xf) - 0x9)) : (0x30 | (value&0xf)));
		value >>= 4;
	}
}


//Ready to be used outside this module.
// Convert a word (uint32_t unsigned 32 bit binary) to into a (up to) 10 char string, with no leading zeroes.
void conv_uint32_to_dec(uint32_t value, uint8_t *outstring)
{
	uint8_t iter, pos;
	const uint32_t power10[10] = {1000000000, 100000000, 10000000, 1000000, 100000, 10000, 1000, 100, 10, 1};

	bool insertzero = false;
	for(iter=0; iter<=10; iter++)	//Fill outstring with \0
		(outstring[iter]) = 0;

	if(value == 0)
		outstring[0] = '0';
	else
	{
		pos = 0;
		for(iter=0; iter<10; iter++)
		{
			if ((value >= power10[iter]) || insertzero)
			{
				outstring[pos] = '0';
				while (value >= power10[iter])
				{
					value -= power10[iter];
					outstring[pos] += 1;
					insertzero = true;
				}
			}	//if (value > power10[iter])
			if (insertzero)
				pos++;
		}	//for(iter=0; iter<5; iter++)
	}
}



/*************************************************************************************************/
/*************************************** stdio ***************************************************/
/*************************************************************************************************/
//Used as an internal function.
// It is a low level output called from printf, by libc stdio fwrite functions.
int _write(int file, char *ptr, int len)
{
	// If the target file isn't stdout/stderr, then return an error
	// since we don't _actually_ support file handles
	if (file != STDOUT_FILENO && file != STDERR_FILENO) {
		// Set the errno code (requires errno.h)
		errno = EIO;
		return -1;
	}
	int i;
	for (i = 0; i < len; i++)
	{
		// If we get a newline character, also be sure to send the carriage
		// return character first, otherwise the serial console may not
		// actually return to the left.
		if (ptr[i] == '\n') 
			while (ring_tx_put_ch('\r') == 0xFFFF)  //usart_send_blocking(USART_PORT, '\r');
				__asm("nop");
		// Write the character to send to the USART transmit buffer, and block
		// until it has been sent.
		while (ring_tx_put_ch(ptr[i]) == 0xFFFF)	//usart_send_blocking(USART_PORT, ptr[i]);
	__asm("nop");
	}
	// Return the number of bytes we sent
	return i;
}

/*
// Called by the libc stdio fread fucntions
int _read(int file, char *ptr, int len)
{
	int qtyy_in_line = 0;

	if (file > 2)
		return -1;

	get_buffered_line(&qtyy_in_line, *ptr, int len);

	return qtyy_in_line; // return the length we got
}


void get_buffered_line(int *qtyy_in_buf, char *buf[buflen], int buflen) 
{
	char	ch;
	int qtty = 0;

	if (*qtyy_in_buf)
		return;

	for(;;)
	{
		//ch = usart_recv_blocking(USART2);
		if(!serial_available_get_char())
			__asm("nop");
		ch = serial_get_char();
		if (ch == '\r')
		{
			*buf[*qtyy_in_buf] = '\n';
			(*qtyy_in_buf)++;
			*buf[*qtyy_in_buf] = '\0';
			//usart_send_blocking(USART2, '\r');
			while (ring_tx_put_ch('\r') == 0xFFFF)
				__asm("nop");
			//usart_send_blocking(USART2, '\n');
			while (ring_tx_put_ch('\n') == 0xFFFF)
				__asm("nop");
			return;
		}
		if ((ch == '\010') || (ch == '\177'))
		{
			// ^H or DEL erase a character
			if ((*qtyy_in_buf) == 0)
			{
				//usart_send_blocking(USART2, '\a');
				while (ring_tx_put_ch('\a') == 0xFFFF)
					__asm("nop");
			}
			else
				back_up();
		}
		else if (ch == 0x17)
		{
		// ^W erases a word
			while (((*qtyy_in_buf) > 0) && (!(isspace((int) *buf[*qtyy_in_buf]))))
				back_up();
		}
		else if (ch == 0x15)
		{
			// ^U erases the line
			while ((*qtyy_in_buf) > 0)
			{
				back_up();
			}
		}
		else
		{
			// Non-editing character so insert it
			if (*qtyy_in_buf) < (buflen - 1)) 
			{
				*buf[*qtyy_in_buf] = ch;
				(*qtyy_in_buf)++;
				//usart_send_blocking(USART2, c);
				while (ring_tx_put_ch(ch) == 0xFFFF)
					__asm("nop");
			} 
			else
			{
				//Buffer is already full
				//usart_send_blocking(USART2, '\a');
				while (ring_tx_put_ch('\a') == 0xFFFF)
					__asm("nop");
			}
		}
	}
}

// back up the cursor one space
static inline void back_up(void)
{
	(*qtyy_in_buf)--;
	//usart_send_blocking(USART2, '\010');
	while (ring_tx_put_ch('\010') == 0xFFFF)
		__asm("nop");
	//usart_send_blocking(USART2, ' ');
	while (ring_tx_put_ch(' ') == 0xFFFF)
		__asm("nop");
	//usart_send_blocking(USART2, '\010');
	while (ring_tx_put_ch('\010') == 0xFFFF)
		__asm("nop");
}
*/

/*************************************************************************************************/
/*************************************************************************************************/
/******************************************* ISR's ***********************************************/
/*************************************************************************************************/
/*************************************************************************************************/
//XON/XOFF flux control protocol is implemented on RX
void usart1_isr(void)
{
	isr_serial();
}


void usart2_isr(void)
{
	isr_serial();
}


void isr_serial(void)
{
	uint16_t qty_in_buffer;
	//Clear RX errors: IDLE (bit 4), ORE (3), Noise detected (2), FRE (1), Parity Error (0)
	//A read to the USART_SR register followed by a read to the USART_DR register
	if ((USART_SR(USART_PORT) && 0B11111) != 0)
	{
		__asm("nop");
	}

	// Check if it were called because of RXNE.
	if (((USART_CR1(USART_PORT) & USART_CR1_RXNEIE) != 0) && ((USART_SR(USART_PORT) & USART_SR_RXNE) != 0))
	{
		// Retrieve the data from the peripheral and put in rx_ring.
		qty_in_buffer = ring_put_ch(&rx_ring, (uint8_t)usart_recv(USART_PORT));
		if (enable_xon_xoff)
		{
			if (qty_in_buffer >= (uint16_t)X_OFF_TRIGGER)
			{
				xon_condition = false;
				if (!xoff_condition)										//To send X_OFF only once
				{
					xoff_condition = true;
					// Enable Transmit request
					xonoff_sendnow = true;
					usart_enable_tx_interrupt(USART_PORT);//Force Enable transmission request
				}
			}
			//else if (qty_in_buffer <= (uint16_t)X_ON_TRIGGER)
			else if (qty_in_buffer <= X_ON_TRIGGER)
			{
				xoff_condition = false;
				if (!xon_condition)											//To send X_ON only once
				{
					xon_condition = true;
					// Enable Transmit request
					xonoff_sendnow = true;
					usart_enable_tx_interrupt(USART_PORT);//Force Enable transmission request
				}  //if (!xon_condition)
			} //else if (*qty_in_buffer <= (uint16_t)X_ON_TRIGGER)
		} //if (enable_xon_xoff)
	}

	// Check if it was called because of TXE.
	if(((USART_CR1(USART_PORT) & USART_CR1_TXEIE)!=0) && ((USART_SR(USART_PORT) & USART_SR_TXE)!=0))
	{
		uint16_t data = 0;

		if (xonoff_sendnow)
		{
			if(xoff_condition)
				data = X_OFF;
			if(xon_condition)
				data = X_ON;
			xonoff_sendnow = false;
		}
		else
			data = ring_get_ch(&tx_ring, &qty_in_buffer);

		if (data == 0xFFFF) 
		{ // Disable the TXE interrupt: It's no longer needed.
			USART_CR1(USART_PORT) &= ~USART_CR1_TXEIE;
		}
		else 
		{ //  Put data into the transmit register.
			usart_send(USART_PORT, (uint16_t) data);
		}
	}
}
