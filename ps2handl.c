#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>

#include "serial.h"
#include "ps2handl.h"
#include "hr_timer.h"
#include "port_def.h"
//Use Tab width=2


//States definitions of PS/2 clock interrupt machine 
#define	PS2INT_RECEIVE										0x400
#define	PS2INT_SEND_COMMAND								0x401
#define	PS2INT_WAIT_FOR_COMMAND_ACK				0x402
#define	PS2INT_SEND_ARGUMENT							0x403
#define	PS2INT_WAIT_FOR_ARGUMENT_ACK			0x404

//PS/2 keyboard iteration constants
#define COMM_TYPE3_NO_REPEAT							0xF8	//248 Type 3 command
#define COMM_READ_ID											0xF2	//242
#define COMM_SET_TYPEMATIC_RATEDELAY			0xF3	//243 Type 2 command
#define COMM_SET_RESET_LEDS								0xED	//237
#define COMM_ECHO													0xEE	//238
#define ARG_NO_ARG												0xFF	//255
#define ARG_LOWRATE_LOWDELAY							0x7F	//Type 2 (Delay 1 second to repeat. 2cps repeat rate)
#define KB_ACKNOWLEDGE										0xFA
#define KB_FIRST_ID												0xAB
#define KB_SECOND_ID											0x83
#define KB_SUCCESSFULL_BAT								0xAA
#define KB_ERROR_BAT											0xFC


//Global Vars
extern bool update_ps2_leds;											//Declared on msxmap.cpp
volatile uint16_t ps2int_state;
volatile uint8_t ps2int_TX_bit_idx;
volatile uint8_t ps2int_RX_bit_idx;
extern uint16_t state_overflow_tim2;							//Declared on hr_timer_delay.c
extern uint64_t TIM2_Update_Cnt;									//Declared on hr_timer_delay.c Overflow of time_between_ps2clk

volatile uint8_t command, argument;

volatile uint32_t prev_systicks;
extern uint32_t systicks;													//Declared on msxhid.cpp
extern uint64_t acctimeps2data0;									//Declared on hr_timer_delay.c
extern uint8_t scancode[4];												//declared on msxmap.cpp
extern uint64_t time_between_ps2clk;							//Declared on hr_timer_delay.c
extern uint16_t fail_count;												//declared on msxhid.cpp
volatile bool formerps2datapin;
volatile bool ps2_keyb_detected, ps2numlockstate;
volatile bool command_ok;

volatile bool mount_scancode_OK;									 	//used on mount_scancode()
volatile bool ps2_keystr_e0 = false;
volatile bool ps2_keystr_e1 = false;
volatile bool ps2_keystr_f0 = false;
volatile uint8_t ps2_byte_received;
volatile uint8_t mount_scancode_count_status = 0;

//Need to stay as global to avoid creating different instancies
volatile uint8_t ps2_recv_buffer[PS2_RECV_BUFFER_SIZE];
volatile uint8_t ps2_recv_buff_put;
volatile uint8_t ps2_recv_buff_get;

//Prototypes not declared in ps2handl.h
void init_ps2_recv_buffer(void);
bool available_ps2_byte(void);
uint8_t get_ps2_byte(volatile uint8_t*);
void send_start_bit_next(uint16_t);
void ps2_clock_send(bool);
void ps2_clock_receive(bool);
void ps2_send_command(uint8_t, uint8_t);
void reset_mount_scancode_machine(void);


//Power on PS/2 Keyboard and related pins setup
void power_on_ps2_keyboard()
{
	//Set Alternate function
	gpio_set_af(PS2_CLOCK_PIN_PORT, GPIO_AF1, PS2_POWER_CTR_PIN | PS2_CLOCK_PIN_ID);
	gpio_set_af(PS2_DATA_PIN_PORT, GPIO_AF1, PS2_DATA_PIN_ID);
	//Power pin control
	gpio_mode_setup(PS2_POWER_CTR_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PS2_POWER_CTR_PIN);
	gpio_set_output_options(PS2_POWER_CTR_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, PS2_POWER_CTR_PIN);

	gpio_set(PS2_POWER_CTR_PORT, PS2_POWER_CTR_PIN);

	// PS/2 keyboard Clock and Data pins
	gpio_set(PS2_CLOCK_PIN_PORT, PS2_CLOCK_PIN_ID); //Hi-Z
	gpio_mode_setup(PS2_CLOCK_PIN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, PS2_CLOCK_PIN_ID);
	gpio_set_output_options(PS2_CLOCK_PIN_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, PS2_CLOCK_PIN_ID);
	exti_select_source(PS2_CLOCK_PIN_EXTI, PS2_CLOCK_PIN_PORT);
	exti_set_trigger(PS2_CLOCK_PIN_EXTI, EXTI_TRIGGER_FALLING);
	exti_reset_request(PS2_CLOCK_PIN_EXTI);
	exti_enable_request(PS2_CLOCK_PIN_EXTI);
	gpio_port_config_lock(PS2_CLOCK_PIN_PORT, PS2_CLOCK_PIN_ID);

	// PS/2 keyboard Data pin
		gpio_set(PS2_DATA_PIN_PORT, PS2_DATA_PIN_ID);		//Hi-Z
	gpio_mode_setup(PS2_DATA_PIN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, PS2_DATA_PIN_ID);
	gpio_set_output_options(PS2_DATA_PIN_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, PS2_DATA_PIN_ID);
	gpio_port_config_lock(PS2_DATA_PIN_PORT, PS2_DATA_PIN_ID);

	// Enable EXTI9_5 interrupt.
	nvic_enable_irq(NVIC_EXTI15_10_IRQ);

	//High priority to avoid PS/2 interrupt loss
	nvic_set_priority(NVIC_EXTI15_10_IRQ, 50);

	//Starts with RX state: PS2INT_RECEIVE
	ps2int_state = PS2INT_RECEIVE;
	ps2int_RX_bit_idx = 0;
	
	init_ps2_recv_buffer();
}


void power_off_ps2_keyboard()
{
	gpio_clear(PS2_POWER_CTR_PORT, PS2_POWER_CTR_PIN);
	usart_send_string((uint8_t*)"\r\nPS/2 interface powered down.\r\n\n");
}


// Initialize receive ringbuffer
void init_ps2_recv_buffer()
{
	ps2_recv_buff_put=0;
	ps2_recv_buff_get=0;
	for(uint8_t i=0; i<PS2_RECV_BUFFER_SIZE; ++i)
	{
		ps2_recv_buffer[i]=0;
	}
}

// Verify if there is an available ps2_byte_received on the receive ring buffer, but does not fetch this one
bool available_ps2_byte()
{
	uint8_t i = ps2_recv_buff_get;
	if(i == ps2_recv_buff_put)
		//No char in buffer
		return false;
	else
		return true;
}

// Fetches the next ps2_byte_received from the receive ring buffer
uint8_t get_ps2_byte(volatile uint8_t *buff)
{
	uint8_t i, result;

	i = ps2_recv_buff_get;
	if(i == ps2_recv_buff_put)
		//No char in buffer
		return 0;
	result = buff[i];
	i++;
	ps2_recv_buff_get = i & (uint16_t)(PS2_RECV_BUFFER_SIZE - 1); //if(i >= (uint16_t)SERIAL_RING_BUFFER_SIZE)	i = 0;
	return result;
}


bool ps2_keyb_detect(void)
{
	uint32_t systicks_start_command;	//Initial time mark
	uint8_t mountstring[16];					//Used in usart_send_string()
	
	//Wait for 2.5s to keyboard execute its own power on and BAT (Basic Assurance Test) procedure
	systicks_start_command = systicks;
	ps2_keyb_detected = false;
	while ( ((systicks-systicks_start_command) < (25*3)) && (!available_ps2_byte()) )	//Wait 2500ms for keyboard power on
		prev_systicks = systicks;	//To avoid errors on keyboard power up BEFORE the first access
	if ((systicks-systicks_start_command) >= (25*3))
	{
		//User messages
		usart_send_string((uint8_t*)"..  Timeout on BAT: No keyboard!\r\n");
		return ps2_keyb_detected;
	}
	//PS/2 keyboard might already sent its BAT result. Check it:
	ps2_byte_received = get_ps2_byte(&ps2_recv_buffer[0]);
	if(ps2_byte_received != 0)
	{
		if(ps2_byte_received == KB_SUCCESSFULL_BAT)
		{
			//User messages
			usart_send_string((uint8_t*)"..  BAT (Basic Assurance Test) OK in ");
			conv_uint32_to_dec((prev_systicks - systicks_start_command), &mountstring[0]);
			usart_send_string((uint8_t*)&mountstring[0]);
			usart_send_string((uint8_t*)" ticks;\r\n");
		}
		else
		{
			//User messages
			usart_send_string((uint8_t*)"..  BAT not OK: Received 0x");
			conv_uint8_to_2a_hex(ps2_byte_received, &mountstring[0]);
			usart_send_string((uint8_t*)&mountstring[0]);
			usart_send_string((uint8_t*)" instead of 0xAA\r\n");
		}
	}
	
	//Send command Read ID. It musts responds with 0xFA (implicit), 0xAB, 0x83
	// Wait clock line to be unactive for 100 ms (3 systicks)
	//usart_send_string((uint8_t*)"Sending Read ID comm\r\n");
	systicks_start_command = systicks;
	ps2_send_command(COMM_READ_ID, ARG_NO_ARG); //Read ID command.
	while (!command_ok && (systicks - systicks_start_command)<(3*3)) //Must be excecuted in less than 100ms
	{
		prev_systicks = systicks;	//To avoid errors on keyboard power up BEFORE the first access
		if(systicks != systicks_start_command)
		{
			prev_systicks = systicks;	//To avoid errors on keyboard power up BEFORE the first access
		}
	}
	if (command_ok)
	{
		//usart_send_string((uint8_t*)"Waiting 0xAB\r\n");
		systicks_start_command = systicks;
		while(!available_ps2_byte()&& (systicks - systicks_start_command)<(1*3))
    __asm("nop");
		ps2_byte_received = get_ps2_byte(&ps2_recv_buffer[0]);
		if(ps2_byte_received == KB_FIRST_ID)
		{
			//usart_send_string((uint8_t*)"Waiting 0x83\r\n");
			systicks_start_command = systicks;
			while(!available_ps2_byte() && (systicks - systicks_start_command)<(1*3))
			__asm("nop");
			ps2_byte_received = get_ps2_byte(&ps2_recv_buffer[0]);
			if(ps2_byte_received == KB_SECOND_ID)
			{
				ps2_keyb_detected = true;
				//User messages
				usart_send_string((uint8_t*)"..  PS/2 Keyboard detected;\r\n");
			}
			else
			{
				//usart_send_string((uint8_t*)"Did not receive 0x83");
				usart_send_string((uint8_t*)"..  PS/2 Keyboard not detected!\r\n");
				return ps2_keyb_detected;
			}
		}
		else
		{
			//usart_send_string((uint8_t*)"Did not receive 0xAB");
			return ps2_keyb_detected;
		}
	}
	else
	{
		//User messages
		/*usart_send_string((uint8_t*)"PS/2 ReadID command not OK. Elapsed time was ");
		conv_uint32_to_8a_hex((systicks - systicks_start_command), &mountstring[0]);
		usart_send_string((uint8_t*)&mountstring[0]);
		usart_send_string((uint8_t*)"\r\n"); */
		return ps2_keyb_detected;
	}

	//The objective of this block is to minimize the keyboard interruptions, to keep time to high priority MSX interrupts.
	//Send type 3 command 0xFA (Set Key Type Make/Break - This one only disables typematic repeat):
	//  If it does not receive "ack" (0xFA), then send type 2 command 0xF3 + 0x7F (2cps repeat rate + 1 second delay)
	//  It musts respond with an "ack" after the first byte, than with a second "ack" after the second byte.
	//User messages
	//usart_send_string((uint8_t*)"Type 2 sets typematic repeat 0xF3 0x7F requested\r\n");
	
	//Type 2 command: Set typematic rate to 2 cps and delay to 1 second.
	systicks_start_command = systicks;
	ps2_send_command(COMM_SET_TYPEMATIC_RATEDELAY, ARG_LOWRATE_LOWDELAY);
	while (!command_ok && (systicks - systicks_start_command)<(2*3)) //Must be excecuted in less than 200ms
		__asm("nop");
	if (command_ok)
	{
		//User messages
		usart_send_string((uint8_t*)"..  Delay 1 second to repeat, 2cps repeat rate (Type 2 command) OK;\r\n");
	}
	else
	{
		//User messages
		usart_send_string((uint8_t*)"..  Type 3 Disables typematic repeat 0xFA requested\r\n");

		//.1 second delay (to display serial contents) ONLY TO DEBUG
		systicks_start_command = systicks;
		while ((systicks - systicks_start_command)<(1*3))
			__asm("nop");

		systicks_start_command = systicks;
		//Type 3 command: Set All Keys Make/Break: This one only disables typematic repeat and applies to all keys
		ps2_send_command(COMM_TYPE3_NO_REPEAT, ARG_NO_ARG);
		while (!command_ok && (systicks - systicks_start_command)<(1*3)) //Must be excecuted in less than 100ms
			__asm("nop");
		if (command_ok)
		{
			//User messages
			usart_send_string((uint8_t*)"..  Type 3 Disables typematic 0xFA repeat OK\r\n");
		}
	}
	return ps2_keyb_detected;
}


/*************************************************************************************************/
/******************************  Support to other ISR's ******************************************/
/*************************************************************************************************/

void ps2_send_command(uint8_t cmd, uint8_t argm)
{
	uint8_t mountstring[16];					//Used in usart_send_string()
	if(ps2int_state != PS2INT_RECEIVE)
	{
		//User messages
		usart_send_string((uint8_t*)"0x");
		conv_uint16_to_4a_hex(ps2int_state, &mountstring[0]);
		usart_send_string((uint8_t*)&mountstring[0]);
		usart_send_string((uint8_t*)", instead of 0x0400\r\n");
	}
	command =  cmd;
	argument = argm;
	ps2int_state = PS2INT_SEND_COMMAND;
	send_start_bit_now();
}


void ps2_update_leds(bool num, bool caps, bool scroll)
{
	uint8_t mountstring[16];					//Used in usart_send_string()
	if(ps2int_state != PS2INT_RECEIVE)
	{
		//User messages
		usart_send_string((uint8_t*)"0x");
		conv_uint16_to_4a_hex(ps2int_state, &mountstring[0]);
		usart_send_string((uint8_t*)&mountstring[0]);
		usart_send_string((uint8_t*)", instead of 0x0400\r\n");
	}
	command = COMM_SET_RESET_LEDS;
	argument = (scroll<<0)|(num<<1)|(caps<<2);
	ps2int_state = PS2INT_SEND_COMMAND;
	send_start_bit_now();
}


//Insert a delay before run send_start_bit_now()
void send_start_bit_next(uint16_t x_usec)
{
	delay_usec(x_usec, send_start_bit_now); //wait x_usec and go to send_start_bit on TIM2 overflow interrupt
}


//This three functions are the split of Transmit Initiator, to avoid stuck inside an interrupt due to 120u and 20usec
void send_start_bit_now(void)
{
	timer_disable_irq(TIM2, TIM_DIER_CC1IE);	// Disable interrupt on Capture/Compare1, but keeps on overflow
	exti_disable_request(PS2_CLOCK_PIN_EXTI);
	command_ok = false; //Here command_OK is initialized
	gpio_set(PS2_DATA_PIN_PORT, PS2_DATA_PIN_ID);
	gpio_clear(PS2_CLOCK_PIN_PORT, PS2_CLOCK_PIN_ID);
	//if keyboard interrupt was not disabled, it would be interrupted here, pointing to something not coded
	//Something was wrong with original delay, so I decided to use TIM2 Capture/Compare interrupt
	// See hr_timer_delay.c file
	//now insert a 120us delay and run step 2 of send_start_bit function
	prev_systicks = systicks;
	delay_usec(120, send_start_bit2); //wait 120usec and go to send_start_bit2 on TIM2 overflow interrupt
}


void send_start_bit2(void) //Second part of send_start_bit
{
	gpio_clear(PS2_DATA_PIN_PORT, PS2_DATA_PIN_ID); //this is the start bit
	//now insert a 10us delay and run step 3 of send_start_bit function
	delay_usec(10, send_start_bit3); /*wait 10usec and go to send_start_bit3 on TIM2 overflow interrupt*/
}


void send_start_bit3(void) //Third part of send_start_bit
{
	prev_systicks = systicks;
	//Rise clock edge starts the PS/2 device to receive command/argument
	gpio_set(PS2_CLOCK_PIN_PORT, PS2_CLOCK_PIN_ID);

	ps2int_TX_bit_idx = 0;	// In TX Int, as we don't manage start bit inside int, idx can start with 0.

	prepares_capture(TIM2);
	exti_reset_request(PS2_CLOCK_PIN_EXTI);
	exti_enable_request(PS2_CLOCK_PIN_EXTI);
}


/*****************  Excecution of bitbang Support to other ISR's **********************/

/*  Enter point of PS/2 clock line, called from interrupt handled by msxhid  */
/*  Here is decided if the int is going to be treated as send or receive*/
void ps2_clock_update(bool ps2datapin_logicstate)
{
	if (!ps2datapin_logicstate && (ps2datapin_logicstate == formerps2datapin))
	{
		//State low is repeated
		acctimeps2data0 += time_between_ps2clk;
		if (acctimeps2data0 >= 200000) //.2s
		{
			gpio_set(PS2_DATA_PIN_PORT, PS2_DATA_PIN_ID);
			ps2int_state = PS2INT_RECEIVE;
			ps2int_RX_bit_idx = 0;
		}
	}
	else
		acctimeps2data0 = 0;		//Reset acc time counter
	formerps2datapin = ps2datapin_logicstate;	//To compare at next bit
	uint8_t mountstring[16]; //Used in usart_send_string()
	/*Any keyboard interrupt that comes after 900 micro seconds means an error condition,
	but I`m considering it as an error for about 100 ms, to acommodate this to power on, to answer 
	to Read ID command. I observed this behavior on my own PS/2 keyboard. It is huge!*/
	if( ((ps2int_state == PS2INT_SEND_COMMAND) || (ps2int_state == PS2INT_SEND_ARGUMENT))
				&& ((ps2int_TX_bit_idx != 0) && (systicks - prev_systicks) > 1) )
	{	//reset to PS/2 receive condition
		gpio_set(PS2_DATA_PIN_PORT, PS2_DATA_PIN_ID);
		//User messages
		usart_send_string((uint8_t*)"ps2_clock_sent reseted - Timeout = ");
		conv_uint32_to_dec((systicks - prev_systicks), &mountstring[0]);
		usart_send_string((uint8_t*)&mountstring[0]);
		usart_send_string((uint8_t*)", ps2int_TX_bit_idx = ");
		conv_uint32_to_dec((uint32_t)ps2int_TX_bit_idx, &mountstring[0]);
		usart_send_string((uint8_t*)&mountstring[0]);
		usart_send_string((uint8_t*)", ");
		conv_uint16_to_4a_hex(ps2int_state, &mountstring[0]);
		usart_send_string((uint8_t*)&mountstring[0]);
		usart_send_string((uint8_t*)";\r\n");
		ps2int_state = PS2INT_RECEIVE;
		ps2int_RX_bit_idx = 0;
	}
	
	if( (ps2int_state == PS2INT_SEND_COMMAND) || (ps2int_state == PS2INT_SEND_ARGUMENT) )
	{
		ps2_clock_send(ps2datapin_logicstate);
	}
	else
	{
		ps2_clock_receive(ps2datapin_logicstate);
	}
}


void ps2_clock_send(bool ps2datapin_logicstate)
{
	//uint8_t mountstring[16]; //Used in usart_send_string()
	prev_systicks = systicks;
	//Time check - The same for all bits
	if (time_between_ps2clk > 10000) // time >10ms
	{
		usart_send_string((uint8_t*)"Time > 10ms on TX");
	}
	//|variável| = `if`(condição) ? <valor1 se true> : <valor2 se false>;:
	//Only two TX states of send: ps2_send_command & send_argument
	uint8_t data_byte = (ps2int_state == PS2INT_SEND_COMMAND) ? command : argument;
	//if( (ps2int_TX_bit_idx >= 0) && (ps2int_TX_bit_idx < 8) )
	if(ps2int_TX_bit_idx < 8)
	{
		bool bit = data_byte & (1 << (ps2int_TX_bit_idx));
		ps2int_TX_bit_idx++;
		if(bit)
		{
			gpio_set(PS2_DATA_PIN_PORT, PS2_DATA_PIN_ID);
			//User messages
			/*usart_send_string((uint8_t*)"sent bit #");
			conv_uint32_to_dec((uint32_t)ps2int_TX_bit_idx-1, &mountstring[0]);
			usart_send_string((uint8_t*)&mountstring[0]);
			usart_send_string((uint8_t*)": ");
			usart_send_string((uint8_t*)"1\r\n");*/
		}
		else
		{
			gpio_clear(PS2_DATA_PIN_PORT, PS2_DATA_PIN_ID);
			//User messages
			/*usart_send_string((uint8_t*)"sent bit #");
			conv_uint32_to_dec((uint32_t)ps2int_TX_bit_idx-1, &mountstring[0]);
			usart_send_string((uint8_t*)&mountstring[0]);
			usart_send_string((uint8_t*)": ");
			usart_send_string((uint8_t*)"0\r\n");*/
		}
	}
	else if(ps2int_TX_bit_idx == 8)
	{//parity
		bool parity =! __builtin_parity(data_byte);
		//User messages
		//usart_send_string((uint8_t*)"sent p: "); //This print continues below
		if(parity)
		{
			gpio_set(PS2_DATA_PIN_PORT, PS2_DATA_PIN_ID);
			//User messages
			//usart_send_string((uint8_t*)"1\r\n");
		}
		else
		{
			gpio_clear(PS2_DATA_PIN_PORT, PS2_DATA_PIN_ID);
			//User messages
			//usart_send_string((uint8_t*)"0\r\n");
		}
		ps2int_TX_bit_idx = 9;
	}
	else if(ps2int_TX_bit_idx == 9)
	{//stop bit
		gpio_set(PS2_DATA_PIN_PORT, PS2_DATA_PIN_ID);
		ps2int_TX_bit_idx = 10;
		//User messages
		//usart_send_string((uint8_t*)"sent stop\r\n");
	}
	else if(ps2int_TX_bit_idx >= 10)
	{
		if(ps2datapin_logicstate == false)
		{
			//  ACK bit ok
			//User messages
			/*usart_send_string((uint8_t*)"TX Data sent OK: 0x");
			conv_uint8_to_2a_hex(data_byte, &mountstring[0]);
			usart_send_string((uint8_t*)&mountstring[0]);
			usart_send_string((uint8_t*)", 0x");
			conv_uint16_to_4a_hex(ps2int_state, &mountstring[0]);
			usart_send_string((uint8_t*)&mountstring[0]);
			usart_send_string((uint8_t*)"\r\n");*/
			
		}
		else
		{
			// Ack bit NOT ok
			gpio_set(PS2_DATA_PIN_PORT, PS2_DATA_PIN_ID);	//To warranty that is not caused by let this pin LOW
			//User messages
			/*usart_send_string((uint8_t*)"Trying to send 0x");
			conv_uint8_to_2a_hex(data_byte, &mountstring[0]);
			usart_send_string((uint8_t*)&mountstring[0]);
			usart_send_string((uint8_t*)". ACK bit not received from keyboard\r\n");*/
			ps2int_state = PS2INT_RECEIVE; //force to receive_status in absence of something better
			ps2int_RX_bit_idx = 0;
		}

		if(ps2int_state == PS2INT_SEND_COMMAND)
		{
			//For me, after commmand sent, you have to wait the PS/2 Acknowlodge from the command,
			//but the original logic pointed to state=PS2INT_RECEIVE.
			ps2int_state = PS2INT_WAIT_FOR_COMMAND_ACK;
			ps2int_RX_bit_idx =  0;
			//User messages
			/*usart_send_string((uint8_t*)"TX: new 0x");
			conv_uint16_to_4a_hex(ps2int_state, &mountstring[0]);
			usart_send_string((uint8_t*)&mountstring[0]);
			usart_send_string((uint8_t*)"\r\n");*/
		}
		else if(ps2int_state == PS2INT_SEND_ARGUMENT)
		{
			ps2int_state = PS2INT_WAIT_FOR_ARGUMENT_ACK;
			ps2int_RX_bit_idx =  0;
			//User messages
			/*usart_send_string((uint8_t*)"TX: new 0x");
			conv_uint16_to_4a_hex(ps2int_state, &mountstring[0]);
			usart_send_string((uint8_t*)&mountstring[0]);
			usart_send_string((uint8_t*)"\r\n");*/
		}
		else
		{
			gpio_set(PS2_DATA_PIN_PORT, PS2_DATA_PIN_ID);
			ps2int_state = PS2INT_RECEIVE;
			ps2int_RX_bit_idx =  0;
			//User messages
			/*usart_send_string((uint8_t*)"TX: new 0x");
			conv_uint16_to_4a_hex(ps2int_state, &mountstring[0]);
			usart_send_string((uint8_t*)&mountstring[0]);
			usart_send_string((uint8_t*)"\r\n");*/
		}
	}
}


void ps2_clock_receive(bool ps2datapin_logicstate)
{
	static uint8_t data_word, stop_bit, parity_bit;
	uint8_t mountstring[16]; //Used in usart_send_string()

	//Verify RX timeout, that is quite restricted, if compared to Send Command/Argument
	if ( (ps2int_RX_bit_idx != 0) && (time_between_ps2clk > 120) )  //because if RX_bit_idx == 0 will be the reset
	{	
		//usart_send_string((uint8_t*)"ps2_clock_receive - Timeout\r\n");
		ps2int_RX_bit_idx = 0;
	}
	prev_systicks = systicks;

	if(ps2int_RX_bit_idx == 0)
	{
		//Force this interface to put data line in Hi-Z to avoid unspected behavior in case of errors
		gpio_set(PS2_DATA_PIN_PORT, PS2_DATA_PIN_ID);
		//User messages
		//usart_send_string((uint8_t*)"RX: ps2int_state = 0x");
		//conv_uint16_to_4a_hex(ps2int_state, &mountstring[0]);
		//usart_send_string((uint8_t*)&mountstring[0]);
		//usart_send_string((uint8_t*)"\r\n");
		data_word = 0;
		stop_bit = 0xff;
		parity_bit = 0xff;
		ps2int_RX_bit_idx = 1; //Points to the next bit
		bool start_bit = ps2datapin_logicstate;
		if(start_bit)
		{
			fail_count++;
  		ps2int_RX_bit_idx = 0; //reset
		}
	}
	else if( (ps2int_RX_bit_idx>0) && (ps2int_RX_bit_idx<9) ) // collect bits 1 to 8 (D0 to D7)
	{
		data_word |= (ps2datapin_logicstate << (ps2int_RX_bit_idx - 1));
		ps2int_RX_bit_idx++;
	}
	else if(ps2int_RX_bit_idx == 9)
	{
		parity_bit = ps2datapin_logicstate;
		ps2int_RX_bit_idx++;
	}
	else if(ps2int_RX_bit_idx == 10)
	{	 // start + 8 + stop + parity (but started with 0)
		ps2int_RX_bit_idx = 0;	//next (reset) PS/2 receive condition

		stop_bit = ps2datapin_logicstate;
		bool parity_ok = __builtin_parity((data_word<<1)|parity_bit);
		//User messages
		/*usart_send_string((uint8_t*)"RX Data: 0x");
		conv_uint8_to_2a_hex(data_word, &mountstring[0]);
		usart_send_string((uint8_t*)&mountstring[0]);
		if(parity_ok)
			usart_send_string((uint8_t*)", pbit OK,");
		else
			usart_send_string((uint8_t*)", pbit issue,");
		if(stop_bit == 1)
			usart_send_string((uint8_t*)" sbit OK,");
		else
			usart_send_string((uint8_t*)" sbit issue,");
		usart_send_string((uint8_t*)" 0x");
		conv_uint16_to_4a_hex(ps2int_state, &mountstring[0]);
		usart_send_string((uint8_t*)&mountstring[0]);
		usart_send_string((uint8_t*)"\r\n");*/

		if(parity_ok && (stop_bit == 1) ) //start bit condition was already tested above
		{	/* ps2int_status receive procesing block (begin) */
			if (ps2int_state == PS2INT_RECEIVE)
			{
				//this put routine is new
				uint8_t i = ps2_recv_buff_put;
				uint8_t i_next = i + 1;
				i_next &= (uint8_t)(PS2_RECV_BUFFER_SIZE - 1);
				if (i_next != ps2_recv_buff_get)
				{
					ps2_recv_buffer[i] = data_word;
					ps2_recv_buff_put = i_next;
				}
			}

			else if (ps2int_state == PS2INT_WAIT_FOR_COMMAND_ACK)
			{
				if(data_word == KB_ACKNOWLEDGE) //0xFA is Acknowledge from PS/2 keyboard
				{
					if(argument == ARG_NO_ARG) //0xFF is an empty argument
					{
						//no argument: set to receive
						ps2int_state = PS2INT_RECEIVE;
						ps2int_RX_bit_idx = 0;//reset PS/2 receive condition
						command_ok = true;
					}
					else
					{
						//argument is not empty (!=ARG_NO_ARG). Send it
						ps2int_state = PS2INT_SEND_ARGUMENT;
						send_start_bit_next(150);
					}
				}
				else if(data_word == 0xfe) //0xFE is Resend
				{
					ps2int_state = PS2INT_SEND_COMMAND;
					send_start_bit_next(150); //Send BOTH command and argument
				}	//if(data_word==0xfe) //0xFE is Resend
				else
				{
					//User messages
					usart_send_string((uint8_t*)"Got unexpected command response: 0x");
					conv_uint8_to_2a_hex(data_word, &mountstring[0]);
					usart_send_string((uint8_t*)&mountstring[0]);
					usart_send_string((uint8_t*)"\r\n");
					ps2int_state = PS2INT_RECEIVE;
					ps2int_RX_bit_idx = 0;//reset PS/2 receive condition
					fail_count++;
				}	//else if(data_word==0xfe) //0xFE is Resend
			}

			else if (ps2int_state == PS2INT_WAIT_FOR_ARGUMENT_ACK)
			{
				if(data_word == KB_ACKNOWLEDGE) //Acknowledge from PS/2 keyboard
				{
					//Acknowledge received => set to receive
					ps2int_state = PS2INT_RECEIVE;
					ps2int_RX_bit_idx = 0;//Prepares for the next PS/2 receive condition
					command_ok = true;
				}
				else if(data_word == 0xfe) //0xFE is Resend
				{
					ps2int_state = PS2INT_SEND_COMMAND; //Resend BOTH command AND argument
					send_start_bit_next(150);
				}
				else
				{
					ps2int_state = PS2INT_RECEIVE;
					ps2int_RX_bit_idx = 0;//reset PS/2 receive condition
					//User messages
					usart_send_string((uint8_t*)"Got unexpected command response: 0x");
					conv_uint8_to_2a_hex(data_word, &mountstring[0]);
					usart_send_string((uint8_t*)&mountstring[0]);
					usart_send_string((uint8_t*)"\r\n");
				}
			}		/* ps2int_status receive procesing block (wnd) */
		}	//if(start_bit==0 && stop_bit==1 && parity_ok)
		else
		{
			//User messages
			/*usart_send_string((uint8_t*)"Framming Error. RX Data: 0x");
			conv_uint8_to_2a_hex(data_byte, &mountstring[0]);
			usart_send_string((uint8_t*)&mountstring[0]);
			usart_send_string((uint8_t*)", parity ");
			mountstring = parity_bit ? "1" : "0";
			usart_send_string((uint8_t*)&mountstring[0]);
			usart_send_string((uint8_t*)", Stop ");
			mountstring = stop_bit ? "1" : "0";
			usart_send_string((uint8_t*)&mountstring[0]);
			usart_send_string((uint8_t*)"\r\n"); */
			fail_count++;
			ps2int_RX_bit_idx = 0;
		}
	}	//else if(ps2int_RX_bit_idx==10)
}


void reset_mount_scancode_machine()
{
	mount_scancode_count_status = 0;
	ps2_keystr_e0 = false;
	ps2_keystr_e1 = false;
	ps2_keystr_f0 = false;
}


bool mount_scancode()
{
	// static uint16_t prev_state_index=0;
	if (!mount_scancode_OK)
	{	
		while(available_ps2_byte() && !mount_scancode_OK)
		{
			ps2_byte_received = get_ps2_byte(&ps2_recv_buffer[0]);
			//User messages
			/*usart_send_string((uint8_t*)"Mount_scancode RX Ch=");
			conv_uint8_to_2a_hex(ps2_byte_received, &mountstring[0]);
			usart_send_string((uint8_t*)&mountstring[0]);
			usart_send_string((uint8_t*)"\r\n"); */
			switch (mount_scancode_count_status)
			{
			case 0:	//Está lendo o primeiro byte do ps2_byte_received
			{
				if((ps2_byte_received > 0) && (ps2_byte_received < 0xE0)) //Se até 0xDF cai aqui
				{
					//Concluded. 1 byte only scan code.
					scancode[1] = ps2_byte_received;
					//Conclui scan
					scancode[0] = 1;
					mount_scancode_OK = true;
					reset_mount_scancode_machine();
					return true;
				}
				if(ps2_byte_received == 0xE0)
				{
					//0xE0 + (Any != 0xF0) <= 2 bytes
					//0xE0 + 0xF0 + (Any)  <= 3 bytes
					ps2_keystr_e0 = true;
					scancode[1] = ps2_byte_received;
					scancode[0] = 1;
					mount_scancode_count_status = 1; //points to next case
					break;
				}
				if(ps2_byte_received == 0xE1)
				{
					// Pause/Break key: 8 bytes (0xE1 + 7 bytes). Store only the 3 first bytes and discard the others
					ps2_keystr_e1 = true;
					scancode[1] = ps2_byte_received;
					scancode[0] = 1;
					mount_scancode_count_status = 1; //points to next case
					break;
				}
				if(ps2_byte_received == 0xF0)
				{
					//Always 2 bytes 0xF0 + (Any)
					ps2_keystr_f0 = true;
					scancode[1] = ps2_byte_received;
					scancode[0] = 1;
					mount_scancode_count_status = 1; //points to next case
					break;
				}
 				break;	//case syntax suggested
			}	//case 0:
			case 1:
			{
				if (ps2_keystr_e0 == true)
				{
					if(ps2_byte_received != 0xF0)
					{
						if(ps2_byte_received != 0x12)
						{
							//2 bytes and this ps2_byte_received is != 0xF0 e != 0x12, so, concluded
							scancode[2] = ps2_byte_received;
							scancode[0] = 2;
							//task concluded
							mount_scancode_OK = true;
							reset_mount_scancode_machine();
							return true;
						}
						else
						{
							//Discard E0 12 here
							reset_mount_scancode_machine();
							break;
						}
					}
					else //if(ps2_byte_received == 0xF0)
					{
							//3 bytes, but I'm reading the second one (E0 F0)
							scancode[2] = ps2_byte_received;
							scancode[0] = 2;
							mount_scancode_count_status = 2; //points to next case
							break;
					}
				}
				if (ps2_keystr_e1)  //Break key (8 bytes)
				{
					//8 bytes (0xE1 + 7 bytes). Ler apenas os 3 iniciais e desconsiderar os demais. Estou lendo o segundo byte
					scancode[2] = ps2_byte_received;
					scancode[0] = 2;
					mount_scancode_count_status = 2; //points to next case
					break;
				}
				if (ps2_keystr_f0 == true)
				{
					//São 2 bytes, logo, terminou
					// Exception are all the keys starting with E0 7C. Discard this,
					// If you want to map these keys, fix them in the excel origin file.
					scancode[2] = ps2_byte_received;
					scancode[0] = 2;
					//Conclui scan
					mount_scancode_OK = true;
					reset_mount_scancode_machine();
					return true;
				}
 				break;	//case syntax suggested
			}	//case 1:
			case 2:
			//Está lendo o terceiro byte do ps2_byte_received
			{
				if (ps2_keystr_e0 == true)
				{
					//São 3 bytes, e estou lendo o terceiro byte, logo, terminou.
					// Exception is the PrintScreen break: It will be returned as one 3 bytes ps2_byte_received releases:
					// E0 F0 7C (and E0 F0 12 is dicarded), but this key is not present on MSX.
					// If you want to map this key, fix it in excel file and click on the black keyboard to rerun macro
					if(ps2_byte_received != 0x12)
					{
						scancode[3] = ps2_byte_received;
						scancode[0] = 3;
						//Conclui scan
						mount_scancode_OK = true;
						reset_mount_scancode_machine();
						return true;
					}
					else
					{
						//Discard E0 F0 12 here
						reset_mount_scancode_machine();
						break;
					}
				}
				else if (ps2_keystr_e1 == true)  //Break key (8 bytes)
				{
					//São 8 bytes (0xE1 + 7 bytes). Ler apenas os 3 iniciais e desprezar os demais.
					//Estou lendo o terceiro byte.
					scancode[3] = ps2_byte_received;
					scancode[0] = 3;
					mount_scancode_count_status = 3; //points to next case
					break;
				}  	
 				break;	//case syntax suggested
			}	//case 2:
			//Está lendo o quarto byte do ps2_byte_received Pause/Break
			case 3:
			{
				if (ps2_keystr_e1 == true)  //Break key (8 bytes)
				{
					//São 8 bytes (0xE1 + 7 bytes). Ler apenas os 3 iniciais e desprezar os demais. 
					//Estou lendo o quarto byte. Não o armazeno.
					mount_scancode_count_status = 4; //points to next case
					break;
				}	   	
 				break;	//case syntax suggested
			}	//case 3:
			//if (mount_scancode_count_status == 4)  //Está lendo o quinto byte do ps2_byte_received Pause/Break
			case 4:
			{
				if (ps2_keystr_e1 == true)  //Break key (8 bytes)
				{
					//São 8 bytes (0xE1 + 7 bytes). Ler apenas os 3 iniciais e desprezar os demais. 
					//Estou lendo o quinto byte. Não o armazeno.
					mount_scancode_count_status = 5; //points to next case
					break;
				}	   	
 				break;	//case syntax suggested
			}		//case 4:
			//Está lendo o sexto byte do ps2_byte_received Pause/Break
			case 5:
			{
				if (ps2_keystr_e1 == true)  //Break key (8 bytes)
				{
					//São 8 bytes (0xE1 + 7 bytes). Ler apenas os 3 iniciais e desprezar os demais. 
					//Estou lendo o sexto byte. Não o armazeno.
					mount_scancode_count_status = 6; //points to next case
					break;
				}	   	
 				break;	//case syntax suggested
			}	//case 5
			//Está lendo o sétimo byte do ps2_byte_received Pause/Break
			case 6:
			{
				if (ps2_keystr_e1 == true)  //Break key (8 bytes)
					{
						//São 8 bytes (0xE1 + 7 bytes). Ler apenas os 3 iniciais e desprezar os demais. 
						//Estou lendo o sétimo byte. Não o armazeno.
						mount_scancode_count_status = 7; //points to next case
						break;
					}	   	
 				break;	//case syntax suggested
			}	//case 6
			case 7:
			  //Está lendo o oitavo byte do ps2_byte_received Pause/Break
			{
				//São 8 bytes (0xE1 + 7 bytes). Ler apenas os 3 iniciais e desprezar os demais. 
				//Estou lendo o oitavo byte e não o armazeno, logo, terminou
				//Conclui scan
				mount_scancode_OK = true;
				reset_mount_scancode_machine();
				return true;
			}
			default:
				break;
			}	//switch (mount_scancode_count_status)
		} //while((ps2_byte_received=get_ps2_byte(&ps2_recv_buffer[0]))!=0 && !mount_scancode_OK)
		return false;
	}//if (!mount_scancode_OK)
	return false;
}


void general_debug_setup(void)
{
	gpio_mode_setup(USER_KEY_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, USER_KEY_PIN_ID);

	gpio_mode_setup(INT_PS2_PIN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, INT_PS2_PIN_ID);
	gpio_set_output_options(INT_PS2_PIN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, INT_PS2_PIN_ID);
	gpio_set(INT_PS2_PIN_PORT, INT_PS2_PIN_ID); //Default condition is "1"

	gpio_mode_setup(TIM2CC1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TIM2CC1_PIN_ID);
	gpio_set_output_options(TIM2CC1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, TIM2CC1_PIN_ID);
	gpio_set(TIM2CC1_PORT, TIM2CC1_PIN_ID); //Default condition is "1"

	gpio_mode_setup(TIM2UIF_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TIM2UIF_PIN_ID);
	gpio_set_output_options(TIM2UIF_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, TIM2UIF_PIN_ID);
	gpio_set(TIM2UIF_PORT, TIM2UIF_PIN_ID); //Default condition is "1"
	
	gpio_mode_setup(Dbg_Yint_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, Dbg_Yint_pin_id);
	gpio_set_output_options(Dbg_Yint_port, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, Dbg_Yint_pin_id);
	gpio_set(Dbg_Yint_port, Dbg_Yint_pin_id); //Default condition is "1"
}

void put_pullups_on_non_used_pins(void)
{
	//Left these pins as inputs, but floating state is avoided by activating internal pull-up resistor.
	gpio_mode_setup(AVAILABLE_A2_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, AVAILABLE_A2_PIN_ID);
	gpio_mode_setup(AVAILABLE_A3_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, AVAILABLE_A3_PIN_ID);
	gpio_mode_setup(AVAILABLE_A11_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, AVAILABLE_A11_PIN_ID);
	gpio_mode_setup(AVAILABLE_A12_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, AVAILABLE_A12_PIN_ID);
	gpio_mode_setup(AVAILABLE_B2_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, AVAILABLE_B2_PIN_ID);	//This port has a R=10K to GND
}

/*************************************************************************************************/
/*************************************************************************************************/
/******************************************* ISR's ***********************************************/
/*************************************************************************************************/
/*************************************************************************************************/
void exti15_10_isr(void) // PS/2 Clock
{
	//Now starts PS2Clk interrupt handler
	//Debug & performance measurement
	gpio_clear(INT_PS2_PIN_PORT, INT_PS2_PIN_ID); //Signs start of interruption
	//This was the ISR of PS/2 clock pin. It jumps to ps2_clock_update.
	//It is an important ISR, but it does not require critical timming resources as MSX Y scan does.
	if(exti_get_flag_status(PS2_CLOCK_PIN_EXTI))  // EXTI15
	{
		bool ps2datapin_logicstate=gpio_get(PS2_DATA_PIN_PORT, PS2_DATA_PIN_ID);
		ps2_clock_update(ps2datapin_logicstate);
		exti_reset_request(PS2_CLOCK_PIN_EXTI);
	}
	//Debug & performance measurement
	gpio_set(INT_PS2_PIN_PORT, INT_PS2_PIN_ID); //Signs end of interruption. Default condition is "1"
}
