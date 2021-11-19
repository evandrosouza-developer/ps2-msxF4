#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>

#include "msxmap.h"
#include "port_def.h"
#include "ps2handl.h"
#include "serial.h"
#include "dbasemgt.h"

//Use Tab width=2

#define MAX_LINHAS_VARRIDAS								320	//To acommodate maximum size of 3K (3072 lines of 10 bytes each - header included)
#define MAX_TIME_OF_IDLE_KEYSCAN_SYSTICKS	5		//30 / 5 = 6 times per second is the maximum sweep speed
#define	Y_SHIFT														6		//Shift colunm

#define PRESS_RELEASE_BIT_POSITION				3
#define PRESS_RELEASE_BIT									1 << PRESS_RELEASE_BIT_POSITION //8

//Variáveis globais: Visíveis por todo o contexto do programa
uint8_t* base_of_database;
extern uint32_t systicks;											//Declared on sys_timer.cpp
extern bool ps2numlockstate;									//Declared on ps2handl.c
volatile bool shiftstate;
volatile uint16_t linhavarrida;
volatile bool update_ps2_leds;
//Place to store previous time for each Y last scan
volatile uint32_t previous_y_systick[ 16 ] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

//Variable used to store the values of the X for each Y scan - Each Y has its own image of BSSR register
uint32_t x_bits[ 16 ]; //All pins that interface with PORT B of 8255 must have high level as default

uint8_t y_dummy;															//Read from MSX Database to sinalize "No keys mapping"
volatile uint32_t formerscancode;
volatile uint8_t scancode[4];									//O 1º é a quantidade de bytes;

uint8_t CtrlAltDel;
//First record of unused V.1.0 Database
uint8_t UNUSED_DATABASE[(uint8_t)DB_NUM_COLS] = {0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20};

extern bool enable_xon_xoff;									//Declared on serial.c

#define DISPATCH_QUEUE_SIZE							16
uint8_t dispatch_keys_queue_buffer[DISPATCH_QUEUE_SIZE];

struct ring dispatch_keys_queue;

// Table to translate the Y order: From port read to the expected one:
const uint8_t Y_XLAT_TABLE[uint8_t(16)] = { 0b0000, 0b1000, 0b0100, 0b1100,
																						0b0010, 0b1010, 0b0110, 0b1110,
																			 			0b0001, 0b1001, 0b0101, 0b1101,
																						0b0011, 0b1011, 0b0111, 0b1111};


//typedef const uint8_t DEFAULT_MSX_KEYB_DATABASE_CONVERSION[(uint16_t)320][(uint8_t)DB_NUM_COLS] t_database;
//typedef (const uint8_t t_database[(uint16_t)320][(uint8_t)DB_NUM_COLS]);
//typedef struct { int dia; char mes[10]; int ano;} Data;

void msxmap::msx_interface_setup(void)
{
	//Set Alternate function
	gpio_set_af(Y0_port, GPIO_AF3, Y0_pin_id | Y1_pin_id | Y2_pin_id | Y3_pin_id);
	gpio_set_af( X_port, GPIO_AF3, X0_pin_id | X1_pin_id | X2_pin_id | X3_pin_id | X4_pin_id | X5_pin_id | X6_pin_id | X7_pin_id);

	//Not the STM32 default, but it is the default MSX state: Release MSX keys;
	gpio_set(X_port,
	X7_pin_id | X6_pin_id | X5_pin_id | X4_pin_id | X3_pin_id | X2_pin_id | X1_pin_id | X0_pin_id);

	//Init output port B
	//gpio_mode_setup(X7_port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, 
	//X7_pin_id | X6_pin_id | X5_pin_id | X4_pin_id | X3_pin_id | X2_pin_id | X1_pin_id | X0_pin_id);
	gpio_mode_setup(X_port, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,
	X7_pin_id | X6_pin_id | X5_pin_id | X4_pin_id | X3_pin_id | X2_pin_id | X1_pin_id | X0_pin_id);
	gpio_set_output_options(X_port, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
	X7_pin_id | X6_pin_id | X5_pin_id | X4_pin_id | X3_pin_id | X2_pin_id | X1_pin_id | X0_pin_id);

	//Init startup state of BSSR image to each Y scan
	for(uint8_t i = 0; i < 16; i++)
		x_bits[ i ] = X7_SET_OR | X6_SET_OR | X5_SET_OR | X4_SET_OR | X3_SET_OR | X2_SET_OR | X1_SET_OR | X0_SET_OR;
	
	// Initialize dispatch_keys_queue ringbuffer
	ring_init(&dispatch_keys_queue, dispatch_keys_queue_buffer);
	for(uint8_t i=0; i<DISPATCH_QUEUE_SIZE; ++i)
		dispatch_keys_queue.data[i]=0;

	// GPIO pins for MSX keyboard Y scan (PC3:0 of the MSX 8255 - PC3 MSX 8255 Pin 17)
	//gpio_set(Y3_port, Y3_pin_id); //pull up resistor
	gpio_mode_setup(Y3_port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, Y3_pin_id); // PC3 (MSX 8255 Pin 17)
	exti_select_source(Y3_exti, Y3_port);
	exti_set_trigger(Y3_exti, EXTI_TRIGGER_BOTH); //Interrupt on change
	exti_reset_request(Y3_exti);
	exti_enable_request(Y3_exti);
	gpio_port_config_lock(Y3_port, Y3_pin_id);

	// GPIO pins for MSX keyboard Y scan (PC3:0 of the MSX 8255 - PC2 MSX 8255 Pin 16)
	//gpio_set(Y2_port, Y2_pin_id); //pull up resistor
	gpio_mode_setup(Y2_port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, Y2_pin_id); // PC2 (MSX 8255 Pin 16)
	exti_select_source(Y2_exti, Y2_port);
	exti_set_trigger(Y2_exti, EXTI_TRIGGER_BOTH); //Interrupt on change
	exti_reset_request(Y2_exti);
	exti_enable_request(Y2_exti);
	gpio_port_config_lock(Y2_port, Y2_pin_id);

	// GPIO pins for MSX keyboard Y scan (PC3:0 of the MSX 8255 - PC1 MSX 8255 Pin 15)
	//gpio_set(Y1_port, Y1_pin_id); //pull up resistor
	gpio_mode_setup(Y1_port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, Y1_pin_id); // PC1 (MSX 8255 Pin 15)
	exti_select_source(Y1_exti, Y1_port);
	exti_set_trigger(Y1_exti, EXTI_TRIGGER_BOTH); //Interrupt on change
	exti_reset_request(Y1_exti);
	exti_enable_request(Y1_exti);
	gpio_port_config_lock(Y1_port, Y1_pin_id);

	// GPIO pins for MSX keyboard Y scan (PC3:0 of the MSX 8255 - PC0 MSX 8255 Pin 14)
	//gpio_set(Y0_port, Y0_pin_id); //pull up resistor
	gpio_mode_setup(Y0_port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, Y0_pin_id); // PC0 (MSX 8255 Pin 14)
	exti_select_source(Y0_exti, Y0_port);
	exti_set_trigger(Y0_exti, EXTI_TRIGGER_BOTH); //Interrupt on change
	exti_reset_request(Y0_exti);
	exti_enable_request(Y0_exti);
	gpio_port_config_lock(Y0_port, Y0_pin_id);

	// GPIO pins for MSX keyboard Y scan (PC3:0 of the MSX 8255 - PC0 MSX 8255 Pin 14),
	// CAP_LED and KANA/Cyrillic_LED (mapped to scroll lock) to replicate in PS/2 keyboard
	// Set to input and enable internal pulldown

	// CAPS_LED
	gpio_mode_setup(CAPSLOCK_port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, CAPSLOCK_pin_id); // CAP_LED (MSX 8255 Pin 11)
	gpio_port_config_lock(CAPSLOCK_port, CAPSLOCK_pin_id);

	// Kana LED
	gpio_mode_setup(KANA_port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, KANA_pin_id); // KANA_LED - Mapeado para Scroll Lock
	gpio_set(KANA_port, KANA_pin_id); //pull up resistor
	gpio_port_config_lock(KANA_port, KANA_pin_id);

	// Enable EXTI9_5 interrupt. (for Y - bits 3 to 0)
	nvic_enable_irq(NVIC_EXTI9_5_IRQ);

	//Highest priority to avoid interrupt Y scan
	nvic_set_priority(NVIC_EXTI9_5_IRQ, 10); 		//Y3 to Y0
}


// Verify if there is an available ps2_byte_received on the receive ring buffer, but does not fetch this one
//Input: void
//Output: True if there is char available in input buffer or False if none
bool msxmap::available_msx_disp_keys_queue_buffer(void)
{
	uint8_t i = dispatch_keys_queue.get_ptr;
	if(i == dispatch_keys_queue.put_ptr)	//msx_dispatch_kq_put)
		return false;	//No char in buffer
	else
		return true;
}


// Fetches the next ps2_byte_received from the receive ring buffer
//Input: void
//Outut: Available byte read
uint8_t msxmap::get_msx_disp_keys_queue_buffer(void)
{
	uint8_t i, result;

	i = dispatch_keys_queue.get_ptr;
	if(i == dispatch_keys_queue.put_ptr)
		//No char in buffer
		return 0;
	result = dispatch_keys_queue.data[i];
	i++;
	dispatch_keys_queue.get_ptr = i & (uint8_t)(DISPATCH_QUEUE_SIZE- 1); //if(i) >= (uint16_t)DISPATCH_QUEUE_SIZE)	i = 0;
	return result;
}


/*Put a MSX Key (byte) into a buffer to get it mounted to x_bits
* Input: uint8_t as parameter
* Output: Total number of bytes in buffer, or ZERO if buffer was already full.*/
uint8_t msxmap::put_msx_disp_keys_queue_buffer(uint8_t data_word)
{
	uint8_t i, i_next;
	
	i = dispatch_keys_queue.put_ptr;
	i_next = i + 1;
	i_next &= (uint8_t)(DISPATCH_QUEUE_SIZE - 1);
	if (i_next != dispatch_keys_queue.get_ptr)
	{
		dispatch_keys_queue_buffer[i] = data_word;
		dispatch_keys_queue.put_ptr = i_next;
		return (uint8_t)(DISPATCH_QUEUE_SIZE - dispatch_keys_queue.get_ptr + dispatch_keys_queue.put_ptr) & (uint8_t)(DISPATCH_QUEUE_SIZE - 1);
	}
	else 
		return 0;
}


//The objective of this routine is implement a smooth typing.
//The usage is to put a byte key (bit 7:4 represents Y, bit 3 is the release=1/press=0, bits 2:0 are the X )
//F = 15Hz. This routine is called from Ticks interrupt routine
void msxmap::msxqueuekeys(void)
{
	uint8_t x_local, y_local, readkey;
	bool x_local_setb;
	if (available_msx_disp_keys_queue_buffer())
	{
		//This routine is allways called AFTER the mapped condition has been tested 
		readkey = get_msx_disp_keys_queue_buffer();
		y_local = (readkey & 0xF0) >> 4;
		x_local = readkey & 0x07;
		x_local_setb = ((readkey & PRESS_RELEASE_BIT) >> PRESS_RELEASE_BIT_POSITION) == (uint8_t)1;
		// Compute x_bits of ch and verifies when Y was last updated,
		// with aim to update MSX keys, no matters if the MSX has the interrupts stucked. 
		compute_x_bits_and_check_interrupt_stuck(y_local, x_local, x_local_setb);
	}
}


void msxmap::convert2msx()
{
	switch(CtrlAltDel)
	{
		case 0:
		{
			//First state: No reset keys
			if (
			(scancode[0] == (uint8_t)1) 		&&
			(scancode[1] == (uint8_t)0x14) ) //Left Control
				CtrlAltDel = 1;
			break;
		}
		case 1:
		{
			//Second state: Left Control
			if (
			(scancode[0] == (uint8_t)1) 		&&
			(scancode[1] == (uint8_t)0x11) ) //Left Alt
				//Left Control + Left Alt are pressed together
				CtrlAltDel = 2;
			else
				CtrlAltDel = 0;
			break;
		}
		case 2:
		{
			//Third state: Num Pad Del
			if (
			(scancode[0] == (uint8_t)1) 		&&
			(scancode[1] == (uint8_t)0x71) ) //Num pad Del
			{
				//Left Control + Left Alt + Num Pad Del are pressed together
				power_off_ps2_keyboard();
				//User messages
				usart_send_string((uint8_t*)"Reset requested\r\n");
				//Wait here .3 second to consolidate this power off
				uint32_t readsysticks = systicks;
				while (systicks <= (readsysticks + 10)) __asm("nop");	//wait 1/3 second
				systick_interrupt_disable();
				for(;;) {};//Wait here until reset
			}
			else
				CtrlAltDel = 0;
		}
	}

	if (
	(scancode[0] == (uint8_t)1) 		&&
	(scancode[1] == (uint8_t)0x77) )
	{
		//NumLock Pressed. Toggle NumLock status
		ps2numlockstate = !ps2numlockstate;
		update_ps2_leds = true;  //this will force update_leds at main loop => To save interrupt time
		return;
	}

	if (
	( scancode[0] == (uint8_t)1)	  &&
	((scancode[1] == (uint8_t)0x12) || (scancode[1] == (uint8_t)0x59)) )
	{
		//Shift Pressed.
		shiftstate = true;
	}

	if (
	( scancode[0] == (uint8_t)2)		&&
	( scancode[1] == (uint8_t)0xF0) &&
	((scancode[2] == (uint8_t)0x12) || (scancode[2]== (uint8_t)0x59)) )
	{
		//Shift Released (Break code).
		shiftstate = false;
		//return;
	}

	//Now searches for PS/2 scan code in MSX Table to match. First search first colunm
	linhavarrida = 1;
	while ( 
	//(MSX_KEYB_DATABASE_CONVERSION[linhavarrida][0] != scancode[1]) &&
	//(base_of_database[linhavarrida][0] != scancode[1]) &&
	(*(base_of_database+linhavarrida*DB_NUM_COLS+0) != scancode[1]) &&
	(linhavarrida < MAX_LINHAS_VARRIDAS) )
	{
		linhavarrida++;
	}
	if (
	(scancode[0] == (uint8_t)1) && 
	(scancode[1]  == *(base_of_database+linhavarrida*DB_NUM_COLS+0)) && 
	(linhavarrida < MAX_LINHAS_VARRIDAS) )
	{
		//1 byte key
		//msx_dispatch(linhavarrida);
		msx_dispatch();
		return;
	}
	if ((scancode[0] >= (uint8_t)1) &&
	(linhavarrida < MAX_LINHAS_VARRIDAS) )
	{
		//2 bytes key, then now search match on second byte of scancode
		while ( 
		(*(base_of_database+linhavarrida*DB_NUM_COLS+1) != scancode[2]) &&
		(linhavarrida < MAX_LINHAS_VARRIDAS))
		{
			linhavarrida++;
		}
		if(
		(scancode[0] == (uint8_t)2) && 
		(scancode[1] == *(base_of_database+linhavarrida*DB_NUM_COLS+0)) && 
		(scancode[2] == *(base_of_database+linhavarrida*DB_NUM_COLS+1)) )
		{
			//Ok: Matched code
			//msx_dispatch(linhavarrida);
			msx_dispatch();
			return;
		}
		//3 bytes key, then now search match on third byte of scancode
		while (
		(*(base_of_database+linhavarrida*DB_NUM_COLS+2) != scancode[3]) &&
		(linhavarrida < MAX_LINHAS_VARRIDAS) )
		{
			linhavarrida++;
		}
		if( 
		(scancode[0] == 3) && 
		(scancode[1] == *(base_of_database+linhavarrida*DB_NUM_COLS+0)) && 
		(scancode[2] == *(base_of_database+linhavarrida*DB_NUM_COLS+1)) &&
		(scancode[3] == *(base_of_database+linhavarrida*DB_NUM_COLS+2)) )
		{
			//Ok: Matched code
			//msx_dispatch(linhavarrida);
			msx_dispatch();
			return;
		}
	}	//if ((scancode[0] >= (uint8_t)1) && (linhavarrida < MAX_LINHAS_VARRIDAS) )
}	//void msxmap::convert2msx()

	/*
	The structure of the Database is:
		The  three first columns of each line are the mapped scan codes;
		The 4th column is The Control Byte, detailed bellow:
		CONTROL BYTE:
			High nibble is Reserved;
			(bit 3) Combined Shift;
			(bit 2) Reserved-Not used;
			(bits 1-0) Modifyer Type:
			.0 - Default mapping
			.1 - NumLock Status+Shift changes
			.2 - PS/2 Shift
			.3 - Reserved-Not used
		
		This table has 3 modifyers: Up two MSX keys are considered to each mapping behavior modifying:
		
		5th and 6th columns have the mapping ".0 - Default mapping";
		7th e 8th columns have the mapping ".1 - NumLock Status+Shift changes";
		9th and 10th columns have the mapping ".2 - PS/2 Shift", where I need to
		release the sinalized Shift in PS/2 to the MSX and put the coded key, and so,
		release them, reapplying the Shift key, deppending on the initial state;
		
		
		Each column has a MSX coded key with the following structure:
		(Bit 7:4) MSX Y PPI 8255 PC3:0 is send to an OC outputs BCD decoder, for example:
							In the case of Hotbit HB8000, the keyboard scan is done as a 9 columns scan, CI-16 7445N 08 to 00;
							If equals to 1111 (Y=15), there is no MSX key mapped.
		(Bit 3)		0: keypress
							1: key release;
		(Bit 2:0) MSX X, ie, which bit will carry the key, to be read by PPI 8255 PB7:0.
	
	
	Now in my native language: Português
	 Primeiramente verifico se este scan, que veio em scancode[n], está referenciado na tabela MSX_KEYB_DATABASE_CONVERSION.
	 Esta tabela, montada em excel, está pronta para ser colada
	 A tabela já está com sort, para tornar possível executar uma pesquisa otimizada.
	
	 As três primeiras posições (colunas) de cada linha são os scan codes mapeados;
	 A 4ª coluna é o controle, e tem a seguinte estrutura:
		CONTROL BYTE:
		High nibble is Reserved; 
		(bit 3) Combined Shift;
		(bit 2) Reserved-Not used;
		(bits 1-0) Modifyer Type:
		.0 - Default mapping
		.1 - NumLock Status+Shift changes
		.2 - PS/2 Shift
		.3 - Reserved-Not used
	
	Esta tabela contém 3 modificadores: São codificadas até 2 teclas do MSX para cada modificador de mapeamento:
	
	 5ª e 6ª colunas contém o mapeamento ".0 - Default mapping";
	 7ª e 8ª colunas contém o mapeamento ".1 - NumLock Status+Shift changes";
	 9ª e 10ª colunas contém o mapeamento ".2 - PS/2 Shift", onde necessito
	 liberar o Shift sinalizado no PS/2 para o MSX e colocar as teclas codificadas,
	 e liberá-las, reinserindo o Shift, se aplicável;
	
	
	 Cada coluna contém uma tecla codificada para o MSX, com a seguinte estrutura:
	 (Bit 7:4) MSX Y PPI 8255 PC3:0 é enviada à um decoder BCD com OC outputs.
						 MSX Y PPI 8255 PC3:0. No caso do Hotbit, o scan é feito em 9 colunas, CI-16 7445N 08 a 00;
						 Se 1111 (Y=15), não há tecla MSX mapeada.
	 (Bit 3)	 0: keypress
						 1: key release;
	 (Bit 2:0) MSX X, ou seja, qual bit levará a informação da tecla, a ser lida pela PPI 8255 PB7:0.
	
	*/

//void msxmap::msx_dispatch(volatile uint16_t linhavarrida)
void msxmap::msx_dispatch(void)
{
	volatile uint8_t y_local, x_local;
	uint16_t msx_Y_scan;
	volatile bool x_local_setb;

	//Qual o tipo de Mapeamento?
	
	switch(*(base_of_database+linhavarrida*DB_NUM_COLS+3) & 0x03)
	{
		case 0:
		{	// .0 - Mapeamento default (Colunas 4 e 5)
			// Tecla 1:
			y_local = (*(base_of_database+linhavarrida*DB_NUM_COLS+4) & (uint8_t)0xF0) >> 4;
			// Verifica se está mapeada
			if (y_local != y_dummy)
			{
				x_local = *(base_of_database+linhavarrida*DB_NUM_COLS+4) & (uint8_t)0x07;
				x_local_setb = ((*(base_of_database+linhavarrida*DB_NUM_COLS+4) & (uint8_t)PRESS_RELEASE_BIT) >> PRESS_RELEASE_BIT_POSITION) == (uint8_t)1;
				// Calcula x_bits da Tecla 1 e verifica se o tempo em que foi atualizado o dado de Linha X da Coluna Y,
				// com a finalidade de atualizar teclas mesmo sem o PPI ser atualizado. 
				compute_x_bits_and_check_interrupt_stuck(y_local, x_local, x_local_setb);
			}
			// Tecla 2:
			y_local = (*(base_of_database+linhavarrida*DB_NUM_COLS+5) & (uint8_t)0xF0) >> 4;
			// Verifica se está mapeada
			if (y_local != y_dummy)
			{
				put_msx_disp_keys_queue_buffer(*(base_of_database+linhavarrida*DB_NUM_COLS+5));
			}
			break;
		} // .0 - Mapeamento default (Colunas 4 e 5)

		case 1:
		{	// .1 - Mapeamento NumLock (Colunas 6 e 7)
			if (ps2numlockstate ^ shiftstate)
			{
				//numlock e Shift tem estados diferentes
				// Tecla 1 (PS/2 NumLock ON (Default)):
				y_local = (*(base_of_database+linhavarrida*DB_NUM_COLS+4) & (uint8_t)0xF0)  >> 4;
				// Verifica se está mapeada
				if (y_local != y_dummy)
				{
					x_local = *(base_of_database+linhavarrida*DB_NUM_COLS+4) & (uint8_t)0x07;
					x_local_setb = ((*(base_of_database+linhavarrida*DB_NUM_COLS+4) & PRESS_RELEASE_BIT) >> PRESS_RELEASE_BIT_POSITION) == (uint8_t)1;
					// Calcula x_bits da Tecla 1 e verifica se o tempo em que foi atualizado o dado de Linha X da Coluna Y,
					// com a finalidade de atualizar teclas mesmo sem o PPI ser atualizado. 
					compute_x_bits_and_check_interrupt_stuck(y_local, x_local, x_local_setb);
				}
				// Tecla 2 (PS/2 NumLock ON (Default)):
				y_local = (*(base_of_database+linhavarrida*DB_NUM_COLS+5) & (uint8_t)0xF0) >> 4;
				// Verifica se está mapeada
				if (y_local != y_dummy)
				{
					put_msx_disp_keys_queue_buffer(*(base_of_database+linhavarrida*DB_NUM_COLS+5));
				}
			}
			else //if (ps2numlockstate ^ shiftstate)
			{
				//numlock e Shift tem mesmo estado
				// Verifica se ha teclas mapeadas
				// Tecla 1 (PS/2 NumLock OFF):
				y_local = (*(base_of_database+linhavarrida*DB_NUM_COLS+6) & (uint8_t)0xF0)  >> 4;
				// Verifica se está mapeada
				if (y_local != y_dummy)
				{
					x_local = *(base_of_database+linhavarrida*DB_NUM_COLS+6) & (uint8_t)0x07;
					x_local_setb = ((*(base_of_database+linhavarrida*DB_NUM_COLS+6) & (uint8_t)PRESS_RELEASE_BIT) >> PRESS_RELEASE_BIT_POSITION) == (uint8_t)1;
					// Calcula x_bits da Tecla 1 e verifica se o tempo em que foi atualizado o dado de Linha X da Coluna Y,
					// com a finalidade de atualizar teclas mesmo sem o PPI ser atualizado. 
					compute_x_bits_and_check_interrupt_stuck(y_local, x_local, x_local_setb);
				}
				// Tecla 2 (PS/2 NumLock OFF):
				y_local = (*(base_of_database+linhavarrida*DB_NUM_COLS+7) & (uint8_t)0xF0) >> (uint8_t)4;
				// Verifica se está mapeada
				if (y_local != y_dummy)
				{
					put_msx_disp_keys_queue_buffer(*(base_of_database+linhavarrida*DB_NUM_COLS+7));
				}
			} //if (ps2numlockstate ^ shiftstate)
			break;
		}  // .1 - Mapeamento NumLock (Colunas 6 e 7)

		case 2:
		{	// .2 - Mapeamento alternativo (PS/2 Left and Right Shift)  (Colunas 8 e 9)
			if (!shiftstate)
			{
				// Tecla 1 (PS/2 NumLock ON (Default)):
				y_local = (*(base_of_database+linhavarrida*DB_NUM_COLS+4) & (uint8_t)0xF0)  >> 4;
				// Verifica se está mapeada
				if (y_local != y_dummy)
				{
					x_local = *(base_of_database+linhavarrida*DB_NUM_COLS+4) & (uint8_t)0x07;
					x_local_setb = ((*(base_of_database+linhavarrida*DB_NUM_COLS+4) & (uint8_t)PRESS_RELEASE_BIT) >> PRESS_RELEASE_BIT_POSITION) == (uint8_t)1;
					// Calcula x_bits da Tecla 1 e verifica se o tempo em que foi atualizado o dado de Linha X da Coluna Y,
					// com a finalidade de atualizar teclas mesmo sem o PPI ser atualizado. 
					compute_x_bits_and_check_interrupt_stuck(y_local, x_local, x_local_setb);
				}
				// Tecla 2 (PS/2 NumLock ON (Default)):
				y_local = (*(base_of_database+linhavarrida*DB_NUM_COLS+5) & (uint8_t)0xF0) >> 4;
				// Verifica se está mapeada
				if (y_local != y_dummy)
				{
					put_msx_disp_keys_queue_buffer(*(base_of_database+linhavarrida*DB_NUM_COLS+5));
				}
			}
			else //if (!shiftstate) => now, after this else, shiftstate is true (ON)
			{
				// Verifica se ha teclas mapeadas
				// Tecla 1 (PS/2 Left and Right Shift):
				y_local = (*(base_of_database+linhavarrida*DB_NUM_COLS+8) & (uint8_t)0xF0) >> 4;
				// Verifica se está mapeada
				if (y_local != y_dummy)
				{
					if (*(base_of_database+linhavarrida*DB_NUM_COLS+8) == (uint8_t)0x64) //if CODE key is to be pressed
					{
						x_bits[Y_SHIFT] = (x_bits[Y_SHIFT] | X0_SET_OR) & X0_SET_AND;		//then Release MSX Shift key
						//verify if the Y interrupts are stucked 
						if (systicks - previous_y_systick[Y_SHIFT] > MAX_TIME_OF_IDLE_KEYSCAN_SYSTICKS)
						{
							//MSX is not updating Y, so updating keystrokes by interrupts is not working
							//Verify the actual hardware Y_SCAN
							// First I have to disable Y_SCAN interrupts, to avoid misspelling due to updates
							exti_disable_request(Y3_exti | Y2_exti | Y1_exti | Y0_exti);
							// Read the MSX keyboard Y scan through GPIO pins A5:A8, mask to 0 other bits and rotate right 5
							msx_Y_scan = (gpio_port_read(Y0_port) & Y_MASK) >> 5;
							if (msx_Y_scan == Y_SHIFT)
							{
								GPIOB_BSRR = x_bits[Y_SHIFT]; //Atomic GPIOB update => Release and press MSX keys for this column
								//update time marker for previous_y_systick[Y_SHIFT]
								previous_y_systick[Y_SHIFT] = systicks;
							}
							//Than reenable Y_SCAN interrupts
							exti_enable_request(Y0_exti | Y1_exti | Y2_exti | Y3_exti);
						}
					}  //if (*(base_of_database+linhavarrida*DB_NUM_COLS+8) == 0x64) //if CODE key pressed
					if (*(base_of_database+linhavarrida*DB_NUM_COLS+8) == (uint8_t)0x6C) //if CODE key released
					{
						x_bits[Y_SHIFT] = (x_bits[Y_SHIFT] & X0_CLEAR_AND) | X0_CLEAR_OR;		//then Press MSX Shift key
						if (systicks - previous_y_systick[Y_SHIFT] > MAX_TIME_OF_IDLE_KEYSCAN_SYSTICKS)
						{
							//MSX is not updating Y, so updating keystrokes by interrupts is not working
							//Verify the actual hardware Y_SCAN
							// First I have to disable Y_SCAN interrupts, to avoid misspelling due to updates
							exti_disable_request(Y3_exti | Y2_exti | Y1_exti | Y0_exti);
							// Read the MSX keyboard Y scan through GPIO pins A5:A8, mask to 0 other bits and rotate right 5
							msx_Y_scan = (gpio_port_read(Y0_port) & Y_MASK) >> 5;
							if (msx_Y_scan == Y_SHIFT)
							{
								GPIOB_BSRR = x_bits[Y_SHIFT];
								//update time marker for previous_y_systick[Y_SHIFT]
								previous_y_systick[Y_SHIFT] = systicks;
							}
							//Than reenable Y_SCAN interrupts
							exti_enable_request(Y0_exti | Y1_exti | Y2_exti | Y3_exti);
						}
					}	//if (*(base_of_database+linhavarrida*DB_NUM_COLS+8) == 0x6C) //if CODE key released
					//Process Code key 
					put_msx_disp_keys_queue_buffer(*(base_of_database+linhavarrida*DB_NUM_COLS+8));
				}
				// Tecla 2 (PS/2 Left and Right Shift):
				y_local = (*(base_of_database+linhavarrida*DB_NUM_COLS+9) & (uint8_t)0xF0) >> 4;
				// Verifica se está mapeada
				if (y_local != y_dummy)
				{
					put_msx_disp_keys_queue_buffer(*(base_of_database+linhavarrida*DB_NUM_COLS+9));
				}
			} //if (shiftstate)
		}  // .2 - Mapeamento alternativo  (PS/2 Left  and Right Shift)  (Colunas 8 e 9)
	}
}

void msxmap::compute_x_bits_and_check_interrupt_stuck (
	volatile uint8_t y_local, uint8_t x_local, bool x_local_setb)
{
	uint16_t msx_Y_scan;
	switch (x_local)
	{
		case 0:
		{
			if (x_local_setb)	//key release
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] | X0_SET_OR) & X0_SET_AND;
 				break;
			}
			else 							//keypress
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] & X0_CLEAR_AND) | X0_CLEAR_OR;
				break;
			}
		}
		case 1:
		{
			if (x_local_setb)	//key release
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] | X1_SET_OR) & X1_SET_AND;
				break;
			}
			else							//keypress
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] & X1_CLEAR_AND) | X1_CLEAR_OR;
				break;
			}
		}
		case 2:
		{
			if (x_local_setb)
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] | X2_SET_OR) & X2_SET_AND;
				break;
			}
			else
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] & X2_CLEAR_AND) | X2_CLEAR_OR;
				break;
			}
		}
		case 3:
		{
			if (x_local_setb)
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] | X3_SET_OR) & X3_SET_AND;
				break;
			}
			else
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] & X3_CLEAR_AND) | X3_CLEAR_OR;
				break;
			}
		}
		case 4:
		{
			if (x_local_setb)
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] | X4_SET_OR) & X4_SET_AND;
				break;
			}
			else
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] & X4_CLEAR_AND) | X4_CLEAR_OR;
				break;
			}
		}
		case 5:
		{
			if (x_local_setb)
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] | X5_SET_OR) & X5_SET_AND;
				break;
			}
			else
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] & X5_CLEAR_AND) | X5_CLEAR_OR;
				break;
			}
		}
		case 6:
		{
			if (x_local_setb)
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] | X6_SET_OR) & X6_SET_AND;
				break;
			}
			else
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] & X6_CLEAR_AND) | X6_CLEAR_OR;
				break;
			}
		}
		case 7:
		{
			if (x_local_setb)
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] | X7_SET_OR) & X7_SET_AND;
				break;
			}
			else
			{
				x_bits[Y_XLAT_TABLE[y_local]] = (x_bits[Y_XLAT_TABLE[y_local]] & X7_CLEAR_AND) | X7_CLEAR_OR;
				break;
			}
		}
	}

	//ver se o tempo em que foi atualizado o dado de Linha X da Coluna Y, com a finalidade de atualizar teclas mesmo sem o PPI ser atualizado. 
	if (systicks - previous_y_systick[y_local] > MAX_TIME_OF_IDLE_KEYSCAN_SYSTICKS)
	{
		//MSX is not updating Y, so updating keystrokes by interrupts is not working
		//Verify the actual hardware Y_SCAN
		// First I have to disable Y_SCAN interrupts, to avoid misspelling due to updates
		exti_disable_request(Y3_exti | Y2_exti | Y1_exti | Y0_exti);
		// Read the MSX keyboard Y scan through G5PIO pins A5:A8, mask to 0 other bits and rotate right 5
		msx_Y_scan = (gpio_port_read(Y0_port) & Y_MASK) >> 5;
		
		GPIO_BSRR(X_port) = x_bits[msx_Y_scan]; //Atomic GPIOB update => Release and press MSX keys for this column
		
		//Than reenable Y_SCAN interrupts
		exti_enable_request(Y0_exti | Y1_exti | Y2_exti | Y3_exti);
	}
}


/*************************************************************************************************/
/*************************************************************************************************/
/******************************************* ISR's ***********************************************/
/*************************************************************************************************/
/*************************************************************************************************/
void exti9_5_isr(void) // PC3, PC2, PC1 and PC0 - This ISR works like interrupt on change of each one of Y connected pins
{
	volatile uint16_t msx_Y_scan;

	//Debug & performance measurement
	//gpio_clear(Dbg_Yint_port, Dbg_Yint0e1_pin_id); //Signs start of interruption. This line is useful only to measure performance, ie, only in development phase
	GPIO_BSRR(Dbg_Yint_port) = Dbg_Yint_pin_id << 16;

	// Read the MSX keyboard Y scan through GPIO pins A5:A8, mask to 0 other bits and rotate right 5
	//msx_Y_scan = Y_XLAT_TABLE[(gpio_port_read(Y0_port) & Y_MASK) >> 5];
	msx_Y_scan = (gpio_port_read(Y0_port) & Y_MASK) >> 5;

 	GPIO_BSRR(X_port) = x_bits[msx_Y_scan]; //Atomic GPIOB update => Release and press MSX keys for this column. This ends time criticity.

	//Debug & performance measurement
	//gpio_set(Dbg_Yint_port, Dbg_Yint_pin_id); //Signs end of interruption. Default condition is "1". This line is useful only to measure performance, ie, only in development phase
	GPIO_BSRR(Dbg_Yint_port) = Dbg_Yint_pin_id;

	// Clear interrupt Y Scan flags, including that not used on this ISR
	// if(exti_get_flag_status(EXTI9), (EXTI6), (EXTI4) and (EXTI3))
	exti_reset_request(Y0_exti | Y1_exti | Y2_exti | Y3_exti);
    
	//Update systicks (time stamp) for this Y
	previous_y_systick[msx_Y_scan]  = systicks;
}
