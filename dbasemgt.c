/*
 * This file was part of the libopencm3 project, but highly modifyed.
 *
 * Copyright (C) 2021 Evandro Souza <evandro.r.souza@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

//Use Tab width=2

//This original SW is compiled to a Sharp/Epcom MSX HB-8000 and a brazilian ABNT2 PS/2 keyboard (ID=275)
//But it is possible to update the table sending a Intel Hex File through serial, using the
//following communication configuration:
//Speed: 115200 bps
//8 bits
//No parity
//1 Stop bit
//no hardware flow control


#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "dbasemgt.h"
#include "database.c"
#include "port_def.h"
#include "serial.h"


//Processor related sizes and adress:
#define USART_ECHO_EN 						1	//All chars received in Intel Hex are echoed in serial routines
#define STRING_MOUNT_BUFFER_SIZE	128
#define MAX_ERASE_TRIES						3
#define FLASH_WRONG_DATA_WRITTEN 	0x80
#define RESULT_OK 								0

//Ref.: RM0368 Rev 5 manual, item 3.8.2, Flash key register (FLASH_KEYR) - Page 61/847..
//Some "stranger thing" occurred here in libopencm3, as it linked wrong definitions.
#define FPEC_KEY1									((uint32_t)0x45670123)
#define FPEC_KEY2									((uint32_t)0xCDEF89AB)


//Prototype area:
//Usart operations
void usart_get_string_line(uint8_t*, uint16_t);
//Intel Hex operations
void get_valid_intelhex_file(uint8_t*, uint16_t, uint8_t*);
void usart_get_intel_hex(uint8_t*, uint16_t, uint8_t*);
bool validate_intel_hex_record(uint8_t*, uint8_t*, uint8_t*, uint16_t*, uint8_t*);
//flash operations
uint32_t flash_program_data(uint8_t *flash_buffer_ram);
void check_flash_error(void);
void check_flash_locked(void);
bool check_sector3_erased(bool*, uint16_t*);
void cleanupFlash(bool*, uint16_t*);
//bool check_sector5_erased(bool*, uint16_t*);


//Global var area:
bool error_intel_hex;
bool abort_intelhex_reception;
bool compatible_database;
extern uint8_t UNUSED_DATABASE[DB_NUM_COLS];	//Declared on msxmap.cpp
extern uint32_t *base_of_database;						//Declared on msxmap.cpp
extern volatile bool update_ps2_leds;					//Declared on msxmap.cpp
extern uint8_t y_dummy;												//Declared on msxmap.cpp
extern bool ps2numlockstate;									//Declared on ps2handl.c
extern bool enable_xon_xoff;									//Declared on serial.c


void database_setup(void)
{
	uint8_t flash_buffer_ram[DATABASE_SIZE]; 			//Local variable, in aim to not consume resources in the main functional machine
	uint8_t ch, str_mount[STRING_MOUNT_BUFFER_SIZE];
	uint32_t iter;
	volatile uint32_t*base_of_database32;
	volatile uint8_t*base_of_database8;
	void *void_ptr;
	bool sector_erased = true;
	uint16_t attempts_erasing_sector;

	compatible_database = true;
	
	//The user wants to reset Database to system's defaults
	//Firstly verify if some character was sent to USART during BAT waiting
	if (serial_available_get_char())
	{
		//Clean RX serial buffer
		while (serial_available_get_char())
			ch = serial_get_char();
		//USER_KEY is exclusive of WeAct board
		if (!gpio_get(USER_KEY_PORT, USER_KEY_PIN_ID))
		{
			usart_send_string((uint8_t*)"\r\n\nOk. Now release user key...");
			while (!gpio_get(USER_KEY_PORT, USER_KEY_PIN_ID))	//But stay here until the button is released
				__asm("NOP");
		}
		usart_send_string((uint8_t*)"\r\nCleanup Database Flash. Press ""&"" to proceed or any other key to abort");
		//Read a key
		while (!serial_available_get_char())
			__asm("nop");
		ch = serial_get_char();
		if(ch == '&')
			cleanupFlash(&sector_erased, &attempts_erasing_sector);
	}
	if (!gpio_get(USER_KEY_PORT, USER_KEY_PIN_ID))
	{
		//USER_KEY is pressed: Perform Flash clean
		usart_send_string((uint8_t*)"\r\n\nOk. Now release user key...");
		while (!gpio_get(USER_KEY_PORT, USER_KEY_PIN_ID))	//But stay here until the button is released
			__asm("NOP");
		usart_send_string((uint8_t*)"\r\nCleanup Database Flash. Press ""&"" to proceed or any other key to abort");
		//Read a key
		while (!serial_available_get_char())
			__asm("nop");
		ch = serial_get_char();
		if(ch == '&')
			cleanupFlash(&sector_erased, &attempts_erasing_sector);
		else
			usart_send_string((uint8_t*)"\r\n\n");

	}	//if (!gpio_get(USER_KEY_PORT, USER_KEY_PIN_ID))

	// if INITIAL_DATABASE is unprogrammed
	bool empty_database = true;
	base_of_database32 = (uint32_t *)INITIAL_DATABASE;
	for(iter = 0; iter < (DATABASE_SIZE / sizeof(uint32_t)); iter++)
	{
		if(*(base_of_database32 + iter) != 0xFFFFFFFF)
		{
			empty_database = false;
			break;	//quit: for(iter = 0; iter < DATABASE_SIZE; iter += 4)
		}
	}

	//If it is unprogrammed, fill that area with the contents of DEFAULT_MSX_KEYB_DATABASE_CONVERSION
	if (empty_database)
	{
		//Database is empty. First copy DEFAULT_MSX_KEYB_DATABASE_CONVERSION to RAM
		base_of_database8 = (uint8_t*)&DEFAULT_MSX_KEYB_DATABASE_CONVERSION[0][0];
		for(iter = 0; iter < DATABASE_SIZE; iter ++)
		{
				ch = *(base_of_database8 + iter);
				flash_buffer_ram[iter] = ch;
		}

		//Then put it on INITIAL_DATABASE place => programming flash memory
		base_of_database = (uint32_t*)INITIAL_DATABASE;
		//Information to user
		usart_send_string((uint8_t*)"\r\nInitializing Default Database on flash memory...");
		usart_send_string((uint8_t*)"\r\n\nBase Address  Size\r\n");
		uintptr_t base_of_database_num = (uintptr_t)base_of_database;	//Convert from pointer to integer
		void_ptr = &str_mount;
		conv_uint32_to_8a_hex(((uint32_t)base_of_database_num), void_ptr);
		usart_send_string((uint8_t*)" 0x");
		usart_send_string((uint8_t*)str_mount);
		usart_send_string((uint8_t*)"   ");
		conv_uint32_to_dec((uint32_t)DATABASE_SIZE, void_ptr);
		usart_send_string((uint8_t*)str_mount);
		wait_tx_ends();
		check_flash_locked();
		flash_program((uint32_t)base_of_database_num, flash_buffer_ram, DATABASE_SIZE);
		check_flash_error();

		usart_send_string((uint8_t*)"\r\n\nVerification of written data:\r\n");
		//verify if correct data was written
		for (iter = 0; iter < DATABASE_SIZE; iter+=4)	//DATABASE_SIZE must be checked
		{
			uint32_t iter_div4 = iter / sizeof(uint32_t);// Pointer of uint32_t has a step of 4 bytes
			if(	*(base_of_database + iter_div4) != *((uint32_t*)(flash_buffer_ram + iter)) )
			{
				usart_send_string((uint8_t*)"Wrong data written into flash memory\r\n");
				usart_send_string((uint8_t*)"Dest add => 0x");
				void_ptr = &str_mount;
				conv_uint32_to_8a_hex((uintptr_t)(base_of_database + iter_div4), void_ptr);
				usart_send_string((uint8_t*)str_mount);
				usart_send_string((uint8_t*)", found = 0x");
				conv_uint32_to_8a_hex(*(base_of_database + iter_div4), void_ptr);
				usart_send_string((uint8_t*)str_mount);
				usart_send_string((uint8_t*)". Source add => 0x");
				conv_uint32_to_8a_hex((uintptr_t)(flash_buffer_ram + iter), void_ptr);
				usart_send_string((uint8_t*)str_mount);
				usart_send_string((uint8_t*)", was = 0x");
				conv_uint32_to_8a_hex(*((uint32_t*)(flash_buffer_ram + iter)), void_ptr);
				usart_send_string((uint8_t*)str_mount);
				usart_send_string((uint8_t*)"\r\nLocked!");
				return;
			}
		} //for (iter = 0; iter < DATABASE_SIZE; iter+=4)	//3K must be checked
	}	//if (empty_database)

	//Searching a valid (useful) Database. Unused databases are marked as UNUSED_DATABASE
	volatile uint32_t displacement = 0;
	base_of_database8 = (uint8_t*)((uint32_t)INITIAL_DATABASE - displacement);

	while(((uint32_t)INITIAL_DATABASE - displacement) >= (uint32_t)FLASH_SECTOR3_BASE)
	{
		//Searching a valid (useful) Database. Unused databases are marked as UNUSED_DATABASE
		if(
		*(base_of_database8+0) == UNUSED_DATABASE[0] &&
		*(base_of_database8+1) == UNUSED_DATABASE[1] &&
		*(base_of_database8+2) == UNUSED_DATABASE[2] &&
		*(base_of_database8+3) == UNUSED_DATABASE[3] &&
		*(base_of_database8+4) == UNUSED_DATABASE[4] &&
		*(base_of_database8+5) == UNUSED_DATABASE[5] &&
		*(base_of_database8+6) == UNUSED_DATABASE[5] &&
		*(base_of_database8+7) == UNUSED_DATABASE[7] &&
		*(base_of_database8+8) == UNUSED_DATABASE[8] &&
		*(base_of_database8+9) == UNUSED_DATABASE[9] )
		{
			displacement += DATABASE_SIZE;
			base_of_database8 = (uint8_t*)((uint32_t)INITIAL_DATABASE - displacement);
		}
		else
			break;	//if it is here, it found a valid Database
	}

	//Check Database consistensy (CheckSum & BCC of the first 319 blocks of 10 bytes each)
	uint8_t checksum = 0, bcc = 0;	//bcc is a vertical parity
	for (iter = 0; iter < (DATABASE_SIZE - DB_NUM_COLS); iter ++)
	{
		checksum += *(base_of_database8 + iter);
		bcc ^= *(base_of_database8 + iter);
	}	//for (iter = 0; iter < (DATABASE_SIZE - DB_NUM_COLS); iter ++)
	if( ((*(base_of_database8 + (DATABASE_SIZE - 1))) != checksum)||
			((*(base_of_database8 + (DATABASE_SIZE - 2))) != bcc)			||
			 (*(base_of_database8 + 0) != 1) 													||
			 (*(base_of_database8 + 1) != 0) )
	{
		void_ptr = &str_mount;
		//Display bcc
		wait_tx_ends();
		usart_send_string((uint8_t*)"\r\n\nAddress: 0x");
		conv_uint32_to_8a_hex((uintptr_t)(base_of_database8 + (DATABASE_SIZE - 2)), void_ptr);
		usart_send_string((uint8_t*)str_mount);
		usart_send_string((uint8_t*)": computed BCC = 0x");
		conv_uint8_to_2a_hex(bcc, void_ptr);
		usart_send_string((uint8_t*)str_mount);
		usart_send_string((uint8_t*)", but found 0x");
		conv_uint8_to_2a_hex(*(base_of_database8 + (DATABASE_SIZE - 2)), void_ptr);
		usart_send_string((uint8_t*)str_mount);
		//Display checksum
		usart_send_string((uint8_t*)"\r\nAddress: 0x");
		conv_uint32_to_8a_hex((uintptr_t)(base_of_database8 + (DATABASE_SIZE - 1)), void_ptr);
		usart_send_string((uint8_t*)str_mount);
		usart_send_string((uint8_t*)": computed CheckSum = 0x");
		conv_uint8_to_2a_hex(checksum, void_ptr);
		usart_send_string((uint8_t*)str_mount);
		usart_send_string((uint8_t*)", but found 0x");
		conv_uint8_to_2a_hex(*(base_of_database8 + (DATABASE_SIZE - 1)), void_ptr);
		usart_send_string((uint8_t*)str_mount);
		//Display version
		usart_send_string((uint8_t*)"\r\nDatabase version ");
		conv_uint32_to_dec((uint32_t)(*(base_of_database8 + 0)), void_ptr);
		usart_send_string((uint8_t*)str_mount);
		usart_send_string((uint8_t*)".");
		conv_uint32_to_dec((uint32_t)(*(base_of_database8 + 1)), void_ptr);
		usart_send_string((uint8_t*)str_mount);
		usart_send_string((uint8_t*)"\r\n\n!!!Attention!!! => No valid Database found. Please update it!\r\n\n");
		compatible_database = false;
		wait_tx_ends();
	}
	else
	{
		y_dummy 				=  *(base_of_database8 + 3) & 0x0F; //Low nibble (no keys at this column)
		ps2numlockstate = (*(base_of_database8 + 3) & 0x10) != 0; //Bit 4
		enable_xon_xoff = (*(base_of_database8 + 3) & 0x20) != 0; //Bit 5
		update_ps2_leds = true;
		base_of_database = (uint32_t*)base_of_database8;
	}
	flash_lock();
}


void cleanupFlash(bool *sector_erased, uint16_t *attempts_erasing_sector)
{
	uint8_t str_mount[STRING_MOUNT_BUFFER_SIZE];

	//0x800C0000 (Sector 3)
	*attempts_erasing_sector = 0;
	while (*attempts_erasing_sector < MAX_ERASE_TRIES) //3 tries
	{
		//Information to user
		usart_send_string((uint8_t*)"\r\nErasing flash memory...\r\nBase Address  Size  Status\r\n 0x");
		conv_uint32_to_8a_hex(((uint32_t)FLASH_SECTOR3_BASE), (uint8_t*)&(str_mount));
		usart_send_string((uint8_t*)str_mount);
		usart_send_string((uint8_t*)"  ");
		conv_uint32_to_dec((uint32_t)(FLASH_SECTOR3_TOP - FLASH_SECTOR3_BASE + 1), (uint8_t*)&(str_mount));
		usart_send_string((uint8_t*)str_mount);
		usart_send_string((uint8_t*)" ");
		wait_tx_ends();
		//Erase sector FLASH_SECTOR3_NUMBER only if not erased
		for (uint32_t iter = 0; iter < (FLASH_SECTOR3_TOP + 1 - FLASH_SECTOR3_BASE); iter+=4)	//All sector bytes must be checked
		{
			if(	(*(uint32_t *)(FLASH_SECTOR3_BASE + iter)) != 0xFFFFFFFF )
			{
				//FLASH_SECTOR3_NUMBER is not erased
				check_flash_locked();
				flash_erase_sector(FLASH_SECTOR3_NUMBER, FLASH_CR_PROGRAM_X8); //program_size	0 (8-bit), 1 (16-bit), 2 (32-bit), 3 (64-bit)
				check_flash_error();
				break;	//quit: for(iter = 0; iter < DATABASE_SIZE; iter += 4)
			}
		}
		//Now confirm cleaning
		if(check_sector3_erased(sector_erased, attempts_erasing_sector))
			break;	// this break quits "while (attempts_erasing_sector < 3) //MAX_ERASE_TRIES tries"
		usart_send_string((uint8_t*)"\r\n");
	}	//0x800C0000 (Sector 3) while (*attempts_erasing_sector < MAX_ERASE_TRIES)
}	//if (!gpio_get(USER_KEY_PORT, USER_KEY_PIN_ID))


bool check_sector3_erased(bool *sect_erased, uint16_t *attempts_erasing_sector)
{
	//Now confirm cleaning
	*sect_erased = true;
	for (uint32_t iter = 0; iter < (FLASH_SECTOR3_TOP + 1 - FLASH_SECTOR3_BASE); iter+=4)	//All sector bytes must be checked
	{
		if(	(*(volatile uint32_t *)(FLASH_SECTOR3_BASE + iter)) != 0xFFFFFFFF )
		{
			(*attempts_erasing_sector)++;
			*sect_erased = false;
			//Information to user - continued
			if(*attempts_erasing_sector == 1)
				usart_send_string((uint8_t*)"Not OK at first attempt");
			if(*attempts_erasing_sector == 2)
				usart_send_string((uint8_t*)"Not OK at second attempt");
			if(*attempts_erasing_sector == MAX_ERASE_TRIES)
				usart_send_string((uint8_t*)"Not OK at third attempt\r\n");
			break;	//quit "for (iter = 0; iter < (FLASH_SECTOR3_TOP + 1 - FLASH_SECTOR3_BASE); iter += 4)"
		}	//if(	(*(uint32_t *)(FLASH_SECTOR3_BASE + iter)) != 0xFFFFFFFF )
	} //for (iter = 0; iter < (FLASH_SECTOR3_TOP + 1 - FLASH_SECTOR3_BASE); iter+=4)
	if(*sect_erased)
	{
		//Information to user - continued
		usart_send_string((uint8_t*)" Successful\r\n");
		//Flash Database zone cleared. Points to default (Initial) Database address
		base_of_database = (uint32_t*)((uint32_t)INITIAL_DATABASE);
	}	//if(!attempts_erasing_sector)
return *sect_erased;
}


void check_flash_locked(void)
{
	//Unlock FLASH_if it is locked
	while(FLASH_SR & FLASH_SR_BSY)	//FLASH_CR is read only when FLASH_SR_BSY is set
		__asm("nop");									//To avoid compiler optimizations
	if(FLASH_CR & FLASH_CR_LOCK)
	{
		//flash unlock => used to allow access to the Flash control register and so,
		//to allow program and erase operations.
		FLASH_KEYR = FPEC_KEY1;
		FLASH_KEYR = FPEC_KEY2;
	}
}


void check_flash_error(void)
{
	//Read FLASH_SR (Flash status register), searching for errors
	if(FLASH_SR & (1 << 14))	//RDERR (1 << 14): Read Protection Error (pcrop)
	{
		usart_send_string((uint8_t*)"RDERR: Read Protection Error (pcrop)\r\n");
		FLASH_SR |= (1 << 14);	//Cleared by writing 1.
	}
	if(FLASH_SR & FLASH_SR_PGSERR)	//PGSERR: Programming sequence error
	{
		usart_send_string((uint8_t*)"PGSERR: Programming sequence error\r\n");
		FLASH_SR |= FLASH_SR_PGSERR;	//Cleared by writing 1.
	}
	if(FLASH_SR & FLASH_SR_PGAERR)	//PGAERR: Programming alignment error
	{
		usart_send_string((uint8_t*)"PGAERR: Programming alignment error\r\n");
		FLASH_SR |= FLASH_SR_PGAERR;	//Cleared by writing 1.
	}
	if(FLASH_SR & FLASH_SR_WRPERR)//WRPERR: Write protection error
	{
		usart_send_string((uint8_t*)"WRPERR: Write protection error\r\n");
		FLASH_SR |= FLASH_SR_WRPERR;	//Cleared by writing 1.
	}
	if(FLASH_SR & FLASH_SR_OPERR)	//OPERR: Operation error. This bit is set only if error interrupts are enabled (ERRIE = 1).
	{
		usart_send_string((uint8_t*)"OPERR: Operation error\r\n");
		FLASH_SR |= FLASH_SR_OPERR;	//Cleared by writing 1.
	}
	wait_tx_ends();
}


void usart_get_string_line(uint8_t *ser_inp_line, uint16_t str_max_size)
{ //Reads until CR and returns an ASCIIZ on *ser_inp_line
	uint8_t sign = 0;
	uint16_t iter = 0;

redohere:
	while(iter < str_max_size)
	{
		//wait until next char is available
		while (!serial_available_get_char())
			__asm("nop");
		sign = serial_get_char();

#if USART_ECHO_EN == 1
		if (sign == 3)
		{
			while (!serial_put_char('^'))
				__asm("nop");
			while (!serial_put_char('C'))
				__asm("nop");
		}
		else if (sign != '\r')  //if sign == '\r' do nothing
		{
			if (sign == '\n')
			{
				while (!serial_put_char('\r'))
					__asm("nop");
				while (!serial_put_char('\n'))
					__asm("nop");
			}
			else
				while (!serial_put_char(sign))
					__asm("nop");
		}
#endif

		if(sign == 3)
			abort_intelhex_reception = true;
		else
		{
			//only accepts char between '0' ~ ':', 'A' ~ 'F', 'a' ~ 'f'
			if( ( ((sign > ('/')) && (sign < (':'+1))) ||
					(((sign&0x5F) > ('A'-1)) && ((sign&0x5F) < ('G'))))	)
				//assembling the record
				ser_inp_line[iter++] = sign;
			else if( (iter > 10) && ((sign == '\n') || (sign == '\r')) )	//	'\r' or '\n'
			{	//End of record. Close the ASCIIZ.
				ser_inp_line[iter++] = 0;
				break;
			}
			else
			{	
				//The minimum valid record is 11 characters, so redo.
				iter = 0;
				goto redohere;
			}
		}
	} //while(iter < str_max_size)
	// Indicates that we received new line data, and it is going to be shiftingalidated.
	gpio_toggle(GPIOC, GPIO13);	//Toggle LED each received line
}


void get_valid_intelhex_file(uint8_t *ser_inp_line, uint16_t str_max_size, uint8_t *ram_buffer)
{
	error_intel_hex = true;
	abort_intelhex_reception = false;
	while (error_intel_hex == true)
	{
		usart_get_intel_hex(ser_inp_line, str_max_size, ram_buffer);
		if (error_intel_hex == true)
		{
			usart_send_string((uint8_t*)"\r\n\n\n\nERROR in Intel Hex. ERROR\r\n\nPlease resend the Intel Hex...");
		}
		else if (abort_intelhex_reception)
		{
			usart_send_string((uint8_t*)"\r\n\n\n\n!!!!!INTERRUPTED!!!!!\r\n\nPlease resend the Intel Hex...");
		}
		else
			return;
	}
}


void usart_get_intel_hex(uint8_t *ser_inp_line, uint16_t str_max_size, uint8_t *ram_buffer)
{
	uint8_t intel_hex_localreg_data[STRING_MOUNT_BUFFER_SIZE / 2], intel_hex_numofdatabytes, intel_hex_type;
	uint16_t i, count_rx_intelhex_bytes, count_IHdata_record, intel_hex_address, first_data_address_intel_hex;
	uint16_t shifting;
	bool seek_first_intel_hex_address_for_data = true;
	uint8_t str_mount[STRING_MOUNT_BUFFER_SIZE];
	
	intel_hex_numofdatabytes = 0; //Force init of uint8_t here
	intel_hex_type = 0; //Force init of uint8_t here
	count_rx_intelhex_bytes = 0; //Force init of uint16_t here
	count_IHdata_record = 0;
	intel_hex_address = 0; //Force init of uint16_t here
	first_data_address_intel_hex = 0; //Force init of uint16_t here
	error_intel_hex = false;
	seek_first_intel_hex_address_for_data = true;

	//Init RAM Buffer
	for(i=0 ; i < (DATABASE_SIZE) ; i++)
	{
		ram_buffer[i] = (uint8_t)0xFF;
	}

	while (intel_hex_type != 1) //Run until intel_hex_type == 1: means end-of-file record
	{
		usart_get_string_line(ser_inp_line, str_max_size);
		if (!abort_intelhex_reception)
		{
			//Is it a valid Intel Hex record?
			if (validate_intel_hex_record(ser_inp_line, &intel_hex_numofdatabytes, &intel_hex_type,
					&intel_hex_address, &intel_hex_localreg_data[0]))
			{
				//":llaaaatt[dd...]cc"	tt is the field that represents the HEX record type, which may be one of the following:
				//											00 - data record
				//											01 - end-of-file record
				//											02 - extended segment address record
				//											04 - extended linear address record
				//											05 - start linear address record (MDK-ARM only)

				switch(intel_hex_type)
				{
					case 0:  //00 - data record
					{
						count_IHdata_record ++;
						if (seek_first_intel_hex_address_for_data)
						{
							first_data_address_intel_hex = intel_hex_address;
							seek_first_intel_hex_address_for_data = false;
						}
						//Now fill in RAM Buffer with data record
						for (i = 0; i < intel_hex_numofdatabytes; i++)
						{
							shifting = ((uint16_t)intel_hex_address - (uint16_t)first_data_address_intel_hex);
							ram_buffer[shifting + i] = intel_hex_localreg_data[i];
							count_rx_intelhex_bytes++;
						}
						break;
					}	//case 0:  //00 - data record
					case 1: //01 - end-of-file record
					{
						//Information to user
						usart_send_string((uint8_t*)"\r\n\nReceive concluded. It has been received ");
						conv_uint32_to_dec((uint32_t)(count_IHdata_record), (uint8_t*)&(str_mount));
						usart_send_string((uint8_t*)str_mount);
						usart_send_string((uint8_t*)" IntelHex data records,\r\nwith an amount of ");
						conv_uint32_to_dec((uint32_t)(count_rx_intelhex_bytes), (uint8_t*)&(str_mount));
						usart_send_string((uint8_t*)str_mount);
						usart_send_string((uint8_t*)" data bytes.");
						/*// Wait user knowledge
						while(serial_available_get_char())
							bin = serial_get_char();
						usart_send_string((uint8_t*)" data bytes.\r\nPress any key to conclude...");
						while(!serial_available_get_char())
							__asm("nop");
						bin = serial_get_char();
						bin &= 0xFF;	//Only to avoid "warning: variable 'bin' set but not used"*/
						return;
						break;
					}
					case 2:  //02 - extended segment address record
					{
						__asm("nop"); //not useful in this code
						break;
					}
					case 4: //04 - extended linear address record
					{
						if (intel_hex_numofdatabytes != 2) //intel_hex_numofdatabytes must be 2;
						{
							error_intel_hex = true;
						}
						if (intel_hex_localreg_data[0] != 0x08) //First data must be 0x08
						{
							error_intel_hex = true;
						}
						break;
					} //case 4: //04 - extended linear address record
					case 5:  //05 - start linear address record (MDK-ARM only)
					{
						__asm("nop"); //not useful in this code
						break;
					}
				}
			}
			else	//if (validate_intel_hex_record(ser_inp_line, intel_hex_numofdatabytes, intel_hex_type, intel_hex_address, intel_hex_localreg_data))
			{
				//Invalid record received
				usart_send_string((uint8_t*)" Error: Invalid");
				error_intel_hex = true;
			}
		} //if (validate_intel_hex_record(ser_inp_line, &intel_hex_numofdatabytes, &intel_hex_type, &intel_hex_address, &intel_hex_localreg_data[0]))
	} //while (intel_hex_type != 0) //Run until case 1: means end-of-file record
}


bool validate_intel_hex_record(uint8_t *ser_inp_line, uint8_t *intel_hex_numofdatabytes,
		 uint8_t *intel_hex_type, uint16_t *intel_hex_address, uint8_t *intel_hex_localreg_data)
{
	uint8_t checksum_read, checksum_calc;
	int16_t i;

	//Record Intel Hex ":llaaaatt[dd...]cc"

	//":llaaaatt[dd...]cc" : is the colon that starts every Intel HEX record
	if (ser_inp_line[0] != ':')
	{
		//Error: Not started with ":"
		error_intel_hex = true;
		return false;
	}
	
	//":llaaaatt[dd...]cc" ll is the record-length field that represents the number of data bytes (dd) in the record
	*intel_hex_numofdatabytes = conv_2a_hex_to_uint8(ser_inp_line, 1);

	//":llaaaatt[dd...]cc" aaaa is the address field that represents the starting address for subsequent data in the record
	*intel_hex_address = (conv_2a_hex_to_uint8(ser_inp_line, 3) << 8) + (conv_2a_hex_to_uint8(ser_inp_line, 5));
	
	//":llaaaatt[dd...]cc"	tt is the field that represents the HEX record type, which may be one of the following:
	//											00 - data record
	//											01 - end-of-file record
	//											02 - extended segment address record
	//											04 - extended linear address record
	//											05 - start linear address record (MDK-ARM only)
	*intel_hex_type = conv_2a_hex_to_uint8(ser_inp_line, 7);
	
	//":llaaaatt[dd...]cc"	dd is a data field that represents one byte of data. A record may have multiple data bytes. The number of data bytes in the record must match the number specified by the ll field
	if (*intel_hex_numofdatabytes > 0)
	{
		for(i = 0; i < *intel_hex_numofdatabytes; i ++) //2 ASCII bytes per each byte of data in the record
		{
			intel_hex_localreg_data[i] = conv_2a_hex_to_uint8(ser_inp_line, 2 * i + 9);
		}
	}

	//":llaaaatt[dd...]cc"	cc is the checksum field that represents the checksum of the record. The checksum is calculated by summing the values of all hexadecimal digit pairs in the record modulo 256 and taking the two's complement
	checksum_read = conv_2a_hex_to_uint8(ser_inp_line, 2 * (*intel_hex_numofdatabytes) + 9);
	//Now compute Checksum
	checksum_calc = *intel_hex_numofdatabytes; // Checksum init
	for(i = 0; i < *intel_hex_numofdatabytes; i++)
		checksum_calc += intel_hex_localreg_data[i];
	checksum_calc += *intel_hex_type;
	checksum_calc += (uint8_t)((*intel_hex_address & 0xFF00) >> 8) + (uint8_t)(*intel_hex_address & 0x00FF);
	checksum_calc += checksum_read;
	//And check the computed Checksum
	if (checksum_calc != 0)
		error_intel_hex = true;
	return !error_intel_hex;
}


uint32_t flash_program_data(uint8_t *flash_buffer_ram)	//Local flash_buffer_ram declarated at flashF4_rw
//uint32_t flash_program_data(void)	//global flash_buffer_ram
{
	uint32_t iter;
	uint32_t displacement;
	bool DBaseSizeErasedPlaceFound = false, sector_erased = true;
	uint8_t str_mount[20];

	//verify:
	//1) If there is enough DATABASE_SIZE room in sector FLASH_SECTOR3_NUMBER to acomodate a new Database image
	displacement = 0;
	base_of_database = (uint32_t*)((uint32_t)INITIAL_DATABASE);
	//Information to user
	usart_send_string((uint8_t*)"\r\n\nSearching for an empty ");
	void *void_ptr = &str_mount;
	conv_uint32_to_dec((uint32_t)DATABASE_SIZE, void_ptr);
	usart_send_string((uint8_t*)str_mount);
	//usart_send_string((uint8_t*)" bytes in sector 3 of flash memory...\r\nBase Address  Size  Status\r\n");
	usart_send_string((uint8_t*)" bytes in sector ");
	conv_uint32_to_dec((uint32_t)FLASH_SECTOR3_NUMBER, void_ptr);
	usart_send_string((uint8_t*)str_mount);
	usart_send_string((uint8_t*)" of flash memory...\r\nBase Address  Size  Status\r\n");
	while( (((uint32_t)INITIAL_DATABASE - displacement) >= (uint32_t)FLASH_SECTOR3_BASE) && !DBaseSizeErasedPlaceFound)
	{
		//Information to user
		conv_uint32_to_8a_hex(((uint32_t)INITIAL_DATABASE - displacement), void_ptr);
		usart_send_string((uint8_t*)" 0x");
		usart_send_string((uint8_t*)str_mount);
		usart_send_string((uint8_t*)"    ");
		conv_uint32_to_dec((uint32_t)DATABASE_SIZE, void_ptr);
		usart_send_string((uint8_t*)str_mount);

		DBaseSizeErasedPlaceFound = true;
		for (iter = 0; iter < (DATABASE_SIZE/sizeof(uint32_t)); iter++)
		{
			//Searching for a DATABASE_SIZE room in the address range of flash sector FLASH_SECTOR3_NUMBER to acomodate a new Database image
			if(	*(base_of_database + iter) != 0xFFFFFFFF )
			{
				usart_send_string((uint8_t*)"  Not available\r\n");
				DBaseSizeErasedPlaceFound = false;
				displacement += DATABASE_SIZE;
				base_of_database = (uint32_t*)((uint32_t)INITIAL_DATABASE - displacement);
				break;	// this break quits 'for (iter; iter < DATABASE_SIZE; iter+=4)'
			}
		} //for (iter; iter < DATABASE_SIZE; iter+=4)
		if(iter >= (DATABASE_SIZE/sizeof(uint32_t)) && DBaseSizeErasedPlaceFound)
		{
			//DATABASE_SIZE page is free on "base_of_database" address
			usart_send_string((uint8_t*)"  Ok!\r\n");
		}
	}	//while( (((uint32_t)INITIAL_DATABASE - displacement) >= (uint32_t)FLASH_SECTOR3_BASE) && !DBaseSizeErasedPlaceFound)

	//2) If there is no room in sector FLASH_SECTOR3_NUMBER, then erase this sector
	if (!DBaseSizeErasedPlaceFound)
	{
		//3K free room was not found: Perform erase of sector FLASH_SECTOR3_NUMBER
		uint16_t attempts_erasing_sector = 0;
		cleanupFlash(&sector_erased, &attempts_erasing_sector);
	}	//if (!DBaseSizeErasedPlaceFound)
	
	//Programming Flash Memory
	//Information to user
	usart_send_string((uint8_t*)"\r\nProgramming flash memory...\r\nDest address  Size  Origin RAM address\r\n");
	uintptr_t base_of_database_num = (uintptr_t)base_of_database;	//Convert from pointer to integer
	void_ptr = &str_mount;
	conv_uint32_to_8a_hex(((uint32_t)base_of_database_num), void_ptr);
	usart_send_string((uint8_t*)" 0x");
	usart_send_string((uint8_t*)str_mount);
	usart_send_string((uint8_t*)"  ");
	conv_uint32_to_dec((uint32_t)DATABASE_SIZE, void_ptr);
	usart_send_string((uint8_t*)str_mount);
	usart_send_string((uint8_t*)"  0x");
	conv_uint32_to_8a_hex(((uintptr_t)flash_buffer_ram), void_ptr);
	usart_send_string((uint8_t*)str_mount);
	wait_tx_ends();
	check_flash_locked();
	flash_program((uint32_t)base_of_database_num, flash_buffer_ram, DATABASE_SIZE);
	check_flash_error();

	usart_send_string((uint8_t*)"\r\n\nVerification of written data:");
	//verify if correct data was written
	for (iter = 0; iter < (DATABASE_SIZE); iter += 4)	//All DATABASE_SIZE must be checked
	{
		uint32_t iter_div4 = iter / sizeof(uint32_t);// Pointer of uint32_t has a step of 4 bytes
		if(	*(base_of_database + iter_div4) != *((uint32_t*)(flash_buffer_ram + iter)) )
		{
			if(*(base_of_database + iter) == 0xFFFFFFFF)
			{
				usart_send_string((uint8_t*)"\r\n(RAM buffer address = 0x");
				conv_uint32_to_8a_hex((uintptr_t)(flash_buffer_ram + iter), void_ptr);
				usart_send_string((uint8_t*)str_mount);
				usart_send_string((uint8_t*)". Was = 0x");
				conv_uint32_to_8a_hex(*(base_of_database + iter_div4), void_ptr);
				usart_send_string((uint8_t*)str_mount);
				usart_send_string((uint8_t*)", while flash address 0x");
				conv_uint32_to_8a_hex((uintptr_t)(base_of_database + iter_div4), void_ptr);
				usart_send_string((uint8_t*)str_mount);
				usart_send_string((uint8_t*)" remains erased.\r\nLocked!");
				wait_tx_ends();
			}
			else
			{
				usart_send_string((uint8_t*)"\r\nWrong data written into flash memory:\r\n");
				usart_send_string((uint8_t*)"Dest add => 0x");
				void_ptr = &str_mount;
				conv_uint32_to_8a_hex((uintptr_t)(base_of_database + iter_div4), void_ptr);
				usart_send_string((uint8_t*)str_mount);
				usart_send_string((uint8_t*)", found = 0x");
				conv_uint32_to_8a_hex(*(base_of_database + iter_div4), void_ptr);
				usart_send_string((uint8_t*)str_mount);
				usart_send_string((uint8_t*)". Source add => 0x");
				conv_uint32_to_8a_hex((uintptr_t)(flash_buffer_ram + iter), void_ptr);
				usart_send_string((uint8_t*)str_mount);
				usart_send_string((uint8_t*)", was = 0x");
				conv_uint32_to_8a_hex(*((uint32_t*)(flash_buffer_ram + iter)), void_ptr);
				usart_send_string((uint8_t*)str_mount);
				usart_send_string((uint8_t*)"\r\nLocked!");
			}
			return FLASH_WRONG_DATA_WRITTEN;
		}	//if(	*(base_of_database + iter_div4) != *((uint32_t*)(flash_buffer_ram + iter)) )
	} //for (iter = 0; iter < DATABASE_SIZE; iter+=4)	//3K must be checked
	usart_send_string((uint8_t*)" Succesfully done!");

	//if flash sector FLASH_SECTOR3_NUMBER was just erased, there is no need to "invalidate" former Database,
	//as there is no former Database
	base_of_database_num = (uintptr_t)base_of_database;	//Convert from pointer to integer
	if (base_of_database_num != INITIAL_DATABASE)
	{
		usart_send_string((uint8_t*)"\r\n\nNulling former Database...\r\n");

		base_of_database += (DATABASE_SIZE /sizeof(uint32_t));	//points to former Database
		base_of_database_num = (uintptr_t)base_of_database;			//Convert from pointer to integer

		//Make former Database not useable anymore
		for(iter = 0; iter < sizeof(UNUSED_DATABASE); iter ++)
		{
			uint8_t ch = UNUSED_DATABASE[iter];
			wait_tx_ends();
			check_flash_locked();
			flash_program_byte(base_of_database_num + iter, ch);
			flash_wait_for_last_operation();
			check_flash_error();
			//Now check written data
			if((*(uint8_t*)(base_of_database_num+iter)) == ch)
			{
				//usart_send_string((uint8_t*)" <= Ok (\r\n");
				/*usart_send_string((uint8_t*)" <= Ok (0x");
				conv_uint8_to_2a_hex(ch, void_ptr);
				usart_send_string((uint8_t*)str_mount);
				usart_send_string((uint8_t*)")\r\n");*/
			}
			else	//if (*(uint8_t*)(base_of_database_num+iter) = ch)
			{
				//Information to user
				usart_send_string((uint8_t*)"Address: 0x");
				conv_uint32_to_8a_hex((uintptr_t)(base_of_database_num + iter), void_ptr);
				usart_send_string((uint8_t*)str_mount);
				usart_send_string((uint8_t*)" <= Wrong (0x");
				conv_uint8_to_2a_hex((*(uint8_t*)(base_of_database_num+iter)), void_ptr);
				conv_uint8_to_2a_hex(ch, void_ptr);
				usart_send_string((uint8_t*)str_mount);
				usart_send_string((uint8_t*)" instead of 0x");
				conv_uint8_to_2a_hex(ch, void_ptr);
				usart_send_string((uint8_t*)str_mount);
				usart_send_string((uint8_t*)")\r\n");
				flash_lock();
				return FLASH_WRONG_DATA_WRITTEN;
			}	//if (*(uint8_t*)(base_of_database+iter) = ch)
		}	//for(iter = 0; iter < sizeof(UNUSED_DATABASE); iter ++)
}
	usart_send_string((uint8_t*)"\r\nSuccessful!\r\n\nNow, please TURN OFF to plug the PS/2 keyboard!");
	flash_lock();
	return RESULT_OK;
}	//uint32_t flash_program_data(uint32_t *flash_buffer_ram)


int flashF4_rw(void)	//was main. It is int to allow simulate as a single module
{
	uint32_t result = 0;
	uint8_t str_mount[STRING_MOUNT_BUFFER_SIZE];
	uint8_t flash_buffer_ram[DATABASE_SIZE]; //Local variable, in aim to not consume resources in the main functional machine
	void *ptr_flash_buffer_ram;

	usart_send_string((uint8_t*)"It is ready to update the Database! To do so, now\r\n");
	usart_send_string((uint8_t*)"please send the new Database file in Intel Hex format!");
	usart_send_string((uint8_t*)"\r\n\nOr turn off now...");
	get_valid_intelhex_file(&str_mount[0], STRING_MOUNT_BUFFER_SIZE, &flash_buffer_ram[0]);
	
	ptr_flash_buffer_ram = flash_buffer_ram;
	wait_tx_ends();
	result = flash_program_data(ptr_flash_buffer_ram);
	
	switch(result)
	{
	case RESULT_OK: //everything ok
		break;
	case FLASH_WRONG_DATA_WRITTEN: //data read from Flash is different than written data
		break;

	default: //wrong flags' values in Flash Status Register (FLASH_SR)
		usart_send_string((uint8_t*)"\r\nWrong value of FLASH_SR: ");
		conv_uint32_to_8a_hex(result, str_mount);
		usart_send_string(&str_mount[0]);
		break;
	}
	//send end_of_line
	usart_send_string((uint8_t*)"\r\n");
	return result;
}	//int flashF4_rw(void)	//was main. It is int to allow simulate as a single module
