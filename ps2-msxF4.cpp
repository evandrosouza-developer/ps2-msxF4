/*
 * This file was part of the libopencm3, but it was edited to implement a PS/2 to MSX keyboard project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/usart.h>

#include "sys_timer.h"
#include "serial.h"
#include "hr_timer.h"
#include "ps2handl.h"
#include "msxmap.h"
#include "dbasemgt.h"
#include "SpecialFaultHandlers.h"
#include "port_def.h"
#include "cdcacm.h"

//#define DO_PRAGMA(x) _Pragma (#x)
//#define TODO(x) DO_PRAGMA(message (#x))

#define  DELAY_JHONSON	6

//Variáveis globais
extern uint32_t systicks;													//Declared on sys_timer.cpp
extern bool mount_scancode_OK; 										//Declared on ps2handl.c
extern bool ps2_keyb_detected;										//Declared on ps2handl.c
extern bool ps2numlockstate;											//Declared on ps2handl.c
extern bool command_ok, ps2int_RX_bit_idx;				//Declared on ps2handl.c
extern bool caps_state, kana_state;								//Declared on ps2handl.c
extern bool caps_former, kana_former;							//Declared on ps2handl.c
extern volatile bool update_ps2_leds;							//Declared on ps2handl.c
extern bool compatible_database;									//Declared on msxmap.cpp
extern uint8_t scancode[4];												//Declared on msxmap.cpp
extern uint32_t formerscancode;										//Declared on msxmap.cpp
extern bool do_next_keep_alive;										//Declared on msxmap.cpp
extern char serial_no[LEN_SERIAL_No + 1];			//Declared on cdcacm.c


int main(void)
{
	rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);

	// Disable JTAG, enable SWD. This frees up GPIO PA15 (JTDI), PB3 (JTDO / TRACESWO) and PB4 (NJTRST)
	// GPIO PA13 (SWDIO) and PA14 (SWCLK) are still used by STlink SWD.

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	power_on_ps2_keyboard();

	//Debug & performance measurement
	general_debug_setup();

	//Minimize interferencies from unused pins left open
	put_pullups_on_non_used_pins();

	//Get Serial number based on unique microcontroller factory masked number
	serialno_read(serial_no);

	serial_setup();

	//User messages
	serial_send_string((uint8_t*)"\r\n\n\nPS/2 keyboard Interface for MSX, based on STM32F401\r\nSerial number ");
	serial_send_string((uint8_t*)serial_no);
	serial_send_string((uint8_t*)"\r\nFirmware built on ");
	serial_send_string((uint8_t*)__DATE__);
	serial_send_string((uint8_t*)" ");
	serial_send_string((uint8_t*)__TIME__);
	serial_send_string((uint8_t*)"\r\n\nBooting ...\r\n\n");

	//User messages
	serial_send_string((uint8_t*)"PS/2 Interface powered up.\r\n\nConfiguring:\r\n");
	serial_send_string((uint8_t*)". ARM System Timer;\r\n");

	systick_setup();
	
	// Turn on the Independent WatchDog Timer
	iwdg_set_period_ms(67);	// 2 x sys_timer
	iwdg_start();

	//User messages
	serial_send_string((uint8_t*)". High resolution Timer;\r\n");

	// Now configure High Resolution Timer for PS/2 Clock interrupts (via CC) and micro second Delay
	tim_setup(TIM2);

	//User messages
	serial_send_string((uint8_t*)". PS/2 Port: Waiting up to 2.5s (75 ticks) with powered on keyboard\r\n");
	serial_send_string((uint8_t*)"  to proceed BAT:\r\n");

	ps2_keyb_detect();

	//User messages
	serial_send_string((uint8_t*)". Database with know-how to manage and interface PS/2 Keyboard to MSX;\r\n");
	//Check the Database version, get y_dummy, ps2numlockstate and enable_xon_xoff
	database_setup();

	//User messages
	serial_send_string((uint8_t*)". 5V compatible pin ports and interrupts to interface to MSX.\r\n");

	msxmap object;
	object.msx_interface_setup();

	if (!compatible_database || !ps2_keyb_detected)
	{
		//Here it will allow update the Database Conversion, as the PS/2 keyboard was not detected
		//or Database is incompatible, so at this point, this module is not working as target planned.

		//No PS/2 keyboard, so power off the PS/2 port
		power_off_ps2_keyboard();

		//Init procedure to fillin MSXTABLE by receiving an INTEL HEX through USART1
		//Disables all interrupts but systicks and USART
		exti_disable_request(Y3_exti | Y2_exti | Y1_exti | Y0_exti);
		timer_disable_irq(TIM2, TIM_DIER_CC1IE | TIM_DIER_UIE);
		exti_disable_request(PS2_CLOCK_PIN_EXTI);

		//Read the MSX Database Table Intel Hex by serial and flashes up to 3K, from 0x0800F400 to 0x0800FFFF
		flashF4_rw();

		//Halt here.
		for(;;);
	}

	//User messages
	serial_send_string((uint8_t*)"\r\nBoot complete. Be welcome!\r\n");

	//Test keyboard leds (Jhonson Counter = for humans)
	uint32_t systicks_base = systicks;
	ps2_update_leds(false, false, false);
	while (systicks < (systicks_base + 1 * DELAY_JHONSON))
		__asm("nop");

	ps2_update_leds(true, false, false);
	while (systicks < (systicks_base + 2 * DELAY_JHONSON))
		__asm("nop");

	ps2_update_leds(true, true, false);
	while (systicks < (systicks_base + 3 * DELAY_JHONSON))
		__asm("nop");

	ps2_update_leds(true, true, true);
	while (systicks < (systicks_base + 4 * DELAY_JHONSON))
		__asm("nop");

	ps2_update_leds(false, true, true);
	while (systicks < (systicks_base + 5 * DELAY_JHONSON))
		__asm("nop");

	ps2_update_leds(false, false, true);
	while (systicks < (systicks_base + 6 * DELAY_JHONSON))
		__asm("nop");

	ps2_update_leds(false, false, false);
	while (systicks < (systicks_base + 7 * DELAY_JHONSON))
		__asm("nop");


	/*********************************************************************************************/
	/************************************** Main Loop ********************************************/
	/*********************************************************************************************/
	uint32_t* ptr_scancode = (uint32_t*)&scancode[0];
	for(;;)
	{
		//The first functionality running in the main loop
		//New key processing:
		if (mount_scancode())
		{
			//At this point, we already have the assembled compound keys code, so
			//to avoid unnecessary processing at convert2msx, check if this last scancode
			//is different from the former one.
			if (*ptr_scancode != formerscancode)
			{
				//Serial message the keyboard change
				/*uint8_t mountstring[3];
				serial_send_string((uint8_t*)"Bytes qty=");
				conv_uint8_to_2a_hex(scancode[0], &mountstring[0]);
				serial_send_string(&mountstring[0]);
				serial_send_string((uint8_t*)"; Scan code=");
				conv_uint8_to_2a_hex(scancode[1], &mountstring[0]);
				serial_send_string(&mountstring[0]);
				serial_send_string((uint8_t*)"; ");
				conv_uint8_to_2a_hex(scancode[2], &mountstring[0]);
				serial_send_string(&mountstring[0]);
				serial_send_string((uint8_t*)"; ");
				conv_uint8_to_2a_hex(scancode[3], &mountstring[0]);
				serial_send_string(&mountstring[0]);
				serial_send_string((uint8_t*)"\r\n"); */
				//Toggle led each keyboard change (both new presses and releases).
				gpio_toggle(GPIOC, GPIO13); //Toggle led to sinalize a scan code is beeing send to convert2msx
				// Do the MSX search and conversion
				msxmap objeto;
				objeto.convert2msx();
			}	//if (scancode != formerscancode)
			//update former keystroke
			formerscancode = *ptr_scancode;
			//Now we can reset to prepair to do a new mount_scancode (Clear to read a new key press or release)
			*ptr_scancode = 0;
			mount_scancode_OK = false;
		}	//if (mount_scancode())

		//If keyboard is not responding to commands, reinit it through reset
		if(!(systicks & (uint32_t)0x7F))
		{	//each ~4s (128 * 1/30)
			if(do_next_keep_alive)	//when time comes up, do it just once
			{
				do_next_keep_alive = false;
				if(!keyboard_check_alive())
				{
					//User messages
					serial_send_string((uint8_t*)"Keyboard is non responsive (KeepAlive): Reset requested by the system.\r\n");
					reset_requested();
				}
			}
		}
		else
			do_next_keep_alive = true;

		//The second functionality running in main loop: Update the keyboard leds
		if(	command_ok	&&		//Only does led update when the previous one is concluded
				update_ps2_leds )
		{
			update_ps2_leds = false;
			caps_former = caps_state;
			kana_former = kana_state;
			ps2_update_leds(ps2numlockstate, !caps_state, !kana_state);
		}	//if ( update_ps2_leds || (caps_state != caps_former) || (kana_state != kana_former) )

		//Keep RX serial buffer empty and echoes to output
		while(serial_available_get_char())
		{
			uint8_t ch = serial_get_char();
			//Don't send X_OFF to avoid to lock the remote terminal, if it has X_ON/X_OFF enabled
			if(ch != X_OFF)
				serial_put_char(ch);
			else
				serial_send_string((uint8_t*)"<X_OFF>");
		}
	}	//for(;;)

	return 0; //Suppose never reach here
} //int main(void)
