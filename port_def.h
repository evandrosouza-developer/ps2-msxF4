#pragma once

//Use Tab width=2

#ifdef __cplusplus
extern "C" {
#endif

//Microcontroller STM32F1 or STM32F4
/*#ifndef STM32F103
#define STM32F103
#endif*/
#ifndef STM32F401
#define STM32F401
#endif

#define	USE_USB										false

#define	FREQ_INT_SYSTICK					30
#define	MAX_TIMEOUT2RX_INTEL_HEX	11 * FREQ_INT_SYSTICK
#define	MAX_TIMEOUT2AMPERSAND			6	 * FREQ_INT_SYSTICK

#define	LEN_SERIAL_No							16

#define TIM_HR_TIMER							TIM4

#define IRQ_PRI_TIM								10		//Keyboard and high resolution timer
#define IRQ_PRI_EXT15							10		//Keyboard, while int above is not working
#define IRQ_PRI_Y_SCAN						20		//Colunm rows from 8255 (MSX)
#define IRQ_PRI_USB								30		//USB
#define IRQ_PRI_USART							50		//USART serial
#define IRQ_PRI_SYSTICK						100		//ARM system timer

#define X_ON											17
#define X_OFF 										19
#define USART_PORT 								USART1

#define EMBEDDED_BLUE_LED_PORT		GPIOC
#define EMBEDDED_BLUE_LED_PIN			GPIO13
#define X_port										GPIOB
#define X7_pin_id									GPIO0
#define X6_pin_id									GPIO15
#define X5_pin_id									GPIO1
#define X4_pin_id									GPIO14
#define X3_pin_id									GPIO3
#define X2_pin_id									GPIO13
#define X1_pin_id									GPIO10
#define X0_pin_id									GPIO12
#define Y3_port										GPIOA
#define Y3_pin_id									GPIO5
#define Y3_exti										EXTI5
#define Y2_port										GPIOA
#define Y2_pin_id									GPIO6
#define Y2_exti										EXTI6
#define Y1_port										GPIOA
#define Y1_pin_id									GPIO7
#define Y1_exti										EXTI7
#define Y0_port										GPIOA
#define Y0_pin_id									GPIO8
#define Y0_exti										EXTI8
#define CAPSLOCK_PORT							GPIOB
#define CAPSLOCK_PIN_ID						GPIO4
#define CAPSLOCK_exti							EXTI4
#define KANA_PORT									GPIOB
#define KANA_PIN_ID								GPIO6
#define KANA_exti									EXTI6

#define Y_MASK										0x1E0	//Valid bits: 16, 15, 14 and 13.


#define PS2_DATA_PIN_PORT					GPIOB
#define PS2_DATA_PIN_ID						GPIO5
#define PS2_CLOCK_PIN_PORT				GPIOA
#define PS2_CLOCK_PIN_ID					GPIO15
#define PS2_CLOCK_PIN_EXTI				EXTI15
#define PS2_POWER_CTR_PORT				GPIOA
#define PS2_POWER_CTR_PIN					GPIO4	


//Debug facilities
#define USER_KEY_PORT							GPIOA
#define USER_KEY_PIN_ID						GPIO0
#define INT_PS2_PIN_PORT					GPIOA
#define INT_PS2_PIN_ID						GPIO1
#define PS2_START_SEND_PORT				GPIOA
#define PS2_START_SEND_PIN_ID			GPIO2
#define Dbg_Yint_port							GPIOB
#define Dbg_Yint_pin_id						GPIO7
#define TIM2UIF_PORT							GPIOB
#define TIM2UIF_PIN_ID						GPIO8
#define TIM2CC1_PORT							GPIOB
#define TIM2CC1_PIN_ID						GPIO9


//Available resources
#define AVAILABLE_A3_PORT					GPIOA
#define AVAILABLE_A3_PIN_ID				GPIO3
#define AVAILABLE_A11_PORT				GPIOA
#define AVAILABLE_A11_PIN_ID 			GPIO11		//USB-
#define AVAILABLE_A12_PORT				GPIOA
#define AVAILABLE_A12_PIN_ID			GPIO12		//USB+
#define AVAILABLE_B2_PORT					GPIOB
#define AVAILABLE_B2_PIN_ID				GPIO2			//Boot1 pin (There is a R=10K to GND)


#define X7_SET_AND								0xFFFEFFFF  // 4294901759
#define X6_SET_AND								0x7FFFFFFF  // 2147483647
#define X5_SET_AND								0xFFFDFFFF  // 4294836223
#define X4_SET_AND								0xBFFFFFFF  // 3221225471
#define X3_SET_AND								0xFFF7FFFF  // 4294443007
#define X2_SET_AND								0xDFFFFFFF  // 3758096383
#define X1_SET_AND								0xFBFFFFFF  // 4227858431
#define X0_SET_AND								0xEFFFFFFF  // 4026531839
#define X7_CLEAR_OR								0x00010000  // 65536
#define X6_CLEAR_OR								0x80000000  // 2147483648
#define X5_CLEAR_OR								0x00020000  // 131072
#define X4_CLEAR_OR								0x40000000  // 1073741824
#define X3_CLEAR_OR								0x00080000  // 524288
#define X2_CLEAR_OR								0x20000000  // 536870912
#define X1_CLEAR_OR								0x04000000  // 67108864
#define X0_CLEAR_OR								0x10000000  // 268435456
#define X7_CLEAR_AND							0xFFFFFFFE  // 4294967294
#define X6_CLEAR_AND							0xFFFF7FFF  // 4294934527
#define X5_CLEAR_AND							0xFFFFFFFD  // 4294967293
#define X4_CLEAR_AND							0xFFFFBFFF  // 4294950911
#define X3_CLEAR_AND							0xFFFFFFF7  // 4294967287
#define X2_CLEAR_AND							0xFFFFDFFF  // 4294959103
#define X1_CLEAR_AND							0xFFFFFBFF  // 4294966271
#define X0_CLEAR_AND							0xFFFFEFFF  // 4294963199
#define X7_SET_OR									0x00000001  // 1
#define X6_SET_OR									0x00008000  // 32768
#define X5_SET_OR									0x00000002  // 2
#define X4_SET_OR									0x00004000  // 16384
#define X3_SET_OR									0x00000008  // 8
#define X2_SET_OR									0x00002000  // 8192
#define X1_SET_OR									0x00000400  // 1024
#define X0_SET_OR									0x00001000  // 4096

//#define MMIO32(addr)	(*(volatile uint32_t *)(addr))
#define DBGMCU_APB1_FZ						MMIO32(DBGMCU_BASE+8)
#define DBG_I2C3_SMBUS_TIMEOUT		1<<23
#define DBG_I2C2_SMBUS_TIMEOUT		1<<22
#define DBG_I2C1_SMBUS_TIMEOUT		1<<21
#define DBG_IWDG_STOP							1<<12
#define DBG_WWDG_STOP							1<<11
#define DBG_RTC_STOP							1<<10
#define DBG_TIM5_STOP							1<<11
#define DBG_TIM4_STOP							1<<11
#define DBG_TIM3_STOP							1<<11
#define DBG_TIM2_STOP							1<<0

#ifdef __cplusplus
}
#endif
