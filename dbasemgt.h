#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define DB_NUM_COLS						8						// Number of colunms of Database
#define N_DATABASE_REGISTERS	320
#define DATABASE_SIZE 				N_DATABASE_REGISTERS * DB_NUM_COLS	//It will resolve 2560

#define NUM_DATABASE_IMG 			(uint16_t)( (FLASH_SECTOR3_TOP+1-FLASH_SECTOR3_BASE)/DATABASE_SIZE )	//It will resolve 6

//Address of Base of flash page, used to put various Databases without need of erase each time
//update process is done. In STM32F4CCU6, the Database remains on the following flash address,
//with an amount of 16K (Flash Sector 3: 0x800C000 to 0x800FFFF)
#define INITIAL_DATABASE			0x0800F200	//SECTOR 3 - Place to put the initial (compilation time) database
#define FLASH_SECTOR3_BASE		0x0800C000
#define FLASH_SECTOR3_TOP			0x0800FFFF
#define FLASH_SECTOR3_NUMBER	3

//Entry points
int flashF4_rw(void);
void database_setup(void);


#ifdef __cplusplus
}
#endif
