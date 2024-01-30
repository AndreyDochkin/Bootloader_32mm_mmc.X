#ifndef __BOOTLOADER_H__
#define __BOOTLOADER_H__

//#define PROGRAM_FLASH_END_ADRESS (0x9D000000+BMXPFMSZ-1)
#define PROGRAM_FLASH_END_ADRESS (0x9D000000+0x30000-1)


//-------User configurable macros begin---------
#define MAJOR_VERSION 1
#define MINOR_VERSION 3


/* APP_FLASH_BASE_ADDRESS and APP_FLASH_END_ADDRESS reserves program Flash for the application*/ 
/* Rule: 
 		1)The memory regions kseg0_program_mem, kseg0_boot_mem, exception_mem and 
 		kseg1_boot_mem of the application linker script must fall with in APP_FLASH_BASE_ADDRESS 
 		and APP_FLASH_END_ADDRESS
 		 
 		2)The base address and end address must align on  4K address boundary */
 		
#define APP_FLASH_BASE_ADDRESS 	0x9D004000 
#define APP_FLASH_END_ADDRESS   PROGRAM_FLASH_END_ADRESS

/* Address of  the Flash from where the application starts executing */
/* Rule: Set APP_FLASH_BASE_ADDRESS to _RESET_ADDR value of application linker script*/

#define USER_APP_RESET_ADDRESS 	(APP_FLASH_BASE_ADDRESS + 0x1000)

#define VALID_BOOT_WRITE_END_ADRESSS 0x9D03FFF0
#define VALID_BOOT_JUMP_APP_ADRESSS  0x9D03FFF8
#define VALID_APP_START_ADRESSS      0x9D03FFE0

#define VALID_BOOT_WRITE_END_TAG     0xBA
#define VALID_BOOT_JUMP_APP_TAG      0xB1
#define VALID_APP_START_TAG          0xAC
//-------User configurable macros end-----------

#endif
