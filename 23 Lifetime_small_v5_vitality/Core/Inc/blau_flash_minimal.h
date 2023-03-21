/*
 * blau_flash_minimal.h
 *
 *  Created on: Mar 31, 2021
 *	   Version: V0.0
 *      Author: ruven
 */

/*
 *	ReadMe
 *
 *	Setup in CubeMX
 *		- LPTIM1
 *			o Mode: Count internal clock events
 *			o enable LPTIM1 interrupt
*		- Clock confuguration
 *			o LPTIM1 Clock Mux to LSI
 *		- CRC
 *			o Activated
 *		- USART 3
 *			o Asynchronous Mode
 *
 *	Library usage
 *		- Copy blau_flash_minimal.c in Projectname/Core/Src
 *		- Copy blau_flash_minimal.h in Projectname/Core/Inc
 *		- include blau_flash_minimal.h in main
 *		- call bfmin_init() in main after peripheral initialization
 *
 *	Overall function lifetime counter
 *
 * 		- Data format
 * 			o liftime counter stores the number of minutes, that the microcontroller was running.
 * 			o the counter stores 4 bytes
 * 				x 2^32 / 60 / 24 / 365 gives 8171.6 years
 * 		- On startup:
 * 			o start LPTIM1 in interrupt mode
 * 			o copy restart counter and liftime counter from flash into RAM
 * 			o if lifetime counter is empty (0xffffffff) this means the value was not written on brown out
 * 				x check if the life time rescue value is valid, if so use is as lifetime
 *		- Timing:
 *			o LPTIM1 is used for timing.
 *			o after first 60 seconds:
 *				x increment restart and lifetime counter on RAM
 *				x erase liftime flash page
 *				x write restart counter to flash
 *				x write lifetime_rescue counter to flash
 *				x enable PVD interrupt
 *			o every 60s LPTIM1 interrupt
 *				x increment lifetime counter on RAM
 *		- On brown out
 *			o PVD Interrupt not enabled yet
 *				x do nothing
 *			o PVD interrupt enabled
 *				x write liftime counter from RAM into flash
 *
 *	Overall function parameter storage
 *		Stores up to 30 64bit parameter on flash
 *
 * 		- On startup:
 * 			o copy all parameters from flash to RAM
 *		- On parameter store
 *			o write parameter to RAM
 *			o if slot is empty (0xfffffffffffffffff)
 *				x write parameter to address
 *			o if address is not empty
 *				x erase flash page
 *				x write all(set) parameters from RAM to flash
 *		- On parameter read
 *			o return parameter slot from RAM
 *
 *
 * Flash functional description: Reference Manual RM0444 Rev 4 Page 62
 *
 * 		- A Main memory block containing 64 pages of 2 Kbytes, each page with eight rows of 256 bytes.
 *		- Writable address space: 0x0800 0000 - 0x0801 FFFF (64 Pages)
 *		- One page 2kB equals 0x07ff
 *		- Writable in double words (8B)
 *		- One page contains 256 double words
 *
 */

#include "main.h"
//#include <stdlib.h>
//#include <stdio.h>

#ifndef INC_BLAU_FLASH_MINIMAL_H_
#define INC_BLAU_FLASH_MINIMAL_H_

//Properties of the flash memory on STM32G071K8
#define BFMIN_FLASH_PAGE_SIZE 2048 //bytes
#define BFMIN_NR_OF_PAGES 64
#define BFMIN_MEMORY_BASE_ADDRESS 0x08000000 //Start address of flash memory
#define BFMIN_FLASH_SIZE BFMIN_FLASH_PAGE_SIZE*BFMIN_NR_OF_PAGES
#define BFMIN_FLASH_CHUNK_SIZE 8 // Flash is only writable in 8 byte(doubleword) chunks

//blau flash minimal settings

//lifetime counter settings
#define BFMIN_LIFETIME_PAGE (BFMIN_NR_OF_PAGES-1) //Flash page which is used to store lifetime data
#define BFMIN_LIFETIME_PAGE_ADDRESS BFMIN_MEMORY_BASE_ADDRESS+BFMIN_LIFETIME_PAGE*BFMIN_FLASH_PAGE_SIZE
#define BFMIN_LIFETIME_ADDRESS BFMIN_LIFETIME_PAGE_ADDRESS
#define BFMIN_RESTARTS_ADDRESS BFMIN_LIFETIME_PAGE_ADDRESS+2*BFMIN_FLASH_CHUNK_SIZE
#define BFMIN_LIFETIME_RESCUE_ADDRESS BFMIN_LIFETIME_PAGE_ADDRESS+4*BFMIN_FLASH_CHUNK_SIZE
#define BFMIN_MIRROR_LIFETIME false //Store counters twice with checksum to ensure repairability #not implemented yet


//constant parameter storage settings
#define BFMIN_STORAGE_PAGE (BFMIN_NR_OF_PAGES-2) //Flash page which is used to store constant parameters
#define BFMIN_STORAGE_PAGE_ADDRESS BFMIN_MEMORY_BASE_ADDRESS+BFMIN_STORAGE_PAGE*BFMIN_FLASH_PAGE_SIZE
#define BFMIN_NR_OF_PARAMETER_SLOTS 30
#define BFMIN_NR_OF_BYTES_BETWEEN_SLOTS 3*BFMIN_FLASH_CHUNK_SIZE
#define BFMIN_MIRROR_PARAMETERS false //Store each parameter twice with checksum to ensure repairability #not implemented yet





typedef enum {
	BFMIN_NoError,
	BFMIN_BflashChecksumError,
	BFMIN_AdrNotAligned,
	BFMIN_AdrOutOfPage,
	BFMIN_InvalidValue,
	BFMIN_NotImplemented
}bfmin_error;



void bfmin_test();
void bfmin_init();
void bfmin_crc_init();



uint64_t bfmin_read_address(uint32_t bfmin_ptr);
bfmin_error bfmin_write(uint32_t bfmin_addr, uint64_t bfmin_data);


bfmin_error bfmin_reset_memory();
bfmin_error bfmin_erase_page(uint8_t bfmin_page);

bfmin_error bfmin_reset_counter();

uint8_t bfmin_calc_checksum(uint64_t* data_ptr);
bfmin_error bfmin_print_parameter(uint8_t parameter);
bfmin_error bfmin_print_lifetime();


uint32_t bfmin_get_lifetime_in_minutes();
uint32_t bfmin_get_restarts();
bfmin_error bfmin_store_parameter(uint8_t slot, uint64_t parameter);
bfmin_error bfmin_read_parameter(uint8_t slot, uint64_t* parameter);





#endif /* INC_BLAU_FLASH_MINIMAL_H_ */
