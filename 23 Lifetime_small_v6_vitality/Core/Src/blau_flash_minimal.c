/*
 * blau_flash_minimal.c
 *
 *  Created on: Mar 31, 2021
 *	   Version: V0.0
 *      Author: ruven
 */




#include <blau_flash_minimal.h>
#include "main.h"
#include <stdlib.h>
#include <stdio.h>

extern UART_HandleTypeDef huart3;
extern LPTIM_HandleTypeDef hlptim1;
extern CRC_HandleTypeDef hcrc;

extern 	uint16_t life_mean;
extern uint16_t life_count;
extern uint16_t life_flash;

//uart variables
uint8_t u_out[256]; //uart out buffer; reduce size if RAM is short
uint8_t u_len; //nuber of bytes to send

//lifetime variables
uint32_t bfmin_restarts;
uint32_t bfmin_lifetime_in_minutes;
uint8_t bfmin_is_first_interrupt=1;

//parameter variables
uint64_t parameters[BFMIN_NR_OF_PARAMETER_SLOTS];


/*
 * Example implementation for blau_flash_minimal library
 */
void bfmin_test()
{
	//bfmin_reset_counter();
	u_len = sprintf((char*)u_out,"\r\nTest Implementation:\r\n\r\n");
	HAL_UART_Transmit(&huart3, u_out, u_len, 100);

	//initialize
	bfmin_init();
	//bfmin_reset_memory();


	//print lifetime and restart count
	bfmin_print_lifetime();

	//set some parameters
	int cnt;
//	for(cnt=0;cnt<BFMIN_NR_OF_PARAMETER_SLOTS;cnt++)
//	{
//		bfmin_store_parameter(cnt, 573%cnt);
//	}
	//bfmin_store_parameter(4, 111);
	bfmin_print_parameter(4);

	//print all set parameters
	for(cnt=0;cnt<BFMIN_NR_OF_PARAMETER_SLOTS;cnt++)
	{
		bfmin_print_parameter(cnt);
	}


	//do nothing
//	while(1)
//	{
//
//	}
}



/*
 * Initialize flash
 */
void bfmin_init()
{

	bfmin_crc_init();

	//copy values to RAM
	bfmin_lifetime_in_minutes = bfmin_read_address((uint32_t)BFMIN_LIFETIME_ADDRESS)&0xffffffff;
	if(bfmin_lifetime_in_minutes == 0xffffffff)
	{
		uint32_t lifetime_rescue = bfmin_read_address((uint32_t)BFMIN_LIFETIME_RESCUE_ADDRESS)&0xffffffff;
		if(lifetime_rescue != 0xffffffff)
		{
			bfmin_lifetime_in_minutes = lifetime_rescue;
		}else
		{
			bfmin_reset_counter();
		}
	}
	bfmin_restarts = bfmin_read_address((uint32_t)BFMIN_RESTARTS_ADDRESS)&0xffffffff;
	int cnt;
	for(cnt=0;cnt<BFMIN_NR_OF_PARAMETER_SLOTS;cnt++)
	{
		bfmin_read_parameter(cnt, &parameters[cnt]);
	}



	//PVD configuration
	PWR_PVDTypeDef sConfigPVD;
	sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING; //IT_Rising triggers if voltage falls below threshold
	sConfigPVD.PVDLevel = PWR_PVDLEVEL_6; //Falling voltage threshold 2.9V #RM0444 Page 145
	HAL_PWREx_ConfigPVD(&sConfigPVD);

	//Clock source is 32kHz internal clock
	hlptim1.Instance = LPTIM1;
	hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
	hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV128;
	hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
	hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
	hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
	hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
	hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
	hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
	HAL_LPTIM_Init(&hlptim1);

	HAL_LPTIM_Counter_Start_IT(&hlptim1, 15000);//tick = 250Hz -> periode of 15000 = 60s

}

void bfmin_crc_init()
{
	hcrc.Instance = CRC;
	hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
	hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
	hcrc.Init.GeneratingPolynomial = 0x9b;
	hcrc.Init.CRCLength = CRC_POLYLENGTH_8B;
	hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
	hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
	hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_HALFWORDS;
	HAL_CRC_Init(&hcrc);
}



/*
 * read a doubleword
 * bfmin_ptr: Address in flash
 * return: Value at address
 */
uint64_t bfmin_read_address(uint32_t bfmin_ptr)
{
	return(uint64_t)*(uint64_t *)bfmin_ptr;
}




/*
 * Write a doubleword to a specific address
 * bfmin_addr: Address in flash to write to
 * bfmin_data: Value to write
 * return: Status of operation
 */
bfmin_error bfmin_write(uint32_t bfmin_addr, uint64_t bfmin_data)
{
	//check if address is aligned
	if(bfmin_addr%BFMIN_FLASH_CHUNK_SIZE!=0)
	{
		return BFMIN_AdrNotAligned;
	}

	//check if flash pointer is on bfmin flash page

	if((bfmin_addr >= BFMIN_LIFETIME_PAGE_ADDRESS && bfmin_addr < BFMIN_LIFETIME_PAGE_ADDRESS+BFMIN_FLASH_PAGE_SIZE) ||
			(bfmin_addr >= BFMIN_STORAGE_PAGE_ADDRESS && bfmin_addr < BFMIN_STORAGE_PAGE_ADDRESS+BFMIN_FLASH_PAGE_SIZE))
	{
		HAL_FLASH_Unlock();
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, bfmin_addr, bfmin_data);
		FLASH_WaitForLastOperation(10);
		HAL_FLASH_Lock();
	}else
	{
		return BFMIN_AdrOutOfPage;
	}
	return BFMIN_NoError;

}

/*
 * Reset liftime and restart counter to 0
 * return: Status of operation
 */
bfmin_error bfmin_reset_counter()
{
	bfmin_erase_page(BFMIN_LIFETIME_PAGE);
	bfmin_write(BFMIN_LIFETIME_ADDRESS, 0);
	bfmin_write(BFMIN_RESTARTS_ADDRESS, 0);
	return BFMIN_NoError;
}

/*
 * Get lifetime count in minutes
 * return: Lifetime count
 */
uint32_t bfmin_get_lifetime_in_minutes()
{
	return bfmin_lifetime_in_minutes;
}

/*
 * Get restart count
 * return: restart count
 */
uint32_t bfmin_get_restarts()
{
	return bfmin_restarts;
}

/*
 * Store a 64 bit parameter
 * 		slot: slot where to store the parameter (0-29)
 * parameter: 64bit value
 * return: Status of operation
 */
bfmin_error bfmin_store_parameter(uint8_t slot, uint64_t parameter)
{
	if(parameter == 0xffffffffffffffff)
	{
		return BFMIN_InvalidValue;
	}else if(parameter == parameters[slot])
	{
		return BFMIN_NoError;
	}
	parameters[slot] = parameter;
	uint64_t tmp;
	switch(bfmin_read_parameter(slot, &tmp))
	{
	case BFMIN_AdrOutOfPage:
		return BFMIN_AdrOutOfPage;
		break;
	case BFMIN_InvalidValue:
		bfmin_write(BFMIN_STORAGE_PAGE_ADDRESS+slot*BFMIN_NR_OF_BYTES_BETWEEN_SLOTS,parameter);
		return BFMIN_NoError;
		break;
	default:
		break;
	}

	bfmin_erase_page(BFMIN_STORAGE_PAGE);
	uint8_t cnt;
	for(cnt=0;cnt<BFMIN_NR_OF_PARAMETER_SLOTS;cnt++)
	{
		if(parameters[cnt]!=0xffffffffffffffff)
		{
			bfmin_write(BFMIN_STORAGE_PAGE_ADDRESS+cnt*BFMIN_NR_OF_BYTES_BETWEEN_SLOTS,parameters[cnt]);
		}
	}
	return BFMIN_NoError;

}

/*
 * Read parameter
 * 		slot: slot of the parameter to read (0-29)
 * parameter: pointer of 64bit variable where the parameter gets written to
 * return: Status of operation
 */
bfmin_error bfmin_read_parameter(uint8_t slot, uint64_t* parameter)
{
	if(slot>=BFMIN_NR_OF_PARAMETER_SLOTS)
	{
		return BFMIN_AdrOutOfPage;
	}
	(*parameter) = bfmin_read_address(BFMIN_STORAGE_PAGE_ADDRESS+slot*BFMIN_NR_OF_BYTES_BETWEEN_SLOTS);
	if(*parameter == 0xffffffffffffffff)
	{
		return BFMIN_InvalidValue;
	}
	else
	{
		return BFMIN_NoError;

	}
}


/*
 * Erase page
 * return: Status of operation
 */
bfmin_error bfmin_erase_page(uint8_t bfmin_page)
{
	bfmin_error err = BFMIN_NoError;
	uint32_t flash_err;
	FLASH_EraseInitTypeDef hflash;
	hflash.NbPages = 1;
	hflash.TypeErase = FLASH_TYPEERASE_PAGES;
	hflash.Page = bfmin_page;
	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&hflash, &flash_err);//Erase page one
	HAL_FLASH_Lock();

	return err;

}

/*
 * Erase all data, reset time and restart counter
 * return: Status of operation
 */
bfmin_error bfmin_reset_memory()
{
	bfmin_error err = BFMIN_NoError;
	uint32_t flash_err;
	FLASH_EraseInitTypeDef hflash;
	hflash.NbPages = 1;
	hflash.TypeErase = FLASH_TYPEERASE_PAGES;
	hflash.Page = BFMIN_LIFETIME_PAGE;
	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&hflash, &flash_err);//Erase page one
	hflash.Page = BFMIN_STORAGE_PAGE;
	HAL_FLASHEx_Erase(&hflash, &flash_err);//Erase page one
	HAL_FLASH_Lock();

	return err;

}



/*
 * calcualte the 8 bit checksum of a doubleword
 * data_ptr: pointer to first halfword of data
 * return: checksume
 */
uint8_t bfmin_calc_checksum(uint64_t* data_ptr)
{
	// CRC is set to half word input, data is 8 byte
	// so 4 half words are used for calculation
	return HAL_CRC_Calculate(&hcrc, (uint32_t *)data_ptr, 4);
}



/*
 * print the content of a bflash_entry
 * slot: parameter to print
 * return: Status of operation
 */
bfmin_error bfmin_print_parameter(uint8_t slot)
{
	if(slot>=BFMIN_NR_OF_PARAMETER_SLOTS)
	{
		return BFMIN_AdrOutOfPage;
	}
	if(parameters[slot]!= 0xffffffffffffffff)
	{
		u_len = sprintf((char*)u_out,"Parameter %u: %lu\r\n",slot, parameters[slot]);
		HAL_UART_Transmit(&huart3, u_out, u_len, 100);
	}

	return BFMIN_NoError;
}

/*
 * Print lifetime counter and reset counter
 * return: Status of operation
 */
bfmin_error bfmin_print_lifetime()
{

	u_len = sprintf((char*)u_out,"Lifetime in Minutes: %lu\r\nRestarts: %lu\r\n", bfmin_lifetime_in_minutes,bfmin_restarts);
	HAL_UART_Transmit(&huart3, u_out, u_len, 100);

	return BFMIN_NoError;
}


/*
 * Brown out interrupt routine
 */
void HAL_PWREx_PVD_Rising_Callback(void)
{
	bfmin_write(BFMIN_LIFETIME_ADDRESS, bfmin_lifetime_in_minutes);
	//HAL_UART_Transmit(&huart3, (uint8_t *)"Brown out\r\n", 11, 100);
}

/*
 * 60s periodic LPTIM1 interrupt
 */
void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
	update_life_flash();
	bfmin_lifetime_in_minutes++;
	bfmin_write(BFMIN_LIFETIME_RESCUE_ADDRESS, bfmin_lifetime_in_minutes);
	if(bfmin_is_first_interrupt)
	{

		bfmin_restarts++;
		bfmin_erase_page(BFMIN_LIFETIME_PAGE);
		bfmin_write(BFMIN_RESTARTS_ADDRESS, bfmin_restarts);
		bfmin_write(BFMIN_LIFETIME_RESCUE_ADDRESS, bfmin_lifetime_in_minutes);
		HAL_PWREx_EnablePVD();
		bfmin_is_first_interrupt=0;
	}
}

void update_life_flash(){
	uint16_t actlife;
	actlife=life_mean/life_count;

	if ((actlife<life_flash)&(life_count>20)){
		life_flash=life_flash-5;
		bfmin_store_parameter(6,(uint64_t)life_flash);
	}

	if(life_flash>100){
		life_flash=100;
		bfmin_store_parameter(6,(uint64_t)100);
	}
	if(life_flash<0){
		life_flash=0;
		bfmin_store_parameter(6,(uint64_t)0);
	}
	life_mean=0;
	life_count=0;
}
