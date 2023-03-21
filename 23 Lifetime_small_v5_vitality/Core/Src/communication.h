/*
 * communicaion.h
 *
 *  Created on: 13-Jan-2023
 *      Author: PC-Nr37
 */
#include "main.h"
#include "blau_flash_minimal.h"

#ifndef SRC_COMMUNICATION_H_
#define SRC_COMMUNICATION_H_



#endif /* SRC_COMMUNICATION_H_ */

//will decode decode tha command based on befehl[3] and befehl[4] values and assign the string to len which needs to be print

//
/*


void decode_command_new(){
	//for writing-
	if(befehl[3]=='w'){
		//-serial
		if(befehl[4]=='s'){
			//serial_flash=0;
			serial_flash=atoi(&befehl[5]);
			bfmin_store_parameter(7,(uint64_t)serial_flash);
			len=sprintf(str,"Write %d",serial_flash);
		}
		//- which version to print
		else if(befehl[4]=='v'){
			printversion=atoi(&befehl[5]);
			bfmin_store_parameter(5,(uint64_t)printversion);
			len=sprintf(str,"version set to %d",printversion);
		}

		else if(befehl[4]=='r'){
			bfmin_store_parameter(6,(uint64_t) 100);
			len=sprintf(str,"Life reset to %d",life_flash);
		}

		else{pflag=0;}//setting the flag again to 0 so that it does not print again if received any wrong befehl[4] value.

	}
	//for reading values from laser
	else if(befehl[3]=='r'){
		//reading serial
		if(befehl[4]=='s'){
			len=sprintf(str,"%d",serial_flash);
		}
		//read values in different versions
		else if(befehl[4]=='v'){
			if(printversion==1){
				len=sprintf(str,"Version%d\r%d\r%d\r%d\r%d\r\r",version,(uint16_t)((float)current/0.36),u_ref-pdsig,bfmin_lifetime_in_minutes/60,ADC_mean[0]);
			}
			else if(printversion==2){
				len=sprintf(str,"Version %d\r%d\r%d\r%d\r%d\r%d\r\r",2,(uint16_t)((float)current/0.36),u_ref-pdsig,bfmin_lifetime_in_minutes/60,ADC_mean[0],life/100);
			}
			else if(printversion==3){
				len = sprintf( str," temp*100;%d;Uc: %d;I mA*3.6;%d;PD mV;%d;count;%d; dac;%d; lifetime [min];%d; restarts; %d;life*100;%d;life_flash %d;delay %d \r",temp,ADC_mean[1],current,pdsig,ADC_count, dac_value,bfmin_lifetime_in_minutes,bfmin_restarts,life,life_flash,mydelay);//,M2M_PWM_CNT);
			}
			else{pflag=0;}//setting the flag again to 0 so that it does not print again if received any wrong befehl[4] value.
		}
		else{pflag=0;}//setting the flag again to 0 so that it does not print again if received any wrong befehl[4] value.
	}
	else{pflag=0;}//setting the flag again to 0 so that it does not print again if received any wrong befehl[4] value.
}


*/

//
