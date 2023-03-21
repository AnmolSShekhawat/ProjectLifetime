/* USER CODE BEGIN Header */
/**
 * Programm für Stromtreiber_3A_lifetime_v4 Platine
 * Versuch der Leistungsregelung über die Photodiode und Ausgabe der Lebensdauer in %
 * Strom-Temp-Kennlinie für 76mW abgelegt
 * Funktion sehr gut, siehe Messreport, jetzt mit dig Modulation
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include<string.h>
#include <inttypes.h>
#include <math.h>
#include "blau_flash_minimal.h"
#include "communication.h"
//#include "blau_flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_channels 8
#define Ringbuffer_Size 150
//#define laser 1// rotes Modul
//#define laser 2// grünes Modul
#define laser 3// blaues Modul

#define PWM_period 50
#define PWM_duty 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac1;

LPTIM_HandleTypeDef hlptim1;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
uint32_t ADC_ringbuffer[ADC_channels*Ringbuffer_Size];
//uint32_t ADC_mean[ADC_channels*Ringbuffer_Size];
uint32_t ADC_count=0;
uint32_t PWM_count = 0;
uint32_t ADC_mean[ADC_channels];
//uint32_t ADC_test[Ringbuffer_Size];
float pd;
int16_t temp;
uint32_t current;
uint32_t teccurrent;
uint8_t str[200]="leer";
uint16_t u_ref=3300;
uint16_t modmax=3056;
uint16_t udmax=4000;
//uint16_t udhalf=3319;
uint16_t sdmin=0;
uint16_t smmin=50;
uint16_t sdhalf=3319;
uint16_t sdmax=4000;
uint16_t smmax=2460;
uint16_t tecvalue=0;
int32_t integral=0;
//uint16_t solltemp=2500;
uint16_t teccontrol=0;
uint16_t tecmax=3700;
float teccalc=0;
float v=0.5;
float h=0.1;
float m=2.0;
uint16_t pdset;
//Muster-NR319
//#ifdef laser==1
//	uint16_t pdset=1210;
//#endif
////Muster-Nr326
//#ifdef laser==2
//	uint16_t pdset=2800;//540;//2050;
//#endif
uint64_t m_pointer;
uint16_t version=1;
extern uint32_t bfmin_lifetime_in_minutes;
extern uint32_t bfmin_restarts;

volatile uint16_t mycounter1;
volatile uint16_t mycounter2;
volatile uint16_t mydelay;
uint16_t dutymax=20;
uint16_t pulselengthmax=380;
uint16_t pulsebreakmin=50;
uint16_t laserenable=1;

uint16_t dac_value=2500;
uint16_t ampli=1600;
volatile uint8_t ADC_ready=1;
uint8_t adc_stop = 1;
uint8_t ringbuffer_lock = 0; //
uint32_t Ringbuffer_PTR;
uint32_t M2M_ADC_CNT;
uint32_t M2M_PWM_CNT;
uint8_t adc_busy = 0;
uint8_t adc_restart_but_discard_last = 0;
volatile uint8_t enablelaser=1;
uint8_t overtemp=0;
volatile uint8_t adc_ready=0;
uint16_t len;
uint16_t life;
uint16_t life_mean=100;
uint16_t life_count=0;
int16_t life_flash=100;
uint8_t print=0;
uint16_t pdsig;
int pflag=0; //flag for enabling serial print
int start_command=0; //flag
int dly =15;//delay for changing rs485 to sending mode
uint16_t serial_flash;//stores serial in flash memory
uint8_t befehl[30]="";
uint8_t pos=0;
//uint16_t v=1;
//int16_t temp=250;
//int16_t solltemp=250;
volatile uint32_t dacadresse=0;
uint16_t dac_min=500;
uint64_t printversion=3;
//uint32_t bflash_active_page_address=0, bflash_active_write_address=0,bflash_active_read_address=0;

 /*
  * Debugging variables
  */
uint32_t main_cycle_count=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_CRC_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
int16_t readThermistor(uint16_t);
uint32_t interpolateDac(uint32_t mod);
uint16_t lifetime(float current, float temperature);
uint16_t tempcomp(float current, float temperature);
void update_life_flash(uint16_t actlife);
void TIM14_Callback();
//void DMA_M2M_CB();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
if (laser==1)
	 pdset=1900;//1210;
//Muster-Nr326: 200mv->1550
if (laser==2)
	pdset=1650;//540;//2050;
if (laser==3)
	pdset=2100;
	//char uart_data[]="test";
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_CRC_Init();
  MX_LPTIM1_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  //HAL_NVIC_DisableIRQ(EXTI4_15_IRQn); //Disable EXTI interrupt


  //htim16.Instance->ARR = PWM_period;
  //htim16.Instance->CCR1 = PWM_duty;

  //HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); //Start PWM Generator ###FOR DEVELOPMENT USE ONLY###
   //__HAL_TIM_DISABLE_IT(&htim14,TIM_IT_CC1);
   //__HAL_TIM_ENABLE_IT(&htim16,TIM_IT_UPDATE);



  //HAL_DMA_RegisterCallback(&hdma_memtomem_dma1_channel3, HAL_DMA_XFER_CPLT_CB_ID, (void *)DMA_M2M_CB);
 // bflash_reset_memory();
  //bflash_bo_init();
  //bflash_print_last_entry();
	//bflash_entry entr;
	//bflash_read_entry(bflash_active_read_address, &entr);
	//bflash_print_entry(entr);
  //HAL_TIM_Base_Start_IT(&htim17);
  HAL_ADCEx_Calibration_Start(&hadc1);

  HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);
  dacadresse=(DAC1_BASE+0x08);


  //HAL_DAC_Start(&hdac1,DAC_CHANNEL_2);




  //__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT); //Disable half transfer interrupt, saves time
  //HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  //HAL_NVIC_EnableIRQ(TIM2_IRQn);
  //HAL_NVIC_EnableIRQ(TIM3_IRQn);
 // HAL_TIM_Base_Start(&htim14);
  //HAL_TIM_OC_Start_IT(&htim14, TIM_CHANNEL_1);  //misst Anzeit
  //HAL_TIM_OC_Start_IT(&htim15, TIM_CHANNEL_1);  //misst Pausenzeit
  //TIM15->CCR1=pulselengthmax*100/dutymax;   // maximale Pausenzeit setzten
 // TIM14->CCR1=pulselengthmax;
 // HAL_TIM_OC_Start_IT(&htim16, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim16);
 // HAL_TIM_Base_Start(&htim17);
  //__HAL_TIM_ENABLE(&htim17);
 // __HAL_TIM_ENABLE_IT(&htim14,TIM_IT_UPDATE);
 // __HAL_TIM_ENABLE_IT(&htim14,TIM_IT_CC1);
  //TIM14->CCER=1;

  //__HAL_TIM_ENABLE_IT(&htim15,TIM_IT_UPDATE);

 //HAL_TIM_OnePulse_Start_IT(&htim14, TIM_CHANNEL_1);
 // __HAL_TIM_ENABLE_IT(&htim14,TIM_IT_UPDATE);
 // __HAL_TIM_ENABLE_IT(&htim14,TIM_IT_CC1);
  //TIM_CCxChannelCmd(&htim14, TIM_CHANNEL_1, TIM_CCx_ENABLE);
  //__HAL_TIM_ENABLE(&htim14);
  //HAL_TIM_PWM_Start_IT(&htim14, TIM_CHANNEL_1);
  //HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
  //__HAL_TIM_ENABLE_IT(&htim3,TIM_IT_CC1);
  //HAL_TIM_OnePulse_Start_IT(&htim2, TIM_CHANNEL_1);
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  //HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
 // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
 // HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

 // __HAL_TIM_ENABLE_IT(&htim2,TIM_IT_CC1);
 // TIM_CCxChannelCmd(&htim2, TIM_CHANNEL_1, TIM_CCx_ENABLE);

  //__HAL_TIM_ENABLE_IT(&htim3,TIM_IT_CC1);
  //TIM_CCxChannelCmd(&htim3, TIM_CHANNEL_1, TIM_CCx_ENABLE);
  //__HAL_TIM_ENABLE(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //dac_value=2500;
  //GPIOC->BRR = (uint32_t)GPIO_PIN_6;
  //HAL_TIM_Base_Start_IT(&htim15);

 HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  //HAL_NVIC_EnableIRQ(EXTI0_1_IRQn); //Enable EXTI Interrupt
 // HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn); //Enable ADC DMA Interrupt
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_ringbuffer,ADC_channels*Ringbuffer_Size);

//  HAL_NVIC_EnableIRQ(USART3_4_LPUART1_IRQn);
  HAL_UART_Receive_IT(&huart3, befehl, 1);

	bfmin_init();
//	bfmin_reset_memory();
//	//bfmin_store_parameter(6,(uint64_t) 100); //reset life_flash to 100
//	//bfmin_print_lifetime();
//	bfmin_read_parameter(2, &m_pointer);
//	sdmin=(uint32_t)m_pointer;
//	bfmin_read_parameter(3, &m_pointer);
//	m=(float)m_pointer/4096;
	bfmin_read_parameter(5,&m_pointer);
	printversion=(uint16_t)m_pointer;
	bfmin_read_parameter(6,&m_pointer);
	life_flash=(int16_t)m_pointer;

	bfmin_read_parameter(7,&m_pointer);//setting the pointer to index 7 from flash memory
	serial_flash=(uint16_t)m_pointer;// assigning that index value to serial_flash(int)

	//HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
  //HAL_ADC_Start_DMA(&hadc1, pData, Length);
//  uint32_t v=0;
  //calculateinterpol();
  //if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11))
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
  while (1)
  {

	  //for(int i =0;i<10;i++);
	  //main_cycle_count++;
	 	//dac_value=ADC_mean[4];
	 // HAL_Delay(100);
	  //TIM1->CCR1=(uint32_t)v++;
	  //if(v>10)
		//  v=0;
		//sprintf(str,"ADC4 %" PRIu16 "\n\r", ADC_ringbuffer[4])
		//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1 , DAC_ALIGN_12B_R, dac_value);
		if(adc_ready==1){
			adc_ready=0;
			//mydelay=__HAL_TIM_GET_COUNTER(&htim14);
			ADC_mean[0]=0;
			ADC_mean[1]=0;
			ADC_mean[2]=0;
			ADC_mean[3]=0;
			ADC_mean[4]=0;
			ADC_mean[5]=0;

			//ADC_mean[5]=0;
			ADC_count=0;
			for(int i=0;i<Ringbuffer_Size;i++){
				if((ADC_ringbuffer[7+i*8]>1000)&(ADC_ringbuffer[i*8]>1000)){
					ADC_mean[1]+=ADC_ringbuffer[2+8*i];
					ADC_mean[2]+=ADC_ringbuffer[3+8*i];
					ADC_mean[3]+=ADC_ringbuffer[4+8*i];  //PD
			//		life=lifetime((float)ADC_mean[3],(float)ADC_mean[0]);
//					if(print==1){
//						len = sprintf(str,"i=%d;%d;%d;%d;%d;%d;%d \r\n",i,ADC_ringbuffer[i],ADC_ringbuffer[i+1],ADC_ringbuffer[i+2],ADC_ringbuffer[i+3],ADC_ringbuffer[i+4],ADC_ringbuffer[i+5]);
						//HAL_UART_Transmit_DMA(&huart3, str, len);
//					}
					ADC_count++;
				}
				ADC_mean[0]+=ADC_ringbuffer[1+8*i];  //temp
				ADC_mean[4]+=ADC_ringbuffer[5+8*i];  //mod
				ADC_mean[5]+=ADC_ringbuffer[6+8*i];
			//	ADC_test[i]=ADC_ringbuffer[4+8*i];


			}
			//for(int i=0;i<Ringbuffer_Size*ADC_channels;i++){
			//	ADC_ringbuffer[i]=0;
			//}
			//ADC_mean[0]=ADC_mean[0]*u_ref/4095/Ringbuffer_Size;
			temp=readThermistor((uint16_t)(ADC_mean[0]*u_ref/4095/Ringbuffer_Size));  //temp NTC 10K
			ADC_mean[4]=ADC_mean[4]*u_ref/4095/Ringbuffer_Size; //Mod in D, alt:PD
			ADC_mean[5]=ADC_mean[5]*u_ref/4095/Ringbuffer_Size; //I_Tec
			if(ADC_count>10){
				//ADC_mean[0]=ADC_mean[0]*u_ref/4095/ADC_count-600;  //temp LM61

				ADC_mean[1]=ADC_mean[1]*u_ref/4095/ADC_count; //UCollektor, alt: mod in
				current=ADC_mean[2]*u_ref/4095/ADC_count;  //I, alt:U collector
				pdsig=ADC_mean[3]*u_ref/4095/ADC_count; //PD, alt: I Laserdiode

			}
			//temp=ADC_mean[0];
			pd=(float)pdsig;
			//current=ADC_mean[2]/3.6;
			teccurrent=ADC_mean[5]*2;

			if(temp>4500)
				overtemp=1;
			if(temp<4000)
				overtemp=0;
			__disable_irq();
			if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6)==GPIO_PIN_SET)&(ADC_count>10)){
//				//Muster-NR319
	        if (laser==1){
				if(pdsig<(pdset))
					dac_value--;
				if(pdsig>(pdset))
					dac_value++;
			}
				//Muster-NR326
			if (laser==2){
				if(pdsig<(pdset))
					dac_value++;
				if(pdsig>(pdset))
					dac_value--;
			}
	        if (laser==3){
				if(pdsig<(pdset))
					dac_value--;
				if(pdsig>(pdset))
					dac_value++;
			}
				//----------------------------
				//dac_value=2650;
				if(dac_value>4000)
						dac_value=4000;
				if(dac_value<1)
						dac_value=1;
				if(overtemp){
					dac_value=0;
					//__disable_irq();
					HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1 , DAC_ALIGN_12B_R, (uint32_t)dac_value);
					//__enable_irq();
				}
				//__disable_irq();
				ampli=dac_value;
				//*(uint32_t*)(dacadresse)=(uint32_t)ampli;
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1 , DAC_ALIGN_12B_R, (uint32_t)dac_value);


			//	__disable_irq();
			//	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1 , DAC_ALIGN_12B_R, 1960);
			//	ampli=1960;
				//__enable_irq();
				//ampli=dac_value;
			}
			__enable_irq();
//			else{
//				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1 , DAC_ALIGN_12B_R, (uint32_t)dac_min);
//				ampli=dac_value;
//			}
			//__enable_irq();
			//TIM1->CCR3=(uint32_t)(ADC_mean[4]/25);
//			if((teccontrol)){
//				teccalc=((float)temp-(float)solltemp)*v+h*(float)integral;
//				if(print==1){
//					if((temp>solltemp)&(integral<1000))
//						integral++;
//					else if(integral>1)
//						integral--;
//				}
//				if((teccalc>(float)tecvalue)&(teccurrent<tecmax))
//					tecvalue++;
//				else if((teccalc<(float)tecvalue)&(tecvalue>1))
//					tecvalue--;
//			}
			//if(tecvalue>100)
			//	tecvalue=100;

//			TIM1->CCR3=(uint32_t)tecvalue;

			if(print==1){
				print=0;
				if((dac_value>dac_min)&(ADC_count>10)){
					life=lifetime((float)current,(float)temp);
					life_mean=life_mean+life/100;
					life_count++;
				}
			}

				//__HAL_TIM_SetCounter(&htim14,0);
				HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_ringbuffer,ADC_channels*Ringbuffer_Size);

		}



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_19CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_19CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV128;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 64000;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin){
	__disable_irq;
	GPIOC->BRR = (uint32_t)GPIO_PIN_6;
		*(uint32_t*)(dacadresse)=(uint32_t)dac_min;   //laser ausschalten
	__enable_irq;
	//	__HAL_TIM_DISABLE(&htim17);
	//	__HAL_TIM_SET_COUNTER(&htim17,0);
	//	__HAL_TIM_ENABLE(&htim17);

}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin){


	//	mycounter2=__HAL_TIM_GET_COUNTER(&htim17);
	//	mydelay=mycounter2;//-mycounter1;

		//if(mydelay>20){
		__disable_irq;
			*(uint32_t*)(dacadresse)=(uint32_t)ampli;
			GPIOC->BSRR = (uint32_t)GPIO_PIN_6;  //PIN um ADC anzuzeigen das LD an
		__enable_irq;
		//	__HAL_TIM_DISABLE(&htim17);
		//	__HAL_TIM_ENABLE(&htim15);
		//	if(mydelay<300){
		//		TIM15->ARR=mydelay/2;
		//	}
		//	else{
		//		TIM15->ARR=150;
		//	}
	}


		//	HAL_TIM_OnePulse_Start_IT(&htim14, TIM_CHANNEL_1);
	//	laserenable=0;
	//	__HAL_TIM_ENABLE(&htim14);
	//	__HAL_TIM_SET_COUNTER(&htim14,0);
	//	__HAL_TIM_DISABLE(&htim15);
//		TIM14->CCR1=pulselengthmax;
	//}
//	else if(mydelay>pulsebreakmin){
//		*(uint32_t*)(dacadresse)=(uint32_t)ampli;
//		GPIOC->BSRR = (uint32_t)GPIO_PIN_6;  //PIN um ADC anzuzeigen das LD an
//	//	laserenable=0;
//		__HAL_TIM_SET_COUNTER(&htim14,0);
//		TIM14->CCR1=mydelay*dutymax/100;
//	}
	//mydelay=mycounter2-mycounter1;



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//GPIOA->BSRR = (uint32_t)GPIO_PIN_11;
//	if(!adc_stop) // If ADC is not locked
//	{
//		if(!ringbuffer_lock && !adc_restart_but_discard_last) //If ringbuffer is not locked
//		{
//			hadc->DMA_Handle->Instance->CMAR += 4*ADC_channels; //increment DMA destination by number of adc channels
//			if(hadc->DMA_Handle->Instance->CMAR >= (uint32_t)(&ADC_ringbuffer[ADC_channels*Ringbuffer_Size])) //if ringbuffer pointer exceeds its size
//			{
//				hadc->DMA_Handle->Instance->CMAR = (uint32_t)ADC_ringbuffer; //set DMA destination to ringbuffer start address
//			}
//			ADC_count++; //Increment number of conversions
//		}
//
//		__HAL_TIM_ENABLE(&htim6); //start adc trigger timer
//	}
	//adc_restart_but_discard_last = 0;
	//adc_busy = 0;
	//GPIOA->BRR = (uint32_t)GPIO_PIN_11;
	HAL_ADC_Stop_DMA(&hadc1);
	adc_ready=1;
}

//void TIM14_Callback(){
//	if(__HAL_TIM_GET_FLAG(&htim14,TIM_FLAG_CC1)==1){
//		__HAL_TIM_CLEAR_FLAG(&htim14,TIM_FLAG_CC1);
//		laserenable=0;
//		GPIOC->BRR = (uint32_t)GPIO_PIN_6;
//		*(uint32_t*)(dacadresse)=(uint32_t)dac_min;   //laser ausschalten
//
//	}
//	if(__HAL_TIM_GET_FLAG(&htim14,TIM_FLAG_UPDATE)==1){
//		__HAL_TIM_CLEAR_FLAG(&htim14,TIM_FLAG_UPDATE);
//		laserenable=1;
//		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)==GPIO_PIN_SET){
//			*(uint32_t*)(dacadresse)=(uint32_t)ampli;
//			GPIOC->BSRR = (uint32_t)GPIO_PIN_6;  //PIN um ADC anzuzeigen das LD an
//			__HAL_TIM_ENABLE(&htim14);
//		}
//	}
//}

//void TIM15_Callback(){
//	__HAL_TIM_CLEAR_FLAG(&htim15,TIM_FLAG_UPDATE);
//	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6)==GPIO_PIN_SET){
//		GPIOC->BRR = (uint32_t)GPIO_PIN_6;
//		*(uint32_t*)(dacadresse)=(uint32_t)dac_min;   //laser ausschalten
//		//mycounter1=__HAL_TIM_GET_COUNTER(&htim17);
//		__HAL_TIM_DISABLE(&htim17);
//		__HAL_TIM_SET_COUNTER(&htim17,0);
//		__HAL_TIM_ENABLE(&htim17);
//	}
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim==&htim16)
		print=1;
}
uint8_t check_serial(int first){
	if(first==serial_flash){
		return (uint8_t)1;
	}
	else{
		return 0;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void printlen(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);//setting rs485 in sending mode
	HAL_Delay(dly);
	HAL_UART_Transmit_DMA(&huart3, str, len);
	HAL_Delay(dly);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);//setting rs485 in receiving mode
	//befehl[]='\0';
}

//will decode decode tha command based on befehl[3] and befehl[4] values and assign the string to len which needs to be print
void decode_command(){
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
		//reset flash
		else if(befehl[4]=='r'){
			//reset life flash
			life_flash=100;
			bfmin_store_parameter(6,(uint64_t)life_flash);
			//reset serial flash
			serial_flash=1;
			bfmin_store_parameter(7,(uint64_t)serial_flash);
			len=sprintf(str,"Serial set to  %d; Life reset to %d",serial_flash,life_flash);
			printlen();
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
				if(laser==2){
				len = sprintf( str," temp*100;%d;Uc: %d;I mA;%lu;PD mV;%d;count;%d; dac;%d; lifetime [min];%d; restarts; %d;life*100;%d;life_flash %d;delay %d \r",temp,ADC_mean[1],(int)(current/1.8),pdsig,ADC_count, dac_value,(int)bfmin_lifetime_in_minutes,bfmin_restarts,life,life_flash,mydelay);//,M2M_PWM_CNT);
				}
				else
				len=sprintf( str," temp*100;%d;Uc: %d;I mA*3.6;%d;PD mV;%d;count;%d; dac;%d; lifetime [min];%d; restarts; %d;life*100;%d;life_flash %d;delay %d \r",temp,ADC_mean[1],current,pdsig,ADC_count, dac_value,(int)bfmin_lifetime_in_minutes,bfmin_restarts,life,life_flash,mydelay);//,M2M_PWM_CNT);
			}

			else{pflag=0;}//setting the flag again to 0 so that it does not print again if received any wrong befehl[4] value.
		}
		else{pflag=0;}//setting the flag again to 0 so that it does not print again if received any wrong befehl[4] value.
	}
	else{pflag=0;}//setting the flag again to 0 so that it does not print again if received any wrong befehl[4] value.
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(befehl[pos]=='#' && start_command==0)
	{
				start_command=1;
	}

	else if (start_command==1 && befehl[pos]!='\0'){
		pos++;
	}

//	if(befehl[pos]!='\0')
//		pos++;

	else if (befehl[pos]=='\0' && start_command==1){
		start_command=0;
		pos=0;
		if(check_serial(atoi(&befehl[0]))){
			pflag=1;
			decode_command();
			if(pflag){
				printlen();
			}
		}




		if(strstr(befehl,"lifereset")){
			bfmin_store_parameter(6,(uint64_t) 100);
		}

	}
	else{
		//for getting life curve variables
		// will give the following data whenever any data is send
		if(0){
		len = sprintf( str,"temp*100;%d;Uc: %d;I mA*3.6;%d;PD mV;%d;count;%d; dac;%d; lifetime [min];%d; restarts; %d;life*100;%d;life_flash %d;delay %d \r",temp,ADC_mean[1],current,pdsig,ADC_count, dac_value,bfmin_lifetime_in_minutes,bfmin_restarts,life,life_flash,mydelay);//,M2M_PWM_CNT);
		printlen();
		}
		//reset flash value to 100
		if(0){
		bfmin_store_parameter(6,(uint64_t) 100);
		len=sprintf(str,"Life reset to %d",life_flash);
		}
		// store serial 001
		if(0){
		serial_flash=1;
		bfmin_store_parameter(7,(uint64_t)serial_flash);
		len=sprintf(str,"Write %d",serial_flash);
		printlen();
		}
	}
	HAL_UART_Receive_IT(&huart3, &(befehl[pos]), 1);
}




void calculateinterpol(){
//	//float dac=sdmin+m*mod;
//	sdmin=2*sdhalf-sdmax;
//	m=(float)(sdmax-sdmin)/((float)(smmax-smmin));
//	bfmin_store_parameter(2,(uint64_t)sdmin);
//	bfmin_store_parameter(3,(uint64_t)(m*4096));
}

uint32_t interpolateDac(uint32_t mod){
	float dac=sdmin+m*mod;
	if(dac>4096)
		dac=4096;
	else if(mod<smmin)
		dac=0;
	return((uint32_t)dac);
}

int16_t readThermistor(uint16_t u_ntc)
{
	//uint16_t u_ntc=1000;
	const double BALANCE_RESISTOR   = 10000.0;
	//const double MAX_ADC            = 4095.0;
	const double BETA               = 3375.0;
	const double ROOM_TEMP          = 298.15;
	const double RESISTOR_ROOM_TEMP = 10000.0;
	//double currentTemperature = 0;
  // variables that live in this function
  double rThermistor = 0;            // Holds thermistor resistance value
  double tKelvin     = 0;            // Holds calculated temperature
  float tCelsius    = 0;            // Hold temperature in celsius


  rThermistor = (BALANCE_RESISTOR * (u_ref-u_ntc))/u_ntc;
  tKelvin = (BETA * ROOM_TEMP)/(BETA + (ROOM_TEMP * log(rThermistor / RESISTOR_ROOM_TEMP)));
  tCelsius = tKelvin - 273.15;
  if(tCelsius>200)
	  tCelsius=200;
  if(tCelsius<-200)
	  tCelsius=-200;
  return (int16_t)(tCelsius*100);    // Return the temperature in Celsius
  }







// Calclation of lifetime: deviation from polyfit
uint16_t lifetime(float current, float temperature){

	uint16_t life_cal=0;
	float p1;//9.6261e-07;//0.000000554738;
	float p2;//7.5319e-03;//0.00894725823;
	float p3;//3.6230e+02;//386.0153307115;
	//0.00000055473     0.00894725823   337.01533071153
//Muster-NR319 660nm 200mW
	if (laser==1){
		p1=1.8451e-06;
		p2=1.8806e-03;
		p3=3.5548e+02;

		//200mW
		/*
	 p1=7.9674e-07; 	//3.6321e-06;//9.6261e-07;//0.000000554738;
	 p2=7.1406e-03;		//-1.0418e-02;//7.5319e-03;//0.00894725823;
	 p3=4.2635e+02;		//4.5571e+02;//3.6230e+02;//386.0153307115;
	 */
	}
//Muster-NR326 520nm 100mW
	if (laser==2){
	 p1=2.6956e-06        ;//9.6261e-07;//0.000000554738;
	 p2=  3.8446e-03    ;//7.5319e-03;//0.00894725823;
	 p3=   4.1007e+02 ;//3.6230e+02;//386.0153307115;
	}

//Muster-NR328 405nm 100mW
	if (laser==3){
	 p1= 1.6859e-06;//9.6261e-07;//0.000000554738;
	 p2= 2.4674e-03;//7.5319e-03;//0.00894725823;
	 p3= 4.0913e+02;//3.6230e+02;//386.0153307115;
	}
//1.0793e-06   6.3147e-03   3.5973e+02


	float currentc;

	currentc=p1*temperature*temperature+p2*temperature+p3;
	life_cal=(u_int16_t)(10000-50000*(current-currentc)/current);
//	if(life_cal>10000);
//		life_cal=10000;
	return life_cal;
}



uint16_t tempcomp(float current, float temperature){


	float p1=0.0000076468;
	float p2= -0.0063316042;
	float p3=1051.1463683339;
	float currentc;

	currentc=current+(temperature-3000)*0.01;
	return (u_int16_t)(currentc);
}











/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
