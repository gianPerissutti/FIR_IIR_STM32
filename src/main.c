/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "filtros_digitales.h"
#include "usbd_cdc_if.h"
#include "low_pass_IIR.h"
#include "low_pass_FIR.h"
#include "band_pass_IIR.h"
#include "band_pass_FIR.h"
#include "high_pass_IIR.h"
#include "high_pass_FIR.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define MAX_SIZE 40
#define BUFFER_SIZE 250
#define LOW_PASS  0
#define BAND_PASS 1
#define HIGH_PASS 2
#define FIR		  0
#define IIR		  1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint16_t adc_val= 0;
uint16_t adc_conversion_complete = 0;
float x[MAX_SIZE];
float y[MAX_SIZE];
uint8_t get_data = 0;
uint8_t send_data= 0;
float buffer_data[BUFFER_SIZE+2];
uint16_t buffer_index =0;

uint8_t filter_band = LOW_PASS;
uint8_t filter_type = FIR;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
int filter_type1(void);
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
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_ADC_Start_IT(&hadc1);
  HAL_DAC_Start(&hdac, DAC1_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  int16_t output_volt = 0;
  char msg_data[25];

  while (1)
  {

	 if(send_data)
	 {
		 for(unsigned int i = 0; i<BUFFER_SIZE; i++)
		 {
			sprintf(msg_data, "%2.3f \n", buffer_data[i]);
			CDC_Transmit_FS(msg_data,strlen(msg_data));
			HAL_Delay(50);
		 }
		 send_data = 0;
	 }
	 //FIR-----------------------------------------------------
	 if((FIR == filter_type) && (!send_data))//FIR
	 {

		 if(LOW_PASS == filter_band)
		 {
			 for(unsigned int i= 0; i<MAX_SIZE;i++)
			   {
			 	  y[i]=0.0;
			 	  x[i]=0.0;
			   }
		 }
		 while((LOW_PASS == filter_band) && (FIR == filter_type) && (!send_data))
		 {
			 if(adc_conversion_complete)
			 {
				 x[0] = ((adc_val-512)*1.15)/1024.0f;
				 FIR_filter(x, y,b_low_FIR,order_low_FIR);
				 output_volt = ((y[0]*255-127)/1.15);
				 HAL_DAC_SetValue(&hdac,DAC1_CHANNEL_1, DAC_ALIGN_8B_R,output_volt );
				 adc_conversion_complete= 0;
			 }
		 }
		 if((BAND_PASS==filter_band) && (!send_data))
		 {
			 for(unsigned int i= 0; i<MAX_SIZE;i++)
		 	 {
				 y[i]=0.0;
				 x[i]=0.0;
		 	 }
		 }

		 while((BAND_PASS==filter_band) && (FIR == filter_type)&& (!send_data))
		{
			 if(adc_conversion_complete)
			 {
				 x[0] = ((adc_val-512)*1.15)/1024.0f;
				 FIR_filter(x, y,b_band_FIR,order_band_FIR);
				 output_volt = ((y[0]*255-127)/1.15);
				 HAL_DAC_SetValue(&hdac,DAC1_CHANNEL_1, DAC_ALIGN_8B_R,output_volt );
				 adc_conversion_complete= 0;
			 }
		}
		 if(HIGH_PASS ==filter_band)
		{
			 for(unsigned int i= 0; i<MAX_SIZE;i++)
			{
				 y[i]=0.0;
				 x[i]=0.0;
			}
		}

		 while((HIGH_PASS==filter_band) && (FIR == filter_type) && (!send_data))
		{
			 if(adc_conversion_complete)
			 {
				 x[0] = ((adc_val-512)*1.15)/1024.0f;
				 FIR_filter(x, y,b_high_FIR, order_high_FIR);
				 output_volt = ((y[0]*255-127)/1.15);
				 HAL_DAC_SetValue(&hdac,DAC1_CHANNEL_1, DAC_ALIGN_8B_R,output_volt );
				 adc_conversion_complete= 0;
			 }
		}

	 }
	 //IIR-----------------------------------------------------
	 if( (IIR == filter_type) && (!send_data))
	 	 {//IIR

	 		 if(LOW_PASS == filter_band)
	 		 {
	 			 for(unsigned int i= 0; i<MAX_SIZE;i++)
	 			   {
	 			 	  y[i]=0.0;
	 			 	  x[i]=0.0;
	 			   }
	 		 }
	 		 while((LOW_PASS == filter_band) && (IIR == filter_type) && (!send_data))
	 		 {
	 			 if(adc_conversion_complete)
	 			 {
	 				 x[0] = ((adc_val-512)*1.15)/1024.0f;
	 				 IIR_filter(x, y,a_low_IIR,b_low_IIR, order_low_IIR);
	 				 output_volt = ((y[0]*255-127)/1.15);
	 				 HAL_DAC_SetValue(&hdac,DAC1_CHANNEL_1, DAC_ALIGN_8B_R,output_volt );
	 				 adc_conversion_complete= 0;
	 			 }
	 		 }
	 		 if(BAND_PASS==filter_band)
	 		 {
	 			 for(unsigned int i= 0; i<MAX_SIZE;i++)
	 		 	 {
	 				 y[i]=0.0;
	 				 x[i]=0.0;
	 		 	 }
	 		 }

	 		 while((BAND_PASS == filter_band) && (IIR == filter_type) && (!send_data))
	 		{
	 			 if(adc_conversion_complete)
	 			 {
	 				 x[0] = ((adc_val-512)*1)/1024.0f;
	 				 IIR_filter(x, y,a_bpass_IIR,b_bpass_IIR, order_bpass_IIR);
	 				 output_volt = ((y[0]*255-127)/1);
	 				 HAL_DAC_SetValue(&hdac,DAC1_CHANNEL_1, DAC_ALIGN_8B_R,output_volt );
	 				 adc_conversion_complete= 0;
	 			 }
	 		}
	 		 if((HIGH_PASS==filter_band) && (!send_data))
	 		{
	 			 for(unsigned int i= 0; i<MAX_SIZE;i++)
	 			{
	 				 y[i]=0.0;	 				 x[i]=0.0;
	 			}
	 		}

	 		 while(HIGH_PASS==filter_band && (IIR == filter_type) && (!send_data))
	 		{
	 			 if(adc_conversion_complete)
	 			 {
	 				 x[0] = ((adc_val-512)*1.15)/1024.0f;
	 				 IIR_filter(x, y,a_high_IIR,b_high_IIR, order_high_IIR);
	 				 output_volt = ((y[0]*255-127)/1.15);
	 				 HAL_DAC_SetValue(&hdac,DAC1_CHANNEL_1, DAC_ALIGN_8B_R,output_volt );
	 				 adc_conversion_complete= 0;
	 			 }
	 		}

	 	 }



  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */






  }
  /* USER CODE END 3 */


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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : filter_switch_b1_Pin filter_switch_b0_Pin fir_iir_switch_Pin */
  GPIO_InitStruct.Pin = filter_switch_b1_Pin|filter_switch_b0_Pin|fir_iir_switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	adc_val = HAL_ADC_GetValue(&hadc1);
	adc_conversion_complete= 1;
	if(get_data==1)
	{
		buffer_data[buffer_index]= y[0];
		buffer_index++;
		if(buffer_index >= BUFFER_SIZE)
		{
			get_data= 0;
			send_data = 1;
			buffer_index= 0;
		}
	}

}

int filter_type1()
{
	int ret = 0;
	if((!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)) && (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)))
	{
		ret = LOW_PASS;
	}
	if((HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)) && (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)))
	{
		ret = BAND_PASS;
	}
	if((HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)) && (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)))
	{
		ret = BAND_FILTER;
	}
	return ret;
}

void CDC_ReceiveCallBack(uint8_t *buf, uint32_t len)
{
	char *buffer_recibido = buf;
	char* msg;
	if((strcmp(buffer_recibido,"data_req") == 0) && (!send_data))
	{
		get_data = 1;
		msg = "Obteniendo datos \n\r\0";
		CDC_Transmit_FS(msg,strlen((char* )msg));
	}
	if((strcmp(buffer_recibido,"IIR_req") == 0) && (!send_data))
	{
		filter_type = IIR;
		msg ="Se cambio a IIR \n\0";
		CDC_Transmit_FS(msg,strlen((char* )msg));
	}
	if((strcmp(buffer_recibido,"FIR_req") == 0) && (!send_data))
	{
		filter_type = FIR;
		msg ="Se cambio a FIR \n\0";
		CDC_Transmit_FS(msg,strlen((char* )msg));
	}
	if((strcmp(buffer_recibido,"LPf_req") == 0) && (!send_data))
	{
		filter_band = LOW_PASS;
		msg ="Se cambio a pasabajos";
		CDC_Transmit_FS(msg,strlen((char* )msg));
	}
	if((strcmp(buffer_recibido,"BPf_req") == 0) && (!send_data))
	{
		msg ="Se cambio a pasabanda";
		filter_band = BAND_PASS;
		CDC_Transmit_FS(msg,strlen((char* )msg));
	}
	if((strcmp(buffer_recibido,"HPf_req") == 0) && (!send_data))
	{
		msg ="Se cambio a pasaalto";
		filter_band = HIGH_PASS;
		CDC_Transmit_FS(msg,strlen((char* )msg));
	}
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
  __disable_irq();
  while (1)
  {

  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
