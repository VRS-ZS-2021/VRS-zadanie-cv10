/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void proccesDmaData(uint8_t sign);
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

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  USART2_RegisterCallback(proccesDmaData); //DMA callback init

   dutyCycle = 0; //default duty cycle
   mode = 0;
   countUpDown = 1;


   /*While loop definitions*/
  char data_to_send[100];
  char actual_data_buffer[256];
  strcpy(actual_data_buffer,"");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  dutyCycle = 99;
	  LL_mDelay(1000);
	  dutyCycle = 30;
	  //Sending info about buffer capacity status
	  	  uint16_t buffer_state = getBufferState();
	  	  float buffer_percentage = (float) (buffer_state) / (float) (DMA_USART2_BUFFER_SIZE)*100;
	  	  sprintf(data_to_send, "Buffer capacity: %d bytes, occupied memory: %d bytes, load [in %%]:%.2f%%\r\n", DMA_USART2_BUFFER_SIZE, buffer_state, buffer_percentage);

	  	  USART2_PutBuffer((uint8_t *) data_to_send, strlen(data_to_send));
	  	  LL_mDelay(1000);
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(8000000);
  LL_SetSystemCoreClock(8000000);
}

/* USER CODE BEGIN 4 */
void proccesDmaData(uint8_t sign)
{
	static uint8_t allow = 1;
	static uint8_t calculate_sign = 0;
	static uint8_t lowercase_char = 0;
	static uint8_t uppercase_char = 0;

		// type your algorithm here:
	char data_number_to_send[100];
	if(sign == '\r') return; //filtering endline character from PuTTY


	if(sign == '#' && allow == 0){ //zapneme povolenie pre citanie znakov, startovaci znak je prijaty len raz
		allow = 1;
		calculate_sign = 0;
		lowercase_char = 0;
		uppercase_char = 0;
	}
	if(sign == '$' && allow == 1){ // po prijati ukoncovacieho znaku vypneme povolenie a vypiseme data
		allow = 0;
		sprintf(data_number_to_send, "Number of lowercase characters: %d , Number of uppercase characters: %d\r\n", lowercase_char,uppercase_char);
		USART2_PutBuffer((uint8_t *) data_number_to_send, strlen(data_number_to_send));
	}
	if(calculate_sign > 35){//ak sme prekrocili pocet znakov o 35, prestaneme znaky ratat a zahodime data
		allow = 0;
		calculate_sign = 0;
		lowercase_char = 0;
		uppercase_char = 0;
	}
	if (allow == 1){//ratanie malych/velkych pismen
		calculate_sign++;
		if(sign >= 'a' && sign <= 'z'){
			lowercase_char++;
		}
		if(sign >= 'A' && sign <= 'Z'){
			uppercase_char++;
		}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
