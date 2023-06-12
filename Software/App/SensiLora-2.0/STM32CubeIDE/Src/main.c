 /*!
 * Timer to handle the application Tx Led to toggle
 */
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_lorawan.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SensiLoraBSP.h"
#include "UsbService.h"
#include "lora_app.h"
#include "sys_app.h"
#include "stm32_lpm.h"
#include "radio_board_if.h"
#include "stm32_timer.h"
#include "sys_sensors.h"
#include "Usb.h"
#include "sx1276.h"
#include "LmHandler.h"
#include "adc.h"
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
/* USER CODE BEGIN PFP */
static void powerOffDevice(void);
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

  /* Reset USB pins */
  GPIO_InitTypeDef GPIO_InitStruct;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_LoRaWAN_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
      MX_LoRaWAN_Process();
    /* USER CODE BEGIN 3 */

      HAL_Delay(1);
      if (BspIsPowerOff()) {
          powerOffDevice();
      }
  }
  /* USER CODE END 3 */
}

void handlerTask(void)
{

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	  /* Set Voltage scale1 as MCU will run at 32MHz */
	  __HAL_RCC_PWR_CLK_ENABLE();
	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI |RCC_OSCILLATORTYPE_HSI48;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_6;
	  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
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
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Power Off Device
  * @param  None
  * @retval None
  */
void powerOffDevice(void)
{
    SystemApp_StopAllTimer();
    LoRaWAN_StopTimer();
    BspLedOff(LED_RED);
    BspLedOn(LED_GREEN);
    HAL_Delay(2000);
    LmHandlerStop();
    SX1276SetSleep();
    BspLedOff(LED_GREEN);

    UTIL_TIMER_DeInit();

    uint32_t rimask_bit= __get_PRIMASK();
    Sx_Board_IoDeInit();
    HAL_ADC_MspDeInit(&hadc1);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

    __set_PRIMASK(rimask_bit);
    SET_BIT(PWR->CR, PWR_CR_CWUF);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);

    NVIC_SystemReset();
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    BspLedOn(LED1);
    BspLedOff(LED2);

  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
      BspLedToggle(LED1);
      HAL_Delay(500);
      BspLedToggle(LED2);
      HAL_Delay(500);
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
  while (1)
  {
  }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
