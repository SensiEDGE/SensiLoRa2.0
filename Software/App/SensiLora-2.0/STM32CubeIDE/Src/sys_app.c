/**
  ******************************************************************************
  * @file    sys_app.c
  * @author  MCD Application Team
  * @brief   Initializes HW and SW system entities (not related to the radio)
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

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "platform.h"
#include "sys_app.h"
#include "adc_if.h"
#include "radio_board_if.h"
#include "stm32_systime.h"
#include "stm32_lpm.h"
#include "utilities_def.h"
#include "sys_debug.h"
#include "rtc_if.h"
#include "sys_sensors.h"

/* USER CODE BEGIN Includes */
#include "UsbService.h"
#include "SensiLoraBSP.h"
#include "usb_device.h"
#include "Usb.h"
/* USER CODE END Includes */
/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */
/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
bool mFlagUsb = false;
static bool flagStartUsbTask = false;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
#define MAX_TS_SIZE (int) 16

/**
  * @brief Timer to handle the UsbService
  */
static UTIL_TIMER_Object_t OnWorkLedTimer;
static UTIL_TIMER_Object_t OffWorkLedTimer;
static UTIL_TIMER_Object_t UsbLedOnTimer;
static UTIL_TIMER_Object_t UsbLedOffTimer;

static void OnWorkTimerLedEvent(void *context);
static void OffWorkTimerLedEvent(void *context);
static void usbLedEventOn(void *context);
static void usbLedEventOff(void *context);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief Returns sec and msec based on the systime in use
  * @param none
  * @retval  none
  */
static void TimestampNow(uint8_t *buff, uint16_t *size);

/**
  * @brief  it calls UTIL_ADV_TRACE_VSNPRINTF
  */
static void tiny_snprintf_like(char *buf, uint32_t maxsize, const char *strFormat, ...);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
/**
  * @brief initialises the system (dbg pins, trace, mbmux, systiemr, LPM, ...)
  * @param none
  * @retval  none
  */
void SystemApp_Init(void)
{
  /* USER CODE BEGIN SystemApp_Init_1 */

  /* USER CODE END SystemApp_Init_1 */

  /*Initialises timer and RTC*/
  UTIL_TIMER_Init();

  BspButtonInit(BUTTON_USER, BUTTON_MODE_EXTI);
  BspLedInit(LED_RED);
  BspLedInit(LED_GREEN);
  BspLedOn(LED_RED);

  /* Configure the debug mode*/
  BspMeasBatInit();
  BspMeasBatOn();
  HAL_Delay(5);
  BspMeasBatOff();
  HAL_Delay(100);
  /*Initialize the temperature and Battery measurement services */
  SYS_InitMeasurement();

  /*Initialize the terminal */
  UTIL_ADV_TRACE_Init();
  UTIL_ADV_TRACE_RegisterTimeStampFunction(TimestampNow);

  /*Set verbose LEVEL*/
  UTIL_ADV_TRACE_SetVerboseLevel(VERBOSE_LEVEL);

  Sx_Board_Bus_Init();
  Sx_Board_IoInit();
  /*Initialize the Sensors */
  Sensors_Init();

  /*Init low power manager*/
  UTIL_LPM_Init();
  /* Disable Stand-by mode */
  UTIL_LPM_SetOffMode((1 << CFG_LPM_APPLI_Id), UTIL_LPM_DISABLE);

#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 1)
  /* Disable Stop Mode */
  UTIL_LPM_SetStopMode((1 << CFG_LPM_APPLI_Id), UTIL_LPM_DISABLE);
#elif !defined (LOW_POWER_DISABLE)
#error LOW_POWER_DISABLE not defined
#endif /* LOW_POWER_DISABLE */
  /* USER CODE BEGIN SystemApp_Init_2 */
  UsbServiceInit();
//   Wait Usb Init
  HAL_Delay(1000);

  UTIL_TIMER_Create(&OnWorkLedTimer, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnWorkTimerLedEvent, NULL);
  UTIL_TIMER_SetPeriod(&OnWorkLedTimer, 5000);

  UTIL_TIMER_Create(&OffWorkLedTimer, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OffWorkTimerLedEvent, NULL);
  UTIL_TIMER_SetPeriod(&OffWorkLedTimer, 300);

  UTIL_TIMER_Create(&UsbLedOnTimer, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, usbLedEventOn, NULL);
  UTIL_TIMER_SetPeriod(&UsbLedOnTimer, 1500);

  UTIL_TIMER_Create(&UsbLedOffTimer, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, usbLedEventOff, NULL);
  UTIL_TIMER_SetPeriod(&UsbLedOffTimer, 100);

  UTIL_TIMER_Start(&OffWorkLedTimer);

  SystemApp_ResetFlagUsb();
  /* USER CODE END SystemApp_Init_2 */
}

/**
  * @brief  Start Task USB
  * @param  none
  * @retval none
  */
void SystemApp_StartTaskUsbService(void)
{
    flagStartUsbTask = true;
    SystemApp_StartTimerUsbLedActive();
    Sensors_InitAll();
    Sensors_EnableAll();
}

/**
  * @brief  Stop Task USB
  * @param  none
  * @retval none
  */
void SystemApp_StopTaskUsbService(void)
{
    flagStartUsbTask = false;
    SystemApp_StopTimerUsbLedDeActive();
    Sensors_Restart();
}

/**
  * @brief  Start Timer USB LED Activity
  * @param  none
  * @retval none
  */
void SystemApp_StartTimerUsbLedActive(void)
{
    UTIL_TIMER_Stop(&OffWorkLedTimer);
    UTIL_TIMER_Stop(&OnWorkLedTimer);
    UTIL_TIMER_Start(&UsbLedOnTimer);
    BspLedOff(LED_RED);
}

/**
  * @brief  Stop Timer USB LED Activity
  * @param  none
  * @retval none
  */
void SystemApp_StopTimerUsbLedDeActive(void)
{
    UTIL_TIMER_Stop(&UsbLedOnTimer);
    UTIL_TIMER_Stop(&UsbLedOffTimer);
    BspLedOff(LED_GREEN);
    UTIL_TIMER_Start(&OnWorkLedTimer);
}

/**
  * @brief  Stop All Timer
  * @param  none
  * @retval none
  */
void SystemApp_StopAllTimer(void)
{
    UTIL_TIMER_Stop(&UsbLedOnTimer);
    UTIL_TIMER_Stop(&UsbLedOffTimer);
    UTIL_TIMER_Stop(&OnWorkLedTimer);
}

/**
  * @brief Check and Go to Sleep Mode
  * @param none
  * @retval  none
  */
void SystemApp_Idle(void)
{
  /* USER CODE BEGIN UTIL_SEQ_Idle_1 */
    if (mFlagUsb && isActiveUsb() == false) {
        //When connecting to a computer, a delay is used to initialize the USB
        HAL_Delay(1000);
        mFlagUsb = false;

    } else {
        UTIL_LPM_EnterLowPower();
    }
  /* USER CODE END UTIL_SEQ_Idle_1 */
  /* USER CODE BEGIN UTIL_SEQ_Idle_2 */

  /* USER CODE END UTIL_SEQ_Idle_2 */
}

/**
  * @brief  Reset Flag activity USB
  * @param  none
  * @retval none
  */
void SystemApp_ResetFlagUsb(void)
{
    mFlagUsb = false;
}

/**
  * @brief  callback to get the battery level in % of full charge (254 full charge, 0 no charge)
  * @param  none
  * @retval battery level
  */
uint16_t SystemApp_GetBatteryLevel(void)
{
  uint16_t batteryLevel = 0;

  /* USER CODE BEGIN GetBatteryLevel_0 */
  BspMeasBatOn();
  /* USER CODE END GetBatteryLevel_0 */

  batteryLevel = (uint16_t) SYS_GetBatteryLevel();

  if (batteryLevel > VDD_BAT)
  {
    batteryLevel = VDD_BAT;
  }
  else if (batteryLevel < VDD_MIN)
  {
	  batteryLevel = VDD_MIN;
  }

  /* USER CODE BEGIN GetBatteryLevel_2 */
  BspMeasBatOff();
  /* USER CODE END GetBatteryLevel_2 */

  return batteryLevel;
}

/**
  * @brief  callback to get the current temperature in the MCU
  * @param  none
  * @retval temperature level
  */
uint16_t SystemApp_GetTemperatureLevel(void)
{
  uint16_t temperatureLevel = 0;

  temperatureLevel = (uint16_t)(SYS_GetTemperatureLevel() / 256);
  /* USER CODE BEGIN GetTemperatureLevel */

  /* USER CODE END GetTemperatureLevel */
  return temperatureLevel;
}


/**
  * @brief  callback to get the board 64 bits unique ID
  * @param  unique ID
  * @retval none
  */
void SystemApp_GetUniqueId(uint8_t *id)
{
  /* USER CODE BEGIN GetUniqueId_1 */

  /* USER CODE END GetUniqueId_1 */
  uint32_t ID_1_3_val = HAL_GetUIDw0() + HAL_GetUIDw2();
  uint32_t ID_2_val = HAL_GetUIDw1();

  id[7] = (ID_1_3_val) >> 24;
  id[6] = (ID_1_3_val) >> 16;
  id[5] = (ID_1_3_val) >> 8;
  id[4] = (ID_1_3_val);
  id[3] = (ID_2_val) >> 24;
  id[2] = (ID_2_val) >> 16;
  id[1] = (ID_2_val) >> 8;
  id[0] = (ID_2_val);

  /* USER CODE BEGIN GetUniqueId_2 */

  /* USER CODE END GetUniqueId_2 */
}

/**
  * @brief  callback to get the board 32 bits unique ID (LSB)
  * @param  none
  * @retval devAddr Device Address
  */
uint32_t SystemApp_GetDevAddr(void)
{
  return ((HAL_GetUIDw0()) ^ (HAL_GetUIDw1()) ^ (HAL_GetUIDw2()));
}

/* USER CODE BEGIN ExF */

/* USER CODE END ExF */

/* Private functions ---------------------------------------------------------*/
static void TimestampNow(uint8_t *buff, uint16_t *size)
{
  /* USER CODE BEGIN TimestampNow_1 */

  /* USER CODE END TimestampNow_1 */
  SysTime_t curtime = SysTimeGet();
  tiny_snprintf_like((char *)buff, MAX_TS_SIZE, "%ds%03d:", curtime.Seconds, curtime.SubSeconds);
  *size = strlen((char *)buff);
  /* USER CODE BEGIN TimestampNow_2 */

  /* USER CODE END TimestampNow_2 */
}

/* Disable StopMode when traces need to be printed */
void UTIL_ADV_TRACE_PreSendHook(void)
{
  /* USER CODE BEGIN UTIL_ADV_TRACE_PreSendHook_1 */

  /* USER CODE END UTIL_ADV_TRACE_PreSendHook_1 */
  UTIL_LPM_SetStopMode((1 << CFG_LPM_UART_TX_Id), UTIL_LPM_DISABLE);
  /* USER CODE BEGIN UTIL_ADV_TRACE_PreSendHook_2 */

  /* USER CODE END UTIL_ADV_TRACE_PreSendHook_2 */
}
/* Re-enable StopMode when traces have been printed */
void UTIL_ADV_TRACE_PostSendHook(void)
{
  /* USER CODE BEGIN UTIL_LPM_SetStopMode_1 */

  /* USER CODE END UTIL_LPM_SetStopMode_1 */
  UTIL_LPM_SetStopMode((1 << CFG_LPM_UART_TX_Id), UTIL_LPM_ENABLE);
  /* USER CODE BEGIN UTIL_LPM_SetStopMode_2 */

  /* USER CODE END UTIL_LPM_SetStopMode_2 */
}

static void tiny_snprintf_like(char *buf, uint32_t maxsize, const char *strFormat, ...)
{
  /* USER CODE BEGIN tiny_snprintf_like_1 */

  /* USER CODE END tiny_snprintf_like_1 */
  va_list vaArgs;
  va_start(vaArgs, strFormat);
  UTIL_ADV_TRACE_VSNPRINTF(buf, maxsize, strFormat, vaArgs);
  va_end(vaArgs);
  /* USER CODE BEGIN tiny_snprintf_like_2 */

  /* USER CODE END tiny_snprintf_like_2 */
}

/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */
/* HAL overload functions ---------------------------------------------------------*/

/**
  * @brief This function configures the source of the time base.
  * @brief  don't enable systick
  * @param TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  /*Don't enable SysTick if TIMER_IF is based on other counters (e.g. RTC) */
  /* USER CODE BEGIN HAL_InitTick_1 */

  /* USER CODE END HAL_InitTick_1 */
  return HAL_OK;
  /* USER CODE BEGIN HAL_InitTick_2 */

  /* USER CODE END HAL_InitTick_2 */
}

/**
  * @brief Provide a tick value in millisecond measured using RTC
  * @note This function overwrites the __weak one from HAL
  * @retval tick value
  */
uint32_t HAL_GetTick(void)
{
  /* USER CODE BEGIN HAL_GetTick_1 */

  /* USER CODE END HAL_GetTick_1 */
  return RTC_IF_GetTimerValue();
  /* USER CODE BEGIN HAL_GetTick_2 */

  /* USER CODE END HAL_GetTick_2 */
}

/**
  * @brief This function provides delay (in ms)
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(__IO uint32_t Delay)
{
  /* USER CODE BEGIN HAL_Delay_1 */

  /* USER CODE END HAL_Delay_1 */
  RTC_IF_DelayMs(Delay);   /* based on RTC */
  /* USER CODE BEGIN HAL_Delay_2 */

  /* USER CODE END HAL_Delay_2 */
}

/* USER CODE BEGIN Overload_HAL_weaks */
/**
  * @brief  USB task Process
  * @param  none
  * @retval none
  */
void SystemApp_UsbProcess(void)
{
    if (flagStartUsbTask) {
        UsbTask();
    }
}

/**
  * @brief  Event USB LED On
  * @param  context - pointer on Timer context
  * @retval none
  */
static void usbLedEventOn(void *context)
{
    BspLedOn(LED_GREEN);
    UTIL_TIMER_Start(&UsbLedOffTimer);
}

/**
  * @brief  Event USB LED off
  * @param  context - pointer on Timer context
  * @retval none
  */
static void usbLedEventOff(void *context)
{
    BspLedOff(LED_GREEN);
    UTIL_TIMER_Start(&UsbLedOnTimer);
}

/**
  * @brief  Event Off Work LED
  * @param  context - pointer on Timer context
  * @retval none
  */
static void OffWorkTimerLedEvent(void *context)
{
  /* USER CODE BEGIN OnTxTimerLedEvent_1 */

  /* USER CODE END OnTxTimerLedEvent_1 */
    BspLedOff(LED_RED);
    UTIL_TIMER_Start(&OnWorkLedTimer);
  /* USER CODE BEGIN OnTxTimerLedEvent_2 */

  /* USER CODE END OnTxTimerLedEvent_2 */
}

/**
  * @brief  Event On Work LED
  * @param  context - pointer on Timer context
  * @retval none
  */
static void OnWorkTimerLedEvent(void *context)
{
  /* USER CODE BEGIN OnTxTimerLedEvent_1 */

  /* USER CODE END OnTxTimerLedEvent_1 */
    BspLedOn(LED_RED);
    UTIL_TIMER_Start(&OffWorkLedTimer);
  /* USER CODE BEGIN OnTxTimerLedEvent_2 */

  /* USER CODE END OnTxTimerLedEvent_2 */
}

/* USER CODE END Overload_HAL_weaks */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

