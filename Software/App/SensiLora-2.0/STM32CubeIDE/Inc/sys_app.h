/**
  ******************************************************************************
  * @file    sys_app.h
  * @author  MCD Application Team
  * @brief   Function prototypes for sys_app.c file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYS_APP_H__
#define __SYS_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "sys_conf.h"
#include "stm32_adv_trace.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
#define APP_PPRINTF(...)  do{ } while( UTIL_ADV_TRACE_OK \
                              != UTIL_ADV_TRACE_COND_FSend(VLEVEL_ALWAYS, T_REG_OFF, TS_OFF, __VA_ARGS__) ) /* Polling Mode */
#define APP_TPRINTF(...)   do{ {UTIL_ADV_TRACE_COND_FSend(VLEVEL_ALWAYS, T_REG_OFF, TS_ON, __VA_ARGS__);} }while(0); /* with timestamp */
#define APP_PRINTF(...)   do{ {UTIL_ADV_TRACE_COND_FSend(VLEVEL_ALWAYS, T_REG_OFF, TS_OFF, __VA_ARGS__);} }while(0);

#if defined (APP_LOG_ENABLED) && (APP_LOG_ENABLED == 1)
#define APP_LOG(TS,VL,...)   do{ {UTIL_ADV_TRACE_COND_FSend(VL, T_REG_OFF, TS, __VA_ARGS__);} }while(0);
#elif defined (APP_LOG_ENABLED) && (APP_LOG_ENABLED == 0) /* APP_LOG disabled */
#define APP_LOG(TS,VL,...)
#else
#error "APP_LOG_ENABLED not defined or out of range <0,1>"
#endif /* APP_LOG_ENABLED */

#define LED_Init( x )                   BSP_LED_Init( x );
#define LED_Toggle( x )                 BSP_LED_Toggle( x );
#define LED_On( x )                     BSP_LED_On( x );
#define LED_Off( x )                    BSP_LED_Off( x );

/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief initialises the system (dbg pins, trace, systiemr, LPM, ...)
  * @param none
  * @retval  none
  */
void SystemApp_Init(void);

/**
  * @brief  callback to get the battery level in % of full charge (254 full charge, 0 no charge)
  * @param  none
  * @retval battery level
  */
uint16_t SystemApp_GetBatteryLevel(void);

/**
  * @brief  callback to get the current temperature in the MCU
  * @param  none
  * @retval temperature level
  */
uint16_t SystemApp_GetTemperatureLevel(void);

/**
  * @brief  callback to get the board 64 bits unique ID
  * @param  unique ID
  * @retval none
  */
void SystemApp_GetUniqueId(uint8_t *id);

/**
  * @brief  callback to get the board 32 bits unique ID (LSB)
  * @param  none
  * @retval devAddr Device Address
  */
uint32_t SystemApp_GetDevAddr(void);

/* USER CODE BEGIN EFP */

/**
  * @brief  Reset Flag activity USB
  * @param  none
  * @retval none
  */
void SystemApp_ResetFlagUsb(void);

/**
  * @brief  Start Task USB
  * @param  none
  * @retval none
  */
void SystemApp_StartTaskUsbService(void);

/**
  * @brief  Stop Task USB
  * @param  none
  * @retval none
  */
void SystemApp_StopTaskUsbService(void);

/**
  * @brief  Start Timer USB LED Activity
  * @param  none
  * @retval none
  */
void SystemApp_StartTimerUsbLedActive(void);

/**
  * @brief  Stop Timer USB LED Activity
  * @param  none
  * @retval none
  */
void SystemApp_StopTimerUsbLedDeActive(void);

/**
  * @brief  Stop All Timer
  * @param  none
  * @retval none
  */
void SystemApp_StopAllTimer(void);

/**
  * @brief  USB task Process
  * @param  none
  * @retval none
  */
void SystemApp_UsbProcess(void);

/**
  * @brief  Start Task Usb Service
  * @param  none
  * @retval none
  */
void SystemApp_StartTaskUsbService(void);

/**
  * @brief Check and Go to Sleep Mode
  * @param none
  * @retval  none
  */
void SystemApp_Idle(void);

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __SYS_APP_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
