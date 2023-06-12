/**
  ******************************************************************************
  * @file    sys_sensors.h
  * @author  MCD Application Team
  * @brief   Header for sensors application
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
#ifndef __SENSORS_H__
#define __SENSORS_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "sensor.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/**
  * Sensor data parameters
  */
typedef struct
{
  float pressure;         /*!< in mbar */
  float temperature;      /*!< in degC */
  float humidity;         /*!< in % */
  int32_t latitude;       /*!< latitude converted to binary */
  int32_t longitude ;     /*!< longitude converted to binary */
  int16_t altitudeGps;    /*!< in m */
  int16_t altitudeBar ;   /*!< in m * 10 */
  SensorAxes_t accelAxes;
  SensorAxes_t gyroAxes;
  SensorAxes_t magnetoAxes;
  uint32_t lights;
  uint32_t IR;
  uint32_t RGB;
  uint32_t MIC;
  /**more may be added*/
  /* USER CODE BEGIN sensor_t */

  /* USER CODE END sensor_t */
} sensor_t;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
#if defined (SENSOR_ENABLED) && (SENSOR_ENABLED == 1) && defined (X_NUCLEO_IKS01A2)
#define HTS221_0    0U
#define LPS22HB_0   1U
#endif /* SENSOR_ENABLED & X_NUCLEO_IKS01A2 */

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  initialises the environmental sensor
  */
/**
 * @brief  Init Sensors
 * @param  None
 * @retval None
 */
void Sensors_Init(void);

/**
 * @brief  Init All Sensors
 * @param  None
 * @retval None
 */
void Sensors_InitAll(void);

/**
 * @brief  DeInit Sensors
 * @param  None
 * @retval None
 */
void Sensors_DeInit(void);

/**
 * @brief  Restart Sensors
 * @param  None
 * @retval None
 */
void Sensors_Restart(void);

/**
 * @brief  Read Data Sensors
 * @param  sensor_data - pointer to sensor_t structure
 * @retval None
 */
void Sensors_Read(sensor_t *sensor_data);

/**
 * @brief  Enable Sensors
 * @param  None
 * @retval None
 */
void Sensors_Enable(void);

/**
 * @brief  Disable Sensors
 * @param  None
 * @retval None
 */
void Sensors_Disable(void);

/**
 * @brief  Enable All Sensors
 * @param  None
 * @retval None
 */
void Sensors_EnableAll(void);

/**
 * @brief  Disable All Sensors
 * @param  None
 * @retval None
 */
void Sensors_DisableAll(void);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __SENSORS_H__ */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
