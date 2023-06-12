/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sys_sensors.c
  * @author  MCD Application Team
  * @brief   Manages the sensors on the application
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stdint.h"
#include "sys_conf.h"
#include "sys_sensors.h"
#include "SensiLora.h"

/* USER CODE BEGIN Includes */
#include "utilities_conf.h"
#include "sys_app.h"
#include "FlashManager.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
#define STSOP_LATTITUDE           ((float) 43.618622 )  /*!< default latitude position */
#define STSOP_LONGITUDE           ((float) 7.051415  )  /*!< default longitude position */
#define MAX_GPS_POS               ((int32_t) 8388607 )  /*!< 2^23 - 1 */
#define HUMIDITY_DEFAULT_VAL      50.0f                 /*!< default humidity */
#define TEMPERATURE_DEFAULT_VAL   18.0f                 /*!< default temperature */
#define PRESSURE_DEFAULT_VAL      1000.0f               /*!< default pressure */

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

/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Read Data Sensors
 * @param  sensor_data - pointer to sensor_t structure
 * @retval None
 */
void Sensors_Read(sensor_t *sensor_data)
{
  /* USER CODE BEGIN EnvSensors_Read_1 */
  float humValue = 0;
  float tempValue = 0;
  float pressValue = 0;
  SensorAxes_t accAxes = {0};
  SensorAxes_t gyroAxes = {0};
  SensorAxes_t magAxes = {0};
  uint32_t lightValue = 0;
  FlashEnableSens_t dataEnableSens;
  FlashMemoryGetEnableStatusSens(&dataEnableSens);

  if (dataEnableSens.mTemp) {
      SensiLoraSensorGetValue(SENSI_HTS221, SENSI_TEMPERATURE, &tempValue);
  }
  if (dataEnableSens.mHum) {
      SensiLoraSensorGetValue(SENSI_HTS221, SENSI_HUMIDITY, &humValue);
  }
  if (dataEnableSens.mPress) {
      SensiLoraSensorGetValue(SENSI_LPS22HH, SENSI_PRESSURE, &pressValue);
  }
  if (dataEnableSens.mAcc) {
      SensiLoraSensorGetValue(SENSI_LSM6DSOX, SENSI_ACCEL, (float*)&accAxes);
  }
  if (dataEnableSens.mGyro) {
      SensiLoraSensorGetValue(SENSI_LSM6DSOX, SENSI_GYR, (float*)&gyroAxes);
  }
  if (dataEnableSens.mMag) {
      SensiLoraSensorGetValue(SENSI_LIS2MDL, SENSI_MAG, (float*)&magAxes);
  }
  if (dataEnableSens.mLight) {
      SensiLoraSensorGetValue(SENSI_APDS9250, SENSI_LIGHT, (float*)&lightValue);
  }

  sensor_data->humidity    = humValue;
  sensor_data->temperature = tempValue;
  sensor_data->pressure    = pressValue;

  sensor_data->latitude  = (int32_t)((STSOP_LATTITUDE  * MAX_GPS_POS) / 90);
  sensor_data->longitude = (int32_t)((STSOP_LONGITUDE  * MAX_GPS_POS) / 180);

  sensor_data->accelAxes = accAxes;
  sensor_data->gyroAxes = gyroAxes;
  sensor_data->magnetoAxes = magAxes;

  sensor_data->lights = lightValue;
  /* USER CODE END EnvSensors_Read_1 */
  /* USER CODE BEGIN EnvSensors_Read_Last */

  /* USER CODE END EnvSensors_Read_Last */
}

/**
 * @brief  Init Sensors
 * @param  None
 * @retval None
 */
void Sensors_Init(void)
{
    FlashEnableSens_t dataEnableSens;

    InitFlashM();
    FlashMemoryGetEnableStatusSens(&dataEnableSens);
  /* USER CODE BEGIN EnvSensors_Init */

	if (dataEnableSens.mTemp) {
	    SensiLoraSensorInit(SENSI_HTS221, SENSI_TEMPERATURE);
	}

	if (dataEnableSens.mHum) {
	    SensiLoraSensorInit(SENSI_HTS221, SENSI_HUMIDITY);
	}

	if (dataEnableSens.mPress) {
	    SensiLoraSensorInit(SENSI_LPS22HH, SENSI_PRESSURE);
	}

	if (dataEnableSens.mAcc) {
	    SensiLoraSensorInit(SENSI_LSM6DSOX, SENSI_ACCEL);
	}

	if (dataEnableSens.mGyro) {
	    SensiLoraSensorInit(SENSI_LSM6DSOX, SENSI_GYR);
	}

	if (dataEnableSens.mMag) {
	    SensiLoraSensorInit(SENSI_LIS2MDL, SENSI_MAG);
	}

	if (dataEnableSens.mLight) {
	    SensiLoraSensorInit(SENSI_APDS9250, SENSI_LIGHT);
	}
  /* USER CODE END EnvSensors_Init_Last */
}

/**
 * @brief  Init All Sensors
 * @param  None
 * @retval None
 */
void Sensors_InitAll(void)
{
    SensiLoraSensorInit(SENSI_HTS221, SENSI_TEMPERATURE);
    SensiLoraSensorInit(SENSI_HTS221, SENSI_HUMIDITY);
    SensiLoraSensorInit(SENSI_LPS22HH, SENSI_PRESSURE);
    SensiLoraSensorInit(SENSI_LSM6DSOX, SENSI_ACCEL);
    SensiLoraSensorInit(SENSI_LSM6DSOX, SENSI_GYR);
    SensiLoraSensorInit(SENSI_LIS2MDL, SENSI_MAG);
    SensiLoraSensorInit(SENSI_APDS9250, SENSI_LIGHT);
}

/**
 * @brief  DeInit Sensors
 * @param  None
 * @retval None
 */
void Sensors_DeInit(void)
{
    SensiLoraSensorDeInit(SENSI_HTS221);
    SensiLoraSensorDeInit(SENSI_LPS22HH);
    SensiLoraSensorDeInit(SENSI_LSM6DSOX);
    SensiLoraSensorDeInit(SENSI_LIS2MDL);
    SensiLoraSensorDeInit(SENSI_APDS9250);
}

/* USER CODE BEGIN EF */
/**
 * @brief  Enable Sensors
 * @param  None
 * @retval None
 */
void Sensors_Enable(void)
{
    FlashEnableSens_t dataEnableSens;
    FlashMemoryGetEnableStatusSens(&dataEnableSens);

    if (dataEnableSens.mTemp) {
        SensiLoraSensorEnable(SENSI_HTS221, SENSI_TEMPERATURE);
    }
    if (dataEnableSens.mHum) {
        SensiLoraSensorEnable(SENSI_HTS221, SENSI_HUMIDITY);
    }
    if (dataEnableSens.mPress) {
        SensiLoraSensorEnable(SENSI_LPS22HH, SENSI_PRESSURE);
    }
    if (dataEnableSens.mAcc) {
        SensiLoraSensorEnable(SENSI_LSM6DSOX, SENSI_ACCEL);
    }
    if (dataEnableSens.mGyro) {
        SensiLoraSensorEnable(SENSI_LSM6DSOX, SENSI_GYR);
    }
    if (dataEnableSens.mMag) {
        SensiLoraSensorEnable(SENSI_LIS2MDL, SENSI_MAG);
    }
    if (dataEnableSens.mLight) {
        SensiLoraSensorEnable(SENSI_APDS9250, SENSI_LIGHT);
    }
}

/**
 * @brief  Disable Sensors
 * @param  None
 * @retval None
 */
void Sensors_Disable(void)
{
    FlashEnableSens_t dataEnableSens;
    FlashMemoryGetEnableStatusSens(&dataEnableSens);

    if (dataEnableSens.mTemp) {
        SensiLoraSensorDisable(SENSI_HTS221, SENSI_TEMPERATURE);
    }
    if (dataEnableSens.mHum) {
        SensiLoraSensorDisable(SENSI_HTS221, SENSI_HUMIDITY);
    }
    if (dataEnableSens.mPress) {
        SensiLoraSensorDisable(SENSI_LPS22HH, SENSI_PRESSURE);
    }
    if (dataEnableSens.mAcc) {
        SensiLoraSensorDisable(SENSI_LSM6DSOX, SENSI_ACCEL);
    }
    if (dataEnableSens.mGyro) {
        SensiLoraSensorDisable(SENSI_LSM6DSOX, SENSI_GYR);
    }
    if (dataEnableSens.mMag) {
        SensiLoraSensorDisable(SENSI_LIS2MDL, SENSI_MAG);
    }
    if (dataEnableSens.mLight) {
        SensiLoraSensorDisable(SENSI_APDS9250, SENSI_LIGHT);
    }
}

/**
 * @brief  Enable All Sensors
 * @param  None
 * @retval None
 */
void Sensors_EnableAll(void)
{
    SensiLoraSensorEnable(SENSI_HTS221, SENSI_TEMPERATURE);
    SensiLoraSensorEnable(SENSI_HTS221, SENSI_HUMIDITY);
    SensiLoraSensorEnable(SENSI_LPS22HH, SENSI_PRESSURE);
    SensiLoraSensorEnable(SENSI_LSM6DSOX, SENSI_ACCEL);
    SensiLoraSensorEnable(SENSI_LSM6DSOX, SENSI_GYR);
    SensiLoraSensorEnable(SENSI_LIS2MDL, SENSI_MAG);
    SensiLoraSensorEnable(SENSI_APDS9250, SENSI_LIGHT);

}

/**
 * @brief  Disable All Sensors
 * @param  None
 * @retval None
 */
void Sensors_DisableAll(void)
{
    SensiLoraSensorDisable(SENSI_HTS221, SENSI_TEMPERATURE);
    SensiLoraSensorDisable(SENSI_HTS221, SENSI_HUMIDITY);
    SensiLoraSensorDisable(SENSI_LPS22HH, SENSI_PRESSURE);
    SensiLoraSensorDisable(SENSI_LSM6DSOX, SENSI_ACCEL);
    SensiLoraSensorDisable(SENSI_LSM6DSOX, SENSI_GYR);
    SensiLoraSensorDisable(SENSI_LIS2MDL, SENSI_MAG);
    SensiLoraSensorDisable(SENSI_APDS9250, SENSI_LIGHT);
}

/**
 * @brief  Restart Sensors
 * @param  None
 * @retval None
 */
void Sensors_Restart(void)
{
    Sensors_DeInit();
    Sensors_Init();
}

/* USER CODE END EF */

/* Private Functions Definition -----------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
