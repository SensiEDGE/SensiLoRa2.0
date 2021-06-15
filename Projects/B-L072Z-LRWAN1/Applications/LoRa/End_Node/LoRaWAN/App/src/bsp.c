/**
 ******************************************************************************
 * @file    bsp.c
 * @author  MCD Application Team
 * @brief   manages the sensors on the application
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
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
#include <string.h>
#include <stdlib.h>
#include "hw.h"
#include "timeServer.h"
#include "bsp.h"
#if defined(LRWAN_NS1)
#include "lrwan_ns1_humidity.h"
#include "lrwan_ns1_pressure.h"
#include "lrwan_ns1_temperature.h"
#else  /* not LRWAN_NS1 */
#if defined(SENSOR_ENABLED)
#if defined (X_NUCLEO_IKS01A1)
#warning "Do not forget to select X_NUCLEO_IKS01A1 files group instead of X_NUCLEO_IKS01A2"
#include "x_nucleo_iks01a1_humidity.h"
#include "x_nucleo_iks01a1_pressure.h"
#include "x_nucleo_iks01a1_temperature.h"
#else  /* not X_NUCLEO_IKS01A1 */
#include "x_nucleo_iks01a2_humidity.h"
#include "x_nucleo_iks01a2_pressure.h"
#include "x_nucleo_iks01a2_temperature.h"
#include "x_nucleo_iks01a2_accelero.h"
#include "x_nucleo_iks01a2_gyro.h"
#include "x_nucleo_iks01a2_magneto.h"
#include "x_nucleo_iks01a2_light.h"
#endif  /* X_NUCLEO_IKS01A1 */
#endif  /* SENSOR_ENABLED */
#endif  /* LRWAN_NS1 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define STSOP_LATTITUDE ((float) 43.618622 )
#define STSOP_LONGITUDE ((float) 7.051415  )
#define MAX_GPS_POS ((int32_t) 8388607  ) // 2^23 - 1
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
#if defined(SENSOR_ENABLED) || defined (LRWAN_NS1)
void* HUMIDITY_handle = NULL;
void* TEMPERATURE_handle = NULL;
void* PRESSURE_handle = NULL;
void* ACCELERO_handle = NULL;
void* GYRO_handle = NULL;
void* MAGNETO_handle = NULL;
void* LIGHT_handle = NULL;
#endif

void BSP_sensor_Read(sensor_t* sensor_data)
{
    /* USER CODE BEGIN 5 */
    float HUMIDITY_Value = 0;
    float TEMPERATURE_Value = 0;
    float PRESSURE_Value = 0;
    SensorAxes_t ACCELERO_Axes;
    SensorAxes_t GYRO_Axes;
    SensorAxes_t MAGNETO_Axes;
    uint32_t LIGHT_Value = 0;
    uint32_t IR_Value = 0;
    uint32_t RGB_Value = 0;

#if defined(SENSOR_ENABLED) || defined (LRWAN_NS1)
    BSP_HUMIDITY_Get_Hum(HUMIDITY_handle, &HUMIDITY_Value);
    BSP_TEMPERATURE_Get_Temp(TEMPERATURE_handle, &TEMPERATURE_Value);
    BSP_PRESSURE_Get_Press(PRESSURE_handle, &PRESSURE_Value);

    BSP_ACCELERO_Get_Axes(ACCELERO_handle, &ACCELERO_Axes);
    BSP_GYRO_Get_Axes(GYRO_handle, &GYRO_Axes);
    BSP_MAGNETO_Get_Axes(MAGNETO_handle, &MAGNETO_Axes);

    BSP_LIGHT_Get_Light(LIGHT_handle, &LIGHT_Value, &IR_Value, &RGB_Value);

#endif
    sensor_data->humidity = HUMIDITY_Value;
    sensor_data->temperature = TEMPERATURE_Value;
    sensor_data->pressure = PRESSURE_Value;

    sensor_data->latitude = (int32_t) ((STSOP_LATTITUDE * MAX_GPS_POS) / 90);
    sensor_data->longitude = (int32_t) ((STSOP_LONGITUDE * MAX_GPS_POS) / 180);

    sensor_data->accelAxes = ACCELERO_Axes;
    sensor_data->gyroAxes = GYRO_Axes;
    sensor_data->magnetoAxes = MAGNETO_Axes;

    sensor_data->lights = LIGHT_Value;
    sensor_data->IR = IR_Value;
    sensor_data->RGB = RGB_Value;

    /* USER CODE END 5 */
}

void BSP_sensor_Init(void)
{
    /* USER CODE BEGIN 6 */

#if defined(SENSOR_ENABLED) || defined (LRWAN_NS1)
    /* Initialize sensors */
    BSP_HUMIDITY_Init(HTS221_H_0, &HUMIDITY_handle);
    BSP_TEMPERATURE_Init(HTS221_T_0, &TEMPERATURE_handle);
    BSP_PRESSURE_Init(LPS22HB_P_0, &PRESSURE_handle);
    BSP_ACCELERO_Init(LSM6DSL_X_0, &ACCELERO_handle);
    BSP_GYRO_Init(LSM6DSL_G_0, &GYRO_handle);
    BSP_MAGNETO_Init(LIS3MDL_0, &MAGNETO_handle);
    BSP_LIGHT_Init(APDS9250_L_0, &LIGHT_handle);

    /* Enable sensors */
    BSP_HUMIDITY_Sensor_Enable(HUMIDITY_handle);
    BSP_TEMPERATURE_Sensor_Enable(TEMPERATURE_handle);
    BSP_PRESSURE_Sensor_Enable(PRESSURE_handle);
    BSP_ACCELERO_Sensor_Enable(ACCELERO_handle);
    BSP_GYRO_Sensor_Enable(GYRO_handle);
    BSP_MAGNETO_Sensor_Enable(MAGNETO_handle);
#endif
    /* USER CODE END 6 */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
