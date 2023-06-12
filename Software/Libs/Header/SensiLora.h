/**
 ******************************************************************************
 * @file    SensiLora.h
 * @date    30-June-2022
 * @brief   The SensiLora class description.
 *          This class is used to work with sensors on board.
 ******************************************************************************
 *
 * COPYRIGHT(c) 2022 SensiEDGE
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of SensiEDGE LTD nor the names of its contributors may
 *      be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _SENSI_LORA_H
#define _SENSI_LORA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "SensiLoraConf.h"
#include "env_sensor.h"

#include "lps22hh.h"
#include "lsm6dsox.h"
#include "APDS9250_Driver_HL.h"
#include "hts221.h"
#include "lis2mdl.h"
#include "AT25XE041B.h"
#include "stdbool.h"

typedef enum {
	SENSI_TEMPERATURE = 0,
	SENSI_PRESSURE,
	SENSI_HUMIDITY,
	SENSI_ACCEL,
	SENSI_GYR,
	SENSI_MAG,
	SENSI_LIGHT,
	SENSI_FLASH,
} Measuring_t;

typedef enum {
	SENSI_HTS221 = 0,
	SENSI_LPS22HH,
	SENSI_APDS9250,
	SENSI_LSM6DSOX,
	SENSI_LIS2MDL,
	SENSI_MP34DT05,
	SENSI_COUNT
} Sensors_t;

/* Environmental Sensor instance Info */
typedef struct
{
  uint8_t Temperature;
  uint8_t Pressure;
  uint8_t Humidity;
  uint8_t LowPower;
  float   HumMaxOdr;
  float   TempMaxOdr;
  float   PressMaxOdr;
} sensorsValue_t;

/**
 * @brief  Initializes the sensor
 * @param  Instance - Type sensor. Use enum sensors_t.
 * @param  Functions - Type measuring. Use enum measuring_t.
 * @retval BSP status
 */
int32_t SensiLoraSensorInit(uint32_t Instance, uint32_t Functions);

/**
 * @brief  DeInitializes the sensor
 * @param  Instance - Type sensor. Use enum sensors_t.
 * @retval BSP status
 */
int32_t SensiLoraSensorDeInit(uint32_t Instance);

/**
 * @brief  Test work sensor
 * @param  Instance - Type sensor. Use enum sensors_t.
 * @retval true - sensor work. false - sensor not working
 */
int32_t SensiLoraSensorEnable(uint32_t Instance, uint32_t Function);

/**
 * @brief  Enable sensor
 * @param  Instance - Type sensor. Use enum sensors_t.
 * @param  Functions - Type measuring. Use enum measuring_t.
 * @retval BSP status
 */
int32_t SensiLoraSensorDisable(uint32_t Instance, uint32_t Function);

/**
 * @brief  Disable sensor
 * @param  Instance - Type sensor. Use enum sensors_t.
 * @param  Functions - Type measuring. Use enum measuring_t.
 * @retval BSP status
 */
int32_t SensiLoraSensorGetValue(uint32_t Instance, uint32_t Function, float *Value);

/**
 * @brief  Get sensor value
 * @param  Instance - Type sensor. Use enum sensors_t.
 * @param  Functions - Type measuring. Use enum measuring_t.
 * @param  Value -  pointer to sensor value
 * @retval BSP status
 */
bool SensiLoraSensorTest(uint32_t Instance);

/**
 * @brief  Update sensor status.
 * @param  value - Type sensor. Use enum Measuring_t.
 * @retval None
 */
void SensiLoraSensorUpdate(Measuring_t value);

#ifdef __cplusplus
}
#endif

#endif /* _SENSI_LORA_H */
