/**
 ******************************************************************************
 * @file    SensiLora.c
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
#include "SensiLora.h"
#include "b_l072z_lrwan1_errno.h"
#include "FlashManager.h"
#include "hts221.h"
#include "UsbService.h"
#include "lora_app.h"
#include "usb.h"

static int32_t Hts221Add(uint32_t Functions);
static int32_t Lps22hhAdd(uint32_t Functions);
static int32_t Lsm6dsoxAdd(uint32_t Functions);
static int32_t Lis2mdlAdd(uint32_t Functions);
static int32_t Apds9250Add(uint32_t Functions);

static HTS221_Object_t sHts221Obj;
static LPS22HH_Object_t sLps22hhObj;
static LIS2MDL_Object_t sLismdlObj;
static LSM6DSOX_Object_t sLsm6dsoxObj;
static APDS9250_Object_t sApds9250Obj;

/**
 * @brief  Initializes the sensor
 * @param  Instance - Type sensor. Use enum Sensors_t.
 * @param  Functions - Type measuring. Use enum Measuring_t.
 * @retval BSP status
 */
int32_t SensiLoraSensorInit(uint32_t Instance, uint32_t Functions)
{
    int32_t ret = BSP_ERROR_NONE;

    if (Instance == SENSI_HTS221 && Functions == SENSI_TEMPERATURE) {
		if (Hts221Add(Functions) != BSP_ERROR_NONE) {
			return BSP_ERROR_NO_INIT;
		}
	} else if (Instance == SENSI_HTS221 && Functions == SENSI_HUMIDITY) {
		if (Hts221Add(Functions) != BSP_ERROR_NONE) {
			return BSP_ERROR_NO_INIT;
		}
	} else if (Instance == SENSI_LPS22HH) {
    	if (Lps22hhAdd(Functions) != BSP_ERROR_NONE) {
            return BSP_ERROR_NO_INIT;
        }
	} else if (Instance == SENSI_LSM6DSOX && Functions == SENSI_ACCEL) {
    	if (Lsm6dsoxAdd(Functions) != BSP_ERROR_NONE) {
            return BSP_ERROR_NO_INIT;
        }
	} else if (Instance == SENSI_LSM6DSOX && Functions == SENSI_GYR) {
    	if (Lsm6dsoxAdd(Functions) != BSP_ERROR_NONE) {
            return BSP_ERROR_NO_INIT;
        }
	}else if (Instance == SENSI_LIS2MDL) {
    	if (Lis2mdlAdd(Functions) != BSP_ERROR_NONE) {
            return BSP_ERROR_NO_INIT;
        }
	} else if (Instance == SENSI_APDS9250) {
    	if (Apds9250Add(Functions) != BSP_ERROR_NONE) {
            return BSP_ERROR_NO_INIT;
        }
	}
    return ret;
}

/**
 * @brief  DeInitializes the sensor
 * @param  Instance - Type sensor. Use enum Sensors_t.
 * @retval BSP status
 */
int32_t SensiLoraSensorDeInit(uint32_t Instance)
{
    int32_t ret = BSP_ERROR_NONE;

    if (Instance == SENSI_HTS221) {
        return HTS221_DeInit(&sHts221Obj);
    } else if (Instance == SENSI_LPS22HH) {
        return LPS22HH_DeInit(&sLps22hhObj);
    } else if (Instance == SENSI_LSM6DSOX) {
        return LSM6DSOX_DeInit(&sLsm6dsoxObj);
    }else if (Instance == SENSI_LIS2MDL) {
        return LIS2MDL_DeInit(&sLismdlObj);
    } else if (Instance == SENSI_APDS9250) {
        return APDS9250_DeInit(&sApds9250Obj);
    }
    return ret;
}

/**
 * @brief  Test work sensor
 * @param  Instance - Type sensor. Use enum Sensors_t.
 * @retval true - sensor work. false - sensor not working
 */
bool SensiLoraSensorTest(uint32_t Instance)
{
    bool ret = false;
    uint8_t id = 0;

    switch (Instance) {
	    case SENSI_HTS221:
		    if (HTS221_ReadID(&sHts221Obj, &id) == HTS221_OK && id == HTS221_ID) {
			    ret = true;
		    }
		    break;
	    case SENSI_LPS22HH:
		    if (LPS22HH_ReadID(&sLps22hhObj, &id) == LPS22HH_OK && id == LPS22HH_ID) {
			    ret = true;
		    }
		    break;
	    case SENSI_APDS9250:
		    if (APDS9250_Get_WhoAmI(&sApds9250Obj, &id) == COMPONENT_OK && id == APDS9250_ID) {
			    ret = true;
		    }
		    break;
	    case SENSI_LSM6DSOX:
		    if (LSM6DSOX_ReadID(&sLsm6dsoxObj, &id) == LSM6DSOX_OK && id == LSM6DSOX_ID) {
			    ret = true;
		    }
		    break;
	    case SENSI_LIS2MDL:
		    if (LIS2MDL_ReadID(&sLismdlObj, &id) == LIS2MDL_OK && id == LIS2MDL_ID) {
			    ret = true;
		    }
		    break;
	    default:
		   break;
    }
    return ret;
}

/**
 * @brief  Enable sensor
 * @param  Instance - Type sensor. Use enum Sensors_t.
 * @param  Functions - Type measuring. Use enum Measuring_t.
 * @retval BSP status
 */
int32_t SensiLoraSensorEnable(uint32_t Instance, uint32_t Function)
{
	int32_t ret = 0;

	if (Instance == SENSI_HTS221 && Function == SENSI_TEMPERATURE) {
		ret = HTS221_TEMP_Enable(&sHts221Obj);
		SensiLoraSensorUpdate(SENSI_TEMPERATURE);
	} else if (Instance == SENSI_HTS221 && Function == SENSI_HUMIDITY) {
		ret = HTS221_HUM_Enable(&sHts221Obj);
		SensiLoraSensorUpdate(SENSI_HUMIDITY);
	} else if (Instance == SENSI_LPS22HH) {
		ret = LPS22HH_PRESS_Enable(&sLps22hhObj);
		SensiLoraSensorUpdate(SENSI_LPS22HH);
	} else if (Instance == SENSI_LSM6DSOX && Function == SENSI_ACCEL) {
		ret = LSM6DSOX_ACC_Enable(&sLsm6dsoxObj);
		SensiLoraSensorUpdate(SENSI_ACCEL);
	} else if (Instance == SENSI_LSM6DSOX && Function == SENSI_GYR) {
		ret = LSM6DSOX_GYRO_Enable(&sLsm6dsoxObj);
		SensiLoraSensorUpdate(SENSI_GYR);
	}else if (Instance == SENSI_LIS2MDL) {
		ret = LIS2MDL_MAG_Enable(&sLismdlObj);
		SensiLoraSensorUpdate(SENSI_LIS2MDL);
	} else if (Instance == SENSI_APDS9250) {
		ret = APDS9250_Sensor_Enable(&sApds9250Obj);
		SensiLoraSensorUpdate(SENSI_APDS9250);
	}

	return ret;
}

/**
 * @brief  Disable sensor
 * @param  Instance - Type sensor. Use enum Sensors_t.
 * @param  Functions - Type measuring. Use enum Measuring_t.
 * @retval BSP status
 */
int32_t SensiLoraSensorDisable(uint32_t Instance, uint32_t Function)
{
	int32_t ret = 0;

	if (Instance == SENSI_HTS221 && Function == SENSI_TEMPERATURE) {
		ret = HTS221_TEMP_Disable(&sHts221Obj);
	} else if (Instance == SENSI_HTS221 && Function == SENSI_HUMIDITY) {
		ret = HTS221_HUM_Disable(&sHts221Obj);
	} else if (Instance == SENSI_LPS22HH) {
		ret = LPS22HH_PRESS_Disable(&sLps22hhObj);
	} else if (Instance == SENSI_LSM6DSOX && Function == SENSI_ACCEL) {
		ret = LSM6DSOX_ACC_Disable(&sLsm6dsoxObj);
	} else if (Instance == SENSI_LSM6DSOX && Function == SENSI_GYR) {
		ret = LSM6DSOX_GYRO_Disable(&sLsm6dsoxObj);
	}else if (Instance == SENSI_LIS2MDL) {
		ret = LIS2MDL_MAG_Disable(&sLismdlObj);
	} else if (Instance == SENSI_APDS9250) {
		ret = APDS9250_Sensor_Disable(&sApds9250Obj);
	}
	return ret;
}

/**
 * @brief  Get sensor value
 * @param  Instance - Type sensor. Use enum Sensors_t.
 * @param  Functions - Type measuring. Use enum Measuring_t.
 * @param  Value -  pointer to sensor value
 * @retval BSP status
 */
int32_t SensiLoraSensorGetValue(uint32_t Instance, uint32_t Function, float *Value)
{
  int32_t ret = 0;

  if (Instance == SENSI_HTS221 && Function == SENSI_TEMPERATURE) {
	  ret = HTS221_TEMP_GetTemperature(&sHts221Obj, Value);
	  if (UsbComPortStatus()) {
	      //Compensate for the heating of the board, subtract 0.2 degrees from the measured temperature
	      *Value -= 3.2;
	  } else {
	      //Compensate for the heating of the board, subtract 3.2 degrees from the measured temperature when device connect to SensiConfigurator program
	      *Value -= 0.2;
	  }
  } else if (Instance == SENSI_HTS221 && Function == SENSI_HUMIDITY) {
	  ret = HTS221_HUM_GetHumidity(&sHts221Obj, Value);
	  if (!UsbComPortStatus()) {
          //Compensate for the heating of the board, subtract 4.0 degrees from the measured humidity
	      *Value -= 4.0;
	  }
  } else if (Instance == SENSI_LPS22HH) {
	  ret = LPS22HH_PRESS_GetPressure(&sLps22hhObj, Value);
	  *Value += 2.3;
  } else if (Instance == SENSI_LSM6DSOX && Function == SENSI_ACCEL) {
	  ret = LSM6DSOX_ACC_GetAxes(&sLsm6dsoxObj, (LSM6DSOX_Axes_t*)Value);
  } else if (Instance == SENSI_LSM6DSOX && Function == SENSI_GYR) {
	  ret = LSM6DSOX_GYRO_GetAxes(&sLsm6dsoxObj, (LSM6DSOX_Axes_t*)Value);
  }else if (Instance == SENSI_LIS2MDL) {
	  ret = LIS2MDL_MAG_GetAxes(&sLismdlObj, (LIS2MDL_Axes_t*)Value);
  } else if (Instance == SENSI_APDS9250) {
	  ret = APDS9250_Get_Light(&sApds9250Obj, (uint32_t*)Value);
  }
  return ret;
}

/**
 * @brief  Update sensor status.
 * @param  value - Type sensor. Use enum Measuring_t.
 * @retval None
 */
void SensiLoraSensorUpdate(Measuring_t value)
{
    bool en = 0;
    uint8_t param1 = 0;
    uint8_t param2 = 0;

    switch (value) {
        case SENSI_ACCEL:
            FlashMemoryGetAccSett(&en, &param1, &param2);
            LSM6DSOX_ACC_SetFullScale(&sLsm6dsoxObj, param1);
            LSM6DSOX_ACC_SetOutputDataRate(&sLsm6dsoxObj, param2);
            break;
        case SENSI_GYR:
            FlashMemoryGetGyroSett(&en, &param1, &param2);
            LSM6DSOX_GYRO_SetFullScale(&sLsm6dsoxObj, param1);
            LSM6DSOX_GYRO_SetOutputDataRate(&sLsm6dsoxObj, param2);
            break;
        case SENSI_MAG:
            FlashManagerGetMagSett(&en, &param1);
            LIS2MDL_MAG_SetOutputDataRate(&sLismdlObj, param1);
            break;
        case SENSI_PRESSURE:
            FlashManagerGetPressureSett(&en, &param1);
            LPS22HH_PRESS_SetOdr(&sLps22hhObj, param1);
            break;
        case SENSI_HUMIDITY:
            FlashManagerGetHumSett(&en, &param1, &param2);
            HTS221_HUM_SetAvg(&sHts221Obj, param1);
            HTS221_HUM_SetOdr(&sHts221Obj, param2);
            break;
        case SENSI_TEMPERATURE:
            FlashManagerGetTempSett(&en, &param1);
            HTS221_TEMP_SetAvg(&sHts221Obj, param1);
          break;
        case SENSI_LIGHT:
            FlashManagerGetLightSett(&en, &param1, &param2);
            APDS9250_S_LS_RESOLUTION_(&sApds9250Obj, param1);
            APDS9250_S_LS_RATE_(&sApds9250Obj, param2);
          break;
        default:
          break;
    }
}

/**
 * @brief  Register Bus IOs for instance 0 if component ID is OK
 * @param  Functions Environmental sensor functions. Could be :
 *         - SENSI_TEMPERATURE and/or SENSI_HUMIDITY
 * @retval BSP status
 */
int32_t Hts221Add(uint32_t Functions)
{
    HTS221_IO_t            io_ctx;
    uint8_t                id;
    int32_t                ret = BSP_ERROR_NONE;

    /* Configure the environmental sensor driver */
    io_ctx.BusType     = HTS221_I2C_BUS; /* I2C */
    io_ctx.Address     = HTS221_I2C_ADDRESS;
    io_ctx.Init        = SENSI_LORA_I2C_Init;
    io_ctx.DeInit      = SENSI_LORA_I2C_DeInit;
    io_ctx.ReadReg     = SENSI_LORA_I2C_ReadReg;
    io_ctx.WriteReg    = SENSI_LORA_I2C_WriteReg;
    io_ctx.GetTick     = SENSI_LORA_GetTick;

    if (sHts221Obj.is_initialized == 1) {
        return ret;
    }

    if (HTS221_RegisterBusIO(&sHts221Obj, &io_ctx) != HTS221_OK) {
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
    }
    else if (HTS221_ReadID(&sHts221Obj, &id) != HTS221_OK) {
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
    }
    else if(id != HTS221_ID){
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
    } else if (HTS221_Init(&sHts221Obj) != HTS221_OK) {
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
    } else {
    }

    return ret;
}

/**
 * @brief  Register Bus IOs for instance 1 if component ID is OK
 * @param  Functions Environmental sensor functions. Could be :
 *         - SENSI_PRESSURE
 * @retval BSP status
 */
int32_t Lps22hhAdd(uint32_t Functions)
{
    LPS22HH_IO_t            io_ctx;
    uint8_t                 id;
    int32_t                 ret = BSP_ERROR_NONE;

    /* Configure the pressure driver */
    io_ctx.BusType     = LPS22HH_I2C_BUS; /* I2C */
    io_ctx.Address     = LPS22HH_I2C_ADD_H;
    io_ctx.Init        = SENSI_LORA_I2C_Init;
    io_ctx.DeInit      = SENSI_LORA_I2C_DeInit;
    io_ctx.ReadReg     = SENSI_LORA_I2C_ReadReg;
    io_ctx.WriteReg    = SENSI_LORA_I2C_WriteReg;
    io_ctx.GetTick     = SENSI_LORA_GetTick;

    if (sLps22hhObj.is_initialized == 1) {
        return ret;
    }
    if (LPS22HH_RegisterBusIO(&sLps22hhObj, &io_ctx) != LPS22HH_OK)
    {
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
    }
    else if (LPS22HH_ReadID(&sLps22hhObj, &id) != LPS22HH_OK)
    {
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
    }
    else if (id != LPS22HH_ID)
    {
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
    } else if (LPS22HH_Init(&sLps22hhObj) != LPS22HH_OK)
    {
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
    }
    else {
    }
    return ret;
}

/**
 * @brief  Register Bus IOs for instance 1 if component ID is OK
 * @param  Functions Environmental sensor functions. Could be :
 *         - SENSI_ACCEL or SENSI_GYR
 * @retval BSP status
 */
int32_t Lsm6dsoxAdd(uint32_t Functions)
{
    LSM6DSOX_IO_t           io_ctx;
    uint8_t                 id;
    int32_t                 ret = BSP_ERROR_NONE;

    /* Configure the pressure driver */
    io_ctx.BusType     = LSM6DSOX_I2C_BUS; /* I2C */
    io_ctx.Address     = LSM6DSOX_I2C_ADD_L;
    io_ctx.Init        = SENSI_LORA_I2C_Init;
    io_ctx.DeInit      = SENSI_LORA_I2C_DeInit;
    io_ctx.ReadReg     = SENSI_LORA_I2C_ReadReg;
    io_ctx.WriteReg    = SENSI_LORA_I2C_WriteReg;
    io_ctx.GetTick     = SENSI_LORA_GetTick;

    if (sLsm6dsoxObj.is_initialized == 1) {
        return ret;
    }

    if (LSM6DSOX_RegisterBusIO(&sLsm6dsoxObj, &io_ctx) != LSM6DSOX_OK)
    {
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
    }
    else if (LSM6DSOX_ReadID(&sLsm6dsoxObj, &id) != LSM6DSOX_OK)
    {
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
    }
    else if (id != LSM6DSOX_ID)
    {
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
    }
    else if (LSM6DSOX_Init(&sLsm6dsoxObj) != LSM6DSOX_OK){
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
    } else {
    }
    return ret;
}

/**
 * @brief  Register Bus IOs for instance 1 if component ID is OK
 * @param  Functions Environmental sensor functions. Could be :
 *         - SENSI_MAG
 * @retval BSP status
 */
static int32_t Lis2mdlAdd(uint32_t Functions)
{
    LIS2MDL_IO_t            io_ctx;
    uint8_t                 id;
    int32_t                 ret = BSP_ERROR_NONE;

    /* Configure the pressure driver */
    io_ctx.BusType     = LIS2MDL_I2C_BUS; /* I2C */
    io_ctx.Address     = LIS2MDL_I2C_ADD;
    io_ctx.Init        = SENSI_LORA_I2C_Init;
    io_ctx.DeInit      = SENSI_LORA_I2C_DeInit;
    io_ctx.ReadReg     = SENSI_LORA_I2C_ReadReg;
    io_ctx.WriteReg    = SENSI_LORA_I2C_WriteReg;
    io_ctx.GetTick     = SENSI_LORA_GetTick;

    if (sLismdlObj.is_initialized == 1) {
        return ret;
    }

    if (LIS2MDL_RegisterBusIO(&sLismdlObj, &io_ctx) != LIS2MDL_OK)
    {
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
    }
    else if (LIS2MDL_ReadID(&sLismdlObj, &id) != LIS2MDL_OK)
    {
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
    }
    else if (id != LIS2MDL_ID)
    {
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
    } else if (LIS2MDL_Init(&sLismdlObj) != LIS2MDL_OK){
    } else {
    }
    return ret;
}

/**
 * @brief  Register Bus IOs for instance 1 if component ID is OK
 * @param  Functions Environmental sensor functions. Could be :
 *         - SENSI_LIGHT
 * @retval BSP status
 */
static int32_t Apds9250Add(uint32_t Functions)
{
    APDS9250_IO_t           io_ctx;
    uint8_t                 id;
    int32_t                 ret = BSP_ERROR_NONE;

    /* Configure the pressure driver */
    io_ctx.BusType     = APDS9250_I2C_BUS; /* I2C */
    io_ctx.Address     = APDS9250_ADDRESS_DEFAULT;
    io_ctx.Init        = SENSI_LORA_I2C_Init;
    io_ctx.DeInit      = SENSI_LORA_I2C_DeInit;
    io_ctx.ReadReg     = SENSI_LORA_I2C_ReadReg;
    io_ctx.WriteReg    = SENSI_LORA_I2C_WriteReg;

    if (sApds9250Obj.isInitialized == 1) {
        return ret;
    }

    if (APDS9250_RegisterBusIO(&sApds9250Obj, &io_ctx) != LIS2MDL_OK)
    {
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
    }
    else if (APDS9250_Get_WhoAmI(&sApds9250Obj, &id) != LIS2MDL_OK)
    {
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
    }
    else if (id != APDS9250_ID)
    {
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
    }
    else if (APDS9250_Init(&sApds9250Obj) != COMPONENT_OK) {
        ret = BSP_ERROR_UNKNOWN_COMPONENT;
    } else {
    }

    return ret;
}

