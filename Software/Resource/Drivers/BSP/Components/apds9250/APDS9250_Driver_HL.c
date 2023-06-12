/**
 ******************************************************************************
 * @file    APDS9250_Driver_HL.c
 * @author  MEMS Application Team
 * @version V3.0.0
 * @date    12-August-2019
 * @brief   This file provides a set of high-level functions needed to manage
 the APDS9250 sensor
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
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

/* Includes ------------------------------------------------------------------*/
#include "APDS9250_Driver_HL.h"
#include <math.h>

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup APDS9250 APDS9250
 * @{
 */

/** @addtogroup APDS9250_Callable_Private_FunctionPrototypes Callable private function prototypes
 * @{
 */


/**
 * @}
 */

/** @addtogroup APDS9250_Private_Variables Private variables
 * @{
 */

/**
 * @brief APDS9250 humidity driver structure
 */

/**
 * @brief APDS9250 temperature driver structure
 */

/** @addtogroup APDS9250_Callable_Private_Functions Callable private functions
 * @{
 */

/**
 * @brief  Register Component Bus IO operations
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
DrvStatusTypeDef APDS9250_RegisterIO(APDS9250_Object_t *pObj, APDS9250_IO_t *pIO)
{
  int32_t ret = COMPONENT_OK;

  if (pObj == NULL)
  {
    ret = COMPONENT_ERROR;
  }
  else
  {

    if (APDS9250_RegisterBusIO(pObj,pIO) != APDS9250_ERROR)
    {
      ret = COMPONENT_ERROR;
    }
  }

  return ret;
}

/**
 * @brief Initialize the APDS9250 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef APDS9250_Init(APDS9250_Object_t *pObj)
{

    /* Check device ID */
    if (APDS9250_Check_WhoAmI(pObj) == COMPONENT_ERROR)
    {
        return COMPONENT_ERROR;
    }

    /* Power down the device */
    if (APDS9250_S_LS_MODE_((void*) pObj, APDS9250_LS_ACTIVE) == APDS9250_ERROR)
    {
        return COMPONENT_ERROR;
    }

    /* Enable Block Data Update */
    if (APDS9250_S_LS_RESOLUTION_((void*) pObj, APDS9250_20bit) == APDS9250_ERROR)
    {
        return COMPONENT_ERROR;
    }

    /* Set Output Data Rate */
    if (APDS9250_S_LS_RATE_((void*) pObj, APDS9250_100ms) == APDS9250_ERROR)
    {
        return COMPONENT_ERROR;
    }

    pObj->isInitialized = 1;

    return COMPONENT_OK;
}

/**
 * @brief Deinitialize the APDS9250 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef APDS9250_DeInit(APDS9250_Object_t *pObj)
{

    if (APDS9250_Check_WhoAmI(pObj) == COMPONENT_ERROR)
    {
        return COMPONENT_ERROR;
    }

    /* Disable the component */
    if (APDS9250_Sensor_Disable(pObj) == COMPONENT_ERROR)
    {
        return COMPONENT_ERROR;
    }

    /* Power down the device */
    if (APDS9250_S_LS_MODE_((void*) pObj, APDS9250_LS_STANDBY) == APDS9250_ERROR)
    {
        return COMPONENT_ERROR;
    }

    pObj->isInitialized = 0;

    return COMPONENT_OK;
}

/**
 * @brief Enable the APDS9250 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
 DrvStatusTypeDef APDS9250_Sensor_Enable(APDS9250_Object_t *pObj)
{
    /* Check if the component is already enabled */
    if (pObj->isEnabled == 1)
    {
        return COMPONENT_OK;
    }

    /* Power down the device */
    if (APDS9250_S_LS_MODE_((void*) pObj, APDS9250_LS_ACTIVE) == APDS9250_ERROR)
    {
        return COMPONENT_ERROR;
    }

    pObj->isEnabled = 1;

    return COMPONENT_OK;
}

/**
 * @brief Disable the APDS9250 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef APDS9250_Sensor_Disable(APDS9250_Object_t *pObj)
{

    /* Check if the component is already disabled */
    if (pObj->isEnabled == 0)
    {
        return COMPONENT_OK;
    }

    /* Power down the device */
    if (APDS9250_S_LS_MODE_((void*) pObj, APDS9250_LS_STANDBY) == APDS9250_ERROR)
    {
        return COMPONENT_ERROR;
    }

    pObj->isEnabled = 0;

    return COMPONENT_OK;
}

/**
 * @brief Get the WHO_AM_I ID of the APDS9250 sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef APDS9250_Get_WhoAmI(APDS9250_Object_t *pObj,
                                            uint8_t* who_am_i)
{

    /* Read WHO AM I register */
    if (APDS9250_R_WHO_AM_I_((void*) pObj, (uint8_t*) who_am_i) == APDS9250_ERROR)
    {
        return COMPONENT_ERROR;
    }

    return COMPONENT_OK;
}

/**
 * @brief Check the WHO_AM_I ID of the APDS9250 sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef APDS9250_Check_WhoAmI(APDS9250_Object_t *pObj)
{

    uint8_t who_am_i = 0x00;

    if (APDS9250_Get_WhoAmI(pObj, &who_am_i) == COMPONENT_ERROR)
    {
        return COMPONENT_ERROR;
    }
    if (who_am_i != pObj->who_am_i)
    {
        return COMPONENT_ERROR;
    }

    return COMPONENT_OK;
}

/**
 * @brief Get the APDS9250 sensor axes
 * @param handle the device handle
 * @param magnetic_field pointer where the values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef APDS9250_Get_Data(APDS9250_Object_t *pObj,
                                           uint32_t* ambientLight, uint32_t* irLight,
                                           uint32_t* rgbLightd)
{
    if (APDS9250__G_LS_LIGHT_DATA_t(pObj, (uint32_t*) ambientLight) == APDS9250_ERROR)
    {
        return COMPONENT_ERROR;
    }

    if (APDS9250_G_LS_IR_DATA_(pObj, (uint32_t*) irLight) == APDS9250_ERROR)
    {
        return COMPONENT_ERROR;
    }

    return COMPONENT_OK;
}


/**
 * @brief Get the APDS9250 sensor axes
 * @param handle the device handle
 * @param magnetic_field pointer where the values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef APDS9250_Get_Light(APDS9250_Object_t *pObj,
                                           uint32_t* ambientLight)
{
    if (APDS9250__G_LS_LIGHT_DATA_t(pObj, (uint32_t*) ambientLight) == APDS9250_ERROR)
    {
        return COMPONENT_ERROR;
    }

    return COMPONENT_OK;
}

/**
 * @brief Read the data from register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef APDS9250_Read_Reg(APDS9250_Object_t *pObj,
                                          uint8_t reg, uint8_t* data)
{

    if (APDS9250_ReadReg((void*) pObj, reg, 1, data) == APDS9250_ERROR)
    {
        return COMPONENT_ERROR;
    }

    return COMPONENT_OK;
}

/**
 * @brief Write the data to register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef APDS9250_Write_Reg(APDS9250_Object_t *pObj,
                                           uint8_t reg, uint8_t data)
{

    if (APDS9250_WriteReg((void*) pObj, reg, 1, &data) == APDS9250_ERROR)
    {
        return COMPONENT_ERROR;
    }

    return COMPONENT_OK;
}
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/