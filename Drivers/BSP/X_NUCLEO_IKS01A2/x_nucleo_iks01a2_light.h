/**
 ******************************************************************************
 * @file    x_nucleo_iks01a2_light.h
 * @author  MEMS Application Team
 * @version V3.0.0
 * @date    12-August-2019
 * @brief   This file contains definitions for x_nucleo_iks01a2_light.c
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __X_NUCLEO_IKS01A2_LIGHT_H
#define __X_NUCLEO_IKS01A2_LIGHT_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "APDS9250_Driver_HL.h"
#include "x_nucleo_iks01a2.h"


/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A2 X_NUCLEO_IKS01A2
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A2_LIGHT Light
 * @{
 */

/** @addtogroup X_NUCLEO_IKS01A2_LIGHT_Public_Types Public types
  * @{
  */

typedef enum
{
  LIGHT_SENSORS_AUTO = -1, /* Always first element and equal to -1 */
  APDS9250_L_0,                    /* APDS9250 Default on board. */
} LIGHT_ID_t;

/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A2_LIGHT_Public_Defines Public defines
  * @{
  */

#define LIGHT_SENSORS_MAX_NUM 1

/**
 * @}
 */

/** @addtogroup X_NUCLEO_IKS01A2_TEMPERATURE_Public_Function_Prototypes Public function prototypes
 * @{
 */

/* Sensor Configuration Functions */
DrvStatusTypeDef BSP_LIGHT_Init( LIGHT_ID_t id, void **handle );
DrvStatusTypeDef BSP_LIGHT_DeInit( void **handle );
DrvStatusTypeDef BSP_LIGHT_Sensor_Enable( void *handle );
DrvStatusTypeDef BSP_LIGHT_Sensor_Disable( void *handle );
DrvStatusTypeDef BSP_LIGHT_IsInitialized( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_LIGHT_IsEnabled( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_LIGHT_IsCombo( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_LIGHT_Get_Instance( void *handle, uint8_t *instance );
DrvStatusTypeDef BSP_LIGHT_Get_WhoAmI( void *handle, uint8_t *who_am_i );
DrvStatusTypeDef BSP_LIGHT_Check_WhoAmI( void *handle );
DrvStatusTypeDef BSP_LIGHT_Get_Light( void *handle, uint32_t* ambientLight, uint32_t* irLight, uint32_t* rgbLight );
DrvStatusTypeDef BSP_LIGHT_Get_DRDY_Status( void *handle, uint8_t *status );

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

#ifdef __cplusplus
}
#endif

#endif /* __X_NUCLEO_IKS01A2_TEMPERATURE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
