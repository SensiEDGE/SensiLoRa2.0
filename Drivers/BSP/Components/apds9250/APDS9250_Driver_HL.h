/**
 ******************************************************************************
 * @file    APDS9250_Driver_HL.h
 * @author  MEMS Application Team
 * @version V3.0.0
 * @date    12-August-2019
 * @brief   This file contains definitions for the APDS9250_Driver_HL.c firmware driver
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
#ifndef __APDS9250_DRIVER_HL_H
#define __APDS9250_DRIVER_HL_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "light.h"

/* Include sensor component drivers. */
#include "APDS9250_Driver.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup COMPONENTS COMPONENTS
 * @{
 */

/** @addtogroup APDS9250 APDS9250
 * @{
 */

/** @addtogroup APDS9250_Public_Constants Public constants
 * @{
 */

#define APDS9250_SENSORS_MAX_NUM  1     /**< APDS9250 max number of instances */

/** @addtogroup APDS9250_I2C_Addresses APDS9250 I2C Addresses
 * @{
 */

#define APDS9250_ADDRESS_DEFAULT  0x52  /**< APDS9250 I2C Address */

/**
 * @}
 */

/**
 * @}
 */

/** @addtogroup APDS9250_Public_Types APDS9250 Public Types
 * @{
 */

/**
 * @brief APDS9250 light specific data internal structure definition
 */
/* _NOTE_: Not used - type reserved for future purposes. */
typedef struct
{
  uint8_t dummy;
} APDS9250_Data_t;

/**
 * @}
 */

/** @addtogroup APDS9250_Public_Variables Public variables
 * @{
 */

extern LIGHT_Drv_t APDS9250_Drv;

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

#endif /* __APDS9250_DRIVER_HL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
