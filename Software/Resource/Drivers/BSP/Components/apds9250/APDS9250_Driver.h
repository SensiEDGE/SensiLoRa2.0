/**
 ******************************************************************************
 * @file    APDS9250_driver.h
 * @author  MEMS Application Team
 * @version V1.1
 * @date    25-February-2019
 * @brief   LSM303AGR Magnetometer header driver file
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
#ifndef __APDS9250_DRIVER__H
#define __APDS9250_DRIVER__H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

typedef enum
{
    APDS9250_SUCCESS = 0x01, APDS9250_ERROR = 0x00
} APDS9250status_t;

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

/** @addtogroup  Interfaces_Functions
 * @brief       This section provide a set of functions used to read and
 *              write a generic register of the device.
 *              MANDATORY: return 0 -> no Error.
 * @{
 *
 */

typedef int32_t (*stmdev_write_ptr)(void*, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*stmdev_read_ptr)(void*, uint8_t, uint8_t*, uint16_t);

/** Device Identification (Who am I) **/
#define APDS9250_ID                  0xB2U
#define APDS9250_CTRL_REG            0x00U

typedef struct
{
    uint8_t not_used_01 :1;
    uint8_t ls_en :1;
    uint8_t cs_mode :1;
    uint8_t not_used_02 :1;
    uint8_t sw_reset :1;
    uint8_t not_used_03 :3;
} apds9250_control_reg_t;

#define APDS9250_LS_MEAS_RATE        0x04U
typedef struct
{
    uint8_t ls_meas :3;
    uint8_t not_used_01 :1;
    uint8_t ls_resolution :3;
    uint8_t not_used_03 :1;
} apds9250_ls_meas_rate_reg_t;

#define APDS9250_LS_GAIN             0x05U
typedef struct
{
    uint8_t ls_gain :3;
    uint8_t not_used_01 :5;
} apds9250_ls_gain_reg_t;

#define APDS9250_WHO_AM_I            0x06U

#define APDS9250_STATUS_REG          0x07U
typedef struct
{
    uint8_t not_used_01 :3;
    uint8_t ls_data :1;
    uint8_t ls_interrupt :1;
    uint8_t power_on :1;
    uint8_t not_used_02 :2;
} apds9250_status_reg_t;

#define APDS9250_DATA_IR0            0x0AU
#define APDS9250_DATA_IR1            0x0BU
#define APDS9250_DATA_IR2            0x0CU

#define APDS9250_DATA_GREEN0         0x0DU
#define APDS9250_DATA_GREEN1         0x0EU
#define APDS9250_DATA_GREEN2         0x0FU

#define APDS9250_DATA_BLUE0          0x10U
#define APDS9250_DATA_BLUE1          0x11U
#define APDS9250_DATA_BLUE2          0x12U

#define APDS9250_DATA_RED0           0x13U
#define APDS9250_DATA_RED1           0x14U
#define APDS9250_DATA_RED2           0x15U

#define APDS9250_INT_CONF            0x19U
typedef struct
{
    uint8_t not_used_01 :2;
    uint8_t ls_int_en :1;
    uint8_t ls_var_mode :1;
    uint8_t ls_int_sel :2;
    uint8_t not_used_02 :2;
} apds9250_int_conf_t;

#define APDS9250_INT_PERSISTENCE     0x1AU
typedef struct
{
    uint8_t not_used_01 :4;
    uint8_t ls_persist :4;
} apds9250_int_persistence_t;

#define APDS9250_THRES_UP0           0x21U
#define APDS9250_THRES_UP1           0x22U
#define APDS9250_THRES_UP2           0x23U

#define APDS9250_THRES_LOW0          0x24U
#define APDS9250_THRES_LOW1          0x25U
#define APDS9250_THRES_LOW2          0x26U

#define APDS9250_THRES_VAR           0x27U
typedef struct
{
    uint8_t ls_thres_var :3;
    uint8_t not_used_01 :5;
} apds9250_thres_var_t;

/**
 * @defgroup HTS221_Register_Union
 * @brief    This union group all the registers that has a bitfield
 *           description.
 *           This union is useful but not need by the driver.
 *
 *           REMOVING this union you are compliant with:
 *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
 *
 * @{
 *
*/
typedef int32_t (*APDS9250_Init_Func)(void);
typedef int32_t (*APDS9250_DeInit_Func)(void);
typedef int32_t (*APDS9250_WriteReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*APDS9250_ReadReg_Func)(uint16_t, uint16_t, uint8_t *, uint16_t);
typedef int32_t (*APDS9250_write_ptr)(void *, uint8_t*, uint16_t);
typedef int32_t (*APDS9250_read_ptr) (void *, uint8_t*, uint16_t);


typedef struct
{
   uint32_t                   BusType; /*0 means I2C, 1 means SPI 4-Wires, 2 means SPI-3-Wires */
   uint8_t                    Address;
   APDS9250_Init_Func          Init;
   APDS9250_DeInit_Func        DeInit;
   APDS9250_WriteReg_Func      WriteReg;
   APDS9250_ReadReg_Func       ReadReg;
  // AT25XE041B_GetTick_Func       GetTick;
} APDS9250_IO_t;

typedef struct {
  /** Component mandatory fields **/
	APDS9250_IO_t        IO;
	APDS9250_write_ptr      WriteReg;
	APDS9250_read_ptr       ReadReg;
	uint8_t isInitialized;
	uint8_t isEnabled;
	uint8_t who_am_i;
  /** Customizable optional pointer **/
  void *handle;
} APDS9250_Object_t;


APDS9250status_t APDS9250_RegisterBusIO(APDS9250_Object_t *pObj, APDS9250_IO_t *pIO);
APDS9250status_t APDS9250_ReadReg(APDS9250_Object_t *pObj, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data );
APDS9250status_t APDS9250_WriteReg(APDS9250_Object_t *pObj, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data );

APDS9250status_t APDS9250_R_WHO_AM_I_(APDS9250_Object_t *pObj, uint8_t *value);

typedef enum
{
    APDS9250_LS_STANDBY = 0, APDS9250_LS_ACTIVE = 1, APDS9250_LS_ND = 255,
} apds9250_ls_enable_t;

APDS9250status_t APDS9250_S_LS_MODE_(APDS9250_Object_t *pObj, apds9250_ls_enable_t val);
APDS9250status_t APDS9250_G_LS_MODE_(APDS9250_Object_t *pObj, apds9250_ls_enable_t* val);

typedef enum
{
    APDS9250_20bit = 0,
    APDS9250_19bit = 1,
    APDS9250_18bit = 2,
    APDS9250_17bit = 3,
    APDS9250_16bit = 4,
    APDS9250_13bit = 5,
    APDS9250_NDbit = 255,
} apds9250_ls_resolution_t;

APDS9250status_t APDS9250_S_LS_RESOLUTION_(APDS9250_Object_t *pObj, apds9250_ls_resolution_t val);
APDS9250status_t APDS9250_G_LS_RESOLUTION_(APDS9250_Object_t *pObj, apds9250_ls_resolution_t* val);

typedef enum
{
    APDS9250_25ms = 0,
    APDS9250_50ms = 1,
    APDS9250_100ms = 2,
    APDS9250_200ms = 3,
    APDS9250_500ms = 4,
    APDS9250_1000ms = 5,
    APDS9250_2000ms = 6,
    APDS9250_5000ms = 7,
    APDS9250_NDms = 255,
} apds9250_ls_rate_t;

APDS9250status_t APDS9250_S_LS_RATE_(APDS9250_Object_t *pObj, apds9250_ls_rate_t val);
APDS9250status_t APDS9250_G_LS_RATE_(APDS9250_Object_t *pObj, apds9250_ls_rate_t* val);
APDS9250status_t APDS9250_G_LS_IR_DATA_(APDS9250_Object_t *pObj, uint32_t* val);
APDS9250status_t APDS9250_G_LS_GREEN_DATA_(APDS9250_Object_t *pObj, uint32_t* val);
APDS9250status_t APDS9250_G_LS_BLUE_DATA_(APDS9250_Object_t *pObj, uint32_t* val);
APDS9250status_t APDS9250_G_LS_RED_DATA_(APDS9250_Object_t *pObj, uint32_t* val);
APDS9250status_t APDS9250__G_LS_LIGHT_DATA_t(APDS9250_Object_t *pObj, uint32_t* val);

#ifdef __cplusplus
}
#endif

#endif
