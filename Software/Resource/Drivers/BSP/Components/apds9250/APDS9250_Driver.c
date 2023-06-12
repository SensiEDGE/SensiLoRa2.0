/*
 ******************************************************************************
 * @file    APDS9250_driver.c
 * @author  Sensors Software Solution Team
 * @brief   APDS9250 driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include "APDS9250_driver.h"

uint8_t Sensor_IO_Write(void* handle, uint8_t WriteAddr,
                               uint8_t* pBuffer, uint16_t nBytesToWrite);
uint8_t Sensor_IO_Read(void* handle, uint8_t ReadAddr, uint8_t* pBuffer,
                              uint16_t nBytesToRead);

/**
 * @brief  Register Component Bus IO operations
 * @param  pObj the device pObj
 * @retval 0 in case of success, an error code otherwise
 */
APDS9250status_t APDS9250_RegisterBusIO(APDS9250_Object_t *pObj, APDS9250_IO_t *pIO)
{
  int32_t ret = APDS9250_SUCCESS;

  if (pObj == NULL)
  {
    ret = APDS9250_ERROR;
  }
  else
  {
    pObj->IO.Init      = pIO->Init;
    pObj->IO.DeInit    = pIO->DeInit;
    pObj->IO.BusType   = pIO->BusType;
    pObj->IO.Address   = pIO->Address;
    pObj->IO.WriteReg  = pIO->WriteReg;
    pObj->IO.ReadReg   = pIO->ReadReg;
    pObj->handle   = pObj;

    if (pObj->IO.Init == NULL)
    {
      ret = APDS9250_ERROR;
    }
    else if (pObj->IO.Init() != APDS9250_SUCCESS)
    {
      ret = APDS9250_ERROR;
    }
  }

  return ret;
}

/*******************************************************************************
 * Function Name   : APDS9250_ReadReg
 * Description   : Generic Reading function. It must be fullfilled with either
 *         : I2C or SPI reading functions
 * Input       : Register Address
 * Output      : Data Read
 * Return      : None
 *******************************************************************************/
APDS9250status_t APDS9250_ReadReg(APDS9250_Object_t *pObj, uint8_t RegAddr,
                                  uint16_t NumByteToRead, uint8_t* Data)
{
    if (Sensor_IO_Read(pObj, RegAddr, Data, NumByteToRead))
        return APDS9250_ERROR;
    else
        return APDS9250_SUCCESS;
}

/*******************************************************************************
 * Function Name   : APDS9250_WriteReg
 * Description   : Generic Writing function. It must be fullfilled with either
 *         : I2C or SPI writing function
 * Input       : Register Address, Data to be written
 * Output      : None
 * Return      : None
 *******************************************************************************/
APDS9250status_t APDS9250_WriteReg(APDS9250_Object_t *pObj, uint8_t RegAddr,
                                   uint16_t NumByteToWrite, uint8_t* Data)
{
    if (Sensor_IO_Write(pObj, RegAddr, Data, NumByteToWrite))
        return APDS9250_ERROR;
    else
        return APDS9250_SUCCESS;
}

/**************** Base Function  *******************/

/*******************************************************************************
 * Function Name  : APDS9250_R_WHO_AM_I_
 * Description    : Read WHO_AM_I_BIT
 * Input          : Pointer to u8_t
 * Output         : Status of WHO_AM_I_BIT
 * Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
 *******************************************************************************/
APDS9250status_t APDS9250_R_WHO_AM_I_(APDS9250_Object_t *pObj, uint8_t* value)
{
    if (!APDS9250_ReadReg(pObj, APDS9250_WHO_AM_I, 1, (uint8_t*) value))
        return APDS9250_ERROR;

    return APDS9250_SUCCESS;
}

/**
 * @brief  LS mode.[set]
 *
 * @param  ctx     read / write interface definitions
 * @param  val     change the values of ls_mode in reg CTRL_REG
 * @retval         interface status (MANDATORY: return 0 -> no Error)
 *
 */
APDS9250status_t APDS9250_S_LS_MODE_(APDS9250_Object_t *pObj, apds9250_ls_enable_t val)
{
    apds9250_control_reg_t reg;

    if (!APDS9250_ReadReg(pObj, APDS9250_CTRL_REG, 1, (uint8_t*) &reg))
        return APDS9250_ERROR;

    reg.ls_en = (uint8_t) val;

    if (!APDS9250_WriteReg(pObj, APDS9250_CTRL_REG, 1, (uint8_t*) &reg))
        return APDS9250_ERROR;

    return APDS9250_SUCCESS;
}

/**
 * @brief  LS mode.[get]
 *
 * @param  ctx     read / write interface definitions
 * @param  val     Get the values of ls_mode in reg CTRL_REG
 * @retval         interface status (MANDATORY: return 0 -> no Error)
 *
 */
APDS9250status_t APDS9250_G_LS_MODE_(APDS9250_Object_t *pObj, apds9250_ls_enable_t* val)
{
    apds9250_control_reg_t reg;

    if (!APDS9250_ReadReg(pObj, APDS9250_CTRL_REG, 1, (uint8_t*) &reg))
        return APDS9250_ERROR;

    switch (reg.ls_en)
    {
    case APDS9250_LS_STANDBY:
        *val = APDS9250_LS_STANDBY;
        break;
    case APDS9250_LS_ACTIVE:
        *val = APDS9250_LS_ACTIVE;
        break;
    default:
        *val = APDS9250_LS_ND;
        break;
    }

    return APDS9250_SUCCESS;
}

/**
 * @brief  LS Resolution/Bit Width.[set]
 *
 * @param  ctx     read / write interface definitions
 * @param  val     change the values of LS Resolution/Bit Width in reg LS_MEAS_RATE
 * @retval         interface status (MANDATORY: return 0 -> no Error)
 *
 */
APDS9250status_t APDS9250_S_LS_RESOLUTION_(APDS9250_Object_t *pObj,
                                           apds9250_ls_resolution_t val)
{
    apds9250_ls_meas_rate_reg_t reg;

    if (!APDS9250_ReadReg(pObj, APDS9250_LS_MEAS_RATE, 1, (uint8_t*) &reg))
        return APDS9250_ERROR;

    reg.ls_resolution = (uint8_t) val;

    if (!APDS9250_WriteReg(pObj, APDS9250_LS_MEAS_RATE, 1, (uint8_t*) &reg))
        return APDS9250_ERROR;

    return APDS9250_SUCCESS;

}

/**
 * @brief  LS Resolution/Bit Width.[get]
 *
 * @param  ctx     read / write interface definitions
 * @param  val     Get the values of LS Resolution/Bit Width in reg LS_MEAS_RATE
 * @retval         interface status (MANDATORY: return 0 -> no Error)
 *
 */
APDS9250status_t APDS9250_G_LS_RESOLUTION_(APDS9250_Object_t *pObj,
                                           apds9250_ls_resolution_t* val)
{
    apds9250_ls_meas_rate_reg_t reg;

    if (!APDS9250_ReadReg(pObj, APDS9250_LS_MEAS_RATE, 1, (uint8_t*) &reg))
        return APDS9250_ERROR;

    switch (reg.ls_resolution)
    {
    case APDS9250_20bit:
        *val = APDS9250_20bit;
        break;
    case APDS9250_19bit:
        *val = APDS9250_19bit;
        break;
    case APDS9250_18bit:
        *val = APDS9250_18bit;
        break;
    case APDS9250_17bit:
        *val = APDS9250_17bit;
        break;
    case APDS9250_16bit:
        *val = APDS9250_16bit;
        break;
    case APDS9250_13bit:
        *val = APDS9250_13bit;
        break;
    default:
        *val = APDS9250_NDbit;
        break;
    }

    return APDS9250_SUCCESS;
}

/**
 * @brief  LS Measurement Rate.[set]
 *
 * @param  ctx     read / write interface definitions
 * @param  val     change the values of LS Measurement Rate in reg LS_MEAS_RATE
 * @retval         interface status (MANDATORY: return 0 -> no Error)
 *
 */
APDS9250status_t APDS9250_S_LS_RATE_(APDS9250_Object_t *pObj, apds9250_ls_rate_t val)
{
    apds9250_ls_meas_rate_reg_t reg;

    if (!APDS9250_ReadReg(pObj, APDS9250_LS_MEAS_RATE, 1, (uint8_t*) &reg))
        return APDS9250_ERROR;

    reg.ls_meas = (uint8_t) val;

    if (!APDS9250_WriteReg(pObj, APDS9250_LS_MEAS_RATE, 1, (uint8_t*) &reg))
        return APDS9250_ERROR;

    return APDS9250_SUCCESS;

}

/**
 * @brief  LS Measurement Rate.[get]
 *
 * @param  ctx     read / write interface definitions
 * @param  val     Get the values of LS Measurement Rate in reg LS_MEAS_RATE
 * @retval         interface status (MANDATORY: return 0 -> no Error)
 *
 */
APDS9250status_t APDS9250_G_LS_RATE_(APDS9250_Object_t *pObj, apds9250_ls_rate_t* val)
{
    apds9250_ls_meas_rate_reg_t reg;

    if (!APDS9250_ReadReg(pObj, APDS9250_LS_MEAS_RATE, 1, (uint8_t*) &reg))
        return APDS9250_ERROR;

    switch (reg.ls_meas)
    {
    case APDS9250_25ms:
        *val = APDS9250_25ms;
        break;
    case APDS9250_50ms:
        *val = APDS9250_50ms;
        break;
    case APDS9250_100ms:
        *val = APDS9250_100ms;
        break;
    case APDS9250_200ms:
        *val = APDS9250_200ms;
        break;
    case APDS9250_500ms:
        *val = APDS9250_500ms;
        break;
    case APDS9250_1000ms:
        *val = APDS9250_1000ms;
        break;
    case APDS9250_2000ms:
        *val = APDS9250_2000ms;
        break;
    case APDS9250_5000ms:
        *val = APDS9250_5000ms;
        break;
    default:
        *val = APDS9250_NDms;
        break;
    }

    return APDS9250_SUCCESS;
}

/**
 * @brief  IR channel output data.[get]
 *
 * @param  ctx     read / write interface definitions
 * @param  val     Get the values of IR channel output data in reg LS_DATA_IR
 * @retval         interface status (MANDATORY: return 0 -> no Error)
 *
 */
APDS9250status_t APDS9250_G_LS_IR_DATA_(APDS9250_Object_t *pObj, uint32_t* val)
{
    uint8_t adc_data[3];

    if (!APDS9250_ReadReg(pObj, APDS9250_DATA_IR0, 3, adc_data))
        return APDS9250_ERROR;

    *val = ((uint32_t) (adc_data[0]) & 0x000000ff) | (((uint32_t) (adc_data[1])
            & 0x000000ff)
                                                      << 8)
           | (((uint32_t) (adc_data[2]) & 0x000000ff) << 16);

    return APDS9250_SUCCESS;
}

/**
 * @brief  ALS/CS Green channel output data.[get]
 *
 * @param  ctx     read / write interface definitions
 * @param  val     Get the values of ALS/CS Green channel output data in reg LS_DATA_GREEN
 * @retval         interface status (MANDATORY: return 0 -> no Error)
 *
 */
APDS9250status_t APDS9250_G_LS_GREEN_DATA_(APDS9250_Object_t *pObj, uint32_t* val)
{
    uint8_t adc_data[3];

    if (!APDS9250_ReadReg(pObj, APDS9250_DATA_GREEN0, 3, adc_data))
        return APDS9250_ERROR;

    *val = ((uint32_t) (adc_data[0]) & 0x000000ff) | (((uint32_t) (adc_data[1])
            & 0x000000ff)
                                                      << 8)
           | (((uint32_t) (adc_data[2]) & 0x000000ff) << 16);

    return APDS9250_SUCCESS;
}

/**
 * @brief  CS Blue channel output data.[get]
 *
 * @param  ctx     read / write interface definitions
 * @param  val     Get the values of CS Blue channel output data in reg LS_DATA_BLUE
 * @retval         interface status (MANDATORY: return 0 -> no Error)
 *
 */
APDS9250status_t APDS9250_G_LS_BLUE_DATA_(APDS9250_Object_t *pObj, uint32_t* val)
{
    uint8_t adc_data[3];

    if (!APDS9250_ReadReg(pObj, APDS9250_DATA_BLUE0, 3, adc_data))
        return APDS9250_ERROR;

    *val = ((uint32_t) (adc_data[0]) & 0x000000ff) | (((uint32_t) (adc_data[1])
            & 0x000000ff)
                                                      << 8)
           | (((uint32_t) (adc_data[2]) & 0x000000ff) << 16);

    return APDS9250_SUCCESS;
}

/**
 * @brief  CS Red channel output data.[get]
 *
 * @param  ctx     read / write interface definitions
 * @param  val     Get the values of CS Red channel output data in reg LS_DATA_RED
 * @retval         interface status (MANDATORY: return 0 -> no Error)
 *
 */
APDS9250status_t APDS9250_G_LS_RED_DATA_(APDS9250_Object_t *pObj, uint32_t* val)
{
    uint8_t adc_data[3];

    if (!APDS9250_ReadReg(pObj, APDS9250_DATA_RED0, 3, adc_data))
        return APDS9250_ERROR;

    *val = ((uint32_t) (adc_data[0]) & 0x000000ff) | (((uint32_t) (adc_data[1])
            & 0x000000ff)
                                                      << 8)
           | (((uint32_t) (adc_data[2]) & 0x000000ff) << 16);

    return APDS9250_SUCCESS;
}

/**
 * @brief  Light.[get]
 *
 * @param  ctx     read / write interface definitions
 * @param  val     Get the values of Light data
 * @retval         interface status (MANDATORY: return 0 -> no Error)
 *
 */
APDS9250status_t APDS9250__G_LS_LIGHT_DATA_t(APDS9250_Object_t *pObj, uint32_t* val)
{
    uint8_t adc_data[6];

    if (!APDS9250_ReadReg(pObj, APDS9250_DATA_IR0, 6, adc_data))
        return APDS9250_ERROR;

    uint32_t ir_value = ((uint32_t) (adc_data[0]) & 0x000000ff)
            | (((uint32_t) (adc_data[1]) & 0x000000ff) << 8)
            | (((uint32_t) (adc_data[2]) & 0x000000ff) << 16);

    uint32_t green_value = ((uint32_t) (adc_data[3]) & 0x000000ff)
            | (((uint32_t) (adc_data[4]) & 0x000000ff) << 8)
            | (((uint32_t) (adc_data[5]) & 0x000000ff) << 16);

    uint32_t factor = ir_value > green_value ? 35 : 46;

    uint32_t lux = ((green_value * factor) / 3) / 100;
//        *val = (uint16_t) lux;
    *val = lux;

    return APDS9250_SUCCESS;
}

uint8_t Sensor_IO_Write(void* handle, uint8_t WriteAddr, uint8_t* pBuffer, uint16_t nBytesToWrite)
{
	APDS9250_Object_t *pObj = (APDS9250_Object_t *)handle;

  return pObj->IO.WriteReg(pObj->IO.Address, WriteAddr, pBuffer, nBytesToWrite);
}

uint8_t Sensor_IO_Read(void* handle, uint8_t ReadAddr, uint8_t* pBuffer, uint16_t nBytesToRead)
{
	APDS9250_Object_t *pObj = (APDS9250_Object_t *)handle;

  return pObj->IO.ReadReg(pObj->IO.Address, ReadAddr, pBuffer, nBytesToRead);
}

/**
 * @}
 *
 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
