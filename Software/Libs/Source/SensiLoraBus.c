/**
 ******************************************************************************
 * @file    SensiLoraBus.h
 * @date    15-June-2022
 * @brief   The SensiLoraBSP description.
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

/* Includes ------------------------------------------------------------------*/
#include "SensiLoraBus.h"
#include "SensiLoraErrno.h"
#include "stm32l0xx_hal.h"

#define TIMEOUT_DURATION 2000
#define RADIO_SPI_BAUDRATE              10000000U /* 16M Sigfox, 10M Lora */
#define FLASH_SPI_BAUDRATE              1000000U
#define BUS_I2C1_POLL_TIMEOUT           0x1000U

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
I2C_HandleTypeDef hi2c1;

__weak HAL_StatusTypeDef MX_SPI2_Init(SPI_HandleTypeDef *hspi);
__weak HAL_StatusTypeDef MX_SPI1_Init(SPI_HandleTypeDef *hspi);
__weak HAL_StatusTypeDef MX_I2C1_Init(I2C_HandleTypeDef *hi2c);

static void SPI1_MspInit(SPI_HandleTypeDef *spiHandle);
static void SPI1_MspDeInit(SPI_HandleTypeDef *spiHandle);
static void SPI2_MspInit(SPI_HandleTypeDef *spiHandle);
static void SPI2_MspDeInit(SPI_HandleTypeDef *spiHandle);
static uint32_t SPI_GetPrescaler(uint32_t clock_src_hz, uint32_t baudrate_mbps);
static void I2C1_MspInit(I2C_HandleTypeDef *hI2c);
static void I2C1_MspDeInit(I2C_HandleTypeDef *hI2c);

/**
  * @brief  Initializes SPI HAL.
  * @retval None
  * @retval BSP status
  */
int32_t BusSPI1_Init(void)
{
  int32_t ret = BSP_ERROR_NONE;

  hspi1.Instance  = SPI1;
  if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_RESET)
  {
    /* Init the SPI Msp */
    SPI1_MspInit(&hspi1);

    /* Init the SPI */
    if (MX_SPI1_Init(&hspi1) != HAL_OK)
    {
      ret = BSP_ERROR_BUS_FAILURE;
    }
  }

  return ret;
}

/**
  * @brief  DeInitializes SPI HAL.
  * @retval None
  * @retval BSP status
  */
int32_t BusSPI1_DeInit(void)
{
  int32_t ret = BSP_ERROR_BUS_FAILURE;

  SPI1_MspDeInit(&hspi1);

  if (HAL_SPI_DeInit(&hspi1) == HAL_OK)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief  Write Data through SPI BUS.
  * @param  pData: Data
  * @param  len: Length of data in byte
  * @retval BSP status
  */
int32_t BusSPI1_Send(uint8_t *pData, uint16_t len)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

  if (HAL_SPI_Transmit(&hspi1, pData, len, TIMEOUT_DURATION) == HAL_OK)
  {
    ret = len;
  }
  return ret;
}

/**
  * @brief  Receive Data from SPI BUS
  * @param  pData: Data
  * @param  len: Length of data in byte
  * @retval BSP status
  */
int32_t BusSPI1_Recv(uint8_t *pData, uint16_t len)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

  if (HAL_SPI_Receive(&hspi1, pData, len, TIMEOUT_DURATION) == HAL_OK)
  {
    ret = len;
  }
  return ret;
}

/**
  * @brief  Send and Receive data to/from SPI BUS (Full duplex)
  * @param  pData: Data
  * @param  len: Length of data in byte
  * @retval BSP status
  */
int32_t BusSPI1_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t len)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

  if (HAL_SPI_TransmitReceive(&hspi1, pTxData, pRxData, len, TIMEOUT_DURATION) == HAL_OK)
  {
    ret = len;
  }
  return ret;
}

/* SPI1 init function */

__weak HAL_StatusTypeDef MX_SPI1_Init(SPI_HandleTypeDef *hspi)
{
  HAL_StatusTypeDef ret = HAL_OK;
  hspi->Instance = SPI1;
  hspi->Init.Mode = SPI_MODE_MASTER;
  hspi->Init.Direction = SPI_DIRECTION_2LINES;
  hspi->Init.DataSize = SPI_DATASIZE_8BIT;
  hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi->Init.NSS = SPI_NSS_SOFT;
  /* SPI1 is on APB2 for L0x3 -> HAL_RCC_GetPCLK2Freq */
  hspi->Init.BaudRatePrescaler = SPI_GetPrescaler(HAL_RCC_GetPCLK2Freq(), RADIO_SPI_BAUDRATE);
  hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi->Init.TIMode = SPI_TIMODE_DISABLE;
  hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi->Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(hspi) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  return ret;
}

static void SPI1_MspInit(SPI_HandleTypeDef *spiHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
  /* Enable Peripheral clock */
  __HAL_RCC_SPI1_CLK_ENABLE();

  /**SPI1 GPIO Configuration
  PA5     ------> SPI1_SCK
  PA6     ------> SPI1_MISO
  PA7     ------> SPI1_MOSI
    */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = BUS_SPI1_MOSI_GPIO_AF;
  GPIO_InitStruct.Pin = BUS_SPI1_MOSI_GPIO_PIN;
  HAL_GPIO_Init(BUS_SPI1_MOSI_GPIO_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Alternate = BUS_SPI1_MISO_GPIO_AF;
  GPIO_InitStruct.Pin = BUS_SPI1_MISO_GPIO_PIN;
  HAL_GPIO_Init(BUS_SPI1_MISO_GPIO_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Alternate = BUS_SPI1_SCK_GPIO_AF;
  GPIO_InitStruct.Pin = BUS_SPI1_SCK_GPIO_PIN;
  HAL_GPIO_Init(BUS_SPI1_SCK_GPIO_PORT, &GPIO_InitStruct);


  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
}

static void SPI1_MspDeInit(SPI_HandleTypeDef *spiHandle)
{
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
  /* Peripheral clock disable */
  __HAL_RCC_SPI1_CLK_DISABLE();

  /**SPI1 GPIO Configuration
  PA5     ------> SPI1_SCK
  PA6     ------> SPI1_MISO
  PA7     ------> SPI1_MOSI
    */
  HAL_GPIO_DeInit(BUS_SPI1_MOSI_GPIO_PORT, BUS_SPI1_MOSI_GPIO_PIN);
  HAL_GPIO_DeInit(BUS_SPI1_MISO_GPIO_PORT, BUS_SPI1_MISO_GPIO_PIN);
  HAL_GPIO_DeInit(BUS_SPI1_SCK_GPIO_PORT, BUS_SPI1_SCK_GPIO_PIN);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
}

/**
  * @brief  Initializes SPI HAL.
  * @retval None
  * @retval BSP status
  */
int32_t BusSPI2_Init(void)
{
  int32_t ret = BSP_ERROR_NONE;

  if (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_RESET)
  {
    /* Init the SPI Msp */
    SPI2_MspInit(&hspi2);


    /* Init the SPI */
    if (MX_SPI2_Init(&hspi2) != HAL_OK)
    {
      ret = BSP_ERROR_BUS_FAILURE;
    }
  }

  return ret;
}

/**
  * @brief  DeInitializes SPI HAL.
  * @retval None
  * @retval BSP status
  */
int32_t BusSPI2_DeInit(void)
{
  int32_t ret = BSP_ERROR_BUS_FAILURE;

  SPI2_MspDeInit(&hspi2);

  if (HAL_SPI_DeInit(&hspi2) == HAL_OK)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief  Write Data through SPI BUS.
  * @param  pData: Data
  * @param  len: Length of data in byte
  * @retval BSP status
  */
int32_t BusSPI2_Send(uint8_t *pData, uint16_t len, uint32_t timeout)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

  if (HAL_SPI_Transmit(&hspi2, pData, len, timeout) == HAL_OK)
  {
    ret = len;
  }
  return ret;
}

/**
  * @brief  Receive Data from SPI BUS
  * @param  pData: Data
  * @param  len: Length of data in byte
  * @retval BSP status
  */
int32_t BusSPI2_Recv(uint8_t *pData, uint16_t len, uint32_t timeout)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

  if (HAL_SPI_Receive(&hspi2, pData, len, timeout) == HAL_OK)
  {
    ret = len;
  }
  return ret;
}

/**
  * @brief  Send and Receive data to/from SPI BUS (Full duplex)
  * @param  pData: Data
  * @param  len: Length of data in byte
  * @retval BSP status
  */
int32_t BusSPI2_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t len)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

  if (HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, len, TIMEOUT_DURATION) == HAL_OK)
  {
    ret = len;
  }
  return ret;
}

/**
  * @brief  Return system tick in ms
  * @retval Current HAL time base time stamp
  */
int32_t BusGetTick(void)
{
  return HAL_GetTick();
}

/* SPI1 init function */

__weak HAL_StatusTypeDef MX_SPI2_Init(SPI_HandleTypeDef *hspi)
{
  HAL_StatusTypeDef ret = HAL_OK;
  hspi->Instance = SPI2;
  hspi->Init.Mode = SPI_MODE_MASTER;
  hspi->Init.Direction = SPI_DIRECTION_2LINES;
  hspi->Init.DataSize = SPI_DATASIZE_8BIT;
  hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi->Init.NSS = SPI_NSS_SOFT;
  hspi->Init.BaudRatePrescaler = SPI_GetPrescaler(HAL_RCC_GetPCLK2Freq(), FLASH_SPI_BAUDRATE);
  hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi->Init.TIMode = SPI_TIMODE_DISABLE;
  hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi->Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(hspi) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  return ret;
}

static void SPI2_MspInit(SPI_HandleTypeDef *spiHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
  /* Enable Peripheral clock */
  __HAL_RCC_SPI2_CLK_ENABLE();


  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = BUS_SPI2_MOSI_GPIO_AF;
  GPIO_InitStruct.Pin = BUS_SPI2_MOSI_GPIO_PIN;
  HAL_GPIO_Init(BUS_SPI2_MOSI_GPIO_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Alternate = BUS_SPI2_MISO_GPIO_AF;
  GPIO_InitStruct.Pin = BUS_SPI2_MISO_GPIO_PIN;
  HAL_GPIO_Init(BUS_SPI2_MISO_GPIO_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Alternate = BUS_SPI2_SCK_GPIO_AF;
  GPIO_InitStruct.Pin = BUS_SPI2_SCK_GPIO_PIN;
  HAL_GPIO_Init(BUS_SPI2_SCK_GPIO_PORT, &GPIO_InitStruct);
  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
}

static void SPI2_MspDeInit(SPI_HandleTypeDef *spiHandle)
{
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
  /* Peripheral clock disable */
  __HAL_RCC_SPI2_CLK_DISABLE();

  /**SPI1 GPIO Configuration
  PA5     ------> SPI1_SCK
  PA6     ------> SPI1_MISO
  PA7     ------> SPI1_MOSI
    */
  HAL_GPIO_DeInit(BUS_SPI2_MOSI_GPIO_PORT, BUS_SPI2_MOSI_GPIO_PIN);
  HAL_GPIO_DeInit(BUS_SPI2_MISO_GPIO_PORT, BUS_SPI2_MISO_GPIO_PIN);
  HAL_GPIO_DeInit(BUS_SPI2_SCK_GPIO_PORT, BUS_SPI2_SCK_GPIO_PIN);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
}

static uint32_t SPI_GetPrescaler(uint32_t clock_src_hz, uint32_t baudrate_mbps)
{
  uint32_t divisor = 0;
  uint32_t spi_clk = clock_src_hz;
  uint32_t presc = 0;

  static const uint32_t baudrate[] =
  {
    SPI_BAUDRATEPRESCALER_2,
    SPI_BAUDRATEPRESCALER_4,
    SPI_BAUDRATEPRESCALER_8,
    SPI_BAUDRATEPRESCALER_16,
    SPI_BAUDRATEPRESCALER_32,
    SPI_BAUDRATEPRESCALER_64,
    SPI_BAUDRATEPRESCALER_128,
    SPI_BAUDRATEPRESCALER_256,
  };

  while (spi_clk > baudrate_mbps)
  {
    presc = baudrate[divisor];
    if (++divisor > 7)
    {
      break;
    }

    spi_clk = (spi_clk >> 1);
  }

  return presc;
}

/**
  * @brief  Initialize I2C HAL
  * @retval BSP status
  */
int32_t BusI2C1_Init(void)
{

  int32_t ret = BSP_ERROR_NONE;

  hi2c1.Instance  = I2C1;

  if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_RESET)
  {
    /* Init the I2C Msp */
    I2C1_MspInit(&hi2c1);
    if (ret == BSP_ERROR_NONE)
    {
        /* Init the I2C */
        if (MX_I2C1_Init(&hi2c1) != HAL_OK)
        {
          ret = BSP_ERROR_BUS_FAILURE;
        }
        else if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
        {
          ret = BSP_ERROR_BUS_FAILURE;
        }
        else
        {
          ret = BSP_ERROR_NONE;
        }
      }
    }
  return ret;
}

/**
  * @brief  DeInitialize I2C HAL.
  * @retval BSP status
  */
int32_t BusI2C1_DeInit(void)
{
  int32_t ret = BSP_ERROR_NONE;

  /* DeInit the I2C */
  I2C1_MspDeInit(&hi2c1);

  /* DeInit the I2C */
  if (HAL_I2C_DeInit(&hi2c1) != HAL_OK)
  {
    ret = BSP_ERROR_BUS_FAILURE;
  }

  return ret;
}

/**
  * @brief  Check whether the I2C bus is ready.
  * @param DevAddr : I2C device address
  * @param Trials : Check trials number
  * @retval BSP status
  */
int32_t BusI2C1_IsReady(uint16_t DevAddr, uint32_t Trials)
{
  int32_t ret = BSP_ERROR_NONE;

  if (HAL_I2C_IsDeviceReady(&hi2c1, DevAddr, Trials, BUS_I2C1_POLL_TIMEOUT) != HAL_OK)
  {
    ret = BSP_ERROR_BUSY;
  }

  return ret;
}

/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  DevAddr Device address on Bus.
  * @param  Reg    The target register address to write
  * @param  pData  Pointer to data buffer to write
  * @param  Length Data Length
  * @retval BSP status
  */

int32_t BusI2C1_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  if (HAL_I2C_Mem_Write(&hi2c1, DevAddr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, BUS_I2C1_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF)
    {
      ret = BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

/**
  * @brief  Read a register of the device through BUS
  * @param  DevAddr Device address on Bus.
  * @param  Reg    The target register address to read
  * @param  pData  Pointer to data buffer to read
  * @param  Length Data Length
  * @retval BSP status
  */
int32_t  BusI2C1_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  if (HAL_I2C_Mem_Read(&hi2c1, DevAddr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, BUS_I2C1_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF)
    {
      ret = BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

/**

  * @brief  Write a value in a register of the device through BUS.
  * @param  DevAddr Device address on Bus.
  * @param  Reg    The target register address to write

  * @param  pData  Pointer to data buffer to write
  * @param  Length Data Length
  * @retval BSP statu
  */
int32_t BusI2C1_WriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  if (HAL_I2C_Mem_Write(&hi2c1, DevAddr, Reg, I2C_MEMADD_SIZE_16BIT, pData, Length, BUS_I2C1_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF)
    {
      ret = BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

/**
  * @brief  Read registers through a bus (16 bits)
  * @param  DevAddr: Device address on BUS
  * @param  Reg: The target register address to read
  * @param  Length Data Length
  * @retval BSP status
  */
int32_t  BusI2C1_ReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  if (HAL_I2C_Mem_Read(&hi2c1, DevAddr, Reg, I2C_MEMADD_SIZE_16BIT, pData, Length, BUS_I2C1_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      ret =  BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

/**
  * @brief  Send an amount width data through bus (Simplex)
  * @param  DevAddr: Device address on Bus.
  * @param  pData: Data pointer
  * @param  Length: Data length
  * @retval BSP status
  */
int32_t BusI2C1_Send(uint16_t DevAddr, uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  if (HAL_I2C_Master_Transmit(&hi2c1, DevAddr, pData, Length, BUS_I2C1_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      ret = BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }

  return ret;
}

/**
  * @brief  Receive an amount of data through a bus (Simplex)
  * @param  DevAddr: Device address on Bus.
  * @param  pData: Data pointer
  * @param  Length: Data length
  * @retval BSP status
  */
int32_t BusI2C1_Recv(uint16_t DevAddr, uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  if (HAL_I2C_Master_Receive(&hi2c1, DevAddr, pData, Length, BUS_I2C1_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      ret = BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

/* I2C1 init function */

__weak HAL_StatusTypeDef MX_I2C1_Init(I2C_HandleTypeDef *hi2c)
{
  HAL_StatusTypeDef ret = HAL_OK;
  hi2c->Instance = I2C1;
  hi2c->Init.Timing = 0x00300F38;
  hi2c->Init.OwnAddress1 = 0;
  hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c->Init.OwnAddress2 = 0;
  hi2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(hi2c) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  if (HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  if (HAL_I2CEx_ConfigDigitalFilter(hi2c, 0) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  return ret;
}

static void I2C1_MspInit(I2C_HandleTypeDef *i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**I2C1 GPIO Configuration
  PB8     ------> I2C1_SCL
  PB9     ------> I2C1_SDA
    */
  GPIO_InitStruct.Pin = BUS_I2C1_SCL_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = BUS_I2C1_SCL_GPIO_AF;
  HAL_GPIO_Init(BUS_I2C1_SCL_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BUS_I2C1_SDA_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = BUS_I2C1_SDA_GPIO_AF;
  HAL_GPIO_Init(BUS_I2C1_SDA_GPIO_PORT, &GPIO_InitStruct);

  /* Peripheral clock enable */
  __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
}

static void I2C1_MspDeInit(I2C_HandleTypeDef *i2cHandle)
{
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
  /* Peripheral clock disable */
  __HAL_RCC_I2C1_CLK_DISABLE();

  /**I2C1 GPIO Configuration
  PB8     ------> I2C1_SCL
  PB9     ------> I2C1_SDA
    */
  HAL_GPIO_DeInit(BUS_I2C1_SCL_GPIO_PORT, BUS_I2C1_SCL_GPIO_PIN);

  HAL_GPIO_DeInit(BUS_I2C1_SDA_GPIO_PORT, BUS_I2C1_SDA_GPIO_PIN);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
