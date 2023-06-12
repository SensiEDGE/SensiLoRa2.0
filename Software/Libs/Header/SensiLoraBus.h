/**
 ******************************************************************************
 * @file    SensiLoraBus.h
 * @date    15-June-2022
 * @brief   The SensiLoraBus description.
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
#ifndef __SENSILOLRA_BUS_H
#define __SENSILOLRA_BUS_H

#include "stm32l0xx_nucleo_conf.h"
#include "b_l072z_lrwan1_errno.h"
#include "stdbool.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BUS_SPI1_SCK_GPIO_PIN            GPIO_PIN_3
#define BUS_SPI1_MISO_GPIO_PIN           GPIO_PIN_6
#define BUS_SPI1_MOSI_GPIO_PIN           GPIO_PIN_7
#define BUS_SPI1_SCK_GPIO_PORT           GPIOB
#define BUS_SPI1_MISO_GPIO_PORT          GPIOA
#define BUS_SPI1_MOSI_GPIO_PORT          GPIOA
#define BUS_SPI1_SCK_GPIO_AF             GPIO_AF0_SPI1
#define BUS_SPI1_MOSI_GPIO_AF            GPIO_AF0_SPI1
#define BUS_SPI1_MISO_GPIO_AF            GPIO_AF0_SPI1
#define BUS_SPI1_SCK_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define BUS_SPI1_MOSI_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define BUS_SPI1_MISO_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

#define BUS_SPI2_SCK_GPIO_PIN            GPIO_PIN_13
#define BUS_SPI2_MISO_GPIO_PIN           GPIO_PIN_14
#define BUS_SPI2_MOSI_GPIO_PIN           GPIO_PIN_15
#define BUS_SPI2_SCK_GPIO_PORT           GPIOB
#define BUS_SPI2_MISO_GPIO_PORT          GPIOB
#define BUS_SPI2_MOSI_GPIO_PORT          GPIOB
#define BUS_SPI2_SCK_GPIO_AF             GPIO_AF0_SPI2
#define BUS_SPI2_MOSI_GPIO_AF            GPIO_AF0_SPI2
#define BUS_SPI2_MISO_GPIO_AF            GPIO_AF0_SPI2
#define BUS_SPI2_SCK_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define BUS_SPI2_MOSI_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define BUS_SPI2_MISO_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()

#define BUS_I2C1_INSTANCE I2C1
#define BUS_I2C1_SCL_GPIO_PORT GPIOB
#define BUS_I2C1_SCL_GPIO_AF GPIO_AF4_I2C1
#define BUS_I2C1_SCL_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define BUS_I2C1_SCL_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()
#define BUS_I2C1_SCL_GPIO_PIN GPIO_PIN_8
#define BUS_I2C1_SDA_GPIO_PIN GPIO_PIN_9
#define BUS_I2C1_SDA_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()
#define BUS_I2C1_SDA_GPIO_PORT GPIOB
#define BUS_I2C1_SDA_GPIO_AF GPIO_AF4_I2C1
#define BUS_I2C1_SDA_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

int32_t BusGetTick(void);

int32_t BusSPI1_Init(void);
int32_t BusSPI1_DeInit(void);
int32_t BusSPI1_Send(uint8_t *pData, uint16_t len);
int32_t BusSPI1_Recv(uint8_t *pData, uint16_t len);
int32_t BusSPI1_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t len);

int32_t BusSPI2_Init(void);
int32_t BusSPI2_DeInit(void);
int32_t BusSPI2_Send(uint8_t *pData, uint16_t len, uint32_t timeout);
int32_t BusSPI2_Recv(uint8_t *pData, uint16_t len, uint32_t timeout);
int32_t BusSPI2_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t len);

int32_t BusI2C1_Init(void);
int32_t BusI2C1_DeInit(void);
int32_t BusI2C1_IsReady(uint16_t DevAddr, uint32_t Trials);
int32_t BusI2C1_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BusI2C1_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BusI2C1_WriteReg16(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BusI2C1_ReadReg16(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BusI2C1_Send(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BusI2C1_Recv(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BusI2C1_SendRecv(uint16_t DevAddr, uint8_t *pTxdata, uint8_t *pRxdata, uint16_t Length);

#ifdef __cplusplus
}
#endif

#endif /* __SENSILOLRA_BUS_H */

