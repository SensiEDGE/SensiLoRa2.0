/**
 ******************************************************************************
 * @file    SensiLoraConf.h
 * @date    30-June-2022
 * @brief   This file is configuration sensors on board.
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
#ifndef _SENSI_LORA_CONF_H
#define _SENSI_LORA_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "SensiLoraBus.h"

#define SENSI_LORA_I2C_Init BusI2C1_Init
#define SENSI_LORA_I2C_DeInit BusI2C1_DeInit
#define SENSI_LORA_I2C_ReadReg BusI2C1_ReadReg
#define SENSI_LORA_I2C_WriteReg BusI2C1_WriteReg

#define SENSI_LORA_SPI_Init BusSPI2_Init
#define SENSI_LORA_SPI_DeInit BusSPI2_DeInit
#define SENSI_LORA_SPI_WRITE BusSPI2_Send
#define SENSI_LORA_SPI_READ BusSPI2_Recv
#define SENSI_LORA_SPI_FLUSH_RX BusSPI2_FlushRx
#define SENSI_LORA_GetTick BusGetTick

#ifdef __cplusplus
}
#endif

#endif /* _SENSI_LORA_CONF_H */
