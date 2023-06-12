/**
 ******************************************************************************
 * @file    SensiLoraBSP.c
 * @date    15-June-2022
 * @brief   The SensiLoraBSP source file.
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

#include "SensiLoraBSP.h"
#include <stdlib.h>
#include "stm32_lpm.h"
#include "sys_app.h"
#include "radio_board_if.h"
#include "stm32_timer.h"
#include "sys_sensors.h"
#include "Usb.h"
#include "sx1276.h"
#include "LmHandler.h"
#include "adc.h"

/**
  * @brief 32L082MLM DISCO BSP Driver version number V1.0.0
  */
#define __B_L072Z_LRWAN1_BSP_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __B_L072Z_LRWAN1_BSP_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __B_L072Z_LRWAN1_BSP_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __B_L072Z_LRWAN1_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __B_L072Z_LRWAN1_BSP_VERSION         ((__B_L072Z_LRWAN1_BSP_VERSION_MAIN << 24)\
                                              |(__B_L072Z_LRWAN1_BSP_VERSION_SUB1 << 16)\
                                              |(__B_L072Z_LRWAN1_BSP_VERSION_SUB2 << 8 )\
                                              |(__B_L072Z_LRWAN1_BSP_VERSION_RC))

/** @defgroup B-L072Z-LRWAN1_LOW_LEVEL_Private_Variables
  * @{
  */
GPIO_TypeDef *LED_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT};
const uint16_t LED_PIN[LEDn] = {LED1_PIN, LED2_PIN};

GPIO_TypeDef *BUTTON_PORT[BUTTONn] = {KEY_BUTTON_GPIO_PORT };
const uint16_t BUTTON_PIN[BUTTONn] = {KEY_BUTTON_PIN };
const uint8_t BUTTON_IRQn[BUTTONn] = {KEY_BUTTON_EXTI_IRQn };
static bool flagPowerOff = false;

/**
  * @brief  This method returns the B-L072Z-LRWAN1 BSP Driver revision
  * @param  None
  * @retval version : 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BspGetVersion(void)
{
  return __B_L072Z_LRWAN1_BSP_VERSION;
}

/**
  * @brief  Init pin CS Flash
  * @param  None
  * @retval None
  */
void BspCsFlashInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	/* Enable the GPIO_LED Clock */
	CS_FLASH_GPIO_CLK_ENABLE();

	/* Configure the GPIO_LED pin */
	GPIO_InitStruct.Pin = CS_FLASH_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	HAL_GPIO_Init(CS_FLASH_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  Enable pin CS Flash
  * @param  None
  * @retval None
  */
void BspCsFlashOn(void)
{
	HAL_GPIO_WritePin(CS_FLASH_GPIO_PORT, CS_FLASH_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  Disable pin CS Flash
  * @param  None
  * @retval None
  */
void  BspCsFlashOff(void)
{
	HAL_GPIO_WritePin(CS_FLASH_GPIO_PORT, CS_FLASH_PIN, GPIO_PIN_SET);
}

/**
  * @brief  Init pin measurement voltage Battery
  * @param  None
  * @retval None
  */
void BspMeasBatInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	/* Enable the GPIO_LED Clock */
	V_BAT_MEAS_GPIO_CLK_ENABLE();

	/* Configure the GPIO_LED pin */
	GPIO_InitStruct.Pin = V_BAT_MEAS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	HAL_GPIO_Init(V_BAT_MEAS_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  Enable pin  measurement voltage Battery
  * @param  None
  * @retval None
  */
void BspMeasBatOn(void)
{
	HAL_GPIO_WritePin(V_BAT_MEAS_GPIO_PORT, V_BAT_MEAS_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  Disable pin  measurement voltage Battery
  * @param  None
  * @retval None
  */
void  BspMeasBatOff(void)
{
    HAL_GPIO_WritePin(V_BAT_MEAS_GPIO_PORT, V_BAT_MEAS_PIN, GPIO_PIN_SET);
}

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval None
  */
void BspLedInit(Led_TypeDef Led)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    /* Enable the GPIO_LED Clock */
    LEDx_GPIO_CLK_ENABLE(Led);

    /* Configure the GPIO_LED pin */
    GPIO_InitStruct.Pin = LED_PIN[Led];
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    HAL_GPIO_Init(LED_PORT[Led], &GPIO_InitStruct);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval None
  */
void BspLedOn(Led_TypeDef Led)
{
    HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_SET);
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval None
  */
void BspLedOff(Led_TypeDef Led)
{
    HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET);
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval None
  */
void BspLedToggle(Led_TypeDef Led)
{
    HAL_GPIO_TogglePin(LED_PORT[Led], LED_PIN[Led]);
}

/**
  * @brief  Get on or off Led.
  * @param  Led: Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval true - on Led, false - off Led
  */
bool BspLedStatus(Led_TypeDef Led)
{
	return HAL_GPIO_ReadPin(LED_PORT[Led], LED_PIN[Led]);
}

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter should be: BUTTON_KEY
  * @param  ButtonMode: Specifies Button mode.
  *   This parameter can be one of following parameters:
  *     @arg  BUTTON_MODE_GPIO: Button will be used as simple IO
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                            generation capability
  * @retval None
  */
void BspButtonInit(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Enable the BUTTON Clock */
    BUTTONx_GPIO_CLK_ENABLE(Button);
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    if (ButtonMode == BUTTON_MODE_GPIO) {
        /* Configure Button pin as input */
        GPIO_InitStruct.Pin = BUTTON_PIN[Button];
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
    }

    if (ButtonMode == BUTTON_MODE_EXTI) {
        /* Configure Button pin as input with External interrupt */
        GPIO_InitStruct.Pin = BUTTON_PIN[Button];
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
        HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);

        /* Enable and set Button EXTI Interrupt to the lowest priority */
        NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0);
        HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
    }
}

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter should be: BUTTON_KEY
  * @retval The Button GPIO pin value.
  */
uint32_t BspButtonGetState(Button_TypeDef Button)
{
    return HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

/**
  * @brief  Button interrupt handler
  * @param  None
  * @retval None
  */
void BspButtonHandler(void)
{
    if (UsbComPortStatus()) {
        return;
    }

    uint32_t l_timeOut = 2000 + HAL_GetTick();
    UTIL_ADV_TRACE_PreSendHook();

    do {
        if (BspButtonGetState(BUTTON_USER) ==  GPIO_PIN_SET) {
            UTIL_ADV_TRACE_PostSendHook();
            __enable_irq();
            return;
        }
    } while(l_timeOut > HAL_GetTick());

    flagPowerOff = true;
}

/**
  * @brief  Status Sleep mode for board
  * @param  None
  * @retval true - go to power off device, false - not power off device
  */
bool BspIsPowerOff(void)
{
    return flagPowerOff;
}

/**
* @brief Interrupt callback for GPIOs
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    if ( GPIO_Pin == USER_BUTTON_PIN) {
        BspButtonHandler();
    }
}
