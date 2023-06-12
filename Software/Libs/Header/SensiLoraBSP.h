/**
 ******************************************************************************
 * @file    SensiLoraBSP.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSI_LORA_BSP_H
#define __SENSI_LORA_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "stdlib.h"
#include <stdbool.h>

typedef enum
{
  LED1 = 0,
  LED_GREEN = LED1,
  LED2 = 1,
  LED_RED = LED2,
} Led_TypeDef;

typedef enum
{
  BUTTON_USER = 0,
  /* Alias */
  BUTTON_KEY  = BUTTON_USER
} Button_TypeDef;

typedef enum
{
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;

#define LEDn                               2

#define LED1_PIN                           GPIO_PIN_5
#define LED1_GPIO_PORT                     GPIOB
#define LED1_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()

#define LED2_PIN                           GPIO_PIN_5
#define LED2_GPIO_PORT                     GPIOA
#define LED2_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)    do { \
                                                switch( __INDEX__ ) \
                                                {\
                                                  case LED1: \
                                                    LED1_GPIO_CLK_ENABLE();   \
                                                    break;\
                                                  case LED2: \
                                                    LED2_GPIO_CLK_ENABLE();   \
                                                    break;\
                                                  default:\
                                                    break;\
                                                }\
                                              } while(0)
#define LEDx_GPIO_CLK_DISABLE(__INDEX__)   do { \
                                                switch( __INDEX__ ) \
                                                {\
                                                  case LED1: \
                                                    LED1_GPIO_CLK_DISABLE();   \
                                                    break;\
                                                  case LED2: \
                                                    LED2_GPIO_CLK_DISABLE();   \
                                                    break;\
                                                  default:\
                                                    break;\
                                                }\
                                              } while(0)
/**
  * @}
  */

/** @addtogroup B-L072Z-LRWAN1_LOW_LEVEL_BUTTON
  * @{
  */
#define BUTTONn                            1

/**
  * @brief Key push-button
  */
#define CS_FLASH_PIN                           GPIO_PIN_12
#define CS_FLASH_GPIO_PORT                     GPIOB
#define CS_FLASH_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()
#define CS_FLASH_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()

#define V_MIC_PIN                           GPIO_PIN_8
#define V_MIC_GPIO_PORT                     GPIOA
#define V_MIC_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define V_MIC_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()


#define V_BAT_MEAS_PIN                           GPIO_PIN_2
#define V_BAT_MEAS_GPIO_PORT                     GPIOA
#define V_BAT_MEAS_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define V_BAT_MEAS_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()

#define USER_BUTTON_PIN                         GPIO_PIN_2
#define USER_BUTTON_GPIO_PORT                   GPIOB
#define USER_BUTTON_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()
#define USER_BUTTON_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()
#define USER_BUTTON_EXTI_LINE                   GPIO_PIN_2
#define USER_BUTTON_EXTI_IRQn                   EXTI2_3_IRQn
/* Aliases */
#define KEY_BUTTON_PIN                          USER_BUTTON_PIN
#define KEY_BUTTON_GPIO_PORT                    USER_BUTTON_GPIO_PORT
#define KEY_BUTTON_GPIO_CLK_ENABLE()            USER_BUTTON_GPIO_CLK_ENABLE()
#define KEY_BUTTON_GPIO_CLK_DISABLE()           USER_BUTTON_GPIO_CLK_DISABLE()
#define KEY_BUTTON_EXTI_LINE                    USER_BUTTON_EXTI_LINE
#define KEY_BUTTON_EXTI_IRQn                    USER_BUTTON_EXTI_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)     do {KEY_BUTTON_GPIO_CLK_ENABLE();  } while(0)
#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)    do {KEY_BUTTON__GPIO_CLK_DISABLE();} while(0)

/**
  * @brief  This method returns the B-L072Z-LRWAN1 BSP Driver revision
  * @param  None
  * @retval version : 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BspGetVersion(void);

/**
  * @brief  Init pin CS Flash
  * @param  None
  * @retval None
  */
void BspCsFlashInit(void);

/**
  * @brief  On pin CS Flash
  * @param  None
  * @retval None
  */
void BspCsFlashOn(void);

/**
  * @brief  Off pin CS Flash
  * @param  None
  * @retval None
  */
void BspCsFlashOff(void);

/**
  * @brief  Init pin measurement voltage Battery
  * @param  None
  * @retval None
  */
void BspMeasBatInit(void);

/**
  * @brief  Enable pin  measurement voltage Battery
  * @param  None
  * @retval None
  */
void BspMeasBatOn(void);

/**
  * @brief  Disable pin  measurement voltage Battery
  * @param  None
  * @retval None
  */
void BspMeasBatOff(void);

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured.
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval None
  */
void BspLedInit(Led_TypeDef Led);

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval None
  */
void BspLedOn(Led_TypeDef Led);

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval None
  */
void BspLedOff(Led_TypeDef Led);

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval None
  */
void BspLedToggle(Led_TypeDef Led);

/**
  * @brief  Get on or off Led.
  * @param  Led: Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval true - on Led, false - off Led
  */
bool BspLedStatus(Led_TypeDef Led);

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
void BspButtonInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode);

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter should be: BUTTON_KEY
  * @retval The Button GPIO pin value.
  */
uint32_t BspButtonGetState(Button_TypeDef Button);

/**
  * @brief  Button interrupt handler
  * @param  None
  * @retval None
  */
void BspButtonHandler(void);

/**
  * @brief  Sleep mode for board
  * @param  None
  * @retval true - go to power off device, false - not power off device
  */
bool BspIsPowerOff(void);

#ifdef __cplusplus
}
#endif

#endif /* __SENSI_LORA_BSP_H */
