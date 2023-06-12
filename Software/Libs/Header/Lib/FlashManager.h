/**
 ******************************************************************************
 * @file    FlashManager.h
 * @date    07-October-2022
 * @brief   The FlashManager description.
 *
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
#ifndef __FLASH_MANAGER_H_
#define __FLASH_MANAGER_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "AT25XE041B.h"

typedef struct Enable{
    bool mAcc;
    bool mGyro;
    bool mMag;
    bool mPress;
    bool mHum;
    bool mTemp;
    bool mLight;
} FlashEnableSens_t;

/**
 * @brief  Initializes the FlashManager
 * @param  None
 * @retval None
 */
void InitFlashM();

/**
 * @brief  Get Enable Status Sensors
 * @param  data - pointer to FlashEnableSens_t structure
 * @retval None
 */
void FlashMemoryGetEnableStatusSens(FlashEnableSens_t *data);

/**
 * @brief  Save settings Accelerometer sensor
 * @param  en - true - active sensor, false - disable sensor
 *         fullScale - 0 -  2g
 *                     1 -  16g
 *                     2 -  4g
 *                     3 -  8g
 *         odr -       0 -  ODR OFF
 *                     1 -  12Hz5
 *                     2 -  26Hz
 *                     3 -  52Hz
 *                     4 -  104Hz
 *                     5 -  208Hz
 *                     6 -  417Hz
 *                     7 -  833Hz
 *                     8 -  1667Hz
 *                     9 -  3333Hz
 *                     10 -  6667Hz
 * @retval None
 */
void FlashMemorySaveAccSett(bool en, uint8_t fullScale, uint8_t odr);

/**
* @brief  Get settings Accelerometer sensor
* @param  en - pointer to bool variable
*         fullScale - pointer to uint8_t variable
*         odr - pointer to uint8_t variable
* @retval true - successful, false - error
*/
bool FlashMemoryGetAccSett(bool *en, uint8_t *fullScale, uint8_t *odr);

/**
* @brief  Save settings Gyroscope sensor
* @param  en - true - active sensor, false - disable sensor
*         fullScale - 0 -  250dps
*                     1 -  500dps
*                     2 -  1000dps
*                     3 -  2000dps
*         odr -       0 -  ODR OFF
*                     1 -  12Hz5
*                     2 -  26Hz
*                     3 -  52Hz
*                     4 -  104Hz
*                     5 -  208Hz
*                     6 -  417Hz
*                     7 -  833Hz
*                     8 -  1667Hz
*                     9 -  3333Hz
*                     10 -  6667Hz
* @retval None
*/
void FlashMemorySaveGyroSett(bool en, uint8_t fullScale, uint8_t odr);

/**
* @brief  Get settings Gyroscope sensor
* @param  en - pointer to bool variable
*         fullScale - pointer to uint8_t variable
*         odr - pointer to uint8_t variable
* @retval true - successful, false - error
*/
bool FlashMemoryGetGyroSett(bool *en, uint8_t *fullScale, uint8_t *odr);

/**
 * @brief  Save settings Magnetic sensor
 * @param  en - true - active sensor, false - disable sensor
 *         odr - 0 -  10Hz
 *               1 -  20Hz
 *               2 -  50Hz
 *               3 -  100Hz
 * @retval None
 */
void FlashMemorySaveMagSett(bool en, uint8_t odr);

/**
* @brief  Read settings Magnetic sensor
* @param  en - pointer to bool variable
*         odr - pointer to uint8_t variable
* @retval true - successful, false - error
*/
bool FlashManagerGetMagSett(bool *en, uint8_t *odr);

/**
 * @brief  Save settings Pressure sensor
 * @param  en - true - active sensor, false - disable sensor
 *         odr - 0 -  Power Down
 *               1 -  1Hz
 *               2 -  10Hz
 *               3 -  25Hz
 *               4 -  50Hz
 *               5 -  75Hz
 *               6 -  100Hz
 *               7 -  200Hz
 * @retval None
 */
void FlashManagerSavePressureSett(bool en, uint8_t odr);

/**
 * @brief  Read status Pressure sensor
 * @param  en - pointer to bool variable
 *         odr - pointer to uint8_t variable
 * @retval true - successful, false - error
 */
bool FlashManagerGetPressureSett(bool *en, uint8_t *odr);

/**
 * @brief  Save settings Humidity sensor
 * @param  en - true - active sensor, false - disable sensor
 *         avg - 0 - 4 Avg
 *               1 - 8 Avg
 *               2 - 16 Avg
 *               3 - 32 Avg
 *               4 - 64 Avg
 *               5 - 128 Avg
 *               6 - 256 Avg
 *               7 - 512 Avg
 *         odr -  0 -  One short
 *                1 -  1Hz
 *                2 -  7Hz
 *                3 -  12,5Hz
 * @retval None
 */
void FlashManagerSaveHumSett(bool en, uint8_t avg, uint8_t odr);

/**
 * @brief  Read setttings Humidity sensor
 * @param  en - pointer to bool variable
 *         avg - pointer to uint8_t variable
 *         odr - pointer to uint8_t variable
 * @retval true - successful, false - error
 */
bool FlashManagerGetHumSett(bool *en, uint8_t *avg, uint8_t *odr);

/**
 * @brief  Save settins Temperature sensor
 * @param  en - true - active sensor, false - disable sensor
 *         avg -  0 - 2 Avg
 *                1 - 4 Avg
 *                2 - 8 Avg
 *                3 - 16 Avg
 *                4 - 32 Avg
 *                5 - 64 Avg
 *                6 - 128 Avg
 *                7 - 256 Avg
 * @retval None
 */
void FlashManagerSaveTempSett(bool en, uint8_t avg);

/**
 * @brief  Read settings Temperature sensor
 * @param  en - pointer to bool variable
 *         avg - pointer to uint8_t variable
 * @retval true - successful, false - error
 */
bool FlashManagerGetTempSett(bool *en, uint8_t *avg);

/**
 * @brief  Save settings Light sensor
 * @param  en - true - active sensor, false - disable sensor
 *         bit -  0 - 20 Bit
 *                1 - 19 Bit
 *                2 - 18 Bit
 *                3 - 17 Bit
 *                4 - 16 Bit
 *                5 - 13 Bit
 *         ms - 0 - 25 ms
 *              1 - 50 ms
 *              2 - 100 ms
 *              3 - 200 ms
 *              4 - 500 ms
 *              5 - 1000 ms
 *              6 - 2000 ms
 *              7 - 5000 ms
 * @retval None
 */
void FlashManagerSaveLightSett(bool en, uint8_t bit, uint8_t ms);

/**
 * @brief  Get settings Light sensor
 * @param  en - pointer to bool variable
 *         bit - pointer to uint8_t variable
 *         ms - pointer to uint8_t variable
 * @retval true - successful, false - error
 */
bool FlashManagerGetLightSett(bool *en, uint8_t *bit, uint8_t *ms);

/**
 * @brief  Save Period Transmit Lora
 * @param  s - time in seconds, min - 10s, max - 86400s
 * @retval None
 */
void FlashManagerSavePeriodTime(uint32_t sec);

/**
 * @brief  Get Period Transmit Lora
 * @param  none
 * @retval time Period Transmit in seconds
 */
uint32_t FlashManagerGetPeriodTime(void);

/**
 * @brief  Save Power Transmit Lora
 * @param  power - min -5db, max - 14db
 * @retval None
 */
void FlashManagerSavePowerTransmit(int8_t power);

/**
 * @brief  Get Power Transmit Lora
 * @param  None
 * @retval value power - min -5db, max - 14db
 */
int8_t FlashManagerGetPowerTransmit(void);

/**
 * @brief  Save Region Lora
 * @param  region - (1 << Region with LoRaMacRegion_t)
 * @retval None
 */
void FlashManagerSaveRegion(uint32_t region);

/**
 * @brief  Get Region Lora
 * @param  none
 * @retval Region Lora
 */
uint32_t FlashManagerGetRegion(void);

/**
 * @brief  Set default settings
 * @param  None
 * @retval None
 */
void FlashManagersetDefaultSett(void);

#endif /* __FLASH_MANAGER_H_ */
