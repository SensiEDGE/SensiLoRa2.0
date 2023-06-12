/**
 ******************************************************************************
 * @file    FlashManager.c
 * @date    07-October-2022
 * @brief   The Source file.
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "AT25XE041B.h"
#include "FlashManager.h"
#include "APDS9250_driver.h"
#include "lsm6dsox_reg.h"
#include "lps22hh_reg.h"
#include "lis2mdl_reg.h"
#include "hts221_reg.h"
#include "sys_app.h"
#include "LoraMac.h"

#define FLASH_ADDRESS         0
#define FLASH_FLAG_SETT       127

//#define FLASH_LOG_ENABLE

#ifdef FLASH_LOG_ENABLE
#define FLASH_LOG(...)              APP_LOG(TS_ON, VLEVEL_M, ##__VA_ARGS__)
#else
#define FLASH_LOG(...)
#endif

typedef struct Data {
    uint8_t mIsSett;
    bool mAccEn;
    uint8_t mAccFullSc;
    uint8_t mAccOdr;
    bool mGyroEn;
    uint8_t mGyroFullSc;
    uint8_t mGyroOdr;
    bool mMagEn;
    uint8_t mMagOdr;
    bool mPressEn;
    uint8_t mPressOdr;
    bool mHumEn;
    uint8_t mHumAvg;
    uint8_t mHumOdr;
    bool mTempEn;
    uint8_t mTempAvg;
    bool mLightEn;
    uint8_t mLightBit;
    uint8_t mLightMs;
    uint32_t mPeriod;
    int8_t mPower;
    uint32_t mRegion;
} Data_t;

static Data_t mData;

#ifdef FLASH_LOG_ENABLE
static void printData(void);
#endif

static void setDefaultRegion(void);
static void readData(void);
static void saveData(void);

/**
 * @brief  Initializes the FlashManager
 * @param  None
 * @retval None
 */
void InitFlashM()
{
    static bool initFlag = false;

    if (!initFlag) {
        initFlag = true;

        AT25XE041B_Init();
        readData();

        if (mData.mIsSett != FLASH_FLAG_SETT) {
            FlashManagersetDefaultSett();
            setDefaultRegion();
        }

#ifdef FLASH_LOG_ENABLE
        printData();
#endif
    }
}

/**
 * @brief  Get Enable Status Sensors
 * @param  data - pointer to FlashEnableSens_t structure
 * @retval None
 */
void FlashMemoryGetEnableStatusSens(FlashEnableSens_t *data)
{
    if (data != NULL) {
        data->mAcc = mData.mAccEn;
        data->mGyro = mData.mGyroEn;
        data->mMag = mData.mMagEn;
        data->mPress = mData.mPressEn;
        data->mHum = mData.mHumEn;
        data->mTemp = mData.mTempEn;
        data->mLight = mData.mLightEn;
    } else {
        FLASH_LOG("FLASH_ERROR, Status Sensors NULL");
    }
}

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
void FlashMemorySaveAccSett(bool en, uint8_t fullScale, uint8_t odr)
{
    if (mData.mAccEn != en || mData.mAccFullSc != fullScale || mData.mAccOdr != odr) {
        mData.mAccEn = en;
        mData.mAccFullSc = fullScale;
        mData.mAccOdr = odr;
        saveData();
    }
}

/**
* @brief  Get settings Accelerometer sensor
* @param  en - pointer to bool variable
*         fullScale - pointer to uint8_t variable
*         odr - pointer to uint8_t variable
* @retval true - successful, false - error
*/
bool FlashMemoryGetAccSett(bool *en, uint8_t *fullScale, uint8_t *odr)
{
    if (en == NULL || fullScale == NULL || odr == NULL) {
        FLASH_LOG("FLASH_ERROR, Accelerometer NULL");
        return false;
    }
    *en = mData.mAccEn;
    *fullScale = mData.mAccFullSc;
    *odr = mData.mAccOdr;

	return true;
}

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
void FlashMemorySaveGyroSett(bool en, uint8_t fullScale, uint8_t odr)
{
    if (mData.mGyroEn != en || mData.mGyroFullSc != fullScale || mData.mGyroOdr != odr) {
        mData.mGyroEn = en;
        mData.mGyroFullSc = fullScale;
        mData.mGyroOdr = odr;
        saveData();
    }
}

/**
* @brief  Get settings Gyroscope sensor
* @param  en - pointer to bool variable
*         fullScale - pointer to uint8_t variable
*         odr - pointer to uint8_t variable
* @retval true - successful, false - error
*/
bool FlashMemoryGetGyroSett(bool *en, uint8_t *fullScale, uint8_t *odr)
{
   if (en == NULL || fullScale == NULL || odr == NULL) {
       FLASH_LOG("FLASH_ERROR, Gyroscope NULL");
       return false;
   }
   *en = mData.mGyroEn;
   *fullScale = mData.mGyroFullSc;
   *odr = mData.mGyroOdr;

   return true;
}

/**
 * @brief  Save settings Magnetic sensor
 * @param  en - true - active sensor, false - disable sensor
 *         odr - 0 -  10Hz
 *               1 -  20Hz
 *               2 -  50Hz
 *               3 -  100Hz
 * @retval None
 */
void FlashMemorySaveMagSett(bool en, uint8_t odr)
{
    if (mData.mMagEn != en ||  mData.mMagOdr != odr) {
        mData.mMagEn = en;
        mData.mMagOdr = odr;
        saveData();
    }
}

/**
* @brief  Read settings Magnetic sensor
* @param  en - pointer to bool variable
*         odr - pointer to uint8_t variable
* @retval true - successful, false - error
*/
bool FlashManagerGetMagSett(bool *en, uint8_t *odr)
{
    if (en == NULL|| odr == NULL) {
        FLASH_LOG("FLASH_ERROR, Magnetic NULL");
        return false;
    }

    *en = mData.mMagEn;
    *odr = mData.mMagOdr;

    return true;
}

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
void FlashManagerSavePressureSett(bool en, uint8_t odr)
{
    if (mData.mPressEn != en ||  mData.mPressOdr != odr) {
        mData.mPressEn = en;
        mData.mPressOdr = odr;
        saveData();
    }
}

/**
 * @brief  Read status Pressure sensor
 * @param  en - pointer to bool variable
 *         odr - pointer to uint8_t variable
 * @retval true - successful, false - error
 */
bool FlashManagerGetPressureSett(bool *en, uint8_t *odr)
{
    if (en == NULL|| odr == NULL) {
        FLASH_LOG("FLASH_ERROR, Pressure NULL");
        return false;
    }

    *en = mData.mPressEn;
    *odr = mData.mPressOdr;

    return true;
}

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
void FlashManagerSaveHumSett(bool en, uint8_t avg, uint8_t odr)
{
    if (mData.mHumEn != en || mData.mHumAvg != avg || mData.mHumOdr != odr) {
        mData.mHumEn = en;
        mData.mHumAvg = avg;
        mData.mHumOdr = odr;
        saveData();
    }
}

/**
 * @brief  Read settings Humidity sensor
 * @param  en - pointer to bool variable
 *         avg - pointer to uint8_t variable
 *         odr - pointer to uint8_t variable
 * @retval true - successful, false - error
 */
bool FlashManagerGetHumSett(bool *en, uint8_t *avg, uint8_t *odr)
{
    if (en == NULL || avg == NULL || odr == NULL) {
        FLASH_LOG("FLASH_ERROR, Humidity NULL");
        return false;
    }

    *en = mData.mHumEn;
    *avg = mData.mHumAvg;
    *odr = mData.mHumOdr;

    return true;
}

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
void FlashManagerSaveTempSett(bool en, uint8_t avg)
{
    if (mData.mTempEn != en || mData.mTempAvg != avg) {
        mData.mTempEn = en;
        mData.mTempAvg = avg;
        saveData();
    }
}

/**
 * @brief  Read settings Temperature sensor
 * @param  en - pointer to bool variable
 *         avg - pointer to uint8_t variable
 * @retval true - successful, false - error
 */
bool FlashManagerGetTempSett(bool *en, uint8_t *avg)
{
    if (en == NULL || avg == NULL) {
        FLASH_LOG("FLASH_ERROR, Temperature NULL");
        return false;
    }

    *en = mData.mTempEn;
    *avg = mData.mTempAvg;

    return true;
}

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
void FlashManagerSaveLightSett(bool en, uint8_t bit, uint8_t ms)
{
    if (mData.mLightEn != en || mData.mLightBit != bit ||  mData.mLightMs != ms) {
        mData.mLightEn = en;
        mData.mLightBit = bit;
        mData.mLightMs = ms;
        saveData();
    }
}

/**
 * @brief  Get settings Light sensor
 * @param  en - pointer to bool variable
 *         bit - pointer to uint8_t variable
 *         ms - pointer to uint8_t variable
 * @retval true - successful, false - error
 */
bool FlashManagerGetLightSett(bool *en, uint8_t *bit, uint8_t *ms)
{
    if (en == NULL || bit == NULL || ms == NULL) {
        FLASH_LOG("FLASH_ERROR, Light NULL");
        return false;
    }

    *en = mData.mLightEn;
    *bit = mData.mLightBit;
    *ms = mData.mLightMs;

    return true;
}

/**
 * @brief  Save Period Transmit Lora
 * @param  s - time in seconds, min - 10s, max - 86400s
 * @retval None
 */
void FlashManagerSavePeriodTime(uint32_t sec)
{
    if (mData.mPeriod != sec) {
        mData.mPeriod = sec;
        saveData();
    }
}

/**
 * @brief  Get Period Transmit LoRa
 * @param  none
 * @retval time Period Transmit in seconds
 */
uint32_t FlashManagerGetPeriodTime(void)
{
    return mData.mPeriod;
}

/**
 * @brief  Save Power Transmit LoRa
 * @param  power - min -5db, max - 14db
 * @retval None
 */
void FlashManagerSavePowerTransmit(int8_t power)
{
    if (mData.mPower != power) {
        mData.mPower = power;
        saveData();
    }
}

/**
 * @brief  Save Power Transmit LoRa
 * @param  power - min -5db, max - 14db
 * @retval None
 */
int8_t FlashManagerGetPowerTransmit(void)
{
    return mData.mPower;
}

/**
 * @brief  Save Region LoRa
 * @param  region - (1 << Region with LoRaMacRegion_t)
 * @retval None
 */
void FlashManagerSaveRegion(uint32_t region)
{
    if (mData.mRegion != region) {
        mData.mRegion = region;
        saveData();
    }
}

/**
 * @brief  Get Region LoRa
 * @param  none
 * @retval Region LoRa
 */
uint32_t FlashManagerGetRegion(void)
{
    return mData.mRegion;
}

/**
 * @brief  Set default settings
 * @param  None
 * @retval None
 */
void FlashManagersetDefaultSett(void)
{
    mData.mIsSett = FLASH_FLAG_SETT;
    mData.mAccEn = true;
    mData.mAccFullSc = LSM6DSOX_2g;
    mData.mAccOdr = LSM6DSOX_XL_ODR_12Hz5;
    mData.mGyroEn = true;
    mData.mGyroFullSc = LSM6DSOX_250dps;
    mData.mGyroOdr = LSM6DSOX_GY_ODR_12Hz5;
    mData.mMagEn = true;
    mData.mMagOdr = LIS2MDL_ODR_10Hz;
    mData.mHumEn = true;
    mData.mPressEn = true;
    mData.mPressOdr = LPS22HH_1_Hz;
    mData.mHumAvg = HTS221_H_AVG_32;
    mData.mHumOdr = HTS221_ODR_12Hz5;
    mData.mTempEn = true;
    mData.mTempAvg = HTS221_T_AVG_16;
    mData.mLightEn = true;
    mData.mLightBit = APDS9250_18bit;
    mData.mLightMs = APDS9250_100ms;
    mData.mPeriod = 10;
    mData.mPower = 7;

    FLASH_LOG("FLASH, Set Default Settings\r\n");
}

/**
 * @brief  Set Default Region in Flash
 * @param  None
 * @retval None
 */
void setDefaultRegion(void)
{
    mData.mRegion = 1<<LORAMAC_REGION_EU868;
    saveData();
}

/**
 * @brief  Read data with EEPROM
 * @param  None
 * @retval None
 */
void readData(void)
{
    AT25XE041B_ReadByteArray(FLASH_ADDRESS, (uint8_t*)&mData, sizeof(mData));
}

/**
 * @brief  Save data to EEPROM
 * @param  None
 * @retval None
 */
void saveData(void)
{
    uint16_t len = 0;
    AT25XE041B_WriteByteArray(FLASH_ADDRESS, (uint8_t*)&mData, &len, sizeof(mData));
}

#ifdef FLASH_LOG_ENABLE
/**
 * @brief  Print Data Flash
 * @param  None
 * @retval None
 */
void printData(void)
{
    FLASH_LOG("...Print Data Flash...\r\n");
    FLASH_LOG("FLASH_ACC.EN=%d;FS=%d;ODR=%d\r\n", mData.mAccEn, mData.mAccFullSc, mData.mAccOdr);
    FLASH_LOG("FLASH_GYRO.EN=%d;FS=%d;ODR=%d\r\n", mData.mGyroEn, mData.mGyroFullSc, mData.mGyroOdr);
    FLASH_LOG("FLASH_MAG.EN=%d;ODR=%d\r\n", mData.mMagEn, mData.mMagOdr);
    FLASH_LOG("FLASH_PRESS.EN=%d;ODR=%d\r\n", mData.mPressEn, mData.mPressOdr);
    FLASH_LOG("FLASH_HUM.EN=%d;ODR=%d;AVG=%d\r\n", mData.mHumEn, mData.mHumOdr, mData.mHumAvg);
    FLASH_LOG("FLASH_TEMP.EN=%d;AVG=%d\r\n", mData.mTempEn, mData.mTempAvg);
    FLASH_LOG("FLASH_LIGHT.EN=%d;BIT=%d;MS=%d\r\n", mData.mLightEn, mData.mLightBit, mData.mLightMs);
    FLASH_LOG("FLASH_PER=%d\r\n", mData.mPeriod);
    FLASH_LOG("FLASH_POW=%d\r\n", mData.mPower);
    FLASH_LOG("FLASH_REGION=%d\r\n", mData.mRegion);
}
#endif
