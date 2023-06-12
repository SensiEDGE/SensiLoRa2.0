/**
 ******************************************************************************
 * @file    UsbService.c
 * @date    30-June-2022
 * @brief   The  source file.
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
#include "UsbService.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "Usb.h"
#include "SensiLora.h"
#include "secure-element.h"
#include "LoRaMac.h"
#include "LmHandlerTypes.h"
#include "sys_sensors.h"
#include "SensiLoraBSP.h"
#include "FlashManager.h"
#include "lora_info.h"
#include "usbd_cdc_if.h"
#include "lora_app.h"
#include "sys_app.h"
#include "adc_if.h"

//#define USB_LOG_ACTIVE

#ifdef USB_LOG_ACTIVE
#define LOG_USB(...)              APP_LOG(TS_ON, VLEVEL_M, ##__VA_ARGS__)
#else
#define LOG_USB(...)
#endif

/* Hex 8 split buffer*/
#define HEX8(X)   X[0], X[1], X[2], X[3], X[4], X[5], X[6], X[7]
#define HEX16(X)  HEX8(X), X[8], X[9], X[10], X[11], X[12], X[13], X[14], X[15]

static char sBuff[128];

static const char kRegionAs[] = "AS923";
static const char kRegionAu[] = "AU915";
static const char kRegionEu[] = "EU868";
static const char kRegionKr[] = "KR920";
static const char kRegionIn[] = "IN865";
static const char kRegionUs[] = "US915";

static CommissioningParams_t sNetwoekData =
{
  .DevEui = { 0 },
  .JoinEui = { 0 },
  .NetworkId = 0,
  .DevAddr = 0,
};

typedef struct {
    float x;
    float y;
    float z;
} Axes_t;

static void UsbServiceDoCommand(char *command, uint16_t size);
static void sendBattery(void);
static void sendInfo(void);
static void sendEUI(void);
static void sendEnv(void);
static void sendAcc(void);
static void sendOk(void);
static void sendError(void);
static void sendGyro(void);
static void sendMag(void);
static void sendStatusLed(void);
static void sendLight(void);
static float getBattPercent(float volt);
static char *getRegion(void);
static void sendAccSett(void);
static void sendGyroSett(void);
static void sendMagSett(void);
static void sendPressSett(void);
static void sendHumSett(void);
static void sendTempSett(void);
static void sendLightSett(void);
static void sendTimePeriodSett(void);
static void sendTransmitPower(void);
static void sendStatusButton(void);
static void setRegion(const char *region,uint16_t size);
static void UsbServiceParser(void);
static void sendDataMatlab(uint8_t sens);
static void getDataAcc(Axes_t *axes);
static void getDataGyro(Axes_t *axes);
static void getDataMag(Axes_t *axes);

/**
  * @brief  Init Usb Service
  * @param  None
  * @retval None
  */
void UsbServiceInit(void)
{
	UsbInit();
}

/**
  * @brief  Usb Task. When the com port is open, then this function should be called in a loop to process usb commands
  * @param  None
  * @retval None
  */
void UsbTask(void)
{
    UsbServiceParser();
}

/**
 * @brief:   Parser Usb Command
 * @param:   None
 * @return:  None
 */
void UsbServiceParser(void)
{
	uint16_t len = UsbBytesAvailable();
	uint8_t byte = 0;
	uint16_t receiv = 0;
	uint8_t size = 0;

	if (len) {
		len++;
		memset(sBuff, 0, sizeof(sBuff));

		for (uint16_t i = 0; i < len; i++) {
			UsbReadData(&byte, sizeof(byte), &receiv);

			if(byte == '\n' || byte == '\r' || receiv == 0) {
			    if (size > 1) {
			        UsbServiceDoCommand(sBuff, size);
			    }
				return;
			} else {
			    sBuff[size] = byte;
				size++;
			}
		}
	}
}

/**
 * @brief:   Parser Usb Command
 * @param:   command - pointer on buffer with command
 * @param:   size - size buffer with command
 * @return:  None
 */
void UsbServiceDoCommand(char *command, uint16_t size)
{
	char *pIndex = NULL;
	int mode = 0;
	int crc = 0;

	if (command == NULL || size == 0) {
	    return;
	}

	pIndex =  memchr(command, ',', size);

	if (pIndex == NULL) {
	    if (strstr(command, "MATLAB_SENS") != NULL) {
            sscanf(command, "MATLAB_SENS=%d", &mode);
	    }

	    sendDataMatlab(mode);
        return;
	}

	int sizeCommand = strlen(command) - strlen (pIndex);
	sscanf(pIndex, ",%04X", &crc);

	uint32_t check = UsbcalculateCrc16(command, sizeCommand);

	if (check != crc) {
	    sendError();
	    return;
	}

	if (strstr(command, "INFO") != NULL) {
		sscanf(command, "INFO=%d", &mode);

		switch(mode) {
			case USB_CM_INFO_DEV:
				sendInfo();
				break;
			case USB_CM_BATT:
				sendBattery();
				break;
			case USB_CM_EUI:
				sendEUI();
				break;
			case USB_CM_ENV:
				sendEnv();
				break;
			case USB_CM_ACC:
				sendAcc();
				break;
			case USB_CM_GYRO:
				sendGyro();
				break;
			case USB_CM_MAG:
				sendMag();
				break;
			case USB_CM_ST_LED:
				sendStatusLed();
				break;
			case USB_CM_LIGHT:
				sendLight();
				break;
			case USB_CM_ACC_SETT:
			    sendAccSett();
			    break;
			case USB_CM_GYRO_SETT:
			    sendGyroSett();
			    break;
			case USB_CM_MAG_SETT:
			    sendMagSett();
			    break;
			case USB_CM_PRESS_SETT:
			    sendPressSett();
			    break;
			case USB_CM_HUM_SETT:
			    sendHumSett();
			    break;
           case USB_CM_TEMP_SETT:
                sendTempSett();
                break;
            case USB_CM_LIGHT_SETT:
                sendLightSett();
                break;
            case USB_CM_TIME_PER_SETT:
                sendTimePeriodSett();
                break;
            case USB_CM_POWER_TRANSMIT:
                sendTransmitPower();
                break;
            case USB_CM_STATUS_BUTTON:
                sendStatusButton();
                break;
			default:
				sendError();
				return;
		}
	} else if(strstr(command, "SETT") != NULL) {
		int en = 0;
		int param1 = 0;
		int param2 = 0;
		char reg[5] = {0};

		sscanf(command, "SETT:ID=%d", &mode);

		char *settings = strchr(command, '.');

		if (settings == NULL) {
            sendError();
            return;
		}

		switch (mode) {
			case USB_SETT_ACC:
				sscanf(settings, ".EN=%d;FS=%d;ODR=%d", &en, &param1, &param2);
				FlashMemorySaveAccSett(en, param1, param2);
				SensiLoraSensorUpdate(SENSI_ACCEL);
				LOG_USB("USB_ACC.EN=%d;FS=%d;ODR=%d\n", en, param1, param2);
				break;
			case USB_SETT_GYRO:
                sscanf(settings, ".EN=%d;FS=%d;ODR=%d", &en, &param1, &param2);
                FlashMemorySaveGyroSett(en, param1, param2);
                SensiLoraSensorUpdate(SENSI_GYR);
                LOG_USB("USB_GYRO.EN=%d;FS=%d;ODR=%d\n", en, param1, param2);
				break;
            case USB_SETT_MAG:
                sscanf(settings, ".EN=%d;ODR=%d", &en, &param1);
                FlashMemorySaveMagSett(en, param1);
                SensiLoraSensorUpdate(SENSI_MAG);
                LOG_USB("USB_MAG.EN=%d;ODR=%d\n", en, param1);
                break;
            case USB_SETT_PRESS:
                sscanf(settings, ".EN=%d;ODR=%d", &en, &param1);
                FlashManagerSavePressureSett(en, param1);
                SensiLoraSensorUpdate(SENSI_PRESSURE);
                LOG_USB("USB_PRESS.EN=%d;ODR=%d\n", en, param1);
                break;
            case USB_SETT_HUM:
                sscanf(settings, ".EN=%d;ODR=%d;AVG=%d", &en, &param1, &param2);
                FlashManagerSaveHumSett(en, param2, param1);
                SensiLoraSensorUpdate(SENSI_HUMIDITY);
                LOG_USB("USB_HUM.EN=%d;ODR=%d;AVG=%d\n", en, param1, param2);
                break;
            case USB_SETT_TEMP:
                sscanf(settings, ".EN=%d;AVG=%d", &en, &param1);
                FlashManagerSaveTempSett(en, param1);
                SensiLoraSensorUpdate(SENSI_TEMPERATURE);
                LOG_USB("USB_TEMP.EN=%d;AVG=%d\n", en, param1);
                break;
            case USB_SETT_LIGHT:
                sscanf(settings, ".EN=%d;BIT=%d;MS=%d", &en, &param1, &param2);
                FlashManagerSaveLightSett(en, param1, param2);
                SensiLoraSensorUpdate(SENSI_LIGHT);
                LOG_USB("USB_LIGHT.EN=%d;BIT=%d;MS=%d\n", en, param1, param2);
                break;
            case USB_SETT_PER:
                sscanf(settings, ".PER=%d", &param1);
                FlashManagerSavePeriodTime(param1);
                LOG_USB("USB_PER=%d\n", param1);
                break;
            case USB_SETT_POW:
                sscanf(settings, ".POW=%d", &param1);
                FlashManagerSavePowerTransmit(param1);
                LOG_USB("USB_POW=%d\n", param1);
                break;
            case USB_SETT_REGION:
                sscanf(settings, ".REG=%s", reg);
                setRegion(reg, sizeof(reg));
                LOG_USB("USB_REG=%s\n", reg);
                break;
            default:
            	sendError();
            	return;
		}
		sendOk();
	} else if(strstr(command, "RESET") != NULL) {
        LOG_USB("USB_RESET\n");
        sendOk();
        NVIC_SystemReset();
	} else if(strstr(command, "DEFAULT") != NULL) {
        LOG_USB("USB_DEFAULT\n");
        FlashManagersetDefaultSett();
        Sensors_Restart();
        Sensors_InitAll();
        sendOk();
	} else {
	    sendError();
        return;
	}
}

/**
 * @brief:   Send Command "OK"
 * @param:   None
 * @return:  None
 */
void sendOk(void)
{
	const char kOk[] = {"OK"};
	UsbSendDataWithCrc((uint8_t*)kOk, strlen(kOk));
}

/**
 * @brief:   Send Command "ERROR"
 * @param:   None
 * @return:  None
 */
void sendError(void)
{
	const char kError[] = {"ERROR"};
	UsbSendDataWithCrc((uint8_t*)kError, strlen(kError));
}

/**
 * @brief:   Send pressure, temperature and humidity values
 * @param:   None
 * @return:  None
 */
void sendEnv(void)
{
    float humValue = 0;
    float tempValue = 0;
    float pressValue = 0;

    SensiLoraSensorGetValue(SENSI_HTS221, SENSI_TEMPERATURE, &tempValue);
    SensiLoraSensorGetValue(SENSI_HTS221, SENSI_HUMIDITY, &humValue);
    SensiLoraSensorGetValue(SENSI_LPS22HH, SENSI_PRESSURE, &pressValue);

	memset(sBuff, 0, sizeof(sBuff));

	sprintf(sBuff, "PRESS=%.2f;HUM=%.2f;TEMP=%.2f", pressValue, humValue, tempValue);

	UsbSendDataWithCrc((uint8_t*)sBuff, strlen(sBuff));
}

/**
 * @brief:   Send accelerometer values
 * @param:   None
 * @return:  None
 */
void sendAcc(void)
{
	memset(sBuff, 0, sizeof(sBuff));
	Axes_t axes = {0};

	getDataAcc(&axes);
	sprintf(sBuff, "ACC=%.2f;%.2f;%.2f", axes.x, axes.y, axes.z);

	UsbSendDataWithCrc((uint8_t*)sBuff, strlen(sBuff));
}

/**
 * @brief:   Send gyroscope values
 * @param:   None
 * @return:  None
 */
void sendGyro(void)
{
	memset(sBuff, 0, sizeof(sBuff));
	Axes_t axes = {0};

	getDataGyro(&axes);
	sprintf(sBuff, "GYRO=%.4f;%.4f;%.4f", axes.x, axes.y, axes.z);

	UsbSendDataWithCrc((uint8_t*)sBuff, strlen(sBuff));
}

/**
 * @brief:   Send Magnetic values
 * @param:   None
 * @return:  None
 */
void sendMag(void)
{
    memset(sBuff, 0, sizeof(sBuff));
    Axes_t axes = {0};

	getDataMag(&axes);
	sprintf(sBuff, "MAG=%.2f;%.2f;%.2f", axes.x, axes.y, axes.z);

	UsbSendDataWithCrc((uint8_t*)sBuff, strlen(sBuff));
}

/**
 * @brief:   Send status Green Led
 * @param:   None
 * @return:  None
 */
void sendStatusLed(void)
{
    memset(sBuff, 0, sizeof(sBuff));
	bool st = BspLedStatus(LED_GREEN);
	sprintf(sBuff, "LED=%d", (uint8_t)st);

	UsbSendDataWithCrc((uint8_t*)sBuff, strlen(sBuff));
}

/**
 * @brief:   Send Lux, Light sensor
 * @param:   None
 * @return:  None
 */
void sendLight(void)
{
    uint32_t lightValue = 0;
    SensiLoraSensorGetValue(SENSI_APDS9250, SENSI_LIGHT, (float*)&lightValue);

	memset(sBuff, 0, sizeof(sBuff));
	sprintf(sBuff, "LIGHT=%ld", lightValue);

	UsbSendDataWithCrc((uint8_t*)sBuff, strlen(sBuff));
}

/**
 * @brief:   Send about device
 * @param:   None
 * @return:  None
 */
void sendInfo(void)
{
    memset(sBuff, 0, sizeof(sBuff));

	char *region = getRegion();
	sprintf(sBuff, "DEV=%s;VERS=%s;REG=%s", NAME_DEVICE, FIRM_VERSION, region);

	UsbSendDataWithCrc((uint8_t*)sBuff, strlen(sBuff));
}

/**
 * @brief:   Send battery information
 * @param:   None
 * @return:  None
 */
void sendBattery(void)
{
    memset(sBuff, 0, sizeof(sBuff));
	float level = SystemApp_GetBatteryLevel() / 100.0;
	float per = getBattPercent(level);

	sprintf(sBuff, "BAT=%.2f;VOLT=%.2f", per, level);

	UsbSendDataWithCrc((uint8_t*)sBuff, strlen(sBuff));
}

/**
 * @brief:   Send Lora information data
 * @param:   None
 * @return:  None
 */
void sendEUI(void)
{
    memset(sBuff, 0, sizeof(sBuff));
	Key_t *keyItem;
	SecureElementStatus_t retval = SECURE_ELEMENT_ERROR;
	MibRequestConfirm_t mibReq;
	uint32_t index = 0;

    retval = SecureElementGetKeyByID(APP_KEY, &keyItem);
    if (retval == SECURE_ELEMENT_SUCCESS)
    {
        index = sprintf(sBuff, "AppKey=%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X;", HEX16(keyItem->KeyValue));
    }

    mibReq.Type = MIB_DEV_EUI;
    LoRaMacMibGetRequestConfirm(&mibReq);
    memcpy1(sNetwoekData.DevEui, mibReq.Param.DevEui, 8);
    mibReq.Type = MIB_JOIN_EUI;
    LoRaMacMibGetRequestConfirm(&mibReq);
    memcpy1(sNetwoekData.JoinEui, mibReq.Param.JoinEui, 8);

    index += sprintf(&sBuff[index], "DevEui=%02X%02X%02X%02X%02X%02X%02X%02X;",
    		HEX8(sNetwoekData.DevEui));
    sprintf(&sBuff[index], "JoinEui=%02X%02X%02X%02X%02X%02X%02X%02X",
         HEX8(sNetwoekData.JoinEui));

    UsbSendDataWithCrc((uint8_t*)sBuff, strlen(sBuff));
}

/**
 * @brief:   Send settings Accelerometer sensor
 * @param:   None
 * @return:  None
 */
void sendAccSett(void)
{
    memset(sBuff, 0, sizeof(sBuff));
	bool en = false;
	uint8_t fs = 0;
	uint8_t odr = 0;

	FlashMemoryGetAccSett(&en, &fs, &odr);

	sprintf(sBuff, "EN=%d;FS=%d;ODR=%d", en, fs, odr);

	UsbSendDataWithCrc((uint8_t*)sBuff, strlen(sBuff));
}

/**
 * @brief:   Send settings Gyroscope sensor
 * @param:   None
 * @return:  None
 */
void sendGyroSett(void)
{
    memset(sBuff, 0, sizeof(sBuff));
    bool en = false;
    uint8_t fs = 0;
    uint8_t odr = 0;

    FlashMemoryGetGyroSett(&en, &fs, &odr);

    sprintf(sBuff, "EN=%d;FS=%d;ODR=%d", en, fs, odr);

    UsbSendDataWithCrc((uint8_t*)sBuff, strlen(sBuff));
}

/**
 * @brief:   Send settings Magnetic sensor
 * @param:   None
 * @return:  None
 */
void sendMagSett(void)
{
    memset(sBuff, 0, sizeof(sBuff));
    bool en = false;
    uint8_t odr = 0;

    FlashManagerGetMagSett(&en, &odr);

    sprintf(sBuff, "EN=%d;ODR=%d", en, odr);

    UsbSendDataWithCrc((uint8_t*)sBuff, strlen(sBuff));
}

/**
 * @brief:   Send settings Pressure sensor
 * @param:   None
 * @return:  None
 */
void sendPressSett(void)
{
    memset(sBuff, 0, sizeof(sBuff));
    bool en = false;
    uint8_t odr = 0;

    FlashManagerGetPressureSett(&en, &odr);

    sprintf(sBuff, "EN=%d;ODR=%d", en, odr);

    UsbSendDataWithCrc((uint8_t*)sBuff, strlen(sBuff));
}

/**
 * @brief:   Send settings Humidity sensor
 * @param:   None
 * @return:  None
 */
void sendHumSett(void)
{
    memset(sBuff, 0, sizeof(sBuff));
    bool en = false;
    uint8_t avg = 0;
    uint8_t odr = 0;

    FlashManagerGetHumSett(&en, &avg, &odr);

    sprintf(sBuff, "EN=%d;AVG=%d;ODR=%d", en, avg, odr);

    UsbSendDataWithCrc((uint8_t*)sBuff, strlen(sBuff));
}

/**
 * @brief:   Send settings Temperature sensor
 * @param:   None
 * @return:  None
 */
void sendTempSett(void)
{
    memset(sBuff, 0, sizeof(sBuff));
    bool en = false;
    uint8_t avg = 0;

    FlashManagerGetTempSett(&en, &avg);

    sprintf(sBuff, "EN=%d;AVG=%d", en, avg);

    UsbSendDataWithCrc((uint8_t*)sBuff, strlen(sBuff));
}

/**
 * @brief:   Send settings Light sensor
 * @param:   None
 * @return:  None
 */
void sendLightSett(void)
{
    memset(sBuff, 0, sizeof(sBuff));
    bool en = false;
    uint8_t bit = 0;
    uint8_t ms = 0;

    FlashManagerGetLightSett(&en, &bit, &ms);

    sprintf(sBuff, "EN=%d;BIT=%d;MS=%d", en, bit, ms);

    UsbSendDataWithCrc((uint8_t*)sBuff, strlen(sBuff));
}

/**
 * @brief:   Send period Transmit Lora data
 * @param:   None
 * @return:  None
 */
void sendTimePeriodSett(void)
{
    memset(sBuff, 0, sizeof(sBuff));
    uint32_t time = FlashManagerGetPeriodTime();

    sprintf(sBuff, "PER=%ld", time);

    UsbSendDataWithCrc((uint8_t*)sBuff, strlen(sBuff));
}

/**
 * @brief:   Send Power Transmit Lora
 * @param:   None
 * @return:  None
 */
void sendTransmitPower(void)
{
    memset(sBuff, 0, sizeof(sBuff));
    int power = FlashManagerGetPowerTransmit();

    sprintf(sBuff, "POW=%d", power);

    UsbSendDataWithCrc((uint8_t*)sBuff, strlen(sBuff));
}

/**
 * @brief:   Send Status User Button
 * @param:   None
 * @return:  None
 */
void sendStatusButton(void)
{
    memset(sBuff, 0, sizeof(sBuff));
    int status = BspButtonGetState(BUTTON_USER);

    sprintf(sBuff, "BUTTON=%d", status);

    UsbSendDataWithCrc((uint8_t*)sBuff, strlen(sBuff));
}

/**
 * @brief:   Get Battery Percent
 * @param:   volt - voltage battery
 * @return:  percent battery 0 - 100%
 */
float getBattPercent(float volt)
{
    float result = 0;

    if(volt >= 4.19f){
        result = 100.0f;
    } else if(volt >= 4.1f){
        result = ((((volt - 4.1f) * 2.2f) / 0.1f) + 97.8f);
    } else if(volt >= 4.0f){
        result = ((((volt - 4.0f) * 9.8f) / 0.1f) + 88.0f);
    } else if(volt >= 3.9f){
        result = ((((volt - 3.9f) * 11.2f) / 0.1f) + 76.8f);
    } else if(volt >= 3.8f){
        result = ((((volt - 3.8f) * 14.6f) / 0.1f) + 62.2f);
    } else if(volt >= 3.7f){
        result = ((((volt - 3.7f) * 19.0f) / 0.1f) + 43.2f);
    } else if(volt >= 3.6f){
        result = ((((volt - 3.6f) * 22.9f) / 0.1f) + 20.3f);
    } else if(volt >= 3.5f){
        result = ((((volt - 3.5f) * 20.3f) / 0.1f) + 0.0f);
    } else {
        result = 0.0f;
    }

    return result;
}

/**
 * @brief:  Get Region Lora
 * @param:  None
 * @return: pointer to buffer with Region
 */
char *getRegion(void)
{
    char *point = NULL;
    uint32_t region = FlashManagerGetRegion();

    switch(region) {
        case (1 << LORAMAC_REGION_AS923):
            point = (char*)kRegionAs;
            break;
        case (1 << LORAMAC_REGION_AU915):
            point = (char*)kRegionAu;
            break;
        case (1 << LORAMAC_REGION_EU868):
            point = (char*)kRegionEu;
            break;
        case (1 << LORAMAC_REGION_KR920):
            point = (char*)kRegionKr;
            break;
        case (1 << LORAMAC_REGION_IN865):
            point = (char*)kRegionIn;
            break;
        case (1 << LORAMAC_REGION_US915):
            point = (char*)kRegionUs;
            break;
        default:
            point = (char*)kRegionEu;
            break;
    }
    return point;
}

/**
 * @brief:  Set Region Lora
 * @param:  region - pointer to buffer with Region
 * @param:  size - size buffer with Region
 * @return: None
 */
void setRegion(const char *region, uint16_t size)
{
    if (region == NULL || size == 0) {
        return;
    }

    if (!strncmp(kRegionAs, region, size)) {
        FlashManagerSaveRegion(1 << LORAMAC_REGION_AS923);
    }

    if (!strncmp(kRegionAu, region, size)) {
        FlashManagerSaveRegion(1 << LORAMAC_REGION_AU915);
    }

    if (!strncmp(kRegionEu, region, size)) {
        FlashManagerSaveRegion(1 << LORAMAC_REGION_EU868);
    }

    if (!strncmp(kRegionKr, region, size)) {
        FlashManagerSaveRegion(1 << LORAMAC_REGION_KR920);
    }

    if (!strncmp(kRegionIn, region, size)) {
        FlashManagerSaveRegion(1 << LORAMAC_REGION_IN865);
    }

    if (!strncmp(kRegionUs, region, size)) {
        FlashManagerSaveRegion(1 << LORAMAC_REGION_US915);
    }
}

/**
 * @brief:  Send Data for MATLAB via USB
 * @param:  sens - status sensor. Status Bit 1 - enable, 0 - disable
 *          Sensors: Bit 0 - accelerometer
 *                   Bit 1 - gyroscope
 *                   Bit 2 - magnetometer
 *                   Bit 3 - humidity
 *                   Bit 4 - pressure
 *                   Bit 5 - temperature
 *                   Bit 6 - light
 * @return: None
 */
void sendDataMatlab(uint8_t sens)
{
    memset(sBuff, 0, sizeof(sBuff));
    char *index = sBuff;

    if (sens & (1 << 0)) {
        Axes_t axes = {0};
        getDataAcc(&axes);
        index +=sprintf(index, "%.2f;%.2f;%.2f;", axes.x, axes.y, axes.z);
        LOG_USB("USB_ACC\n");
    }

    if (sens & (1 << 1)) {
        Axes_t axes = {0};
        getDataGyro(&axes);
        index +=sprintf(index, "%.4f;%.4f;%.4f;", axes.x, axes.y, axes.z);
        LOG_USB("USB_GYRO\n");
    }

    if (sens & (1 << 2)) {
        Axes_t axes = {0};
        getDataMag(&axes);
        index +=sprintf(index, "%.2f;%.2f;%.2f;", axes.x, axes.y, axes.z);
        LOG_USB("USB_MAG\n");
    }

    if (sens & (1 << 3)) {
        float humValue = 0;
        SensiLoraSensorGetValue(SENSI_HTS221, SENSI_HUMIDITY, &humValue);
        index +=sprintf(index, "%.2f;", humValue);
        LOG_USB("USB_HUM\n");
    }

    if (sens & (1 << 4)) {
        float pressValue = 0;
        SensiLoraSensorGetValue(SENSI_LPS22HH, SENSI_PRESSURE, &pressValue);
        index +=sprintf(index, "%.2f;", pressValue);
        LOG_USB("USB_PRESS\n");
    }

    if (sens & (1 << 5)) {
        float tempValue = 0;
        SensiLoraSensorGetValue(SENSI_HTS221, SENSI_TEMPERATURE, &tempValue);
        index +=sprintf(index, "%.2f;", tempValue);
        LOG_USB("USB_TEMP\n");
    }

    if (sens & (1 << 6)) {
        int lightValue = 0;
        SensiLoraSensorGetValue(SENSI_APDS9250, SENSI_LIGHT, (float*)&lightValue);
        index +=sprintf(index, "%d;", lightValue);
        LOG_USB("USB_LIGHT\n");
    }

    uint8_t len = strlen(sBuff);
    len -= 1;
    sBuff[len] = 0;

    sprintf(&sBuff[len], "\r\n");
    UsbSendData((uint8_t*)sBuff, strlen(sBuff));
}

/**
 * @brief:  Get Data with Accelerometer
 * @param:  axes - pointer to structure Axes_t
 * @return: None
 */
void getDataAcc(Axes_t *axes)
{
    if (axes == NULL)
    {
        return;
    }

    const float sMgToMs = 9.80665;
    SensorAxes_t accAxes = {0};
    SensiLoraSensorGetValue(SENSI_LSM6DSOX, SENSI_ACCEL, (float*)&accAxes);

    // Convert to m/s2
    axes->x = (float)accAxes.AXIS_X / 1000.0 * sMgToMs;
    axes->y =(float)accAxes.AXIS_Y / 1000.0 * sMgToMs;
    axes->z = (float)accAxes.AXIS_Z / 1000.0 * sMgToMs;
}

/**
 * @brief:  Get Data with Gyroscope
 * @param:  axes - pointer to structure Axes_t
 * @return: None
 */
void getDataGyro(Axes_t *axes)
{
    if (axes == NULL)
    {
        return;
    }

    const float sDpsToRads = 0.017453;
    SensorAxes_t gyroAxes = {0};
    SensiLoraSensorGetValue(SENSI_LSM6DSOX, SENSI_GYR, (float*)&gyroAxes);

    //Convert to rad/s
    axes->x = (float)gyroAxes.AXIS_X /1000.0 * sDpsToRads;
    axes->y =(float)gyroAxes.AXIS_Y /1000.0 * sDpsToRads;
    axes->z = (float)gyroAxes.AXIS_Z /1000.0 * sDpsToRads;
}

/**
 * @brief:  Get Data with Magnetometer
 * @param:  axes - pointer to structure Axes_t
 * @return: None
 */
void getDataMag(Axes_t *axes)
{
    if (axes == NULL) {
        return;
    }

    SensorAxes_t magAxes = {0};
    SensiLoraSensorGetValue(SENSI_LIS2MDL, SENSI_MAG, (float*)&magAxes);

    //Convert to micro-Teslas
    axes->x = (float)magAxes.AXIS_X * 0.1;
    axes->y =(float)magAxes.AXIS_Y * 0.1;
    axes->z = (float)magAxes.AXIS_Z * 0.1;
}
