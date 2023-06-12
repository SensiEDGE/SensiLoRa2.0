/**
 ******************************************************************************
 * @file    UsbService.h
 * @date    30-June-2022
 * @brief   The UsbService description.
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

#ifndef USB_SERVICE_H__
#define USB_SERVICE_H__

#include "main.h"

typedef enum  {
	USB_CM_INFO_DEV = 0,
	USB_CM_BATT,
	USB_CM_EUI,
	USB_CM_ENV,
	USB_CM_ACC,
	USB_CM_GYRO,
	USB_CM_MAG,
	USB_CM_ST_LED,
	USB_CM_LIGHT,
	USB_CM_ACC_SETT,
	USB_CM_GYRO_SETT,
	USB_CM_MAG_SETT,
	USB_CM_PRESS_SETT,
	USB_CM_HUM_SETT,
	USB_CM_TEMP_SETT,
	USB_CM_LIGHT_SETT,
	USB_CM_TIME_PER_SETT,
	USB_CM_POWER_TRANSMIT,
	USB_CM_STATUS_BUTTON
} usbInfoCommand_t;

typedef enum  {
    USB_SETT_ACC = 0,
    USB_SETT_GYRO,
    USB_SETT_MAG,
    USB_SETT_PRESS,
    USB_SETT_HUM,
    USB_SETT_TEMP,
    USB_SETT_LIGHT,
    USB_SETT_PER,
    USB_SETT_POW,
    USB_SETT_REGION
} usbSettCommand_t;

/**
  * @brief  Init Usb Service
  * @param  None
  * @retval None
  */
void UsbServiceInit(void);

/**
  * @brief  Usb Task. When the com port is open, then this function should be called in a loop to process usb commands
  * @param  None
  * @retval None
  */
void UsbTask(void);

#endif //USB_SERVICE_H__
