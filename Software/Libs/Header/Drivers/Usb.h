/**
 ******************************************************************************
 * @file    Usb.h
 * @date    30-June-2022
 * @brief   The USB driver.
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

#ifndef __USB_H
#define __USB_H

#include "main.h"
#include <stdbool.h>

/**
 * @brief:  Init Usb
 * @param:  None
 * @return: None
 */
void UsbInit(void);

/**
 * @brief:  DeInit Usb
 * @param:  None
 * @return: None
 */
void UsbDeinit(void);

/**
* @brief  Send a data to Usb
* @param  buf: pointer to data source array
* @param  len: array length
* @retval Value true - no error, false - error, no write data
*/
bool UsbSendData(uint8_t* buf, uint16_t len);

/**
* @brief  Send a data with crc-16bit to Usb
* @param  buf: pointer to data source array
* @param  len: array length
* @retval Value true - no error, false - error, no write data
*/
bool UsbSendDataWithCrc(uint8_t* buf, uint16_t len);

/**
* @brief  Receive a data from Usb
* @param  buf: pointer to destination data array
* @param  lenReceived: result array length
* @param  len: maximum number of bytes that data buffer can received
* @retval Value true - no error, false - error
*/
bool UsbReadData(uint8_t* buf, uint16_t len, uint16_t* lenReceived);

/**
* @brief  How byte in the RingBuffer
* @param  None
* @retval Number byte
*/
uint16_t UsbBytesAvailable(void);

/**
* @brief  Clear RingBuffer
* @param  None
* @retval None
*/
void UsbClearRxBuffer(void);

/**
* @brief  Calculate CRC 16bit
* @param  pBlock - pointer to buffer, len - size buffer
* @retval CRC 16bit
*/
uint16_t UsbcalculateCrc16(char *pBlock, uint16_t len);

/**
 * @brief:  Status Usb Com port
 * @param:  None
 * @return: true - Com port Open, false - Com port close
 */
bool UsbComPortStatus(void);

#endif //__USB_H
