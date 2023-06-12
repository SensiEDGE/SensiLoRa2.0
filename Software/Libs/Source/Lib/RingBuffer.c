/**
 ******************************************************************************
 * @file    RingBuffer.c
 * @date    30-June-2022
 * @brief   The RingBuffer  source file.
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

#include "RingBuffer.h"
#include <stdbool.h>
#include <stddef.h>

static uint8_t sBuffer[512];
static uint16_t sBufferSize = sizeof(sBuffer)/sizeof(sBuffer[0]);
static uint16_t sPutIndex;
static uint16_t sGetIndex;
static uint16_t sDataLength;

static bool RignBuffGetByte(uint8_t *byte);

/**
* @brief   Put new element to ring buffer. 
* @param   element:  new element.
* @retval  none.
*/
void RignBuffPut(uint8_t element)
{
    //if buffer is not empty
    if(sDataLength != 0){
        //if buffer is full
		if(sPutIndex == sGetIndex){
			sGetIndex++;
            sDataLength--;
			if(sGetIndex == sBufferSize)
				sGetIndex = 0;
		}
	}
    //put element to buffer
    sBuffer[sPutIndex++] = element;
    if(sPutIndex == sBufferSize)
        sPutIndex = 0;
    sDataLength++;
}

/**
* @brief   Get a number of elements from ring buffer. 
* @param   data:     Specify the pointer to array, where to save data.
* @param   length:   Specify the link to variable, vhere to set length of
*                    read data from ring buffer.
* @param   maxSize:  Specify  number of data to read from buffer.
* @retval  none.
*/
void RignBuffGet(uint8_t* data, uint16_t *length, uint16_t maxSize)
{
    bool l_flag = false;

    *length = 0;
    if (sDataLength == 0 || data == NULL || length == NULL) {
        return;
    }
    
    //Protects against sudden change of this->sDataLength
    uint32_t dataLength = sDataLength;
    
    if(maxSize > dataLength)
        maxSize = dataLength;
    
    for(uint32_t i = 0; i < maxSize; i++) {

        l_flag  = RignBuffGetByte(&data[i]);
        *length = *length +1;

        if (l_flag == false) {

            break;
        }
    }
}

/**
* @brief   Get a length of received elements in ring buffer. 
* @param   none.
* @retval  uint16_t: length of received elements.
*/
uint16_t RignBuffGetDataLenght()
{
    return sDataLength;
}

/**
* @brief   Clear the ring buffer. 
* @param   none.
* @retval  none.
*/
void RignBuffClear()
{
    sGetIndex = sPutIndex = sDataLength = 0;
}

/**
* @brief   Get first element from ring buffer.
*          You must check data length before calling this function.
*          If data length is 0 - You shouldn't call this function.
*          In other way there may be problems.
* @param   byte
* @retval  none.
*/
bool RignBuffGetByte(uint8_t *byte)
{
    if (sDataLength == 0 || byte == NULL) {

        return false;
    }

    *byte = sBuffer[sGetIndex++];
    if(sGetIndex == sBufferSize)
        sGetIndex = 0;
    sDataLength--;

    return true;
}

