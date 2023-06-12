/**
 ******************************************************************************
 * @file    RingBuffer.h
 * @date    30-June-2022
 * @brief   The RingBuffer class description.
 ******************************************************************************
 *
 * COPYRIGHT(c) 2022 SensiEDGE LTD
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

#ifndef __RINGBUFFER_H
#define __RINGBUFFER_H

#include <stdint.h>

/**
* @brief   Put new element to ring buffer.
* @param   element:  new element.
* @retval  none.
*/
void RignBuffPut(uint8_t element);

/**
* @brief   Get a number of elements from ring buffer.
* @param   data:     Specify the pointer to array, where to save data.
* @param   length:   Specify the link to variable, vhere to set length of
*                    read data from ring buffer.
* @param   maxSize:  Specify  number of data to read from buffer.
* @retval  none.
*/
void RignBuffGet(uint8_t* data, uint16_t *length, uint16_t maxSize);

/**
* @brief   Get a length of received elements in ring buffer.
* @param   none.
* @retval  uint16_t: length of received elements.
*/
uint16_t RignBuffGetDataLenght();

/**
* @brief   Clear the ring buffer.
* @param   none.
* @retval  none.
*/
void RignBuffClear();

#endif

