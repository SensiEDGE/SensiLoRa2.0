/**
 ******************************************************************************
 * @file    stm32_adv_trace.c
 * @author  MCD Application Team
 * @brief   This file contains the advanced trace utility functions.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32_adv_trace.h"
#include "stdarg.h"
#include "stdio.h"

/** @addtogroup ADV_TRACE
 * @{
 */

/* Private defines -----------------------------------------------------------*/

/** @defgroup ADV_TRACE_Private_defines ADV_TRACE Privates defines
 *  @{
 */

/**
 *  @brief  memory address of the trace buffer location.
 *  This define can be used, to change the buffer location.
 *
 */
#if !defined(UTIL_ADV_TRACE_MEMLOCATION)
#define UTIL_ADV_TRACE_MEMLOCATION
#endif

#if defined(UTIL_ADV_TRACE_OVERRUN)
/**
 *  @brief  List the overrun status.
 *  list of the overrun status used to handle the overrun trace evacuation.
 *
 *  @note only valid if UTIL_ADV_TRACE_OVERRUN has been enabled inside utilities conf
 */
typedef enum {
  TRACE_OVERRUN_NONE = 0, /*!<overrun status none.                        */
  TRACE_OVERRUN_INDICATION, /*!<overrun status an indication shall be sent. */
  TRACE_OVERRUN_TRANSFERT, /*!<overrun status data transfer ongoing.       */
  TRACE_OVERRUN_EXECUTED, /*!<overrun status data transfer complete.      */
} TRACE_OVERRUN_STATUS;
#endif

#if defined(UTIL_ADV_TRACE_UNCHUNK_MODE)
/**
 *  @brief  List the unchunk status.
 *  list of the unchunk status used to handle the unchunk case.
 *
 *  @note only valid if UTIL_ADV_TRACE_UNCHUNK_MODE has been enabled inside utilities conf
 */
typedef enum {
  TRACE_UNCHUNK_NONE = 0,     /*!<unchunk status none.                            */
  TRACE_UNCHUNK_DETECTED,     /*!<unchunk status an unchunk has been detected.    */
  TRACE_UNCHUNK_TRANSFER      /*!<unchunk status an unchunk transfer is ongoing. */
} TRACE_UNCHUNK_STATUS;
#endif
/**
 * @}
 */

/**
 *  @brief  advanced macro to override to enable debug mode
 */
#ifndef UTIL_ADV_TRACE_DEBUG
#define UTIL_ADV_TRACE_DEBUG(...)
#endif
/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/** @defgroup ADV_TRACE_private_typedef ADV_TRACE private typedef
 *  @{
 */

/**
 *  @brief  ADV_TRACE_Context.
 *  this structure contains all the data to handle the trace context.
 *
 *  @note some part of the context are depend with the selected switch inside the configuration file
 *  UTIL_ADV_TRACE_UNCHUNK_MODE, UTIL_ADV_TRACE_OVERRUN, UTIL_ADV_TRACE_CONDITIONNAL
 */
typedef struct {
#if defined(UTIL_ADV_TRACE_UNCHUNK_MODE)
  uint16_t unchunk_enabled;                              /*!<unchunk enable.                           */
  TRACE_UNCHUNK_STATUS unchunk_status;                   /*!<unchunk transfer status.                  */
#endif
#if defined(UTIL_ADV_TRACE_OVERRUN)
  TRACE_OVERRUN_STATUS OverRunStatus;                    /*!<overrun status.                             */
  cb_overrun *overrun_func;                               /*!<overrun function                            */
#endif
#if defined(UTIL_ADV_TRACE_CONDITIONNAL)
  cb_timestamp *timestamp_func; /*!<ptr of function used to insert time stamp.        */
  uint8_t  CurrentVerboseLevel; /*!<verbose level used.                                */
  uint32_t RegionMask; /*!<mask of the enabled region.                                */
#endif
  uint16_t TraceRdPtr; /*!<read pointer the trace system.                             */
  uint16_t TraceWrPtr; /*!<write pointer the trace system.                            */
  uint16_t TraceSentSize; /*!<size of the latest transfer.                            */
  uint16_t TraceLock; //lock counter of the trace system.
  uint16_t Size;     //size buffer.
} ADV_TRACE_Context;

/**
 *  @}
 */

/* Private variables ---------------------------------------------------------*/
/** @defgroup ADV_TRACE_private_variable ADV_TRACE private variable
 * private variable of the advanced trace system.
 *  @{
 */

/**
 * @brief trace context
 * this variable contains all the internal data of the advanced trace system.
 */
static ADV_TRACE_Context ADV_TRACE_Ctx;
static UTIL_ADV_TRACE_MEMLOCATION uint8_t ADV_TRACE_Buffer[UTIL_ADV_TRACE_FIFO_SIZE];

#if defined(UTIL_ADV_TRACE_CONDITIONNAL) && defined(UTIL_ADV_TRACE_UNCHUNK_MODE)
/**
 * @brief temporary buffer used by UTIL_ADV_TRACE_COND_FSend
 * a temporary buffers variable used to evaluate a formatted string size.
 */
static uint8_t sztmp[UTIL_ADV_TRACE_TMP_BUF_SIZE];
#endif
/**
 * @}
 */

/* Private function prototypes -----------------------------------------------*/
/** @defgroup ADV_TRACE_private_function ADV_TRACE private function
 *
 *  @{
 */
static void TRACE_TxCpltCallback(void *Ptr);
static int16_t TRACE_AllocateBufer(uint16_t Size, uint16_t *Pos);
static UTIL_ADV_TRACE_Status_t TRACE_Send(void);

static void TRACE_Lock(void);
static void TRACE_UnLock(void);
static uint32_t TRACE_IsLocked(void);

/**
 * @}
 */

/* Functions Definition ------------------------------------------------------*/

/** @addtogroup ADV_TRACE_exported_function
 *  @{
 */
UTIL_ADV_TRACE_Status_t UTIL_ADV_TRACE_Init(void)
{
  /* initialize the Ptr for Read/Write */
  (void)UTIL_ADV_TRACE_MEMSET8(&ADV_TRACE_Ctx, 0x0, sizeof(ADV_TRACE_Context));
  (void)UTIL_ADV_TRACE_MEMSET8(&ADV_TRACE_Buffer, 0x0, sizeof(ADV_TRACE_Buffer));

#if defined(UTIL_ADV_TRACE_UNCHUNK_MODE)
  UTIL_ADV_TRACE_DEBUG("\nUNCHUNK_MODE\n");
#endif
  /* Allocate Lock resource */
  UTIL_ADV_TRACE_INIT_CRITICAL_SECTION();

  /* Initialize the Low Level interface */
  return UTIL_TraceDriver.Init(TRACE_TxCpltCallback);
}

UTIL_ADV_TRACE_Status_t UTIL_ADV_TRACE_DeInit(void)
{
  /* Un-initialize the Low Level interface */
  return UTIL_TraceDriver.DeInit();
}

uint8_t UTIL_ADV_TRACE_IsBufferEmpty(void)
{
  /* check of the buffer is empty */
  if(ADV_TRACE_Ctx.TraceWrPtr == ADV_TRACE_Ctx.TraceRdPtr)
    return 1;
  return 0;
}

UTIL_ADV_TRACE_Status_t UTIL_ADV_TRACE_StartRxProcess(void (*UserCallback)(uint8_t *PData, uint16_t Size, uint8_t Error))
{
  /* start the RX process */
  return UTIL_TraceDriver.StartRx(UserCallback);
}

#if defined(UTIL_ADV_TRACE_CONDITIONNAL)
UTIL_ADV_TRACE_Status_t UTIL_ADV_TRACE_COND_FSend(uint32_t VerboseLevel, uint32_t Region, uint32_t TimeStampState, const char *strFormat, ...)
{
  va_list vaArgs;

  uint8_t buf[UTIL_ADV_TRACE_TMP_BUF_SIZE+UTIL_ADV_TRACE_TMP_MAX_TIMESTMAP_SIZE];

  uint16_t buff_size = 0u;

  /* check verbose level */
  if(!(ADV_TRACE_Ctx.CurrentVerboseLevel >= VerboseLevel))
  {
    return UTIL_ADV_TRACE_GIVEUP;
  }

  if((Region & ADV_TRACE_Ctx.RegionMask) != Region)
  {
    return UTIL_ADV_TRACE_REGIONMASKED;
  }

  if((ADV_TRACE_Ctx.timestamp_func != NULL) && (TimeStampState != 0u))
  {
    ADV_TRACE_Ctx.timestamp_func(buf,&buff_size);
  }

  va_start(vaArgs, strFormat);
  buff_size += (uint16_t) UTIL_ADV_TRACE_VSNPRINTF((char* )(buf + buff_size), UTIL_ADV_TRACE_TMP_BUF_SIZE, strFormat, vaArgs);
  va_end(vaArgs);

  return UTIL_ADV_TRACE_Send(buf, buff_size);

}
#endif

UTIL_ADV_TRACE_Status_t UTIL_ADV_TRACE_FSend(const char *strFormat, ...)
{
  uint8_t buf[UTIL_ADV_TRACE_TMP_BUF_SIZE];
  va_list vaArgs;

  va_start(vaArgs, strFormat);
  uint16_t bufSize = (uint16_t) UTIL_ADV_TRACE_VSNPRINTF((char* )buf, UTIL_ADV_TRACE_TMP_BUF_SIZE, strFormat, vaArgs);
  va_end(vaArgs);

  return UTIL_ADV_TRACE_Send(buf, bufSize);
}

#if defined(UTIL_ADV_TRACE_CONDITIONNAL)
UTIL_ADV_TRACE_Status_t UTIL_ADV_TRACE_COND_ZCSend_Allocation(uint32_t VerboseLevel, uint32_t Region, uint32_t TimeStampState, uint16_t length, uint8_t **pData, uint16_t *FifoSize, uint16_t *WritePos)
{
  UTIL_ADV_TRACE_Status_t ret = UTIL_ADV_TRACE_OK;
  uint16_t writepos;
  uint8_t timestamp_ptr[UTIL_ADV_TRACE_TMP_MAX_TIMESTMAP_SIZE];
  uint16_t timestamp_size = 0u;

  /* check verbose level */
  if(!(ADV_TRACE_Ctx.CurrentVerboseLevel >= VerboseLevel))
  {
    return UTIL_ADV_TRACE_GIVEUP;
  }

  if((Region & ADV_TRACE_Ctx.RegionMask) != Region)
  {
    return UTIL_ADV_TRACE_REGIONMASKED;
  }

  if((ADV_TRACE_Ctx.timestamp_func != NULL) && (TimeStampState != 0u))
  {
    ADV_TRACE_Ctx.timestamp_func(timestamp_ptr, &timestamp_size);
  }

  TRACE_Lock();
  
  /* if allocation is ok, write data into the buffer */
  if (TRACE_AllocateBufer(length+timestamp_size, &writepos) != -1)
  {
    /* fill time stamp information */
    for (uint16_t index = 0u; index < timestamp_size; index++)
    {
      ADV_TRACE_Buffer[writepos] = timestamp_ptr[index];
      writepos = (uint16_t) ((writepos + 1u) % UTIL_ADV_TRACE_FIFO_SIZE);
    }

    /*user fill */
    *pData = ADV_TRACE_Buffer;
    *FifoSize = (uint16_t) UTIL_ADV_TRACE_FIFO_SIZE;
    *WritePos = writepos;
  }
  else
  {
    TRACE_UnLock();
    ret = UTIL_ADV_TRACE_MEM_FULL;
  }
  return ret;
}

UTIL_ADV_TRACE_Status_t UTIL_ADV_TRACE_COND_ZCSend_Finalize(void)
{
  return UTIL_ADV_TRACE_ZCSend_Finalize();
}
#endif

UTIL_ADV_TRACE_Status_t UTIL_ADV_TRACE_ZCSend_Allocation(uint16_t Length, uint8_t **pData, uint16_t *FifoSize, uint16_t *WritePos)
{
  UTIL_ADV_TRACE_Status_t ret = UTIL_ADV_TRACE_OK;
  uint16_t writepos;

  TRACE_Lock();

  /* if allocation is ok, write data into the buffer */
  if (TRACE_AllocateBufer(Length,&writepos)  != -1)
  {
    /*user fill */
    *pData = ADV_TRACE_Buffer;
    *FifoSize = UTIL_ADV_TRACE_FIFO_SIZE;
    *WritePos = (uint16_t)writepos;
  }
  else
  {
    TRACE_UnLock();
    ret = UTIL_ADV_TRACE_MEM_FULL;
  }

  return ret;
}

UTIL_ADV_TRACE_Status_t UTIL_ADV_TRACE_ZCSend_Finalize(void)
{
  TRACE_UnLock();
  return TRACE_Send();
}

#if defined(UTIL_ADV_TRACE_CONDITIONNAL)
UTIL_ADV_TRACE_Status_t UTIL_ADV_TRACE_COND_Send(uint32_t VerboseLevel, uint32_t Region, uint32_t TimeStampState, const uint8_t *pData, uint16_t Length)
{
  UTIL_ADV_TRACE_Status_t ret;
  uint16_t writepos;
  uint32_t idx;
  uint8_t timestamp_ptr[UTIL_ADV_TRACE_TMP_MAX_TIMESTMAP_SIZE];
  uint16_t timestamp_size = 0u;

  /* check verbose level */
  if(!(ADV_TRACE_Ctx.CurrentVerboseLevel >= VerboseLevel))
  {
    return UTIL_ADV_TRACE_GIVEUP;
  }

  if((Region & ADV_TRACE_Ctx.RegionMask) != Region)
  {
    return UTIL_ADV_TRACE_REGIONMASKED;
  }

  if((ADV_TRACE_Ctx.timestamp_func != NULL) && (TimeStampState != 0u))
  {
    ADV_TRACE_Ctx.timestamp_func(timestamp_ptr, &timestamp_size);
  }

  TRACE_Lock();

  /* if allocation is ok, write data into the buffer */
  if (TRACE_AllocateBufer(Length + timestamp_size, &writepos) != -1)
  {
    /* fill time stamp information */
    for (idx = 0; idx < timestamp_size; idx++)
    {
      ADV_TRACE_Buffer[writepos] = timestamp_ptr[idx];
      writepos = (uint16_t) ((writepos + 1u) % UTIL_ADV_TRACE_FIFO_SIZE);
    }

    for (idx = 0u; idx < Length; idx++)
    {
      ADV_TRACE_Buffer[writepos] = pData[idx];
      writepos = (uint16_t) ((writepos + 1u) % UTIL_ADV_TRACE_FIFO_SIZE);
    }

    TRACE_UnLock();
    ret = TRACE_Send();
  }
  else
  {
    TRACE_UnLock();
    ret = UTIL_ADV_TRACE_MEM_FULL;
  }

  return ret;
}
#endif

UTIL_ADV_TRACE_Status_t UTIL_ADV_TRACE_Send(const uint8_t *pData, uint16_t Length)
{
  UTIL_ADV_TRACE_Status_t ret = UTIL_ADV_TRACE_OK;
  uint32_t idx;

  TRACE_Lock();

  memset(ADV_TRACE_Buffer, 0 ,sizeof(ADV_TRACE_Buffer));
  for (idx = 0u; idx < Length; idx++)
  {
      ADV_TRACE_Buffer[idx] = pData[idx];
  }
  ADV_TRACE_Ctx.Size = Length;
  TRACE_UnLock();

  ret = TRACE_Send();

  return ret;
}

#if defined(UTIL_ADV_TRACE_OVERRUN)
void UTIL_ADV_TRACE_RegisterOverRunFunction(cb_overrun *cb)
{
  ADV_TRACE_Ctx.overrun_func = *cb;
}
#endif

#if defined(UTIL_ADV_TRACE_CONDITIONNAL)
void UTIL_ADV_TRACE_RegisterTimeStampFunction(cb_timestamp *cb)
{
  ADV_TRACE_Ctx.timestamp_func = *cb;
}

void UTIL_ADV_TRACE_SetVerboseLevel(uint8_t Level)
{
  ADV_TRACE_Ctx.CurrentVerboseLevel = Level;
}

uint8_t UTIL_ADV_TRACE_GetVerboseLevel(void)
{
  return ADV_TRACE_Ctx.CurrentVerboseLevel;
}

void UTIL_ADV_TRACE_SetRegion(uint32_t Region)
{
  ADV_TRACE_Ctx.RegionMask |= Region;
}

uint32_t UTIL_ADV_TRACE_GetRegion(void)
{
  return ADV_TRACE_Ctx.RegionMask;
}

void UTIL_ADV_TRACE_ResetRegion(uint32_t Region)
{
  ADV_TRACE_Ctx.RegionMask &= ~Region;
}
#endif

__WEAK void UTIL_ADV_TRACE_PreSendHook(void)
{
}

__WEAK void UTIL_ADV_TRACE_PostSendHook(void)
{
}

/**
 * @}
 */

/** @addtogroup ADV_TRACE_private_function
 *  @{
 */

/**
 * @brief send the data of the trace to low layer
 * @retval Status based on @ref UTIL_ADV_TRACE_Status_t
 */
static UTIL_ADV_TRACE_Status_t TRACE_Send(void)
{
  UTIL_ADV_TRACE_Status_t ret = UTIL_ADV_TRACE_OK;
  uint8_t *ptr = NULL;



  if(TRACE_IsLocked() == 0u)
  {
      TRACE_Lock();

      ptr = &ADV_TRACE_Buffer[0];

      UTIL_ADV_TRACE_PreSendHook();

      ret = UTIL_TraceDriver.Send(ptr, ADV_TRACE_Ctx.Size);

      UTIL_ADV_TRACE_PostSendHook();
      TRACE_UnLock();

  }


  return ret;
}

/**
 * @brief Tx callback called by the low layer level to inform a transfer complete
 * @param Ptr pointer not used only for HAL compatibility
 * @retval none
 */
static void TRACE_TxCpltCallback(void *Ptr)
{
  uint8_t *ptr = NULL;
  UTIL_ADV_TRACE_ENTER_CRITICAL_SECTION();


  ADV_TRACE_Ctx.TraceRdPtr = (ADV_TRACE_Ctx.TraceRdPtr + ADV_TRACE_Ctx.TraceSentSize) % UTIL_ADV_TRACE_FIFO_SIZE;


  if((ADV_TRACE_Ctx.TraceRdPtr != ADV_TRACE_Ctx.TraceWrPtr) && (1u == ADV_TRACE_Ctx.TraceLock))
  {
      if(ADV_TRACE_Ctx.TraceWrPtr > ADV_TRACE_Ctx.TraceRdPtr)
      {
        ADV_TRACE_Ctx.TraceSentSize = ADV_TRACE_Ctx.TraceWrPtr - ADV_TRACE_Ctx.TraceRdPtr;
      }
      else /* TraceRdPtr > TraceWrPtr */
      {
        ADV_TRACE_Ctx.TraceSentSize = UTIL_ADV_TRACE_FIFO_SIZE - ADV_TRACE_Ctx.TraceRdPtr;
      }

    ptr = &ADV_TRACE_Buffer[ADV_TRACE_Ctx.TraceRdPtr];
    UTIL_ADV_TRACE_EXIT_CRITICAL_SECTION();
    UTIL_ADV_TRACE_DEBUG("\n--TRACE_Send(%d-%d)--\n", ADV_TRACE_Ctx.TraceRdPtr, ADV_TRACE_Ctx.TraceSentSize);
    UTIL_TraceDriver.Send(ptr, ADV_TRACE_Ctx.TraceSentSize);
  }
  else
  {
    UTIL_ADV_TRACE_EXIT_CRITICAL_SECTION();
    UTIL_ADV_TRACE_PostSendHook();
    TRACE_UnLock();
  }
}

/**
 * @brief  allocate space inside the buffer to push data
 * @param  Size to allocate within fifo
 * @param  Pos position within the fifo
 * @retval write position inside the buffer is -1 no space available.
 */
static int16_t TRACE_AllocateBufer(uint16_t Size, uint16_t *Pos)
{
  uint16_t freesize;
  int16_t ret = -1;

  UTIL_ADV_TRACE_ENTER_CRITICAL_SECTION();

  if(ADV_TRACE_Ctx.TraceWrPtr == ADV_TRACE_Ctx.TraceRdPtr)
  {
#ifdef UTIL_ADV_TRACE_UNCHUNK_MODE
    freesize = (uint16_t)(UTIL_ADV_TRACE_FIFO_SIZE - ADV_TRACE_Ctx.TraceWrPtr);
    if((Size >= freesize) && (ADV_TRACE_Ctx.TraceRdPtr > Size))
    {
      ADV_TRACE_Ctx.unchunk_status = TRACE_UNCHUNK_DETECTED;
      ADV_TRACE_Ctx.unchunk_enabled = ADV_TRACE_Ctx.TraceWrPtr;
      freesize = ADV_TRACE_Ctx.TraceRdPtr;
      ADV_TRACE_Ctx.TraceWrPtr = 0;
    }
#else
    /* need to add buffer full management*/
    freesize = (int16_t)UTIL_ADV_TRACE_FIFO_SIZE;
#endif
  }
  else
  {
#ifdef UTIL_ADV_TRACE_UNCHUNK_MODE
    if (ADV_TRACE_Ctx.TraceWrPtr > ADV_TRACE_Ctx.TraceRdPtr)
    {
      freesize = (uint16_t)(UTIL_ADV_TRACE_FIFO_SIZE - ADV_TRACE_Ctx.TraceWrPtr);
      if((Size >= freesize) && (ADV_TRACE_Ctx.TraceRdPtr > Size)) 
      {
        ADV_TRACE_Ctx.unchunk_status = TRACE_UNCHUNK_DETECTED;
        ADV_TRACE_Ctx.unchunk_enabled = ADV_TRACE_Ctx.TraceWrPtr;
        freesize = ADV_TRACE_Ctx.TraceRdPtr;
        ADV_TRACE_Ctx.TraceWrPtr = 0;
      }
    }
    else
    {
      freesize = (uint16_t)(ADV_TRACE_Ctx.TraceRdPtr - ADV_TRACE_Ctx.TraceWrPtr);
      }
#else
    if (ADV_TRACE_Ctx.TraceWrPtr > ADV_TRACE_Ctx.TraceRdPtr)
    {
      freesize = UTIL_ADV_TRACE_FIFO_SIZE - ADV_TRACE_Ctx.TraceWrPtr + ADV_TRACE_Ctx.TraceRdPtr;
    }
    else
    {
      freesize = ADV_TRACE_Ctx.TraceRdPtr - ADV_TRACE_Ctx.TraceWrPtr;
    }
#endif
  }

  if(freesize > Size)
  {
    *Pos = ADV_TRACE_Ctx.TraceWrPtr;
    ADV_TRACE_Ctx.TraceWrPtr = (ADV_TRACE_Ctx.TraceWrPtr + Size) % UTIL_ADV_TRACE_FIFO_SIZE;
    ret = 0;
#if defined(UTIL_ADV_TRACE_OVERRUN)
    if(ADV_TRACE_Ctx.OverRunStatus == TRACE_OVERRUN_EXECUTED)
    {
      /* clear the over run */
      ADV_TRACE_Ctx.OverRunStatus = TRACE_OVERRUN_NONE;
    }
#endif

#ifdef UTIL_ADV_TRACE_UNCHUNK_MODE
    UTIL_ADV_TRACE_DEBUG("\n--TRACE_AllocateBufer(%d-%d-%d::%d-%d)--\n", freesize - Size, Size, ADV_TRACE_Ctx.unchunk_enabled, ADV_TRACE_Ctx.TraceRdPtr, ADV_TRACE_Ctx.TraceWrPtr);
#else
    UTIL_ADV_TRACE_DEBUG("\n--TRACE_AllocateBufer(%d-%d::%d-%d)--\n",freesize - Size, Size, ADV_TRACE_Ctx.TraceRdPtr, ADV_TRACE_Ctx.TraceWrPtr);
#endif
  }
#if defined(UTIL_ADV_TRACE_OVERRUN)
  else
  {
    if((ADV_TRACE_Ctx.OverRunStatus == TRACE_OVERRUN_NONE) && (NULL != ADV_TRACE_Ctx.overrun_func))
    {
      UTIL_ADV_TRACE_DEBUG(":TRACE_OVERRUN_INDICATION");
      ADV_TRACE_Ctx.OverRunStatus = TRACE_OVERRUN_INDICATION;
    }
  }
#endif

  UTIL_ADV_TRACE_EXIT_CRITICAL_SECTION();
  return ret;
}

/**
 * @brief  Lock the trace buffer.
 * @retval None.
 */
static void TRACE_Lock(void)
{
  UTIL_ADV_TRACE_ENTER_CRITICAL_SECTION();
  ADV_TRACE_Ctx.TraceLock++;
  UTIL_ADV_TRACE_EXIT_CRITICAL_SECTION();
}

/**
 * @brief  UnLock the trace buffer.
 * @retval None.
 */
static void TRACE_UnLock(void)
{
  UTIL_ADV_TRACE_ENTER_CRITICAL_SECTION();
  ADV_TRACE_Ctx.TraceLock--;
  UTIL_ADV_TRACE_EXIT_CRITICAL_SECTION();
}

/**
 * @brief  UnLock the trace buffer.
 * @retval None.
 */
static uint32_t TRACE_IsLocked(void)
{
  return (ADV_TRACE_Ctx.TraceLock == 0u? 0u: 1u);
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

