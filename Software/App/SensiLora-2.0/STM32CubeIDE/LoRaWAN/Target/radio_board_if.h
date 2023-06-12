/**
  ******************************************************************************
  * @file    radio_board_if.h
  * @author  MCD Application Team
  * @brief   Header for Radio interface configuration
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RADIO_BOARD_IF_H
#define RADIO_BOARD_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmwx1zzabz_0xx.h"

#define Sx_Board_IoInit            CMWX1ZZABZ0XX_RADIO_IoInit
#define Sx_Board_IoDeInit          CMWX1ZZABZ0XX_RADIO_IoDeInit
#define Sx_Board_IoIrqInit         CMWX1ZZABZ0XX_RADIO_IoIrqInit
#define Sx_Board_SendRecv          CMWX1ZZABZ0XX_RADIO_SendRecv
#define Sx_Board_ChipSelect        CMWX1ZZABZ0XX_RADIO_ChipSelect
#define Sx_Board_CheckRfFrequency  CMWX1ZZABZ0XX_RADIO_CheckRfFrequency
#define Sx_Board_Reset             CMWX1ZZABZ0XX_RADIO_Reset
#define Sx_Board_SetXO             CMWX1ZZABZ0XX_RADIO_SetXO
#define Sx_Board_GetWakeUpTime     CMWX1ZZABZ0XX_RADIO_GetWakeUpTime
#define Sx_Board_GetPaSelect       CMWX1ZZABZ0XX_RADIO_GetPaSelect
#define Sx_Board_SetAntSw          CMWX1ZZABZ0XX_RADIO_SetAntSw
#define Sx_Board_Bus_Init          CMWX1ZZABZ0XX_RADIO_Bus_Init
#define Sx_Board_Bus_deInit        CMWX1ZZABZ0XX_RADIO_Bus_deInit
#define Sx_Board_GetDio1PinState   CMWX1ZZABZ0XX_RADIO_GetDio1PinState

/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* RADIO_BOARD_IF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

