/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v2.0_Cube
  * @brief          : This file implements the USB Device
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/** @addtogroup PROJECTS PROJECTS
  * @{
  */
   
/** @addtogroup STEVAL-STRKT01 STEVAL-STRKT01
  * @{
  */  
   
/** @addtogroup STEVAL-STRKT01_ASSET_TRACKER Asset Tracker
  * @{
  */    
    
/** @addtogroup STEVAL-STRKT01_LoRaWAN LoRaWAN
  * @{
 */

/** @addtogroup STEVAL-STRKT01_BSP_INTERFACE BSP Interface
  * @{
 */

/** @addtogroup STEVAL-STRKT01_APP_USB USB
  * @{
  */    
   
/** @addtogroup STEVAL-STRKT01_APP_USB_DEVICE USB DEVICE
  * @{
 */

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STEVAL-STRKT01_APP_USB_DEVICE_Private_FunctionPrototypes USB DEVICE Private Function Prototypes
  * @brief Private function prototypes
  * @{
 */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/**
  * @}
  */

/** @addtogroup STEVAL-STRKT01_APP_USB_DEVICE_Exported_Variables USB DEVICE Exported Variables
  * @brief Exported variables
  * @{
 */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;

pUsbConnectionNotify_t UsbConnectionNotify = {0};

/**
  * @}
  */


/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */
static bool sActiveUsb;
bool sIsdisconnectUsb;

/**
  * @brief Status active Usb
  * @param None
  * @retval true - Active Usb, false - not active Usb
  */
bool isActiveUsb(void)
{
    if (sIsdisconnectUsb == true) {
        if (sActiveUsb == true) {
            sActiveUsb = false;
        }
        sIsdisconnectUsb = false;
    }
    return sActiveUsb;
}
/* USER CODE END 1 */

/** @addtogroup STEVAL-STRKT01_APP_USB_DEVICE_Public_Functions USB DEVICE Public Functions
  * @brief Public functions
  * @{
 */
static void userPCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SetupStage((USBD_HandleTypeDef*)hpcd->pData, (uint8_t *)hpcd->Setup);
  
  sActiveUsb = true;
  sIsdisconnectUsb = false;

  if(hpcd->Setup[0] == 0x00032221)
  {
    if(UsbConnectionNotify)UsbConnectionNotify(1);
  }
  else if(hpcd->Setup[0] == 0x00022221)
  {
    if(UsbConnectionNotify)UsbConnectionNotify(0);
  }
}


/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void HW_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */

  /* USER CODE END USB_DEVICE_Init_PreTreatment */
  
  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    
  }
  
  extern PCD_HandleTypeDef hpcd_USB_FS;
  HAL_PCD_RegisterCallback(&hpcd_USB_FS, HAL_PCD_SETUPSTAGE_CB_ID, userPCD_SetupStageCallback);
  
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
  {
    
  }
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
  {
    
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    
  }
  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */
  
  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}


/**
  * @brief Stop the USB library and terminate the USB device
  * @param None
  * @retval None
  */
void HW_USB_DEVICE_DeInit(void)
{
    USBD_Stop(&hUsbDeviceFS);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
