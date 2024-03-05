/**
  ******************************************************************************
  * @file    usbd_msc_data.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides all the vital inquiry pages and sense data.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_msc_data.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup MSC_DATA 
  * @brief Mass storage info/data module
  * @{
  */ 

/** @defgroup MSC_DATA_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup MSC_DATA_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup MSC_DATA_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup MSC_DATA_Private_Variables
  * @{
  */ 


/* USB Mass storage Page 0 Inquiry Data */
const uint8_t  MSC_Page00_Inquiry_Data[] = {//7						
	0x00,		
	0x00, 
	0x00, 
	(LENGTH_INQUIRY_PAGE00 - 4),
	0x00, 
	0x80, 
	0x83 
};  



#if (0)  //old version
/* USB Mass storage sense 6  Data */
const uint8_t  MSC_Mode_Sense6_data[] = {
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00, 
	0x00,
	0x00
};	
/* USB Mass storage sense 10  Data */
const uint8_t  MSC_Mode_Sense10_data[] = {
	0x00,
	0x06, 
	0x00, 
	0x00, 
	0x00, 
	0x00, 
	0x00, 
	0x00
};

#else
/* USB Mass storage sense 6 Data */
uint8_t MSC_Mode_Sense6_data[MODE_SENSE6_LEN] =
{
  0x03,     /* MODE DATA LENGTH. The number of bytes that follow. */
  0x00,     /* MEDIUM TYPE. 00h for SBC devices. */
  0x00,     /* DEVICE-SPECIFIC PARAMETER. For SBC devices: 
             *   bit 7: WP. Set to 1 if the media is write-protected.
             *   bits 6..4: reserved
             *   bit 4: DPOFUA. Set to 1 if the device supports the DPO and FUA bits (used in caching)
             *   bits 3..0: reserved*/
  0x00      /* Put Product Serial number */
};


/* USB Mass storage sense 10  Data */
uint8_t MSC_Mode_Sense10_data[MODE_SENSE10_LEN] =
{
  0x07,     /* MODE DATA LENGTH. The number of bytes that follow. */
  0x00,     /* MEDIUM TYPE. 00h for SBC devices. */
  0x00,     /* DEVICE-SPECIFIC PARAMETER. For SBC devices: 
             *   bit 7: WP. Set to 1 if the media is write-protected.
             *   bits 6..4: reserved
             *   bit 4: DPOFUA. Set to 1 if the device supports the DPO and FUA bits (used in caching)
             *   bits 3..0: reserved*/
  0x00,     /* Reserved */
  0x00,     /* Reserved */
  0x00,     /* Reserved */
  0x00,     /* Reserved */
  0x00      /* BLOCK DESCRIPTOR LENGTH. The length in bytes of all block descriptors in the
             *   mode parameter list. */
};
#endif

/**
  * @}
  */ 


/** @defgroup MSC_DATA_Private_FunctionPrototypes
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup MSC_DATA_Private_Functions
  * @{
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
