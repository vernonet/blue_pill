/**
  ******************************************************************************
  * @file    USB_Device/MSC_Standalone/Src/usbd_storage.c
  * @author  MCD Application Team
  * @version V1.6.0
  * @date    12-May-2017
  * @brief   Memory management layer
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright © 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------ */
#include "usbd_storage.h"
#include "usb_device.h"
#include "W25X80.h"
//#include "flashchips.h"
//#include "chip_drv.h"
//#include "spi.h"
//#include "stm32f4_discovery.h"
//#include "ff.h"
//#include "Driver_SPI.h"
//#include <stdlib.h>
//#include <string.h>


/* Private typedef ----------------------------------------------------------- */
/* Private define ------------------------------------------------------------ */
//#define STORAGE_LUN_NBR                  1
//#define STORAGE_BLK_NBR                  0x10000
//#define STORAGE_BLK_SIZ                  0x200

/* Private macro ------------------------------------------------------------- */
/* Private variables --------------------------------------------------------- */
/* USB Mass storage Standard Inquiry Data */
extern int8_t STORAGE_Inquirydata_FS[];

/* Private function prototypes ----------------------------------------------- */
int8_t STORAGE_Init(uint8_t lun);
int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t * block_num,
                           uint16_t * block_size);
int8_t STORAGE_IsReady(uint8_t lun);
int8_t STORAGE_IsWriteProtected(uint8_t lun);
int8_t STORAGE_Read(uint8_t lun, uint8_t * buf, uint32_t blk_addr,
                    uint16_t blk_len);
int8_t STORAGE_Write(uint8_t lun, uint8_t * buf, uint32_t blk_addr,
                     uint16_t blk_len);
int8_t STORAGE_GetMaxLun(void);
//static uint32_t GetSector(uint32_t Address);
//static uint32_t GetSectorSize(uint32_t Sector);
static int8_t Write_LL(uint32_t  dest, uint8_t * buf, uint16_t len);
int8_t Sector_Erase(void);
//static void MX_TIM6_Init(void);

uint32_t FirstSector = 0, NbOfSectors = 0, Address = 0, Sector = 0;
uint32_t SectorError = 0, last_wr_adr=0xffffffff, modulo=0;
/*Variable used for Erase procedure*/
uint8_t flash_eraset = 0, temp=0,Wr_Protect, mod=0;
uint8_t write_started =0;
uint16_t block_num_cnt=0;
//IWDG_HandleTypeDef hiwdg;
extern uint8_t complet, error;
//TIM_HandleTypeDef htim6;
extern uint8_t inter;
volatile uint32_t  ttt=0;
extern __IO uint32_t CRCValue_nominal;
bool backup_mode = false;  //backup firmware first
extern uint32_t spi_speed;
ARM_SPI_STATUS  sts;


 


uint8_t a[5];

int32_t file_size = 0;



USBD_StorageTypeDef USBD_DISK_fops = {
  STORAGE_Init,
  STORAGE_GetCapacity,
  STORAGE_IsReady,
  STORAGE_IsWriteProtected,
  STORAGE_Read,
  STORAGE_Write,
  STORAGE_GetMaxLun,
  STORAGE_Inquirydata_FS,
};

/* Private functions --------------------------------------------------------- */
/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/




/**
  * @brief  Initailizes the storage unit (medium)       
  * @param  lun: Logical unit number
  * @retval Status (0 : Ok / -1 : Error)
  */
int8_t STORAGE_Init(uint8_t lun)
{
  switch (lun)
  {
    case 0:
      //HAL_FLASH_Unlock(); 
      break;
    case 1:
      return 1;
    default:
      return 1;
  }
  return 0;
}

/**
  * @brief  Returns the medium capacity.      
  * @param  lun: Logical unit number
  * @param  block_num: Number of total block number
  * @param  block_size: Block size
  * @retval Status (0: Ok / -1: Error)
  */
int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t * block_num,
                           uint16_t * block_size)
{
  switch (lun)
  {
    case 0:
      *block_num  = FLASH_SECTOR_COUNT;
      *block_size = FLASH_SECTOR_SIZE;
      break;
    case 1:
      return 1;
    default:
      return 1;
  
  }
  return 0;
}

/**
  * @brief  Checks whether the medium is ready.  
  * @param  lun: Logical unit number
  * @retval Status (0: Ok / -1: Error)
  */
int8_t STORAGE_IsReady(uint8_t lun)
{
   switch (lun)
  {
    case 0:
      return 0;
//      break;
    case 1:
      return 1;
    default:
      return 1;
  }
}

/**
  * @brief  Checks whether the medium is write protected.
  * @param  lun: Logical unit number
  * @retval Status (0: write enabled / -1: otherwise)
  */
int8_t STORAGE_IsWriteProtected(uint8_t lun)
{
//	if (Wr_Protect) return -1;
//    else return 0;
	return 0;
}

/**
  * @brief  Reads data from the medium.
  * @param  lun: Logical unit number
  * @param  blk_addr: Logical block address
  * @param  blk_len: Blocks number
  * @retval Status (0: Ok / -1: Error)
  */
int8_t STORAGE_Read(uint8_t lun, uint8_t * buf, uint32_t blk_addr,
                    uint16_t blk_len)
{

	uint32_t * buf32 = (uint32_t*)buf;
	
  switch (lun)
  {
    case 0:			
			ReadData(blk_addr*STORAGE_BLK_SIZ, buf32, STORAGE_BLK_SIZ*blk_len);									
			//USBD_UsrLog("\n\r RD blk_addr-> %d blk_len-> %d", blk_addr,blk_len);
      break;
    case 1:
      break;
    default:
      return 1;
  }
  return 0;
}





/**
  * @brief  LL Writes data into the medium.
  * @param  dest: Destination addres
  * @param  src: Pointer to source data
  * @param  len: number byte for writing
  * @retval Status (0 : Ok / 1 : Error)
  */
static int8_t Write_LL(uint32_t  dest, uint8_t * src, uint16_t len)
{	                   
	 if (ProgramData (dest , src, len) != ARM_DRIVER_OK) return ARM_DRIVER_ERROR;							
		 				 			 
	   else	 return ARM_DRIVER_OK;
 
}

/**
  * @brief  Writes data into the medium.
  * @param  lun: Logical unit number
  * @param  blk_addr: Logical block address
  * @param  blk_len: Blocks number
  * @retval Status (0 : Ok / -1 : Error)
  */
int8_t STORAGE_Write(uint8_t lun, uint8_t * buf, uint32_t blk_addr,
                     uint16_t blk_len)
{                       
 
  switch (lun)
  {
    case 0:	 
     if (EraseSector (blk_addr*STORAGE_BLK_SIZ) != ARM_DRIVER_OK) {
			 return ARM_DRIVER_ERROR;
		 }									 				 			 

	     else if (Write_LL(blk_addr*STORAGE_BLK_SIZ, buf, blk_len*STORAGE_BLK_SIZ) == ARM_DRIVER_ERROR) 
							 { 
								return ARM_DRIVER_ERROR;
							 }			
		
     break;
   case 1:
     break;
   default:
     return 1;
  }
  return 0;
}

/**
  * @brief  Returns the Max Supported LUNs.   
  * @param  None
  * @retval Lun(s) number
  */
int8_t STORAGE_GetMaxLun(void)
{
  return (STORAGE_LUN_NBR - 1);
}




/**********************************END OF FILE**************************************/
