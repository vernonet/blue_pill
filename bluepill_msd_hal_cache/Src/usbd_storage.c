/**
  ******************************************************************************
  * @file    USB_Device/MSC_Standalone/Src/usbd_storage.c
  * @author  MCD Application Team
  * @version V1.6.0
  * @date    12-May-2017
  * @brief   Memory management layer
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------ */
#include "usbd_storage.h"
#include "usb_device.h"
#include "W25X80.h"


/* Private typedef ----------------------------------------------------------- */
/* Private define ------------------------------------------------------------ */


/* Private macro ------------------------------------------------------------- */
/* Private variables --------------------------------------------------------- */
/* USB Mass storage Standard Inquiry Data */
extern int8_t STORAGE_Inquirydata_FS[];
extern TIM_HandleTypeDef htim2;


struct sec_cache_  sec_cache = {0};
extern volatile bool chk_wr_cache;

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
static int8_t Write_LL(uint32_t  dest, uint8_t * buf, uint16_t len);
int8_t Sector_Erase(void);
/*Variable used for Erase procedure*/
uint8_t  temp=0,Wr_Protect;
extern uint8_t complet, error;
extern __IO uint32_t CRCValue_nominal;
bool backup_mode = false;  //backup firmware first
ARM_SPI_STATUS  sts;



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
      *block_num  = FLASH_SECTOR_COUNT*(FLASH_SECTOR_SIZE/MSC_SECTOR_SIZE);
      *block_size = MSC_SECTOR_SIZE;
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

//			 if (chk_wr_cache && sec_cache.is_cached && sec_cache.need_wr)  {
//				 EraseSector ((sec_cache.blk_adr&SECTOR_MASK)*STORAGE_BLK_SIZ);	 
//				 Write_LL ((sec_cache.blk_adr&SECTOR_MASK)*STORAGE_BLK_SIZ, &sec_cache.buf[0], FLASH_SECTOR_SIZE);
//				 sec_cache.is_cached = false;
//				 chk_wr_cache = false;
//				 sec_cache.need_wr = false;	 
//				 __HAL_TIM_SET_COUNTER(&htim2, 1);
//		   }
		
			if (sec_cache.is_cached) {
        if (sec_cache.blk_adr>>3 == blk_addr>>3){
					memcpy(buf, &sec_cache.buf[(blk_addr&CACHE_MASK)*STORAGE_BLK_SIZ], STORAGE_BLK_SIZ*blk_len);
					//sec_cache.blk_adr = blk_addr;
				}
          else {
						if (sec_cache.need_wr)  ReadData(blk_addr*STORAGE_BLK_SIZ, buf32, STORAGE_BLK_SIZ*blk_len);	
						   else {
								  ReadData((blk_addr&SECTOR_MASK)*STORAGE_BLK_SIZ, &sec_cache.buf[0], FLASH_SECTOR_SIZE);
									memcpy(buf, &sec_cache.buf[(blk_addr&CACHE_MASK)*STORAGE_BLK_SIZ], STORAGE_BLK_SIZ*blk_len);		
									sec_cache.is_cached = true;
									sec_cache.blk_adr   = blk_addr;	
							 }
											
					}
      }				
			  else {
					ReadData((blk_addr&SECTOR_MASK)*STORAGE_BLK_SIZ, &sec_cache.buf[0], FLASH_SECTOR_SIZE);
          memcpy(buf, &sec_cache.buf[(blk_addr&CACHE_MASK)*STORAGE_BLK_SIZ], STORAGE_BLK_SIZ*blk_len);		
          sec_cache.is_cached = true;
					sec_cache.need_wr   = false;
				  sec_cache.blk_adr   = blk_addr;					
					// ReadData(blk_addr*STORAGE_BLK_SIZ, buf32, STORAGE_BLK_SIZ*blk_len);
				}
			USBD_UsrLog("\n\r RD blk_addr-> %d blk_len-> %d", blk_addr,blk_len);
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
	 if (ProgramData (dest , src, len) != len) {
		   return ARM_DRIVER_ERROR;							
	 }
		 				 			 
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
 uint32_t * buf32 = (uint32_t*)&sec_cache.buf[(blk_addr&CACHE_MASK)*STORAGE_BLK_SIZ];	
 
  switch (lun)
  {
    case 0:	 
		
		if (sec_cache.is_cached && sec_cache.need_wr) {
        if ((sec_cache.blk_adr>>3) == (blk_addr>>3)){
					memcpy(&sec_cache.buf[(blk_addr&CACHE_MASK)*STORAGE_BLK_SIZ], buf, STORAGE_BLK_SIZ*blk_len);
					//sec_cache.blk_adr = blk_addr;
					sec_cache.need_wr   = true;
				}
				  else {
						if (EraseSector ((sec_cache.blk_adr&SECTOR_MASK)*STORAGE_BLK_SIZ) != ARM_DRIVER_OK) {
			         return ARM_DRIVER_ERROR;
		        }
              else if (Write_LL((sec_cache.blk_adr&SECTOR_MASK)*STORAGE_BLK_SIZ, &sec_cache.buf[0], FLASH_SECTOR_SIZE) == ARM_DRIVER_ERROR) 
							 { 
								return ARM_DRIVER_ERROR;
							 }
						__HAL_TIM_SET_COUNTER(&htim2, 1);	 
						ReadData((blk_addr&SECTOR_MASK)*STORAGE_BLK_SIZ, &sec_cache.buf[0], FLASH_SECTOR_SIZE);
						memcpy(&sec_cache.buf[(blk_addr&CACHE_MASK)*STORAGE_BLK_SIZ], buf, STORAGE_BLK_SIZ*blk_len);
						sec_cache.is_cached = true;	 
						sec_cache.need_wr   = true;			 
            sec_cache.blk_adr   = blk_addr;						 
					}
		}
      else {
				__HAL_TIM_SET_COUNTER(&htim2, 1);
				ReadData((blk_addr&SECTOR_MASK)*STORAGE_BLK_SIZ, &sec_cache.buf[0], FLASH_SECTOR_SIZE);	
        memcpy(&sec_cache.buf[(blk_addr&CACHE_MASK)*STORAGE_BLK_SIZ], buf, STORAGE_BLK_SIZ*blk_len);				
				sec_cache.is_cached = true;
				sec_cache.need_wr   = true;
				sec_cache.blk_adr   = blk_addr; 
			}
		 USBD_UsrLog("\n\r                                      WR blk_addr-> %d blk_len-> %d", blk_addr,blk_len); 
			
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
