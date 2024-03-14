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
 * <h2><center>&copy; Copyright ɠ2016 STMicroelectronics International N.V.
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
#include "RTE_Device.h"
#include "ff.h"
#include "usbd_storage.h"
#include "usb_device.h"
#include "flashchips.h"
#include "chip_drv.h"
#include "spi.h"
#include "Driver_SPI.h"

/* Private typedef ----------------------------------------------------------- */
/* Private define ------------------------------------------------------------ */

/* Private macro ------------------------------------------------------------- */
/* Private variables --------------------------------------------------------- */
/* USB Mass storage Standard Inquiry Data */
extern int8_t STORAGE_Inquirydata_FS[];

/* Private function prototypes ----------------------------------------------- */
int8_t STORAGE_Init(uint8_t lun);
int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t *block_num,
                           uint16_t *block_size);
int8_t STORAGE_IsReady(uint8_t lun);
int8_t STORAGE_IsWriteProtected(uint8_t lun);
int8_t STORAGE_Read(uint8_t lun, uint8_t *buf, uint32_t blk_addr,
                    uint16_t blk_len);
int8_t STORAGE_Write(uint8_t lun, uint8_t *buf, uint32_t blk_addr,
                     uint16_t blk_len);
int8_t STORAGE_GetMaxLun(void);
// static uint32_t GetSector(uint32_t Address);
// static uint32_t GetSectorSize(uint32_t Sector);
static int8_t Write_LL(Dst_data dst_dta, uint32_t dest, uint8_t *buf, uint16_t len);
int8_t Sector_Erase(void);


uint32_t  modulo = 0;
/*Variable used for Erase procedure*/
uint8_t flash_eraset = 0, temp = 0, Wr_Protect, mod = 0;
uint16_t block_num_cnt = 0;
extern uint8_t complet, error, error_sts;
extern uint8_t inter;
volatile uint32_t ttt = 0;
extern __IO uint32_t CRCValue_nominal, CRCValue_actual_2;
Media_mode device_mode;
extern uint32_t spi_speed;
ARM_SPI_STATUS sts;

uint8_t FAT[STORAGE_BLK_SIZ * (FAT_DIRECTORY_BLK+2)] __attribute__((aligned(4))) = {

    0xEB, 0x3C, 0x90, 0x4D, 0x53, 0x44, 0x4F, 0x53, 0x35, 0x2E, 0x30, 0x00, 0x02, 0x40, 0x01, 0x00,
    0x02, 0xE0, 0x00, 0x00, 0x00, 0xF8, 0x07, 0x00, 0x11, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x1D, 0x00, 0x02, 0x00, 0x80, 0x00, 0x29, 0xFC, 0xEE, 0xCB, 0x4C, 0x53, 0x50, 0x49, 0x2D, 0x50,
    0x52, 0x4F, 0x47, 0x20, 0x20, 0x20, 0x46, 0x41, 0x54, 0x31, 0x32, 0x20, 0x20, 0x20}; 

extern const struct flashchip flashchips[];
extern const struct flashchip *flschip;

FRESULT res;
FRESULT check;
FATFS *fs; 

const unsigned char second_sec_2[4] = {0xF8, 0xFF, 0xFF, 0x00};	 //0xf0
const unsigned char second_sec_3[6] = {0xF0, 0xFF, 0xFF, 0x00, 0x40, 0x00};
		
 const unsigned char Label_disk[32] = {
    0x53, 0x50, 0x49, 0x2D, 0x50, 0x52, 0x4F, 0x47, 0x20, 0x20, 0x20, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x48, 0x53, 0xA8, 0x4E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};
 
const unsigned char backup_fil[28] = {
    0x46, 0x49, 0x52, 0x4D, 0x57, 0x41, 0x52, 0x45, 0x42, 0x49, 0x4E, 0x20, 0x18, 0x7A, 0x62, 0x7F,
    0xB6, 0x4E, 0xB6, 0x4E, 0x00, 0x00, 0x63, 0x7F, 0xB6, 0x4E, 0x02, 0x00 
};

int32_t file_size = 0;
bool fat_directory_blk_written = false;
//bool last_data_blk_written     = false;
volatile uint32_t wr_data_adr = 0;
uint32_t fat_file_data_blk = 0, fat_offset = 0;
bool file_data_blk_changed = false;

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

DWORD get_fattime(void)
{
  // use the project compilation date
  char year[5] = {BUILD_YEAR_CH0, BUILD_YEAR_CH1, BUILD_YEAR_CH2, BUILD_YEAR_CH3, 0};
  char month[3] = {BUILD_MONTH_CH0, BUILD_MONTH_CH1, 0};
  char day[3] = {BUILD_DAY_CH0, BUILD_DAY_CH1, 0};
	char hour[3] = {BUILD_HOUR_CH0, BUILD_HOUR_CH1, 0};
	char min[3] = {BUILD_MIN_CH0, BUILD_MIN_CH1, 0};
	char sec[3] = {BUILD_SEC_CH0, BUILD_SEC_CH1, 0};

  return ((atoi(year) - 1980) << 25) // Year = 2006   return	((2019UL-1980) << 25)
         | (atoi(month) << 21)       // Month = Feb     (5UL << 21)
         | (atoi(day) << 16)         // Day = 9         (9UL << 16)
         | (atoi(hour) << 11)        // Hour = 22       (22U << 11)
         | (atoi(min) << 5)          // Min = 30        (30U << 5)
         | (atoi(sec) >> 1)          // Sec = 0         (0U >> 1)
      ;
}

//start_cls 2 or 3, first data cluster *NUMBER*
int8_t create_fs(const char * fil_str, uint8_t start_cls) {   //create fat for BACKUP, INFO device modes 
	uint8_t i=0, pos, num;
	uint16_t clusters;
	FAT_BOOTSECTOR *fat_tmp;
	
	FAT12_FAT_TABLE * tbl	= (FAT12_FAT_TABLE*)&FAT[0x200+pos];	
	if (start_cls == 2) {
		pos = 3;
		num = 3;
	}
	  else {
			pos = 6;
			num = 5;
		}
	
	for (int k=0;k<NUMBER_OF_FAT_TABLES;k++) {
	i=0;
	if (start_cls == 2) memcpy(&FAT[0] + (1+k*SECTOR_PER_FAT)*STORAGE_BLK_SIZ, &second_sec_2[0], sizeof second_sec_2); 
		 else memcpy(&FAT[0] + (1+k*SECTOR_PER_FAT)*STORAGE_BLK_SIZ, &second_sec_3[0], sizeof second_sec_3); 
		
	tbl	= (FAT12_FAT_TABLE*)&FAT[0x200+pos+k*SECTOR_PER_FAT*STORAGE_BLK_SIZ];
	fat_tmp = (FAT_BOOTSECTOR *) &FAT[0];
	clusters = fat_tmp->NumberOfSectors32/fat_tmp->SectorsPerCluster;
	//clusters = (*(uint32_t*)&FAT[0x20])/(*(uint8_t*)&FAT[0x0d]);	//cluster cnt;		
	//clusters -=	0;  //-2
	while (clusters>1){			
			if (clusters == 1) {tbl->cluster0 = END_OF_CHAIN; break;}
					 else tbl->cluster0 = num+0 + i*4;
			clusters--;
			if (clusters == 1) {tbl->cluster1 = END_OF_CHAIN; break;}
					 else tbl->cluster1 = num+1 + i*4;
			clusters--;
			if (clusters == 1) {tbl->cluster2 = END_OF_CHAIN; break;}
					 else tbl->cluster2 = num+2 + i*4;
			clusters--;
			if (clusters == 1) {tbl->cluster3 = END_OF_CHAIN; break;}
					 else tbl->cluster3 = num+3 + i*4;
			clusters--;
			i++;
			tbl++;
	}
 }

#ifdef USE_LFN
  // I use FatFS for long filename
  fs = (FATFS *)malloc(sizeof(FATFS));
  FIL fil;
  char sd_path[4] = "0:/";
  char filename[FF_MAX_LFN+1] = {0}; // 46
	uint8_t count;
	
	if (device_mode == INFO) count = sizeof filename - 1;  //not need ext of file
	   else count = sizeof filename - 5;
  memcpy(filename, fil_str, (strlen(fil_str) > count) ? count : strlen(fil_str));
  while (strchr(filename, '/') != NULL)
  { // replace '/' on '_'
    *strchr(filename, '/') = '_';
  }
  if (flschip != flash_id_to_entry(GENERIC_MANUF_ID, GENERIC_DEVICE_ID) && (flschip) && device_mode != INFO)
  {
    while (strlen(filename) >= count)  //remove the rest of the truncated file name
    {
      if (strrchr(filename, '_') > strrchr(filename, ')'))
        *strrchr(filename, '_') = '\0';
      else
        *(strrchr(filename, ')') + 1) = '\0';
    }
  }
  if (device_mode != INFO) strcat(filename, ".bin");
  check = f_mount(fs, sd_path, 0);
  check |= f_open(&fil, filename, FA_CREATE_ALWAYS);  //filename leght must be <=64 chars(FF_MAX_LFN)
  memcpy(&FAT[0] + STORAGE_BLK_SIZ * FAT_DIRECTORY_BLK, fs->win, sizeof fs->win); 
  //check = f_write(&fil, "123456", 6, &bw);
  check |= f_close(&fil);
  check |= f_mount(NULL, sd_path, 0);
  free(fs);
  if (check)
    return -1;
  #if (0)	
  uint8_t *buf_temp = &FAT[STORAGE_BLK_SIZ * FAT_DIRECTORY_BLK + (sizeof fs->win -8)];
  while (*(uint32_t *)buf_temp == 0)
  { // find position for file size
    buf_temp = buf_temp - 0x10;
  }
  *(uint32_t *)(buf_temp + 4) = (flschip) ? flschip->total_size * 1024 : 0; // set  file size
  *(uint32_t *)(buf_temp) = *(uint32_t *)(buf_temp) + 0x00020000;
  #else	
	 if (set_file_size(&FAT[STORAGE_BLK_SIZ * FAT_DIRECTORY_BLK], (flschip) ? flschip->total_size * 1024 : 0))
		 return -1;
  #endif	 

  // memset(&FAT[0]+26*STORAGE_BLK_SIZ, 0, STORAGE_BLK_SIZ);
#else
  memcpy(&FAT[STORAGE_BLK_SIZ * FAT_DIRECTORY_BLK + 0x20], backup_fil, sizeof(backup_fil));
  memset(&FAT[STORAGE_BLK_SIZ * FAT_DIRECTORY_BLK + 0x20], 0x20, 8);
  memcpy(&FAT[STORAGE_BLK_SIZ * FAT_DIRECTORY_BLK + 0x20], flschip->name, strlen(flschip->name) <= 8 ? strlen(flschip->name) : 8); // copy file name & etc.
  *(uint32_t *)&FAT[STORAGE_BLK_SIZ * FAT_DIRECTORY_BLK + 28 + 0x20] = flschip->total_size * 1024;                                 // set  file size
#endif
  return 0;
}

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
    // HAL_FLASH_Unlock();
    break;
  case 1:
    return -1;
  default:
    return -1;
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
int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t *block_num,
                           uint16_t *block_size)
{
  switch (lun)
  {
  case 0:
		if (device_mode == PROG || device_mode == VERIFY) {
    *block_num = (flschip) ? flschip->total_size * 1024 / STORAGE_BLK_SIZ + FAT_FILE_DATA_BLK_LIN : 1024 * 1024 / STORAGE_BLK_SIZ + FAT_FILE_DATA_BLK_LIN; // 0x1D;   //STORAGE_BLK_NBR;
		}
		else *block_num = (flschip) ? flschip->total_size * 1024 / STORAGE_BLK_SIZ + FAT_FILE_DATA_BLK : 1024 * 1024 / STORAGE_BLK_SIZ + FAT_FILE_DATA_BLK;
    *block_size = STORAGE_BLK_SIZ;
    break;
  case 1:
    return -1;
  default:
    return -1;
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
    return -1;
  default:
    return -1;
  }
}

/**
 * @brief  Checks whether the medium is write protected.
 * @param  lun: Logical unit number
 * @retval Status (0: write enabled / -1: otherwise)
 */
int8_t STORAGE_IsWriteProtected(uint8_t lun)
{
  USBD_UsrLog("\n\r IsWriteProtected -> %d ", Wr_Protect);
  if (Wr_Protect)
    return -1;
  else
    return 0;
}

/**
 * @brief  Reads data from the medium.
 * @param  lun: Logical unit number
 * @param  blk_addr: Logical block address
 * @param  blk_len: Blocks number
 * @retval Status (0: Ok / -1: Error)
 */
int8_t STORAGE_Read(uint8_t lun, uint8_t *buf, uint32_t blk_addr,
                    uint16_t blk_len)
{
  //  int8_t ret = -1;
  uint32_t *buf32 = (uint32_t *)buf;

  switch (lun)
  {
  case 0:
    // for backup firmware mode
    if (blk_addr >= fat_file_data_blk && device_mode == BACKUP)
    {
      if (flschip)
        ReadData((blk_addr - fat_file_data_blk) * STORAGE_BLK_SIZ, buf32, STORAGE_BLK_SIZ * blk_len, true);
      else
        memset(buf, 0, STORAGE_BLK_SIZ * blk_len);
    }
    // fill zeros if blk_addr>sizeof(FAT)/STORAGE_BLK_SIZ
    else if (blk_addr >= sizeof(FAT) / STORAGE_BLK_SIZ)
      memset(buf, 0, STORAGE_BLK_SIZ * blk_len);
    else
    {
      memcpy(buf, &FAT[0] + blk_addr * STORAGE_BLK_SIZ, STORAGE_BLK_SIZ * blk_len);
    }
    USBD_UsrLog("\n\r RD blk_addr-> %d blk_len-> %d", blk_addr, blk_len);
    break;
  case 1:
    break;
  default:
    return -1;
  }
  return 0;
}

int8_t Sector_Erase(void)
{
  if (EraseChip())
    return -1;

  return 0;
}

int8_t Prepare_FAT(uint32_t size_in_kb, const char *fil_str, const char *dsk_lbl)
{
  int8_t ret = 0;
  FAT_BOOTSECTOR * boot_sct;	
	uint32_t sec_count;
	
	file_size = 0;
	wr_data_adr = 0;
	fat_file_data_blk = FAT_FILE_DATA_BLK;
	fat_offset = FAT_OFFSET;
	if (device_mode == PROG || device_mode == VERIFY)
	   sec_count = (size_in_kb * 1024 / STORAGE_BLK_SIZ) + FAT_FILE_DATA_BLK_LIN;  //FAT_FILE_DATA_BLK
	else 
		sec_count = (size_in_kb * 1024 / STORAGE_BLK_SIZ) + FAT_FILE_DATA_BLK;
	boot_sct = (FAT_BOOTSECTOR *)&FAT[0];
	if (sec_count < 0x10000) {  //only two bytes,  <=65535 sectors
		boot_sct->NumberOfSectors16 = sec_count;
	 }
	boot_sct->SectorsPerCluster = SECTORS_PER_CLUSTER;
	boot_sct->NumberOfFatTables = NUMBER_OF_FAT_TABLES;
	boot_sct->ReservedSectors   = RESVD_SEC_CNT; 
	boot_sct->NumberOfSectors32 = sec_count;	
	boot_sct->EndOfSectorMarker = 0xAA55;
	memset(&FAT[STORAGE_BLK_SIZ], 0, sizeof(FAT) - STORAGE_BLK_SIZ); //clear any other data
  memcpy(&FAT[0] + STORAGE_BLK_SIZ, &second_sec_2[0], sizeof second_sec_2);                                 //+0x0200
	if (NUMBER_OF_FAT_TABLES == 2) {
		memcpy(&FAT[0] + STORAGE_BLK_SIZ * (SECTOR_PER_FAT * 1 + 1), &second_sec_2[0], sizeof second_sec_2);      //+0x1000
	}
  memcpy(&FAT[0] + STORAGE_BLK_SIZ * FAT_DIRECTORY_BLK, (uint8_t *)&Label_disk[0], sizeof(Label_disk)); //+0x1E00

  if (dsk_lbl)
    memcpy(&FAT[0] + STORAGE_BLK_SIZ * FAT_DIRECTORY_BLK + 4, dsk_lbl, 6);
  if (device_mode == BACKUP || device_mode == INFO)
  {
    if (create_fs(fil_str, 2))
      return -1;
      // fatfs вешает spi
#ifdef USE_LFN
//			Initialize(NULL);
//			PowerControl(ARM_POWER_FULL);
//			ptrSPI->Control(ARM_SPI_SET_BUS_SPEED, spi_speed);
#endif
  }

  // if (ret) Error_Handler();

  return ret;
}

/**
 * @brief  LL Writes data into the medium.
 * @param  dest: Destination addres
 * @param  src: Pointer to source data
 * @param  len: number byte for writing
 * @retval Status (0 : Ok / -1 : Error)
 */
static int8_t Write_LL(Dst_data dst_dta, uint32_t dest, uint8_t *src, uint16_t len)
{
  int8_t stat;
  uint32_t *src32 = (uint32_t *)src;
  uint8_t *crc_buf_;

  if (dst_dta == SPI_DATA)
  {
    // if the file size is not a multiple of the sector size, last block
    if ((modulo) && ((dest - fat_offset) == (file_size - modulo)))
    {
      // len = modulo;                   //chunk
      memset(src + modulo, FLASH_ERASED_VALUE, len - modulo);
    }

    // if writing data
    if (dest < (flschip->total_size * 1024 + fat_offset))
    {
      if (device_mode == BACKUP || device_mode == INFO)
      {
        return ARM_DRIVER_OK;
      }
      else if (device_mode == VERIFY)
      {
        stat = (int8_t)VerifyData(dest - fat_offset, src32, len, false);
        if (stat != ARM_DRIVER_OK)
          return stat;
        crc_buf_ = malloc(len);
        if ((modulo) && ((dest - fat_offset) == ((file_size / STORAGE_BLK_SIZ) * STORAGE_BLK_SIZ)))
        {
          len = modulo;
          // USBD_UsrLog("\n\r len = modulo");
        }
        ReadData(dest - fat_offset, (uint32_t *)crc_buf_, len, true); // sizeof(crc_buf)
        CRCValue_actual_2 = CalcCRC32_n(crc_buf_, len, CRCValue_actual_2, CRC_BUFF_SZE);
        free(crc_buf_);
      }
      else if (device_mode == PROG)
      {
        stat = (int8_t)ProgramData(dest - fat_offset, src32, len, false);
        if (stat != ARM_DRIVER_OK)
          return stat;
        crc_buf_ = malloc(len);
        if ((modulo) && ((dest - fat_offset) == ((file_size / STORAGE_BLK_SIZ) * STORAGE_BLK_SIZ)))
        {
          len = modulo;
          // USBD_UsrLog("\n\r len = modulo");
        }
        ReadData(dest - fat_offset, (uint32_t *)crc_buf_, len, true); // sizeof(crc_buf)
        CRCValue_actual_2 = CalcCRC32_n(crc_buf_, len, CRCValue_actual_2, CRC_BUFF_SZE);
        free(crc_buf_);
      }
      else
        return -1; // unknown mode
    }
    // wrong address
    else
      return -1;
  }
  // else writing  FAT
  else if (dst_dta == FAT_DATA)
  {
    if ((dest <= ((uint32_t)&FAT[0] + sizeof(FAT))) && (dest >= ((uint32_t)&FAT[0])))
      memcpy((uint8_t *)dest, src, len);
    // wrong address
    else
      return -1;
  }
  // unknown Dst_data mode
  else
    return -1;

  return ARM_DRIVER_OK;
}

/**
 * @brief  Writes data into the medium.
 * @param  lun: Logical unit number
 * @param  blk_addr: Logical block address
 * @param  blk_len: Blocks number
 * @retval Status (0 : Ok / -1 : Error)
 */
int8_t STORAGE_Write(uint8_t lun, uint8_t *buf, uint32_t blk_addr,
                     uint16_t blk_len)
{
  uint32_t adr = 0;
  int8_t stat;

  switch (lun)
  {
  case 0:
    USBD_UsrLog("\n\r WR blk_addr-> %d blk_len-> %d", blk_addr, blk_len);
    if (!(error))
    { //
      if (blk_addr >= fat_file_data_blk)
      { 
				//determine the data block number, it may vary depending on the operating system
				if (!file_data_blk_changed) {
					switch (blk_addr)
					{
						case FAT_FILE_DATA_BLK:
							fat_file_data_blk = FAT_FILE_DATA_BLK;
						  fat_offset = FAT_OFFSET;
						  file_data_blk_changed = true;
							break;
						case FAT_FILE_DATA_BLK_LIN:
							fat_file_data_blk = FAT_FILE_DATA_BLK_LIN;
						  fat_offset = FAT_OFFSET_LIN;
						  file_data_blk_changed = true;
							break;
            default:
							fat_file_data_blk = FAT_FILE_DATA_BLK;
						  fat_offset = FAT_OFFSET;
						  file_data_blk_changed = true;
              break;
					}
					USBD_UsrLog("\n\r fat_file_data_blk -> %d", fat_file_data_blk);
				}
				// file body
        if ((modulo) && (blk_addr == (fat_file_data_blk + file_size / STORAGE_BLK_SIZ)))
        {
          // last block && not aligned
          CRCValue_nominal = CalcCRC32_n(buf, modulo, CRCValue_nominal, CRC_BUFF_SZE);
        }
        // standart
        else
          CRCValue_nominal = CalcCRC32_n(buf, blk_len * STORAGE_BLK_SIZ, CRCValue_nominal, CRC_BUFF_SZE);
      }
      if (blk_addr < fat_file_data_blk)
      {
        if (blk_addr == FAT_DIRECTORY_BLK && (device_mode == PROG || device_mode == VERIFY))
        {
					USBD_UsrLog("\n\r file_size offset 3C -> 0x%X", *(uint32_t *)(buf + 0x1C + 0x20)); 
          // 0x1C- file_size offset
          if (*(uint32_t *)(buf + 0x1C + 0x20))
          {
//            file_size = 0;
#if (0)						
            uint8_t *buf_temp = buf + 0x1C + 0xC0;
            while (file_size == 0)
            { // find file size
              file_size = *(uint32_t *)(buf_temp);
              buf_temp = buf_temp - 0x20;
            }
#else						
						file_size = find_file_size(buf);
#endif
            if (file_size)
            {
							fat_directory_blk_written = true;
							USBD_UsrLog("\n\r file_size -> 0x%X wr_data_adr -> 0x%X", file_size, wr_data_adr);
              if (file_size > flschip->total_size * 1024)
              {
                USBD_UsrLog("\n\r ERROR, file size > flash size!!!");
                return -1;
              }
              modulo = file_size % STORAGE_BLK_SIZ; // chunk of file
              if (modulo)
                mod = 1;
              else
                mod = 0;
							//for linux
							if (wr_data_adr == (file_size / STORAGE_BLK_SIZ + mod)*STORAGE_BLK_SIZ ) { //==
								USBD_UsrLog("\n\r Complet for linux");
								Wr_Protect = 1;
							  complet = 1;
								fat_directory_blk_written = false;
								wr_data_adr = 0;
								file_data_blk_changed = false;
							}
            }
          }
        }
      }
		
      // if block number >FAT_DIRECTORY_BLK & <FAT_FILE_DATA_BLK -> skip writing
      if (!(blk_addr<fat_file_data_blk & blk_addr> FAT_DIRECTORY_BLK))  //blk_addr> FAT_DIRECTORY_BLK
      {	
				if (blk_addr >= fat_file_data_blk) {
					// writing data
					USBD_UsrLog("  WR_LL wr_data_adr_-> 0x%X", (blk_addr-fat_file_data_blk)* STORAGE_BLK_SIZ);
					stat = Write_LL(SPI_DATA, blk_addr * STORAGE_BLK_SIZ, buf, blk_len * STORAGE_BLK_SIZ);
					wr_data_adr += blk_len * STORAGE_BLK_SIZ;
				}
				else {
				// writing FAT
         stat = Write_LL(FAT_DATA, (uint32_t)&FAT[0] + blk_addr * STORAGE_BLK_SIZ, buf, blk_len * STORAGE_BLK_SIZ);
				}
		
        if (stat)
        {
          error_sts = stat;
          USBD_UsrLog("\n\r ERROR -> %s %d", __FILE__, __LINE__);
          USBD_UsrLog("\n\r ERROR Write_LL -> %d", error_sts);
          // complet = 1;
          // return -1;
          // Error_Handler();
        }
      }

      if (file_size > 0)
      {
					if (wr_data_adr == (file_size / STORAGE_BLK_SIZ + mod)*STORAGE_BLK_SIZ ) { //==
				  //if (blk_addr == (fat_file_data_blk-1) + mod + file_size/STORAGE_BLK_SIZ){       //If the file is recorded fully
						    USBD_UsrLog("\n\r wr_data_adr 0x%X file_size -> 0x%X", wr_data_adr, file_size);
								if (fat_directory_blk_written) {
									USBD_UsrLog("\n\r Complet for windows");
									Wr_Protect = 1;
									complet = 1;
									fat_directory_blk_written = false;
									wr_data_adr = 0;
							    file_data_blk_changed = false;
						 }
					}
      }
    }
    else
      return -1;
    break;
  case 1:
    break;
  default:
    return -1;
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

uint32_t find_file_size(uint8_t * ptr){
	
	FAT_LONGENTRY * long_entry;
	FAT_SHORTENTRY * short_entry;
	uint32_t fls_sze;

#ifdef USE_LFN	
	long_entry = (FAT_LONGENTRY *) (ptr + 0x20);
	if (!long_entry->LFN.char0) {
		return 0;
	}
	else {
		if (FAT_DirectoryEntryType(ptr + 0x20) == FAT_ATTR_LongFileNameEntry)
	    short_entry	 = (FAT_SHORTENTRY *) (ptr + 0x20 + long_entry->LFN.LFN_RecordSeqNum.LFN_RecSeqNum * 0x20);
		else
			short_entry	 = (FAT_SHORTENTRY *) (ptr + 0x20);
		fls_sze = short_entry->FileSizeInBytes;
	}

#else
	short_entry	 = (FAT_SHORTENTRY *) (ptr + 0x20);
	fls_sze = short_entry->FileSizeInBytes;
#endif	
	
	return fls_sze;
}


int8_t set_file_size(uint8_t * ptr, uint32_t sze){
	
	FAT_LONGENTRY * long_entry;
	FAT_SHORTENTRY * short_entry;
	uint32_t fls_sze;

#ifdef USE_LFN	
	long_entry = (FAT_LONGENTRY *) (ptr + 0x20);
	if (!long_entry->LFN.char0) {
		return 1;
	}
	else {
		if (FAT_DirectoryEntryType(ptr + 0x20) == FAT_ATTR_LongFileNameEntry)
	      short_entry	 = (FAT_SHORTENTRY *) (ptr + 0x20 + long_entry->LFN.LFN_RecordSeqNum.LFN_RecSeqNum * 0x20);
		else
			  short_entry	 = (FAT_SHORTENTRY *) (ptr + 0x20);
		
		short_entry->FileSizeInBytes = sze;
		short_entry->Cluster = 2; //2 or 3
	}

#else
	short_entry	 = (FAT_SHORTENTRY *) (ptr + 0x20);
	short_entry->FileSizeInBytes = sze;
#endif	
	
	return 0;
}

//get  DirectoryEntryType
FAT_ATTR_TYPE FAT_DirectoryEntryType(uint8_t * directoryEntryStart)
{
    return *( directoryEntryStart + 11 ) & FAT_ATTR_TYPE_VALID_MASK;
}

/**********************************END OF FILE**************************************/
