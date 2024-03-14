/**
  ******************************************************************************
  * @file    USB_Device/MSC_Standalone/Inc/usbd_storage.h
  * @author  MCD Application Team
  * @version V1.6.0
  * @date    12-May-2017
  * @brief   Header for usbd_storage.c module
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_STORAGE_H_
#define __USBD_STORAGE_H_

/* Includes ------------------------------------------------------------------*/
#include "usbd_msc.h"
#include <stdlib.h>
//#include "flash_conf.h"

#define STORAGE_LUN_NBR                  1
#define STORAGE_BLK_NBR                  32797  //32768//640      //32768//    
#define STORAGE_BLK_SIZ                  0x200    
#define SECTORS_PER_CLUSTER              64U
#define SECTOR_PER_FAT                   07U
#define NUMBER_OF_FAT_TABLES	           01U
#define RESVD_SEC_CNT                    01U
#define FAT_DIRECTORY_BLK                (SECTOR_PER_FAT*NUMBER_OF_FAT_TABLES+RESVD_SEC_CNT )//15U
#define MAX_ROOT_DIR_ENTRIES             0xE0U   //boot_sec[17]  0x0E
#define ROOT_DIR_SIZE                    ((MAX_ROOT_DIR_ENTRIES*32)/STORAGE_BLK_SIZ)  //0x0EU RootDirSectors = (BPB_RootEntCnt * 32) / BPB_BytsPerSec
#define FAT_FILE_DATA_BLK                (FAT_DIRECTORY_BLK + ROOT_DIR_SIZE) //29U
#define FAT_FILE_DATA_BLK_LIN            (FAT_DIRECTORY_BLK + ROOT_DIR_SIZE + SECTORS_PER_CLUSTER)
#define FAT_OFFSET                       (FAT_FILE_DATA_BLK*STORAGE_BLK_SIZ)  //0A 0x1400   1B 3600
#define FAT_OFFSET_LIN                   (FAT_FILE_DATA_BLK_LIN*STORAGE_BLK_SIZ)
#define END_OF_CHAIN                     (0xfff)

#define FAT_ATTR_TYPE_VALID_MASK          0x3F

typedef enum {
  BACKUP   = 0,
  VERIFY   = 1,
  PROG     = 2,
	INFO     = 3,
}Media_mode;

typedef enum {
  FAT_DATA   = 0,
  SPI_DATA   = 1,
  RAM_DATA   = 2,
}Dst_data;



#define FLASH_DISK_START_ADDRESS              0                  
#define FIRMWARE_SIZE
#define FLASH_DISK_SIZE                       STORAGE_BLK_NBR*STORAGE_BLK_SIZ  //0x50000         
//#define FLASH_PAGE_SIZE                     0x200                            /* 2K per page */
#define WAIT_TIMEOUT                		      100000 
#define FLASH_TIMEOUT                         50000U                           /* 50 s */


// __DATE__  __DATE__  __DATE__  __DATE__  __DATE__  __DATE__  __DATE__  __DATE__  __DATE__
// Example of __DATE__ string: "Jul 27 2012"
//                              01234567890

#define BUILD_YEAR_CH0 (__DATE__[ 7])
#define BUILD_YEAR_CH1 (__DATE__[ 8])
#define BUILD_YEAR_CH2 (__DATE__[ 9])
#define BUILD_YEAR_CH3 (__DATE__[10])


#define BUILD_MONTH_IS_JAN (__DATE__[0] == 'J' && __DATE__[1] == 'a' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_FEB (__DATE__[0] == 'F')
#define BUILD_MONTH_IS_MAR (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'r')
#define BUILD_MONTH_IS_APR (__DATE__[0] == 'A' && __DATE__[1] == 'p')
#define BUILD_MONTH_IS_MAY (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'y')
#define BUILD_MONTH_IS_JUN (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_JUL (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'l')
#define BUILD_MONTH_IS_AUG (__DATE__[0] == 'A' && __DATE__[1] == 'u')
#define BUILD_MONTH_IS_SEP (__DATE__[0] == 'S')
#define BUILD_MONTH_IS_OCT (__DATE__[0] == 'O')
#define BUILD_MONTH_IS_NOV (__DATE__[0] == 'N')
#define BUILD_MONTH_IS_DEC (__DATE__[0] == 'D')


#define BUILD_MONTH_CH0 \
    ((BUILD_MONTH_IS_OCT || BUILD_MONTH_IS_NOV || BUILD_MONTH_IS_DEC) ? '1' : '0')

#define BUILD_MONTH_CH1 \
    ( \
        (BUILD_MONTH_IS_JAN) ? '1' : \
        (BUILD_MONTH_IS_FEB) ? '2' : \
        (BUILD_MONTH_IS_MAR) ? '3' : \
        (BUILD_MONTH_IS_APR) ? '4' : \
        (BUILD_MONTH_IS_MAY) ? '5' : \
        (BUILD_MONTH_IS_JUN) ? '6' : \
        (BUILD_MONTH_IS_JUL) ? '7' : \
        (BUILD_MONTH_IS_AUG) ? '8' : \
        (BUILD_MONTH_IS_SEP) ? '9' : \
        (BUILD_MONTH_IS_OCT) ? '0' : \
        (BUILD_MONTH_IS_NOV) ? '1' : \
        (BUILD_MONTH_IS_DEC) ? '2' : \
        /* error default */    '?' \
    )

#define BUILD_DAY_CH0 ((__DATE__[4] >= '0') ? (__DATE__[4]) : '0')
#define BUILD_DAY_CH1 (__DATE__[ 5])

#define BUILD_HOUR_CH0 (__TIME__[0])
#define BUILD_HOUR_CH1 (__TIME__[1])

#define BUILD_MIN_CH0 (__TIME__[3])
#define BUILD_MIN_CH1 (__TIME__[4])

#define BUILD_SEC_CH0 (__TIME__[6])
#define BUILD_SEC_CH1 (__TIME__[7])

// __DATE__  __DATE__  __DATE__  __DATE__  __DATE__  __DATE__  __DATE__  __DATE__  __DATE__


typedef  struct FAT12_FAT_TABLE {  //struct size  6 bytes for 4 clusters
	uint32_t cluster0:12;
	uint32_t cluster1:12;
	uint32_t cluster2:12;
	uint32_t cluster3:12;
} __attribute__((packed)) FAT12_FAT_TABLE;


// Media descriptor
typedef enum uint8_t
{
    FLOPPY                        = 0xf0,
    HARD_DRIVE                    = 0xf8,
    FLOPPY_320K_1                 = 0xfa,
    FLOPPY_640K                   = 0xfb,
    FLOPPY_180K                   = 0xfc,
    FLOPPY_360K                   = 0xfd,
    FLOPPY_160K                   = 0xfe,
    FLOPPY_320K_2                 = 0xff,
} MEDIA ;

typedef struct  FAT_BOOTSECTOR{
    uint8_t   jmp[3];                
    char      OemName[8];
    uint16_t  BytesPerSector;        // legal == { 512, 1024, 2048, 4096 }
    uint8_t   SectorsPerCluster;     // legal == { 1, 2, 4, 8, 16, 32, 64, 128 }
    uint16_t  ReservedSectors;       // must not be zero; legal for FAT12/16 == { 1 }, typically 32 for FAT32
    uint8_t   NumberOfFatTables;     // must not be zero; warn if this is not set to the value 1 or 2
    uint16_t  MaxRootDirEntries;     // legal for FAT12/16 == N * (BytesPerSector / 32), N is non-zero; must be {0} for FAT32
    uint16_t  NumberOfSectors16;     // must be {0} for FAT32; if {0}, then NumberOfSectors32 must be non-zero
    MEDIA     MediaDescriptor;       // legacy
    uint16_t  SectorsPerFat16;       // must be {0} for FAT32; must be non-zero for FAT12/16
    uint16_t  SectorsPerTrack;       // legacy
    uint16_t  HeadsPerCylinder;      // legacy
    uint32_t  NumHiddenSectors;      // legacy
    uint32_t  NumberOfSectors32;     // must be non-zero for FAT32; must be >= 0x10000 if NumberOfSectors16 is zero
	  uint8_t   DriveNumber;
    uint8_t   Unused;
    uint8_t   ExtBootSignature;
    uint32_t  SerialNumber;          // only valid if ExtBootSignature == 0x29
    char      VolumeLabel[11];       // only valid if ExtBootSignature == 0x29
    char      FileSystemLabel[8];
	  uint8_t   BootCode[448];         // 420 for FAT32, 448 for FAT16/12
    uint16_t  EndOfSectorMarker;     //0xAA55
	} __attribute__((packed)) FAT_BOOTSECTOR;

	
// FAT Directory Attribute
// Therefore, every usage will require the bitfield annotation (' : 6').
typedef enum 
{
    FAT_ATTR_LongFileNameEntry        = 0x0F, // special case
    // bits 6 & 7 are reserved
    FAT_ATTR_NoneOrFile               = 0x00,
    FAT_ATTR_ReadOnly                 = 0x01, // bit0
    FAT_ATTR_Hidden                   = 0x02, // bit1
    FAT_ATTR_System                   = 0x04, // bit2
    FAT_ATTR_VolumeId                 = 0x08, // bit3
    FAT_ATTR_Directory                = 0x10, // bit4
    FAT_ATTR_Archive                  = 0x20, // bit5
    // Common valid combinations of above
    FAT_ATTR_ReadOnly_Hidden                          = 0x03,
    FAT_ATTR_ReadOnly_System                          = 0x05,
    FAT_ATTR_Hidden_System                            = 0x06,
    FAT_ATTR_ReadOnly_Hidden_System                   = 0x07,
    // VolumeID is set only alone, or in FAT_ATTR_LongFileNameEntry
    // as above, + _Directory
    FAT_ATTR_ReadOnly_Hidden_Directory                = 0x13,
    FAT_ATTR_ReadOnly_System_Directory                = 0x15,
    FAT_ATTR_Hidden_System_Directory                  = 0x16,
    FAT_ATTR_ReadOnly_Hidden_System_Directory         = 0x17,
    // as above, + _Archive
    FAT_ATTR_ReadOnly_Hidden_Archive                  = 0x23,
    FAT_ATTR_ReadOnly_System_Archive                  = 0x25,
    FAT_ATTR_Hidden_System_Archive                    = 0x26,
    FAT_ATTR_ReadOnly_Hidden_System_Archive           = 0x27,
    FAT_ATTR_ReadOnly_Hidden_Directory_Archive        = 0x33,
    FAT_ATTR_ReadOnly_System_Directory_Archive        = 0x35,
    FAT_ATTR_Hidden_System_Directory_Archive          = 0x36,
    FAT_ATTR_ReadOnly_Hidden_System_Directory_Archive = 0x37,
} FAT_ATTR_TYPE;	
	
typedef struct
{
    uint8_t   LFN_RecSeqNum   : 6; // bit0-5 LFN sequence number (1..63)
    uint8_t   Last_LFN_record : 1; // bit6   Last LFN record in the sequence
    uint8_t   LFN_Erased      : 1; // bit7   LFN record is an erased long name entry or maybe if it is part of an erased long name?
} __attribute__((packed)) tLFN_RecordSeqNum;



// FAT Long directory entry
typedef union ulfn
    {
        tLFN_RecordSeqNum LFN_RecordSeqNum; // LFN Record Sequence Number
        unsigned char char0;
    } ULFN;
		
typedef struct
{
    ULFN    LFN;
    wchar_t UnicodeChar1[5];      // 5 UNICODE characters, LFN first part.
    FAT_ATTR_TYPE Attribute : 6;  // This field contains the special value of 0Fh, which indicates an LFN entry.
    uint8_t   Reserved;
    uint8_t   ChkShortName; // Checksum of short name entry, used to validate that the LFN entry belongs to the short name entry following. According to Ralf Brown's interrupt list, the checksum is computed by adding up all the short name characters and rotating the intermediate value right by one bit position before adding each character.
    wchar_t UnicodeChar2[6];      // 6 UNICODE characters, LFN 2nd part.
    uint16_t  Cluster; // Initial cluster number, which is always zero for LFN entries.
    wchar_t UnicodeChar3[2];      // 2 UNICODE characters, LFN 3rd part.
} FAT_LONGENTRY;	


// FAT Short directory entry
typedef struct // Normal-Short structure
{
    char    Name[8]; // Blank-padded name
    char    Extension[3]; //Blank-padded extension
    FAT_ATTR_TYPE Attribute : 6; // only low six bits are valid
    uint8_t   Reserved;
    uint8_t   CreateTime10ms; //10-ms units "Create Time" refinement
    uint16_t  CreateTime;
    uint16_t  CreateDate;
    uint16_t  AccessDate;
    uint16_t  HighCluster; // Used on FAT32 only
    uint16_t  UpdateTime;
    uint16_t  UpdateDate;
    uint16_t  Cluster; // first cluster *NUMBER*
    uint32_t  FileSizeInBytes; // file size in bytes (always zero for directories).
} FAT_SHORTENTRY;

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int8_t STORAGE_Init(uint8_t lun);
int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t * block_num, uint16_t * block_size);
int8_t STORAGE_IsReady(uint8_t lun);
int8_t STORAGE_IsWriteProtected(uint8_t lun);
int8_t STORAGE_Read (uint8_t lun, uint8_t * buf, uint32_t blk_addr, uint16_t blk_len);
int8_t STORAGE_Write(uint8_t lun, uint8_t * buf, uint32_t blk_addr, uint16_t blk_len);
int8_t STORAGE_GetMaxLun(void);
int8_t Prepare_FAT(uint32_t size_in_kb, const char *fil_str, const char *dsk_lbl);
int8_t create_fs(const char * fil_str, uint8_t start_cls);
uint32_t find_file_size(uint8_t * ptr);
int8_t set_file_size(uint8_t * ptr, uint32_t sze);
FAT_ATTR_TYPE FAT_DirectoryEntryType(uint8_t * directoryEntryStart);

extern USBD_StorageTypeDef  USBD_DISK_fops;

#endif /* __USBD_STORAGE_H_ */
 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
