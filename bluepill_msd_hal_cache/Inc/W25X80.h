/*
 * Copyright (c) 2017-2018 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * -----------------------------------------------------------------------
 *
 * $Date:        19. April 2018
 * $Revision:    V1.1
 *
 * Driver:       Driver_Flash# (default: Driver_Flash0)
 * Project:      Flash Device Description for Micron W25x80 (SPI)
 * -------------------------------------------------------------------- */


#include <stdint.h>
#include "Driver_Flash.h"
#include "Driver_SPI.h"
#include "flash_conf.h"

#define CMD_READ_DATA           (0x03U)
#define CMD_READ4B_DATA         (0x13U)
#define CMD_READ_STATUS         (0x05U)
#define CMD_WRITE_ENABLE        (0x06U)
#define CMD_WRITE_DISABLE       (0x04U)
#define CMD_PAGE_PROGRAM        (0x02U)
#define CMD_PAGE4B_PROGRAM      (0x12U)
#define CMD_READ_RDID           (0x9fU) // Read Manufacturer and JDEC Device ID 
#define CMD_READ_RDID_SST       (0x90U)
#define CMD_READ_CONF_REG       (0x15U)
#define CMD_READ_FLAG_STATUS    (0x70U)
#define CMD_SECTOR_ERASE        (0x20U)//(0xD8U)//(0xD8U)  ////////////////////////////////////////////////////////////////////////////////////////
#define CMD_SECTOR4B_ERASE      (0x21U)
#define CMD_BULK_ERASE          (0xC7U)

#define WINBOND_NEX_ID		      0xEF	/* Winbond (ex Nexcom) serial flashes */
#define WINBOND_NEX_W25X10	    0x3011
#define WINBOND_NEX_W25X20	    0x3012
#define WINBOND_NEX_W25X40	    0x3013
#define WINBOND_NEX_W25X80	    0x3014
#define WINBOND_NEX_W25X16	    0x3015
#define WINBOND_NEX_W25X32	    0x3016
#define WINBOND_NEX_W25X64	    0x3017
#define WINBOND_NEX_W25Q40_V	  0x4013	/* W25Q40BV; W25Q40BL (2.3-3.6V) */
#define WINBOND_NEX_W25Q80_V	  0x4014	/* W25Q80BV */
#define WINBOND_NEX_W25Q16_V	  0x4015	/* W25Q16CV; W25Q16DV */
#define WINBOND_NEX_W25Q32_V	  0x4016	/* W25Q32BV; W25Q32FV in SPI mode (default) */
#define WINBOND_NEX_W25Q64_V	  0x4017	/* W25Q64BV, W25Q64CV; W25Q64FV in SPI mode (default) */
#define MACRONIX_ID		0xC2	/* Macronix (MX) */
#define MACRONIX_MX25L25645G	0x2019


/* Enter 4-byte Address Mode */
#define JEDEC_ENTER_4_BYTE_ADDR_MODE	0xB7
/* Exit 4-byte Address Mode */
#define JEDEC_EXIT_4_BYTE_ADDR_MODE	0xE9
/* Write Extended Address Register */
#define JEDEC_WRITE_EXT_ADDR_REG	0xC5
/* Read Extended Address Register */
#define JEDEC_READ_EXT_ADDR_REG		0xC8

typedef struct _JEDEC_ID {
  uint8_t  man_id;            
  uint16_t dev_id; 
} JEDEC_ID;

int32_t ReadStatusReg (uint8_t cmd, uint8_t *stat);
int32_t ReadConfigReg (uint8_t cmd, uint8_t *stat);
int32_t ReadData (uint32_t addr, void *data, uint32_t cnt);
int32_t ProgramData (uint32_t addr, const void *data, uint32_t cnt);
int32_t ReadSfdpReg (uint8_t cmd, uint32_t* _size);
int32_t ReadJedecId (uint8_t cmd, JEDEC_ID* jdc_id);
//int32_t ReadIdSST ( uint8_t cmd, JEDEC_ID* jdc_id);
int32_t EraseSector (uint32_t addr);
int32_t EraseChip (void);
int32_t Initialize (ARM_Flash_SignalEvent_t cb_event);
int32_t Uninitialize (void);
int32_t PowerControl (ARM_POWER_STATE state);
ARM_SPI_STATUS Spi_status(void);
int spi_enter_4ba(void);

