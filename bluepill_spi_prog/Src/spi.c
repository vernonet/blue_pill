/* Note:
   
*/

#include "Driver_SPI.h"
#include "string.h"
#include "spi.h"
#include "stm32f1xx_hal.h"
#include "usbd_conf.h"

#include "flashchips.h"

extern const struct flashchip * flschip;
extern uint16_t page_sze;
bool in_4ba_mode = false;
int address_high_byte = 0;
bool displ_blk_prot = true;  //display block protections message


static uint32_t address_to_bits(uint32_t addr);


void delay_mic(void)
{
	volatile uint32_t mic = 100;
	 while(mic-->0){};
}

//id - UID of chip, if not use -> id=0U
//attention, the buffer size affects the result
//for the calculation to match with Windows utilities, the buffer size must be 0x200 
uint32_t CalcCRC32(uint8_t *Buf, uint32_t Len, uint32_t id)   //calc CRC hardware
{
        unsigned int i;
        unsigned int Temp;

        __HAL_RCC_CRC_CLK_ENABLE();                  //Разрешить тактирование CRC-юнита

        CRC->CR = 1;
        __asm("nop");                                //Аппаратная готовность за 4 такта, жду...
        __asm("nop");
        __asm("nop");

        // Аппаратный CRC-расчёт работает с 32-битными словами. Т.е. сразу по 4 байта из входной последовательности
        i = Len >> 2;
        while(i--)
        {
                Temp = *((uint32_t*)Buf);
                //Temp = revbit(Temp);            //Переставить биты во входных данных
					      if (id) Temp = Temp ^ id;
					      Temp = __RBIT(Temp);
                CRC->DR = Temp;

                Buf += 4;
        }
        Temp = CRC->DR;
        //Temp = revbit(Temp);                    //Переставить биты в выходных данных
				Temp = __RBIT(Temp);
        
        // Обработать оставшиеся байты (классическим не аппаратным методом), если их число не было кратно 4
        i = Len & 3;
        while(i--)
        {
                Temp ^= (uint32_t)*Buf++;
                
                for(int j=0; j<8; j++)
                        if (Temp & 1)
                                Temp = (Temp >> 1) ^ 0xEDB88320;
                        else
                                Temp >>= 1;
        }

        Temp ^= 0xFFFFFFFFul;
        return Temp;
}


//attention, the buffer size affects the result
//The buffer size for calculating CRCValue_actual and CRCValue_nominal must be the same
uint32_t CalcCRC32_n(uint8_t *Buf, uint32_t Len, uint32_t id, uint32_t buff_sze) 
{
	uint32_t adr = 0, len = 0;
	__IO uint32_t CRCVal = id;
	
	 while (adr < Len)
            {
              if (Len - adr > buff_sze)
                len = buff_sze;
              else
                len = Len - adr;
              CRCVal = CalcCRC32(Buf, len, CRCVal);
              adr += len;
							Buf += len;
            }
						
	return CRCVal;
}

/* Send cmd to chip */
int32_t SendCmd (uint8_t * cmd, uint8_t bytes) {
  int32_t status;
 
  /* Select slave */
  status = ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);

  if (status == ARM_DRIVER_OK) {
   
    status = ptrSPI->Send(cmd, bytes);

    if (status == ARM_DRIVER_OK) {
      while (ptrSPI->GetDataCount() != bytes){
				__asm("nop");
			};
    }
  }
  /* Deselect slave */
  ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
	
	//delay_mic();

  return (status);
}

/* Send command with optional data and wait until busy */
int32_t SendCommand_at45 (uint8_t cmd, uint32_t addr, const uint8_t *data, uint32_t size) {
  uint32_t page_addr;
  uint32_t page_offs;
  uint8_t  buf[4];
  uint8_t  sr;
  int32_t  result, addr_;

	addr_ = at45db_convert_addr(addr, page_sze);  //FLASH_PAGE_SIZE_

  /* Prepare Command with address */
  buf[0] = cmd;
  buf[1] = (uint8_t)(addr_ >> 16);
  buf[2] = (uint8_t)(addr_ >>  8);
  buf[3] = (uint8_t)(addr_ >>  0);

  /* Select Slave */
  result = ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
  if (result != ARM_DRIVER_OK) return result;

  /* Send Command with address */
  result = ptrSPI->Send(buf, 4);
  if (result != ARM_DRIVER_OK) goto transfer_error;
  while (ptrSPI->GetDataCount() != 4){
				  __asm("nop");
			    }

  /* Send Data */
  if ((data != NULL) && (size != 0)) {
    result = ptrSPI->Send(data, size);
    if (result != ARM_DRIVER_OK) goto transfer_error;
    while (ptrSPI->GetDataCount() != size){
				  __asm("nop");
			    };
  }

  /* Deselect Slave */
  result = ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
  if (result != ARM_DRIVER_OK) return result;

  /* Prepare Read Status Command */
  buf[0] = AT45DB_STATUS;
  buf[1] = 0xFF;                /* Dummy byte */

  /* Select Slave */
  result = ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
  if (result != ARM_DRIVER_OK) return result;

  /* Send Command */
  result = ptrSPI->Send(buf, 2);
  if (result != ARM_DRIVER_OK) goto transfer_error;
  while (ptrSPI->GetDataCount() != 2){
				  __asm("nop");
			    };

  /* Check Status Register */
  do {
    result = ptrSPI->Receive(&sr, 1);
    if (result != ARM_DRIVER_OK) goto transfer_error;
    while (ptrSPI->GetDataCount() != 1){
				  __asm("nop");
			    };
  } while ((sr & 0x80) == 0);

  /* Deselect Slave */
  result = ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
  if (result != ARM_DRIVER_OK) return result;

  return ARM_DRIVER_OK;

transfer_error:
  ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
  return ARM_DRIVER_ERROR;
}

/* Read status or flag status register */
int32_t ReadStatusReg (uint8_t cmd, uint8_t *stat) {
  int32_t status; /* driver execution status */
  uint8_t buf[4];

  /* Select Slave */
  status = ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);

  if (status == ARM_DRIVER_OK) {
    /* Set command */
    buf[0] = cmd;

    /* Send command and receive register value */
    status = ptrSPI->Transfer (&buf[0], &buf[2], 2U);

    if (status == ARM_DRIVER_OK) {
      /* Wait till transfer done */
      while (ptrSPI->GetDataCount() != 2U) {
				__asm("nop");
			};

      *stat = buf[3];
    }
  }
  /* Deselect Slave */
  ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
  
  return (status);
}

/* Read  flag config register */
int32_t ReadConfigReg (uint8_t cmd, uint8_t *stat) {
  int32_t status; /* driver execution status */
  uint8_t buf[4];

  /* Select Slave */
  status = ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);

  if (status == ARM_DRIVER_OK) {
    /* Set command */
    buf[0] = cmd;

    /* Send command and receive register value */
    status = ptrSPI->Transfer (&buf[0], &buf[2], 2U);

    if (status == ARM_DRIVER_OK) {
      /* Wait till transfer done */
      while (ptrSPI->GetDataCount() != 2U) {
				__asm("nop");
			};

      *stat = buf[3];
    }
  }
  /* Deselect Slave */
  ptrSPI->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
  
  return (status);
}

/* Write status or flag status register */
static int32_t WriteStatusReg (uint8_t cmd, uint8_t sr) {
  int32_t status; /* driver execution status */
  uint8_t buf[4] = {0};
	int32_t feature_bits = flschip->feature_bits;
	
	if (!(feature_bits & (FEATURE_WRSR_WREN | FEATURE_WRSR_EWSR))) {
		SPI_UsrLog("Missing status register write definition, assuming "
			 "EWSR is needed\n");

		feature_bits |= FEATURE_WRSR_EWSR;
	}
	
	if (feature_bits & FEATURE_WRSR_WREN) {
		SetWriteEnable();
	}
	
	if (feature_bits & FEATURE_WRSR_EWSR) {
	
	 /* Set command */		
		buf[0] = CMD_EWSR;	
		
		status = SendCmd(buf, 1U);
			
		if (status) return status;
	}

    /* Set command */
    buf[0] = cmd;
	  buf[1] = sr;

    status = SendCmd(buf, 2U);
  
  return (status);
}


/* Set "Write enable latch" bit in status register */
int32_t SetWriteEnable (void) {
  int32_t status;
  uint8_t val;
	
	status = ReadStatusReg(CMD_READ_STATUS, &val);
	
	if (status == ARM_DRIVER_OK) {
      /* Check if "Write enable latch" bit set */
      if (val & 0x02U) {
        return ARM_DRIVER_OK;  //no need Write enable latch
      }
    }
	
	/* Set command */	
	val = CMD_WRITE_ENABLE;
 
  status = SendCmd(&val, 1U);

  if (status == ARM_DRIVER_OK) {
    /* Read status */
    val = 0U;

    status = ReadStatusReg(CMD_READ_STATUS, &val);

    if (status == ARM_DRIVER_OK) {
      /* Check if "Write enable latch" bit set */
      if ((val & 0x02U) == 0x00U) {
        status = ARM_DRIVER_ERROR;
      }
    }
  }

  return (status);
}

/* Reset "Write enable latch" bit in status register */
int32_t SetWriteDisable (void) {
  int32_t status;
  uint8_t val;
 
	val = CMD_WRITE_DISABLE;
	
  status = SendCmd(&val, 1U);

  if (status == ARM_DRIVER_OK) {
    /* Read status */
    val = 0U;

    status = ReadStatusReg(CMD_READ_STATUS, &val);

    if (status == ARM_DRIVER_OK) {
      /* Check if "Write enable latch" bit set */
      if ((val & 0x02U) != 0x00U) {
        status = ARM_DRIVER_ERROR;
      }
    }
  }

  return (status);
}


/* A generic block protection disable.
 * Tests if a protection is enabled with the block protection mask (bp_mask) and returns success otherwise.
 * Tests if the register bits are locked with the lock_mask (lock_mask).
 * Tests if a hardware protection is active (i.e. low pin/high bit value) with the write protection mask
 * (wp_mask) and bails out in that case.
 * If there are register lock bits set we try to disable them by unsetting those bits of the previous register
 * contents that are set in the lock_mask. We then check if removing the lock bits has worked and continue as if
 * they never had been engaged:
 * If the lock bits are out of the way try to disable engaged protections.
 * To support uncommon global unprotects (e.g. on most AT2[56]xx1(A)) unprotect_mask can be used to force
 * bits to 0 additionally to those set in bp_mask and lock_mask. Only bits set in unprotect_mask are potentially
 * preserved when doing the final unprotect.
 *
 * To sum up:
 * bp_mask: set those bits that correspond to the bits in the status register that indicate an active protection
 *          (which should be unset after this function returns).
 * lock_mask: set the bits that correspond to the bits that lock changing the bits above.
 * wp_mask: set the bits that correspond to bits indicating non-software removable protections.
 * unprotect_mask: set the bits that should be preserved if possible when unprotecting.
 */
static int32_t spi_disable_blockprotect_generic(uint8_t bp_mask, uint8_t lock_mask, uint8_t wp_mask, uint8_t unprotect_mask)
{
	uint8_t status, stat;
	

	status = ReadStatusReg(CMD_READ_STATUS, &stat);
	if (stat == 0xFF)     //chip not present
	{
		SPI_UsrLog("\ndisable_blockprotect: StatusReg -> 0xFF, maybe chip disconected");
		return ARM_DRIVER_ERROR_STS_REG_FF; 
	}
	if (status != ARM_DRIVER_OK) return status;
	if ((stat & bp_mask) == 0) {
		if (displ_blk_prot) SPI_UsrLog("\nBlock protection is disabled.");
		return ARM_DRIVER_OK;
	}

	SPI_UsrLog("Some block protection in effect, disabling... ");
	if ((stat & lock_mask) != 0) {
		SPI_UsrLog("\n\tNeed to disable the register lock first... ");
		if (wp_mask != 0 && (stat & wp_mask) == 0) {
			SPI_UsrLog("Hardware protection is active, disabling write protection is impossible.\n");
			return 1;
		}
		/* All bits except the register lock bit (often called SPRL, SRWD, WPEN) are readonly. */
		status = WriteStatusReg(CMD_WRSR, stat & ~lock_mask);
		if (status) {
			SPI_UsrLog("spi_write_status_register failed.\n");
			return status;
		}
		status = ReadStatusReg(CMD_READ_STATUS, &stat);
		if (status != ARM_DRIVER_OK) return status;
		if ((stat & lock_mask) != 0) {
			SPI_UsrLog("Unsetting lock bit(s) failed.\n");
			return 1;
		}
		SPI_UsrLog("done.\n");
	}
	/* Global unprotect. Make sure to mask the register lock bit as well. */
	status = WriteStatusReg (CMD_WRSR, stat & ~(bp_mask | lock_mask) & unprotect_mask);
	if (status) {
		SPI_UsrLog("write_status_register failed.\n");
		return status;
	}
	/* WRSR performs a self-timed erase before the changes take effect.
	 * This may take 50-85 ms in most cases, and some chips apparently
	 * allow running RDSR only once. Therefore pick an initial delay of
	 * 100 ms, then wait in 10 ms steps until a total of 5 s have elapsed.
	 */
	HAL_Delay(100);   
	status = ReadStatusReg(CMD_READ_STATUS, &stat);
	if (status != ARM_DRIVER_OK) return status;
	if ((stat & bp_mask) != 0) {
		SPI_UsrLog("Block protection could not be disabled!\n");
		//flash->chip->printlock(flash);
		return ARM_DRIVER_ERROR_BLK_PROT;
	}
	SPI_UsrLog("disabled.\n");
	return ARM_DRIVER_OK;
}

/* A common block protection disable that tries to unset the status register bits masked by 0x3C. */
int32_t spi_disable_blockprotect(void)
{
	return spi_disable_blockprotect_generic( 0x3C, 0, 0, 0xFF);
}



 /* Some Atmel DataFlash chips support per sector protection bits and the write protection bits in the status
 * register do indicate if none, some or all sectors are protected. It is possible to globally (un)lock all
 * sectors at once by writing 0 not only the protection bits (2 and 3) but also completely unrelated bits (4 and
 * 5) which normally are not touched.
 * Affected are all known Atmel chips matched by AT2[56]D[FLQ]..1A? but the AT26DF041. */
int32_t spi_disable_blockprotect_at2x_global_unprotect(void)
{
	return spi_disable_blockprotect_generic(0x0C, 1 << 7, 1 << 4, 0x00);
}

int32_t spi_disable_blockprotect_at25f512a(void)
{
	return spi_disable_blockprotect_generic(0x04, 1 << 7, 0, 0xFF);
}

/* A common block protection disable that tries to unset the status register bits masked by 0x3C (BP0-3) and
 * protected/locked by bit #7. */
int32_t spi_disable_blockprotect_bp3_srwd(void)
{
	return spi_disable_blockprotect_generic(0x3C, 1 << 7, 0, 0xFF);
}

/* A common block protection disable that tries to unset the status register bits masked by 0x7C (BP0-4) and
 * protected/locked by bit #7. */
int32_t spi_disable_blockprotect_bp4_srwd(void)
{
	return spi_disable_blockprotect_generic(0x7C, 1 << 7, 0, 0xFF);
}

int32_t spi_disable_blockprotect_at25f512b(void)
{
	return spi_disable_blockprotect_generic(0x04, 1 << 7, 1 << 4, 0xFF);
}

/* A common block protection disable that tries to unset the status register bits masked by 0x0C (BP0-1) and
 * protected/locked by bit #7. Useful when bits 4-5 may be non-0). */
int32_t spi_disable_blockprotect_bp1_srwd(void)
{
	return spi_disable_blockprotect_generic(0x0C, 1 << 7, 0, 0xFF);
}

/* A common block protection disable that tries to unset the status register bits masked by 0x1C (BP0-2) and
 * protected/locked by bit #7. Useful when bit #5 is neither a protection bit nor reserved (and hence possibly
 * non-0). */
int32_t spi_disable_blockprotect_bp2_srwd(void)
{
	return spi_disable_blockprotect_generic(0x1C, 1 << 7, 0, 0xFF);
}

/* === Intel/Numonyx/Micron - Spansion === */

int32_t spi_disable_blockprotect_n25q(void)
{
	return spi_disable_blockprotect_generic(0x5C, 1 << 7, 0, 0xFF);
}

int32_t spi_disable_blockprotect_at25f(void)
{
	return spi_disable_blockprotect_generic(0x0C, 1 << 7, 0, 0xFF);
}

int32_t spi_disable_blockprotect_at25fs010(void)
{
	return spi_disable_blockprotect_generic(0x6C, 1 << 7, 0, 0xFF);
 }

int32_t spi_disable_blockprotect_at25fs040(void)
{
	return spi_disable_blockprotect_generic(0x7C, 1 << 7, 0, 0xFF);
}

int spi_disable_blockprotect_at45db(void)
{

	int ret;
	uint8_t status;
	
	ret = ReadStatusReg(AT45DB_STATUS, &status);
	if (status == 0xFF)     //chip not present
	{
		SPI_UsrLog("\ndisable_blockprotect: StatusReg -> 0xFF, maybe chip disconected");
		return ARM_DRIVER_ERROR_STS_REG_FF; 
	}
	static const uint8_t cmd[4] = { AT45DB_DISABLE_PROTECT }; /* NB: 4 bytes magic number */;
	ret = SendCmd ((uint8_t *)&cmd[0], 4);
	if (ret != 0) {
		SPI_UsrLog("Sending disable lockdown failed!\n");
		return ret;
	}
	
	ret = ReadStatusReg(AT45DB_STATUS, &status);
	if (ret != 0 || ((status & AT45DB_PROT) != 0)) {
		SPI_UsrLog("Disabling lockdown failed!\n");
		return ARM_DRIVER_ERROR_BLK_PROT;
	}

	return ARM_DRIVER_OK;
}

static int spi_write_extended_address_register(const uint8_t regdata)
{
	int32_t status;
	uint8_t cmd[2] = {JEDEC_WRITE_EXT_ADDR_REG, 0};
	
		/* Enable data write */
    status = SetWriteEnable();
			
		status = SendCmd(cmd, 2U);

	if (status)
		SPI_UsrLog("%s :failed during command execution\n", __func__);
	return status;
}
	
static int spi_set_extended_address(const uint8_t addr_high)
{
	if (address_high_byte != addr_high &&
	    spi_write_extended_address_register(addr_high))
		return ARM_DRIVER_ERROR;
	address_high_byte = addr_high;
	return ARM_DRIVER_OK;
}
	
static int spi_enter_exit_4ba(const bool enter)
{
	uint8_t cmd = enter ? JEDEC_ENTER_4_BYTE_ADDR_MODE : JEDEC_EXIT_4_BYTE_ADDR_MODE;
	//int ret = 1;
	int32_t status;

	if (flschip->feature_bits & FEATURE_4BA_ENTER)  {
			
		status = SendCmd(&cmd, 1U);
	}
	else if (flschip->feature_bits & FEATURE_4BA_ENTER_WREN) {
		/* Enable data write */
    status = SetWriteEnable();
		
		if (status) return ARM_DRIVER_ERROR;
		
		status = SendCmd(&cmd, 1U);
	}
	else if (flschip->feature_bits & FEATURE_4BA_ENTER_EAR7) status = spi_set_extended_address(enter ? 0x80 : 0x00);

	return status;
}

int spi_enter_4ba(void)
{
	int32_t status;
	uint8_t buf = 0xff;
	
	status = spi_enter_exit_4ba(true);
	if (!status) status = ReadConfigReg(CMD_READ_CONF_REG, &buf); 
	if (status != ARM_DRIVER_OK) return status;    
	/* Check Flags Config register value */
	if (buf & (1<<5)) {
		SPI_UsrLog("4BA mode enter OK!\n");	
		in_4ba_mode = true;
	}
	 else return ARM_DRIVER_ERROR;
			
	return status;
}

int spi_exit_4ba(void)
{
	int32_t status;
	uint8_t buf = 0xff;
	
	status = spi_enter_exit_4ba(false);
	if (!status) status = ReadConfigReg(CMD_READ_CONF_REG, &buf); 
	if (status != ARM_DRIVER_OK) return status;    
	/* Check Flags Config register value */
	if (!(buf & (1<<5))) {
		SPI_UsrLog("4BA mode exit OK!\n");
    in_4ba_mode = false;		
	}
	 else return ARM_DRIVER_ERROR;
					
	return status;
}

/* Returns the minimum number of bits needed to represent the given address.
 * FIXME: use mind-blowing implementation. */
static uint32_t address_to_bits(uint32_t addr)
{
	unsigned int lzb = 0;
	while (((1 << (31 - lzb)) & ~addr) != 0)
		lzb++;
	return 32 - lzb;
}


unsigned int at45db_convert_addr(unsigned int addr, unsigned int page_size)
{
	unsigned int page_bits = address_to_bits(page_size - 1);
	unsigned int at45db_addr = ((addr / page_size) << page_bits) | (addr % page_size);
	
	return at45db_addr;
}
/************************************************END************************************************/
