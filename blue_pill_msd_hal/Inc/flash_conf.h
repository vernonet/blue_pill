
//*** <<< Use Configuration Wizard in Context Menu >>> ***
//   <o> SPI Flash <0=>W25X80  <1=>W25Q64
#define TYPE_SPI 0
//*** <<< end of configuration section >>>    ***

#if TYPE_SPI == 0	
  #define FLASH_SECTOR_COUNT      ((uint32_t)256)    /* Number of sectors */
#else
  #if TYPE_SPI == 1
   #define FLASH_SECTOR_COUNT      ((uint32_t)256*8)    /* Number of sectors */
	#else  
	 #define FLASH_SECTOR_COUNT      ((uint32_t)256)    /* Number of sectors */
	#endif 
#endif	

#define FLASH_SECTOR_SIZE       ((uint32_t)0x1000)   /* Sector size: 4kB */
#define FLASH_PAGE_SIZE_        ((uint32_t)256)      /* Programming page size in bytes */
#define FLASH_PROGRAM_UNIT      ((uint32_t)1)        /* Smallest programmable unit in bytes */
#define FLASH_ERASED_VALUE      ((uint8_t)0xFF)      /* Contents of erased memory */
