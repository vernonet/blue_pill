
//*** <<< Use Configuration Wizard in Context Menu >>> ***
//   <o> SPI Flash <0=>W25X80  <1=>W25Q64 <2=>W25Q80 <3=>MX25L25645G
#define TYPE_SPI 3
//
#ifndef DRIVER_SPI_BUS_SPEED
//   <o> SPI_BUS_SPEED <18000000=>18000000 <9000000=>9000000 <4500000=>4500000 <2150000=>2150000
#define DRIVER_SPI_BUS_SPEED             18000000
#endif
//*** <<< end of configuration section >>>    ***


#if TYPE_SPI == 0	
  #define FLASH_SECTOR_COUNT      ((uint32_t)256)    /* Number of sectors */
#elif TYPE_SPI == 1
   #define FLASH_SECTOR_COUNT      ((uint32_t)256*8)    /* Number of sectors */
#elif TYPE_SPI == 2
	 #define FLASH_SECTOR_COUNT      ((uint32_t)256)    /* Number of sectors */ 
#elif TYPE_SPI == 3
	 #define FLASH_SECTOR_COUNT      ((uint32_t)8192)    /* Number of sectors */ 	 
#else  
	 #define FLASH_SECTOR_COUNT      ((uint32_t)256)    /* Number of sectors */ 	 
#endif	

#define MSC_SECTOR_SIZE         ((uint32_t)0x200)
#define FLASH_SECTOR_SIZE       ((uint32_t)0x1000)   /* Sector size: 4kB */
#define FLASH_PAGE_SIZE_        ((uint32_t)256)      /* Programming page size in bytes */
#define FLASH_PROGRAM_UNIT      ((uint32_t)1)        /* Smallest programmable unit in bytes */
#define FLASH_ERASED_VALUE      ((uint8_t)0xFF)      /* Contents of erased memory */
