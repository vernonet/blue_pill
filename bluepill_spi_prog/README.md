# Blue Pill SPI Programmer

It allows you to write different SPI flash memory by drag and drop the file in USB Mass Storage.
Created using Cube MX and Keil, also used source code <a href="https://github.com/flashrom/flashrom" rel="nofollow">Flashrom</a>.
Tested with chips STM32F103C8, STM32F103CB, CKS32F103C8.

# Features

Hardware full-duplex SPI with DMA, multiple clock speeds available (SPI1) :

  - 2250  KHz
  - 4500  KHz
  - 9000  KHz
  - 18000 KHz

 Verified flash memory chips :
 

  - AT26DF041, SST25LF040A, EN25F80, W25Q80, W25Q64, W25X80, W25X40, SST25VF512A, AT25F512A, M25P10-A, Pm25LD010(C), Pm25LV512(A), W25Q16, AT25F1024(A), MX25L6406E, NX25B40,  cFeon Q32b-104Hip, W25Q128BV, MX25L25645G, LE25FU206, LE25FW406A, LE25FU406B, M25P40, MX25L4005AM, MX25L8006E, LE25U20A, W25X20, SST25VF512, MX25L8005, AT45DB081D(256 bytes page), BY25Q32BS, MX25L1005(C), SST25WF040B, PUYA P25D40H, FM25F01, MX25U4035 (chips of the same series but of a different size will also work).

    Any other chips need to be added to files "flashchips.c", "chip_drv.c", "spi.c".

Pins are used to connect the chip to the board:
 - SPI2_MISO Pin - PA6
 - SPI2_MOSI Pin - PA7
 - SPI2_SCK  Pin - PA5
 - SPI2_NSS  Pin - PA4
 
 This can be changed in the file "RTE_Device.h".
![Schematic diagram](https://github.com/vernonet/blue_pill/edit/master/bluepill_spi_prog/schematic_diagram.jpg)


To display debug information set in file "usbd_conf.h" (debugging is output to the SWO port, pin B3) :
 - USBD_DEBUG_LEVEL             1
 - SPI_DEBUG_LEVEL              1
 
 Working in WinXP - Win11, Linux (Ubuntu).
 Problems with Windows 10 - windows 10 creates a folder on the disk - "System Volume Information". To fix this:
   
   Step 1:
     Using GPEDIT to modify Do not allow locations on removable drives to be added to libraries setting:
   In Windows 10 / 8.1 Pro & Enterprise Editions, press Windows Key + R combination, type put gpedit.msc in Run dialog box 
   and  hit Enter to open the Local Group Policy Editor.
   Navigate here: Computer Configuration -> Administrative Templates -> Windows Components -> Search
   In the right pane, look for the setting named Do not allow locations on removable drives to be added to libraries and 
   double  click it.
   Click on Enabled and then click Apply followed by OK. Close the Local Group Policy Editor.
   
   Step 2:
     For Windows 10 this additional step is necessary. Go back to Run, type services.msc, click OK. In the right pane scroll 
   down  to Windows Search, double click it and in Startup type: select Disabled. Click OK and then close. Just to be 
   safe  reboot.
   
   Step 3:
     In addition, disable or stop the service - StorSvc (Service storage or  Служба хранилища), and disable "Autoplay".

To connect chips with a supply voltage of 1.8V, you can use such an <a href="https://www.aliexpress.com/item/1005002895111672.html" rel="nofollow">adapter</a> or take a risk and use it without an adapter.

  
# License

This software is provided under the  <a href="http://unlicense.org/" rel="nofollow">UNLICENSE</a>

