#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

#define UNBOUNCE_CNT     (10)

uint8_t SWPressed (void) {
  volatile uint8_t SWKeyCount = 0;
 
	 while(1)	{
     if (PB_GetState() == 0)  { 			/* Check Button SW          */
			 HAL_Delay(1);
       if (SWKeyCount < UNBOUNCE_CNT) SWKeyCount++;
       else 
				{				
        SWKeyCount = 0;
        return (1);
        }	
    }
		else break;
	} 
  return (0);
}
