
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2020 STMicroelectronics International N.V.
 * All rights reserved.
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"
#include "button.h"

/* USER CODE BEGIN Includes */
#include "usbd_storage.h"
#include "chip_drv.h"
#include "flashchips.h"
#include "Driver_SPI.h"
#include "spi.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t status, st;
__IO uint32_t CRCValue_actual = 0;  // calculated after write flash
__IO uint32_t CRCValue_nominal = 0; // calculated before write flash
// uint8_t crc_buf[0x200] __attribute__((aligned(4)));
uint8_t *crc_buf;
extern uint8_t Wr_Protect;
volatile uint8_t complet = 0, error = 0, error_sts = 0;
extern int32_t file_size;
volatile uint32_t blink = 2;
extern const struct flashchip *flschip;
extern unsigned char boot_sec[];
extern const struct flashchip flashchips[];
extern JEDEC_ID jdc_id_;
const char build_date[] = "Compile date "__DATE__" " __TIME__;
extern Media_mode device_mode;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
bool process_complete(Media_mode mde);
bool prepare_device(Media_mode mde);
void key_led(uint32_t blk);
//void set_msd_size(uint32_t size_in_kb);
const char* getModeName(Media_mode mode); 
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  int32_t stat;
  // uint8_t str_buf[100];
  uint16_t led_delay;
	uint8_t sss;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SPI_UsrLog("\033[2J"); // clear screen
  SPI_UsrLog("\n %s", build_date);
  SPI_UsrLog("\n In base %d flashchips\n", flashchips_in_base());
  HAL_Delay(500);

  if (Initialize(NULL) == ARM_DRIVER_OK)
  { // init SPI
    PowerControl(ARM_POWER_FULL);
    MX_GPIO_Init();
    if (flschip)
    {
			if (flschip->voltage.max && flschip->voltage.max < 3000) {
				SPI_UsrLog("\n Warning, the max Vcc of the chip is %d mV", flschip->voltage.max);
			}
			
      HAL_Delay(800);
      prepare_device(BACKUP);
      key_led(40);
      if (prepare_device(VERIFY))
        key_led(8); // error present
      else
        key_led(40); // no error
      if (prepare_device(PROG))
        Error_Handler(); // error present

      while (1)
      {
        LED_Toggle(); // no error
        HAL_Delay(500);
      }

      /* USER CODE END 2 */

      /* Infinite loop */
      /* USER CODE BEGIN WHILE */

      /* USER CODE END WHILE */
    }
    else
    {
      // flash not identified  or wrong connection
      HAL_Delay(800);
      // flschip = flash_id_to_entry(GENERIC_MANUF_ID, GENERIC_DEVICE_ID);
      //set_msd_size(1024);
      prepare_device(INFO);
      Error_Handler();
    }
  }

  else
    Error_Handler(); // flash not identified
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure the Systick interrupt time
   */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
   */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

//}

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  __HAL_AFIO_REMAP_SWJ_NOJTAG(); // for SWO printf

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_SET); // LED indicate reading writing

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_DP_PORT, USB_DP_PIN, GPIO_PIN_RESET); // USB DP PULLDOWN

  /*Configure GPIO pin : USB_DP */
  GPIO_InitStruct.Pin = USB_DP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(USB_DP_PORT, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = BUTTON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUTTON_GPIO_PORT, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

const char* getModeName(Media_mode mode) 
{
   switch (mode) 
   {
      case BACKUP: return "BACKUP";
      case VERIFY: return "VERIFY";
		  case PROG:   return "PROG";
		  case INFO:   return "INFO";    
   }
}


void key_led(uint32_t blk)
{
  uint32_t blink_ = blk;

  while (1)
  {
    if (blink_-- == 0)
    {
      LED_Toggle();
      blink_ = blk; // 40;
    }
    HAL_Delay(10);
    if (SWPressed() == 1)
    {
      LED_Off();
      break;
    }
  }
}

//* @retval Status (0 : Ok / 1 : Error)
bool prepare_device(Media_mode mde)
{
  int32_t stat = 0;
  uint8_t str_buf[100];
  int8_t sts;
	uint8_t sr;
	int32_t feature_bits = flschip->feature_bits;

  complet = 0;
	file_size = 0;
  if (mde != BACKUP && mde != INFO)
  {
    HAL_GPIO_WritePin(USB_DP_PORT, USB_DP_PIN, GPIO_PIN_RESET); // USB DP PULLDOWN
    MX_USB_DEVICE_DeInit();
  }

  HAL_Delay(200);
  device_mode = mde;
	USBD_UsrLog("\n\r device_mode -> %s", getModeName(mde));

  switch (mde)
  {
  case BACKUP:
    Wr_Protect = 1;
    if (Prepare_FAT(flschip->total_size, flschip->name, "BACKUP"))
      Error_Handler();
    break;
  case VERIFY:
    Wr_Protect = 0;
	  HAL_Delay(1000);
    if (Prepare_FAT(flschip->total_size, NULL, "VERIFY"))
      Error_Handler();
    break;
  case PROG:
    Wr_Protect = 0;
    stat = EraseChip();
    if (stat)
    {
      // flash not erased
      device_mode = INFO;
			USBD_UsrLog("\n\r device_mode -> %s", getModeName(device_mode));

       if (stat == ARM_DRIVER_ERROR_BLK_PROT)
      {
				sprintf((char *)&str_buf[0], "Disable block ptotections error");
      }
      else if (stat == ARM_DRIVER_ERROR_STS_REG_FF)
      {
				sprintf((char *)&str_buf[0], "Chip erase error statusReg 0xFF");
      }
      else if (stat == ARM_DRIVER_ERROR_NOT_BLANK)
      {
				 sprintf((char *)&str_buf[0], "Chip not blank after erase");
      }
      else
      {
				 sprintf((char *)&str_buf[0], "Chip erase error");
      }
			//add status register values to the line
			ReadStatusReg(CMD_READ_STATUS, &sr);
			sprintf((char *)&str_buf[strlen((char *)&str_buf[0])], " SR1_0x%X", sr);
			if (feature_bits & FEATURE_WRSR2) {
				ReadStatusReg(CMD_READ_STATUS2, &sr);
				sprintf((char *)&str_buf[strlen((char *)&str_buf[0])], " SR2_0x%X", sr);
			}	
			if (feature_bits & FEATURE_WRSR3) {
				ReadStatusReg(CMD_READ_STATUS3, &sr);
				sprintf((char *)&str_buf[strlen((char *)&str_buf[0])], " SR3_0x%X ", sr);
			}	
			if (Prepare_FAT(flschip->total_size, (char *)&str_buf[0], "ERROR "))
				Error_Handler();
    }
    else
    {
      if (Prepare_FAT(flschip->total_size, NULL, "PROG  "))
        Error_Handler(); // flash erased OK!
    }
    break;
  case INFO:
    Wr_Protect = 1;
    if (!(jdc_id_.man_id == 0xFF && (jdc_id_.dev_id == 0xFFFF | jdc_id_.dev_id == 0xFF | jdc_id_.dev_id == 0x00)))
    {
      sprintf((char *)&str_buf[0], "unknown SPI chip manId 0x%X devId 0x%X", jdc_id_.man_id, jdc_id_.dev_id);
      if (Prepare_FAT(1024, (char *)&str_buf[0], "ERROR "))
        Error_Handler();
    }
    // if the chip is not recognized and the ID is not readable
    else
    {
      if (Prepare_FAT(1024, "unknown SPI chip or wrong connections", "ERROR "))
        Error_Handler();
    }
    break;
  default:
    break;
  }

  /* Initialize all configured peripherals */
  HAL_Delay(50);
  MX_USB_DEVICE_Init();
  HAL_GPIO_WritePin(USB_DP_PORT, USB_DP_PIN, GPIO_PIN_SET); // USB DP PULLUP

  if (stat)
    return 1;

  if (mde == VERIFY || mde == PROG)
    return (process_complete(mde));

  return 0;
}

//* @retval Status (0 : Ok / 1 : Error)
bool process_complete(Media_mode mde)
{
  int8_t sts = 0;

  CRCValue_actual = 0;
  CRCValue_nominal = 0;
  HAL_Delay(100);
  while (1)
  {
    HAL_Delay(10);
    if (SWPressed() == 1)
      return 0;

    if (complet) // if the write is completed
    {
      HAL_Delay(10);
      HAL_GPIO_WritePin(USB_DP_PORT, USB_DP_PIN, GPIO_PIN_RESET); // USB DP PULLDOWN
      MX_USB_DEVICE_DeInit();
      crc_buf = malloc(CRC_BUFF_SZE);
      if ((file_size > STORAGE_BLK_SIZ) && (file_size <= (flschip->total_size * 1024)))
      {
        volatile uint32_t adr = 0, len = 0;
        while (adr < file_size)
        {
          if (file_size - adr > (sizeof(crc_buf[0]) * CRC_BUFF_SZE))
            len = sizeof(crc_buf[0]) * CRC_BUFF_SZE;
          else
            len = file_size - adr;
          ReadData(adr, (uint32_t *)crc_buf, len); // sizeof(crc_buf)
          CRCValue_actual = CalcCRC32(crc_buf, len, CRCValue_actual);
          adr += len;
        }
        free(crc_buf);
        if ((error_sts) || !(CRCValue_nominal == CRCValue_actual) || (error))
        {
          // error CRC or any other error
          device_mode = INFO;
					USBD_UsrLog("\n\r device_mode -> %s", getModeName(device_mode));
          if (error_sts)
            sts = Prepare_FAT(flschip->total_size, "Error during programming", "ERROR ");
          else if (!(CRCValue_nominal == CRCValue_actual))
          {
            if (mde == VERIFY)
              sts = Prepare_FAT(flschip->total_size, "CRC error after verifing", "ERROR ");
            else
              sts = Prepare_FAT(flschip->total_size, "CRC error after programming", "ERROR ");
          }
          else
            sts = Prepare_FAT(flschip->total_size,"Error during programming-verifing", "ERROR ");
          if (sts)
            Error_Handler(); // error on prepare fat
          HAL_Delay(50);
          MX_USB_DEVICE_Init();
          HAL_GPIO_WritePin(USB_DP_PORT, USB_DP_PIN, GPIO_PIN_SET); // USB DP PULLUP
          return 1;                                                 // Error_Handler();
        }
        else
        {
          // no error, all ok!
          device_mode = INFO;
					USBD_UsrLog("\n\r device_mode -> %s", getModeName(device_mode));
          if (mde == PROG)
            sts = Prepare_FAT(flschip->total_size, "Programming completed successfully", "GOOD ");
          else if (mde == VERIFY)
            sts = Prepare_FAT(flschip->total_size, "Verifing completed successfully", "GOOD ");
          if (sts)
            Error_Handler(); // error on prepare fat
          HAL_Delay(50);
          MX_USB_DEVICE_Init();
          HAL_GPIO_WritePin(USB_DP_PORT, USB_DP_PIN, GPIO_PIN_SET); // USB DP PULLUP
          return 0;
        }
      }
      else
        Error_Handler(); // file size error
    }
  }
}


/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1)
  {
    SPI_UsrLog("\n\r ERROR -> %s %d", file, line);
    while (1)
    {
      LED_Toggle();
      HAL_Delay(80);
    }
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
