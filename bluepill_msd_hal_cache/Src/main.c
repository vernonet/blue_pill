
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"
#include "usbd_storage.h"

/* USER CODE BEGIN Includes */
 #include "W25X80.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
JEDEC_ID jdc_id_;
//uint8_t lbl[] = {'S', 'T', 'M', '_', 'M', 'S', 'D', ' ', ' ', ' ', ' ', 0x08};
extern struct sec_cache_  sec_cache;
extern bool   in_4ba_mode;
volatile bool chk_wr_cache = false;  //flag to periodically check whether the cache needs to be written to disk            
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

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

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	USBD_UsrLog("\033[2J");                           //clear screen
	HAL_Delay(500);

  if (Initialize(NULL) == ARM_DRIVER_OK) {        //init SPI1 
		 int32_t stat = PowerControl(ARM_POWER_FULL);
		 if (stat) {
			 Error_Handler();
		 }
		 
		 ReadJedecId (CMD_READ_RDID, &jdc_id_) ; 
		
#if TYPE_SPI == 0	 
 		 if ((jdc_id_.dev_id !=  WINBOND_NEX_W25X80) || (jdc_id_.man_id != WINBOND_NEX_ID)) {
#elif TYPE_SPI == 1
		if ((jdc_id_.dev_id !=  WINBOND_NEX_W25Q64_V) || (jdc_id_.man_id != WINBOND_NEX_ID)) { 
#elif TYPE_SPI == 2
		if ((jdc_id_.dev_id !=  WINBOND_NEX_W25Q80_V) || (jdc_id_.man_id != WINBOND_NEX_ID)) { 	
#elif TYPE_SPI == 3
		if ((jdc_id_.dev_id !=  MACRONIX_MX25L25645G) || (jdc_id_.man_id != MACRONIX_ID)) { 			 
#else			
    if (0)	{		 
#endif
      MX_GPIO_Init();				
		  Error_Handler();
		 }
		
		  if (jdc_id_.dev_id ==  MACRONIX_MX25L25645G) {
				    uint8_t buf;
					  int ret;
					  ret = ReadConfigReg(CMD_READ_CONF_REG, &buf);  //
            if (ret != ARM_DRIVER_OK) Error_Handler();    
						/* Check Flags Config register value */
						if (!(buf & (1<<5))) {  //5 - 4BA mode flag
							USBD_UsrLog("Enter to 4BA mode!\n");					
							ret = spi_enter_4ba();
							if (ret) {
							USBD_UsrLog("Failed to set correct 4BA mode! Aborting.\n");
							Error_Handler();
		          }
						}
						else {
							USBD_UsrLog("\n 4BA mode already enabled!\n");
							in_4ba_mode = true;
						}
            
	       }

	}
//	sec_cache.is_cached = false;
//	sec_cache.need_wr   = false;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();	
	HAL_Delay(50); 	
  MX_USB_DEVICE_Init();
	HAL_Delay(50); 
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(USB_DP_PORT, USB_DP_PIN,  GPIO_PIN_SET); //USB DP PULLUP
	MX_TIM2_Init();
	HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	   if (chk_wr_cache && sec_cache.is_cached && sec_cache.need_wr)  {
				 EraseSector ((sec_cache.blk_adr&SECTOR_MASK)*STORAGE_BLK_SIZ);	 
				 ProgramData ((sec_cache.blk_adr&SECTOR_MASK)*STORAGE_BLK_SIZ, &sec_cache.buf[0], FLASH_SECTOR_SIZE);
				 sec_cache.is_cached = false;
				 chk_wr_cache = false;
				 sec_cache.need_wr = false;	 
				 __HAL_TIM_SET_COUNTER(&htim2, 1);
		   }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

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
	
	__HAL_AFIO_REMAP_SWJ_NOJTAG();                              //for SWO printf
	
	 /*Configure GPIO pin Output Level */
  LED_Off();    //LED indicate reading writing


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_DP_PORT, USB_DP_PIN, GPIO_PIN_RESET);   //USB DP PULLDOWN

  /*Configure GPIO pin : USB_DP */                           
  GPIO_InitStruct.Pin  = USB_DP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(USB_DP_PORT, &GPIO_InitStruct);
	
	/*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* TIM2 init function */
static void MX_TIM2_Init(void)                  //for flush cache
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 50000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
  while(1)
  {
		LED_Toggle();   
		HAL_Delay(80);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
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
