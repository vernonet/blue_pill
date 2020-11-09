/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define BUTTON_PIN                      GPIO_PIN_9
#define BUTTON_GPIO_PORT                GPIOB
#define LED_PIN                         GPIO_PIN_13
#define LED_GPIO_PORT                   GPIOC
#define USB_DP_PORT                     GPIOA //GPIOA   GPIOB
#define USB_DP_PIN                      GPIO_PIN_12//12  14
#define LED_Off()                       HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_SET)
#define LED_On()                        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET)
#define LED_Toggle()                    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN)
#define PB_GetState()                   HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, BUTTON_PIN)

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
