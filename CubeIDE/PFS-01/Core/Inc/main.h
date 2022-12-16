/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
// Variables for changing usage are gathered here.
//BOARD selection
static const uint8_t SELECT_PFS_01_SINGLE = 0x00;
static const uint8_t SELECT_PFS_01_ASM = 0x01;
//#define SELECT_PFS_01 SELECT_PFS_01_SINGLE; // comment in when PFS-01A only
#define SELECT_PFS_01 SELECT_PFS_01_ASM; // comment in when assembly board

// SLAVE MODE enable
#define enable_pr2_spi_slave 0 // enable or not PR2 slave
#define enable_uart_slave 0 // enable or not UART serial slave
#define enable_usb_slave 0 // enable or not USB serial slave
#define enable_i2c_slave 1 // enable or not USB serial slave

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Software_RESET_Pin GPIO_PIN_13
#define Software_RESET_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_4
#define LED_GPIO_Port GPIOC
#define IMU_FSYNC_Pin GPIO_PIN_11
#define IMU_FSYNC_GPIO_Port GPIOC
#define IMU_nCS_Pin GPIO_PIN_6
#define IMU_nCS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
