/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t uart_rx;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PA15_RESERVED_Pin GPIO_PIN_15
#define PA15_RESERVED_GPIO_Port GPIOA
#define PB3_RESERVED_Pin GPIO_PIN_3
#define PB3_RESERVED_GPIO_Port GPIOB
#define PA12_RESERVED_Pin GPIO_PIN_12
#define PA12_RESERVED_GPIO_Port GPIOA
#define PB4_RESERVED_Pin GPIO_PIN_4
#define PB4_RESERVED_GPIO_Port GPIOB
#define PB4_RESERVED_EXTI_IRQn EXTI4_15_IRQn
#define PC13_RESERVED_Pin GPIO_PIN_13
#define PC13_RESERVED_GPIO_Port GPIOC
#define PC13_RESERVED_EXTI_IRQn EXTI4_15_IRQn
#define PC1_RESERVED_Pin GPIO_PIN_1
#define PC1_RESERVED_GPIO_Port GPIOC
#define PC0_RESERVED_Pin GPIO_PIN_0
#define PC0_RESERVED_GPIO_Port GPIOC
#define PB1_RESERVED_Pin GPIO_PIN_1
#define PB1_RESERVED_GPIO_Port GPIOB
#define PB1_RESERVED_EXTI_IRQn EXTI0_1_IRQn
#define RS485_DE_GPIO_Pin GPIO_PIN_0
#define RS485_DE_GPIO_GPIO_Port GPIOA
#define PC2_RESERVED_Pin GPIO_PIN_2
#define PC2_RESERVED_GPIO_Port GPIOC
#define PA7_RESERVED_Pin GPIO_PIN_7
#define PA7_RESERVED_GPIO_Port GPIOA
#define RS485_RE_GPIO_Pin GPIO_PIN_4
#define RS485_RE_GPIO_GPIO_Port GPIOA
#define STLINK_RX_Pin GPIO_PIN_2
#define STLINK_RX_GPIO_Port GPIOA
#define PB0_RESERVED_Pin GPIO_PIN_0
#define PB0_RESERVED_GPIO_Port GPIOB
#define PB0_RESERVED_EXTI_IRQn EXTI0_1_IRQn
#define PA6_RESERVED_Pin GPIO_PIN_6
#define PA6_RESERVED_GPIO_Port GPIOA
#define STLINK_TX_Pin GPIO_PIN_3
#define STLINK_TX_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */


// RS485_DE Data Enable, Active High
// RS485_RE Receive En, Active Low
#define ENABLE_TRANSMIT() do { \
    /* Disable Receiver */ \
    HAL_GPIO_WritePin(RS485_RE_GPIO_GPIO_Port, RS485_RE_GPIO_Pin, GPIO_PIN_SET); \
    /* Enable Transmitter */ \
    HAL_GPIO_WritePin(RS485_DE_GPIO_GPIO_Port, RS485_DE_GPIO_Pin, GPIO_PIN_SET); \
    } while(0)

#define ENABLE_RECEIVE() do { \
    /* Enable Receiver */ \
    HAL_GPIO_WritePin(RS485_RE_GPIO_GPIO_Port, RS485_RE_GPIO_Pin, GPIO_PIN_RESET); \
    /* Disable Transmitter */ \
    HAL_GPIO_WritePin(RS485_DE_GPIO_GPIO_Port, RS485_DE_GPIO_Pin, GPIO_PIN_RESET); \
    } while(0)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
