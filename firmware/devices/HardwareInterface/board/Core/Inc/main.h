/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

// CAuDri - Manual forward declaration of the HAL handles (for use in config.h)
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern CAN_HandleTypeDef hcan1;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

extern SPI_HandleTypeDef hspi4;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define I2C1_GPIO3_Pin GPIO_PIN_2
#define I2C1_GPIO3_GPIO_Port GPIOE
#define I2C1_GPIO3_EXTI_IRQn EXTI2_IRQn
#define I2C1_GPIO4_Pin GPIO_PIN_3
#define I2C1_GPIO4_GPIO_Port GPIOE
#define I2C1_GPIO5_Pin GPIO_PIN_4
#define I2C1_GPIO5_GPIO_Port GPIOE
#define I2C1_GPIO5_EXTI_IRQn EXTI4_IRQn
#define I2C1_GPIO6_Pin GPIO_PIN_5
#define I2C1_GPIO6_GPIO_Port GPIOE
#define I2C1_GPIO7_Pin GPIO_PIN_6
#define I2C1_GPIO7_GPIO_Port GPIOE
#define I2C1_GPIO7_EXTI_IRQn EXTI9_5_IRQn
#define I2C1_GPIO8_Pin GPIO_PIN_13
#define I2C1_GPIO8_GPIO_Port GPIOC
#define I2C2_GPIO1_Pin GPIO_PIN_5
#define I2C2_GPIO1_GPIO_Port GPIOF
#define I2C2_GPIO1_EXTI_IRQn EXTI9_5_IRQn
#define I2C2_GPIO2_Pin GPIO_PIN_6
#define I2C2_GPIO2_GPIO_Port GPIOF
#define I2C2_GPIO3_Pin GPIO_PIN_7
#define I2C2_GPIO3_GPIO_Port GPIOF
#define I2C2_GPIO3_EXTI_IRQn EXTI9_5_IRQn
#define I2C2_GPIO4_Pin GPIO_PIN_8
#define I2C2_GPIO4_GPIO_Port GPIOF
#define I2C2_GPIO5_Pin GPIO_PIN_9
#define I2C2_GPIO5_GPIO_Port GPIOF
#define I2C2_GPIO5_EXTI_IRQn EXTI9_5_IRQn
#define I2C2_GPIO6_Pin GPIO_PIN_0
#define I2C2_GPIO6_GPIO_Port GPIOC
#define I2C2_GPIO7_Pin GPIO_PIN_1
#define I2C2_GPIO7_GPIO_Port GPIOC
#define I2C2_GPIO7_EXTI_IRQn EXTI1_IRQn
#define I2C2_GPIO8_Pin GPIO_PIN_2
#define I2C2_GPIO8_GPIO_Port GPIOC
#define ANALOG_IN1_Pin GPIO_PIN_0
#define ANALOG_IN1_GPIO_Port GPIOA
#define ANALOG_IN2_Pin GPIO_PIN_1
#define ANALOG_IN2_GPIO_Port GPIOA
#define SERVO1_ADC_Pin GPIO_PIN_4
#define SERVO1_ADC_GPIO_Port GPIOA
#define SERVO2_ADC_Pin GPIO_PIN_5
#define SERVO2_ADC_GPIO_Port GPIOA
#define SERVO1_PWM_Pin GPIO_PIN_6
#define SERVO1_PWM_GPIO_Port GPIOA
#define SERVO2_PWM_Pin GPIO_PIN_7
#define SERVO2_PWM_GPIO_Port GPIOA
#define PWR_EXT_FAULT_Pin GPIO_PIN_11
#define PWR_EXT_FAULT_GPIO_Port GPIOF
#define PWR_EXT_FAULT_EXTI_IRQn EXTI15_10_IRQn
#define PWR_STATUS_Pin GPIO_PIN_12
#define PWR_STATUS_GPIO_Port GPIOF
#define PWR_STATUS_EXTI_IRQn EXTI15_10_IRQn
#define GPIO1_Pin GPIO_PIN_13
#define GPIO1_GPIO_Port GPIOF
#define GPIO1_EXTI_IRQn EXTI15_10_IRQn
#define GPIO2_Pin GPIO_PIN_14
#define GPIO2_GPIO_Port GPIOF
#define GPIO2_EXTI_IRQn EXTI15_10_IRQn
#define GPIO3_Pin GPIO_PIN_15
#define GPIO3_GPIO_Port GPIOF
#define GPIO3_EXTI_IRQn EXTI15_10_IRQn
#define SPI_IO1_Pin GPIO_PIN_10
#define SPI_IO1_GPIO_Port GPIOE
#define SPI_IO2_Pin GPIO_PIN_11
#define SPI_IO2_GPIO_Port GPIOE
#define PWR_EXT_ENABLE_Pin GPIO_PIN_12
#define PWR_EXT_ENABLE_GPIO_Port GPIOB
#define USB2_VBUS_Pin GPIO_PIN_13
#define USB2_VBUS_GPIO_Port GPIOB
#define USB2_D__Pin GPIO_PIN_14
#define USB2_D__GPIO_Port GPIOB
#define USB2_D_B15_Pin GPIO_PIN_15
#define USB2_D_B15_GPIO_Port GPIOB
#define TIM2_GPIO_Pin GPIO_PIN_10
#define TIM2_GPIO_GPIO_Port GPIOD
#define LED1_PWM_Pin GPIO_PIN_14
#define LED1_PWM_GPIO_Port GPIOD
#define LED2_PWM_Pin GPIO_PIN_15
#define LED2_PWM_GPIO_Port GPIOD
#define DEBUG_LED_BLUE_Pin GPIO_PIN_4
#define DEBUG_LED_BLUE_GPIO_Port GPIOG
#define DEBUG_LED_GREEN_Pin GPIO_PIN_5
#define DEBUG_LED_GREEN_GPIO_Port GPIOG
#define DEBUG_LED_RED_Pin GPIO_PIN_6
#define DEBUG_LED_RED_GPIO_Port GPIOG
#define TIM8_IO_Pin GPIO_PIN_8
#define TIM8_IO_GPIO_Port GPIOC
#define USER_BUTTON_Pin GPIO_PIN_8
#define USER_BUTTON_GPIO_Port GPIOA
#define USER_BUTTON_EXTI_IRQn EXTI9_5_IRQn
#define USB1_VBUS_Pin GPIO_PIN_9
#define USB1_VBUS_GPIO_Port GPIOA
#define USB1_D__Pin GPIO_PIN_11
#define USB1_D__GPIO_Port GPIOA
#define USB1_D_A12_Pin GPIO_PIN_12
#define USB1_D_A12_GPIO_Port GPIOA
#define SWD_SWDIO_Pin GPIO_PIN_13
#define SWD_SWDIO_GPIO_Port GPIOA
#define SWD_SWCLK_Pin GPIO_PIN_14
#define SWD_SWCLK_GPIO_Port GPIOA
#define UART3_GPIO_Pin GPIO_PIN_12
#define UART3_GPIO_GPIO_Port GPIOC
#define UART2_GPIO_Pin GPIO_PIN_3
#define UART2_GPIO_GPIO_Port GPIOD
#define UART2_GPIO_EXTI_IRQn EXTI3_IRQn
#define UART6_GPIO_Pin GPIO_PIN_10
#define UART6_GPIO_GPIO_Port GPIOG
#define UART6_GPIO_EXTI_IRQn EXTI15_10_IRQn
#define SWD_SWO_Pin GPIO_PIN_3
#define SWD_SWO_GPIO_Port GPIOB
#define I2C1_GPIO1_Pin GPIO_PIN_0
#define I2C1_GPIO1_GPIO_Port GPIOE
#define I2C1_GPIO1_EXTI_IRQn EXTI0_IRQn
#define I2C1_GPIO2_Pin GPIO_PIN_1
#define I2C1_GPIO2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
