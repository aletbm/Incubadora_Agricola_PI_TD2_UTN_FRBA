/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void StartMenuTask(void * argument);
void StartDebounceTask(void * argument);
void StartMotorTask(void * argument);
void StartControlTask(void * argument);
void StartControlTask(void * argument);
void StartSensorTask(void * argument);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define ENCODER_A_Pin GPIO_PIN_0
#define ENCODER_A_GPIO_Port GPIOC
#define ENCODER_B_Pin GPIO_PIN_1
#define ENCODER_B_GPIO_Port GPIOC
#define ENCODER_B_EXTI_IRQn EXTI1_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define ENCODER_SW_Pin GPIO_PIN_0
#define ENCODER_SW_GPIO_Port GPIOB
#define LAMPARA_Pin GPIO_PIN_10
#define LAMPARA_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_12
#define BUZZER_GPIO_Port GPIOB
#define HUMIDIFICADOR_Pin GPIO_PIN_8
#define HUMIDIFICADOR_GPIO_Port GPIOA
#define COOLER_Pin GPIO_PIN_9
#define COOLER_GPIO_Port GPIOA
#define DHT11_Pin GPIO_PIN_11
#define DHT11_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define MOTOR_Pin GPIO_PIN_4
#define MOTOR_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
typedef enum {
    HUM_STATE_IDLE,      // Esperando
    HUM_STATE_DOSING,    // Inyectando humedad
    HUM_STATE_COOLDOWN   // Esperando estabilización
} HumidifierState_t;

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim12;
extern float last_valid_temp;
extern float last_valid_hum;
extern HumidifierState_t hum_state;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
