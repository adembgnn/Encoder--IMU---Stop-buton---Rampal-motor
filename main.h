/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  * This file contains the common defines of the application.
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
#include <stdbool.h>
#include "math.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
    MOTOR_DURDU = 0,
    MOTOR_ILERI,
    MOTOR_GERI
} MotorState_t;
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
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
// --- Genel Parametreler ---
#define RX_BUFFER_SIZE              1
#define DEFAULT_MOTOR_SPEED         6500
#define DEBOUNCE_DELAY_MS           50

// --- Motor Kontrol Pinleri ---
#define MOTOR_DIR_GPIO_PORT         GPIOB
#define MOTOR_DIR_PIN               GPIO_PIN_0
#define MOTOR_PWM_CHANNEL           TIM_CHANNEL_2

// --- Buzzer Pini ---
#define BUZZER_GPIO_PORT            GPIOC
#define BUZZER_PIN                  GPIO_PIN_0

// --- Çalıştırma Butonu Pini ---
#define RUN_BUTTON_GPIO_PORT        GPIOA
#define RUN_BUTTON_PIN              GPIO_PIN_6

// --- Enkoder ve Telemetri Sabitleri ---
#define ENCODER_PPR                 2400.0f
#define WHEEL_DIAMETER_MM           65.0f
#define WHEEL_CIRCUMFERENCE_CM      (WHEEL_DIAMETER_MM * 3.1415926535f / 10.0f)
#define TELEMETRY_PERIOD_MS         250 // Hız ölçümünü kararlı hale getirmek için 250ms'ye çıkarıldı
#define BUZZER_TOGGLE_PERIOD_MS     250
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
