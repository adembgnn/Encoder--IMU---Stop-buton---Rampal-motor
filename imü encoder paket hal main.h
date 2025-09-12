/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file. (Mecanum Robot v1.5 - Final Synced)
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
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOC
#define ENCODER2_A_Pin GPIO_PIN_0
#define ENCODER2_A_GPIO_Port GPIOA
#define ENCODER2_B_Pin GPIO_PIN_1
#define ENCODER2_B_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define ACIL_STOP_BUTON_Pin GPIO_PIN_6
#define ACIL_STOP_BUTON_GPIO_Port GPIOA
#define ACIL_STOP_BUTON_EXTI_IRQn EXTI9_5_IRQn
#define RGB_Pin GPIO_PIN_0
#define RGB_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_10
#define IMU_SCL_GPIO_Port GPIOB
#define SAG_ARKA_MOTOR_DIR_Pin GPIO_PIN_6
#define SAG_ARKA_MOTOR_DIR_GPIO_Port GPIOC
#define SOL_ARKA_MOTOR_DIR_Pin GPIO_PIN_7
#define SOL_ARKA_MOTOR_DIR_GPIO_Port GPIOC
#define SOL_ON_MOTOR_DIR_Pin GPIO_PIN_8
#define SOL_ON_MOTOR_DIR_GPIO_Port GPIOC
#define SAG_ON_MOTOR_DIR_Pin GPIO_PIN_9
#define SAG_ON_MOTOR_DIR_GPIO_Port GPIOC
#define SAG_ARKA_MOTOR_Pin GPIO_PIN_8
#define SAG_ARKA_MOTOR_GPIO_Port GPIOA
#define SOL_ARKA_MOTOR_Pin GPIO_PIN_9
#define SOL_ARKA_MOTOR_GPIO_Port GPIOA
#define SOL_ON_MOTOR_Pin GPIO_PIN_10
#define SOL_ON_MOTOR_GPIO_Port GPIOA
#define SAG_ON_MOTOR_Pin GPIO_PIN_11
#define SAG_ON_MOTOR_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define ENCODER1_A_Pin GPIO_PIN_15
#define ENCODER1_A_GPIO_Port GPIOA
#define IMU_SDA_Pin GPIO_PIN_12
#define IMU_SDA_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define ENCODER4_A_Pin GPIO_PIN_4
#define ENCODER4_A_GPIO_Port GPIOB
#define ENCODER4_B_Pin GPIO_PIN_5
#define ENCODER4_B_GPIO_Port GPIOB
#define ENCODER3_A_Pin GPIO_PIN_6
#define ENCODER3_A_GPIO_Port GPIOB
#define ENCODER3_B_Pin GPIO_PIN_7
#define ENCODER3_B_GPIO_Port GPIOB
#define ENCODER1_B_Pin GPIO_PIN_9
#define ENCODER1_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
// --- Genel Parametreler ---
#define RX_BUFFER_SIZE              128

// --- Motor Kontrol Pinleri (Anlaşılır İsimler) ---
#define MOTOR_FR_PWM_TIM      htim1
#define MOTOR_FR_PWM_CHANNEL  TIM_CHANNEL_4
#define MOTOR_FR_DIR_PORT     SAG_ON_MOTOR_DIR_GPIO_Port
#define MOTOR_FR_DIR_PIN      SAG_ON_MOTOR_DIR_Pin

#define MOTOR_FL_PWM_TIM      htim1
#define MOTOR_FL_PWM_CHANNEL  TIM_CHANNEL_3
#define MOTOR_FL_DIR_PORT     SOL_ON_MOTOR_DIR_GPIO_Port
#define MOTOR_FL_DIR_PIN      SOL_ON_MOTOR_DIR_Pin

#define MOTOR_BL_PWM_TIM      htim1
#define MOTOR_BL_PWM_CHANNEL  TIM_CHANNEL_2
#define MOTOR_BL_DIR_PORT     SOL_ARKA_MOTOR_DIR_GPIO_Port
#define MOTOR_BL_DIR_PIN      SOL_ARKA_MOTOR_DIR_Pin

#define MOTOR_BR_PWM_TIM      htim1
#define MOTOR_BR_PWM_CHANNEL  TIM_CHANNEL_1
#define MOTOR_BR_DIR_PORT     SAG_ARKA_MOTOR_DIR_GPIO_Port
#define MOTOR_BR_DIR_PIN      SAG_ARKA_MOTOR_DIR_Pin

// --- Enkoder ve Telemetri Sabitleri ---

#define WHEEL_DIAMETER_MM           65.0f
#define WHEEL_CIRCUMFERENCE_CM      (WHEEL_DIAMETER_MM * 3.1415926535f / 10.0f)
#define RAMP_UPDATE_PERIOD_MS       10
#define TELEMETRY_PERIOD_MS         250
#define BUZZER_TOGGLE_PERIOD_MS     250

// Kalman Filtresi Ayarları
#define R_HIZ_NORMAL     0.3f
#define R_HIZ_YAVASLAMA  0.05f

// Diğer Pinler

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
