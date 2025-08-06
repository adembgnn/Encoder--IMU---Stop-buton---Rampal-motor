/* USER CODE BEGIN Header */

/**

******************************************************************************

* @file : main.c

* @brief : Main program body (Nihai Sürüm v13.0 - Full Paket)

*

* @description

* - STM32CubeMX orijinal uzun formatında.

* - Hız ölçümü, dalgalanmayı önlemek için yürüyen ortalama filtresi ile kararlı hale getirildi.

* - Enkoder sayım yönü yazılımsal olarak düzeltildi.

* - Z-Noktası ile mesafe sıfırlama ve mesaj basma devre dışı bırakıldı.

* - Gidilen mesafeden sonra başlangıç noktasına dönüldüğünde mesaj basma özelliği eklendi.

* - Donma ve kilitlenmeye yol açan tüm kritik hatalar giderildi.

******************************************************************************

*/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"

#include "string.h"

#include "math.h"

#include "bno055_stm32.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// YENİ: Hız filtresi için pencere boyutu

#define MOVING_AVG_SIZE 5

#define RAMP_UPDATE_PERIOD_MS 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// --- Global Değişkenler ---

volatile uint32_t g_current_pwm = 0;

volatile uint32_t g_target_pwm = 0;

volatile uint32_t g_last_ramp_time = 0;

uint8_t g_rx_buffer[RX_BUFFER_SIZE];

volatile MotorState_t g_target_motor_state = MOTOR_DURDU;

volatile uint32_t g_target_motor_speed = 0;

volatile MotorState_t g_current_motor_state = MOTOR_DURDU;

bool g_run_button_is_pressed = false;

volatile uint32_t g_last_buzzer_toggle_time = 0;

volatile bool g_new_command_received = false;

volatile char g_received_command;

volatile int32_t g_encoder_last_count = 0;

volatile uint32_t g_last_telemetry_time = 0;

bno055_vector_t euler_angles; // Euler açılarını (yaw, roll, pitch) tutacak değişken

bool imu_baslatildi = false;

// YENİ: Başlangıç noktası mesajı için bayrak

bool g_start_point_message_sent = true;



// YENİ: Hız filtresi için değişkenler

float g_speed_history[MOVING_AVG_SIZE] = {0.0f};

int g_speed_history_index = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

void UpdateMotorRamp(void);

int _write(int file, char *ptr, int len);

void SetMotorHardware(MotorState_t state, uint32_t pwm_value);

void ProcessNewCommand(char cmd);

void UpdateAndDisplayTelemetry(void);

void I2C_Scanner(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */



  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */



  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */



  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_I2C3_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

HAL_TIM_PWM_Start(&htim1, MOTOR_PWM_CHANNEL);

HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);

HAL_UART_Receive_IT(&huart2, g_rx_buffer, RX_BUFFER_SIZE);

__HAL_TIM_SET_COUNTER(&htim2, 0);



printf("\n--- Motor Kontrol v13.1 (Hatalar Giderildi) ---\n");

fflush(stdout);



// I2C hattını tara ve BNO055 sensörünü başlat

I2C_Scanner();

bno055_assignI2C(&hi2c2);

bno055_setup();

bno055_setOperationModeNDOF();

HAL_Delay(100); // Mod değişikliği için bekle



// Başlatmanın başarılı olup olmadığını son bir kez kontrol et

uint8_t chip_id = 0;

bno055_readData(BNO055_CHIP_ID, &chip_id, 1);

if (chip_id == BNO055_ID) {

printf("BNO055 basariyla baslatildi!\n");

imu_baslatildi = true;

} else {

printf("HATA: BNO055 baslatilamadi! Baglantilari kontrol edin.\n");

}



printf("Komutlar: 'i', 'g', 'd'. Motoru calistirmak icin butona basili tutun.\n\n");

fflush(stdout);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

while (1)

{

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

bool komut_yeni_geldi = false;

UpdateMotorRamp();

if (g_new_command_received) {

ProcessNewCommand(g_received_command); // Bu artık sadece hedefi belirliyor

g_new_command_received = false;

komut_yeni_geldi = true; // Yeni komutun geldiğini işaretle

}



bool previous_button_state = g_run_button_is_pressed;

g_run_button_is_pressed = (HAL_GPIO_ReadPin(RUN_BUTTON_GPIO_PORT, RUN_BUTTON_PIN) == GPIO_PIN_RESET);



if (g_run_button_is_pressed != previous_button_state) {

if (g_run_button_is_pressed) {

printf("\n>>> Acil Stop PASIF - Motor calisiyor. <<<\n");

fflush(stdout);

SetMotorHardware(g_target_motor_state, g_target_motor_speed);

} else {

printf("\n>>> ACIL STOP AKTIF - Motor durduruldu! <<<\n");

fflush(stdout);

SetMotorHardware(MOTOR_DURDU, 0);

}

}

if (komut_yeni_geldi && g_run_button_is_pressed)

{

// Hedef değişti ve buton hala basılı, motoru yeni hedefe göre güncelle

SetMotorHardware(g_target_motor_state, g_target_motor_speed);

}

if (g_current_motor_state == MOTOR_GERI && g_run_button_is_pressed) {

if (HAL_GetTick() - g_last_buzzer_toggle_time >= BUZZER_TOGGLE_PERIOD_MS) {

HAL_GPIO_TogglePin(BUZZER_GPIO_PORT, BUZZER_PIN);

g_last_buzzer_toggle_time = HAL_GetTick();

}

} else {

HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_PIN, GPIO_PIN_RESET);

}



if (HAL_GetTick() - g_last_telemetry_time >= TELEMETRY_PERIOD_MS) {

UpdateAndDisplayTelemetry();

g_last_telemetry_time = HAL_GetTick();

}

}

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int _write(int file, char *ptr, int len)

{

HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 100);

return len;

}

void UpdateMotorRamp(void)

{

// Sadece belirli aralıklarla çalış (örn: her 10ms'de bir)

if (HAL_GetTick() - g_last_ramp_time < RAMP_UPDATE_PERIOD_MS) {

return;

}

g_last_ramp_time = HAL_GetTick();



uint32_t ramp_step = 50; // Her 10ms'de PWM ne kadar artsın/azalsın



// Mevcut PWM'i hedefe doğru bir adım yaklaştır

if (g_current_pwm < g_target_pwm) {

g_current_pwm += ramp_step;

if (g_current_pwm > g_target_pwm) {

g_current_pwm = g_target_pwm; // Hedefi geçme

}

}

else if (g_current_pwm > g_target_pwm) {

// Eğer adım boyutu mevcut PWM'den büyükse, doğrudan sıfırla

if (g_current_pwm < ramp_step) {

g_current_pwm = 0;

} else {

g_current_pwm -= ramp_step;

}

}



// Yeni PWM değerini donanıma yaz

__HAL_TIM_SET_COMPARE(&htim1, MOTOR_PWM_CHANNEL, g_current_pwm);



// Eğer motor tamamen durduysa, mevcut durumu güncelle

if (g_current_pwm == 0) {

g_current_motor_state = MOTOR_DURDU;

}

}

void SetMotorHardware(MotorState_t state, uint32_t target_pwm_value)

{

// Motor yönünü ayarla

if (state == MOTOR_ILERI) {

HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT, MOTOR_DIR_PIN, GPIO_PIN_SET);

} else if (state == MOTOR_GERI) {

HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT, MOTOR_DIR_PIN, GPIO_PIN_RESET);

}



// Yeni hedef PWM değerini global değişkene ata

g_target_pwm = target_pwm_value;



// Gerçek durumu, sadece motor hareket ediyorsa güncelle

// Motorun durması, rampa sonunda g_current_pwm sıfır olunca belirlenecek

if (state != MOTOR_DURDU) {

g_current_motor_state = state;

}

}



void ProcessNewCommand(char cmd)

{

switch (cmd) {

case 'i': case 'I':

g_target_motor_state = MOTOR_ILERI;

g_target_motor_speed = DEFAULT_MOTOR_SPEED;

printf("Hedef: ILERI. Calistirmak icin butona basin.\n");

break;

case 'g': case 'G':

g_target_motor_state = MOTOR_GERI;

g_target_motor_speed = DEFAULT_MOTOR_SPEED;

printf("Hedef: GERI. Calistirmak icin butona basin.\n");

break;

case 'd': case 'D':

g_target_motor_state = MOTOR_DURDU;

g_target_motor_speed = 0;

printf("Hedef: DURDU.\n");

break;

default:

printf("Gecersiz komut: '%c'\n", cmd);

return;

}

fflush(stdout);



}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)

{

if (huart->Instance == USART2) {

g_received_command = g_rx_buffer[0];

g_new_command_received = true;

HAL_UART_Receive_IT(&huart2, g_rx_buffer, RX_BUFFER_SIZE);

}

}



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)

{

if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {

/*

* Z-Index sinyalini (sıfırlama ve mesaj) dikkate almamak için

* bu fonksiyonun içi kasıtlı olarak boş bırakılmıştır.

*/

}

}

void I2C_Scanner(void)

{

printf("I2C Hatti Taraniyor...\n");

fflush(stdout);



uint8_t device_count = 0;

for(uint8_t i = 1; i < 128; i++)

{

// HAL_I2C_IsDeviceReady fonksiyonu, belirtilen adrese bir sinyal gönderir.

// Eğer cihaz o adreste ise ve cevap verirse (ACK), fonksiyon HAL_OK döner.

// Adres 7-bit olduğu için 1 bit sola kaydırılır.

if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i << 1), 2, 10) == HAL_OK)

{

// Cihaz bulundu, adresini HEX formatında yazdır.

printf("--> I2C cihazi bulundu, adres: 0x%02X\n", i);

fflush(stdout);

device_count++;

}

}



if (device_count == 0)

{

printf("--> Tarama tamamlandi, hatta bagli cihaz bulunamadi.\n");

printf("--> Lutfen baglantilari, GUC, GND ve PULL-UP direncini kontrol edin.\n");

fflush(stdout);

}

else

{

printf("--> Tarama tamamlandi. Toplam %d cihaz bulundu.\n", device_count);

fflush(stdout);

}

}



void UpdateAndDisplayTelemetry(void)

{

// --- 1. BÖLÜM: ENKODER HESAPLAMALARI (Bu kısım doğru, dokunmuyoruz) ---

int32_t current_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2) * -1;

int32_t count_delta = current_count - g_encoder_last_count;



// Hız Filtresi (Moving Average)

float raw_speed_rpm = ((float)count_delta / ENCODER_PPR / (TELEMETRY_PERIOD_MS / 1000.0f)) * 60.0f;

g_speed_history[g_speed_history_index] = raw_speed_rpm;

g_speed_history_index = (g_speed_history_index + 1) % MOVING_AVG_SIZE;

float total_speed = 0.0f;

for (int i = 0; i < MOVING_AVG_SIZE; i++) {

total_speed += g_speed_history[i];

}

float smoothed_speed_rpm = total_speed / MOVING_AVG_SIZE;

float total_distance_cm = ((float)current_count / ENCODER_PPR) * WHEEL_CIRCUMFERENCE_CM;





// --- 2. BÖLÜM: VERİLERİ YAZDIRMA (DÜZELTİLMİŞ HALİ) ---



// Enkoder verilerini yazdır

printf("Enkoder -> Mesafe: %7.2f cm | Hiz: %6.1f RPM\n", total_distance_cm, smoothed_speed_rpm);



// IMU verilerini yazdır

if (imu_baslatildi) {

// DÜZELTME 1: Doğrudan 3 eksen Euler açılarını okuyan basit fonksiyona geri dönüldü.

bno055_vector_t euler = bno055_getVectorEuler();



// DÜZELTME 2: Kalibrasyon fonksiyonu doğru şekilde çağrıldı ve doğru struct kullanıldı.

bno055_calibration_state_t cal = bno055_getCalibrationState();



// DÜZELTME 3: printf güncellenerek 3 eksen de (Yaw, Roll, Pitch) yazdırıldı.

printf("IMU -> Yaw: %6.1f | Roll: %6.1f | Pitch: %6.1f | Kalibrasyon [S:%d J:%d I:%d M:%d]\n",

euler.x, euler.y, euler.z,

cal.sys, cal.gyro, cal.accel, cal.mag);

}



// --- 3. BÖLÜM: BAŞLANGIÇ NOKTASI KONTROLÜ (Bu kısım doğru, dokunmuyoruz) ---

if (fabs(total_distance_cm) < 0.5f && !g_start_point_message_sent)

{

printf("--- Başlangıç noktasına geri dönüldü! ---\n");

g_start_point_message_sent = true;

}

else if (fabs(total_distance_cm) > 2.0f)

{

g_start_point_message_sent = false;

}



g_encoder_last_count = current_count;

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

__disable_irq();

printf("\n!!! KRITIK HATA OLUSTU !!! Sistem durduruldu.\n");

while (1){}

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

printf("Hata: Gecersiz parametre: %s, satir %lu\r\n", file, line);

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
