/*
 1. Link: https://github.com/Ammaarah2605/Practical-4.git
 2. Group Number: # 46
 3. Members:  LDWZAY001 HNSAMM001
 */
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f4xx.h"
#include "lcd_stm32f4.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Task 2: Assign values to variables
#define NS 128                    // Number of samples in LUT
#define TIM2CLK 16000000         // STM Clock frequency: 16 MHz (from .ioc file)
#define F_SIGNAL 1000            // Frequency of output analog signal (1kHz)
#define DEBOUNCE_DELAY 200       // 200ms debounce delay for button
#define TOTAL_WAVEFORMS 6        // Total number of waveforms (3 basic + 3 audio)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// Task 1: LUT arrays with 128 samples each (0-4095 range for 12-bit resolution)
uint32_t Sin_LUT[NS] = {
		2048, 2149, 2250, 2350, 2450, 2549, 2646, 2742, 2837, 2929,
		3020, 3108, 3193, 3275, 3355, 3431, 3504, 3574, 3639, 3701,
		3759, 3812, 3861, 3906, 3946, 3982, 4013, 4039, 4060, 4076,
		4087, 4094, 4095, 4091, 4082, 4069, 4050, 4026, 3998, 3965,
		3927, 3884, 3837, 3786, 3730, 3671, 3607, 3539, 3468, 3394,
		3316, 3235, 3151, 3064, 2975, 2883, 2790, 2695, 2598, 2500,
		2400, 2300, 2199, 2098, 1997, 1896, 1795, 1695, 1595, 1497,
		1400, 1305, 1212, 1120, 1031, 944, 860, 779, 701, 627, 556,
		488, 424, 365, 309, 258, 211, 168, 130, 97, 69, 45, 26, 13,
		4, 0, 1, 8, 19, 35, 56, 82, 113, 149, 189, 234, 283, 336,
		394, 456, 521, 591, 664, 740, 820, 902, 987, 1075, 1166,
		1258, 1353, 1449, 1546, 1645, 1745, 1845, 1946, 2047
};

uint32_t Saw_LUT[NS] = {
		0, 32, 64, 97, 129, 161, 193, 226, 258, 290, 322, 355, 387,
		419, 451, 484, 516, 548, 580, 613, 645, 677, 709, 742, 774,
		806, 838, 871, 903, 935, 967, 1000, 1032, 1064, 1096, 1129,
		1161, 1193, 1225, 1258, 1290, 1322, 1354, 1386, 1419, 1451,
		1483, 1515, 1548, 1580, 1612, 1644, 1677, 1709, 1741, 1773,
		1806, 1838, 1870, 1902, 1935, 1967, 1999, 2031, 2064, 2096,
		2128, 2160, 2193, 2225, 2257, 2289, 2322, 2354, 2386, 2418,
		2451, 2483, 2515, 2547, 2580, 2612, 2644, 2676, 2709, 2741,
		2773, 2805, 2837, 2870, 2902, 2934, 2966, 2999, 3031, 3063,
		3095, 3128, 3160, 3192, 3224, 3257, 3289, 3321, 3353, 3386,
		3418, 3450, 3482, 3515, 3547, 3579, 3611, 3644, 3676, 3708,
		3740, 3773, 3805, 3837, 3869, 3902, 3934, 3966, 3998, 4031,
		4063, 4095
};

uint32_t Triangle_LUT[NS] = {
		0, 64, 128, 192, 256, 320, 384, 448, 512, 576, 640, 704,
		768, 832, 896, 960, 1024, 1088, 1152, 1216, 1280, 1344,
		1408, 1472, 1536, 1600, 1664, 1728, 1792, 1856, 1920,
		1984, 2048, 2111, 2175, 2239, 2303, 2367, 2431, 2495, 2559,
		2623, 2687, 2751, 2815, 2879, 2943, 3007, 3071, 3135, 3199,
		3263, 3327, 3391, 3455, 3519, 3583, 3647, 3711, 3775, 3839,
		3903, 3967, 4031, 4095, 4031, 3967, 3903, 3839, 3775, 3711,
		3647, 3583, 3519, 3455, 3391, 3327, 3263, 3199, 3135, 3071,
		3007, 2943, 2879, 2815, 2751, 2687, 2623, 2559, 2495, 2431,
		2367, 2303, 2239, 2175, 2111, 2048, 1984, 1920, 1856, 1792,
		1728, 1664, 1600, 1536, 1472, 1408, 1344, 1280, 1216, 1152,
		1088, 1024, 960, 896, 832, 768, 704, 640, 576, 512, 448,
		384, 320, 256, 192, 128, 64
};

// Arrays for audio files generated in Matlab
uint32_t Piano_LUT[NS] = {
		1772, 1669, 1563, 1459, 1364, 1280, 1205, 1137,
		1074, 1016,  963,  913,  866,  826,  798,  781,
		774,  774,  780,  789,  801,  815,  836,  868,
		914,  973, 1043, 1124, 1211, 1301, 1391, 1482,
		1575, 1672, 1775, 1885, 2000, 2118, 2237, 2359,
		2480, 2598, 2712, 2823, 2933, 3045, 3158, 3272,
		3382, 3487, 3583, 3671, 3752, 3827, 3893, 3949,
		3994, 4031, 4059, 4079, 4092, 4095, 4088, 4069,
		4040, 4001, 3954, 3897, 3829, 3750, 3664, 3572,
		3475, 3370, 3256, 3135, 3009, 2881, 2751, 2621,
		2489, 2358, 2229, 2097, 1962, 1823, 1684, 1546,
		1412, 1286, 1171, 1066,  972,  885,  805,  732,
		670,  616,  572,  540,  523,  518,  527,  546,
		574,  608,  650,  703,  770,  852,  951, 1061,
		1180, 1306, 1435, 1567, 1702, 1843, 1991, 2147,
		2311, 2481, 2653, 2824, 2996, 3165, 3331, 3497,
};

uint32_t Guitar_LUT[NS] = {
		404,  446,  460,  473,  488,  497,  505,  516,
		520,  504,  491,  532,  621,  679,  674,  687,
		784,  913,  977,  970,  977, 1058, 1155, 1179,
		1139, 1129, 1197, 1293, 1337, 1310, 1268, 1270,
		1304, 1320, 1307, 1289, 1281, 1299, 1341, 1357,
		1316, 1263, 1273, 1355, 1426, 1432, 1407, 1408,
		1446, 1481, 1479, 1493, 1585, 1694, 1722, 1686,
		1683, 1737, 1772, 1758, 1757, 1816, 1899, 1940,
		1902, 1835, 1822, 1873, 1932, 1961, 1951, 1933,
		1940, 1978, 2027, 2048, 2036, 2048, 2108, 2165,
		2174, 2140, 2107, 2131, 2215, 2286, 2284, 2261,
		2322, 2459, 2556, 2560, 2551, 2632, 2798, 2948,
		3012, 3033, 3096, 3203, 3294, 3352, 3411, 3475,
		3532, 3594, 3664, 3703, 3692, 3674, 3709, 3776,
		3788, 3739, 3726, 3785, 3841, 3829, 3784, 3792,
		3870, 3941, 3943, 3916, 3941, 4029, 4095, 4080,
};

uint32_t Drum_LUT[NS] = {
		336,  465,  331,   95,    1,  108,  355,  536,
		535,  509,  530,  517,  485,  484,  450,  324,
		204,  198,  299,  488,  617,  531,  401,  438,
		559,  608,  628,  733,  845,  778,  560,  478,
		624,  821,  964,  984,  915,  917, 1019, 1127,
		1189, 1263, 1445, 1677, 1849, 1983, 2084, 2093,
		2034, 1990, 2048, 2150, 2153, 2138, 2218, 2329,
		2422, 2459, 2411, 2368, 2370, 2395, 2539, 2713,
		2723, 2695, 2755, 2874, 3028, 3117, 3123, 3117,
		3060, 2985, 2998, 3026, 3004, 3084, 3326, 3541,
		3556, 3424, 3316, 3276, 3239, 3241, 3399, 3639,
		3743, 3750, 3821, 3805, 3573, 3383, 3351, 3323,
		3315, 3280, 3141, 3071, 3081, 3033, 3012, 3071,
		3041, 2882, 2757, 2725, 2701, 2597, 2500, 2597,
		2744, 2661, 2518, 2574, 2613, 2431, 2224, 2221,
		2496, 2766, 2662, 2438, 2361, 2212, 2040, 2030,
};

// Current LUT pointer and waveform index
uint32_t *current_LUT = Sin_LUT;
uint8_t waveform_index = 0;
const char* waveform_names[] = {"Sine", "Sawtooth", "Triangle", "Piano", "Guitar", "Drum"};

// Task 3: Calculate TIM2_Ticks
// TIM2_Ticks = TIM2CLK / (NS * F_SIGNAL) - 1
uint32_t TIM2_Ticks = (TIM2CLK / (NS * F_SIGNAL)) - 1;

uint32_t DestAddress = (uint32_t) &(TIM3->CCR3); // Write LUT TO TIM3->CCR3 to modify PWM duty cycle

// Button debouncing variables
uint32_t last_button_time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void EXTI0_IRQHandler(void);
void lcd_print(const char* str);
void lcd_clear(void);
void lcd_init(void);
void switch_waveform(void);
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  // Task 4.1: Start TIM3 in PWM mode on channel 3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  // Task 4.2: Start TIM2 in Output Compare (OC) mode on channel 1
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);

  // Configure TIM2 for the calculated frequency
  htim2.Instance->ARR = TIM2_Ticks;
  htim2.Instance->CCR1 = TIM2_Ticks / 2;

  // Task 4.3: Start DMA in IT mode on TIM2->CH1
  HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)current_LUT, DestAddress, NS);

  // Task 4.4: Write current waveform to LCD
  lcd_init();
  lcd_clear();
  lcd_print(waveform_names[waveform_index]);

  // Task 4.5: Enable DMA to start transfer from LUT to CCR
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Main loop - all processing is done via DMA and interrupts
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* TIM2_CH1 DMA Init */
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_tim2_ch1.Instance = DMA1_Stream5;
  hdma_tim2_ch1.Init.Channel = DMA_CHANNEL_3;         // TIM2_CH1 is on channel 3
  hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH; // Memory -> TIM3->CCR3
  hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;    // Peripheral address fixed
  hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;        // Memory address increments
  hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_tim2_ch1.Init.Mode = DMA_CIRCULAR;            // Repeat LUT automatically
  hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_tim2_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  if (HAL_DMA_Init(&hdma_tim2_ch1) != HAL_OK)
  {
      Error_Handler();
  }

  /* Link DMA handle to TIM2 handle */
  __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // -------------------------------
  // LCD pins configuration
  // -------------------------------
  // Configure PC14 (RS) and PC15 (E) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Configure PB8 (D4) and PB9 (D5) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure PA12 (D6) and PA15 (D7) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Set all LCD pins LOW initially
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12 | GPIO_PIN_15, GPIO_PIN_RESET);

  // -------------------------------
  // Button0 configuration (PA0)
  // -------------------------------
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // Interrupt on rising edge
  GPIO_InitStruct.Pull = GPIO_PULLUP;         // Use pull-up resistor
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Enable and set EXTI line 0 interrupt priority
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// LCD functions (implement based on your LCD library)
void lcd_init(void) {
    // Your LCD initialization code here
    // Initialize the LCD display
}

void lcd_clear(void) {
    // Your LCD clear code here
    // Clear the LCD display
}

void lcd_print(const char* str) {
    // Your LCD print code here
    // Print string to LCD display
}

// Function to handle waveform switching
void switch_waveform(void) {
    // Disable DMA transfer and abort IT
    __HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
    HAL_DMA_Abort_IT(&hdma_tim2_ch1);

    // Cycle to next waveform
    waveform_index = (waveform_index + 1) % TOTAL_WAVEFORMS;

    // Update current LUT pointer using switch statement
    switch(waveform_index) {
        case 0:
            current_LUT = Sin_LUT;
            break;
        case 1:
            current_LUT = Saw_LUT;
            break;
        case 2:
            current_LUT = Triangle_LUT;
            break;
        case 3:
            current_LUT = Piano_LUT;
            break;
        case 4:
            current_LUT = Guitar_LUT;
            break;
        case 5:
            current_LUT = Drum_LUT;
            break;
        default:
            current_LUT = Sin_LUT;
            waveform_index = 0;
            break;
    }

    // Restart DMA with new LUT
    HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)current_LUT, DestAddress, NS);

    // Update LCD display with current waveform name
    lcd_clear();
    lcd_print(waveform_names[waveform_index]);

    // Re-enable DMA transfer
    __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
}

// Task 5: Button interrupt handler with debouncing
void EXTI0_IRQHandler(void) {
    uint32_t current_time = HAL_GetTick();

    // Debounce check - eliminate noise from bouncing but remain responsive
    if ((current_time - last_button_time) > DEBOUNCE_DELAY) {
        last_button_time = current_time;

        // Switch to next waveform
        switch_waveform();
    }

    // Clear interrupt flags
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

