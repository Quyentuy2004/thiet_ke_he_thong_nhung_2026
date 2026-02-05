/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "printf.h"
#include "i2c-lcd.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ADC_MAX     4095.0f
#define VDDA        3.3f    // nếu bạn đã đo VDDA thì thay bằng biến


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
#define CHARGE_PORT GPIOA
#define CHARGE_PIN  GPIO_PIN_3   // bật/tắt sạc
#define MODE_PORT   GPIOA
#define MODE_PIN    GPIO_PIN_2   // chọn CC/CV (tuỳ mạch)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t ADC_ReadChannelRaw(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel      = channel;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;

    HAL_ADC_ConfigChannel(hadc, &sConfig);

    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 10);
    uint16_t raw = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);

    return raw;
}
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart2;
//uint16_t ADC_Read_PA0(void)
//{
//  ADC_ChannelConfTypeDef sConfig = {0};
//
//  sConfig.Channel      = ADC_CHANNEL_0;          // PA0
//  sConfig.Rank         = 1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
//
//  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
//
//  HAL_ADC_Start(&hadc1);
//  HAL_ADC_PollForConversion(&hadc1, 10);
//  uint16_t raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
//  HAL_ADC_Stop(&hadc1);
//
//  return raw; // 0..4095
//}
float ADC_Raw_to_mV(uint16_t raw)
{
  return raw * 3300 / (4095*1.0); // giả sử VDDA=3.3V
}

uint16_t ADC_Read_PA0(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel      = ADC_CHANNEL_6;          // PA0
  sConfig.Rank         = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;

  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 10);
  uint16_t raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  return raw; // 0..4095
}


uint16_t ADC_Read_PA1(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel      = ADC_CHANNEL_7;          // PA0
  sConfig.Rank         = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;

  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 10);
  uint16_t raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  return raw; // 0..4095
}

//#define ADC_MAX        4095.0f
#define VREFINT_TYP   1.21f   // datasheet STM32F411

float ADC_GetVDDA(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel      = ADC_CHANNEL_VREFINT;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES; // bắt buộc dài

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t vref_raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    if (vref_raw == 0) return 0;

    return VREFINT_TYP * ADC_MAX / vref_raw;
}
float ADC_RawToVolt(uint16_t raw)
{
    float vdda = ADC_GetVDDA();
    return raw * vdda / ADC_MAX;
}
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  uint32_t raw1=0;
uint32_t raw2=0;
float mv1  = 0.0;
 float mv2  = 0.0;
 float Voltage=0.0;
 float Current=0.0;
 lcd_init();
 lcd_clear();
 lcd_put_cur(0, 0);
 lcd_send_string("Set up");
 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
		      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		      HAL_Delay(5000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	  		      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		      raw1=0;
		      raw2=0;
		      raw1 = ADC_Read_PA0();
		      raw2 = ADC_Read_PA1();
		     		    mv1  = ADC_RawToVolt(raw1);
		     		    mv2  = ADC_RawToVolt(raw2);
		     		    myPrintf(&huart2, "PA0 raw=%u  V=%f V\r\n", raw1, mv1);
		     		    HAL_Delay(500);
		     		    myPrintf(&huart2, "PA1 raw=%u  V=%f V\r\n", raw2, mv2);
		     		   HAL_Delay(500);
		     		   Voltage=mv2-mv1/1.5;
					   Current=mv1/(1.5*2.4);
					   myPrintf(&huart2, "Vol=%.2f V  Curr=%.2f A\r\n", Voltage, Current);
					   char mode[20];
					   	char parameter[20];
					   	sprintf(mode, "setup mode");

					   	sprintf(parameter, "V=%.2f V  A=%.2f A", Voltage,Current);

					   	lcd_put_cur(0,0);
					   	lcd_send_string("                "); // 16 spaces
					   	lcd_put_cur(0,0);
					   	lcd_send_string(parameter);

					   	lcd_put_cur(1,0);
					   	lcd_send_string("                ");
					   	lcd_put_cur(1,0);
					   	lcd_send_string(mode);
					   HAL_Delay(500);
		     		   while (Voltage <= 2.2f && Voltage >= 1.5) {
		     			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
		     			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		     			 raw1=0;
		     			 myPrintf(&huart2," CC mode\n");
		     			HAL_Delay(500);
		     					      raw2=0;
		     					      raw1 = ADC_Read_PA0();// do dien ap in
		     					      raw2 = ADC_Read_PA1();// do dien ap rsunt
		     					     		    mv1  = ADC_RawToVolt(raw1);
		     					     		    mv2  = ADC_RawToVolt(raw2);
		     					     		    myPrintf(&huart2, "PA0 raw=%u  V=%f V\r\n", raw1, mv1);
		     					     		    HAL_Delay(500);
		     					     		    myPrintf(&huart2, "PA1 raw=%u  V=%f V\r\n", raw2, mv2);
		     					     		   HAL_Delay(500);
		     					     		 Voltage=mv2-mv1/1.5;
		     								   Current=mv1/(1.5*2.4);
		     								   myPrintf(&huart2, "Vol=%.2f V  Curr=%.2f A\r\n", Voltage, Current);
		     								  char mode[20];
		     								 					   	char parameter[20];
		     								 					   	sprintf(mode, "CC mode");

		     								 					   	sprintf(parameter, "V=%.2f V  A=%.2f A", Voltage,Current);

		     								 					   	lcd_put_cur(0,0);
		     								 					   	lcd_send_string("                "); // 16 spaces
		     								 					   	lcd_put_cur(0,0);
		     								 					   	lcd_send_string(parameter);
		     								 		lcd_put_cur(1,0);
		     								 		lcd_send_string("                ");
		     								  	lcd_put_cur(1,0);
		     								   	lcd_send_string(mode);
		     								   HAL_Delay(500);
		     								  if (Voltage > 2.20f) {
		     									 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
		     								        myPrintf(&huart2,">> Switch to CC mode");
		     								        break;
		     								      }
		     		   }
		     		  while (Voltage > 2.1f) {
		     			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
		     			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		     			 myPrintf(&huart2," CV mode\n");

		     		   HAL_Delay(500);
		     		   raw1=0;
		     		   		     					      raw2=0;
		     		   		     					      raw1 = ADC_Read_PA0();
		     		   		     					      raw2 = ADC_Read_PA1();
		     		   		     					       mv1  = ADC_RawToVolt(raw1);
		     		   		     					      mv2  = ADC_RawToVolt(raw2);
		     		   		     					      myPrintf(&huart2, "PA0 raw=%u  V=%f V\r\n", raw1, mv1);
		     		   		     					     HAL_Delay(500);
		     		   		     					      myPrintf(&huart2, "PA1 raw=%u  V=%f V\r\n", raw2, mv2);
		     		   		     					      char mode[20];
		     		   		     					     	char parameter[20];
		     		   		     					      	sprintf(mode, "CV mode");
		     		   		     					      	sprintf(parameter, "V=%.2f V  A=%.2f A", Voltage,Current);
		     		   		     					     	 lcd_put_cur(0,0);
		     		   		     							 lcd_send_string("                "); // 16 spaces
		     		   		     					       	lcd_put_cur(0,0);
		     		   		     					    	lcd_send_string(parameter);
		     		   		     					     	 lcd_put_cur(1,0);
		     		   		     					     	lcd_send_string("                ");
		     		   		     					    lcd_put_cur(1,0);
		     		   		     					   	lcd_send_string(mode);
		     		   		     					    HAL_Delay(500);
		     		   		     					     		Voltage=mv2-mv1/1.5;
		     		   		     								   Current=mv1/(1.5*2.4);
		     		   		     								   myPrintf(&huart2, "Vol=%.2f V  Curr=%.2f A\r\n", Voltage, Current);
		     		   		     								   HAL_Delay(500);
		     		    if (Current < 0.05f) { // ngắt khi < 50mA (ví dụ)
		     		    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
		     		    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		     		      myPrintf(&huart2, "!! Charge Complete. Charger OFF.");
		     		      while (1) {    HAL_Delay(500); }
		     		    }
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
