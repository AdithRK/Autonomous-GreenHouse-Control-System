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

#include "tim.h" 

#include "gpio.h" 

#include "stm32f405xx.h" 

#include <stdio.h> 

#include <stdint.h> 

#include "lcd.h" 

/* Private includes ----------------------------------------------------------*/ 

/* USER CODE BEGIN Includes */ 

 

/* USER CODE END Includes */ 

 

/* Private typedef -----------------------------------------------------------*/ 

/* USER CODE BEGIN PTD */ 

 

/* USER CODE END PTD */ 

 

/* Private define ------------------------------------------------------------*/ 

/* USER CODE BEGIN PD */ 

/* USER CODE END PD */ 

 

/* Private macro -------------------------------------------------------------*/ 

/* USER CODE BEGIN PM */ 

 

/* USER CODE END PM */ 

 

/* Private variables ---------------------------------------------------------*/ 

 

/* USER CODE BEGIN PV */ 

 

/* USER CODE END PV */ 

 

/* Private function prototypes -----------------------------------------------*/ 

void SystemClock_Config(void); 

/* USER CODE BEGIN PFP */ 

 

/* USER CODE END PFP */ 

 

/* Private user code ---------------------------------------------------------*/ 

/* USER CODE BEGIN 0 */ 

#define DHT11_PORT GPIOB 

#define DHT11_PIN GPIO_PIN_9 

#define BIN1_PIN      3   // PA3 - Motor 2 

#define BIN2_PIN      2   // PA2 - Motor 2 

uint8_t RHI, RHD, TCI, TCD, SUM; 

uint32_t pMillis, cMillis; 

float tCelsius = 0; 

float tFahrenheit = 0; 

float RH = 0; 

int humidity = 0; 

int temperature = 0.0; 

void microDelay (uint16_t delay) 

{ 

  __HAL_TIM_SET_COUNTER(&htim1, 0); 

  while (__HAL_TIM_GET_COUNTER(&htim1) < delay); 

} 

 

uint8_t DHT11_Start (void) 

{ 

  uint8_t Response = 0; 

  GPIO_InitTypeDef GPIO_InitStructPrivate = {0}; 

  GPIO_InitStructPrivate.Pin = DHT11_PIN; 

  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP; 

  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW; 

  GPIO_InitStructPrivate.Pull = GPIO_NOPULL; 

  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as output 

  HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low 

  HAL_Delay(20);   // wait for 20ms 

  HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);   // pull the pin high 

  microDelay (30);   // wait for 30us 

  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT; 

  GPIO_InitStructPrivate.Pull = GPIO_PULLUP; 

  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as input 

  microDelay (40); 

  if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) 

  { 

    microDelay (80); 

    if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1; 

  } 

  pMillis = HAL_GetTick(); 

  cMillis = HAL_GetTick(); 

  while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis) 

  { 

    cMillis = HAL_GetTick(); 

  } 

  return Response; 

} 

 

uint8_t DHT11_Read (void) 

{ 

  uint8_t a,b; 

  for (a=0;a<8;a++) 

  { 

    pMillis = HAL_GetTick(); 

    cMillis = HAL_GetTick(); 

    while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis) 

    {  // wait for the pin to go high 

      cMillis = HAL_GetTick(); 

    } 

    microDelay (40);   // wait for 40 us 

    if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low 

      b&= ~(1<<(7-a)); 

    else 

      b|= (1<<(7-a)); 

    pMillis = HAL_GetTick(); 

    cMillis = HAL_GetTick(); 

    while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis) 

    {  // wait for the pin to go low 

      cMillis = HAL_GetTick(); 

    } 

  } 

  return b; 

} 

void Relay_GPIO_Init(void) 

{ 

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // Enable GPIOA clock 

    GPIOA->MODER &= ~(3 << (4 * 2));       // Clear PA4 mode bits 

    GPIOA->MODER |=  (1 << (4 * 2));       // Set PA4 as general-purpose output 

    GPIOA->ODR |= (1 << 4);  // Turn ON 

    // Also configure PA6 as output for debug 

    GPIOA->MODER &= ~(3 << (2 * 2)); // Clear mode 

    GPIOA->MODER |=  (1 << (2 * 2)); // Set as output 

    GPIOA->MODER &= ~(3 << (3 * 2)); // Clear mode 

    GPIOA->MODER |=  (1 << (3 * 2)); // Set as output 

 

 

} 

void motor_on(void) { 

    GPIOA->ODR |=  (1 << BIN1_PIN); 

    GPIOA->ODR &= ~(1 << BIN2_PIN); 

} 

void motor_off(void) { 

    GPIOA->ODR &= ~(1 << BIN1_PIN); 

    GPIOA->ODR &= ~(1 << BIN2_PIN); 

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

  MX_TIM1_Init(); 

  Relay_GPIO_Init(); 

  /* USER CODE BEGIN 2 */ 

  HAL_TIM_Base_Start(&htim1); 

  /* USER CODE END 2 */ 

  LcdInit(); 

  /* Infinite loop */ 

  /* USER CODE BEGIN WHILE */ 

  char line1[17], line2[17];  // For LCD display 

  while (1) 

  { 

    /* USER CODE END WHILE */ 

  if(DHT11_Start()) 

      { 

        RHI = DHT11_Read(); // Relative humidity integral 

        RHD = DHT11_Read(); // Relative humidity decimal 

        TCI = DHT11_Read(); // Celsius integral 

        TCD = DHT11_Read(); // Celsius decimal 

        SUM = DHT11_Read(); // Check sum 

        if (RHI + RHD + TCI + TCD == SUM) 

        { 

          // Can use RHI and TCI for any purposes if whole number only needed 

          tCelsius = (float)TCI + (float)(TCD/10.0); 

          tFahrenheit = tCelsius * 9/5 + 32; 

          RH = (float)RHI + (float)(RHD/10.0); 

          humidity = (int)RH; 

          temperature = (int)tCelsius; 

 

                  if (humidity > 60) 

                  { 

                	  GPIOA->ODR &= ~(1 << 4); // Turn OFF 

 

                  } 

                  else 

                  { 

                	  GPIOA->ODR |= (1 << 4);  // Turn ON 

                  } 

                  // Update LCD Line 1: Temp 

                     sprintf(line1, "Temp: %d C", temperature); 

                     lprint(0x80, line1);  // Line 1 

 

                     if (temperature >= 28) 

                     { 

                    	 motor_on(); 

                         sprintf(line2, "Mild Hot"); 

                         lprint(0xC0, line2);  // Line 2 

                     } 

                     else 

                     { 

                    	 motor_off(); 

                         sprintf(line2, "Cool        "); 

                         lprint(0xC0, line2);  // Line 2 

                     } 

                 } 

             } 

 

 

        } 

 

 

      HAL_Delay(2000); 

    /* USER CODE BEGIN 3 */ 

 

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

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; 

  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI; 

  RCC_OscInitStruct.PLL.PLLM = 8; 

  RCC_OscInitStruct.PLL.PLLN = 72; 

  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; 

  RCC_OscInitStruct.PLL.PLLQ = 4; 

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) 

  { 

    Error_Handler(); 

  } 

 

  /** Initializes the CPU, AHB and APB buses clocks 

  */ 

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK 

                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2; 

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; 

  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; 

  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; 

  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; 

 

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) 

  { 

    Error_Handler(); 

  } 

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

 

#ifdef  USE_FULL_ASSERT 

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