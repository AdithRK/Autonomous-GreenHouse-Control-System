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
#include <string.h>
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
uint16_t result = 0;
float tCelsius = 0;
float tFahrenheit = 0;
float RH = 0;
int humidity = 0;
int temperature = 0.0;
#define CLK_PIN     11   // PA11 (PB7 in original TM1637 — adjust as needed)

#define DIO_PIN     12   // PA12 (PB6 in original — adjust as needed)

const uint8_t digitToSegment[] = {

    0x3F, 0x06, 0x5B, 0x4F,

    0x66, 0x6D, 0x7D, 0x07,

    0x7F, 0x6F

};

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
    GPIOA->MODER &= ~(3 << (6 * 2));       // Clear PA6 mode bits
	GPIOA->MODER |=  (1 << (6 * 2));       // Set PA6 as general-purpose output
	GPIOA->ODR |= (1 << 6);  // Turn ON
    // motor
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
void adc_init(void)
{
    RCC->AHB1ENR |= (1 << 0);      // Enable GPIOA clock
    GPIOA->MODER |= (3 << (5 * 2));  // PA5 = analog mode (MODER5[1:0] = 11)
    RCC->APB2ENR |= (1 << 8);      // Enable ADC1 clock
    ADC1->SQR3 = 5;                // Select channel 5 (PA5)
    ADC1->CR1 = 0;                 // Disable scan mode
    ADC1->CR2 |= (1 << 1);         // Enable continuous conversion
    ADC1->CR2 |= (1 << 0);         // Enable ADC1
}

// === ADC1 Read Function ===
uint16_t adc_read(void)
{
    ADC1->CR2 |= (1 << 30);           // Start conversion
    while (!(ADC1->SR & (1 << 1)));   // Wait for EOC
    return ADC1->DR;
}

// === LED Bar Initialization (PC0–PC5) ===
void led_init(void)
{
    RCC->AHB1ENR |= (1 << 2);  // Enable GPIOC clock
    for (int i = 0; i <= 5; i++)
        GPIOC->MODER |= (1 << (i * 2));  // PCx as output
}

// === LED Bar Graph Based on ADC Input ===
void led_control(uint16_t adc_value)
{
    GPIOC->ODR &= ~((1 << 0) | (1 << 1) | (1 << 3) | (1 << 4) | (1 << 5)); // Clear PC0,1,3,4,5

    if (adc_value < 500) {
    	GPIOC->ODR &= ~((1 << 0) | (1 << 1) | (1 << 3) | (1 << 4) | (1 << 5)); // Clear PC0,1,3,4,5

    }
    else if (adc_value < 700) {
    	GPIOC->ODR |= (1 << 0);

    }
    else if (adc_value < 1100) {
    	GPIOC->ODR |= (1 << 0) | (1 << 1);

    }
    else if (adc_value < 1800) {
        GPIOC->ODR |= (1 << 0) | (1 << 1) | (1 << 3);
    }
    else if (adc_value < 2200) {
    	GPIOC->ODR |= (1 << 0) | (1 << 1) | (1 << 3) | (1 << 4);

    }
    else if (adc_value < 2000) {
    	GPIOC->ODR |= (1 << 0) | (1 << 1) | (1 << 3) | (1 << 4) | (1 << 5);

    }
    else {
    	GPIOC->ODR |= (1 << 0) | (1 << 1) | (1 << 3) | (1 << 4) | (1 << 5);
    }
}
void delay_us(int us) { for (volatile int i = 0; i < us * 30; i++); }

void set_pin_output(uint32_t pin) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER &= ~(0x3 << (pin * 2));

    GPIOA->MODER |=  (0x1 << (pin * 2));

    GPIOA->PUPDR &= ~(0x3 << (pin * 2));

}

void set_pin_input(uint32_t pin) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER &= ~(0x3 << (pin * 2));

    GPIOA->PUPDR &= ~(0x3 << (pin * 2));

    GPIOA->PUPDR |=  (0x1 << (pin * 2)); // pull-up

}

void write_pin(uint32_t pin, int value) {

    if (value) GPIOA->BSRR = (1 << pin);

    else       GPIOA->BSRR = (1 << (pin + 16));

}

int read_pin(uint32_t pin) {

    return (GPIOA->IDR & (1 << pin)) ? 1 : 0;

}

void tm_start() {

    set_pin_output(DIO_PIN);

    set_pin_output(CLK_PIN);

    write_pin(DIO_PIN, 1); write_pin(CLK_PIN, 1); delay_us(2);

    write_pin(DIO_PIN, 0); delay_us(2); write_pin(CLK_PIN, 0);

}

void tm_stop() {

    write_pin(CLK_PIN, 0); write_pin(DIO_PIN, 0); delay_us(2);

    write_pin(CLK_PIN, 1); delay_us(2); write_pin(DIO_PIN, 1); delay_us(2);

}

int tm_write_byte(uint8_t b) {

    for (int i = 0; i < 8; i++) {

        write_pin(CLK_PIN, 0);

        write_pin(DIO_PIN, b & 0x01);

        delay_us(3);

        write_pin(CLK_PIN, 1); delay_us(3);

        b >>= 1;

    }

    write_pin(CLK_PIN, 0);

    set_pin_input(DIO_PIN);

    delay_us(3); write_pin(CLK_PIN, 1); delay_us(3);

    int ack = read_pin(DIO_PIN);

    write_pin(CLK_PIN, 0);

    set_pin_output(DIO_PIN);

    return ack;

}

void tm_set_brightness(uint8_t brightness) {

    tm_start();

    tm_write_byte(0x88 | (brightness & 0x07));

    tm_stop();

}

void tm_display_digits(uint8_t digits[4]) {

    tm_start(); tm_write_byte(0x40); tm_stop();

    tm_start(); tm_write_byte(0xC0);

    for (int i = 0; i < 4; i++) tm_write_byte(digitToSegment[digits[i] % 10]);

    tm_stop();

}

// ==== Keypad + LEDs ====

const char keymap[4][4] = {

    {'1', '2', '3', 'A'},

    {'4', '5', '6', 'B'},

    {'7', '8', '9', 'C'},

    {'*', '0', '#', 'D'}

};

char entered[5];

int key_index = 0;

void GPIO_Init(void) {

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    // PC10 = Green LED, PC11 = Red LED

    GPIOC->MODER &= ~(0xF << 20);

    GPIOC->MODER |= (1 << 20) | (1 << 22);

    GPIOC->ODR &= ~((1 << 10) | (1 << 11));

    // PB0–PB3 (rows) as output

    for (int i = 0; i < 4; i++) {

        GPIOB->MODER &= ~(3 << (i * 2));

        GPIOB->MODER |= (1 << (i * 2));

        GPIOB->ODR |= (1 << i);

    }

    // PB4–PB7 (cols) as input with pull-up

    for (int i = 4; i < 8; i++) {

        GPIOB->MODER &= ~(3 << (i * 2));

        GPIOB->PUPDR &= ~(3 << (i * 2));

        GPIOB->PUPDR |= (1 << (i * 2));

    }

}

char Keypad_Scan(void) {

    for (int row = 0; row < 4; row++) {

        GPIOB->ODR |= 0x0F;

        GPIOB->ODR &= ~(1 << row);

        HAL_Delay(10);

        for (int col = 0; col < 4; col++) {

            if ((GPIOB->IDR & (1 << (col + 4))) == 0) {

            	HAL_Delay(10);

                while ((GPIOB->IDR & (1 << (col + 4))) == 0);

                return keymap[row][col];

            }

        }

    }

    return 0;

}

// ==== Random Generator ====

static uint32_t lcg_seed = 123456789;

uint32_t rand32() {

    lcg_seed = (1103515245 * lcg_seed + 12345) & 0x7FFFFFFF;

    return lcg_seed;

}

void generate_new_code(uint8_t digits[4], char code_str[5]) {

    uint32_t r = rand32() % 10000;

    digits[0] = (r / 1000) % 10;

    digits[1] = (r / 100) % 10;

    digits[2] = (r / 10) % 10;

    digits[3] = r % 10;

    for (int i = 0; i < 4; i++) code_str[i] = digits[i] + '0';

    code_str[4] = '\0';
    LcdFxn(0, 0x01);
	lprint(0x80, "password change");
	lprint(0xC0, "press *");

    tm_display_digits(digits);


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
  LcdInit();
  adc_init();     // LDR input via PA5
  led_init();     // LED bar PC0–PC5
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  /* USER CODE END 2 */
  set_pin_output(CLK_PIN);

   set_pin_output(DIO_PIN);

   tm_set_brightness(7);



   GPIO_Init();

   lprint(0x80, "Welcome");

   uint8_t digits[4];

   char code_str[5];
   HAL_Delay(1000);


   generate_new_code(digits, code_str);

    uint32_t seconds_counter = 0;

   while (1) {

       char key = Keypad_Scan();

        if (++seconds_counter >= 400) { // ~20s if delayms(10)

            generate_new_code(digits, code_str);

            seconds_counter = 0;

        }

       if (key != 0) {

           if (key == '*') {

               key_index = 0;

               memset(entered, 0, sizeof(entered));

               lprint(0x80, "Security Code     ");

               lprint(0xC0, "                  ");

           } else if (key == '#' && key_index > 0) {

               entered[key_index] = '\0';

               if (strcmp(entered, code_str) == 0) {

                   lprint(0x80, "Access Granted    ");

                   GPIOC->ODR |= (1 << 10);  // Green ON

                   GPIOC->ODR &= ~(1 << 11);
                   HAL_Delay(1000);
                   break;


               } else {

                   lprint(0x80, "Access Denied      ");
                   lprint(0xC0, "Try Again!       ");

//                   generate_new_code(digits, code_str);

                   GPIOC->ODR |= (1 << 11);  // Red ON

                   GPIOC->ODR &= ~(1 << 10);

               }

               HAL_Delay(1000);

               key_index = 0;

               memset(entered, 0, sizeof(entered));

               lprint(0x80, "Security Code      ");

               lprint(0xC0, "                    ");

           } else if (key_index < 4) {

               entered[key_index++] = key;

               char buf[17] = "Entered: ";

               strncpy(&buf[9], entered, key_index);

               buf[9 + key_index] = '\0';

               lprint(0xC0, buf);

           }

       }

       HAL_Delay(10);

   }

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

	  	        if (humidity > 70)
	  	        {
	  	            GPIOA->ODR &= ~(1 << 4); // Turn ON bulb on PA4 (active-low)
	  	            GPIOA->ODR &= ~(1 << 6); // Turn ON bulb on PA6 (active-low)
	  	        }
	  	        else if (humidity > 60)
	  	        {
	  	            GPIOA->ODR |=  (1 << 4); // Turn OFF bulb on PA4 (active-low)
	  	            GPIOA->ODR &= ~(1 << 6); // Turn ON  bulb on PA6 (active-low)
	  	        }
	  	        else
	  	        {
	  	            GPIOA->ODR |=  (1 << 4); // Turn OFF bulb on PA4 (active-low)
	  	            GPIOA->ODR |=  (1 << 6); // Turn OFF bulb on PA6 (active-low)
	  	        }

		  // Update LCD Line 1: Temp
			 sprintf(line1, "Temp: %d C      ", temperature);
			 lprint(0x80, line1);  // Line 1

			 if (temperature >= 29)
			 {
				 motor_on();
				 sprintf(line2, "Mild Hot");
				 lprint(0xC0, line2);  // Line 2
			 }
			 else
			 {
				 motor_off();
				 sprintf(line2, "Cool            ");
				 lprint(0xC0, line2);  // Line 2
			 }
	  	                 }
	               }
	          result = adc_read();  // Read light level
	          led_control(result);  // LED bar update
	          for (volatile uint32_t j = 0; j < 20000; j++) {}  // Delay

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
