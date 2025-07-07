#include "stm32f405xx.h"
#include "tim.h"
#include "gpio.h"
#include "lcd.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

// ==== Definitions ====
#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_9
#define BIN1_PIN 3  // PA3 - Motor 2
#define BIN2_PIN 2  // PA2 - Motor 2

#define PASSKEY "1234"
char entered[5] = {0};
int key_index = 0;

// ==== Global Variables ====
uint8_t RHI, RHD, TCI, TCD, SUM;
uint32_t pMillis, cMillis;
uint16_t result = 0;
float tCelsius = 0, tFahrenheit = 0, RH = 0;
int humidity = 0, temperature = 0;

// ==== Delay ====
void microDelay (uint16_t delay)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

// ==== DHT11 Functions ====
uint8_t DHT11_Start (void)
{
    uint8_t Response = 0;
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);
    HAL_Delay(20);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1);
    microDelay(30);
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
    microDelay(40);
    if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {
        microDelay(80);
        if ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) Response = 1;
    }
    pMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && (HAL_GetTick() - pMillis < 2));
    return Response;
}

uint8_t DHT11_Read (void)
{
    uint8_t a,b=0;
    for (a=0;a<8;a++) {
        pMillis = HAL_GetTick();
        while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && (HAL_GetTick() - pMillis < 2));
        microDelay(40);
        if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) b |= (1<<(7-a));
        else b &= ~(1<<(7-a));
        pMillis = HAL_GetTick();
        while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && (HAL_GetTick() - pMillis < 2));
    }
    return b;
}

// ==== ADC ====
void adc_init(void)
{
    RCC->AHB1ENR |= (1 << 0);
    GPIOA->MODER |= (3 << (5 * 2));
    RCC->APB2ENR |= (1 << 8);
    ADC1->SQR3 = 5;
    ADC1->CR1 = 0;
    ADC1->CR2 |= (1 << 1) | (1 << 0);
}

uint16_t adc_read(void)
{
    ADC1->CR2 |= (1 << 30);
    while (!(ADC1->SR & (1 << 1)));
    return ADC1->DR;
}

// ==== LED Bar ====
void led_init(void)
{
    RCC->AHB1ENR |= (1 << 2);
    for (int i = 0; i <= 5; i++)
        GPIOC->MODER |= (1 << (i * 2));
}

void led_control(uint16_t adc_value)
{
    GPIOC->ODR &= ~((1 << 0) | (1 << 1) | (1 << 3) | (1 << 4) | (1 << 5));
    if (adc_value < 500)       GPIOC->ODR |= (1<<0 | 1<<1 | 1<<3 | 1<<4 | 1<<5);
    else if (adc_value < 700)  GPIOC->ODR |= (1<<0 | 1<<1 | 1<<3 | 1<<4);
    else if (adc_value < 1100) GPIOC->ODR |= (1<<0 | 1<<1 | 1<<3);
    else if (adc_value < 1800) GPIOC->ODR |= (1<<0 | 1<<1);
    else if (adc_value < 2000) GPIOC->ODR |= (1<<0);
}

// ==== Relay and Motor GPIO ====
void Relay_GPIO_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= (1 << (4 * 2));
    GPIOA->ODR |= (1 << 4);
    GPIOA->MODER |= (1 << (BIN1_PIN * 2)) | (1 << (BIN2_PIN * 2));
}

void motor_on(void) {
    GPIOA->ODR |= (1 << BIN1_PIN);
    GPIOA->ODR &= ~(1 << BIN2_PIN);
}

void motor_off(void) {
    GPIOA->ODR &= ~((1 << BIN1_PIN) | (1 << BIN2_PIN));
}

// ==== Keypad ====
extern char Keypad_Scan(void); // Assuming you have it separately

// ==== Main ====
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_TIM1_Init();
    MX_GPIO_Init();
    LcdInit();
    Relay_GPIO_Init();
    adc_init();
    led_init();
    HAL_TIM_Base_Start(&htim1);

    // === PASSKEY CHECK ===
    lprint(0x80, "Enter Passkey:");
    lprint(0xC0, "Press * to Start");
    while (1) {
        char key = Keypad_Scan();
        if (key != 0) {
            if (key == '*') {
                key_index = 0;
                memset(entered, 0, sizeof(entered));
                lprint(0x80, "Enter Code:     ");
                lprint(0xC0, "               ");
            } else if (key == '#' && key_index > 0) {
                entered[key_index] = '\0';
                if (strcmp(entered, PASSKEY) == 0) {
                    lprint(0x80, "Access Granted  ");
                    HAL_Delay(1000);
                    break; // Continue to main system
                } else {
                    lprint(0x80, "Wrong Passkey   ");
                    HAL_Delay(1000);
                    key_index = 0;
                    memset(entered, 0, sizeof(entered));
                }
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

    // === SENSOR + MOTOR SYSTEM ===
    char line1[17], line2[17];
    while (1) {
        if (DHT11_Start()) {
            RHI = DHT11_Read();
            RHD = DHT11_Read();
            TCI = DHT11_Read();
            TCD = DHT11_Read();
            SUM = DHT11_Read();
            if ((RHI + RHD + TCI + TCD) == SUM) {
                tCelsius = TCI + TCD / 10.0;
                RH = RHI + RHD / 10.0;
                humidity = (int)RH;
                temperature = (int)tCelsius;

                if (humidity > 60)
                    GPIOA->ODR &= ~(1 << 4);
                else
                    GPIOA->ODR |= (1 << 4);

                sprintf(line1, "Temp: %d C", temperature);
                lprint(0x80, line1);

                if (temperature >= 28) {
                    motor_on();
                    sprintf(line2, "Mild Hot     ");
                } else {
                    motor_off();
                    sprintf(line2, "Cool         ");
                }
                lprint(0xC0, line2);
            }
        }
        result = adc_read();
        led_control(result);
        HAL_Delay(2000);
    }
}
