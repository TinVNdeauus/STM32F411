/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdint.h>

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
#define WHO_AM_I 0x0F
#define GPIOA_BASE_ADDR 0x40020000
#define GPIOE_BASE_ADDR 0x40021000
#define SPI1_BASE_ADDR 0x40013000
#define CTRL_REG1 0x20
#define STATUS_REG 0x27
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D

int16_t raw_data[3] = {0};
#define ACTIVE 0
#define INACTIVE 1

static void spi_select(char act)
{
    uint32_t *GPIOE_ODR = (uint32_t *)(GPIOE_BASE_ADDR + 0X14);
    if (act == ACTIVE)
    {
        *GPIOE_ODR &= ~(1 << 3);
    }
    else if (act == INACTIVE)
    {
        *GPIOE_ODR |= (1 << 3);
    }
}

void spi_init()
{
    // GPIO PA5 (SPI1_SCK) PA6 (SPI1_MISO) PA7 (SPI1_MOSI)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    uint32_t *GPIOA_MODER = (uint32_t *)(GPIOA_BASE_ADDR + 0x00);
    uint32_t *GPIOA_AFRL = (uint32_t *)(GPIOA_BASE_ADDR + 0x20);
    *GPIOA_MODER &= ~(0b111111 << 10);
    *GPIOA_MODER |= (0b10 << 10) | (0b10 << 12) | (0b10 << 14);
    *GPIOA_AFRL &= ~(0xfff << 20);
    *GPIOA_AFRL |= (5 << 20) | (5 << 24) | (5 << 28);
    // PE3 - OUTPUT mode
    __HAL_RCC_GPIOE_CLK_ENABLE();
    uint32_t *GPIOE_MODER = (uint32_t *)(GPIOE_BASE_ADDR + 0x00);
    *GPIOE_MODER &= ~(0b11 << 6);
    *GPIOA_MODER |= (0b01 << 6);

    spi_select(INACTIVE);
    __HAL_RCC_SPI1_CLK_ENABLE();
    uint32_t *SPI1_CR1 = (uint32_t *)(SPI1_BASE_ADDR + 0x00);
    // STM32 spi in MASTER mode
    *SPI1_CR1 |= (1 << 2);
    // config clock -> F = 1Mhz
    *SPI1_CR1 &= ~(0b111 << 3);
    *SPI1_CR1 |= 0b011 << 3;
    // use software slave management
    *SPI1_CR1 |= (1 << 8) | (1 << 9);
    // enable SPI
    *SPI1_CR1 |= (1 << 6);
}

uint8_t spi_read(uint8_t cmd)
{

    uint32_t *SPI1_DR = (uint32_t *)(SPI1_BASE_ADDR + 0x0C);
    uint32_t *SPI1_SR = (uint32_t *)(SPI1_BASE_ADDR + 0x08);

    spi_select(ACTIVE);
    while (((*SPI1_SR >> 1) & 1) != 1);
    uint8_t data2send = cmd | (1 << 7);
    *SPI1_DR = data2send;
    while(((*SPI1_SR >> 1)& 1) == 1);
    while(((*SPI1_SR >> 0)& 1) != 1);
    while(((*SPI1_SR >> 7) & 1) == 1);

    uint32_t temp = *SPI1_DR;

    while (((*SPI1_SR >> 1) & 1) != 1);
    *SPI1_DR = 0x00;
    while(((*SPI1_SR >> 1)& 1) == 1);
    while(((*SPI1_SR >> 0)& 1) != 1);
    while(((*SPI1_SR >> 7) & 1) == 1);
    // read data from DR
    temp = *SPI1_DR;
    spi_select(INACTIVE);
    return temp;
}

void spi_multi_read(uint8_t cmd, uint8_t* buff, uint16_t size)
{
	uint32_t *SPI1_DR = (uint32_t *)(SPI1_BASE_ADDR + 0x0C);
	uint32_t *SPI1_SR = (uint32_t *)(SPI1_BASE_ADDR + 0x08);

	spi_select(ACTIVE);
	while(((*SPI1_SR >> 1)& 1) != 1);
	uint8_t data2send = cmd |(1<<7) | (1<<6);
	*SPI1_DR = data2send;
	while(((*SPI1_SR >> 1)& 1) == 1);
	while(((*SPI1_SR >> 0)& 1) != 1);
	while(((*SPI1_SR >> 7) & 1) == 1);
	uint32_t temp = *SPI1_DR;
	for (int i=0; i< size; i++)
	{
		while(((*SPI1_SR >> 1)& 1) != 1);
		*SPI1_DR = 0x00;
		while(((*SPI1_SR >> 1)& 1) ==1);
		while(((*SPI1_SR >> 0)& 1) != 1);
		while(((*SPI1_SR >> 7)& 1) == 1);
		buff[i] = *SPI1_DR;
	}
	spi_select(INACTIVE);
	return temp;
}

void spi_write(uint8_t cmd, uint8_t data)
{

    uint32_t *SPI1_DR = (uint32_t *)(SPI1_BASE_ADDR + 0x0C);
    uint32_t *SPI1_SR = (uint32_t *)(SPI1_BASE_ADDR + 0x08);
    spi_select(ACTIVE);
    while (((*SPI1_SR >> 1) & 1) != 1);
    uint8_t data2send = cmd;
    *SPI1_DR = data2send;
    while(((*SPI1_SR >> 1)& 1) == 1);
    while(((*SPI1_SR >> 0)& 1) != 1);
    while(((*SPI1_SR >> 7) & 1) == 1);

    uint32_t temp = *SPI1_DR;

    while (((*SPI1_SR >> 1) & 1) != 1);
    *SPI1_DR = data;
    while(((*SPI1_SR >> 1)& 1) == 1);
    while(((*SPI1_SR >> 0)& 1) != 1);
    while(((*SPI1_SR >> 7) & 1) == 1);
    temp = *SPI1_DR;
    spi_select(INACTIVE);

}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 2 */
  spi_init();
  uint8_t gyro_id = spi_read(WHO_AM_I);
  /* config gyros*/
  // power on with bit 3 to 1 in CTRL1(0x20)


  spi_write(CTRL_REG1, 0b00001111);
  HAL_Delay(1000);
  uint8_t check_data = spi_read(CTRL_REG1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  uint8_t status = spi_read(STATUS_REG);
	  if ((status >> 3) & 1 == 1)
	  {
		  spi_multi_read(OUT_X_L, (uint8_t*) raw_data, sizeof(raw_data));
		  __asm("NOP");
		  HAL_Delay(1000);
	  }
    /* USER CODE BEGIN 3 */
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
