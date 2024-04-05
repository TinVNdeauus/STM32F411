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

#define GPIOB_BASE_ADDR		0x40020400
#define I2C1_BASE_ADDR		0x40005400
void i2c_init()
{
	//setup GPIO
	__HAL_RCC_GPIOB_CLK_ENABLE();
	uint32_t* MODER = (uint32_t*)(GPIOB_BASE_ADDR + 0x00);
	*MODER &= ~((0b11 << 12) | (0b11 << 18));
	*MODER |= (0b10 << 12) | (0b10 << 18);

	uint32_t* AFRL = (uint32_t*)(GPIOB_BASE_ADDR + 0x20);
	*AFRL &= ~(0b1111 << 24);
	*AFRL |= (4<<24);
	uint32_t* AFRH = (uint32_t*)(GPIOB_BASE_ADDR + 0x24);
	*AFRH &= ~(0b1111 << 4);
	*AFRH |= (4 << 4);

	// setup I2C1
	__HAL_RCC_I2C1_CLK_ENABLE();
	uint32_t* CR2 = (uint32_t*)(I2C1_BASE_ADDR + 0x04);
	*CR2 &= ~(0b111111 << 0);
	*CR2 |= 16 << 0;
	uint32_t* CCR = (uint32_t*)(I2C1_BASE_ADDR + 0x1C);
	*CCR = 0;
	*CCR |= 160;
	uint32_t* CR1 = (uint32_t*)(I2C1_BASE_ADDR + 0x00);
	*CR1 |= 1<<0;
}
uint8_t i2c_read(uint8_t reg)
{
	/*
		- generate a start bit
		- send 7 address + 1 write bit (LOW - 0)
		- check ACK from slave
		- send REG ADDR slave
		- check ACK from slave
		- generate a start bit again
		- send 7 bit address + 1 read bit ( HIGH - 1)
		- check ACK from slave
		- read data from slave
		- generate a stop bit
	 */
	const uint8_t slave_addr = 0b0011001;
	const uint8_t write_bit = 0;
	const uint8_t read_bit = 1;
	uint32_t* SR1 = (uint32_t*)(I2C1_BASE_ADDR + 0x14);
	uint32_t* SR2 = (uint32_t*)(I2C1_BASE_ADDR + 0x18);
	uint32_t* CR1 = (uint32_t*)(I2C1_BASE_ADDR + 0x00);
	uint32_t* DR = (uint32_t*)(I2C1_BASE_ADDR + 0x10);
	while(((*SR2 >> 1) & 1) == 1); 	// wait i2c bus free
	*CR1 |= (1<<8); 				// generate a start bit
	while(((*SR1 >> 0 ) & 1) != 1);	// wait start bit generated
	*DR = slave_addr << 1 | write_bit;
	while(((*SR1 >> 1 ) & 1) != 1);	// send address completed
	uint32_t temp = *SR2;			// read SR2 to clear ADDR bit in SR1
	while(((*SR1 >> 10 ) & 1) == 1);// check ACK
	*DR = reg;
	while(((*SR1 >> 7 ) & 1) != 1);	// wait Tx empty ( data transmission is completed)
	while(((*SR1 >> 10 ) & 1) == 1);// check ACK

	*CR1 |= (1<<8); 				// generate a start bit
	while(((*SR1 >> 0 ) & 1) != 1);	// wait start bit generated
	*DR = slave_addr << 1 | read_bit;
	while(((*SR1 >> 1 ) & 1) != 1);	// send address completed
	while(((*SR1 >> 10 ) & 1) == 1);// check ACK
	temp = *SR2;
	while(((*SR1 >> 6 ) & 1) != 1);	// wait RxNE is set
	temp = *DR;
	*CR1 |= (1<<9); 				// generate a stop bit
	return temp;
}
void i2c_write(uint8_t reg, uint8_t val)
{
	/*
	 	- generate a start bit
		- send 7 bit address + 1 write bit
		- check ACK bit from slave
		- send REG ADDR slave
		- check ACK from slave
		- send WRITE DATA
		- check ACK
		- generate a stop bit
	 */
	const uint8_t slave_addr = 0b0011001;
	const uint8_t write_bit = 0;
	uint32_t* SR1 = (uint32_t*)(I2C1_BASE_ADDR + 0x14);
	uint32_t* SR2 = (uint32_t*)(I2C1_BASE_ADDR + 0x18);
	uint32_t* CR1 = (uint32_t*)(I2C1_BASE_ADDR + 0x00);
	uint32_t* DR = (uint32_t*)(I2C1_BASE_ADDR + 0x10);
	while(((*SR2 >> 1) & 1) == 1); 	// wait i2c bus free
	*CR1 |= (1<<8); 				// generate a start bit
	while(((*SR1 >> 0 ) & 1) != 1);	// wait start bit generated
	*DR = slave_addr << 1 | write_bit;
	while(((*SR1 >> 1 ) & 1) != 1);	// send address completed
	uint32_t temp = *SR2;			// read SR2 to clear ADDR bit in SR1
	while(((*SR1 >> 10 ) & 1) == 1);// check ACK
	*DR = reg;
	while(((*SR1 >> 7 ) & 1) != 1);	// wait Tx empty ( data transmission is completed)
	while(((*SR1 >> 10 ) & 1) == 1);// check ACK
	*DR = val;
	while(((*SR1 >> 7 ) & 1) != 1);	// wait Tx empty ( data transmission is completed)
	while(((*SR1 >> 10 ) & 1) == 1);// check ACK
	*CR1 |= (1<<9); 				// generate a stop bit
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
  i2c_init();
  uint8_t slave_id = i2c_read(0x0F);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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
