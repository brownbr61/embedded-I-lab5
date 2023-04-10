/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

void TX_Initialize(uint8_t nbytes) {
  /* CR1 */
  I2C2->CR1 &= ~1; // Reseting PE resets the interrupt flags
  I2C2->CR1 |= 1; // Enables I2C Peripheral PE 
  /* CR2 */
  I2C2->CR2 = 0; // Clear CR2
  I2C2->CR2 |= 0x69 << 1; // L3GD20 slave address in SADD
  I2C2->CR2 |= nbytes << 16; // NBYTES = 1 
  I2C2->CR2 &= ~(1 << 10); // RD_WRN write bit
  I2C2->CR2 |= 1 << 13; // Start bit
}

void TX_Write(uint8_t address) {
  /* Wait for TXIS then write to TXDR */
  while(!(I2C2->ISR & I2C_ISR_TXIS)){}
  I2C2->TXDR = address; // register address or data to set register
}

void TX_Complete() {
  /* Wait for TC */
  while(!(I2C2->ISR & I2C_ISR_TC)){}
}

void RX_Initialize(uint8_t nbytes) {
  /* CR2 */
  I2C2->CR2 |= 0x69 << 1; // L3GD20 slave address in SADD
  I2C2->CR2 |= nbytes << 16; // NBYTES = 1 
  I2C2->CR2 |= 1 << 10; // RD_WRN read bit
  I2C2->CR2 |= 1 << 13; // Start bit
}
void RX_Read(uint8_t value, uint8_t color) {
  /* Wait for RNXE */
	while(!(I2C2->ISR & I2C_ISR_RXNE)){}

  /* Read from RXDR */
  while((I2C2->RXDR & ~value)){ // Value in RXDR register
	  GPIOC->ODR |= 1 << color; // LED On
  }
	GPIOC->ODR &= ~(1 << color); // LED Off
}

void RX_Complete() {
  while(!(I2C2->ISR & I2C_ISR_TC)){ }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */


int main(void)
{
  HAL_Init();           // Reset of all peripherals, init the Flash and Systick
  SystemClock_Config(); // Configure the system clock

  __HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;               
  // Step 5.1 Initialize GPIOs
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

  // Step 5.2
  GPIOB->MODER &= ~((3 << (11 * 2u)) | (3 << (13 * 2u)) | (3 << (14 * 2u))); // Clear bits
  GPIOB->MODER |= (2 << (11 * 2u)) | (2 << (13 * 2u));  // Alternate function mode
  GPIOB->MODER |= 1 << (14 *2u); // Output mode

  GPIOB->OTYPER |= (1 << 11) | (1 << 13); // Open drain output
  GPIOB->OTYPER |= 0 << 14; // Push/Pull mode
	
	GPIOB->PUPDR |= (1 << (11 * 2u)) | (1 << (13 * 2u)); // Pull Up

	GPIOB->ODR |= 1 << 14; // Sets PB14 to logic high 
	
  GPIOB->AFR[1] |= (1<< 12) | (5 << 20); // I2C2_SDA to GPIOB

  GPIOC->MODER |= 1; // PC0 Output mode
  GPIOC->PUPDR |= 1 << 0; // Pull Up
  GPIOC->MODER |= (1 << (6 * 2u)) | (1 << (7 * 2u))
								| (1 << (8 * 2u)) | (1 << (9 * 2u)); // Turn on the lights
  
  // Set type to push-pull output
  GPIOC->OTYPER &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));

  // Set output speed to low speed
  GPIOC->OSPEEDR &= ~(0xff);

  // Set pull-up/down resistors to no
  GPIOC->PUPDR &= ~(0xff);
  //GPIOC->ODR |= ((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
	GPIOC->ODR |= 1 << 0; // PC0 to high
  
  // Step 5.3 

  // All TIMINGR parameters are in lab 5 handout (100kHz)
  I2C2->TIMINGR |= 0x13; // SCLL
  I2C2->TIMINGR |= 0xF << 8;// SCLH 
  I2C2->TIMINGR |= 0x2 << 16;// SDADEL
  I2C2->TIMINGR |= 0x4 << 20;// SCLDEL
  I2C2->TIMINGR |= 1 << 28; // Pre-scale 
	

  // Step 5.4
  // TX_Initialize(1); // Initialize I2C write for # of NBYTES
	// TX_Write(0x0F, 6); // Write Who_am_I address, Red if TXIS not set
  // TX_Complete(7); // Blue LED on if TC flag not thrown
  // RX_Initialize(1); // Initialize I2C read for # of NBYTES
  // RX_Read(0xD3, 6, 8); // Orange if RXNE flag is not thrown
  // RX_Complete(9); // WHO_AM_I value in RXDR, Red if no TC, Green value is wrong
  // I2C2->CR2 |= 1 << 14; // Stop bit to realease the I2C bus

	//Step 5.5
  TX_Initialize(3); // Initialize I2C write for # of NBYTES
	TX_Write(0x20); // Write CTRL_REG1 address, Red if TXIS not set
  TX_Write(0x0F); // Write CTRL_REG1 address, Orange if TXIS not set
  TX_Write(0x0B); // Write CTRL_REG1 address, Orange if TXIS not set
  TX_Complete(); // Blue LED on if TC flag not thrown
  RX_Initialize(3); // Initialize I2C read for # of NBYTES
  RX_Read(0x0F, 9); // X/Y/PD enabled, Red if no RXNE, Orange if value wrong
  RX_Read(0x0B, 9); // X/Y/PD enabled, Red if no RXNE, Orange if value wrong
  RX_Complete(); // WHO_AM_I value in RXDR, Red if no TC, Green value is wrong
  I2C2->CR2 |= 1 << 14; // Stop bit to realease the I2C bus

  //Step 5.6
  while (1){
    HAL_Delay(100); // delays action for 100 ms
    TX_Initialize(2); // Initialize I2C write for # of NBYTES
    TX_Write(0xA8); // Write STATUS_REG address, Red if TXIS not set
    TX_Complete(); // Blue LED on if TC flag not thrown
    RX_Initialize(1); // Initialize I2C read for # of NBYTES
    // Debug
    while(!(I2C2->ISR & I2C_ISR_RXNE)){}

  /* Read from RXDR */
  // if((I2C2->RXDR == 0xBB)){ // Value in RXDR register
	//   GPIOC->ODR |= 1 << 6; // LED On
  // }
  // if((I2C2->RXDR < 0xBC)){ // Value in RXDR register
	//   GPIOC->ODR |= 1 << 7; // LED On
  // }
  //   if((I2C2->RXDR < 0xBD)){ // Value in RXDR register
	//   GPIOC->ODR |= 1 << 8; // LED On
  // }
    RX_Read(0xBB, 6); // STATUS_REG enabled, Red if no RXNE, Orange if value wrong
    RX_Complete(); // WHO_AM_I value in RXDR, Red if no TC, Green value is wrong
  }

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  /* USER CODE BEGIN 7Error_Handler_Debug */
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
