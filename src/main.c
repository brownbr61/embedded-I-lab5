#include "main.h"

struct LEDs leds;
struct SensorData ledSensor;

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

uint8_t RX_Read(uint8_t value, uint8_t color) {
  /* Wait for RNXE */
	while(!(I2C2->ISR & I2C_ISR_RXNE)){}

  /* Read from RXDR */
  while((I2C2->RXDR & ~value)){ // Value in RXDR register
	  GPIOC->ODR |= 1 << color; // LED On
  }
	GPIOC->ODR &= ~(1 << color); // LED Off
  return value;
}

void RX_ReadSlave(uint16_t *value, uint8_t color) {
  /* Wait for RNXE */
	while(!(I2C2->ISR & I2C_ISR_RXNE)){}

  *value = I2C2->RXDR;
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
  HSI48_EN();
  // Enable system clock to be 1ms per tick
  SysTick_Config(1000);
  initUart(&uart);
  // char buffer[20];
  uint16_t value;

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

  transmitValue('\033');
  transmitValue('\143');
  transInt(0xab);
  transmitValue('\r');
  transmitValue('\n');
	

  // Step 5.4
  TX_Initialize(1); // Initialize I2C write for # of NBYTES
	TX_Write(0x0F); // Write Who_am_I address, Red if TXIS not set
  TX_Complete(7); // Blue LED on if TC flag not thrown
  RX_Initialize(1); // Initialize I2C read for # of NBYTES
  RX_ReadSlave(&value, 6); // Orange if RXNE flag is not thrown
  transInt(value);
  transmitValue('\r');
  transmitValue('\n');
  // RX_Complete(9); // WHO_AM_I value in RXDR, Red if no TC, Green value is wrong
  // I2C2->CR2 |= 1 << 14; // Stop bit to realease the I2C bus
  value = 0xb;

	//Step 5.5
  TX_Initialize(3); // Initialize I2C write for # of NBYTES
	TX_Write(0x20); // Write CTRL_REG1 address, Red if TXIS not set
  TX_Write(0x0F); // Write CTRL_REG1 address, Orange if TXIS not set
  TX_Write(0x0B); // Write CTRL_REG1 address, Orange if TXIS not set
  TX_Complete(); // Blue LED on if TC flag not thrown
  RX_Initialize(3); // Initialize I2C read for # of NBYTES
  for (int i = 0; i < 0; i ++) {
    RX_Read(value, 9); // X/Y/PD enabled, Red if no RXNE, Orange if value wrong
    transInt(value);
  }
  for (int i = 0; i < 2; i ++) {
    RX_ReadSlave(&value, 9); // X/Y/PD enabled, Red if no RXNE, Orange if value wrong
    transInt(value);
  }
  RX_Complete(); // WHO_AM_I value in RXDR, Red if no TC, Green value is wrong
  I2C2->CR2 |= 1 << 14; // Stop bit to realease the I2C bus

  transInt(0xab);
  transmitValue('\r');
  transmitValue('\n');

  //Step 5.6
  while (1){
    for (volatile int i = 0; i < 7200000; i++) {}
    
  //   TX_Initialize(2); // Initialize I2C write for # of NBYTES
  //   TX_Write(0xA8); // Write STATUS_REG address, Red if TXIS not set
  //   TX_Complete(); // Blue LED on if TC flag not thrown
  //   RX_Initialize(1); // Initialize I2C read for # of NBYTES
  //   // Debug
  //   while(!(I2C2->ISR & I2C_ISR_RXNE)){}

  // /* Read from RXDR */
  // // if((I2C2->RXDR == 0xBB)){ // Value in RXDR register
	// //   GPIOC->ODR |= 1 << 6; // LED On
  // // }
  // // if((I2C2->RXDR < 0xBC)){ // Value in RXDR register
	// //   GPIOC->ODR |= 1 << 7; // LED On
  // // }
  // //   if((I2C2->RXDR < 0xBD)){ // Value in RXDR register
	// //   GPIOC->ODR |= 1 << 8; // LED On
  // // }
  //   RX_Read(0xBB, 6); // STATUS_REG enabled, Red if no RXNE, Orange if value wrong
  //   RX_Complete(); // WHO_AM_I value in RXDR, Red if no TC, Green value is wrong
  }
}
