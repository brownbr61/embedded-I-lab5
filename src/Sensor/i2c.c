
void configure_master() {

  // enable GPIO B
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  GPIOB->MODER &= ~0xF3 << 22;
  GPIOB->MODER |= 0xA5 << 22;
  GPIOB->AFR[1] = (1 << 12) | (5 << 20); // I2C2_SDA to GPIOB
  GPIOB->AFR[1] = (1 << 12) | (5 << 26); // I2C2_SDA to GPIOB

  // Enable i2c sys clock
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

  // Enable i2c peripheral using

  /* (1) Timing register value is computed with the AN4235 xls file,
  fast Mode @400kHz with I2CCLK = 48MHz, rise time = 140ns,
  fall time = 40ns */
  /* (2) Periph enable */
  /* (3) Slave address = 0x5A, write transfer, 1 byte to transmit, autoend */

  // set parameters in TIMINGR to use 100kHz standard-mode I2C
  I2C2->TIMINGR |= 0x13; // SCLL
  I2C2->TIMINGR |= 0xF << 8;// SCLH 
  I2C2->TIMINGR |= 0x2 << 16;// SDADEL
  I2C2->TIMINGR |= 0x4 << 20;// SCLDEL
  I2C2->TIMINGR |= 1 << 28; // Pre-scale

  I2C2->CR1 = I2C_CR1_PE; /* (2) */
  I2C2->CR2 = I2C_CR2_AUTOEND | (1 << 16) | ((uint32_t)I2C1 << 1); /* (3) */
}

uint8_t whoami() {
  transmitValue('1');
  // Set the number of bytes to transmit = 1
  I2C2->CR2 &= ~(0xFF << 16);
  I2C2->CR2 |= 1 << 16;

  // Set the RD_WRN bit to indicate a write operation
  I2C2->CR2 |= 1 << 10;

  // Set the START bit
  I2C2->CR2 |= 1 << 13;

  // Wait until either of the TXIS or NACKF flags are set
  while (!(I2C2->ISR & I2C_ISR_TXIS));

  transmitValue('2');

  // Write address to i2c tx register
  I2C2->TXDR = 0x69;

  // Wait until TC flag is set
  while (!(I2C2->ISR & I2C_ISR_TC));

  transmitValue('3');

  // Reload the CR2 reg w same param as before, but this time READ
  // Set the number of bytes to transmit = 1
  I2C2->CR2 &= ~(0xFF << 16);
  I2C2->CR2 |= 1 << 16;

  // Set the RD_WRN bit to indicate a read operation
  I2C2->CR2 &= ~(1 << 10);

  // Set the START bit
  I2C2->CR2 |= 1 << 13;

  // Wait until either of the TXIS or NACKF flags are set
  while (!(I2C2->ISR & I2C_ISR_TXIS));

  transmitValue('4');

  uint8_t whoarthey = I2C2->RXDR;

  I2C2->CR2 |= 1 << 14;
  
  return whoarthey;
}

void i2c_tx(char byte) {
  if((I2C2->ISR & I2C_ISR_TXE) == I2C_ISR_TXE)
  {
    I2C2->TXDR = byte; /* Byte to send */
    I2C2->CR2 |= I2C_CR2_START; /* Go */
  }
}

void i2c_rx(char *byte) {
  if ((I2C2->ISR & *byte) == I2C_ISR_RXNE)
  {
    /* Read receive register, will clear RXNE flag */
    if (I2C2->RXDR == *byte)
    {
    (*byte) = 0;
    }
  }
}