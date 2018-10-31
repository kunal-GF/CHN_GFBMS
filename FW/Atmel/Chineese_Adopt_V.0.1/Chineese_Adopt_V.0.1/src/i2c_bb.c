/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "i2c_bb.h"
#ifdef I2C_BB
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/** I2C Bit time Delay for 100Khz bus Speed (12) */
#define I2C_BIT_DELAY() ndelay(12)
/** I2C Half of Bit time Delay for 100Khz bus Speed (6) */
#define I2C_HALFBIT_DELAY() ndelay(6)

#define I2C_SET_SDA() GPIOB->ODR |= SDA_PIN
#define I2C_CLR_SDA() GPIOB->ODR &= ~SDA_PIN
#define I2C_GET_SDA() GPIOB->IDR & SDA_PIN
#define I2C_SET_SCL() GPIOB->ODR |= SCL_PIN
#define I2C_CLR_SCL() GPIOB->ODR &= ~SCL_PIN
#define I2C_BUS_RELEASE() GPIOB->ODR |= (SDA_PIN|SCL_PIN)
#define I2C_SDA_RELEASE() I2C_SET_SDA()
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * Loop based Delay Function to generate the I2C Delays
 *
 * @param del The number of CPU Cycles to be expended in the Delay
 */
void ndelay(uint8_t del)
{
  while(del > 0)
    --del;
}

/* Public functions ----------------------------------------------------------*/

/* Function to Send the Start Condition on the I2C line */
void i2cbb_start(void)
{
  I2C_BUS_RELEASE();
  I2C_BIT_DELAY();    
  I2C_CLR_SDA();
  I2C_BIT_DELAY();    
  I2C_CLR_SCL();
}

/* Function to Send the Data Bytes on the I2C lines*/
uint8_t i2cbb_send(uint8_t data)
{
  uint8_t i, ret;
  for(i=0;i<8;i++)
  {
    if(data & 0x80)
      I2C_SET_SDA();
    else
      I2C_CLR_SDA();

    I2C_BIT_DELAY();
    I2C_SET_SCL();
    I2C_BIT_DELAY();
    I2C_CLR_SCL();
    data <<= 1;
  }
  //=== ACK
  I2C_SDA_RELEASE(); // Release
  I2C_BIT_DELAY();
  ret = I2C_GET_SDA();
  I2C_SET_SCL();
  I2C_BIT_DELAY();
  I2C_CLR_SCL();
  return ret&&1;
}

/* Function to Read data from I2C line and Acknowledge the same */
uint8_t i2cbb_read(uint8_t ack)
{
  uint8_t i, ret = 0;
  // make sure that the SDA pin is released
  I2C_SDA_RELEASE(); // Release
  for(i=0;i<8;i++)
  {
    ret <<= 1;
    I2C_BIT_DELAY();
    I2C_SET_SCL();
    I2C_BIT_DELAY();
    if(I2C_GET_SDA())
      ret |= 1;
    I2C_CLR_SCL();    
  }
  //=== ACK
  if(ack != 0)
    I2C_CLR_SDA();
  I2C_BIT_DELAY();
  I2C_SET_SCL();
  I2C_BIT_DELAY();
  I2C_CLR_SCL();
  I2C_SDA_RELEASE(); // Release
  return ret;
}

/* Function to Generate the Stop Condition on the I2C Line */
void i2cbb_stop(void)
{
  I2C_BIT_DELAY();
  I2C_CLR_SDA();
  I2C_BIT_DELAY();
  I2C_SET_SCL();
  I2C_BIT_DELAY();
  I2C_BUS_RELEASE();
}

#endif /* MEASURE Selector */
