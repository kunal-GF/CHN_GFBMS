#ifndef _I2C_BB_H_
#define _I2C_BB_H_

#include "stm8s.h"

/************************************************************************/
/* Conditionals                                                         */
/************************************************************************/

#define I2C_BB
/**
 * @warning Make sure the Include the Definitions of 'I2C_BB' in the 
 *          Compilation to include the functionality of GPIO Bit Bang I2C
 */
#ifdef I2C_BB

#ifdef __cplusplus
extern "C"
{
#endif

/************************************************************************/
/* Constants                                                            */
/************************************************************************/

/** I2C SDA Pin definition */
#define SDA_PIN GPIO_PIN_5
/** I2C SCL Pin definition */
#define SCL_PIN GPIO_PIN_4

/************************************************************************/
/* API                                                                  */
/************************************************************************/
void i2cbb_start(void);
uint8_t i2cbb_send(uint8_t data);
uint8_t i2cbb_read(uint8_t ack);
void i2cbb_stop(void);

/************************************************************************/
/* External API                                                         */
/************************************************************************/


#ifdef __cplusplus
}
#endif  /** END of C extern Definition */

#endif /* I2C_BB Selector */

#endif /* _I2C_BB_H_ */