/*
 * I2C_process.h
 *
 * Created: 10/24/2018 10:01:51 PM
 *  Author: User
 */ 


#ifndef I2C_PROCESS_H_
#define I2C_PROCESS_H_

/*--------------------------------------------------------AFE Register Maps----------------------------------------------------*/
#define SYS_STAT 0x00
#define CELLBAL1 0x01
#define CELLBAL2 0x02
#define CELLBAL3 0x03
#define SYS_CTRL1 0x04
#define SYS_CTRL2 0x05
#define PROTECT1 0x06
#define PROTECT2 0x07
#define PROTECT3 0x08
#define OV_TRIP 0x09
#define UV_TRIP 0x0A
#define CC_CFG 0x0B
#define VC1_HI 0x0C
#define VC1_LO 0x0D
#define VC2_HI 0x0E
#define VC2_LO 0x0F
#define VC3_HI 0x10
#define VC3_LO 0x11
#define VC4_HI 0x12
#define VC4_LO 0x13
#define VC5_HI 0x14
#define VC5_LO 0x15
#define VC6_HI 0x16
#define VC6_LO 0x17
#define VC7_HI 0x18
#define VC7_LO 0x19
#define VC8_HI 0x1A
#define VC8_LO 0x1B
#define VC9_HI 0x1C
#define VC9_LO 0x1D
#define VC10_HI 0x1E
#define VC10_LO 0x1F
#define VC11_HI 0x20
#define VC11_LO 0x21
#define VC12_HI 0x22
#define VC12_LO 0x23
#define VC13_HI 0x24
#define VC13_LO 0x25
#define VC14_HI 0x26
#define VC14_LO 0x27
#define VC15_HI 0x28
#define VC15_LO 0x29
#define VP_HI 0x2A
#define VP_LO 0x2B
#define TS1_HI 0x2C
#define TS1_LO 0x2D
#define TS2_HI 0x2E
#define TS2_LO 0x2F
#define TS3_HI 0x30
#define TS3_LO 0x31
#define CC_HI 0x32
#define CC_LO 0x33
#define ADCGAIN1 0x50
#define ADCOFFSET 0x51
#define ADCGAIN2 0x59
/*----------------------------------------------------END----------------------------------------------------------*/
/**********************************************AFE I2C definations*************************************************/
#define I2C_TIMEOUT 1000
#define I2C_PULLUP 1
#define SDA_PORT PORTD
#define SDA_PIN 6
#define SCL_PORT PORTD
#define SCL_PIN 7
#define MEMLEN 1
/**BQ7694003 AFE device address*/
#define I2C_7BITADDR 0x08
#define ADDRLEN 1
#define CRC_KEY 0x07





#endif /* I2C_PROCESS_H_ */