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

/************************************************Communication Codes**********************************************/
/** Maximum Buffer Size for Serial data*/
#define COMM_MAX_BUFFER_SZ 235
/** No data available on Serial*/
#define NO_DATA_ON_SERIAL 0

/*************************************************Communication command no for processing************************/
/** Communication Command processing Error */
#define COMM_CMD_ERROR        0
/** Communications is Still receiving data or waiting */
#define COMM_CMD_BUSY         1
/** Communications Command to Read ADC Instantaneously */
#define COMM_CMD_ADC          2
//#define COMM_CMD_LOG          2
//#define COMM_CMD_UPDATE       3



/**********************************************Size definations for communication commands***************************/
/**Size for cells voltage command*/
#define SZ_CMD_LOG             3
#define SZ_CMD_UPDATE          6
#define SZ_CMD_ADC 3


      
/*********************************************EEPROM FILE*****************************************************/
#define FADD_SYS_STATWORD_HI  0x00
#define FADD_SYS_STATWORD_LO  0x01
#define FADD_SYS_STAT         0x02
#define FADD_CELLBAL1         0x03
#define FADD_CELLBAL2         0x04
#define FADD_CELLBAL3         0x05
#define FADD_SYS_CTRL1        0x06
#define FADD_SYS_CTRL2        0x07
#define FADD_PROTECT1         0x08
#define FADD_PROTECT2         0x09
#define FADD_PROTECT3         0x0A
#define FADD_CC_CFG           0x0B
#define FADD_C_OV_TRIP        0x0C
#define FADD_C_OV_CON_H       0x0D
#define FADD_C_OV_CON_L       0x0E
#define FADD_C_OV_COUNT       0x0F
#define FADD_C_UV_TRIP        0x10
#define FADD_C_UV_CON_H       0x11
#define FADD_C_UV_CON_L       0x12
#define FADD_C_UV_COUNT       0x13
#define FADD_P_OV_TRIP_H      0x14
#define FADD_P_OV_TRIP_L      0x15
#define FADD_P_OV_CON_H       0x16
#define FADD_P_OV_CON_L       0x17
#define FADD_P_OV_COUNT       0x18
#define FADD_P_UV_TRIP_H      0x19
#define FADD_P_UV_TRIP_L      0x1A
#define FADD_P_UV_CON_H       0x1B
#define FADD_P_UV_CON_L       0x1C
#define FADD_P_UV_COUNT       0x1D
#define FADD_C_OT_TRIP_H      0x1E
#define FADD_C_OT_TRIP_L      0x1F
#define FADD_C_OT_CON_H       0x20
#define FADD_C_OT_CON_L       0x21
#define FADD_C_OT_COUNT       0x22
#define FADD_C_UT_TRIP_H      0x23
#define FADD_C_UT_TRIP_L      0x24
#define FADD_C_UT_CON_H       0x25
#define FADD_C_UT_CON_L       0x26
#define FADD_C_UT_COUNT       0x27
#define FADD_D_OT_TRIP_H      0x28
#define FADD_D_OT_TRIP_L      0x29
#define FADD_D_OT_CON_H       0x2A
#define FADD_D_OT_CON_L       0x2B
#define FADD_D_OT_COUNT       0x2C
#define FADD_D_UT_TRIP_H      0x2D
#define FADD_D_UT_TRIP_L      0x2E
#define FADD_D_UT_CON_H       0x2F
#define FADD_D_UT_CON_L       0x30
#define FADD_D_UT_COUNT       0x31
#define FADD_OCC_TRIP_H       0x32
#define FADD_OCC_TRIP_L       0x33
#define FADD_OCC_CON_H        0x34
#define FADD_OCC_CON_L        0x35
#define FADD_OCC_COUNT        0x36
#define FADD_OCD_TRIP         0x37
#define FADD_OCD_CON_H        0x38
#define FADD_OCD_CON_L        0x39
#define FADD_OCD_COUNT        0x3A
#define FADD_P_OT_TRIP_H      0x3B
#define FADD_P_OT_TRIP_L      0x3C
#define FADD_P_OT_CON_H       0x3D
#define FADD_P_OT_CON_L       0x3E
#define FADD_P_OT_COUNT       0x3F
#define FADD_P_UT_TRIP_H      0x40
#define FADD_P_UT_TRIP_L      0x41
#define FADD_P_UT_CON_H       0x42
#define FADD_P_UT_CON_L       0x43
#define FADD_P_UT_COUNT       0x44
#define FADD_SCC_TRIP_H       0x45
#define FADD_SCC_TRIP_L       0x46
#define FADD_SCC_CON_H        0x47
#define FADD_SCC_CON_L        0x48
#define FADD_SCC_COUNT        0x49
#define FADD_SCD_TRIP         0x4A
#define FADD_SCD_CON_H        0x4B
#define FADD_SCD_CON_L        0x4C
#define FADD_SCD_COUNT        0x4D
#define FADD_OP_TRIP_H        0x4E
#define FADD_OP_TRIP_L        0x4F
#define FADD_OP_CON_H         0x50
#define FADD_OP_CON_L         0x51
#define FADD_OP_COUNT         0x52

#define FOSC 16000000 // Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

/**Incude header files here*/
#include <SoftI2CMaster.h>


//const char cmd_log[SZ_CMD_LOG]               = "log";
//const char cmd_update[SZ_CMD_UPDATE]         = "update";
const char cmd_adc[SZ_CMD_ADC] = "adc";
/********************************************************Function's declaration************************************************/
/**Function to merge two 8 bit data into one single 16_bit data*/
uint16_t merge_16(uint8_t h_16, uint8_t l_16);
/**Function to merge two 8 bit data into one single 14_bit data*/
uint16_t merge_14(uint8_t h_14, uint8_t l_14);
/**Function to measure coulumb,pack voltage, cell voltages and temperature*/
void measure(void);
/**Function to check, recover and update faults on EEPROM*/ 
void recover_faults(void);
/**Function to setup GPIO*/
void init_GPIO(void);
/**Function to setup pin change interrupt*/
void init_PCI(void);
/**Function to setup AFE*/
void init_AFE(void);
/**Function to load values from flash to AFE*/
void lddflts(void);
/**Function to store serial data into the Serial buffer*/
uint8_t READ_SR_BUFF(void);
/**Function to process serial buffer*/
uint8_t PROCESS_SR_BUFF(unsigned int buff_sz);
/**Function to process data */
uint8_t process(void);
/**Business Logic Processing Loop */
uint8_t BL_Process(uint8_t serial_cmd);
/**Function to read faults from AFE*/
void chk_fault(void);
/**Function to clear buffer*/
void clear_Buffer(void);
/** Funtion to load Default_system settings*/
void Write_Defaults_System(void);
/**Function to produce error message*/
void error(void);
/**Function to write data with CRC8 onto AFE*/
void i2c_WriteWithCRC(uint8_t I2CSlaveAddress, uint8_t Register, uint8_t Data);
/**Function to calculate CRC8*/
unsigned char CRC8(unsigned char *ptr, unsigned char len,unsigned char key);
/**Function to calculate convert string to hex*/
uint8_t str2hex(char s);
/**Function to calculate convert string to hex*/
void i2c_log(void);
/* Communications Processing Loop */
uint8_t Comm_loop(void);
void USART_Init( unsigned int ubrr);
void USART_Transmit( unsigned char data );
unsigned char USART_Receive( void );
void TU_putHex(uint8_t data);
/* Command Processor with Line Ending '\r\n' termination */
uint8_t TU_getln(uint8_t *arr, uint8_t max_sz);
uint8_t UART2_ReceiveData8(void);
uint8_t TU_getc(uint8_t *data);

/******************************************************Variable declaration*******************************************/
volatile uint8_t comm_buffer[COMM_MAX_BUFFER_SZ];
volatile uint8_t serial_command;
volatile uint8_t SYS_STATWORD_HI = 0x00, SYS_STATWORD_LO = 0x00;
volatile uint8_t faults=0x00;
volatile uint8_t afe_isr_reg[235], temp_read=0;
volatile int isr_count=0;
volatile bool erom_cng=1;
volatile uint8_t cd=0x00;
/** Exposed size of the Communication Buffer */
volatile uint8_t comm_sz;
uint8_t* comm_ptr_next;
/** Exposed size of the Command Processed */
uint8_t comm_cmd_sz;
/************************************************************END******************************************************/

/*Interrupt service routine*/
ISR(PCINT1_vect)
{
  /**Clear CC_READY bit in system status register*/
  i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0x80);
  isr_count++;
}
/*End of pin change interrupt routine*/


/*Setup function to be executed at once*/
void setup() 
{
  USART_Init(MYUBRR);
  pinMode(8,OUTPUT);
  pinMode(14,OUTPUT);
  digitalWrite(8,LOW);
  digitalWrite(14,HIGH);

  cli();
  /*Set pin change interrupt control register*/
  PCICR  |= (1<<1);
  /*Set pin change mask register 1*/
  PCMSK1 |= (1<<3);
  /*Enable global interrupts*/
  sei();

  i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0x80);
  /*Defaults parameters*/
  i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x40); 
  i2c_WriteWithCRC(I2C_7BITADDR, CELLBAL1,  0x00);
  i2c_WriteWithCRC(I2C_7BITADDR, CELLBAL2,  0x00);
  i2c_WriteWithCRC(I2C_7BITADDR, CELLBAL3,  0x00);
  i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL1, 0x18);
  i2c_WriteWithCRC(I2C_7BITADDR, PROTECT1,  0x9F);
  i2c_WriteWithCRC(I2C_7BITADDR, PROTECT2,  0x57);
  i2c_WriteWithCRC(I2C_7BITADDR, PROTECT3,  0xF0);
  i2c_WriteWithCRC(I2C_7BITADDR, OV_TRIP,   0xB8);
  i2c_WriteWithCRC(I2C_7BITADDR, UV_TRIP,   0x13);
  i2c_WriteWithCRC(I2C_7BITADDR, CC_CFG,    0x19);
  /*End of default parameters*/
  //Serial.println("Values has been Loaded");
}






void loop()
{
/*
  if(isr_count > 4)
    {
      measure();
      //Serial.println("Read Sucess");
      isr_count=0;
    }
*/
}


void i2c_WriteWithCRC(uint8_t I2CSlaveAddress, uint8_t Register, uint8_t Data)
{
  volatile uint8_t DataBuffer[4];
  DataBuffer[0] = I2CSlaveAddress << 1;
  DataBuffer[1] = Register;
  DataBuffer[2] = Data;
  DataBuffer[3] = CRC8(DataBuffer, 3, CRC_KEY);
  
  i2c_start(DataBuffer[0] | I2C_WRITE);
  i2c_write(DataBuffer[1]);
  i2c_write(DataBuffer[2]);
  i2c_write(DataBuffer[3]);
  i2c_stop();
}

unsigned char CRC8(unsigned char *ptr, unsigned char len,unsigned char key)
{
  volatile unsigned char i;
  volatile unsigned char crc=0;
  while(len--!=0)
  {
    for(i=0x80; i!=0; i/=2)
    {
      if((crc & 0x80) != 0)
      {
        crc *= 2;
        crc ^= key;
      }
      else
        crc *= 2;

      if((*ptr & i)!=0)
        crc ^= key;
    }
    ptr++;
  }
  return(crc);
}

void i2c_log(void)
{
  cli();  
  for(volatile int g=0; g<235; g++)
  {
    Serial.print(afe_isr_reg[g], HEX);
  }
  Serial.print('\r');
  sei();
  i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0x80);
}




