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
#define COMM_CMD_ERROR        1
#define COMM_CMD_LOG          2
#define COMM_CMD_UPDATE       3



/**********************************************Size definations for communication commands***************************/
/**Size for cells voltage command*/
#define SZ_CMD_LOG             3
#define SZ_CMD_UPDATE          6



      
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



/**Incude header files here*/
#include <SoftI2CMaster.h>


const char cmd_log[SZ_CMD_LOG]               = "log";
const char cmd_update[SZ_CMD_UPDATE]         = "update";
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

uint8_t Comm_loop(void);

/******************************************************Variable declaration*******************************************/
volatile uint8_t comm_buffer[COMM_MAX_BUFFER_SZ];
volatile uint8_t serial_command;
volatile uint8_t SYS_STATWORD_HI = 0x00, SYS_STATWORD_LO = 0x00;
volatile uint8_t faults=0x00;
volatile uint8_t afe_isr_reg[235], temp_read=0;
volatile int isr_count=0;
volatile bool erom_cng=1;
volatile uint8_t cd=0x00;
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
  Serial.begin(9600);
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
  Serial.println("Values has been Loaded");
}






void loop()
{
   /*Store and process serial buffer*/
 // BL_Process(PROCESS_SR_BUFF(READ_SR_BUFF()));
  if(isr_count > 4)
    {
      measure();
      Serial.println("Read Sucess");
      isr_count=0;
    }

    serial_command = READ_SR_BUFF();
    //Serial.println(serial_command);
    //delay(1000);
    if(serial_command > 0)
    BL_Process(serial_command);
}










/** Function to store serial data into the Serial buffer*/
uint8_t READ_SR_BUFF(void)
{
    volatile uint8_t i=0;
    clear_Buffer();
    while(Serial.available()>0)
    {
     
      comm_buffer[i] = Serial.read();
      
      if(comm_buffer[i] == '@')
        return process();
        
      i++;
    }
    return 0;
}





/* Business logic process loop */
uint8_t BL_Process(uint8_t serial_cmd)
{
  do{

    Serial.println(serial_cmd);
    if(serial_cmd == COMM_CMD_ERROR)
    {
      Serial.println("Unknown Command Received ");
      break;
    }
    
    if(serial_cmd == COMM_CMD_LOG)
    {
      i2c_log();
      break;
    }

    if(serial_cmd == COMM_CMD_UPDATE)
    {
      Serial.println("sucess");
      break;
    }
  }while(0);

  return 0;
}





uint8_t process(void)
{
  volatile uint8_t ret = COMM_CMD_ERROR;
  do
  {
      if(strncmp(cmd_log, comm_buffer, SZ_CMD_LOG) == 0)
      {
          ret = COMM_CMD_LOG;
          clear_Buffer();
          break;
      }  
  
   }while(0);
  return ret;
}

















/*Function to write hex values to the afe with CRC8*/
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
/*End of function*/

/*Function to calculate CRC8*/
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
/*End of function*/

uint8_t str2hex(char s)
{
  uint8_t ret = 0;
  
  if(s >= '0' && s <= '9')
  {
    ret = s - '0';
  }
  else if(s >= 'A' && s <= 'F')
  {
    ret = s - 'A' + 10;
  }
  else if(s >= 'a' && s <= 'f')
  {
    ret = s - 'a' + 10;
  }
  else
  {
    //error = 1; // Indicate Processing Error
  }
  return ret & 0x0F;
}


void i2c_log(void)
{  
  for(int g=0; g<234; g++)
  {
    Serial.print(afe_isr_reg[g],HEX);
  }
}


void clear_Buffer(void)
{
  for(volatile int j=0; j<=200; j++)
  {
    comm_buffer[j] = 0;
  }
}



void error(void)
{
  Serial.println("ERROR");
}



void measure(void)
{
  cli(); 
  temp_read++;
  
  if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }

    i2c_write(VC1_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[0] = i2c_read(true);
    

    afe_isr_reg[1] = ':';
  
    i2c_write(VC1_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[2] = i2c_read(true);
    i2c_stop();

    
    afe_isr_reg[3] = ':';
    
    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }
    
    i2c_write(VC2_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[4] = i2c_read(true);
   

    afe_isr_reg[5] = ':';
    
    i2c_write(VC2_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[6] = i2c_read(true);
    i2c_stop();

    afe_isr_reg[7] = ':';

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }
    
    i2c_write(VC3_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[8] = i2c_read(true);
    

    afe_isr_reg[9] = ':';
  
    i2c_write(VC3_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[10] = i2c_read(true);
    i2c_stop();

    afe_isr_reg[11] = ':';

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }
    
    i2c_write(VC4_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[12] = i2c_read(true);
   

    afe_isr_reg[13] = ':';
  
    i2c_write(VC4_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[14] = i2c_read(true);
    i2c_stop();

    afe_isr_reg[15] = ':';

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }
    
    i2c_write(VC5_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[16] = i2c_read(true);
    

    afe_isr_reg[17] = ':';
  
    i2c_write(VC5_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[18] = i2c_read(true);
    i2c_stop();

    afe_isr_reg[19] = ':';
   
    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }
    
    i2c_write(VC6_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[20] = i2c_read(true);
   

    afe_isr_reg[21] = ':';
  
    i2c_write(VC6_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[22] = i2c_read(true);
    i2c_stop();

    afe_isr_reg[23] = ':';

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }
    
    i2c_write(VC7_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[24] = i2c_read(true);
    

    afe_isr_reg[25] = ':';
  
    i2c_write(VC7_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[26] = i2c_read(true);
    i2c_stop();

    afe_isr_reg[27] = ':';

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }
    
    i2c_write(VC8_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[28] = i2c_read(true);
   

    afe_isr_reg[29] = ':';
  
    i2c_write(VC8_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[30] = i2c_read(true);
    i2c_stop();

    afe_isr_reg[31] = ':';

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }
    
    i2c_write(VC9_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[32] = i2c_read(true);
    

    afe_isr_reg[33] = ':';
  
    i2c_write(VC9_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[34] = i2c_read(true);
    i2c_stop();

    afe_isr_reg[35] = ':';

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }
    
    i2c_write(VC10_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[36] = i2c_read(true);
   

    afe_isr_reg[37] = ':';
  
    i2c_write(VC10_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[38] = i2c_read(true);
    i2c_stop();

    afe_isr_reg[39] = ':';

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }
    
    i2c_write(VC11_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[40] = i2c_read(true);
    

    afe_isr_reg[41] = ':';
  
    i2c_write(VC11_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[42] = i2c_read(true);
    i2c_stop();

    afe_isr_reg[43] = ':';

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }
    
    i2c_write(VC12_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[44] = i2c_read(true);
    

    afe_isr_reg[45] = ':';
  
    i2c_write(VC12_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[46] = i2c_read(true);
    i2c_stop();

    afe_isr_reg[47] = ':';

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }
    i2c_write(VC13_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[48] = i2c_read(true);
    

    afe_isr_reg[49] = ':';
  
    i2c_write(VC13_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[50] = i2c_read(true);
    i2c_stop();

    afe_isr_reg[51] = ':';
    
    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }
    
    i2c_write(VC14_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[52] = i2c_read(true);
   

    afe_isr_reg[53] = ':';
  
    i2c_write(VC14_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[54] = i2c_read(true);
    i2c_stop();

    afe_isr_reg[55] = ':';

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }
    
    i2c_write(VC15_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[56] = i2c_read(true);
   

    afe_isr_reg[57] = ':';
  
    i2c_write(VC15_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[58] = i2c_read(true);
    i2c_stop();

    afe_isr_reg[59] = ':';

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }
    
    i2c_write(VP_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[60] = i2c_read(true);
    

    afe_isr_reg[61] = ':';
  
    i2c_write(VP_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[62] = i2c_read(true);
    i2c_stop();

    afe_isr_reg[63] = ':';

     if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }
  
    i2c_write(CC_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[64] = i2c_read(true);
    

    afe_isr_reg[65] = ':';
  
    i2c_write(CC_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[66] = i2c_read(true);
    i2c_stop();

     if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }

    afe_isr_reg[67] = ':';

    i2c_write(OV_TRIP);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[68] = i2c_read(true);


    afe_isr_reg[69] = ':';

    i2c_write(OV_TRIP);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[70] = i2c_read(true);
    i2c_stop();

    afe_isr_reg[71] = ':';


    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }

    i2c_write(PROTECT1);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[72] = i2c_read(true);
    

    afe_isr_reg[73] = ':';

    i2c_write(PROTECT1);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[74] = i2c_read(true);
    

    afe_isr_reg[75] = ':';

    i2c_write(PROTECT1);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[76] = i2c_read(true);
   

    afe_isr_reg[77] = ':';

    
    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }
    
    i2c_write(SYS_CTRL1);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[78] = i2c_read(true);

    afe_isr_reg[79] = ':';

    i2c_write(SYS_CTRL1);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[80] = i2c_read(true);
    i2c_stop();


    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }
    afe_isr_reg[81] = ':';

    i2c_write(CELLBAL1);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[82] = i2c_read(true);
    

    afe_isr_reg[83] = ':';

    i2c_write(CELLBAL1);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[84] = i2c_read(true);
    

    afe_isr_reg[85] = ':';

    i2c_write(CELLBAL1);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[86] = i2c_read(true);
    i2c_stop();

    afe_isr_reg[87] = ':';

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }

    i2c_write(SYS_STAT);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[88] = i2c_read(true);
    i2c_stop();
    
   if(erom_cng)
   {
    /**Debug Statement*/
    afe_isr_reg[89] = ':';
    afe_isr_reg[90] = eeprom_read_byte(FADD_SYS_STATWORD_HI);
    afe_isr_reg[91] = ':';
    afe_isr_reg[92] = eeprom_read_byte(FADD_SYS_STATWORD_LO);
    afe_isr_reg[93] = ':';
    afe_isr_reg[94] = eeprom_read_byte(FADD_C_OV_CON_H);
    afe_isr_reg[95] = ':';
    afe_isr_reg[96] = eeprom_read_byte(FADD_C_OV_CON_L);
    afe_isr_reg[97] = ':';
    afe_isr_reg[98] = eeprom_read_byte(FADD_C_OV_COUNT);   
    afe_isr_reg[99] = ':';
    afe_isr_reg[100] = eeprom_read_byte(FADD_C_UV_CON_H);
    afe_isr_reg[101] = ':';
    afe_isr_reg[102] = eeprom_read_byte(FADD_C_UV_CON_L);
    afe_isr_reg[103] = ':';
    afe_isr_reg[104] = eeprom_read_byte(FADD_C_UV_COUNT);
    afe_isr_reg[105] = ':';
    afe_isr_reg[106] = eeprom_read_byte(FADD_P_OV_TRIP_H);
    afe_isr_reg[107] = ':';
    afe_isr_reg[108] = eeprom_read_byte(FADD_P_OV_TRIP_L);
    afe_isr_reg[109] = ':';
    afe_isr_reg[110] = eeprom_read_byte(FADD_P_OV_CON_H);
    afe_isr_reg[111] = ':';
    afe_isr_reg[112] = eeprom_read_byte(FADD_P_OV_CON_L);
    afe_isr_reg[113] = ':';
    afe_isr_reg[114] = eeprom_read_byte(FADD_P_OV_COUNT);
    afe_isr_reg[115] = ':';
    afe_isr_reg[116] = eeprom_read_byte(FADD_P_UV_TRIP_H);
    afe_isr_reg[117] = ':';
    afe_isr_reg[118] = eeprom_read_byte(FADD_P_UV_TRIP_L);
    afe_isr_reg[119] = ':';
    afe_isr_reg[120] = eeprom_read_byte(FADD_P_UV_CON_H);
    afe_isr_reg[121] = ':';
    afe_isr_reg[122] = eeprom_read_byte(FADD_P_UV_CON_L);
    afe_isr_reg[123] = ':';
    afe_isr_reg[124] = eeprom_read_byte(FADD_P_UV_COUNT);
    afe_isr_reg[125] = ':';
    afe_isr_reg[126] = eeprom_read_byte(FADD_C_OT_TRIP_H);
    afe_isr_reg[127] = ':';
    afe_isr_reg[128] = eeprom_read_byte(FADD_C_OT_TRIP_L);
    afe_isr_reg[129] = ':';
    afe_isr_reg[130] = eeprom_read_byte(FADD_C_OT_CON_H);
    afe_isr_reg[131] = ':';
    afe_isr_reg[132] = eeprom_read_byte(FADD_C_OT_CON_L);
    afe_isr_reg[133] = ':';
    afe_isr_reg[134] = eeprom_read_byte(FADD_C_OT_COUNT);
    afe_isr_reg[135] = ':';
    afe_isr_reg[136] = eeprom_read_byte(FADD_C_UT_TRIP_H);
    afe_isr_reg[137] = ':';
    afe_isr_reg[138] = eeprom_read_byte(FADD_C_UT_TRIP_L);
    afe_isr_reg[139] = ':';
    afe_isr_reg[140] = eeprom_read_byte(FADD_C_UT_CON_H);
    afe_isr_reg[141] = ':';
    afe_isr_reg[142] = eeprom_read_byte(FADD_C_UT_CON_L);
    afe_isr_reg[143] = ':';
    afe_isr_reg[144] = eeprom_read_byte(FADD_C_UT_COUNT);
    afe_isr_reg[145] = ':';
    afe_isr_reg[146] = eeprom_read_byte(FADD_D_OT_TRIP_H);
    afe_isr_reg[147] = ':';
    afe_isr_reg[148] = eeprom_read_byte(FADD_D_OT_TRIP_L);
    afe_isr_reg[149] = ':';
    afe_isr_reg[150] = eeprom_read_byte(FADD_D_OT_CON_H);
    afe_isr_reg[151] = ':';
    afe_isr_reg[152] = eeprom_read_byte(FADD_D_OT_CON_L);
    afe_isr_reg[153] = ':';
    afe_isr_reg[154] = eeprom_read_byte(FADD_D_OT_COUNT);
    afe_isr_reg[155] = ':';
    afe_isr_reg[156] = eeprom_read_byte(FADD_D_UT_TRIP_H);
    afe_isr_reg[157] = ':';
    afe_isr_reg[158] = eeprom_read_byte(FADD_D_UT_TRIP_L);
    afe_isr_reg[159] = ':';
    afe_isr_reg[160] = eeprom_read_byte(FADD_D_UT_CON_H);
    afe_isr_reg[161] = ':';
    afe_isr_reg[162] = eeprom_read_byte(FADD_D_UT_CON_L);
    afe_isr_reg[163] = ':';
    afe_isr_reg[164] = eeprom_read_byte(FADD_D_UT_COUNT);
    afe_isr_reg[165] = ':';
    afe_isr_reg[166] = eeprom_read_byte(FADD_OCC_TRIP_H);
    afe_isr_reg[167] = ':';
    afe_isr_reg[168] = eeprom_read_byte(FADD_OCC_TRIP_L);
    afe_isr_reg[169] = ':';
    afe_isr_reg[170] = eeprom_read_byte(FADD_OCC_CON_H);
    afe_isr_reg[171] = ':';
    afe_isr_reg[172] = eeprom_read_byte(FADD_OCC_CON_L);
    afe_isr_reg[173] = ':';
    afe_isr_reg[174] = eeprom_read_byte(FADD_OCC_COUNT);
    afe_isr_reg[175] = ':';
    afe_isr_reg[176] = eeprom_read_byte(FADD_OCD_CON_H);
    afe_isr_reg[177] = ':';
    afe_isr_reg[178] = eeprom_read_byte(FADD_OCD_CON_L);
    afe_isr_reg[179] = ':';
    afe_isr_reg[180] = eeprom_read_byte(FADD_OCD_COUNT);
    afe_isr_reg[181] = ':';
    afe_isr_reg[182] = eeprom_read_byte(FADD_P_OT_TRIP_H);
    afe_isr_reg[183] = ':';
    afe_isr_reg[184] = eeprom_read_byte(FADD_P_OT_TRIP_L);
    afe_isr_reg[185] = ':';
    afe_isr_reg[186] = eeprom_read_byte(FADD_P_OT_CON_H);
    afe_isr_reg[187] = ':';
    afe_isr_reg[188] = eeprom_read_byte(FADD_P_OT_CON_L);
    afe_isr_reg[189] = ':';
    afe_isr_reg[190] = eeprom_read_byte(FADD_P_OT_COUNT);
    afe_isr_reg[191] = ':';
    afe_isr_reg[192] = eeprom_read_byte(FADD_P_UT_TRIP_H);
    afe_isr_reg[193] = ':';
    afe_isr_reg[194] = eeprom_read_byte(FADD_P_UT_TRIP_L);
    afe_isr_reg[195] = ':';
    afe_isr_reg[196] = eeprom_read_byte(FADD_P_UT_CON_H);
    afe_isr_reg[197] = ':';
    afe_isr_reg[198] = eeprom_read_byte(FADD_P_UT_CON_L);
    afe_isr_reg[199] = ':';
    afe_isr_reg[200] = eeprom_read_byte(FADD_P_UT_COUNT);
    afe_isr_reg[201] = ':';
    afe_isr_reg[202] = eeprom_read_byte(FADD_SCC_TRIP_H);
    afe_isr_reg[203] = ':';
    afe_isr_reg[204] = eeprom_read_byte(FADD_SCC_TRIP_L);
    afe_isr_reg[205] = ':';
    afe_isr_reg[206] = eeprom_read_byte(FADD_SCC_CON_H);
    afe_isr_reg[207] = ':';
    afe_isr_reg[208] = eeprom_read_byte(FADD_SCC_CON_L);
    afe_isr_reg[209] = ':';
    afe_isr_reg[210] = eeprom_read_byte(FADD_SCC_COUNT);
    afe_isr_reg[211] = ':';
    afe_isr_reg[212] = eeprom_read_byte(FADD_SCD_CON_H);
    afe_isr_reg[213] = ':';
    afe_isr_reg[214] = eeprom_read_byte(FADD_SCD_CON_L);
    afe_isr_reg[215] = ':';
    afe_isr_reg[216] = eeprom_read_byte(FADD_SCD_COUNT);
    afe_isr_reg[217] = ':';
    afe_isr_reg[218] = eeprom_read_byte(FADD_OP_TRIP_H);
    afe_isr_reg[219] = ':';
    afe_isr_reg[220] = eeprom_read_byte(FADD_OP_TRIP_L);
    afe_isr_reg[221] = ':';
    afe_isr_reg[222] = eeprom_read_byte(FADD_OP_CON_H);
    afe_isr_reg[223] = ':';
    afe_isr_reg[224] = eeprom_read_byte(FADD_OP_CON_L);
    afe_isr_reg[225] = ':';
    afe_isr_reg[226] = eeprom_read_byte(FADD_OP_COUNT);
    afe_isr_reg[235] = ':';
    erom_cng = 0;
    Serial.println("Values Updated from EEPROM to afe_isr[x] buffer");
   }

  if(temp_read > 5)
  {
  afe_isr_reg[227] = ':';
  if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }
  
  i2c_write(TS1_HI);
  i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
  afe_isr_reg[228] = i2c_read(true);
 

  afe_isr_reg[229] = ':';
  
  i2c_write(TS1_HI);
  i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
  afe_isr_reg[230] = i2c_read(true);
  i2c_stop();

  afe_isr_reg[231] = ':';

  if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      /**Debug Statement*/
      Serial.println("I2C device busy");
      _delay_ms(1000);
      return;
    }
    
  i2c_write(TS2_HI);
  i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
  afe_isr_reg[232] = i2c_read(true);

  afe_isr_reg[233] = ':';

  i2c_write(TS2_HI);
  i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
  afe_isr_reg[234] = i2c_read(true);
  i2c_stop();
  temp_read=0;
  }
  sei();
  i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0x80);
  //Serial.println("Read Sucess");
}



