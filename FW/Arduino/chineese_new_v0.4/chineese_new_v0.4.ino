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

#define FOSC 8000000 // Clock Speed
#define MYUBRR FOSC/16/BAUD-1

/************************************************Communication Codes**********************************************/
/** Maximum Buffer Size for Serial data*/
#define COMM_MAX_BUFFER_SZ 235
/** No data available on Serial*/
#define NO_DATA_ON_SERIAL 0

/*************************************************Communication command no for processing************************/
/** Communication Command processing Error */
#define COMM_CMD_ERROR        0
/** Communication command for buy serial*/
#define COMM_CMD_BUSY         1
/**Communication command for dashboard*/
#define COMM_CMD_DB           2
/**Communication command for dashboard*/
#define COMM_CMD_BSBI         3
/**Communication command for dashboard*/
#define COMM_CMD_BSCV         4
/**Communication command for dashboard*/
#define COMM_CMD_BSPI         5
/**Communication command for dashboard*/
#define COMM_CMD_PS           6
/**Communication command for dashboard*/
#define COMM_CMD_CONFIG       7

/**********************************************Size definations for communication commands***************************/
/**Size for data log command*/
#define SZ_CMD_DB             2
/**Size for data log command*/
#define SZ_CMD_BSBI           4
/**Size for data log command*/
#define SZ_CMD_BSCV           4
/**Size for data log command*/
#define SZ_CMD_BSPI           4
/**Size for data log command*/
#define SZ_CMD_PS             2
/**Size for data log command*/
#define SZ_CMD_CONFIG         6





      
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

/**Function to initiallize GPIO and set i2c bus and select mode of AFE*/
void i2c_initz(void);
/**Function to load defaults from EEPROM to AFE*/
void afe_initz(void);
void USART_Init( unsigned int baud);
void UART_SendData8(uint8_t data);
/* Send 1 byte of Data */
void TU_send(uint8_t data);
/* Sends out a NULL terminated string of data */
void TU_puts(uint8_t *array);
/* Send out a NULL terminated string of data with Line Terminator */
void TU_putln(uint8_t *array);
uint8_t UART_ReceiveData8(void);
uint8_t TU_getc(uint8_t *data);
/* Send one byte Hex with '0x' prefix on the serial port */
void TU_putHex(uint8_t data);
void db(void);
void i2c_WriteWithCRC(uint8_t I2CSlaveAddress, uint8_t Register, uint8_t Data);
unsigned char CRC8(unsigned char *ptr, unsigned char len,unsigned char key);
uint8_t str2hex(char s);


void db_log(void);
void bsbi_log(void);



void error(void);
/* Command Processor with Line Ending '\r\n' termination */
uint8_t TU_getln(uint8_t *array, uint8_t max_sz);
uint8_t Comm_loop(void);
void init_PCI(void);
/* Send two byte Hex word with '0x' prefix on the serial port */
void TU_putHexW(uint16_t data);
/*Function to callibrate current*/
void ccbf(uint16_t ctcb);
void calsoc(uint16_t ct);
uint8_t tempcal(uint8_t tempeh, uint8_t tempel);




const char cmd_db[SZ_CMD_DB]          =  "db";
const char cmd_bsbi[SZ_CMD_BSBI]      =  "bsbi";
const char cmd_bscv[SZ_CMD_BSCV]      =  "bscv";
const char cmd_bspi[SZ_CMD_BSPI]      =  "bspi";
const char cmd_ps[SZ_CMD_PS]          =  "ps";
const char cmd_config[SZ_CMD_CONFIG]  =  "config";




volatile uint8_t afe_isr_reg[136], temp_read=0;
volatile bool erom_cng=1;
volatile uint8_t serial_command;
/** Exposed Buffer for Serial data reception */
uint8_t comm_buffer[COMM_MAX_BUFFER_SZ];
/** Exposed size of the Communication Buffer */
uint8_t comm_sz;
/** Exposed size of the Command Processed */
uint8_t comm_cmd_sz;
/** Exposed pointer to next part of Command for further processing */
uint8_t* comm_ptr_next;
volatile uint8_t isr_count=0,isr_count2=0;
volatile uint16_t ccf;
volatile uint16_t ct_sum;
volatile uint8_t ct_sum_count;
volatile uint16_t cmax=0x0000,cmin=0xFFFF;
uint8_t temph,templ;
uint8_t currh,currl;



/**Interrupt service routine*/
ISR(PCINT1_vect)
{
  /**Clear CC_READY bit in system status register*/
  i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0xFF);
  isr_count++;
  //TU_send('A');
}







void setup() 
{
  
  i2c_initz();

  USART_Init(9600);
  
  afe_initz();

  init_PCI();
}






                                                                                                void loop() 
                                                                                                {
                                                                                                   if(isr_count > 3)
                                                                                                    {
                                                                                                      cli();
                                                                                                      db();
                                                                                                      //trip();
                                                                                                      //rel();
                                                                                                      isr_count=0;
                                                                                                      sei();
                                                                                                      i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0xFF);
                                                                                                    }
            
                                                                                                    // Command Loop
                                                                                                    serial_command = Comm_loop();
                                                                                                
                                                                                                    // Process the Serial Commands & Do Business Logic
                                                                                                    BL_Process(serial_command);

                                                                                                }





















/**Function to initiallize GPIO and set i2c bus and select mode of AFE*/
void i2c_initz(void)
{
  /**Set PINB0 pin as OUTPUT*/
  DDRB |= 0b00000001;
  /**Set PINC0 pin as OUTPUT*/
  DDRC |= 0b00000001;
  /**Set PINB0 pin in LOW state*/
  PORTB &= ~(1<<PINB0);
  /**Set PINC0 pin in HIGH state*/
  PORTC |= 1<<PINC0;
}
/**Function ends here*/



/**Function to load defaults from EEPROM to AFE*/
void afe_initz(void)
{
  /*
   *function to load default settings from eeprom to local array;
   *load values from array to afe 
   *
   */
   TU_putln("Writting values to AFE");
   
   ldsett();
   i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL1, afe_isr_reg[128]);
   i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, afe_isr_reg[129]);
   i2c_WriteWithCRC(I2C_7BITADDR, PROTECT1, afe_isr_reg[130]);
   i2c_WriteWithCRC(I2C_7BITADDR, PROTECT2, afe_isr_reg[131]);
   i2c_WriteWithCRC(I2C_7BITADDR, PROTECT3, afe_isr_reg[132]);  
   i2c_WriteWithCRC(I2C_7BITADDR, CC_CFG, afe_isr_reg[133]);
   i2c_WriteWithCRC(I2C_7BITADDR, OV_TRIP, afe_isr_reg[87]);
   i2c_WriteWithCRC(I2C_7BITADDR, UV_TRIP, afe_isr_reg[85]);
   i2c_WriteWithCRC(I2C_7BITADDR, CELLBAL1,  afe_isr_reg[134]);
   i2c_WriteWithCRC(I2C_7BITADDR, CELLBAL2,  afe_isr_reg[135]);
   i2c_WriteWithCRC(I2C_7BITADDR, CELLBAL3,  afe_isr_reg[136]);


   TU_putln("Values written to AFE sucessfully");
}








/**Function to load default values from EEPROM to local array*/
void ldsett(void)
{
  TU_putln("Loading values from EEPROM");
    /**Load fault status higher 8 bits from EEPROM to local array*/
    afe_isr_reg[125] = eeprom_read_byte(0x00);
    /**Load fault status lower 8 bits from EEPROM to local array*/
    afe_isr_reg[126] = eeprom_read_byte(0x01);
    /**Load system status from EEPROM to local array*/
    //afe_isr_reg[127] = eeprom_read_byte(0x02);
    /**Load system control 1 from EEPROM to local array*/
    afe_isr_reg[128] = eeprom_read_byte(0x03);
    /**Load system control 2 from EEPROM to local array*/
    afe_isr_reg[129] = eeprom_read_byte(0x04);
    /**Load protect 1 from EEPROM to local array*/
    afe_isr_reg[130] = eeprom_read_byte(0x05);
    /**Load protect 2 from EEPROM to local array*/
    afe_isr_reg[131] = eeprom_read_byte(0x06);
    /**Load protect 3 from EEPROM to local array*/
    afe_isr_reg[132] = eeprom_read_byte(0x07);
    /**Load cc config from EEPROM to local array*/
    afe_isr_reg[133] = eeprom_read_byte(0x08);
    /**Load cell overvoltage threshold from EEPROM to local array*/
    afe_isr_reg[87] = eeprom_read_byte(0x09);
    /**Load cell over voltage release high order bits from EEPROM to local array*/
    afe_isr_reg[101] = eeprom_read_byte(0x0A);
    /**Load cell over voltage release low order bits from EEPROM to local array*/
    afe_isr_reg[102] = eeprom_read_byte(0x0B);
    /**Load cell over voltage times high order bits from EEPROM to local array*/
    afe_isr_reg[73] = eeprom_read_byte(0x0C);
    /**Load cell over voltage times low order bits from EEPROM to local array*/
    afe_isr_reg[74] = eeprom_read_byte(0x0D);
    /**Load cell undervoltage threshold from EEPROM to local array*/
    afe_isr_reg[85] = eeprom_read_byte(0x0E);
    /**Load cell under voltage release high order bits from EEPROM to local array*/
    afe_isr_reg[99] = eeprom_read_byte(0x0F);
    /**Load cell under voltage release low order bits from EEPROM to local array*/
    afe_isr_reg[100] = eeprom_read_byte(0x10);
    /**Load cell under voltage trip times high order bits from EEPROM to local array*/
    afe_isr_reg[75] = eeprom_read_byte(0x11);
    /**Load cell under voltage trip times low order bits from EEPROM to local array*/
    afe_isr_reg[76] = eeprom_read_byte(0x12);
    /**Load pack overvoltage high bits threshold from EEPROM to local array*/
    afe_isr_reg[89] = eeprom_read_byte(0x13);
    /**Load pack overvoltage low bits threshold from EEPROM to local array*/
    afe_isr_reg[90] = eeprom_read_byte(0x14);
    /**Load pack overvoltage high bits release from EEPROM to local array*/
    afe_isr_reg[103] = eeprom_read_byte(0x15);
    /**Load pack overvoltage lower bits release from EEPROM to local array*/
    afe_isr_reg[104] = eeprom_read_byte(0x16);
    /**Load pack over voltage trip times high bits from EEPROM to local array*/
    afe_isr_reg[77] = eeprom_read_byte(0x17);
    /**Load pack over voltage trip times low bits from EEPROM to local array*/
    afe_isr_reg[78] = eeprom_read_byte(0x18);
    /**Load pack under voltage trip high bits from EEPROM to local array*/
    afe_isr_reg[91] = eeprom_read_byte(0x19);
  /**Load pack under voltage trip low bits from EEPROM to local array*/
    afe_isr_reg[92] = eeprom_read_byte(0x1A);
  /**Load pack under voltage release high bits from EEPROM to local array*/
    afe_isr_reg[105] = eeprom_read_byte(0x1B);
  /**Load pack under voltage release low bits from EEPROM to local array*/
    afe_isr_reg[106] = eeprom_read_byte(0x1C);
  /**Load pack under voltage trip high bits times from EEPROM to local larray*/
    afe_isr_reg[79] = eeprom_read_byte(0x1D);
  /**Load pack under voltage trip low bits times from EEPROM to local larray*/
    afe_isr_reg[80] = eeprom_read_byte(0x1E);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x1F);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x20);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x21);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x22);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x23);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x24);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x25);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x26);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x27);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x28);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x29);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x2A);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x2B);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x2C);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x2D);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x2E);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x2F);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x30);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x31);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x32);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x33);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x34);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x35);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x36);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x37);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x38);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x39);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x3A);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x3B);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x3B);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x3B);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x3B);
  //afe_isr_reg_reg[] = eeprom_read_byte(0x3B);

  /**Load value of cell bal 1*/
    afe_isr_reg[134] = eeprom_read_byte(0x40);
  /**Load value of cell bal 2*/
    afe_isr_reg[135] = eeprom_read_byte(0x41);
  /**Load value of cell bal 3*/
    afe_isr_reg[136] = eeprom_read_byte(0x42);
    
    TU_putln("Values loadaed from EEPROM sucessfully");
}
/**End of function*/












/*Function to setup pin change interrupt*/
void init_PCI(void)
{
  /*Clear all global interrupts*/
  cli();
  /*Set pin change interrupt control register*/
  PCICR  |= (1<<1);
  /*Set pin change mask register 1*/
  PCMSK1 |= (1<<3);
  /*Enable global interrupts*/
  sei();
  i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0xFF);
}



void USART_Init( unsigned int baud)
{
  volatile uint8_t ubbr;
  ubbr = FOSC/16/baud-1;
  /*Set baud rate */
  UBRR0H = (unsigned char)(ubbr>>8);
  UBRR0L = (unsigned char)ubbr;
  /*Enable receiver and transmitter */
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  /* Set frame format: 8data, 2stop bit */
  UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}


void UART_SendData8(uint8_t data)
{
  /* Wait for empty transmit buffer */
  while ( !( UCSR0A & (1<<UDRE0)) )
  ;
  /* Put data into buffer, sends the data */
  UDR0 = data;
}


/* Send 1 byte of Data */
void TU_send(uint8_t data)
{
  /* Wait for empty transmit buffer */
  while ( !( UCSR0A & (1<<UDRE0)) );
  UART_SendData8(data);
}

/* Sends out a NULL terminated string of data */
void TU_puts(uint8_t *array)
{
  uint8_t *p = array;
  while(*p)
  {
    /* Wait for empty transmit buffer */
    while ( !( UCSR0A & (1<<UDRE0)) );
    UART_SendData8(*p);
    ++p;
  }
}

/* Send out a NULL terminated string of data with Line Terminator */
void TU_putln(uint8_t *array)
{
  TU_puts(array);
  TU_puts("\r\n");
}





/**
  * @brief  Returns the most recent received data by the UART2 peripheral.
  * @param  None
  * @retrieval Received Data
  */
uint8_t UART_ReceiveData8(void)
{
  return UDR0;
}


/* Detects if any things is available in the internal buffer 
   then received it in a pointed location 
   If there is nothing then 0 is returned 
*/
uint8_t TU_getc(uint8_t *data)
{
  if((UCSR0A & (1<<RXC0)))
  {
    *data = UART_ReceiveData8();
    return 1;
  }
  return 0;
}


/* Send one byte Hex with '0x' prefix on the serial port */
void TU_putHex(uint8_t data)
{
  uint8_t tmp;
  //TU_puts("0x");
  tmp = (data >> 4) & 0x0F;
  tmp = (tmp > 9)?('A' + tmp - 10):('0' + tmp);
  TU_send(tmp);
  tmp = data & 0x0F;
  tmp = (tmp > 9)?('A' + tmp - 10):('0' + tmp);
  TU_send(tmp);
}















/* if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putln("I2C device busy");
      _delay_ms(1000);
      return;
    }

    i2c_write(VC1_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg_reg[0] = i2c_read(true);*/


/* if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putln("I2C device busy");
      _delay_ms(1000);
      return;
    }

    i2c_write(TS1_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    temph = i2c_read(true);

    i2c_write(TS1_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    templ = i2c_read(true);
    i2c_stop();

    afe_isr_reg[11] = tempcal(temph,templ);

    temph=templ=0;

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putln("I2C device busy");
      _delay_ms(1000);
      return;
    }

    i2c_write(TS2_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    temph = i2c_read(true);

    i2c_write(TS2_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    templ = i2c_read(true);
    i2c_stop();

    afe_isr_reg[12] = tempcal(temph,templ);
    temph=templ=0;
  
  sei();
  i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0xFF);
  //TU_putln("Read Sucess");*/






void db(void)
{
  uint8_t mos;

  if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putln("I2C device busy");
      _delay_ms(1000);
      return;
    }

    i2c_write(TS2_HI);   //VP_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[2] = i2c_read(true);

    i2c_write(TS2_HI);   //VP_LO
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[3] = i2c_read(true);
    i2c_stop();

    /**Reading fresh current measurement*/
    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putln("I2C device busy");
      _delay_ms(1000);
      return;
    }

    i2c_write(TS3_HI);      //CC_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[4] = i2c_read(true);

    i2c_write(TS3_HI);       //CC_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[5] = i2c_read(true);      
    i2c_stop();

    ccbf(0x0000);

    afe_isr_reg[6] = currh;       //dummy data loaded need to be calculated

    afe_isr_reg[7] = currl;       //dummy data loaded need to be calculated

    afe_isr_reg[8] = 0x32;        //dummy data loaded need to be calculated

    afe_isr_reg[9] = 0x0F;        //dummy data loaded need to be calculated
    
    /** Store and map mosfet status from AFE*/
    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putln("I2C device busy");
      _delay_ms(1000);
      return;
    }

    i2c_write(SYS_CTRL2);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    mos = i2c_read(true);
    afe_isr_reg[10] = (mos&0x03);
    i2c_stop();
}


void error(void)
{
 // Serial.println("ERROR");
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





uint8_t str2hex(char s)
{
  volatile uint8_t ret = 0;
  
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


void db_log(void)
{
  cli();  
  for(volatile int g=0; g<11; g++)
  {
    TU_putHex(afe_isr_reg[g]);
    if(g > 9)
    {
      TU_send('\r');
      TU_send('\n');
    }
    
    else
    TU_send(':');
  }
  sei();
  i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0xFF);
}


void bsbi_log(void)
{
  cli();  
  for(volatile int g=11; g<28; g++)
  {
    TU_putHex(afe_isr_reg[g]);
    if(g > 26)
    {
      TU_send('\r');
      TU_send('\n');
    }
    
    else
    TU_send(':');
  }
  sei();
  i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0x80);
}


void bscv_log(void)
{
  cli();  

  for(volatile int g=28; g<56; g++)
  {
    TU_putHex(afe_isr_reg[g]);
    if(g > 54)
    {
      TU_send('\r');
      TU_send('\n');
    }
    
    else
     TU_send(':');
  }
  sei();
  i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0x80);
}


void bspi_log(void)
{
  cli();  

  for(volatile int g=28; g<56; g++)
  {
    TU_putHex(afe_isr_reg[g]);
    if(g > 54)
    {
      TU_send('\r');
      TU_send('\n');
    }
    
    else
    TU_send(':');
  }
  sei();
  i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0x80);
}

void ps_log(void)
{
  cli();  

  for(volatile int g=28; g<56; g++)
  {
    TU_putHex(afe_isr_reg[g]);
    if(g > 54)
    {
      TU_send('\r');
      TU_send('\n');
    }
    
    else
    TU_send(':');
  }
  sei();
  i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0x80);
}

void configw(void)
{
  TU_putln("Writting configuration.....");
}



/* Command Processor with Line Ending '\r\n' termination */
uint8_t TU_getln(uint8_t *arr, uint8_t max_sz)
{
  static uint8_t nCount = 0; // State Counter for Reception
  static uint8_t tm = 0; // Termination Marker 1-First Terminator 0-Nothing
  uint8_t dt, ret = 0;
  
  // Works only if there is some thing in the RX
  while(TU_getc(&dt))
  {
    switch(dt)
    {
      case '\r': // First Terminator Received
        tm = 1; // Alert that First terminator Received
        break;
    
      case '\n': // Second Terminator
        if(tm == 1) // Last terminator received
        {
          tm = 0; // Completed the sequence
          arr[nCount] = 0; // Insert the NULL terminator
          if(nCount == 0) // For 1 character as NULL for only EOL Key
          {
            ret = 1;
          }
          else // In case we had bytes in the Array
          {
            ret = nCount; // Return Array size
            nCount = 0; // Reset the sequence
          }
        }
        else // Wrong terminator received
        {
          ret = 0;  // Clear Buffer Data
          nCount = 0;
          arr[nCount] = 0; // Insert the NULL terminator
        }
        break;
      case '\b': // Backspace received
        if(nCount) // We were already in middle of receiving some thing
        {
          arr[nCount] = 0; // Clear the Last Received
          --nCount; // Decrement the counter
        }
        else // Clear the Data as correctly we do not need backspace
        {
          dt = 0;
        }
        break;
      default:
        // Already receiving the Packet
        if(nCount)
        {
          if(nCount < (max_sz-1)) // Check for space in Buffer added for NULL
          {
            arr[nCount++] = dt;
          }
          else // Buffer is Full need to Return
          {           
            arr[nCount] = 0; // Insert the NULL terminator
            ret = nCount; // Return Array size
            nCount = 0; // Reset the sequence
          }
        }
        else // Initiate the Process of Reception
        {
          arr[nCount++] = dt;// Start the Reception
        }
        break;
    }// End of the Selector Switch(dt)
    
    // Send back the Character (If not Null)
    if(dt)
      TU_send(dt);
    
  }// End of While TU_getc
  
  return ret;
}






/* Communications Processing Loop */
uint8_t Comm_loop(void)
{
  // Check if Data is available
  comm_sz = TU_getln(comm_buffer,COMM_MAX_BUFFER_SZ);
  //If some thing have been receive
  if(comm_sz)
  {
    return process();
  }

  // Nothing to Return - If the command was not there
  return COMM_CMD_BUSY;
}



/* Business Logic Processing Loop */
uint8_t BL_Process(uint8_t serial_cmd)
{
  do{

    if(serial_cmd == COMM_CMD_ERROR)
    {
      TU_putln(" BL: Unknown Command Received ");
      break;
    }

    if(serial_cmd == COMM_CMD_DB)
    {
      db_log();
      break;
    }

    if(serial_cmd == COMM_CMD_BSBI)
    {
      bsbi_log();
      break;
    }

    if(serial_cmd == COMM_CMD_BSCV)
    {
      bscv_log();
      break;
    }

    if(serial_cmd == COMM_CMD_BSPI)
    {
      bspi_log();
      break;
    }

    if(serial_cmd == COMM_CMD_PS)
    {
      ps_log();
      break;
    }

    if(serial_cmd == COMM_CMD_CONFIG)
    {
      configw();
      break;
    }

  }while(0);

  return 0;
}






uint8_t process(void)
{
  uint8_t ret = COMM_CMD_ERROR; // Default Error in Processing
  comm_ptr_next = comm_buffer; // Default First Location
  comm_cmd_sz = 0;  // By Default there is no next command

  do{

    //Check for DB Command
    if(strncmp(cmd_db, comm_buffer, SZ_CMD_DB) == 0)
    {
      
      ret = COMM_CMD_DB;
      break;
    }

    //Check for BSBI Command
    if(strncmp(cmd_bsbi, comm_buffer, SZ_CMD_BSBI) == 0)
    {
      
      ret = COMM_CMD_BSBI;
      break;
    }

    //Check for BSCV Command
    if(strncmp(cmd_bscv, comm_buffer, SZ_CMD_BSCV) == 0)
    {
      
      ret = COMM_CMD_BSCV;
      break;
    }

     //Check for BSCV Command
    if(strncmp(cmd_bspi, comm_buffer, SZ_CMD_BSPI) == 0)
    {
      
      ret = COMM_CMD_BSPI;
      break;
    }

     //Check for BSCV Command
    if(strncmp(cmd_ps, comm_buffer, SZ_CMD_PS) == 0)
    {
      
      ret = COMM_CMD_PS;
      break;
    }

     //Check for BSCV Command
    if(strncmp(cmd_config, comm_buffer, SZ_CMD_CONFIG) == 0)
    {
      
      ret = COMM_CMD_CONFIG;
      break;
    }

  }while(0);

  // Return the Final Command after processing
  return ret;
}



/* Send two byte Hex word with '0x' prefix on the serial port */
void TU_putHexW(uint16_t data)
{
  uint8_t tmp;
  //TU_puts("0x");
  tmp = (data >> 12) & 0x0F;
  tmp = (tmp > 9)?('A' + tmp - 10):('0' + tmp);
  TU_send(tmp);
  tmp = (data >> 8) & 0x0F;
  tmp = (tmp > 9)?('A' + tmp - 10):('0' + tmp);
  TU_send(tmp);
  tmp = (data >> 4) & 0x0F;
  tmp = (tmp > 9)?('A' + tmp - 10):('0' + tmp);
  TU_send(tmp);
  tmp = data & 0x0F;
  tmp = (tmp > 9)?('A' + tmp - 10):('0' + tmp);
  TU_send(tmp);
}

/*Function to callibrate current*/
void ccbf(uint16_t ctcb)
{
  /*
   * callibration of current:
   * 
   * recieved current in miliamps as 16bit in hex
   * store hex in local variable
   * read cc value in a 16 bit variable
   * ccf = miliamps hex/current cc value
   * store ccf value in eeprom current callibration factor address
   * raise flag to refresh values from eeprom to current array
   * 
   */

   currh = 0x01;
   currl = 0xA5;
}


void calsoc(uint16_t ct)
{

  ct_sum = ct_sum+ct;
  ct_sum_count++;
  if(ct_sum_count > 99)
  {
//    ct_sum = (ct_sum/3600)*current callibration factor;
    //save to eeprom
  }
}



uint16_t merge_16(uint8_t h_16, uint8_t l_16)
{
  uint16_t ret_16_merge=0;
  ret_16_merge = (h_16 << 8) | (l_16 & 0xff);
  return ret_16_merge;
}







void trip(void)
{
  /**Checks for cell over voltage fault*/
  if((afe_isr_reg[127]&0x04))
  {
    afe_isr_reg[0] ^= 0x01; 
    afe_isr_reg[125] ^= 0x04; 
  }

  /**Checks for cell under voltage*/
  if((afe_isr_reg[127]&0x08))
  {
    afe_isr_reg[0] ^= 0x02; 
    afe_isr_reg[125] ^= 0x08; 
  }
  
   /**Checks pack overvoltage fault */
  if(!(afe_isr_reg[125] & 0x01))
  {
    if(merge_16(afe_isr_reg[2],afe_isr_reg[3]) > merge_16(afe_isr_reg[89],afe_isr_reg[90]))
    {
      afe_isr_reg[0] ^= 0x04; 
      afe_isr_reg[125] ^= 0x01;
      i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x40);
    }
  }

  /**Checks pack undervoltage fault*/
  if(!(afe_isr_reg[125] & 0x02))
  {
    if(merge_16(afe_isr_reg[2],afe_isr_reg[3]) < merge_16(afe_isr_reg[91],afe_isr_reg[92]))
    {
      afe_isr_reg[0] ^= 0x08; 
      afe_isr_reg[125] ^= 0x02;
      i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x40);
    }
  }
}










//afe_isr_reg[127]


void rel(void)
{
  /**Checks cell overvoltage fault if occured*/
  if((afe_isr_reg[125] & 0x04))
  {
    cvmaxmin();
    if(cmax < merge_16(afe_isr_reg[101],afe_isr_reg[102]))
    {
      afe_isr_reg[0] ^= 0x01;
      afe_isr_reg[125] ^= 0x04;
      i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0x04);
      i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x43);  
    }
  }



  /**Checks cell undervoltage fault if occured*/
  if((afe_isr_reg[125] & 0x08))
  {
    cvmaxmin();
    if(cmin > merge_16(afe_isr_reg[99],afe_isr_reg[100]))
    {
      afe_isr_reg[0] ^= 0x02;
      afe_isr_reg[125] ^= 0x08; 
      i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0x08);  
      i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x43);
    }
  }


  /**Checks pack overvoltage fault if occured*/
  if((afe_isr_reg[125] & 0x01))
  {
    if(merge_16(afe_isr_reg[2],afe_isr_reg[3]) < merge_16(afe_isr_reg[103],afe_isr_reg[104]))
    {
      afe_isr_reg[0] ^= 0x04; 
      afe_isr_reg[125] ^= 0x01;
      i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x43);
    }
  }

  /**Checks pack undervoltage fault if occured*/
  if((afe_isr_reg[125] & 0x02))
  {
    if(merge_16(afe_isr_reg[2],afe_isr_reg[3]) > merge_16(afe_isr_reg[105],afe_isr_reg[106]))
    {
      afe_isr_reg[0] ^= 0x08;
      afe_isr_reg[125] ^= 0x02;
      i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x43);
    }
  }


  
}











void cvmaxmin(void)
{
  
  for(int i=39; i<66;)
  {
    if(merge_16(afe_isr_reg[i],afe_isr_reg[i+1]) > cmax)
    {
      cmax = merge_16(afe_isr_reg[i],afe_isr_reg[i+1]);
    }

   if(merge_16(afe_isr_reg[i],afe_isr_reg[i+1]) < cmin)
   {
      cmin = merge_16(afe_isr_reg[i],afe_isr_reg[i+1]);
   }

   i=i+2;
  }
}






uint8_t tempcal(uint8_t tempeh, uint8_t tempel)
{
  uint32_t tempcalc;
  tempcalc = merge_16(tempeh, tempel);
  tempcalc = (tempcalc*0x17E);
  tempcalc = (tempcalc/0xF4240);
  tempcalc = ((0x2710*tempcalc)/((0x21/0xA)-tempcalc));
  return tempcalc;
}

