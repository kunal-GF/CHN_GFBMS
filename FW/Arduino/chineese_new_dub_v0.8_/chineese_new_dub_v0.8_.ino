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
#define VC2_HI 0x0E
#define VC3_HI 0x10
#define VC4_HI 0x12
#define VC5_HI 0x14
#define VC6_HI 0x16
#define VC7_HI 0x18
#define VC8_HI 0x1A
#define VC9_HI 0x1C
#define VC10_HI 0x1E
#define VC11_HI 0x20
#define VC12_HI 0x22
#define VC13_HI 0x24
#define VC14_HI 0x26
#define VC15_HI 0x28
#define VP_HI 0x2A
#define TS1_HI 0x2C
#define TS2_HI 0x2E
#define TS3_HI 0x30
#define CC_HI 0x32
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

#define FOSC 16000000 // Clock Speed
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
/**Communication command for dashboard*/
#define COMM_CMD_KEY          8
/**Communication command for dashboard*/
#define COMM_CMD_RHG       9                                                 

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
/**Size for data log command*/
#define SZ_CMD_KEY            6
/**Size for data log command*/
#define SZ_CMD_RHG            6         



#define THERMISTORNOMINAL 0x2710
#define BCOEFFICIENT 0xF6E
#define TEMPERATURENOMINAL 0x19
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
void trip(void);

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
uint8_t calsoc(uint16_t ct);
uint8_t tempcal(uint8_t tempeh, uint8_t tempel);




const char cmd_db[SZ_CMD_DB]          =  "db";
const char cmd_bsbi[SZ_CMD_BSBI]      =  "bsbi";
const char cmd_bscv[SZ_CMD_BSCV]      =  "bscv";
const char cmd_bspi[SZ_CMD_BSPI]      =  "bspi";
const char cmd_ps[SZ_CMD_PS]          =  "ps";
const char cmd_config[SZ_CMD_CONFIG]  =  "config";
const char cmd_key[SZ_CMD_KEY]        =  "key:FF";
const char cmd_rhg[SZ_CMD_RHG]        =  "rhg:F0";




volatile uint8_t afe_isr_reg[] = {0x00,0x00,0x85,0x64,0x00,0x73,0x01,0xA5,0x3C,0x0F,
0x03,0x1D,0x1D,0x28,0x23,0x3C,0x01,0x24,0x39,0x19,0x4F,0x80,0x4F,0x20,0x4F,0x2D,0x05,
0x80,0x8F,0xFF,0x8F,0xFF,0x80,0x00,0x10,0x0A,0x00,0x00,0x00,0x26,0x62,0x26,0x86,0x26,
0x10,0x26,0x15,0x2B,0x87,0x26,0x1E,0x26,0x16,0x26,0x19,0x26,0x19,0x26,0x1D,0x25,0x87,
0x26,0x22,0x26,0x1E,0x26,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0x00,0xB3,0x00,0x86,0x97,0x75,0x4A,0x00,0xE9,
0xFF,0x13,0x23,0x18,0x23,0x50,0x29,0xA1,0x8E,0xC9,0x77,0xD6,0x00,0x23,0x1B,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x73,0x05,0xFC,0x00,0x00,0x00,0x18,
0x40,0x1F,0x50,0xF7,0x19,0x00,0x00,0x00,0x1D,0x00,0x00,0x00,0x1D,0x00,0x02,0x00};

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
volatile uint16_t cmax=0x0000,cmin=0xFFFF;
uint8_t temph1,templ1,temph2,templ2,tempmax1,tempmax2;
uint8_t currh,currl;
uint8_t keytog = 0x00;
uint8_t rhgtog = 0x00;
uint8_t h,l;
uint8_t verbo;

const long keyrhgerval = 60000;
unsigned long keyrhgpreviousmilis = 0;

const long ocscerval = 60000;
unsigned long ocscpreviousmilis = 0;

uint16_t tempmap[86] =
{
0x6AA0,0x6625,0x61E0,0x5DCE,0x59EE,0x563B,0x52B5,0x4F58,0x4C23,0x4913,0x4627,
0x435D,0x40B3,0x3E28,0x3BBA,0x3967,0x372E,0x350F,0x3307,0x3116,0x2F3A,0x2D73,
0x2BBF,0x2A1E,0x288E,0x270F,0x25A1,0x2442,0x22F1,0x21AE,0x2079,0x1F50,0x1E33,
0x1D22,0x1C1C,0x1B20,0x1A2F,0x1947,0x1868,0x1792,0x16C5,0x15FF,0x1541,0x148B,
0x13DC,0x1333,0x1290,0x11F4,0x115E,0x10CD,0x1042,0xFBC,0xF3B,0xEBF,0xE47,0xDD4,
0xD65,0xCFA,0xC92,0xC2F,0xBCF,0xB72,0xB19,0xAC3,0xA70,0xA1F,0x9D2,0x987,0x93F,
0x8F9,0x8B6,0x875,0x836,0x7F9,0x7BF,0x786,0x74F,0x71A,0x6E7,0x6B5,0x685,0x657,
0x62A,0x5FF,0x5D5,0x5AC
};


/**Interrupt service routine*/
ISR(PCINT1_vect)
{
  /**Clear CC_READY bit in system status register*/
  i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0x80);
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
                                                                                                   if(isr_count > 100)
                                                                                                    {
                                                                                                      //cli();
                                                                                                      //db();
                                                                                                      //bsbi();
                                                                                                      //bscv();
                                                                                                      cvmaxmin();
                                                                                                      trip();
                                                                                                      rel();
                                                                                                      isr_count=0;
                                                                                                      //sei();
                                                                                                      //i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0xFF);
                                                                                                    }
            
                                                                                                    // Command Loop
                                                                                                    serial_command = Comm_loop();
                                                                                                
                                                                                                    // Process the Serial Commands & Do Business Logic
                                                                                                    BL_Process(serial_command);

                                                                                                    if(keytog == 0x40)
                                                                                                    {
                                                                                                      chk_key_act();
                                                                                                    }
                                                                                                    isr_count++;
                                                                                                }





















/**Function to initiallize GPIO and set i2c bus and select mode of AFE*/
void i2c_initz(void)
{
  /**Set PINB0 pin as OUTPUT*/
  DDRB |= 0b00000001;
  /**Set PINC0 pin as OUTPUT*/
  DDRC |= 0b00000011;
  /**Set PINB0 pin in LOW state*/
  PORTB &= ~(1<<PINB0);
  /**Set PINC0 pin in HIGH state*/
  PORTC |= 1<<PINC0;
  PORTC |= 1<<PINC1;
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
   TU_putHex(0x01);  
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
   TU_putHex(0xFF);
}








/**Function to load default values from EEPROM to local array*/
void ldsett(void)
{
    TU_putHex(0x01);
    /**Load fault status higher 8 bits from EEPROM to local array*/
    afe_isr_reg[0] = eeprom_read_byte(0x00);
    /**Load fault status lower 8 bits from EEPROM to local array*/
    afe_isr_reg[1] = eeprom_read_byte(0x01);
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
    
  /**Load over temperature trip threshold*/
    afe_isr_reg[97] = eeprom_read_byte(0x1F);
    
  /**Load over temperature release*/
    afe_isr_reg[108] = eeprom_read_byte(0x20);
  /**Load over temperature trip times higher bits*/
    afe_isr_reg[81] = eeprom_read_byte(0x21);
  /**Load over temperature trip times lower bits*/
    afe_isr_reg[82] = eeprom_read_byte(0x22);
  /**Load under temperature trip threshold*/
    afe_isr_reg[98] = eeprom_read_byte(0x23);
  /**Load under temperature release*/
    afe_isr_reg[109] = eeprom_read_byte(0x24);
  /**Load under temperature trip times higher bits*/
    afe_isr_reg[83] = eeprom_read_byte(0x25);
  /**Load under teperature trip times lower bits*/
    afe_isr_reg[84] = eeprom_read_byte(0x26);
  /**Load short circuit in discharge trip*/
    afe_isr_reg[138] = eeprom_read_byte(0x27);
  /**Load short circuit times high bits*/
    afe_isr_reg[139] = eeprom_read_byte(0x28);
  /**Load short circuit times lower bits*/
    afe_isr_reg[140] = eeprom_read_byte(0x29);
    
  /**Load overcurrent in charging trip higher bits*/
    afe_isr_reg[93] = eeprom_read_byte(0x2A);
    
  /**Load overcurrent in charging trip lower bits*/
    afe_isr_reg[94] = eeprom_read_byte(0x2B);
    
  /**Load overcurrent in discharging trip higher bits*/
    afe_isr_reg[95] = eeprom_read_byte(0x2C);
    
  /**Load overcurrent in discharging tip lower bits*/
    afe_isr_reg[96] = eeprom_read_byte(0x2D);
    
  /**Load */
  //afe_isr_reg[] = eeprom_read_byte(0x2E);
  
  afe_isr_reg[107] = eeprom_read_byte(0x2F);
  
  //afe_isr_reg[] = eeprom_read_byte(0x30);
  
  /*Load current callibration factor high bits*/
    afe_isr_reg[6] = eeprom_read_byte(0x31);
    
  /*Load current callibartion factor lower bits*/
    afe_isr_reg[7] = eeprom_read_byte(0x32);
    
  //afe_isr_reg[] = eeprom_read_byte(0x33);
  
  afe_isr_reg[121] = eeprom_read_byte(0x34);
  
  afe_isr_reg[122] = eeprom_read_byte(0x35);
  
  afe_isr_reg[123] = eeprom_read_byte(0x36);
  
  afe_isr_reg[124] = eeprom_read_byte(0x37);
  
  //afe_isr_reg[] = eeprom_read_byte(0x38);
  //afe_isr_reg[] = eeprom_read_byte(0x39);
  //afe_isr_reg[] = eeprom_read_byte(0x3A);
  //afe_isr_reg[] = eeprom_read_byte(0x3B);
  //afe_isr_reg[] = eeprom_read_byte(0x3B);
  //afe_isr_reg[] = eeprom_read_byte(0x3B);
  //afe_isr_reg[] = eeprom_read_byte(0x3B);
  //afe_isr_reg[] = eeprom_read_byte(0x3B);
  
  /**Load value of cell bal 1*/
    afe_isr_reg[134] = eeprom_read_byte(0x40);
    
  /**Load value of cell bal 2*/
    afe_isr_reg[135] = eeprom_read_byte(0x41);
    
  /**Load value of cell bal 3*/
    afe_isr_reg[136] = eeprom_read_byte(0x42);

  /**Load value of recharge showup %cetage*/
    afe_isr_reg[35] = eeprom_read_byte(0x43);

     afe_isr_reg[87] = eeprom_read_byte(0x44);

      afe_isr_reg[88] = eeprom_read_byte(0x45);

       afe_isr_reg[85] = eeprom_read_byte(0x46);

        afe_isr_reg[86] = eeprom_read_byte(0x47);
    
    TU_putHex(0xFF);
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
      
      TU_putHex(0x00);
      _delay_ms(1000);
      return;
    }

    i2c_write(VC1_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg_reg[0] = i2c_read(true);*/


/* if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putHex(0x00);
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
      
      TU_putHex(0x00);
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
      
      TU_putHex(0x00);
      _delay_ms(1000);
      return;
    }
    /**Read system status*/
    i2c_write(SYS_STAT);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[127] = i2c_read(true);
    i2c_stop();

  if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putHex(0x00);
      _delay_ms(1000);
      return;
    }

    i2c_write(VP_HI);   //VP_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[2] = i2c_read(true);
    i2c_write(VP_HI);   //VP_LO
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[3] = i2c_read(true);
    i2c_stop();
    
    /**Reading fresh current measurement*/
    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putHex(0x00);
      _delay_ms(1000);
      return;
    }

    i2c_write(CC_HI);      //CC_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[4] = i2c_read(true);
    i2c_write(CC_HI);       //CC_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[5] = i2c_read(true);     
    i2c_stop();
    
    ccbf(0x0000);

    afe_isr_reg[8] = calsoc(merge_16(afe_isr_reg[2], afe_isr_reg[3]));      //calculated soc

    afe_isr_reg[9] = keystate(afe_isr_reg[8]);        //show recharge on 10% SOC
    
    /** Store and map mosfet status from AFE*/
    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putHex(0x00);
      _delay_ms(1000);
      return;
    }

    i2c_write(SYS_CTRL2);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    mos = i2c_read(true);
    afe_isr_reg[10] = (mos&0x03);
    i2c_stop();

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putHex(0x00);
      _delay_ms(1000);
      return;
    }

    i2c_write(TS2_HI);  
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    temph1 = i2c_read(true);
    i2c_write(TS2_HI);
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    templ1 = i2c_read(true);
    i2c_stop();

    tempmax1 = tempcal(temph1, templ1);
    afe_isr_reg[11] = tempmax1;
    //TU_putHex(tempmax1);

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putHex(0x00);
      _delay_ms(1000);
      return;
    }

    i2c_write(TS3_HI);   //VP_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    temph2 = i2c_read(true);
    i2c_write(TS3_HI);   //VP_LO
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    templ2 = i2c_read(true);
    i2c_stop();

    tempmax2 = tempcal(temph2, templ2);
    afe_isr_reg[12] = tempmax2;

    if(tempmax2 > tempmax1)
    {
      afe_isr_reg[137] = tempmax2;
      afe_isr_reg[141] = tempmax1;
    }

    else
    {
      afe_isr_reg[137] = tempmax1;
      afe_isr_reg[141] = tempmax2;
    }  
}

void bsbi(void)
{
  afe_isr_reg[121] = 0x01;
  afe_isr_reg[122] = 0x73;

  afe_isr_reg[123] = 0x05;
  afe_isr_reg[124] = 0xFC;
  
  afe_isr_reg[13] = 0x28;
  afe_isr_reg[14] = 0x23;
  afe_isr_reg[15] = afe_isr_reg[8];
  afe_isr_reg[16] = 0x01;
  afe_isr_reg[17] = 0x24;
  afe_isr_reg[18] = 0x39;
  afe_isr_reg[19] = 0x19;
  afe_isr_reg[20] = 0x4F;
  afe_isr_reg[21] = 0x80;
  afe_isr_reg[22] = 0x4F;
  afe_isr_reg[23] = 0x20;
  afe_isr_reg[24] = 0x4F;
  afe_isr_reg[25] = 0x2D;
  afe_isr_reg[26] = 0x05;
  afe_isr_reg[27] = 0x80;
  afe_isr_reg[28] = 0x8F;
  afe_isr_reg[29] = 0xFF;
  afe_isr_reg[30] = 0x8F;
  afe_isr_reg[31] = 0xFF;
  afe_isr_reg[32] = 0x80;
  afe_isr_reg[33] = 0x00;
  afe_isr_reg[34] = 0x10;
}







void bscv(void)
{
  afe_isr_reg[37] = 0x00;
  afe_isr_reg[38] = 0x00;

  if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putHex(0x00);
      _delay_ms(1000);
      return;
    }

    i2c_write(VC1_HI);   //VP_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[39] = i2c_read(true);
    i2c_write(VC1_HI);   //VP_LO
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[40] = i2c_read(true);
    i2c_stop();

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putHex(0x00);
      _delay_ms(1000);
      return;
    }

    i2c_write(VC2_HI);   //VP_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[41] = i2c_read(true);
    i2c_write(VC2_HI);   //VP_LO
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[42] = i2c_read(true);
    i2c_stop();


    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putHex(0x00);
      _delay_ms(1000);
      return;
    }

    i2c_write(VC3_HI);   //VP_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[43] = i2c_read(true);
    i2c_write(VC3_HI);   //VP_LO
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[44] = i2c_read(true);
    i2c_stop();

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putHex(0x00);
      _delay_ms(1000);
      return;
    }

    i2c_write(VC4_HI);   //VP_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[45] = i2c_read(true);
    i2c_write(VC4_HI);   //VP_LO
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[46] = i2c_read(true);
    i2c_stop();


    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putHex(0x00);
      _delay_ms(1000);
      return;
    }

    i2c_write(VC5_HI);   //VP_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[47] = i2c_read(true);
    i2c_write(VC5_HI);   //VP_LO
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[48] = i2c_read(true);
    i2c_stop();

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putHex(0x00);
      _delay_ms(1000);
      return;
    }

    i2c_write(VC6_HI);   //VP_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[49] = i2c_read(true);
    i2c_write(VC6_HI);   //VP_LO
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[50] = i2c_read(true);
    i2c_stop();


    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putHex(0x00);
      _delay_ms(1000);
      return;
    }

    i2c_write(VC7_HI);   //VP_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[51] = i2c_read(true);
    i2c_write(VC7_HI);   //VP_LO
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[52] = i2c_read(true);
    i2c_stop();

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putHex(0x00);
      _delay_ms(1000);
      return;
    }

    i2c_write(VC8_HI);   //VP_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[53] = i2c_read(true);
    i2c_write(VC8_HI);   //VP_LO
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[54] = i2c_read(true);
    i2c_stop();


    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putHex(0x00);
      _delay_ms(1000);
      return;
    }

    i2c_write(VC9_HI);   //VP_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[55] = i2c_read(true);
    i2c_write(VC9_HI);   //VP_LO
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[56] = i2c_read(true);
    i2c_stop();


    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putHex(0x00);
      _delay_ms(1000);
      return;
    }

    i2c_write(VC10_HI);   //VP_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[57] = i2c_read(true);
    i2c_write(VC10_HI);   //VP_LO
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[58] = i2c_read(true);
    i2c_stop();


    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putHex(0x00);
      _delay_ms(1000);
      return;
    }

    i2c_write(VC11_HI);   //VP_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[59] = i2c_read(true);
    i2c_write(VC11_HI);   //VP_LO
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[60] = i2c_read(true);
    i2c_stop();

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putHex(0x00);
      _delay_ms(1000);
      return;
    }

    i2c_write(VC12_HI);   //VP_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[61] = i2c_read(true);
    i2c_write(VC12_HI);   //VP_LO
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[62] = i2c_read(true);
    i2c_stop();

    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      TU_putHex(0x00);
      _delay_ms(1000);
      return;
    }

    i2c_write(VC13_HI);   //VP_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[63] = i2c_read(true);
    i2c_write(VC13_HI);   //VP_LO
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[64] = i2c_read(true);
    i2c_stop();


    if (!i2c_start_wait((I2C_7BITADDR<<1)|I2C_WRITE))
    {
      
      _delay_ms(1000);
      return;
    }

    i2c_write(VC15_HI);   //VP_HI
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[65] = i2c_read(true);
    i2c_write(VC15_HI);   //VP_LO
    i2c_rep_start((I2C_7BITADDR<<1)|I2C_READ);
    afe_isr_reg[66] = i2c_read(true);
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
      TU_send(':');
      TU_putHex(afe_isr_reg[137]);
      //TU_putHex(afe_isr_reg[127]);
      TU_send('\r');
      TU_send('\n');
    }
    
    else
    TU_send(':');
  }



  /*for(volatile int g=0; g<144; g++)
  {
    TU_puts("0x");
    TU_putHex(afe_isr_reg[g]);
    TU_send(',');
  }*/

  
  sei();
  i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0xFF);
}











void bsbi_log(void)
{ 

  for(volatile int q=121; q<125; q++)
  {
    TU_putHex(afe_isr_reg[q]);
    TU_send(':');
  }
  TU_putHex(afe_isr_reg[6]);
    TU_send(':');
    TU_putHex(afe_isr_reg[7]);
    TU_send(':');
  for(volatile int g=11; g<35; g++)
  {
    TU_putHex(afe_isr_reg[g]);
    if(g > 33)
    {
      TU_send('\r');
      TU_send('\n');
    }
    
    else
    TU_send(':');
  }
}


void bscv_log(void)
{
  cli();

  TU_putHex(afe_isr_reg[121]);
  TU_send(':');
  TU_putHex(afe_isr_reg[122]);
  TU_send(':');

  for(volatile int g=39; g<67; g++)
  {
    TU_putHex(afe_isr_reg[g]);
    if(g > 65)
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

  for(volatile int g=67; g<85; g++)
  {
    TU_putHex(afe_isr_reg[g]);
    if(g > 83)
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

void configwr(void)
{
  uint8_t num = 0; // For reading the Sent Digit
  uint8_t i;       // For Counter
  // Exactly 2 Bytes additional
  if(comm_cmd_sz != 0)
  {
    if (comm_ptr_next[0] == ':')
    {
      cli();
      if(comm_ptr_next[1] == 'w')
      {
        /*
         * FAQ    : where does comm_ptr_next[2] goes ?
         * Answer : comm_ptr_next[2] stores the ':'
         * 
         * FAQ    : What does comm_ptr_next[3] and comm_ptr_next[4] stores ?
         * Answer : comm_ptr_next[3] and comm_ptr_next[4] stores 1 byte place holder of configuration to be altered.
         */
        h = str2hex(comm_ptr_next[3]);
        l = str2hex(comm_ptr_next[4]);
        h = (h<<4) | (l & 0xff);
        switch(h)
        {
          /*
           * FAQ    : Where does comm_ptr_next[5] goes ?
           * Answer : comm_ptr_next[5] stores ':'
           * 
           * 0x06 is for current callibration.
           */
          case 0x06 :
          h=l=0;
          h = (str2hex(comm_ptr_next[6])<<4) | (str2hex(comm_ptr_next[7]) & 0xff);
          l = (str2hex(comm_ptr_next[8])<<4) | (str2hex(comm_ptr_next[9]) & 0xff);
          
          afe_isr_reg[6] = h;
          afe_isr_reg[7] = l;

          eeprom_write_byte(0x31, afe_isr_reg[6]);
          eeprom_write_byte(0x32, afe_isr_reg[7]);
          TU_putHex(0xFF);
          break;

          /*
           * 0x23 is for recharge %centage icon
           */
          case 0x23 :
          h=0;
          h = (str2hex(comm_ptr_next[6])<<4) | (str2hex(comm_ptr_next[7]) & 0xff);
          afe_isr_reg[35] = h;
          
          eeprom_write_byte(0x43, afe_isr_reg[35]);
          TU_putHex(0xFF);
          break;

          /*
           * 0x79 is for cell voltage callibration.
           */
          case 0x79 :
          h=l=0;
          h = (str2hex(comm_ptr_next[6])<<4) | (str2hex(comm_ptr_next[7]) & 0xff);
          l = (str2hex(comm_ptr_next[8])<<4) | (str2hex(comm_ptr_next[9]) & 0xff);

          afe_isr_reg[121] = h;
          afe_isr_reg[122] = l;

          eeprom_write_byte(0x34, afe_isr_reg[121]);
          eeprom_write_byte(0x35, afe_isr_reg[122]);
          TU_putHex(0xFF);
          break;

          /*
           * 0x7B is for pack voltage callibration
           */
          case 0x7B :
          h=l=0;
          h = (str2hex(comm_ptr_next[6])<<4) | (str2hex(comm_ptr_next[7]) & 0xff);
          l = (str2hex(comm_ptr_next[8])<<4) | (str2hex(comm_ptr_next[9]) & 0xff);
          
          afe_isr_reg[123] = h;
          afe_isr_reg[124] = l;

          eeprom_write_byte(0x36, afe_isr_reg[123]);
          eeprom_write_byte(0x37, afe_isr_reg[124]);
          TU_putHex(0xFF);
          break;

          /*
           * 0x0D is for Nominal capacity
           */
          case 0x0D :
          h=l=0;
          h = (str2hex(comm_ptr_next[6])<<4) | (str2hex(comm_ptr_next[7]) & 0xff);

          afe_isr_reg[13] = h;

          eeprom_write_byte(0x37, afe_isr_reg[13]);
          TU_putHex(0xFF);
          break;

         /*
          * 0x55 is for configuring cell under voltage threshold
          */
          case 0x55 :
         
          h=l=0;
          h = (str2hex(comm_ptr_next[6])<<4) | (str2hex(comm_ptr_next[7]) & 0xff);
          l = (str2hex(comm_ptr_next[8])<<4) | (str2hex(comm_ptr_next[9]) & 0xff);

          if(merge_16(h,l) < merge_16(afe_isr_reg[99], afe_isr_reg[100]))
          {
            afe_isr_reg[85] = h;
            afe_isr_reg[86] = l;
            eeprom_write_byte(0x46, afe_isr_reg[85]);
            eeprom_write_byte(0x47, afe_isr_reg[86]);
            TU_putHex(0xFF);
            break;
          }

          else
          {
            TU_putHex(0x00);
            break;
          }

          
          /*
           * 0x57 is for configuring cell over voltage trip threshold
           */
          case 0x57 :
          h=l=0;
          h = (str2hex(comm_ptr_next[6])<<4) | (str2hex(comm_ptr_next[7]) & 0xff);
          l = (str2hex(comm_ptr_next[8])<<4) | (str2hex(comm_ptr_next[9]) & 0xff);

          if(merge_16(h, l) > merge_16(afe_isr_reg[101], afe_isr_reg[102]))
          {
            afe_isr_reg[87] = h;
            afe_isr_reg[88] = l;
            eeprom_write_byte(0x44, afe_isr_reg[87]);
            eeprom_write_byte(0x45, afe_isr_reg[88]);
            TU_putHex(0xFF);
            break;
          }

          else
          {
            TU_putHex(0x00);
            break;
          }
          

          /*
           * 0x59 is for configuring pack over voltage trip threshold
           */
          case 0x59 :
          h=l=0;
          h = (str2hex(comm_ptr_next[6])<<4) | (str2hex(comm_ptr_next[7]) & 0xff);
          l = (str2hex(comm_ptr_next[8])<<4) | (str2hex(comm_ptr_next[9]) & 0xff);

          if(merge_16(h,l) > merge_16(afe_isr_reg[103], afe_isr_reg[104]))
          {
            afe_isr_reg[89] = h;
            afe_isr_reg[90] = l;
            eeprom_write_byte(0x13, afe_isr_reg[89]);
            eeprom_write_byte(0x14, afe_isr_reg[90]);
            TU_putHex(0xFF);
            break;
          }

          else
          {
            TU_putHex(0x00);
            break;
          }


          /*
           * 0x5B is for configuring pack under voltage trip threshold
           */
          case 0x5B :
          h=l=0;
          h = (str2hex(comm_ptr_next[6])<<4) | (str2hex(comm_ptr_next[7]) & 0xff);
          l = (str2hex(comm_ptr_next[8])<<4) | (str2hex(comm_ptr_next[9]) & 0xff);

          if(merge_16(h,l) < merge_16(afe_isr_reg[105], afe_isr_reg[106]))
          {
            afe_isr_reg[91] = h;
            afe_isr_reg[92] = l;
            eeprom_write_byte(0x19, afe_isr_reg[91]);
            eeprom_write_byte(0x1A, afe_isr_reg[92]);
            TU_putHex(0xFF);
            break;
          }

          else
          {
            TU_putHex(0x00);
            break;
          }


          /*
           * 0x5D is for configuring pack over current in charging trip threshold
           */
          case 0x5D :
          h=l=0;
          h = (str2hex(comm_ptr_next[6])<<4) | (str2hex(comm_ptr_next[7]) & 0xff);
          l = (str2hex(comm_ptr_next[8])<<4) | (str2hex(comm_ptr_next[9]) & 0xff);

          afe_isr_reg[93] = h;
          afe_isr_reg[94] = l;

          eeprom_write_byte(0x2A, afe_isr_reg[93]);
          eeprom_write_byte(0x2B, afe_isr_reg[94]);
          TU_putHex(0xFF);
          break;


          /*
           * 0x5F is for configuring pack over current in discharging trip threshold
           */
          case 0x5F :
          h=l=0;
          h = (str2hex(comm_ptr_next[6])<<4) | (str2hex(comm_ptr_next[7]) & 0xff);
          l = (str2hex(comm_ptr_next[8])<<4) | (str2hex(comm_ptr_next[9]) & 0xff);

          afe_isr_reg[95] = h;
          afe_isr_reg[96] = l;
          eeprom_write_byte(0x2C, afe_isr_reg[95]);
          eeprom_write_byte(0x2D, afe_isr_reg[96]);
          TU_putHex(0xFF);
          break;
          
          
       

        /*
           * 0x61 is for configuring pack over temperature trip threshold
           */
          case 0x61 :
          h=0;
          h = (str2hex(comm_ptr_next[6])<<4) | (str2hex(comm_ptr_next[7]) & 0xff);
          


          if(h > afe_isr_reg[108])
          {
            afe_isr_reg[97] = h;
            eeprom_write_byte(0x1F, afe_isr_reg[97]);
            TU_putHex(0xFF);
            break;
          }

          else
          {
            TU_putHex(0x00);
            break;
          }
          


           /*
           * 0x61 is for configuring pack under temperature trip threshold
           */
          case 0x62 :
          h=0;
          h = (str2hex(comm_ptr_next[6])<<4) | (str2hex(comm_ptr_next[7]) & 0xff);

          if(h < afe_isr_reg[109])
          {
            afe_isr_reg[98] = h;
            eeprom_write_byte(0x23, afe_isr_reg[98]);
            TU_putHex(0xFF);
            break;
          }

          else
          {
            TU_putHex(0x00);
            break;
          }

          /*
          * 0x63 is for configuring cell under voltage release threshold
          */
          case 0x63 :
          h=l=0;
          h = (str2hex(comm_ptr_next[6])<<4) | (str2hex(comm_ptr_next[7]) & 0xff);
          l = (str2hex(comm_ptr_next[8])<<4) | (str2hex(comm_ptr_next[9]) & 0xff);

          if(merge_16(h,l) > merge_16(afe_isr_reg[85], afe_isr_reg[86]))
          {
            afe_isr_reg[99] = h;
            afe_isr_reg[100] = l;
            eeprom_write_byte(0x0F, afe_isr_reg[99]);
            eeprom_write_byte(0x10, afe_isr_reg[100]);
            TU_putHex(0xFF);
            break;
          }

          else
          {
            TU_putHex(0x00);
            break;
          }
          

          /*
          * 0x65 is for configuring cell over voltage release threshold
          */
          case 0x65 :
          h=l=0;
          h = (str2hex(comm_ptr_next[6])<<4) | (str2hex(comm_ptr_next[7]) & 0xff);
          l = (str2hex(comm_ptr_next[8])<<4) | (str2hex(comm_ptr_next[9]) & 0xff);

          if(merge_16(h, l) < merge_16(afe_isr_reg[87], afe_isr_reg[88]))
          {
            afe_isr_reg[101] = h;
            afe_isr_reg[102] = l;
            eeprom_write_byte(0x0A, afe_isr_reg[101]);
            eeprom_write_byte(0x0B, afe_isr_reg[102]);
            TU_putHex(0xFF);
            break;
          }

          else
          {
            TU_putHex(0x00);
            break;
          }


          /*
          * 0x67 is for configuring pack over voltage release threshold
          */
          case 0x67 :
          h=l=0;
          h = (str2hex(comm_ptr_next[6])<<4) | (str2hex(comm_ptr_next[7]) & 0xff);
          l = (str2hex(comm_ptr_next[8])<<4) | (str2hex(comm_ptr_next[9]) & 0xff);

          if(merge_16(h, l) < merge_16(afe_isr_reg[89], afe_isr_reg[90]))
          {
            afe_isr_reg[103] = h;
            afe_isr_reg[104] = l;
            eeprom_write_byte(0x15, afe_isr_reg[103]);
            eeprom_write_byte(0x16, afe_isr_reg[104]);
            TU_putHex(0xFF);
            break;
          }

          else
          {
            TU_putHex(0x00);
            break;
          }


          /*
          * 0x69 is for configuring pack under voltage release threshold
          */
          case 0x69 :
          h=l=0;
          h = (str2hex(comm_ptr_next[6])<<4) | (str2hex(comm_ptr_next[7]) & 0xff);
          l = (str2hex(comm_ptr_next[8])<<4) | (str2hex(comm_ptr_next[9]) & 0xff);

          if(merge_16(h, l) > merge_16(afe_isr_reg[91], afe_isr_reg[92]))
          {
            afe_isr_reg[105] = h;
            afe_isr_reg[106] = l;
            eeprom_write_byte(0x1B, afe_isr_reg[105]);
            eeprom_write_byte(0x1C, afe_isr_reg[106]);
            TU_putHex(0xFF);
            break;
          }

          else
          {
            TU_putHex(0x00);
            break;
          }


          







          /*
           * 0x6C is for configuring pack over temperature release threshold
           */
          case 0x6C :
          h=0;
          h = (str2hex(comm_ptr_next[6])<<4) | (str2hex(comm_ptr_next[7]) & 0xff);

          if(h < afe_isr_reg[97])
          {
            afe_isr_reg[108] = h;
            eeprom_write_byte(0x20, afe_isr_reg[108]);
            TU_putHex(0xFF);
            break;
          }

          else
          {
            TU_putHex(0x00);
            break;
          }
          


           /*
           * 0x6D is for configuring pack under temperature release threshold
           */
          case 0x6D :
          h=0;
          h = (str2hex(comm_ptr_next[6])<<4) | (str2hex(comm_ptr_next[7]) & 0xff);

          if(h > afe_isr_reg[98])
          {
            afe_isr_reg[109] = h;
            eeprom_write_byte(0x24, afe_isr_reg[109]);
            TU_putHex(0xFF);
            break;
          }

          else
          {
            TU_putHex(0x00);
            break;
          }

          case 0x6B : 
          h=0;
          h = (str2hex(comm_ptr_next[6])<<4) | (str2hex(comm_ptr_next[7]) & 0xff);

          afe_isr_reg[107] = h;
          eeprom_write_byte(0x2F, afe_isr_reg[107]);
          TU_putHex(0xFF);
          break;
       }
      }

























      if(comm_ptr_next[1] == 'r')
      {
        /*
         * FAQ    : where does comm_ptr_next[2] goes ?
         * Answer : comm_ptr_next[2] stores the ':'
         * 
         * FAQ    : What does comm_ptr_next[3] and comm_ptr_next[4] stores ?
         * Answer : comm_ptr_next[3] and comm_ptr_next[4] stores 1 byte place holder of configuration to be altered.
         */
        h=l=0;
        h = str2hex(comm_ptr_next[3]);
        l = str2hex(comm_ptr_next[4]);
        h = (h<<4) | (l & 0xff);
        switch(h)
        {
          /*
           * FAQ    : Where does comm_ptr_next[5] goes ?
           * Answer : comm_ptr_next[5] stores ':'
           * 
           * 0x06 is for current callibration.
           */
          case 0x06 :
          TU_putHexW(merge_16(afe_isr_reg[6], afe_isr_reg[7]));
          break;

          /*
           * 0x23 is for recharge %centage icon
           */
          case 0x23 :
          TU_putHex(afe_isr_reg[35]);
          break;

          /*
           * 0x79 is for cell voltage callibration.
           */
          case 0x79 :
          TU_putHexW(merge_16(afe_isr_reg[121], afe_isr_reg[122]));
          break;

          /*
           * 0x7B is for pack voltage callibration
           */
          case 0x7B :
          TU_putHexW(merge_16(afe_isr_reg[123], afe_isr_reg[124]));
          break;

          /*
           * 0x0D is for Nominal capacity
           */
          case 0x0D :
          TU_putHex(afe_isr_reg[13]);
          break;

         /*
          * 0x55 is for configuring cell under voltage threshold
          */
          case 0x55 :
          
            TU_putHexW(merge_16(afe_isr_reg[85], afe_isr_reg[86]));
            break;

          
          /*
           * 0x57 is for configuring cell over voltage trip threshold
           */
          case 0x57 :
          TU_putHexW(merge_16(afe_isr_reg[87], afe_isr_reg[88]));
          break;

          /*
           * 0x59 is for configuring pack over voltage trip threshold
           */
          case 0x59 :
          TU_putHexW(merge_16(afe_isr_reg[89], afe_isr_reg[90]));
          break;


          /*
           * 0x5B is for configuring pack under voltage trip threshold
           */
          case 0x5B :
          TU_putHexW(merge_16(afe_isr_reg[91], afe_isr_reg[92]));
          break;


          /*
           * 0x5D is for configuring pack over current in charging trip threshold
           */
          case 0x5D :
          TU_putHexW(merge_16(afe_isr_reg[93], afe_isr_reg[94]));
          break;


          /*
           * 0x5F is for configuring pack over current in discharging trip threshold
           */
          case 0x5F :
          TU_putHexW(merge_16(afe_isr_reg[95], afe_isr_reg[96]));
          break;
          
          
       

        /*
           * 0x61 is for configuring pack over temperature trip threshold
           */
          case 0x61 :
          TU_putHex(afe_isr_reg[97]);
          break;
          


           /*
           * 0x61 is for configuring pack under temperature trip threshold
           */
          case 0x62 :
          TU_putHex(afe_isr_reg[98]);
          break;

          /*
          * 0x63 is for configuring cell under voltage release threshold
          */
          case 0x63 :
          TU_putHexW(merge_16(afe_isr_reg[99], afe_isr_reg[100]));
          break;
          

          /*
          * 0x65 is for configuring cell over voltage release threshold
          */
          case 0x65 :
          TU_putHexW(merge_16(afe_isr_reg[101], afe_isr_reg[102]));
          break;

          /*
          * 0x67 is for configuring pack over voltage release threshold
          */
          case 0x67 :
          TU_putHexW(merge_16(afe_isr_reg[103], afe_isr_reg[104]));
          break;
          /*
          * 0x69 is for configuring pack under voltage release threshold
          */
          case 0x69 :
          TU_putHexW(merge_16(afe_isr_reg[105], afe_isr_reg[106]));
          break;
          /*
           * 0x6C is for configuring pack over temperature release threshold
           */
          case 0x6C :
          TU_putHex(afe_isr_reg[108]);
          break;
           /*
           * 0x6D is for configuring pack under temperature release threshold
           */
          case 0x6D :
          TU_putHex(afe_isr_reg[109]);
          break;

          case 0x6B : 
          TU_putHex(afe_isr_reg[107]);
          break;
       }
      }

      
     }
      
      sei();
      i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0x80);
      return;
    }
    
    TU_putHex(0x00);          
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
    /*if(dt)
      TU_send(dt);*/
    
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
      configwr();
      break;
    }

    if(serial_cmd == COMM_CMD_KEY)
    {
      keyin();
      break;
    }

    if(serial_cmd == COMM_CMD_RHG)
    {
      recharge();
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
      
      if(comm_sz > SZ_CMD_CONFIG)
      {
        ret = COMM_CMD_CONFIG;
        // Update the End Pointer
        comm_cmd_sz = SZ_CMD_CONFIG;
        comm_ptr_next = (uint8_t *)&comm_buffer[SZ_CMD_CONFIG];
      }
      break;
    }

     //Check for BSCV Command
    if(strncmp(cmd_key, comm_buffer, SZ_CMD_KEY) == 0)
    {
      
      ret = COMM_CMD_KEY;
      break;
    }

     //Check for BSCV Command
    if(strncmp(cmd_rhg, comm_buffer, SZ_CMD_RHG) == 0)
    {
      
      ret = COMM_CMD_RHG;
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


uint8_t calsoc(uint16_t ct)
{
  uint16_t fsoc=0x0000;
  fsoc=ct;
 
 ct = map(ct, 0x72BD, 0x9156, 0x00, 0x64);
 
 if(fsoc < 0x72BD)
 ct=0x00;

 if(fsoc > 0x9156)
 ct = 0x64;
 
 return ct;
}



uint16_t merge_16(uint8_t h_16, uint8_t l_16)
{
  uint16_t ret_16_merge=0;
  ret_16_merge = (h_16 << 8) | (l_16 & 0xff);
  return ret_16_merge;
}







void trip(void)
{
  /**cell over voltage fault*/
  if(((afe_isr_reg[127]&0x04) || (cmax > merge_16(afe_isr_reg[87],afe_isr_reg[88]))) && (!(afe_isr_reg[0] & 0x80)) && (rhgtog = 0x00))
  {
  //i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x42); 
  afe_isr_reg[10] = 0x02; 
    afe_isr_reg[0] |= (1<<7); 
    eeprom_update_byte(0x00, afe_isr_reg[0]);
    if(afe_isr_reg[74] == 0xFF)
    {
      afe_isr_reg[73] += 1;
      afe_isr_reg[74] = 0x00;
       
    }
    else
    afe_isr_reg[74] += 1;
    eeprom_update_byte(0x0D, afe_isr_reg[74]);
    eeprom_update_byte(0x0C, afe_isr_reg[73]);
    TU_putln("cell overvoltage fault"); 
    keytog = 0x00;
  }

  /**cell under voltage fault*/
  if(((afe_isr_reg[127]&0x08) || (cmin < merge_16(afe_isr_reg[85],afe_isr_reg[86]))) && (!(afe_isr_reg[0] & 0x40)))
  {
    //i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x41);
    afe_isr_reg[10] = 0x01;
    afe_isr_reg[0] |= (1<<6); 
    eeprom_update_byte(0x00, afe_isr_reg[0]);

    if(afe_isr_reg[76] == 0xFF)
    {
      afe_isr_reg[75] += 1;
      afe_isr_reg[76] = 0x00;
    }
    else
    afe_isr_reg[76] += 1;
    eeprom_update_byte(0x12, afe_isr_reg[76]);
    eeprom_update_byte(0x11, afe_isr_reg[75]);
    if(verbo == 0x40)
    TU_putln("Cell undervoltage fault"); 
    keytog = 0x00;
  }
  
   /**pack overvoltage fault*/
    if((merge_16(afe_isr_reg[2],afe_isr_reg[3]) > merge_16(afe_isr_reg[89],afe_isr_reg[90])) && (!(afe_isr_reg[0] & 0x20)))
    {
      //i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x42);
      afe_isr_reg[10] = 0x02;
      keytog = 0x00;
      afe_isr_reg[0] |= (1 << 5); 
      eeprom_update_byte(0x00, afe_isr_reg[0]);

      if(afe_isr_reg[78] == 0xFF)
      {
        afe_isr_reg[77] += 1;
        afe_isr_reg[78] = 0x00;
      }
      else
      afe_isr_reg[78] += 1;
      eeprom_update_byte(0x12, afe_isr_reg[78]);
      eeprom_update_byte(0x11, afe_isr_reg[77]);
      TU_putln("pack overvoltage fault");
    }
  

  /**pack undervoltage fault*/
    if((merge_16(afe_isr_reg[2],afe_isr_reg[3]) < merge_16(afe_isr_reg[91],afe_isr_reg[92])) && (!(afe_isr_reg[0] & 0x10)))
    {
      //i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x41);
      afe_isr_reg[10] = 0x01;
      keytog = 0x00;
      afe_isr_reg[0] |= (1<<4); 
      eeprom_update_byte(0x00, afe_isr_reg[0]);

      if(afe_isr_reg[80] == 0xFF)
      {
        afe_isr_reg[79] += 1;
        afe_isr_reg[80] = 0x00;
      }
      else
      afe_isr_reg[80] += 1;
      eeprom_update_byte(0x12, afe_isr_reg[80]);
      eeprom_update_byte(0x11, afe_isr_reg[79]);
      TU_putln("pack undervoltage fault");
    }


 /**over current in charging fault*/
    if(merge_16(afe_isr_reg[4],afe_isr_reg[5]) > merge_16(afe_isr_reg[93],afe_isr_reg[94]) && !(afe_isr_reg[4] & 0x80))
    {
      //i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x42);
      afe_isr_reg[10] = 0x02;
      keytog = 0x00;
      afe_isr_reg[0] |= (1<<1); 
      eeprom_update_byte(0x00, afe_isr_reg[0]);

      if(afe_isr_reg[70] == 0xFF)
      {
        afe_isr_reg[69] += 1;
        afe_isr_reg[70] = 0x00;
      }
      else
      afe_isr_reg[70] += 1;
      eeprom_update_byte(0x12, afe_isr_reg[70]);
      eeprom_update_byte(0x11, afe_isr_reg[69]);
      TU_putln("pack over current during charging fault");
      ocscpreviousmilis = millis();
    }

  /**over current in discharging fault*/
  if((merge_16(afe_isr_reg[4],afe_isr_reg[5])& 0x8000) && !(afe_isr_reg[4] & 0x01))
  {
    if(merge_16(afe_isr_reg[4],afe_isr_reg[5]) < merge_16(afe_isr_reg[95],afe_isr_reg[96]))
      {
        //i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x41);
        afe_isr_reg[10] = 0x01;
        keytog = 0x00;
        afe_isr_reg[0] |= (1<<0); 
        eeprom_update_byte(0x00, afe_isr_reg[0]);
  
        if(afe_isr_reg[72] == 0xFF)
        {
          afe_isr_reg[71] += 1;
          afe_isr_reg[72] = 0x00;
        }
        else
        afe_isr_reg[72] += 1;
        eeprom_update_byte(0x12, afe_isr_reg[72]);
        eeprom_update_byte(0x11, afe_isr_reg[71]);
        TU_putln("pack over current during discharging fault");
        ocscpreviousmilis = millis();
      }
      
      if((rhgtog == 0x40) && (merge_16(afe_isr_reg[4],afe_isr_reg[5]) < merge_16(0xFF,0xF4)))
      {
        //i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x41);
        afe_isr_reg[10] = 0x01;
        rhgtog = 0x00;
      }
  }

  /**pack over temperature fault*/
  if((afe_isr_reg[137] > afe_isr_reg[97]) && (!(afe_isr_reg[0] & 0x08)))
  {
    //i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x40);
    afe_isr_reg[10] = 0x00;
    keytog = 0x00;
    afe_isr_reg[0] |= (1<<3);
    eeprom_update_byte(0x00, afe_isr_reg[0]);

    if(afe_isr_reg[72] == 0xFF)
      {
        afe_isr_reg[82] += 1;
        afe_isr_reg[81] = 0x00;
      }
      else
      afe_isr_reg[82] += 1;
      eeprom_update_byte(0x12, afe_isr_reg[82]);
      eeprom_update_byte(0x11, afe_isr_reg[81]);
      TU_putln("pack over temperature fault");
  }


  /**pack under temperature fault*/
  if((afe_isr_reg[141] < afe_isr_reg[98]) && (!(afe_isr_reg[0] & 0x04)))
  {
    //i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x40);
    afe_isr_reg[10] = 0x00;
    afe_isr_reg[10] = 0x00;
    keytog = 0x00;
    afe_isr_reg[0] |= (1<<2);
    eeprom_update_byte(0x00, afe_isr_reg[0]);

    if(afe_isr_reg[84] == 0xFF)
      {
        afe_isr_reg[84] += 1;
        afe_isr_reg[83] = 0x00;
      }
      else
      afe_isr_reg[82] += 1;
      eeprom_update_byte(0x12, afe_isr_reg[84]);
      eeprom_update_byte(0x11, afe_isr_reg[83]);
      TU_putln("pack under temperature fault");
  }
}











void rel(void)
{
  /**cell overvoltage release*/
  if((afe_isr_reg[0] & 0x80))
  {
    if(cmax < merge_16(afe_isr_reg[101],afe_isr_reg[102]))
    {
      afe_isr_reg[0] &= ~(1<<7);
      eeprom_update_byte(0x00, afe_isr_reg[0]);
      i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0x04);
      TU_putln("Cell overvoltage released"); 
      //i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x43);
      afe_isr_reg[10] = 0x03;
      keytog = 0x40;
    }
  }



  /**cell undervoltage release*/
  if(afe_isr_reg[0] & 0x40)
  { 
    if(cmin > merge_16(afe_isr_reg[99],afe_isr_reg[100]))
    {
      afe_isr_reg[0] &= ~(1<<6);
      eeprom_update_byte(0x00, afe_isr_reg[0]);
      i2c_WriteWithCRC(I2C_7BITADDR, SYS_STAT, 0x08); 
      TU_putln("cell under voltage released"); 
      //i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x43);
      afe_isr_reg[10] = 0x03;
      keytog = 0x40;
      rhgtog = 0x00;
      afe_isr_reg[9] = 0x0F;
    }
  }


  /**pack overvoltage release*/
  if((afe_isr_reg[0] & 0x20))
  {
    if(merge_16(afe_isr_reg[2],afe_isr_reg[3]) < merge_16(afe_isr_reg[103],afe_isr_reg[104]))
    {
      afe_isr_reg[0]   &= ~(1<<5); 
      eeprom_update_byte(0x00, afe_isr_reg[0]);
      TU_putln("pack overvoltage release");
      //i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x43);
      afe_isr_reg[10] = 0x03;
      keytog = 0x40;
    }
  }

  /**pack undervoltage release*/
  if((afe_isr_reg[0] & 0x10))
  {
    if(merge_16(afe_isr_reg[2],afe_isr_reg[3]) > merge_16(afe_isr_reg[105],afe_isr_reg[106]))
    {
      afe_isr_reg[0] &= ~(1<<4);
      eeprom_update_byte(0x00, afe_isr_reg[0]);
      TU_putln("pack undervoltage release");
      //i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x43);
      afe_isr_reg[10] = 0x03;
      keytog = 0x40;
      rhgtog = 0x00;
      afe_isr_reg[9] = 0x0F;
    }
  }

  /**pack over current in charging/discharging release*/

  /**
   * ocscerval = 60000;
   *ocscpreviousmilis = 0;
   */
 
  if((afe_isr_reg[0] & 0x02) || (afe_isr_reg[0] & 0x01))
  {
    if(millis() - ocscpreviousmilis >= ocscerval)
    {
      keyrhgpreviousmilis = millis();
      afe_isr_reg[0] &= ~(1<<0);
      afe_isr_reg[0] &= ~(1<<1);
      eeprom_update_byte(0x00, afe_isr_reg[0]);
      TU_putln("overcurrent in C/D released");
      //i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x43);
      afe_isr_reg[10] = 0x03;
      keytog = 0x40;
    }
  }

  /**Pack over temperature release*/
  if((afe_isr_reg[0] & 0x08))
  {
    if(afe_isr_reg[137] < afe_isr_reg[108])
    {
      afe_isr_reg[0] &= ~(1<<3);
      eeprom_update_byte(0x00, afe_isr_reg[0]);
      TU_putln("pack overtemperature released");
      //i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x43);
      afe_isr_reg[10] = 0x03;
      keytog = 0x40;
    }
  }


  /**Pack under temperature release*/
  if((afe_isr_reg[0] & 0x04))
  {
    if(afe_isr_reg[141] > afe_isr_reg[109])
    {
      afe_isr_reg[0] &= ~(1<<2);
      eeprom_update_byte(0x00, afe_isr_reg[0]);
      TU_putln("pack undertemperature released");
      //i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x43);
      afe_isr_reg[10] = 0x03;
      keytog = 0x40;
    }
  }
}











void cvmaxmin(void)
{
  cmax=0x0000;
  cmin=0xFFFF;
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
  uint32_t tempcalc=0;
  tempcalc = merge_16(tempeh, tempel);
  tempcalc = (tempcalc*0x17E);
  tempcalc = (tempcalc/0x3E8);
  tempcalc = ((0x2710*tempcalc)/(0xCE4 - tempcalc));
  for(int i=0; i<86; i++)
  {
    if(tempmap[i] < tempcalc)
    {
      return i;
    }
  }
}

void keyin(void)
{
  /*/
   * No faults should be there to make the drive on
   * soc should be greater then 15%(configurable)
   * (optional) on pressing key after drive on , switch backs off the drive
   */
  if(afe_isr_reg[0]+afe_isr_reg[1] == 0x00)
  {
    //i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x43);
    afe_isr_reg[10] = 0x03;
    keytog = 0x40;
    keyrhgpreviousmilis = millis();
  }
}


void recharge(void)
{
    if((afe_isr_reg[0] & 0x10) || (afe_isr_reg[0] & 0x40))
    {
      afe_isr_reg[0] &= ~(1<<4);
      eeprom_update_byte(0x00, afe_isr_reg[0]);
      //TU_putln("pack undervoltage release");
      //i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x43);
      afe_isr_reg[10] = 0x03;
      rhgtog = 0x40;
    }
}



uint8_t keystate(uint8_t ks)
{
  if((afe_isr_reg[0] & 0x10) || (afe_isr_reg[0] & 0x40))
  return 0x00;
  /**Show recharge*/

  else
  /**Show key*/
  return 0x0F;
}

/*
 * keyrhgerval = 60000;
 * keyrhgpreviousmilis = 0;
 */
void chk_key_act(void)
{
  if(millis() - keyrhgpreviousmilis >= keyrhgerval)
  { 
    if(merge_16(afe_isr_reg[4],afe_isr_reg[5]) < 0x0017)
    {
      //i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x40);
      afe_isr_reg[10] = 0x00;
      keytog = 0x00;
    }
    keyrhgpreviousmilis = millis();
  }
}




