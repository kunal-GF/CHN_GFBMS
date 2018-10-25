/** Maximum Buffer Size for Serial Port */
#define COMM_MAX_BUFFER_SZ 200
/** Communication Command processing Error */
#define COMM_CMD_ERROR        0
/** Communications is Still receiving data or waiting */
#define COMM_CMD_BUSY         1
/** Communications Command to Read ADC Instantaneously */
#define COMM_CMD_ADC          2


#define SZ_CMD_ADC 3

/*Global variables*/
volatile uint8_t serial_command;
/** Exposed size of the Communication Buffer */
extern uint8_t comm_sz;
/** Exposed Buffer for Serial data reception */
extern uint8_t comm_buffer[COMM_MAX_BUFFER_SZ];
/** Exposed pointer to next part of Command for further processing */
extern uint8_t* comm_ptr_next;
/** Exposed size of the Command Processed */
extern uint8_t comm_cmd_sz;

const char cmd_adc[SZ_CMD_ADC] = "adc";



/*Function definations*/
/* Communications Processing Loop */
uint8_t Comm_loop(void);
/*Function to process codes*/
uint8_t process(void);
/*Function to get length of recieved data*/
uint8_t TU_getln(uint8_t *array, uint8_t max_sz);
/* Send out a NULL terminated string of data with Line Terminator */
void TU_putln(uint8_t *array);

void setup() 
{
  
}

void loop() 
{
  // Command Loop
    serial_command = Comm_loop();

    // Process the Serial Commands & Do Business Logic
    BL_Process(serial_command);
}











/* Communications Processing Loop */
uint8_t Comm_loop(void)
{
  // Check if Data is available
  comm_sz = TU_getln(comm_buffer,COMM_MAX_BUFFER_SZ);

  //If some thing have been received
  if(comm_sz)
  {
    // Run the Processing Loop & return the Command
    return process();
  }

  // Nothing to Return - If the command was not there
  return COMM_CMD_BUSY;
}
/*-----------------------------------Function ends here-----------------------------------*/

/*Function to process codes*/
uint8_t process(void)
{
  uint8_t ret = COMM_CMD_ERROR; // Default Error in Processing
  comm_ptr_next = comm_buffer; // Default First Location
  comm_cmd_sz = 0;  // By Default there is no next command

  do{
    //Check for ADC Command
    if(strncmp(cmd_adc, comm_buffer, SZ_CMD_ADC) == 0)
    {
      ret = COMM_CMD_ADC;
      break;
    }
  }while(0);

  // Return the Final Command after processing
  return ret;
}
/*--------------------------------------Function ends here----------------------------------------------------*/


/* Business Logic Processing Loop */
uint8_t BL_Process(uint8_t serial_cmd)
{
  do{

    if(serial_cmd == COMM_CMD_ERROR)
    {
      TU_putln(" BL: Unknown Command Received ");
      break;
    }

    if(serial_cmd == COMM_CMD_ADC)
    {
      TU_putln(" BL: !! Reading all ADC channels !!");
      TU_puts(" BL: Vout= ");
      TU_putln(" mV");
      TU_puts(" BL: I   = ");
      TU_putln(" mA");
      TU_puts(" BL: Vin = ");
      TU_putln(" mV");
      TU_puts(" BL: Vrev = ");
      TU_putln(" mV");
      break;
    }
  }while(0);

  return 0;
}
/*---------------------------------------Funtion ends here----------------------------------------------------------*/

/* Command Processor with Line Ending '\r\n' termination */
uint8_t TU_getln(uint8_t *array, uint8_t max_sz)
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
          array[nCount] = 0; // Insert the NULL terminator
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
          array[nCount] = 0; // Insert the NULL terminator
        }
        break;
      case '\b': // Backspace received
        if(nCount) // We were already in middle of receiving some thing
        {
          array[nCount] = 0; // Clear the Last Received
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
            array[nCount++] = dt;
          }
          else // Buffer is Full need to Return
          {           
            array[nCount] = 0; // Insert the NULL terminator
            ret = nCount; // Return Array size
            nCount = 0; // Reset the sequence
          }
        }
        else // Initiate the Process of Reception
        {
          array[nCount++] = dt;// Start the Reception
        }
        break;
    }// End of the Selector Switch(dt)
    
    // Send back the Character (If not Null)
    if(dt)
      TU_send(dt);
    
  }// End of While TU_getc
  
  return ret;
}
/*---------------------------------------Function ends here---------------------------------------------------*/

/* Send out a NULL terminated string of data with Line Terminator */
void TU_putln(uint8_t *array)
{
  TU_puts(array);
  TU_puts("\r\n");
}
/*-----------------------------------------Function ends here--------------------------------------------------*/

/* Sends out a NULL terminated string of data */
void TU_puts(uint8_t *array)
{
  uint8_t *p = array;
  while(*p)
  {
    while(TX_READY() == RESET);
    UART2_SendData8(*p);
    ++p;
  }
}
/*---------------------------------------------Function ends here----------------------------------------------*/
