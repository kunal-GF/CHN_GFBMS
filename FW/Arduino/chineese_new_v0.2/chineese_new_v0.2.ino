/*Global variables*/
volatile uint8_t serial_command;

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
/*-----------------------------------Function ends-----------------------------------*/

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

    // Check for Status
    if(strncmp(cmd_status, comm_buffer, SZ_CMD_STATUS) == 0)
    {
      ret = COMM_CMD_STATUS;
      break;
    }

    // Check for MOSFET Check command
    if(strncmp(cmd_check_mosfet, comm_buffer, SZ_CMD_CHECK_MOSFET) == 0)
    {
      ret = COMM_CMD_CHECK_MOSFET;
      break;
    }    

    // Check for Read Faults Command
    if(strncmp(cmd_read_faults, comm_buffer, SZ_CMD_READ_FAULTS) == 0)
    {
      ret = COMM_CMD_READ_FAULTS;
      break;
    }

    // Check for Clear Faults Command
    if(strncmp(cmd_clear_faults, comm_buffer, SZ_CMD_CLEAR_FAULTS) == 0)
    {
      if(comm_sz > SZ_CMD_CLEAR_FAULTS)
      {
        ret = COMM_CMD_CLEAR_FAULTS;
        // Update the End Pointer
        comm_cmd_sz = SZ_CMD_CLEAR_FAULTS;
        comm_ptr_next = (uint8_t *)&comm_buffer[SZ_CMD_CLEAR_FAULTS];
      }
      break;
    }

    // Check for Recharge Command
    if(strncmp(cmd_recharge, comm_buffer, SZ_CMD_RECHARGE) == 0)
    {
      if(comm_sz > SZ_CMD_RECHARGE)
      {
        ret = COMM_CMD_RECHARGE;
        // Update the End Pointer
        comm_cmd_sz = SZ_CMD_RECHARGE;
        comm_ptr_next = (uint8_t *)&comm_buffer[SZ_CMD_RECHARGE];
      }
      break;
    }

    // Check for Unlock Command
    if(strncmp(cmd_unlock, comm_buffer, SZ_CMD_UNLOCK) == 0)
    {
      if(comm_sz > SZ_CMD_UNLOCK)
      {
        ret = COMM_CMD_UNLOCK;
        // Update the End Pointer
        comm_cmd_sz = SZ_CMD_UNLOCK;
        comm_ptr_next = (uint8_t *)&comm_buffer[SZ_CMD_UNLOCK];
      }
      break;
    }

    // Check for System ID Config Command
    if(strncmp(cmd_systemid, comm_buffer, SZ_CMD_SYSTEMID) == 0)
    {
      ret = COMM_CMD_SETSYSTEMID; // In any case return the command
      if(comm_sz > SZ_CMD_SYSTEMID)
      {        
        // Update the End Pointer
        comm_cmd_sz = SZ_CMD_SYSTEMID;
        comm_ptr_next = (uint8_t *)&comm_buffer[SZ_CMD_SYSTEMID];
      }
      break;
    }

    // Check for Parameter set 1
    if(strncmp(cmd_setparam1, comm_buffer, SZ_CMD_SETPARAM1) == 0)
    {
      ret = COMM_CMD_SETPARAM1; // In any case return the Command
      if(comm_sz > SZ_CMD_SETPARAM1)
      {
        // Update the End Pointer
        comm_cmd_sz = SZ_CMD_SETPARAM1;
        comm_ptr_next = (uint8_t *)&comm_buffer[SZ_CMD_SETPARAM1];
      }
      break;
    }

    // Check for Parameter set 2
    if(strncmp(cmd_setparam2, comm_buffer, SZ_CMD_SETPARAM2) == 0)
    {
      ret = COMM_CMD_SETPARAM2; // In any case return the Command
      if(comm_sz > SZ_CMD_SETPARAM2)
      {
        // Update the End Pointer
        comm_cmd_sz = SZ_CMD_SETPARAM2;
        comm_ptr_next = (uint8_t *)&comm_buffer[SZ_CMD_SETPARAM2];
      }
      break;
    }

    // Check for Parameter set 3
    if(strncmp(cmd_setparam3, comm_buffer, SZ_CMD_SETPARAM3) == 0)
    {
      ret = COMM_CMD_SETPARAM3; // In any case return the Command
      if(comm_sz > SZ_CMD_SETPARAM3)
      {
        // Update the End Pointer
        comm_cmd_sz = SZ_CMD_SETPARAM3;
        comm_ptr_next = (uint8_t *)&comm_buffer[SZ_CMD_SETPARAM3];
      }
      break;
    }

    // Check for Parameter set 4 Command
    if(strncmp(cmd_setparam4, comm_buffer, SZ_CMD_SETPARAM4) == 0)
    {
      if(comm_sz > SZ_CMD_SETPARAM4)
      {
        ret = COMM_CMD_SETPARAM4;
        // Update the End Pointer
        comm_cmd_sz = SZ_CMD_SETPARAM4;
        comm_ptr_next = (uint8_t *)&comm_buffer[SZ_CMD_SETPARAM4];
      }
      break;
    }

  }while(0);

  // Return the Final Command after processing
  return ret;
}
