



uint8_t arr_src[4]={0x00, 0x01, 0x02, 0x03};
uint8_t arr_dst[4];
void setup() 
{
  Serial.begin(9600);
  /*Write one byte to EEPROM*/
  //eeprom_write_byte(0x00, 0x1C);

  /*Write 2 byte to EEPROM*/
  //eeprom_write_word(0x00, 0xBBAA);

  /*i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL2, 0x43); 
  i2c_WriteWithCRC(I2C_7BITADDR, CELLBAL1,  0x00);
  i2c_WriteWithCRC(I2C_7BITADDR, CELLBAL2,  0x00);
  i2c_WriteWithCRC(I2C_7BITADDR, CELLBAL3,  0x00);
  i2c_WriteWithCRC(I2C_7BITADDR, SYS_CTRL1, 0x18);
  i2c_WriteWithCRC(I2C_7BITADDR, PROTECT1,  0x1F);
  i2c_WriteWithCRC(I2C_7BITADDR, PROTECT2,  0x50);
  i2c_WriteWithCRC(I2C_7BITADDR, PROTECT3,  0xF7);
  i2c_WriteWithCRC(I2C_7BITADDR, OV_TRIP,   0xB8);
  i2c_WriteWithCRC(I2C_7BITADDR, UV_TRIP,   0x13);
  i2c_WriteWithCRC(I2C_7BITADDR, CC_CFG,    0x19);*/

  /**fault status high*/
  eeprom_write_byte(0x00, 0x00);
   /**fault status low*/
  eeprom_write_byte(0x01, 0x00);
   /**system control 1*/
  eeprom_write_byte(0x03, 0x18);
  /**system control 2*/
  eeprom_write_byte(0x04, 0x40);
  /**protect 1*/
  eeprom_write_byte(0x05, 0x1F);
  /**prot7ect 2*/
  eeprom_write_byte(0x06, 0x50);
  /**protect 3*/
  eeprom_write_byte(0x07, 0xF7);
  /**cell bal 1*/
  eeprom_write_byte(0x40, 0x00);
  /**cell bal 2*/
  eeprom_write_byte(0x41, 0x00);
  /**cell bal 3*/
  eeprom_write_byte(0x42, 0x00);
  
  /**cc config*/
  eeprom_write_byte(0x08, 0x19);
  
  /**cell over voltage trip*/
  eeprom_write_byte(0x09, 0xB3);
  
  /**cell over voltage release high*/
  eeprom_write_byte(0x0A, 0x29);
  
  /**cell over voltage release low*/
  eeprom_write_byte(0x0B, 0xA2);
  
  /**cell over voltage count high*/
  eeprom_write_byte(0x0C, 0x00);
  
  /**cell over voltage count low*/
  eeprom_write_byte(0x0D, 0x00);
  
  /**cell under voltage trip threshold*/
  eeprom_write_byte(0x0E, 0x13);
  
  /**cell under voltage release high bit*/
  eeprom_write_byte(0x0F, 0x23);
  
  /**cell under voltage release low bit*/
  eeprom_write_byte(0x10, 0x50);
  
  /** cell under voltage trip count high bits*/
  eeprom_write_byte(0x11, 0x00);
  
  /** cell under voltage trip count low bits*/
  eeprom_write_byte(0x12, 0x00);
  
  /**pack over voltage trip high*/
  eeprom_write_byte(0x13, 0x86);
  
  /**pack over voltage trip low*/
  eeprom_write_byte(0x14, 0x97);
  
  /**pack over voltage release high bits*/
  eeprom_write_byte(0x15, 0x82);
  
  /** pack over voltage release lower bits*/
  eeprom_write_byte(0x16, 0x0A);
  
  /** pack overvoltage trip times high bits*/
  eeprom_write_byte(0x17, 0x00);
  
  /** pack overvoltage trip times low bits*/
  eeprom_write_byte(0x18, 0x00);
  
  /**pack under voltage trip higher 8 bits*/
  eeprom_write_byte(0x19, 0x75);
  
  /**pack under voltage trip lowwer 8 bits*/
  eeprom_write_byte(0x1A, 0x4A);
  
  /**pack under voltage release high bits*/
  eeprom_write_byte(0x1B, 0x77);
  
  /** pack under voltage release low bits*/
  eeprom_write_byte(0x1C, 0xD6);
  
  /** pack under voltage trip times high bits*/
  eeprom_write_byte(0x1D, 0x00);
  
  /** pack under voltage trip times low bits*/
  eeprom_write_byte(0x1E, 0x00);

  /**pack over temperature trip threshold*/
  eeprom_write_byte(0x1F, 0x23);

  /**pack over temperature release threshold*/
  eeprom_write_byte(0x20, 0x23);

  /**pack over temperature trip count higher bits*/
  eeprom_write_byte(0x21, 0x00);

  /**pack over temperature trip count lower bits*/
  eeprom_write_byte(0x22, 0x00);

  /**pack under temperature trip threshold*/
  eeprom_write_byte(0x23, 0x18);

  /**pack under temperature release threshold*/
  eeprom_write_byte(0x24, 0x1B);

  /**pack under temperature trip count times higher bits*/
  eeprom_write_byte(0x25, 0x00);

  /**pack under temperature trip count times lower bits*/
  eeprom_write_byte(0x26, 0x00);

  /**short circuit in discharge faults*/
  eeprom_write_byte(0x27, 0x00);

  /**short circuit in discharge faults count higher bits*/
  eeprom_write_byte(0x28, 0x00);

  /**short circuit in discharge faults count lower bits*/
  eeprom_write_byte(0x29, 0x00);

  /**over current in charge fault higher bits*/
  eeprom_write_byte(0x2A, 0x00);

  /**over current in charge fault lower bits*/
  eeprom_write_byte(0x2B, 0xE9);

  /**over current in discharge fault higher bits*/
  eeprom_write_byte(0x2C, 0xFF);

  /**over current in discharge fault lower bits*/
  eeprom_write_byte(0x2D, 0x13);

  /**over current in charge fault higher bits*/
  //eeprom_write_byte(0x2E, 0x00);

  /**key recharge %centage*/
  eeprom_write_byte(0x43, 0x00);

}

void loop() 
{
  /*Read one byte from EEPROM*/
  //Serial.println(eeprom_read_byte(0x00), HEX);

  /*Read 2 bytes from EEPROM*/
  //Serial.println(eeprom_read_word(0x01), HEX);

  delay(1000);
}
