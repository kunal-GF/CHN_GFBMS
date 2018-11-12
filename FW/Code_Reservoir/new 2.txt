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
            eeprom_write_byte(0x46, afe_isr_reg[87]);
            eeprom_write_byte(0x47, afe_isr_reg[88]);
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
            eeprom_write_byte(0x46, afe_isr_reg[89]);
            eeprom_write_byte(0x47, afe_isr_reg[90]);
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
            eeprom_write_byte(0x46, afe_isr_reg[91]);
            eeprom_write_byte(0x47, afe_isr_reg[92]);
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
          eeprom_write_byte(0x2A, afe_isr_reg[95]);
          eeprom_write_byte(0x2B, afe_isr_reg[96]);
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
