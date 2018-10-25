/*
 * I2C_process.c
 *
 * Created: 10/24/2018 10:02:18 PM
 *  Author: User
 */ 






unsigned char CRC8(unsigned char *ptr, unsigned char len,unsigned char key)
{
	unsigned char i;
	unsigned char crc=0;
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