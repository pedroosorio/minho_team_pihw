/*
   Ada10Dof.h - Library for interfacing with Adafruit10Dof IMU
   Created by Pedro Osório, January, 2017.
   Released into the public domain.
*/

#include "Ada10Dof/Ada10Dof.h"

Ada10Dof::Ada10Dof(uint8_t adaAddress)
{
   i2c_slave_address = adaAddress;
   init = false;
   device_name = "/dev/i2c-1";

   if(i2c_connect(adaAddress)>0)printf("_ADA10DOF: Initialization Successful.\n");
   else printf("_ADA10DOF: Initialization Failed.\n");
}

/* Setup Routines */
/*************************************************************/
int Ada10Dof::i2c_connect(uint8_t adaAddress)
{
   if(init) { printf("_ADA10DOF: I2C Interface already opened"); return -1; }

   printf("_ADA10DOF: Opening Ada10Dof in %s\n",device_name.c_str());
   i2c_slave_address = adaAddress;
   /* Open I2C Interface */
	if((i2c_fd = open(device_name.c_str(), O_RDWR)) < 0){
		perror("_ADA10DOF: Couldn't open I2C Interface.");
      return -1;
	}
	
   if (ioctl (i2c_fd, I2C_SLAVE, i2c_slave_address) < 0){
      perror("_ADA10DOF: Couldn't access I2C Device.");
      return -1;
   }

   init = true;
   return i2c_fd;
}

int Ada10Dof::i2c_start_transmission()
{
   if (ioctl (i2c_fd, I2C_SLAVE, i2c_slave_address) < 0){
      perror("_ADA10DOF: Couldn't access I2C Device.");
      return -1;
    }
   return i2c_fd;
}
/*************************************************************/

int Ada10Dof::i2cRequestByte(uint8_t command)
{
   i2c_start_transmission();
   __u8 byte[1];
   if(i2c_smbus_read_i2c_block_data(i2c_fd,command,1,byte)!=1) return -1;
   return (int)byte[0];
}

int Ada10Dof::i2cRequestWord(uint8_t command)
{
   i2c_start_transmission();
   __u8 bytes[2];
   if(i2c_smbus_read_i2c_block_data(i2c_fd,command,2,bytes)!=2) return -1;
   return ((int)bytes[0]<<8|(int)(bytes[1]&0xFF));  
}

int Ada10Dof::i2cRequestData(uint8_t command, uint8_t length, uint8_t* values)
{
   i2c_start_transmission();
   int bytes = i2c_smbus_read_i2c_block_data(i2c_fd,command,length,values);
   if(bytes!=length) return -1;
   else return bytes;
}

int Ada10Dof::i2cSendData(uint8_t command, uint8_t length, uint8_t* values)
{
   i2c_start_transmission();
   return i2c_smbus_write_i2c_block_data(i2c_fd,command,length,values);
}
