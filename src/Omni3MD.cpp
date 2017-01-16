/*
   Omni3MD.h - Library for interfacing with omni-3md (3 motor driver controller) from www.botnroll.com
   Created by Nino Pereira, April 28, 2011.
   Last Update by José Cruz, July 23, 2013.
   Ported to RaspberryPi 3 by Pedro Osório, January, 2017.
   Released into the public domain.
*/

#include "Omni3MD.h"

Omni3MD::Omni3MD(uint8_t omniAddress)
{
   i2c_slave_address = omniAddress;
   init = false;
   device_name = "/dev/i2c-1";

   if(i2c_connect(omniAddress)>0)printf("_OMNI3MD: Initialization Successful.\n");
   else printf("_OMNI3MD: Initialization Failed.\n");
}

/* Setup Routines */
/*************************************************************/
int Omni3MD::i2c_connect(uint8_t omniAddress)
{
   if(init) { printf("_OMNI3MD: I2C Interface already opened"); return -1; }

   printf("_OMNI3MD: Opening Omni3MD in %s\n",device_name.c_str());
   i2c_slave_address = omniAddress;
   /* Open I2C Interface */
	if((i2c_fd = open(device_name.c_str(), O_RDWR)) < 0){
		perror("_OMNI3MD: Couldn't open I2C Interface.");
      return -1;
	}
	
   if (ioctl (i2c_fd, I2C_SLAVE, i2c_slave_address) < 0){
      perror("_OMNI3MD: Couldn't access I2C Device.");
      return -1;
   }

   init = true;
   return i2c_fd;
}

void Omni3MD::set_i2c_address (uint8_t newAddress)
{
   uint8_t buffer[]={newAddress, KEY1, newAddress, KEY2};
   i2cSendData(COMMAND_I2C_ADD,buffer,sizeof(buffer));
   i2c_slave_address = newAddress;

   if (ioctl (i2c_fd, I2C_SLAVE, i2c_slave_address) < 0){
      perror("_OMNI3MD: Couldn't access I2C Device.");
   }
}
 
void Omni3MD::set_i2c_timeout (uint8_t timeout)
{
   uint8_t buffer[]={timeout, KEY1, timeout, KEY2}; 
   i2cSendData(COMMAND_TIMEOUT_I2C,buffer,sizeof(buffer));
}

void Omni3MD::calibrate(bool way1,bool way2,bool way3)
{
   uint8_t buffer[]={(uint8_t)way1, (uint8_t)way2,(uint8_t)way3, KEY1, KEY2};
   i2cSendData(COMMAND_CALIBRATE,buffer,sizeof(buffer));
}

void Omni3MD::set_PID(int Kp, int Ki, int Kd)
{
}

void Omni3MD::set_ramp(int time, int slope, int Kl)
{
}

void Omni3MD::set_enc_value(uint8_t encoder, int encValue)
{
}

void Omni3MD::set_prescaler(uint8_t encoder, uint8_t value)
{
   uint8_t buffer[]={encoder,value,KEY1,KEY2};
   i2cSendData(COMMAND_PRESCALER_CFG,buffer,sizeof(buffer));
}

void Omni3MD::set_differential(double axis_radius, double whell_radius, double gearbox_factor, double encoder_cpr)
{
}
/*************************************************************/
   
/* Reading Routines */
/*************************************************************/
float Omni3MD::read_temperature()
{
   return (float)(i2cRequestWord(COMMAND_TEMP))/10.0;
}

float Omni3MD::read_battery()
{
   return (float)(i2cRequestWord(COMMAND_BAT))/10.0;
}

float Omni3MD::read_firmware()
{
   uint8_t firmI=i2cRequestByte(COMMAND_FIRMWARE_INT);
   uint8_t firmD=i2cRequestByte(COMMAND_FIRMWARE_DEC);
   int firmwareValue=(firmI*100)+firmD;
   return (float)((firmwareValue/100.0));
}

void Omni3MD::read_firmware(uint8_t*,uint8_t*,uint8_t*)
{
}

uint8_t Omni3MD::read_control_rate()
{
   return i2cRequestByte(COMMAND_CTRL_RATE);
}

int Omni3MD::read_enc1()
{
   return i2cRequestWord(COMMAND_ENC1_INC);
}

int Omni3MD::read_enc2()
{
   return i2cRequestWord(COMMAND_ENC2_INC);
}

int Omni3MD::read_enc3()
{
   return i2cRequestWord(COMMAND_ENC3_INC);
}

int Omni3MD::read_enc1_max()
{
   return i2cRequestWord(COMMAND_ENC1_MAX);
}

int Omni3MD::read_enc2_max()
{
   return i2cRequestWord(COMMAND_ENC2_MAX);
}

int Omni3MD::read_enc3_max()
{
   return i2cRequestWord(COMMAND_ENC3_MAX);
}

void Omni3MD::read_encoders(int*,int*,int*)
{
}

void Omni3MD::read_mov_data(int*,int*,int*,float*,float*)
{
}

void Omni3MD::read_all_data(int*,int*,int*,float*,float*,uint8_t*,uint8_t*,uint8_t*,uint8_t*,int*,int*,int*)
{
}
/*************************************************************/
   
/* Movement Routines */
/*************************************************************/
void Omni3MD::mov_omni(uint8_t linear_speed,int rotational_speed,int direction)
{
}

void Omni3MD::mov_dif_si(double linear_speed,double rotational_speed)
{
}

void Omni3MD::mov_pos(uint8_t motor,int speed,int encPosition,bool stoptorque)
{
}

void Omni3MD::mov_lin3m_pid(int speed1,int speed2,int speed3)
{
}

void Omni3MD::mov_lin1m_pid(uint8_t motor,int speed)
{
}

void Omni3MD::mov_lin3m_nopid(int speed1,int speed2,int speed3)
{
}

void Omni3MD::mov_lin1m_nopid(uint8_t motor,int speed)
{
}

void Omni3MD::stop_motors()
{
   uint8_t buffer[]={KEY1,KEY2};
   i2cSendData(COMMAND_STOP,buffer,sizeof(buffer));
}

void Omni3MD::save_position()
{
   uint8_t buffer[]={KEY1,KEY2};
   i2cSendData(COMMAND_SAVE_POS,buffer,sizeof(buffer));
}
/*************************************************************/

int Omni3MD::i2cRequestByte(uint8_t command)
{
   return i2c_smbus_read_byte_data(i2c_fd,command);  
}

int Omni3MD::i2cRequestWord(uint8_t command)
{
  return i2c_smbus_read_word_data(i2c_fd,command);   
}

int i2cRequestData(uint8_t command, uint8_t length, uint8_t* values)
{
   int bytes = i2c_smbus_read_block_data(i2c_fd,command,values);
   if(bytes!=length) return -1;
   else return bytes;
}

int Omni3MD::i2cSendData(uint8_t command, uint8_t length, uint8_t* values)
{
   return i2c_smbus_write_block_data(i2c_fd,command,length,values);
}
