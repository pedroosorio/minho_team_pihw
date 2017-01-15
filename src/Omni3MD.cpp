/*
   Omni3MD.h - Library for interfacing with omni-3md (3 motor driver controller) from www.botnroll.com
   Created by Nino Pereira, April 28, 2011.
   Last Update by José Cruz, July 23, 2013.
   Ported to RaspberryPi 3 by Pedro Osório, January, 2017.
   Released into the public domain.
*/

#include "Omni3MD.h"

Omni3MD::Omni3MD(byte omniAddress)
{
   i2c_slave_address = omniAddress;
   init = false;
   device_name = "/dev/i2c-1";

   if(i2c_connect(omniAddress)>0)printf("Omni3MD: Initialization Successful.\n");
   else printf("Omni3MD: Initialization Failed.\n");
}

/* Setup Routines */
/*************************************************************/
int Omni3MD::i2c_connect(byte omniAddress)
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
      perror("_OMNI3MD: Couldn't access I2C Device @%x.",i2c_slave_address);
      return -1;
   }

   init = true;
   return i2c_fd;
}

void Omni3MD::set_i2c_address (byte newAddress)
{
}
 
void Omni3MD::set_i2c_timeout (byte timeout)
{
}

void Omni3MD::calibrate(bool way1,bool way2,bool way3)
{
}

void Omni3MD::set_PID(int Kp, int Ki, int Kd)
{
}

void Omni3MD::set_ramp(int time, int slope, int Kl)
{
}

void Omni3MD::set_enc_value(byte encoder, int encValue)
{
}

void Omni3MD::set_prescaler(byte encoder, byte value)
{
}

void Omni3MD::set_differential(double axis_radius, double whell_radius, double gearbox_factor, double encoder_cpr)
{
}
/*************************************************************/
   
/* Reading Routines */
/*************************************************************/
float Omni3MD::read_temperature()
{
   return 0.0;
}

float Omni3MD::read_battery()
{
   return 0.0;
}

float Omni3MD::read_firmware()
{
   return 0.0;
}

void Omni3MD::read_firmware(byte*,byte*,byte*)
{
}

byte Omni3MD::read_control_rate()
{
   return 0;
}

int Omni3MD::read_enc1()
{
   return 0;
}

int Omni3MD::read_enc2()
{
   return 0;
}

int Omni3MD::read_enc3()
{
   return 0;
}

int Omni3MD::read_enc1_max()
{
   return 0;
}

int Omni3MD::read_enc2_max()
{
   return 0;
}

int Omni3MD::read_enc3_max()
{
   return 0;
}

void Omni3MD::read_encoders(int*,int*,int*)
{
}

void Omni3MD::read_mov_data(int*,int*,int*,float*,float*)
{
}

void Omni3MD::read_all_data(int*,int*,int*,float*,float*,byte*,byte*,byte*,byte*,int*,int*,int*)
{
}
/*************************************************************/
   
/* Movement Routines */
/*************************************************************/
void Omni3MD::mov_omni(byte linear_speed,int rotational_speed,int direction)
{
}

void Omni3MD::mov_dif_si(double linear_speed,double rotational_speed)
{
}

void Omni3MD::mov_pos(byte motor,int speed,int encPosition,bool stoptorque)
{
}

void Omni3MD::mov_lin3m_pid(int speed1,int speed2,int speed3)
{
}

void Omni3MD::mov_lin1m_pid(byte motor,int speed)
{
}

void Omni3MD::mov_lin3m_nopid(int speed1,int speed2,int speed3)
{
}

void Omni3MD::mov_lin1m_nopid(byte motor,int speed)
{
}

void Omni3MD::stop_motors()
{
}

void Omni3MD::save_position()
{
}
/*************************************************************/

int Omni3MD::i2cRequestByte(byte command)
{
  union i2c_smbus_data data;
  if(i2c_smbus_access (i2c_fd, I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA, &data)) return -1 ;
  else return data.byte & 0xFF ;   
}

int Omni3MD::i2cRequestWord(byte command)
{
  union i2c_smbus_data data;
  if (i2c_smbus_access (i2c_fd, I2C_SMBUS_READ, command, I2C_SMBUS_WORD_DATA, &data)) return -1 ;
  else return data.word & 0xFFFF ;   
}

void Omni3MD::i2cSendData(byte command, byte buffer[], byte numBytes)
{
   union i2c_smbus_data data; 
   if (i2c_smbus_access (i2c_fd, I2C_SMBUS_WRITE, command, numBytes, &buffer)) return -1 ;  
   else return numBytes;
}
