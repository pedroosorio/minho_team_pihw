/*
   Omni3MD.h - Library for interfacing with omni-3md (3 motor driver controller) from www.botnroll.com
   Created by Nino Pereira, April 28, 2011.
   Last Update by José Cruz, July 23, 2013.
   Ported to RaspberryPi 3's Raspbian by Pedro Osório, January, 2017.
   Released into the public domain.

   For more detailed explanation please refer to 
   http://botnroll.com/omni3md/ for the software and hardware manual.
*/

#include "Omni3MD/Omni3MD.h"

Omni3MD::Omni3MD(uint8_t omniAddress, pthread_mutex_t *i2c_bus_mutex)
{
   i2c_slave_address = omniAddress;
   i2c_mutex = i2c_bus_mutex;
   init = false;
   device_name = "/dev/i2c-1";
   timeout_active = false;

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

int Omni3MD::i2c_start_transmission()
{
   if (ioctl (i2c_fd, I2C_SLAVE, i2c_slave_address) < 0){
      perror("_OMNI3MD: Couldn't access I2C Device.");
      return -1;
    }
   return i2c_fd;
}

void Omni3MD::set_i2c_address (uint8_t newAddress)
{
   uint8_t buffer[]={newAddress, KEY1, newAddress, KEY2};
   i2cSendData(COMMAND_I2C_ADD,sizeof(buffer),buffer);
   i2c_slave_address = newAddress;

   if (ioctl (i2c_fd, I2C_SLAVE, i2c_slave_address) < 0){
      perror("_OMNI3MD: Couldn't access I2C Device.");
   }
}
 
void Omni3MD::set_i2c_timeout (uint8_t timeout)
{
   uint8_t buffer[]={timeout, KEY1, timeout, KEY2}; 
   i2cSendData(COMMAND_TIMEOUT_I2C,sizeof(buffer),buffer);

   if(timeout>0){
      if(i2c_mutex!=NULL){
         // Create safety here
         gettimeofday(&t1,0);
         t0 = t1;
         unsigned int timeout_us = timeout*10000; //in us
         watchdog_pi.id = 1; watchdog_pi.period_us = timeout_us;
         timeout_active = true;
         pthread_create(&watchdog_thread,NULL,watchdog_timer,this);
      }else printf("_OMNI3MD: No i2c_bus_mutex provided. This may cause errors in transmissions\n");
   } else {
      timeout_active = false;   
   }
   
}

void Omni3MD::calibrate(bool way1,bool way2,bool way3)
{
   uint8_t buffer[]={(uint8_t)way1, (uint8_t)way2,(uint8_t)way3, KEY1, KEY2};
   i2cSendData(COMMAND_CALIBRATE,sizeof(buffer),buffer);
}

void Omni3MD::set_PID(int Kp, int Ki, int Kd)
{
   uint8_t buffer[] = {(uint8_t)(Kp>>8),(uint8_t)(Kp&0xFF),(uint8_t)(Ki>>8),(uint8_t)(Ki&0xFF),(uint8_t)(Kd>>8),(uint8_t)(Kd&0xFF)};
   i2cSendData(COMMAND_GAIN_CFG,sizeof(buffer),buffer);
}

void Omni3MD::set_ramp(int time, int slope, int Kl)
{
   uint8_t buffer[] = {(uint8_t)(time>>8),(uint8_t)(time&0xFF),(uint8_t)(slope>>8),(uint8_t)(slope&0xFF),(uint8_t)(Kl>>8),(uint8_t)(Kl&0xFF)};
   i2cSendData(COMMAND_RAMP_CFG,sizeof(buffer),buffer);
}

void Omni3MD::set_enc_value(uint8_t encoder, int encValue)
{
   uint8_t buffer[] = {encoder,(uint8_t)(encValue>>8),(uint8_t)(encValue&0xFF),KEY1,KEY2};
   i2cSendData(COMMAND_POS_ENC_PRESET,sizeof(buffer),buffer);   
}

void Omni3MD::set_prescaler(uint8_t encoder, uint8_t value)
{
   uint8_t buffer[]={encoder,value,KEY1,KEY2};
   i2cSendData(COMMAND_PRESCALER_CFG,sizeof(buffer),buffer);
}

void Omni3MD::set_differential(double axis_radius, double whell_radius, double gearbox_factor, double encoder_cpr)
{
   int axis=(int)(axis_radius*100);
   int wheel=(int)(whell_radius*100);
   int gearb=(int)(gearbox_factor*10);
   int cpr=(int)(encoder_cpr*10);
   uint8_t buffer[] = {(uint8_t)(axis>>8),(uint8_t)(axis&0xFF),(uint8_t)(wheel>>8),(uint8_t)(wheel&0xFF),(uint8_t)(gearb>>8),(uint8_t)(gearb&0xFF),(uint8_t)(cpr>>8),(uint8_t)(cpr&0xFF)};
   i2cSendData(COMMAND_POS_ENC_PRESET,sizeof(buffer),buffer);   
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
   return (float)i2cRequestWord(COMMAND_BAT)/10.0;
}

float Omni3MD::read_firmware()
{
   uint8_t firmI=i2cRequestByte(COMMAND_FIRMWARE_INT);
   uint8_t firmD=i2cRequestByte(COMMAND_FIRMWARE_DEC);
   int firmwareValue=(firmI*100)+firmD;
   return (float)((firmwareValue/100.0));
}

void Omni3MD::read_firmware(uint8_t* firm1,uint8_t* firm2 ,uint8_t* firm3)
{
   uint8_t values[3] = {0,0,0};
   i2cRequestData(COMMAND_FIRMWARE_INT,3,values);  
   *firm1 = values[0]; *firm2 = values[1]; *firm3 = values[2];
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

void Omni3MD::read_encoders(int16_t* enc1,int16_t* enc2,int16_t* enc3)
{
   uint8_t values[6] = {0,0,0,0,0,0};
   i2cRequestData(COMMAND_ENC1_INC,6,values);
   *enc1 = (((int)values[0])<<8)|((int)values[1]&0xFF);
   *enc2 = (((int)(values[2]^0x80))<<8)|((int)values[3]&0xFF);
   *enc3 = (((int)(values[4]^0x80))<<8)|((int)values[5]&0xFF);
}

void Omni3MD::read_mov_data(int16_t* enc1,int16_t* enc2,int16_t* enc3,float* bat,float* temp)
{
   uint8_t values[10] = {0,0,0,0,0,0,0,0,0,0};
   i2cRequestData(COMMAND_ENC1_INC,10,values);
   *enc1 = (((int)values[0])<<8)|((int)values[1]&0xFF);
   *enc2 = (((int)(values[2]^0x80))<<8)|((int)values[3]&0xFF);
   *enc3 = (((int)(values[4]^0x80)<<8))|((int)values[5]&0xFF);
   *bat = ((float)((((int)values[6])<<8)|((int)values[7]&0xFF)))/10.0;
   *temp = ((float)((((int)values[8])<<8)|((int)values[9]&0xFF)))/10.0;
   
}

void Omni3MD::read_all_data(int* enc1,int* enc2,int* enc3,float* bat,float* temp,uint8_t* firm_int,uint8_t* frim_rel,uint8_t* firm_dev,uint8_t* ctrl_rate,int* enc1max,int* enc2max,int* enc3max)
{
}
/*************************************************************/
   
/* Movement Routines */
/*************************************************************/
void Omni3MD::mov_omni(uint8_t linear_speed,int rotational_speed,int direction)
{
   gettimeofday(&t0,0);
   uint8_t buffer[] = {linear_speed,(uint8_t)(rotational_speed>>8),(uint8_t)(rotational_speed&0xFF),(uint8_t)(direction>>8),(uint8_t)(direction&0xFF)};
   i2cSendData(COMMAND_MOV_OMNI,sizeof(buffer),buffer);
}

void Omni3MD::mov_dif_si(double linear_speed,double rotational_speed)
{
   gettimeofday(&t0,0);
   int lin_speed = (int)(linear_speed*1000);
   int rot_speed = (int)(rotational_speed*1000);

   uint8_t buffer[] = {(uint8_t)(lin_speed>>8),(uint8_t)(lin_speed&0xFF),(uint8_t)(rot_speed>>8),(uint8_t)(rot_speed&0xFF)};
   i2cSendData(COMMAND_MOV_DIF_SI,sizeof(buffer),buffer);
}

void Omni3MD::mov_pos(uint8_t motor,int speed,int encPosition,bool stoptorque)
{
   gettimeofday(&t0,0);
   uint8_t buffer[] = {(uint8_t)(speed>>8),(uint8_t)(speed&0xFF),(uint8_t)(encPosition>>8),(uint8_t)(encPosition&0xFF),(uint8_t)stoptorque};
   i2cSendData(COMMAND_MOV_POS,sizeof(buffer),buffer);
}

void Omni3MD::mov_lin3m_pid(int speed1,int speed2,int speed3)
{
   gettimeofday(&t0,0);
   uint8_t buffer[] = {(uint8_t)(speed1>>8),(uint8_t)(speed1&0xFF),(uint8_t)(speed2>>8),(uint8_t)(speed2&0xFF),(uint8_t)(speed3>>8),(uint8_t)(speed3&0xFF)};
   i2cSendData(COMMAND_MOV_LIN3M_PID,sizeof(buffer),buffer);
}

void Omni3MD::mov_lin1m_pid(uint8_t motor,int speed)
{
   uint8_t buffer[] = {motor,(uint8_t)(speed>>8),(uint8_t)(speed&0xFF)};
   i2cSendData(COMMAND_MOV_LIN1M_PID,sizeof(buffer),buffer);
}

void Omni3MD::mov_lin3m_nopid(int speed1,int speed2,int speed3)
{
   gettimeofday(&t0,0);
   uint8_t buffer[] = {(uint8_t)(speed1>>8),(uint8_t)(speed1&0xFF),(uint8_t)(speed2>>8),(uint8_t)(speed2&0xFF),(uint8_t)(speed3>>8),(uint8_t)(speed3&0xFF)};
   i2cSendData(COMMAND_MOV_LIN3M_NOPID,sizeof(buffer),buffer);
}

void Omni3MD::mov_lin1m_nopid(uint8_t motor,int speed)
{
   gettimeofday(&t0,0);
   uint8_t buffer[] = {motor,(uint8_t)(speed>>8),(uint8_t)(speed&0xFF)};
   i2cSendData(COMMAND_MOV_LIN1M_NOPID,sizeof(buffer),buffer);
}

void Omni3MD::stop_motors()
{
   uint8_t buffer[]={KEY1,KEY2};
   i2cSendData(COMMAND_STOP,sizeof(buffer),buffer);
}

void Omni3MD::save_position()
{
   uint8_t buffer[]={KEY1,KEY2};
   i2cSendData(COMMAND_SAVE_POS,sizeof(buffer),buffer);
}
/*************************************************************/

int Omni3MD::i2cRequestByte(uint8_t command)
{
   i2c_start_transmission();
   __u8 byte[1];
   if(i2c_smbus_read_i2c_block_data(i2c_fd,command,1,byte)!=1) return -1;
   return (int)byte[0];
}

int Omni3MD::i2cRequestWord(uint8_t command)
{
   i2c_start_transmission();
   __u8 bytes[2];
   if(i2c_smbus_read_i2c_block_data(i2c_fd,command,2,bytes)!=2) return -1;
   return ((int)bytes[0]<<8|(int)(bytes[1]&0xFF));  
}

int Omni3MD::i2cRequestData(uint8_t command, uint8_t length, uint8_t* values)
{
   i2c_start_transmission();
   int bytes = i2c_smbus_read_i2c_block_data(i2c_fd,command,length,values);
   if(bytes!=length) return -1;
   else return bytes;
}

int Omni3MD::i2cSendData(uint8_t command, uint8_t length, uint8_t* values)
{
   i2c_start_transmission();
   return i2c_smbus_write_i2c_block_data(i2c_fd,command,length,values);
}
