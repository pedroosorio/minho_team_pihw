/*
   Omni3MD.h - Library for interfacing with omni-3md (3 motor driver controller) from www.botnroll.com
   Created by Nino Pereira, April 28, 2011.
   Last Update by José Cruz, July 23, 2013.
   Ported to RaspberryPi 3's Raspbian by Pedro Osório, January, 2017.
   Released into the public domain.
*/

#ifndef Omni3MD_h
#define Omni3MD_h

#include "omni3md_defines.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <sys/types.h>
#include <iostream>

class Omni3MD
{
   public:
   Omni3MD(uint8_t omniAddress);

   /* Setup Routines */
   /*************************************************************/
   int i2c_connect(uint8_t omniAddress);
   int i2c_start_transmission();
   void set_i2c_address (uint8_t newAddress);
   void set_i2c_timeout (uint8_t timeout);
   void calibrate(bool way1,bool way2,bool way3);
   void set_PID(int Kp, int Ki, int Kd);
   void set_ramp(int time, int slope, int Kl);
   void set_enc_value(uint8_t encoder, int encValue);
   void set_prescaler(uint8_t encoder, uint8_t value);
   void set_differential(double axis_radius, double whell_radius, double gearbox_factor, double encoder_cpr);
   /*************************************************************/
   
   /* Reading Routines */
   /*************************************************************/
   float read_temperature();
   float read_battery();
   float read_firmware();
   void read_firmware(uint8_t*,uint8_t*,uint8_t*);
   uint8_t read_control_rate();
   int read_enc1();
   int read_enc2();
   int read_enc3();
   int read_enc1_max();
   int read_enc2_max();
   int read_enc3_max();
   void read_encoders(int*,int*,int*);
   void read_mov_data(int*,int*,int*,float*,float*);
   void read_all_data(int*,int*,int*,float*,float*,uint8_t*,uint8_t*,uint8_t*,uint8_t*,int*,int*,int*);
   /*************************************************************/
   
   /* Movement Routines */
   /*************************************************************/
   void mov_omni(uint8_t linear_speed,int rotational_speed,int direction);
   void mov_dif_si(double linear_speed,double rotational_speed);
   void mov_pos(uint8_t motor,int speed,int encPosition,bool stoptorque);
   void mov_lin3m_pid(int speed1,int speed2,int speed3);
   void mov_lin1m_pid(uint8_t motor,int speed);
   void mov_lin3m_nopid(int speed1,int speed2,int speed3);
   void mov_lin1m_nopid(uint8_t motor,int speed);
   void stop_motors();
   void save_position();
   /*************************************************************/

   private:
   int i2c_fd;
   bool init;
   std::string device_name;
   uint8_t i2c_slave_address;

   int i2cRequestByte(uint8_t command);
   int i2cRequestWord(uint8_t command);
   int i2cRequestData(uint8_t command, uint8_t length, uint8_t* values);
   int i2cSendData(uint8_t command, uint8_t length, uint8_t* values);
};

#endif
