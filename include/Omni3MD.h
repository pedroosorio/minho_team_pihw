/*
   Omni3MD.h - Library for interfacing with omni-3md (3 motor driver controller) from www.botnroll.com
   Created by Nino Pereira, April 28, 2011.
   Last Update by José Cruz, July 23, 2013.
   Ported to RaspberryPi 3 by Pedro Osório, January, 2017.
   Released into the public domain.
*/

#ifndef Omni3MD_h
#define Omni3MD_h

#include "omni3md_defines.h"

class Omni3MD
{
   public:
   Omni3MD(byte omniAddress);

   /* Setup Routines */
   /*************************************************************/
   void i2c_connect(byte omniAddress);
   void set_i2c_address (byte newAddress);
   void set_i2c_timeout (byte timeout);
   void calibrate(bool way1,bool way2,bool way3);
   void set_PID(int Kp, int Ki, int Kd);
   void set_ramp(int time, int slope, int Kl);
   void set_enc_value(byte encoder, int encValue);
   void set_prescaler(byte encoder, byte value);
   void set_differential(double axis_radius, double whell_radius, double gearbox_factor, double encoder_cpr);
   /*************************************************************/
   
   /* Reading Routines */
   /*************************************************************/
   float read_temperature();
   float read_battery();
   float read_firmware();
   void read_firmware(byte*,byte*,byte*);
   byte read_control_rate();
   int read_enc1();
   int read_enc2();
   int read_enc3();
   int read_enc1_max();
   int read_enc2_max();
   int read_enc3_max();
   void read_encoders(int*,int*,int*);
   void read_mov_data(int*,int*,int*,float*,float*);
   void read_all_data(int*,int*,int*,float*,float*,byte*,byte*,byte*,byte*,int*,int*,int*);
   /*************************************************************/
   
   /* Movement Routines */
   /*************************************************************/
   void mov_omni(byte linear_speed,int rotational_speed,int direction);
   void mov_dif_si(double linear_speed,double rotational_speed);
   void mov_pos(byte motor,int speed,int encPosition,bool stoptorque);
   void mov_lin3m_pid(int speed1,int speed2,int speed3);
   void mov_lin1m_pid(byte motor,int speed);
   void mov_lin3m_nopid(int speed1,int speed2,int speed3);
   void mov_lin1m_nopid(byte motor,int speed);
   void stop_motors();
   void save_position();
   /*************************************************************/

   private:
   byte _omniAddress;
   byte i2cRequestByte(byte addressValue, byte command);
   int  i2cRequestWord(byte addressValue, byte command);
   void i2cRequestData(byte adreessValue, byte command);
   void i2cSendData(byte addressValue, byte command, byte buffer[], byte numBytes);
};

#endif
