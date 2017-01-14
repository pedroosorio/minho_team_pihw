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
}

/* Setup Routines */
/*************************************************************/
void Omni3MD::i2c_connect(byte omniAddress)
{
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

byte Omni3MD::i2cRequestByte(byte addressValue, byte command)
{
   return 0;
}

int Omni3MD::i2cRequestWord(byte addressValue, byte command)
{
   return 0;
}

void Omni3MD::i2cRequestData(byte adreessValue, byte command)
{
}

void Omni3MD::i2cSendData(byte addressValue, byte command, byte buffer[], byte numBytes)
{
}
