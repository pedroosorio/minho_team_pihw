/*
   Omni3MD.h - Library for interfacing with omni-3md (3 motor driver controller) from www.botnroll.com
   Created by Nino Pereira, April 28, 2011.
   Last Update by José Cruz, July 23, 2013.
   Ported to RaspberryPi 3's Raspbian by Pedro Osório, January, 2017.
   Released into the public domain.

   For more detailed explanation please refer to 
   http://botnroll.com/omni3md/ for the software and hardware manual.
*/

#ifndef Omni3MD_h
#define Omni3MD_h

#ifndef rttimer_h
#include "utilities/rttimer.h"
#endif

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
#include <vector>

class Omni3MD
{
   public:
   /// \brief class constructor. Assigns desired address to I2C 
   /// file descriptor, it should match device's address. Also
   /// performs the connection to the device.
   /// \param omniAddress - Omni3MD I2C 7bit address (default is 0x18)
   /// \param i2c_bus_mutex - pointer to i2c_bus_mutex used if more
   /// than one device is using i2c bus
   Omni3MD(uint8_t omniAddress, pthread_mutex_t *i2c_bus_mutex = NULL);

   /* Setup Routines */
   /*************************************************************/
   /// \brief function to connect to the I2C bus and to the desired
   /// device.
   /// \param omniAddress - Omni3MD I2C 7bit address (default is 0x18)
   /// \return - returns -1 on error.
   int i2c_connect(uint8_t omniAddress);

   /// \brief function to set Omni3MD's I2C address to be used in 
   /// further I2C bus communications.
   /// \return - returns -1 on error.
   int i2c_start_transmission();

   /// \brief function to change Omni3MD's I2C address. This function
   /// will send the respective command to make the change in the 
   /// board.
   /// \param newAddress - new Omni3MD I2C 7bit address (default 
   ///is 0x18)
   void set_i2c_address (uint8_t newAddress);

   /// \brief function to change Omni3MD's safety timeout. This
   /// function will send the respective command to make the change
   /// in the  board.
   /// \param timeout - value in ten's of milliseconds
   void set_i2c_timeout (uint8_t timeout);

   /// \brief function to start calibration routine. This function
   /// will send the respective command to make the change in the 
   /// board.
   /// \param way1 - direction of calibration for motor 1
   /// \param way2 - direction of calibration for motor 2
   /// \param way3 - direction of calibration for motor 3
   void calibrate(bool way1,bool way2,bool way3);

   /// \brief function to set internal PID control parameters. This
   /// function will send the respective command to make the change 
   /// in the board.
   /// \param Kp - proportional gain for the PID controller
   /// \param Ki - integral gain for the PID controller
   /// \param Kd - derivative gain for the PID controller
   void set_PID(int Kp, int Ki, int Kd);

   /// \brief function to set internal acceleration ramp. This
   /// function will send the respective command to make the change 
   /// in the board.
   /// \param time - acceleration ramp time in milliseconds
   /// \param slope - ramp inclination
   /// \param Kl - special control factor to be applied in PID's
   /// first cycle
   void set_ramp(int time, int slope, int Kl);

   /// \brief function to set encoder value. This function will
   /// send the respective command to make the change in the board.
   /// \param encoder - id of the encoder to be changed
   /// \param encValue - value to be applied to the encoder
   void set_enc_value(uint8_t encoder, int encValue);

   /// \brief function to set the encoder prescaler, subsampling
   /// its pulses. This function will send the respective command
   /// to make the change in the board.
   /// \param encoder - id of the encoder to be changed
   /// \param value - value of the prescaler 
   void set_prescaler(uint8_t encoder, uint8_t value);
   
   /// \brief function to set configuration parameters when
   /// differential movement in SI units is used. This function will
   /// send the respective command to make the change in the board.
   /// \param axis_radius - axis radius in millimeters
   /// \param wheel_radius - wheel radius in millimeters
   /// \param gearbox_factor - major reduction factor of the gearbox
   /// \param encoder_cpr - pulses generated by the encoder per motor
   /// turn
   void set_differential(double axis_radius, double whell_radius, double gearbox_factor, double encoder_cpr);
   /*************************************************************/
   
   /* Reading Routines */
   /*************************************************************/
   /// \brief function to read Omni3MD's temperature.
   /// return - temperature value in ºC
   float read_temperature();

   /// \brief function to read Omni3MD's battery.
   /// return - temperature value in volts
   float read_battery();
   
   /// \brief function to read Omni3MD's firmware version.
   /// return - release and intermediary version of the firmware
   float read_firmware();
   
   /// \brief function to read Omni3MD's firmware version.
   /// param firm1 - pointer to release firmware version
   /// param firm2 - pointer to intermediary firmware version
   /// param firm3 - pointer firmware development version
   void read_firmware(uint8_t* firm1,uint8_t* firm2,uint8_t* firm3);

   /// \brief function to read Omni3MD's control rate.
   /// \return - control rate
   uint8_t read_control_rate();

   /// \brief function to read Omni3MD's encoder 1.
   /// \return - value of encoder 1
   int read_enc1();

   /// \brief function to read Omni3MD's encoder 2.
   /// \return - value of encoder 2
   int read_enc2();

   /// \brief function to read Omni3MD's encoder 3.
   /// \return - value of encoder 3
   int read_enc3();

   /// \brief function to read Omni3MD's maximum value for encoder 1.
   /// \return - maximum value of encoder 1
   int read_enc1_max();

   /// \brief function to read Omni3MD's maximum value for encoder 2.
   /// \return - maximum value of encoder 2
   int read_enc2_max();
   
   /// \brief function to read Omni3MD's maximum value for encoder 3.
   /// \return - maximum value of encoder 3
   int read_enc3_max();

   /// \brief function to read all Omni3MD's encoders.
   /// \param enc1 - pointer to encoder 1 value
   /// \param enc2 - pointer to encoder 2 value
   /// \param enc3 - pointer to encoder 3 value
   void read_encoders(int16_t* enc1,int16_t* enc2,int16_t* enc3);

   /// \brief function to read all Omni3MD's encoders, battery and
   /// temperature.
   /// \param enc1 - pointer to encoder 1 value
   /// \param enc2 - pointer to encoder 2 value
   /// \param enc3 - pointer to encoder 3 value
   /// \param bat - pointer to battery value
   /// \param temp - pointer to temperature value
   void read_mov_data(int16_t* enc1,int16_t* enc2,int16_t* enc3,float* bat,float* temp);

   /// \brief function to read encoders, battery, temperature, firmware
   /// and encoders' maximum value.
   /// \param enc1 - pointer to encoder 1 value
   /// \param enc2 - pointer to encoder 2 value
   /// \param enc3 - pointer to encoder 3 value
   /// \param bat - pointer to battery value
   /// \param temp - pointer to temperature value
   /// \param firm_int - pointer to release firmware version
   /// \param frim_rel - pointer to intermediary firmware version
   /// \param firm_dev - pointer firmware development version
   /// \param ctrl_rate - pointer to control rate
   /// \param enc1max - pointer to encoder 1 maximum value
   /// \param enc2max - pointer to encoder 2 maximum value 
   /// \param enc3max - pointer to encoder 3 maximum value  
   void read_all_data(int* enc1,int* enc2,int* enc3,float* bat,float* temp,uint8_t* firm_int,uint8_t* frim_rel,uint8_t* firm_dev,uint8_t* ctrl_rate,int* enc1max,int* enc2max,int* enc3max);
   /*************************************************************/
   
   /* Movement Routines */
   /*************************************************************/
   /// \brief function to move the platform using holonomic motion.
   /// This function will send the respective command to actuate
   /// the board.
   /// \param linear_speed - linear speed for the holonomic motion
   /// \param rotational_speed - rotational speed for the 
   //  holonomic motion
   /// \param direction - direction of movement
   void mov_omni(uint8_t linear_speed,int rotational_speed,int direction);

   /// \brief function to move the platform using differential SI.
   /// This function will send the respective command to actuate
   /// the board.
   /// \param linear_speed - linear speed for the differential motion
   /// \param rotational_speed - rotational speed for the 
   //  differential motion
   void mov_dif_si(double linear_speed,double rotational_speed);

   /// \brief function to move a motor to a certain position.
   /// This function will send the respective command to actuate
   /// the board.
   /// \param motor - motor to be actuated
   /// \param speed - speed of movement
   /// \param encPosition - target encoder position
   /// \param stoptorque - have stop torque or not
   void mov_pos(uint8_t motor,int speed,int encPosition,bool stoptorque);

   /// \brief function to move the 3 motors with PID control.
   /// This function will send the respective command to actuate
   /// the board.
   /// \param speed1 - speed to apply to motor 1
   /// \param speed2 - speed to apply to motor 2
   /// \param speed3 - speed to apply to motor 3
   void mov_lin3m_pid(int speed1,int speed2,int speed3);
   
   /// \brief function to move the 1 motors with PID control.
   /// This function will send the respective command to actuate
   /// the board.
   /// \param motor - motor to actuate
   /// \param speed - speed to apply to motor
   void mov_lin1m_pid(uint8_t motor,int speed);

   /// \brief function to move the 3 motors without PID control.
   /// This function will send the respective command to actuate
   /// the board.
   /// \param speed1 - speed to apply to motor 1
   /// \param speed2 - speed to apply to motor 2
   /// \param speed3 - speed to apply to motor 3
   void mov_lin3m_nopid(int speed1,int speed2,int speed3);

   /// \brief function to move the 1 motors without PID control.
   /// This function will send the respective command to actuate
   /// the board.
   /// \param motor - motor to actuate
   /// \param speed - speed to apply to motor
   void mov_lin1m_nopid(uint8_t motor,int speed);

   /// \brief function to stop Omni3MD's motors.
   /// This function will send the respective command to actuate
   /// the board.
   void stop_motors();

   /// \brief function to save the current value of the encoders
   /// in board's EEPROM. This function will send the respective
   /// command to actuate the board.
   void save_position();
   /*************************************************************/

   private:
   /// \brief I2C bus file descriptor
   int i2c_fd;
   /// \brief boolean function to control initialization of the 
   /// file descriptor
   bool init;
   /// \brief file name of the I2C bus
   std::string device_name;
   /// \brief defined I2C address for Omni3MD
   uint8_t i2c_slave_address;
   /// \brief mutex for I2C bus
   pthread_mutex_t *i2c_mutex;
   /// \brief timestamps to implement safety timeout
   struct timeval t1,t0;
   /// \brief thread to run watchdog timer
   pthread_t watchdog_thread;
   /// \brief periodic_info to time watchdog function
   periodic_info watchdog_pi;
   /// \brief boolean to detect if timeout is enabled
   bool timeout_active;
   /// \brief file path for configuration file
   std::string filepath;

   /// \brief function to use I2C bus to read a byte from the
   /// specified register. 
   /// \param command - register to be read
   /// \return - returns -1 on failure. returns read byte on success
   int i2cRequestByte(uint8_t command);

   /// \brief function to use I2C bus to read a word from the
   /// specified register. 
   /// \param command - register to be read
   /// \return - returns -1 on failure. returns read word on success
   int i2cRequestWord(uint8_t command);
   
   /// \brief function to use I2C bus to read multiple bytes from the
   /// specified register. 
   /// \param command - register to be read
   /// \param length - amount of bytes to be read     
   /// \param values - pointer to vector of bytes to store read data
   /// \return - returns -1 on failure. returns length on success
   int i2cRequestData(uint8_t command, uint8_t length, uint8_t* values);

   /// \brief function to use I2C bus to write multiple bytes from the
   /// specified register. 
   /// \param command - register to be written
   /// \param length - amount of bytes to write    
   /// \param values - pointer to vector of bytes where data is stored
   /// \return - returns -1 on failure. returns length on success
   int i2cSendData(uint8_t command, uint8_t length, uint8_t* values);

   /// \brief watchdog function thread that detects lack of
   /// movement commands and stops motors when timeout expires
   static inline void *watchdog_timer(void *omni3md)
   {
      Omni3MD *omni = (Omni3MD *)omni3md;
      periodic_info *info = omni->getWatchdogTimer();
      make_periodic(info->period_us,info);
      struct timeval *t0,*t1;
      omni->getTimeVals(&t0,&t1);
      pthread_mutex_t *i2c_mutex;
      omni->getI2Cmutex(&i2c_mutex);
      int timeoutms = info->period_us/1000;

      while(omni->getTimeoutActive()){
         gettimeofday(t1,0);
         if(((float)(t1->tv_sec-t0->tv_sec)*1000.0 + (float)(t1->tv_usec-t0->tv_usec)/1000.0)>=timeoutms){
            pthread_mutex_lock(i2c_mutex);
            omni->stop_motors();
            pthread_mutex_unlock(i2c_mutex);
         }
         
         wait_period(info);   
      }
      
      return NULL;
   }
   
   /// \brief function to return watchdog periodic_info struct
   /// \return - periodic_info struct
   inline periodic_info *getWatchdogTimer() { return &watchdog_pi; };
   
   /// \brief function to return watchdog periodic_info struct
   /// \return - periodic_info struct
   inline bool getTimeoutActive() { return timeout_active; };

   /// \brief function to return watchdog timevals
   /// \param ta - timeval t0
   /// \param tb - timeval t1
   inline void getTimeVals(struct timeval **ta, struct timeval **tb)
   { 
      (*ta)=&t0; (*tb)=(&t1);
   };

   /// \brief function to return i2c_mutex to be used in watchdog
   /// thread
   /// \param mutex - i2c_mutex
   inline void getI2Cmutex(pthread_mutex_t **mutex) 
   {
      (*mutex) = i2c_mutex;
   };
};

#endif
