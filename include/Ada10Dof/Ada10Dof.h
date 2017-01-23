/*
   Ada10Dof.h - Library for interfacing with Adafruit10Dof IMU
   Created by Pedro Osório, January, 2017.
   Released into the public domain.
*/

#ifndef Ada10Dof_h
#define Ada10Dof_h

#include "ada10dof_defines.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <sys/types.h>
#include <vector>
#include "rapidjson/document.h"     // rapidjson's DOM-style API
#include "rapidjson/prettywriter.h" // for stringify JSON
#include <iostream>
#include <sstream>
#include <fstream>
#include <sys/time.h>
using namespace rapidjson;
using namespace std;

#define CONFIG_FILE "/.mtconfigs/.ada10dof.cfg"

class Ada10Dof
{
   public:
   /// \brief class constructor. Assigns desired address to I2C 
   /// file descriptor, it should match device's address. Also
   /// performs the connection to the device.
   Ada10Dof();

   /* Setup Routines */
   /*************************************************************/
   /// \brief function to connect to the I2C bus and to the desired
   /// device.
   /// \param address - one of Ada10Dof I2C 7bit addresses
   /// \return - returns -1 on error.
   int i2c_connect(uint8_t address);

   /// \brief function to set Ada10Dof's I2C address to be used in 
   /// further I2C bus communications.
   /// \param address - one of Ada10Dof I2C 7bit addresses
   /// \return - returns -1 on error.
   int i2c_start_transmission(uint8_t address);

   /// \brief sets the current read value as the reference 0º
   void set_imu_reference();

   /// \brief reads standard file containing IMU configuration
   /// \return - true if reading was successful
   bool read_imu_configuration(int *ret);

   /// \brief set imu configuration
   /// \param st - angle step to be applied
   /// \param imu - imu true values
   /// \return - true if valid configuration
   bool set_imu_configuration(uint8_t st, std::vector<uint16_t> imu);

   /// \brief write standard file containing IMU configuration
   void write_imu_configuration();

   /// \brief gets current imu linearization configuration
   void get_imu_configuration(int *st, std::vector<uint16_t> *imu);
   /*************************************************************/

   /* Reading Routines */
   /*************************************************************/
   /// \brief function that corrects value in raw_imu_value based
   /// on linearization table
   /// \return - corrected imu value
   int correct_imu();

   /// \brief function that reads and computes imu heading
   int get_heading();
   /*************************************************************/

   
   /* IMU Specific setup functions */
   /* Magnetometer */
   /*************************************************************/
   /// \brief reads whoami register and sets everything up
   /// \return - true on success
   bool init_magnetometer();
   /// \brief sets magnetometer gain
   /// \param gain - gain to be set
   void set_magnetometer_gain(Ada10Dof_MagGain gain);
   /// \brief sets magnetometer update rate 
   /// \param rate - update rate in Hz
   void set_magnetometer_rate(Ada10Dof_MagRate rate);
   /// \brief reads magnetometer Z component
   /// \return - value read in Z component of magnetometer
   float read_magnetometer_z();
   /*************************************************************/       
   /* Accelerometer */
   /*************************************************************/ 
   /// \brief reads whoami register and sets everything up
   /// \return - true on success
   bool init_accelerometer();  
   /// \brief sets accelerometer sensing scale
   /// \param scale - scale to be set 
   void set_accelerometer_scale(Ada10Dof_AccelScale scale);
   /// \brief sets magnetometer update rate 
   /// \param rate - update rate in Hz
   void set_accelerometer_rate(Ada10Dof_AccelRate rate);
   /// \brief reads accelerometer X and Y component
   /// \param pitch - x component read from the accelerometer
   /// \param roll - y component read from the accelerometer   
   void read_accelerometer(float *pitch, float *roll);
   /*************************************************************/ 



   private:
   /* I2C Bus communication variables */
   /*************************************************************/ 
   /// \brief I2C bus file descriptor
   int i2c_fd;
   /// \brief boolean function to control initialization of the 
   /// file descriptor
   bool init;
   /// \brief file name of the I2C bus
   std::string device_name;
   /*************************************************************/ 
   
   /* Configuration and Parameters */
   /*************************************************************/
   /// \brief file path for configuration file
   std::string filepath;   
   float mag_gauss_xy;
   float mag_gauss_z;
   float accel_g_lsb;
   /*************************************************************/

   /* Linearization */
   /*************************************************************/ 
   /// \brief step between linearization points   
   uint8_t step;   
   /// \brief vectors to hold imu and real values for the linearization
   std::vector<uint16_t> imu_values,real_values;
   /// \brief pre-computed slope and base values for linearization
   std::vector<double> b,m;
   /// \brief raw value from IMU readings
   float raw_imu_value;
   /// \brief value representing reference 0º
   float alfa;
   /*************************************************************/ 


   /* I2C Bus communication Functions
   /*************************************************************/ 
   /// \brief function to use I2C bus to read a byte from the
   /// specified register. 
   /// \param address - one of Ada10Dof I2C 7bit addresses
   /// \param command - register to be read
   /// \return - returns -1 on failure. returns read byte on success
   int i2cRequestByte(uint8_t address, uint8_t command);

   /// \brief function to use I2C bus to read a word from the
   /// specified register. 
   /// \param address - one of Ada10Dof I2C 7bit addresses
   /// \param command - register to be read
   /// \return - returns -1 on failure. returns read word on success
   int i2cRequestWord(uint8_t address, uint8_t command);
   
   /// \brief function to use I2C bus to read multiple bytes from the
   /// specified register. 
   /// \param address - one of Ada10Dof I2C 7bit addresses
   /// \param command - register to be read
   /// \param length - amount of bytes to be read     
   /// \param values - pointer to vector of bytes to store read data
   /// \return - returns -1 on failure. returns length on success
   int i2cRequestData(uint8_t address, uint8_t command, uint8_t length, uint8_t* values);

   /// \brief function to use I2C bus to write multiple bytes from the
   /// specified register. 
   /// \param address - one of Ada10Dof I2C 7bit addresses
   /// \param command - register to be written
   /// \param length - amount of bytes to write    
   /// \param values - pointer to vector of bytes where data is stored
   /// \return - returns -1 on failure. returns length on success
   int i2cSendData(uint8_t address, uint8_t command, uint8_t length, uint8_t* values);
   /*************************************************************/ 
};
#endif
