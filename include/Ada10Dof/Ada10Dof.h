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

using namespace rapidjson;
using namespace std;

#define CONFIG_FILE "/.mtconfigs/.ada10dof.cfg"

class Ada10Dof
{
   public:
   /// \brief class constructor. Assigns desired address to I2C 
   /// file descriptor, it should match device's address. Also
   /// performs the connection to the device.
   /// \param adaAddress - Ada10Dof I2C 7bit address (default is 0x18)
   Ada10Dof(uint8_t adaAddress);

   /* Setup Routines */
   /*************************************************************/
   /// \brief function to connect to the I2C bus and to the desired
   /// device.
   /// \param adaAddress - Ada10Dof I2C 7bit address (default is 0x18)
   /// \return - returns -1 on error.
   int i2c_connect(uint8_t adaAddress);

   /// \brief function to set Ada10Dof's I2C address to be used in 
   /// further I2C bus communications.
   /// \return - returns -1 on error.
   int i2c_start_transmission();

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


   private:
   /// \brief I2C bus file descriptor
   int i2c_fd;
   /// \brief boolean function to control initialization of the 
   /// file descriptor
   bool init;
   /// \brief file name of the I2C bus
   std::string device_name;
   /// \brief defined I2C address for Ada10Dof
   uint8_t i2c_slave_address;
   /// \brief raw value from IMU readings
   float raw_imu_value;
   /// \brief value representing reference 0º
   float alfa;
   /// \brief file path for configuration file
   std::string filepath;

   /* Linearization */
   /// \brief step between linearization points   
   uint8_t step;   
   /// \brief vectors to hold imu and real values for the linearization
   std::vector<uint16_t> imu_values,real_values;
   /// \brief pre-computed slope and base values for linearization
   std::vector<double> b,m;

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
};
#endif
