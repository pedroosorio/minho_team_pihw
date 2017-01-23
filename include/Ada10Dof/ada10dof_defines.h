/* This header the I2C commands to be sent to Adafruit10DOF IMU
*/
#define _USE_MATH_DEFINES
#include <cmath>
#include <stdint.h>
#include <linux/i2c-dev.h>

#define GRAVITY_EARTH 9.80665F
#define GAUSS_TO_MICROTESLA 100.0F
#define RADTODEG 180.0/M_PI

/* LSM303DLHC ACCELEROMETER + MAGNETOMETER */
/**************************************************************************************/
/**************************************************************************************/

/* I2C ADDRESSES */
#define ACCEL_ADDRESS 0x19 // Default 7bit address
#define MAG_ADDRESS 0x1E // Default 7bit address

/* ACCEL REGISTERS */
/**********************************************************/
typedef enum
{                                                     // DEFAULT    TYPE
   REGISTER_ACCEL_CTRL_REG1_A         = 0x20,   // 00000111   rw
   REGISTER_ACCEL_CTRL_REG2_A         = 0x21,   // 00000000   rw
   REGISTER_ACCEL_CTRL_REG3_A         = 0x22,   // 00000000   rw
   REGISTER_ACCEL_CTRL_REG4_A         = 0x23,   // 00000000   rw
   REGISTER_ACCEL_CTRL_REG5_A         = 0x24,   // 00000000   rw
   REGISTER_ACCEL_CTRL_REG6_A         = 0x25,   // 00000000   rw
   REGISTER_ACCEL_REFERENCE_A         = 0x26,   // 00000000   r
   REGISTER_ACCEL_STATUS_REG_A        = 0x27,   // 00000000   r
   REGISTER_ACCEL_OUT_X_L_A           = 0x28,
   REGISTER_ACCEL_OUT_X_H_A           = 0x29,
   REGISTER_ACCEL_OUT_Y_L_A           = 0x2A,
   REGISTER_ACCEL_OUT_Y_H_A           = 0x2B,
   REGISTER_ACCEL_OUT_Z_L_A           = 0x2C,
   REGISTER_ACCEL_OUT_Z_H_A           = 0x2D,
   REGISTER_ACCEL_FIFO_CTRL_REG_A     = 0x2E,
   REGISTER_ACCEL_FIFO_SRC_REG_A      = 0x2F,
   REGISTER_ACCEL_INT1_CFG_A          = 0x30,
   REGISTER_ACCEL_INT1_SOURCE_A       = 0x31,
   REGISTER_ACCEL_INT1_THS_A          = 0x32,
   REGISTER_ACCEL_INT1_DURATION_A     = 0x33,
   REGISTER_ACCEL_INT2_CFG_A          = 0x34,
   REGISTER_ACCEL_INT2_SOURCE_A       = 0x35,
   REGISTER_ACCEL_INT2_THS_A          = 0x36,
   REGISTER_ACCEL_INT2_DURATION_A     = 0x37,
   REGISTER_ACCEL_CLICK_CFG_A         = 0x38,
   REGISTER_ACCEL_CLICK_SRC_A         = 0x39,
   REGISTER_ACCEL_CLICK_THS_A         = 0x3A,
   REGISTER_ACCEL_TIME_LIMIT_A        = 0x3B,
   REGISTER_ACCEL_TIME_LATENCY_A      = 0x3C,
   REGISTER_ACCEL_TIME_WINDOW_A       = 0x3D
} Ada10Dof_AccelReg;

typedef enum
{
   ACCEL_SCALE_2G = 0x00,
   ACCEL_SCALE_4G = 0x10,
   ACCEL_SCALE_8G = 0x30
} Ada10Dof_AccelScale;

typedef enum
{
   ACCEL_RATE_50 = 0x27,
   ACCEL_RATE_100 = 0x2F,
   ACCEL_RATE_400 = 0x37,
   ACCEL_RATE_1000 = 0x3F,
} Ada10Dof_AccelRate;
/**********************************************************/
/**********************************************************/

/* MAG REGISTERS */
/**********************************************************/
typedef enum
{
   REGISTER_MAG_CRA_REG_M             = 0x00,
   REGISTER_MAG_CRB_REG_M             = 0x01,
   REGISTER_MAG_MR_REG_M              = 0x02,
   REGISTER_MAG_OUT_X_H_M             = 0x03,
   REGISTER_MAG_OUT_X_L_M             = 0x04,
   REGISTER_MAG_OUT_Z_H_M             = 0x05,
   REGISTER_MAG_OUT_Z_L_M             = 0x06,
   REGISTER_MAG_OUT_Y_H_M             = 0x07,
   REGISTER_MAG_OUT_Y_L_M             = 0x08,
   REGISTER_MAG_SR_REG_Mg             = 0x09,
   REGISTER_MAG_IRA_REG_M             = 0x0A,
   REGISTER_MAG_IRB_REG_M             = 0x0B,
   REGISTER_MAG_IRC_REG_M             = 0x0C,
   REGISTER_MAG_TEMP_OUT_H_M          = 0x31,
   REGISTER_MAG_TEMP_OUT_L_M          = 0x32
} Ada10Dof_MagReg;
/**********************************************************/


/* MAG GAIN SETTINGS */
/**********************************************************/
typedef enum
{
   MAGGAIN_1_3                        = 0x20,  // +/- 1.3
   MAGGAIN_1_9                        = 0x40,  // +/- 1.9
   MAGGAIN_2_5                        = 0x60,  // +/- 2.5
   MAGGAIN_4_0                        = 0x80,  // +/- 4.0
   MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
   MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
   MAGGAIN_8_1                        = 0xE0   // +/- 8.1
} Ada10Dof_MagGain;
/**********************************************************/

/* MAG UPDATE RATE SETTINGS */
/**********************************************************/
typedef enum
{
   MAGRATE_0_7                        = 0x00,  // 0.75 Hz
   MAGRATE_1_5                        = 0x01,  // 1.5 Hz
   MAGRATE_3_0                        = 0x02,  // 3.0 Hz
   MAGRATE_7_5                        = 0x03,  // 7.5 Hz
   MAGRATE_15                         = 0x04,  // 15 Hz
   MAGRATE_30                         = 0x05,  // 30 Hz
   MAGRATE_75                         = 0x06,  // 75 Hz
   MAGRATE_220                        = 0x07   // 200 Hz
} Ada10Dof_MagRate;
/**********************************************************/

/**************************************************************************************/
/**************************************************************************************/


#define GYRO_ADDRESS 0x6B // Default 7bit address
