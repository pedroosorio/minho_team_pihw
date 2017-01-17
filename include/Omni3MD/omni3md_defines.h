/* This header the I2C commands to be sent to Omni3MD
*/

#include <stdint.h>
#include <linux/i2c-dev.h>

#define KEY1 0xAA // key used in critical commands
#define KEY2 0x55 // key used in critical commands

/*Read Firmware version*/
/*********************************************************/
#define COMMAND_FIRMWARE_INT 0xFE //Read firmware value (integer value)
#define COMMAND_FIRMWARE_DEC 0xFD //Read firmware value (decimal value)
/*********************************************************

/*Write Commands -> Don't require response from Omni3MD */
/*********************************************************/
#define COMMAND_STOP 0xFC //Stop motors
#define COMMAND_MOV_OMNI 0xFA //Omnidirectional movement
/* All linear command movements are independent movement */
#define COMMAND_MOV_LIN3M_PID 0xF8 //Linear motion of 3 motors with PID
#define COMMAND_MOV_LIN3M_NOPID 0xF7 //Linear motion of 3 motors wout PID 
#define COMMAND_MOV_LIN1M_PID 0xF6 //Linear motion of 1 motor with PID
#define COMMAND_MOV_LIN1M_NOPID 0xF5 //Linear motion of 1 motor wout PID
#define COMMAND_GAIN_CFG 0xF4 //Configuration of kp, ki and kd for PID
#define COMMAND_CALIBRATE 0xFB //Calibrate
    
#define COMMAND_I2C_ADD 0xF2 //Change Omni3MD I2C address
#define COMMAND_TIMEOUT_I2C 0xF1 //Config of protection timeout
#define COMMAND_PRESCALER_CFG 0xF0 //Set the encoders prescaler values
#define COMMAND_POS_ENC_PRESET 0xEF //Preset/reset the encoders count
#define COMMAND_MOV_POS 0xEE //Positional movement
#define COMMAND_SAVE_POS 0xED //Save the encoders state to Omni3MD eeprom
#define COMMAND_MOV_DIF_SI 0xEB //Differential movement with SI units
#define COMMAND_DIF_SI_CFG 0xEA //Config of the differential with SI units
#define COMMAND_RAMP_CFG 0xEC //Configuration of the acceleration ramp and limiar take off parameter
/*********************************************************/

/*Read Commands -> requests to Omni3MD */
/*********************************************************/
#define COMMAND_ENC1_INC 0xD8 //Read encoder1 positional value
#define COMMAND_ENC2_INC 0xD7 //Read encoder2 positional value
#define COMMAND_ENC3_INC 0xD6 //Read encoder3 positional value
#define COMMAND_BAT 0xD5 //Read battery voltage
#define COMMAND_TEMP 0xD4 //Read Omni3MD temperature
#define COMMAND_CTRL_RATE 0xD3 //Read the PID control rate
#define COMMAND_ENC1_MAX 0xD2 //Read encoder1 maximum value at calibration
#define COMMAND_ENC2_MAX 0xD1 //Read encoder1 maximum value at calibration
#define COMMAND_ENC3_MAX 0xD0 //Read encoder1 maximum value at calibration
/*********************************************************/

#define DEFAULT_OMNI_ADDRESS 0x18 // Default 7bit address
