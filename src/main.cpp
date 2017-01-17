#include <iostream>
#include "rttimer.h"
#include "mcp3k8.h"
#include "Omni3MD.h"
#include "ros/ros.h"

// ##### ROS MSG INCLUDES #####
// ############################
#include "minho_team_ros/hardwareInfo.h"
#include "minho_team_ros/controlInfo.h"
#include "minho_team_ros/teleop.h"
#include "minho_team_ros/imuConfig.h"
#include "minho_team_ros/omniConfig.h"
// ############################

// ##### ROS SRV INCLUDES #####
// ############################
#include "minho_team_ros/requestResetEncoders.h"
#include "minho_team_ros/requestResetIMU.h"
#include "minho_team_ros/requestIMULinTable.h"
#include "minho_team_ros/requestOmniProps.h"
#include "minho_team_ros/requestKick.h"
// ############################

using minho_team_ros::hardwareInfo;
using minho_team_ros::controlInfo;
using minho_team_ros::teleop;
using minho_team_ros::requestResetEncoders;
using minho_team_ros::requestResetIMU;
using minho_team_ros::requestOmniProps;
using minho_team_ros::requestIMULinTable;
using minho_team_ros::requestKick;

/// \brief threads to run the different tasks
pthread_t th_enc, th_bat, th_imu;
/// \brief periodic_info structs to time the differents
/// threads
periodic_info pi_enc, pi_bat, pi_imu;

/// \brief hardwareInfo message where data is stored
hardwareInfo hw;
/// \brief boolean variable to store state of teleop
bool teleop_active = false;
/// \brief mutex to protect access to hw
pthread_mutex_t hw_mutex = PTHREAD_MUTEX_INITIALIZER;
/// \brief mutex to protect access to i2c bus
pthread_mutex_t i2c_mutex = PTHREAD_MUTEX_INITIALIZER;

/// \brief timestamps to implement safety timeout
struct timeval t1,t0;

/// \brief publisher of hardwareInfo
ros::Publisher hw_pub;

/// \brief subscriber of controlInfo
ros::Subscriber control_sub;

/// \brief subscriber of teleop
ros::Subscriber teleop_sub;


/// \brief Omni3MD object to communicate through I2C bus
/// with omni motor controller
Omni3MD omni(0x30);

/// \brief MCP3008 object to communicate through SPI bus
/// with 8-channel 10bit ADC
MCP3k8 adc(SPI_DEV_0,1000000,SPI_MODE_0,8,5.0);

/// \brief sets up the task's threads without running them
void setup_threads();
/// \brief thread function that reads Omni3MD's 
/// encoders and battery.
/// \param per_info - pointer to periodic_info to use 
void *readEncoders(void *per_info);
/// \brief thread function that reads and
/// computes IMU heading.
/// \param per_info - pointer to periodic_info to use 
void *readIMU(void *per_info);
/// \brief thread function that reads other 
/// batteries and will be used to read future things
/// \param per_info - pointer to periodic_info to use 
void *readBatteries(void *per_info);

/// \brief controlInfo callback, containing action
/// commands
/// \param msg - message containing contronInfo
void controlInfoCallback(const controlInfo::ConstPtr &msg);
/// \brief teleop callback, containing teleop state
/// \param msg - message containing contronInfo
void teleopCallback(const teleop::ConstPtr &msg);

int main(int argc, char**argv)
{
   ROS_WARN("Attempting to start PiHw services of pihw_node.");
   ros::init(argc, argv, "pihw_node" ,ros::init_options::NoSigintHandler);
   
   ros::NodeHandle pihw_node;
   
   // Setup subscribers, publishers and services
   hw_pub = pihw_node.advertise<hardwareInfo>("/hardwareInfo",1);
   control_sub = pihw_node.subscribe("/controlInfo",1,controlInfoCallback);
   teleop_sub = pihw_node.subscribe("/teleop",1,teleopCallback);
   gettimeofday(&t1,0);
   t0 = t1;

   // Start node
   ROS_WARN("MinhoTeam pihw_node started running on ROS.");
	ros::AsyncSpinner spinner(2);
   pthread_create(&th_enc, NULL, readEncoders, &pi_enc);
   pthread_create(&th_bat, NULL, readIMU, &pi_bat);
   pthread_create(&th_imu, NULL, readBatteries, &pi_imu);
	spinner.start();

   pthread_join(th_enc, NULL);
   pthread_join(th_bat, NULL);
   pthread_join(th_imu, NULL);
   spinner.stop();
	ros::shutdown();
   ROS_ERROR("Exited pihw_node.");
   return 0;
}

void setup_threads()
{
   pi_enc.id = 1; pi_enc.period_us = 25000; //25ms @ 40Hz  
   pi_bat.id = 2; pi_bat.period_us = 100000; //0.1s @ 10Hz 
   pi_imu.id = 3; pi_imu.period_us = 25000; //25ms @ 40Hz 
}

void *readEncoders(void *per_info)
{
   periodic_info *info = (periodic_info *)(per_info);
   make_periodic(info->period_us,info);

   while(1){
      pthread_mutex_lock(&hw_mutex);
      //read encoders
      pthread_mutex_lock(&i2c_mutex);
      omni.read_encoders((int*)&hw.encoder_1,(int*)&hw.encoder_2,(int*)&hw.encoder_3);
      pthread_mutex_unlock(&i2c_mutex);
      pthread_mutex_unlock(&hw_mutex);

      pthread_mutex_lock(&hw_mutex);
      //read battery
      pthread_mutex_lock(&i2c_mutex);
      hw.battery_main = omni.read_battery();
      pthread_mutex_unlock(&i2c_mutex);
      //publish info
      hw_pub.publish(hw);
      pthread_mutex_unlock(&hw_mutex);
      ROS_INFO("Encoders read");      
      wait_period(info);
   }
}

void *readIMU(void *per_info)
{
   periodic_info *info = (periodic_info *)(per_info);
   make_periodic(info->period_us,info);

   while(ros::ok()){
      ROS_INFO("IMU read");
      wait_period(info);
   }
}

void *readBatteries(void *per_info)
{
   periodic_info *info = (periodic_info *)(per_info);
   make_periodic(info->period_us,info);

   while(ros::ok()){
      pthread_mutex_lock(&hw_mutex);     
      //read channel 1 (pc_bat)
      hw.battery_pc = adc.readChannel(0);
      pthread_mutex_unlock(&hw_mutex);

      pthread_mutex_lock(&hw_mutex);
      //read channel 1 (pc_bat)
      hw.battery_camera = adc.readChannel(1);
      pthread_mutex_unlock(&hw_mutex);

      pthread_mutex_lock(&hw_mutex);
      //read channel 3 (free_wheel)
      hw.free_wheel_activated = adc.readChannel(2);
      pthread_mutex_unlock(&hw_mutex);
 
      //Grabber readings will go here

      // Watch dog for timeout
      gettimeofday(&t1,0);
      float elapsed = (float)(t1.tv_sec-t0.tv_sec)*1000.0 + (float)(t1.tv_usec-t0.tv_usec)/1000.0;
      //100ms timeout
      if(elapsed>80) {
         pthread_mutex_lock(&hw_mutex);
         pthread_mutex_lock(&i2c_mutex);
         omni.stop_motors();
         ROS_ERROR("Timeout reached ... Stopping motors.");
         pthread_mutex_unlock(&i2c_mutex);
         pthread_mutex_unlock(&hw_mutex);
      }
      t0 = t1;

      ROS_INFO("Bat read %.2f",elapsed);
      wait_period(info);
   }
}

void controlInfoCallback(const controlInfo::ConstPtr &msg)
{
   if((teleop_active && msg->is_teleop)||(!teleop_active && !msg->is_teleop)){

      int movement_dir = 360-msg->movement_direction; 
      
      pthread_mutex_lock(&hw_mutex); 
      pthread_mutex_lock(&i2c_mutex);
      if(!hw.free_wheel_activated) omni.mov_omni(msg->linear_velocity, msg->angular_velocity, movement_dir);
      else omni.stop_motors();
      pthread_mutex_unlock(&hw_mutex);
      pthread_mutex_unlock(&i2c_mutex);
      
      gettimeofday(&t0,0);
      //Grabber actuaction will go here 
   }     
}

void teleopCallback(const teleop::ConstPtr &msg)
{
   teleop_active = msg->set_teleop;   
}
