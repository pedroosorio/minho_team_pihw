#include <iostream>
#include "utilities/rttimer.h"
#include "MCP3k8/mcp3k8.h"
#include "Omni3MD/Omni3MD.h"
#include "Ada10Dof/Ada10Dof.h"
#include "ros/ros.h"
#include "utilities/hwdefines.h"
#include "utilities/utils.h"
#include <unistd.h>
#include "Alarm/alarm.h"
#include "thPool/thpool.h"

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
#include "minho_team_ros/requestResetIMU.h"
#include "minho_team_ros/requestIMULinTable.h"
#include "minho_team_ros/requestOmniProps.h"
#include "minho_team_ros/requestKick.h"
// ############################

using minho_team_ros::hardwareInfo;
using minho_team_ros::controlInfo;
using minho_team_ros::teleop;
using minho_team_ros::requestResetIMU;
using minho_team_ros::requestOmniProps;
using minho_team_ros::requestIMULinTable;
using minho_team_ros::requestKick;

/*                    VARIABLES                    */
/* *********************************************** */

/*     THREADS     */
/* *************** */
/// \brief threads to run the different tasks
pthread_t th_enc, th_bat, th_imu;
/// \brief periodic_info structs to time the differents
/// threads
periodic_info pi_enc, pi_bat, pi_imu;
/// \brief thread pool to allow dynamic task assignment
/// for alarms and other tasks
threadpool thread_pool;
bool need_alrm_main = true, need_alrm_pc = true;
bool need_alrm_cam = true;
/// \brief mutex to protect access to hw
pthread_mutex_t hw_mutex = PTHREAD_MUTEX_INITIALIZER;
/// \brief mutex to protect access to i2c bus
pthread_mutex_t i2c_mutex = PTHREAD_MUTEX_INITIALIZER;
/* *************** */

/*     ROS DATA     */
/* **************** */
/// \brief hardwareInfo message where data is stored
hardwareInfo hw;
/// \brief boolean variable to store state of teleop
bool teleop_active = false;
/// \brief publisher of hardwareInfo
ros::Publisher hw_pub;
/// \brief subscriber of controlInfo
ros::Subscriber control_sub;
/// \brief subscriber of teleop
ros::Subscriber teleop_sub;
/// \brief service to reset imu reference value (0º)
ros::ServiceServer reset_imu_sv;
/// \brief service to set/get Omni3MD properties
ros::ServiceServer omni_prop_sv;
/// \brief service to set/get IMU linearization table  
ros::ServiceServer imu_table_sv;
/// \brief service to trigger kicking mechanism
ros::ServiceServer kick_sv;
/* **************** */


/*     RESOURCESS     */
/* ****************** */
/// \brief Omni3MD object to communicate through I2C bus
/// with omni motor controller
Omni3MD omni(0x18,&i2c_mutex);
/// \brief Ada10Dof object to communicate through I2C bus
/// with IMU
Ada10Dof imu;
/// \brief MCP3008 object to communicate through SPI bus
/// with 8-channel 10bit ADC
MCP3k8 adc(SPI_DEV_0,1000000,SPI_MODE_0,8,5.0);
/* ****************** */

/* *********************************************** */

/*                    FUNCTIONS                    */
/* *********************************************** */

/*     SETUP FUNCTIONS     */
/* *********************** */
/// \brief sets up the task's threads without running them
void setup_threads();
/// \brief sets up Omni3MD initial values
void setup_omni();
/// \brief sets up Kick GPIO
void setup_kick();
/* *********************** */

/*     WORKER FUNCTIONS     */
/* ************************ */
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
/// \brief instanciates a thread to play an alarm
/// in the Buzzer
/// \param type - type of alarm to be thrown
void play_alarm(ALARM type);
/// \brief function to end pulse for the kick pin
void halt_pulse(void *time);
/* *********************** */


/*     ROS CALLBACKS     */
/* ********************* */
/// \brief controlInfo callback, containing action
/// commands
/// \param msg - message containing contronInfo
void controlInfoCallback(const controlInfo::ConstPtr &msg);
/// \brief teleop callback, containing teleop state
/// \param msg - message containing contronInfo
void teleopCallback(const teleop::ConstPtr &msg);
/// \brief service callback for requestResetIMU to reset imu geo-0º
/// reference.
/// \param req - empty data
/// \param res - empty data
bool resetIMUReferenceService(requestResetIMU::Request &req, requestResetIMU::Response &res);
/// \brief service callback for requestOmniProps service, in order to
/// set or get the configuration for the Omni3MD control parameters
/// \param req - request data, containing the new configuration and a
/// boolean variable to flag if the operation is a set or get
/// \param res . response data containing the current configuration 
/// of the PID and BNM values
bool OmniPropsService(requestOmniProps::Request &req, requestOmniProps::Response &res);
/// \brief service callback for requestIMUTable service, in order to
/// set or get the configuration for the imu linearization table
/// \param req - request data, containing the new configuration and a
/// boolean variable to flag if the operation is a set or get
/// \param res - response data containing the current configuration 
/// of the IMU linearization table
bool IMUTableService(requestIMULinTable::Request &req, requestIMULinTable::Response &res);
/// \brief function to actuate kicker, in order to kick the ball. 
/// Only kicks if the robot detects that has the ball inside
/// \param req - request data received in requestKick service
/// \param res - response data, flaggin if the kick was taken or not
bool kickService(requestKick::Request &req, requestKick::Response &res);

/* ********************* */

/* *********************************************** */

int main(int argc, char**argv)
{
   ROS_WARN("Attempting to start PiHw services of pihw_node.");
   
   std::string iface_ip = getInterfaceIp("eth0");
   ROS_WARN("Current interface ip: %s",iface_ip.c_str());  
   std::size_t found = iface_ip.find_last_of(".");
   std::string master_ip = "http://";
   master_ip += iface_ip.substr(0,found);
   master_ip += ".1:11311";
   ROS_INFO("Setting ROS_MASTER_URI as %s", master_ip.c_str());
   setenv("ROS_MASTER_URI",master_ip.c_str(),1);
   
   ros::init(argc, argv, "pihw_node" ,ros::init_options::NoSigintHandler);
   setup_threads();   
   setup_omni();
   setup_alarm();
   setup_kick();
   ros::NodeHandle pihw_node;
   
   // Setup subscribers, publishers and services
   hw_pub = pihw_node.advertise<hardwareInfo>("/hardwareInfo",1);
   control_sub = pihw_node.subscribe("/controlInfo",1,controlInfoCallback);
   teleop_sub = pihw_node.subscribe("/teleop",1,teleopCallback);
   
   reset_imu_sv = pihw_node.advertiseService("requestResetIMU",
                            resetIMUReferenceService);  
   omni_prop_sv = pihw_node.advertiseService("requestOmniProps",
                            OmniPropsService);
   imu_table_sv = pihw_node.advertiseService("requestIMULinTable",
                            IMUTableService);
   kick_sv = pihw_node.advertiseService("requestKick",
                            kickService);

   // Start node
   ROS_WARN("MinhoTeam pihw_node started running on ROS.");
   
   /* Play ready sound */
   throw_alarm_ready();   
   
   ros::AsyncSpinner spinner(2);
   pthread_create(&th_enc, NULL, readEncoders, &pi_enc);
   pthread_create(&th_bat, NULL, readBatteries, &pi_bat);
   pthread_create(&th_imu, NULL, readIMU, &pi_imu);
   thread_pool = thpool_init(4); // One thread per alarm + kick
	spinner.start();

   pthread_join(th_enc, NULL);
   pthread_join(th_bat, NULL);
   pthread_join(th_imu, NULL);
   thpool_wait(thread_pool);
   thpool_destroy(thread_pool);
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

void setup_omni()
{
   uint8_t f1,f2,f3;
   omni.read_firmware(&f1,&f2,&f3);
   ROS_INFO("Omni3MD: \n\t\t\t\tCtrl Rate %d | Firmware v%d.%d.%d |\n\t\t\t\tTemperature %.2f | Encoder Max %d |\n\t\t\t\tBattery %.2f |", omni.read_control_rate(), f1,f2,f3, omni.read_temperature(), omni.read_enc1_max(),omni.read_battery());

   omni.set_i2c_timeout(10);
}

void setup_kick()
{
   pinMode(KICK_PIN, OUTPUT);
   digitalWrite(KICK_PIN, LOW);
}

void *readEncoders(void *per_info)
{
   periodic_info *info = (periodic_info *)(per_info);
   make_periodic(info->period_us,info);

   while(1){
      pthread_mutex_lock(&hw_mutex);
      //read encoders
      pthread_mutex_lock(&i2c_mutex);
//      omni.read_encoders(&hw.encoder_1,&hw.encoder_2,&hw.encoder_3);
      hw.encoder_1 = omni.read_enc1();
      hw.encoder_2 = omni.read_enc2();
      hw.encoder_3 = omni.read_enc3();
  
//      int16_t a,b,c; float d,e;
//      omni.read_mov_data(&a,&b,&c,&d,&e);
//      printf("%d %d %d %.2f %.2f\n",a,b,c,d,e);

      pthread_mutex_unlock(&i2c_mutex);
      pthread_mutex_unlock(&hw_mutex);

      pthread_mutex_lock(&hw_mutex);
      //read battery
      pthread_mutex_lock(&i2c_mutex);
      hw.battery_main = omni.read_battery()+MAIN_OFF_VOLT;
      pthread_mutex_unlock(&i2c_mutex);
      //publish info
      hw_pub.publish(hw);
      pthread_mutex_unlock(&hw_mutex);
      wait_period(info);
   }
}

void *readIMU(void *per_info)
{
   periodic_info *info = (periodic_info *)(per_info);
   make_periodic(info->period_us,info);

   while(1){

      pthread_mutex_lock(&hw_mutex); 
      pthread_mutex_lock(&i2c_mutex);
      hw.imu_value = imu.get_heading();  
      pthread_mutex_unlock(&i2c_mutex);
      pthread_mutex_unlock(&hw_mutex);

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
      hw.battery_pc = adc.readChannel(0,PC_REF_VOLT);
      hw.battery_pc += PC_OFF_VOLT;
      if(hw.battery_pc>2.5 && hw.battery_pc<PC_CRIT_VOLT) {
         if(need_alrm_pc){
            need_alrm_pc = false;
            play_alarm(PC);
         }
      }else need_alrm_pc = true;
      pthread_mutex_unlock(&hw_mutex);

      pthread_mutex_lock(&hw_mutex);
      //read channel 1 (pc_bat)
      hw.battery_camera = adc.readChannel(1,CAM_REF_VOLT);
      hw.battery_camera += CAM_OFF_VOLT;
      if(hw.battery_camera>2.5 && hw.battery_camera<CAM_CRIT_VOLT){ 
         if(need_alrm_cam){ 
            need_alrm_cam = false;
            play_alarm(CAM);
         }
      }else need_alrm_cam = true;
      pthread_mutex_unlock(&hw_mutex);

      pthread_mutex_lock(&hw_mutex);
      //read channel 3 (free_wheel)
      int freewheel = adc.readChannel(2,5.0);
      bool old_freewheel = hw.free_wheel_activated;
      if(freewheel>2.5) hw.free_wheel_activated = true;
      else hw.free_wheel_activated = false;

      if(old_freewheel!=hw.free_wheel_activated){
         if(hw.free_wheel_activated) play_alarm(FW_ON);
         else play_alarm(FW_OFF);
      }
      pthread_mutex_unlock(&hw_mutex);      

      pthread_mutex_lock(&hw_mutex);
      hw.battery_main += MAIN_OFF_VOLT;
      if(hw.battery_main>2.5 && hw.battery_main<MAIN_CRIT_VOLT){ 
         if(need_alrm_main){ 
            need_alrm_main = false;
            play_alarm(MAIN);
         }   
      }else need_alrm_main = true;
      pthread_mutex_unlock(&hw_mutex);

      //Grabber readings will go here

      wait_period(info);
   }
}

void play_alarm(ALARM type)
{
   ALARM *alarm = new ALARM;
   *alarm = type;
   thpool_add_work(thread_pool, throw_alarm, alarm);  
}

void halt_pulse(void *time)
{
   int *delay = (int *)time;
   usleep(*delay);
   digitalWrite(KICK_PIN,LOW);   
   delete((int *)time);
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

      //Grabber actuaction will go here 
   }     
}

void teleopCallback(const teleop::ConstPtr &msg)
{
   teleop_active = msg->set_teleop;
   if(teleop_active) play_alarm(TELE_ON);
   else play_alarm(TELE_OFF);
}

bool resetIMUReferenceService(requestResetIMU::Request &req, requestResetIMU::Response &res)
{
   imu.set_imu_reference();
   return true; 
}

bool OmniPropsService(requestOmniProps::Request &req, requestOmniProps::Response &res)
{
   return true; 
}

bool IMUTableService(requestIMULinTable::Request &req, requestIMULinTable::Response &res)
{
   if(req.is_set){
      if(imu.set_imu_configuration(req.imuConf.step,req.imuConf.imu_values)){
         ROS_INFO("_ADA10DOF: New parameters for linearization table set.");
         res.imuConf = req.imuConf;    
      } else ROS_ERROR("_ADA10DOF: Wrong parameters for linearization table.");
   } else {
      // read values in imu
      int step; std::vector<uint16_t> imu_values;
      imu.get_imu_configuration(&step,&imu_values);
      res.imuConf.step = step;
      res.imuConf.imu_values = imu_values;  
   }
   
   return true; 
}

bool kickService(requestKick::Request &req, requestKick::Response &res)
{
   int maxTime = KICK_MAX_DURATION;
   int conversion = KICK_STR_CONV;
   if(req.kick_is_pass) { 
      maxTime = PASS_MAX_DURATION;
      conversion = PASS_STR_CONV;
   }

   int kickTime = req.kick_strength*conversion;
   //Todo replace variable ball_sensor with something from the
   // grabbers' feedback
   if(kickTime>0 /*&& hwinfo_msg.ball_sensor==1*/){
      if(kickTime>maxTime)kickTime = maxTime;
      
      int *pulseWidth = new int;
      (*pulseWidth) = kickTime;
      digitalWrite(KICK_PIN, HIGH);
      thpool_add_work(thread_pool, halt_pulse, pulseWidth);
      res.kicked = true; 

   } else res.kicked = false;    

   return true;  
}

