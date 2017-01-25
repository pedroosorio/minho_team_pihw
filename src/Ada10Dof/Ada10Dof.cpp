/*
   Ada10Dof.h - Library for interfacing with Adafruit10Dof IMU
   Created by Pedro Osório, January, 2017.
   Released into the public domain.
*/

#include "Ada10Dof/Ada10Dof.h"


/// \brief mutex to protect acess to raw_imu_value
pthread_mutex_t raw_val_mutex = PTHREAD_MUTEX_INITIALIZER;
/// \brief mutex to protect acess to linearization table
pthread_mutex_t lin_mutex = PTHREAD_MUTEX_INITIALIZER;;

Ada10Dof::Ada10Dof()
{
   init = false;
   device_name = "/dev/i2c-1";
   uint8_t address = MAG_ADDRESS;
   if(i2c_connect(address)>0)printf("_ADA10DOF: Initialization Successful.\n");
   else printf("_ADA10DOF: Initialization Failed.\n");
   
   alfa = raw_imu_value = 0;
   b.clear(); m.clear(); imu_values.clear(); real_values.clear();

   pthread_mutex_lock(&lin_mutex);  
   int ret;
   if(!read_imu_configuration(&ret)){
      step = 90;
      for(int i=0;i<=360;i+=step) { real_values.push_back(i); }
      imu_values = real_values;
   } else for(int i=0;i<=360;i+=step) { real_values.push_back(i); }

   for(int i=0;i<real_values.size()-1;i++){
      m.push_back((double)(real_values[i+1]-real_values[i])/(double)(imu_values[i+1]-imu_values[i]));
      b.push_back((double)real_values[i+1]-m[i]*((double)imu_values[i+1]));
   }
   pthread_mutex_unlock(&lin_mutex);
   
   if(ret==0) { //it means there is no config file
       printf("_ADA10DOF: Errors in JSON file (not found/corrupted). A sample one will be written.\n");
       write_imu_configuration();     
   }

   init_magnetometer();
   init_accelerometer();
   init_gyroscope();
   init_kalman_filter();
}

/* Setup Routines */
/*************************************************************/
int Ada10Dof::i2c_connect(uint8_t address)
{
   if(init) { printf("_ADA10DOF: I2C Interface already opened"); return -1; }

   printf("_ADA10DOF: Opening Ada10Dof in %s\n",device_name.c_str());
   /* Open I2C Interface */
	if((i2c_fd = open(device_name.c_str(), O_RDWR)) < 0){
		perror("_ADA10DOF: Couldn't open I2C Interface.");
      return -1;
	}
	
   if (ioctl (i2c_fd, I2C_SLAVE, address) < 0){
      perror("_ADA10DOF: Couldn't access I2C Device.");
      return -1;
   }

   init = true;
   return i2c_fd;
}

int Ada10Dof::i2c_start_transmission(uint8_t address)
{
   if (ioctl (i2c_fd, I2C_SLAVE, address) < 0){
      perror("_ADA10DOF: Couldn't access I2C Device.");
      return -1;
    }
   return i2c_fd;
}

void Ada10Dof::set_imu_reference()
{
   pthread_mutex_lock(&raw_val_mutex);  
   alfa = raw_imu_value;
   write_imu_configuration();
   pthread_mutex_unlock(&raw_val_mutex);
}

bool Ada10Dof::read_imu_configuration(int *ret)
{
   ifstream infile;
   filepath = getenv("HOME");
   (*ret) = 1;
   filepath += CONFIG_FILE;
   infile.open(filepath.c_str());
   if(!infile.is_open()){ printf("_ADA10DOF: Error opening JSON configuration.\n"); (*ret)=0; return false;}

   std::string config((std::istreambuf_iterator<char>(infile)),
                 std::istreambuf_iterator<char>());
   infile.close();
   Document document;
   char *cstr = new char[config.length() + 1];
   strcpy(cstr, config.c_str());
   if (document.ParseInsitu(cstr).HasParseError() || !document.IsObject()){
      printf("_ADA10DOF: Error parsing JSON file.\n");
      (*ret) = 0;
      return false;
   }else {
      if(document["alfa"].IsNumber()) alfa = document["alfa"].GetDouble();
      else { printf("_ADA10DOF: Error in variable \"alfa\".\n"); return false;} 

      if(document["step"].IsNumber()) step = document["step"].GetDouble();
      else { printf("_ADA10DOF: Error in variable \"step\".\n"); return false;} 

      if(document["imu_values"].IsArray()){
         const Value& a = document["imu_values"];
         if((360.0/(float)step) == a.Size()-1){
            for(SizeType i = 0; i < a.Size(); i++) imu_values.push_back(a[i].GetInt());
         } else { printf("_ADA10DOF: Error in variable \"imu_values\".\n"); return false;}
      }else { printf("_ADA10DOF: Error in variable \"imu_values\".\n"); return false;} 
   }
   
   return true;
}

bool Ada10Dof::set_imu_configuration(uint8_t st, std::vector<uint16_t> imu)
{
   pthread_mutex_lock(&lin_mutex);  
   if((360/st) == imu.size()-1){
      if(st!=step) {
         step = st;
         real_values.clear();
         for(int i=0;i<=360;i+=step) { real_values.push_back(i); }
      }

      imu_values.clear();
      imu_values = imu;
      
      for(int i=0;i<real_values.size()-1;i++){
         m.push_back((double)(real_values[i+1]-real_values[i])/(double)(imu_values[i+1]-imu_values[i]));
         b.push_back((double)real_values[i+1]-m[i]*((double)imu_values[i+1]));
      }
      write_imu_configuration();
      pthread_mutex_unlock(&lin_mutex);
      return true;
   }
   pthread_mutex_unlock(&lin_mutex);
   
   return false;
}

void Ada10Dof::write_imu_configuration()
{
   //write new file
   Document document;
   document.SetObject();
   rapidjson::Document::AllocatorType& alloc = document.GetAllocator();
   document.AddMember("alfa", alfa, alloc);
   document.AddMember("step", step, alloc);

   rapidjson::Value imu_vals(rapidjson::kArrayType);
   for(int i=0;i<imu_values.size();i++) imu_vals.PushBack(imu_values[i],alloc);
   document.AddMember("imu_values", imu_vals, alloc);   
   
   ofstream outfile;
   outfile.open(filepath.c_str());
   if(!outfile.is_open()){ 
      printf("_ADA10DOF: Error opening JSON configuration to write. Will create configuration folder and file.\n"); 
      system("mkdir ~/.mtconfigs");
      outfile.open(filepath.c_str());
   }
   
   StringBuffer sb;
   PrettyWriter<StringBuffer> writer(sb);
   document.Accept(writer);
   std::string data(sb.GetString());
   outfile << data;
   outfile.flush();
   outfile.close();
   printf("_ADA10DOF: New configuration file written.\n");

}

void Ada10Dof::get_imu_configuration(int *st, std::vector<uint16_t> *imu)
{
   (*st) = step;
   (*imu) = imu_values;
}

void Ada10Dof::init_kalman_filter()
{
   memset(&kalman,0,sizeof(MTKalmanFilter));
   // Kalman parameters for Z
   kalman.Q.z = 1; kalman.R.z = 0.5;
   kalman.Q.x = 1; kalman.R.x = 1;
   kalman.Q.y = 1; kalman.R.y = 1;
   kalman.lastCovariance.x = kalman.lastCovariance.y = kalman.lastCovariance.z = 0;
   kalman.covariance = kalman.predictedCovariance = kalman.lastCovariance;
}

/*************************************************************/

/* IMU Specific setup functions */
/* Magnetometer */
/*************************************************************/
bool Ada10Dof::init_magnetometer()
{
   // enable magnetometer
   uint8_t data[1] = {0x00};
   i2cSendData(MAG_ADDRESS,REGISTER_MAG_MR_REG_M,1,data);
   
   uint8_t whoami =  i2cRequestByte(MAG_ADDRESS,REGISTER_MAG_CRA_REG_M);
   // 0x10 default reg value for 15Hz. 0x18 reg value for 75Hz.
   if(whoami == 0x10 || whoami == 0x18){
      set_magnetometer_gain(MAGGAIN_1_3);
      set_magnetometer_rate(MAGRATE_75);
      printf("_ADA10DOF: Magnetometer initialized.\n");
      return true;
   } else { 
      mag_gauss_xy = 1100.0;
      mag_gauss_z  = 980.0;
      printf("_ADA10DOF: Failed to connect to Magnetometer.\n"); 
      return false;
   }
}

void Ada10Dof::set_magnetometer_gain(Ada10Dof_MagGain gain)
{
   uint8_t data[1] = {gain};
   i2cSendData(MAG_ADDRESS,REGISTER_MAG_CRB_REG_M,1,data);

   switch(gain)
   {
    case MAGGAIN_1_3:
      mag_gauss_xy = 1100.0;
      mag_gauss_z  = 980.0;
      break;
    case MAGGAIN_1_9:
      mag_gauss_xy = 855.0;
      mag_gauss_z  = 760.0;
      break;
    case MAGGAIN_2_5:
      mag_gauss_xy = 670.0;
      mag_gauss_z  = 600.0;
      break;
    case MAGGAIN_4_0:
      mag_gauss_xy = 450.0;
      mag_gauss_z  = 400.0;
      break;
    case MAGGAIN_4_7:
      mag_gauss_xy = 400.0;
      mag_gauss_z  = 355.0;
      break;
    case MAGGAIN_5_6:
      mag_gauss_xy = 330.0;
      mag_gauss_z  = 295.0;
      break;
    case MAGGAIN_8_1:
      mag_gauss_xy = 230.0;
      mag_gauss_z  = 205.0;
      break;
   }
}

void Ada10Dof::set_magnetometer_rate(Ada10Dof_MagRate rate)
{
   uint8_t data[1] = {static_cast<uint8_t>(((uint8_t)rate&0x07)<<2)};
   i2cSendData(MAG_ADDRESS,REGISTER_MAG_CRA_REG_M,1,data);
}

float Ada10Dof::read_magnetometer_z(bool magtilt)
{
   int16_t mag_x,mag_y,mag_z;
   uint8_t values[6] = {0,0,0,0,0,0};
   i2cRequestData(MAG_ADDRESS, REGISTER_MAG_OUT_X_H_M, 6, values);
   mag_x = (int16_t)((uint16_t)values[1] | ((uint16_t)values[0] << 8));
   mag_z = (int16_t)((uint16_t)values[3] | ((uint16_t)values[2] << 8));
   mag_y = (int16_t)((uint16_t)values[5] | ((uint16_t)values[4] << 8));
   
   if(magtilt){
      float pitch = 0.0, roll = 0.0;
      read_accelerometer(&pitch,&roll); 
      float cR = (float)cos(roll);
      float sR = (float)sin(roll);
      float cP = (float)cos(-1*pitch);
      float sP = (float)sin(-1*pitch);

      /* The tilt compensation algorithm                            */
      /* Xh = X.cosPitch + Z.sinPitch                               */
      /* Yh = X.sinRoll.sinPitch + Y.cosRoll - Z.sinRoll.cosPitch   */
      mag_x = (mag_x)*cP+(mag_z)*sP;
      mag_y = (mag_y)*sR*sP+(mag_y)*cR-(mag_z)*sR*cP;
   }

   return (float)atan2((float)mag_y/mag_gauss_xy*GAUSS_TO_MICROTESLA
                      ,(float)mag_x/mag_gauss_xy*GAUSS_TO_MICROTESLA)*RADTODEG;
}
/*************************************************************/       
/* Accelerometer */
/*************************************************************/ 
bool Ada10Dof::init_accelerometer()
{
   // enable magnetometer
   uint8_t data[1] = {ACCEL_RATE_50}; // 50Hz
   i2cSendData(ACCEL_ADDRESS,REGISTER_ACCEL_CTRL_REG1_A,1,data);
   
   uint8_t whoami =  i2cRequestByte(ACCEL_ADDRESS,REGISTER_ACCEL_CTRL_REG1_A);
   if(whoami == ACCEL_RATE_50){
      set_accelerometer_scale(ACCEL_SCALE_2G);
      set_accelerometer_rate(ACCEL_RATE_50);
      printf("_ADA10DOF: Accelerometer initialized.\n");
      return true;
   } else { 
      accel_g_lsb = 0.001;
      printf("_ADA10DOF: Failed to connect to Accelerometer.\n"); 
      return false;
   }
}

void Ada10Dof::set_accelerometer_scale(Ada10Dof_AccelScale scale)
{
   uint8_t data[1] = {scale};
   i2cSendData(ACCEL_ADDRESS,REGISTER_ACCEL_CTRL_REG4_A,1,data); 

   switch(scale)
   {
    case ACCEL_SCALE_2G:
      accel_g_lsb = 0.001;
      break;
    case ACCEL_SCALE_4G:
      accel_g_lsb = 0.002;
      break;
    case ACCEL_SCALE_8G:
      accel_g_lsb = 0.0039;
      break;
   }   
}

void Ada10Dof::set_accelerometer_rate(Ada10Dof_AccelRate rate)
{
   uint8_t data[1] = {rate};
   i2cSendData(ACCEL_ADDRESS,REGISTER_ACCEL_CTRL_REG1_A,1,data);   
}

void Ada10Dof::read_accelerometer(float *pitch, float *roll)
{
   float acc_x,acc_y,acc_z;
   uint8_t values[6] = {0,0,0,0,0,0};
   i2cRequestData(ACCEL_ADDRESS, REGISTER_ACCEL_OUT_X_L_A|0x80, 6, values);
   acc_x = ((int16_t)(values[0] | (values[1] << 8)) >> 4)*accel_g_lsb*GRAVITY_EARTH;
   acc_y = ((int16_t)(values[2] | (values[3] << 8)) >> 4)*accel_g_lsb*GRAVITY_EARTH;
   acc_z = ((int16_t)(values[4] | (values[5] << 8)) >> 4)*accel_g_lsb*GRAVITY_EARTH;

   float signOfZ = acc_z >= 0 ? 1.0F : -1.0F;
   (*roll) = (float)atan2(acc_y, sqrt(acc_x*acc_x+acc_z*acc_z));
   (*pitch) = (float)atan2(acc_x, signOfZ*sqrt(acc_y*acc_y+acc_z*acc_z));

}
/*************************************************************/ 
/* Gyroscope */
/*************************************************************/
bool Ada10Dof::init_gyroscope()
{
   uint8_t whoami =  i2cRequestByte(GYRO_ADDRESS,GYRO_REGISTER_WHO_AM_I);
   if(whoami == 0xD4 || whoami == 0xD7){
      //enable gyroscope
      uint8_t data[1] = {0x00};
      i2cSendData(GYRO_ADDRESS,GYRO_REGISTER_CTRL_REG1,1,data);
      data[1] = 0x0F;
      i2cSendData(GYRO_ADDRESS,GYRO_REGISTER_CTRL_REG1,1,data);
      set_gyroscope_range(GYRORANGE_250DPS);
      set_gyroscope_rateBwLevel(GYRORATE_95,GYROBW_LVL2);         
      printf("_ADA10DOF: Gyroscope initialized.\n");
      return true;
   } else { 
      gyro_sens = GYROSENS_250DPS*DPSTORADS;
      printf("_ADA10DOF: Failed to connect to Gyroscope.\n"); 
      return false;
   }
}

void Ada10Dof::set_gyroscope_range(Ada10Dof_GyroRange range)
{
   uint8_t data[1] = {range};
   i2cSendData(GYRO_ADDRESS,GYRO_REGISTER_CTRL_REG4,1,data);   
   
   switch(range)
   {
      case GYRORANGE_250DPS:
         gyro_sens = ((float)GYROSENS_250DPS/100000.0)*DPSTORADS;
         break;
      case GYRORANGE_500DPS:
         gyro_sens = ((float)GYROSENS_500DPS/100000.0)*DPSTORADS;
         break;
      case GYRORANGE_2000DPS:
         gyro_sens = ((float)GYROSENS_2000DPS/100000.0)*DPSTORADS;
         break;
   }  
}

void Ada10Dof::set_gyroscope_rateBwLevel(Ada10Dof_GyroRate rate,Ada10Dof_GyroBWLevel bwlevel)
{
   uint8_t data[1] = {rate};
   data[1]=(rate & ~0x30)|(bwlevel & 0x30);
   i2cSendData(GYRO_ADDRESS,GYRO_REGISTER_CTRL_REG1,1,data);
}

void Ada10Dof::read_gyroscope_z(float *rate_z)
{
   uint8_t values[6] = {0,0,0,0,0,0};
   i2cRequestData(GYRO_ADDRESS, GYRO_REGISTER_OUT_Z_L|0x80, 2, values);
   *rate_z = ((int16_t)(values[0] | (values[1] << 8)))*gyro_sens;
}
/*************************************************************/

/* Kalman Functions */
/*************************************************************/
void Ada10Dof::compute_kalman_z(float rateZ)
{
   kalman.predictedState.z = corrected_imu - 0.025*rateZ*RADTODEG;
   while(kalman.predictedState.z > 360.0) kalman.predictedState.z -= 360.0;
   while(kalman.predictedState.z < 0.0) kalman.predictedState.z += 360.0;
   corrected_imu = correct_imu();
   if(corrected_imu < 90.0 && kalman.predictedState.z > 270.0) kalman.predictedState.z -= 360.0;
   if(corrected_imu > 270.0 && kalman.predictedState.z < 90.0) kalman.predictedState.z += 360.0;
   kalman.predictedCovariance.z = kalman.lastCovariance.z + kalman.Q.z;
   kalman.K.z = kalman.predictedCovariance.z/(kalman.predictedCovariance.z+kalman.R.z);
   corrected_imu = kalman.predictedState.z + kalman.K.z*(corrected_imu - kalman.predictedState.z);
   while(corrected_imu > 360.0) corrected_imu -= 360.0;
   while(corrected_imu < 0.0) corrected_imu += 360.0;
   kalman.covariance.z = (1-kalman.K.z)*kalman.predictedCovariance.z;  
}
/*************************************************************/


/* Reading Routines */
/*************************************************************/
int Ada10Dof::correct_imu()
{
   int counter = 0;
   pthread_mutex_lock(&raw_val_mutex);
   float raw = raw_imu_value;
   pthread_mutex_unlock(&raw_val_mutex);
   
   if(raw<alfa) raw = raw+360-alfa;
   else raw = raw-alfa;

   pthread_mutex_lock(&lin_mutex);
   while(raw>imu_values[counter] && counter<imu_values.size()) counter++;
   int ret = (int)b[counter-1]+m[counter-1]*raw;
   pthread_mutex_unlock(&lin_mutex);

   return ret;
}

/* Implement all the reading stuff here */

int Ada10Dof::get_heading()
{
   float rateZ = 0.0;
   raw_imu_value = read_magnetometer_z(true);
   read_gyroscope_z(&rateZ);
   compute_kalman_z(rateZ);
   return corrected_imu;   
}
/*************************************************************/

/* I2C Bus communication Functions
   /*************************************************************/ 
int Ada10Dof::i2cRequestByte(uint8_t address, uint8_t command)
{
   i2c_start_transmission(address);
   __u8 byte[1];
   if(i2c_smbus_read_i2c_block_data(i2c_fd,command,1,byte)!=1) return -1;
   return (int)byte[0];
}

int Ada10Dof::i2cRequestWord(uint8_t address, uint8_t command)
{
   i2c_start_transmission(address);
   __u8 bytes[2];
   if(i2c_smbus_read_i2c_block_data(i2c_fd,command,2,bytes)!=2) return -1;
   return ((int)bytes[0]<<8|(int)(bytes[1]&0xFF));  
}

int Ada10Dof::i2cRequestData(uint8_t address, uint8_t command, uint8_t length, uint8_t* values)
{
   i2c_start_transmission(address);
   int bytes = i2c_smbus_read_i2c_block_data(i2c_fd,command,length,values);
   if(bytes!=length) return -1;
   else return bytes;
}

int Ada10Dof::i2cSendData(uint8_t address, uint8_t command, uint8_t length, uint8_t* values)
{
   i2c_start_transmission(address);
   return i2c_smbus_write_i2c_block_data(i2c_fd,command,length,values);
}
   /*************************************************************/ 
