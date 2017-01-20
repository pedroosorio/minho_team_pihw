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

Ada10Dof::Ada10Dof(uint8_t adaAddress)
{
   i2c_slave_address = adaAddress;
   init = false;
   device_name = "/dev/i2c-1";

   if(i2c_connect(adaAddress)>0)printf("_ADA10DOF: Initialization Successful.\n");
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
       printf("_ADA10DOF: No configuration file found. A sample one will be written.");
       write_imu_configuration();     
   }
}

/* Setup Routines */
/*************************************************************/
int Ada10Dof::i2c_connect(uint8_t adaAddress)
{
   if(init) { printf("_ADA10DOF: I2C Interface already opened"); return -1; }

   printf("_ADA10DOF: Opening Ada10Dof in %s\n",device_name.c_str());
   i2c_slave_address = adaAddress;
   /* Open I2C Interface */
	if((i2c_fd = open(device_name.c_str(), O_RDWR)) < 0){
		perror("_ADA10DOF: Couldn't open I2C Interface.");
      return -1;
	}
	
   if (ioctl (i2c_fd, I2C_SLAVE, i2c_slave_address) < 0){
      perror("_ADA10DOF: Couldn't access I2C Device.");
      return -1;
   }

   init = true;
   return i2c_fd;
}

int Ada10Dof::i2c_start_transmission()
{
   if (ioctl (i2c_fd, I2C_SLAVE, i2c_slave_address) < 0){
      perror("_ADA10DOF: Couldn't access I2C Device.");
      return -1;
    }
   return i2c_fd;
}

void Ada10Dof::set_imu_reference()
{
   pthread_mutex_lock(&raw_val_mutex);  
   alfa = raw_imu_value;
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
      cout << "Error parsing JSON file \n";
      return false;
   }else {
      if(document["alfa"].IsNumber()) alfa = document["alfa"].GetDouble();
      else { printf("_ADA10DOF: Error in variable \"alfa\"\n"); return false;} 

      if(document["step"].IsNumber()) step = document["step"].GetDouble();
      else { printf("_ADA10DOF: Error in variable \"step\"\n"); return false;} 

      if(document["imu_values"].IsArray()){
         const Value& a = document["imu_values"];
         if((360.0/(float)step) == a.Size()-1){
            for(SizeType i = 0; i < a.Size(); i++) imu_values.push_back(a[i].GetInt());
         } else { printf("_ADA10DOF: Error in variable \"imu_values\"\n"); return false;}
      }else { printf("_ADA10DOF: Error in variable \"imu_values\"\n"); return false;} 
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
   }
   
   outfile.open(filepath.c_str());
   StringBuffer sb;
   PrettyWriter<StringBuffer> writer(sb);
   document.Accept(writer);
   std::string data(sb.GetString());
   outfile << data;
   outfile.close();
   printf("_ADA10DOF: New configuration file written.\n");

}

void Ada10Dof::get_imu_configuration(int *st, std::vector<uint16_t> *imu)
{
   (*st) = step;
   (*imu) = imu_values;
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
   
   pthread_mutex_lock(&lin_mutex);
   while(raw>imu_values[counter] && counter<imu_values.size()) counter++;
   int ret = (int)b[counter-1]+m[counter-1]*raw;
   pthread_mutex_unlock(&lin_mutex);

   return ret;
}

int Ada10Dof::get_heading()
{
   pthread_mutex_lock(&raw_val_mutex);
   /* Do i2c readings and stuff, computing value to raw_imu_value */
   pthread_mutex_unlock(&raw_val_mutex);

   return correct_imu();
}
/*************************************************************/

int Ada10Dof::i2cRequestByte(uint8_t command)
{
   i2c_start_transmission();
   __u8 byte[1];
   if(i2c_smbus_read_i2c_block_data(i2c_fd,command,1,byte)!=1) return -1;
   return (int)byte[0];
}

int Ada10Dof::i2cRequestWord(uint8_t command)
{
   i2c_start_transmission();
   __u8 bytes[2];
   if(i2c_smbus_read_i2c_block_data(i2c_fd,command,2,bytes)!=2) return -1;
   return ((int)bytes[0]<<8|(int)(bytes[1]&0xFF));  
}

int Ada10Dof::i2cRequestData(uint8_t command, uint8_t length, uint8_t* values)
{
   i2c_start_transmission();
   int bytes = i2c_smbus_read_i2c_block_data(i2c_fd,command,length,values);
   if(bytes!=length) return -1;
   else return bytes;
}

int Ada10Dof::i2cSendData(uint8_t command, uint8_t length, uint8_t* values)
{
   i2c_start_transmission();
   return i2c_smbus_write_i2c_block_data(i2c_fd,command,length,values);
}
