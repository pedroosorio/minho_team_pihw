//Test program for SPI and I2C in RPI

#include "mcp3k8.h"
#include "Omni3MD.h"
#include <sys/time.h>
#include <pthread.h>

int main(int argc, char**argv)
{
   printf("Omni3MD library test.\n");
   Omni3MD omni(0x18);
   pthread_t main_thread = pthread_self();
   printf("Ctrl Rate: %d\n", omni.read_control_rate());
   printf("Firmware: %.3f\n", omni.read_firmware());
   printf("Temperature: %.2f\n", omni.read_temperature());   

   int enc1, enc2, enc3;
   for(unsigned long int i=0;i<500000;i++){
      enc1 = omni.read_enc1(); enc2 = omni.read_enc2(); enc3 = omni.read_enc3();
      printf("Encoders: %d | %d | %d \n",enc1,enc2,enc3);
      printf("Battery: %.2f\n",omni.read_battery());
      usleep(20000);
   }
   return 0;
}
