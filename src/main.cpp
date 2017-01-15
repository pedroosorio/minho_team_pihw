//Test program for SPI and I2C in RPI

#include "mcp3k8.h"
#include "Omni3MD.h"
#include <sys/time.h>
#include <pthread.h>

int main(int argc, char**argv)
{
   printf("Omni3MD library test.\n");
   Omni3MD omni(DEFAULT_OMNI_ADDRESS);
   pthread_t main_thread = pthread_self();
   
   int enc1, enc2, enc3;
   for(int i=0;i<500;i++){
      enc1 = omni.read_enc1(); enc2 = omni.read_enc2(); enc3 = omni.read_enc3();
      printf("Encoders: %d | %d | %d \n",enc1,enc2,enc3);
      sleep(1);
   }
   return 0;
}
