//Test program for SPI and I2C in RPI

#include "mcp3k8.h"
#include "Omni3MD.h"
#include <sys/time.h>

int fd = -1;
struct timeval tv1, tv2;
float val = 0;
int main(int argc, char**argv)
{
   MCP3k8 bat_measure = MCP3k8(SPI_DEV_1,1000000,SPI_MODE_0,8,4.8);
   for(int i=0;i<4;i++){
      val = bat_measure.readChannel(i);
      printf("MCP3008 CH%d : %.2f\n",i,val);
   }
   bat_measure.closeMCP3k8();
   
   Omni3MD omni(0x10);
   printf("%d",omni.read_enc1());
   return 0;
}
