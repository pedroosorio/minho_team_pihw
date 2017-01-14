//Test program for SPI and I2C in RPI

#include "mcp3008.h"
#include <sys/time.h>

int fd = -1;
struct timeval tv1, tv2;
float val = 0;
int main(int argc, char**argv)
{
   fd = init_mcp3k8(SPI_DEV_0,1000000,SPI_MODE_0,8);
   for(int i=0;i<4;i++){
      val = read_channel_mcp3k8(3.3,i);
      printf("MCP3008 CH%d : %.2f %lu\n",i,val,(tv2.tv_usec - tv1.tv_usec));
   }
   close_mcp3k8(fd);
   return 0;
}
