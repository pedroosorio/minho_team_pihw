/* This librarie interfaces SPI 4-CHannel ADC MCP3008 using wiringPi's SPI */
#include "MCP3k8/mcp3k8.h"


MCP3k8::MCP3k8(SPI_DEV channel, uint fclk, uchar mode, uchar bitsWord, float vref)
{  
   // Default values and initializations
   memset(channels,0,sizeof(channels));  
   init = false;
   spi_fd = -1;
   memcpy(spi_dev_name,"/dev/spidev0.",14*sizeof(char));
   spi_channel = 0;
   spi_fclk = 1000000; //1MHz
   spi_bitsWord = 8;
   sz_ioc = sizeof(struct spi_ioc_transfer);
   memset(&spi_type_ioc,0,sz_ioc);
   sz_dout = 3*sizeof(uchar);
   memset(data_out,0,sz_dout);
   voltage_reference = 3.3;

   //Assignings and SPI interface initialization
   if(channel==SPI_DEV_0){strcat(spi_dev_name,"0\0");}
   else if(channel==SPI_DEV_1){strcat(spi_dev_name,"1\0");}
   else {perror("_MCP3008: Invalid SPI Channel"); return;} 
   printf("_MCP3008: Opening MCP3008 in %s\n",spi_dev_name);
   
   spi_channel = channel;
   spi_fclk = fclk;
   spi_mode = mode;
   spi_bitsWord = bitsWord;
   voltage_reference = vref;
   
   if(openMCP3k8()>=0) printf("_MCP3008: Initialization Successful.\n");
   else printf("_MCP3008: Initialization Failed.\n");
}

int MCP3k8::openMCP3k8()
{
   if(init) { printf("_MCP3008: SPI Interface already opened"); return -1; }

   int ret = -1;
   spi_fd = open(spi_dev_name,O_RDWR);
   if(spi_fd<0) { perror("_MCP3008: Couldn't open SPI Interface."); return -1; }
   
   ret = ioctl(spi_fd,SPI_IOC_WR_MODE,&spi_mode);
   if(ret<0) { perror("_MCP3008: Couldn't set SPI mode (WR)."); return -1; }

   ret = ioctl(spi_fd,SPI_IOC_RD_MODE,&spi_mode);
   if(ret<0) { perror("_MCP3008: Couldn't set SPI mode (RD)."); return -1; }

   ret = ioctl(spi_fd,SPI_IOC_WR_BITS_PER_WORD,&spi_bitsWord);
   if(ret<0) { perror("_MCP3008: Couldn't set SPI Bits per Word (WR)."); return -1; }
   
   ret = ioctl(spi_fd,SPI_IOC_RD_BITS_PER_WORD,&spi_bitsWord);
   if(ret<0) { perror("_MCP3008: Couldn't set SPI Bits per Word (RD)."); return -1; }
   
   ret = ioctl(spi_fd,SPI_IOC_WR_MAX_SPEED_HZ,&spi_fclk);
   if(ret<0) { perror("_MCP3008: Couldn't set SPI Clock Frequency (WR)."); return -1; }

   ret = ioctl(spi_fd,SPI_IOC_RD_MAX_SPEED_HZ,&spi_fclk);
   if(ret<0) { perror("_MCP3008: Couldn't set SPI Clock Frequency (RD)."); return -1; }
   
   init = true;
   
   spi_type_ioc.tx_buf = 0;
   spi_type_ioc.rx_buf = 0;
   spi_type_ioc.len = 0;
   spi_type_ioc.delay_usecs = 0 ;
   spi_type_ioc.speed_hz = spi_fclk;
   spi_type_ioc.bits_per_word = spi_bitsWord;
   spi_type_ioc.cs_change = 0;

   data_out[0] = 1;
   data_out[1] = 0b1000; //default to channel 0
   data_out[2] = 0;
      
   return spi_fd;
}

int MCP3k8::closeMCP3k8()
{
   int ret = -1;
   if(spi_fd<0) {
      printf("_MCP3008: Invalid SPI Interface file descriptor.\n");
      return ret;
   }

   ret = close(spi_fd);
   if(ret<0) { perror("_MCP3008: Couldn't close SPI Interface."); }
   else { 
      init = false; 
      printf("_MCP3008: %s closed.\n",spi_dev_name); 
   } 

   return ret; 
}

float MCP3k8::readChannel(char channel, float vref)
{
   if(!init) { perror(MISSING_INIT); return -1.0; }  
   if(channel<0 || channel>=N_CHANNELS) return -1.0;
   else {
      uchar data[3];
      memcpy(data,data_out,sz_dout);
      data[1] = 0b10000000|(((channel&7)<<4));
      if(spi_rw(data,sz_dout)>=0){
         int adc_val = 0;
         adc_val = (data[1]<<8)&0b1100000000;
         adc_val |= (data[2]&0xff);
         return ((float)adc_val*vref)/1023.0;
      } else perror("_MCP3008: Error reading channel.");
      
      return -1.0;     
   }
}

void MCP3k8::readAll(float *vec)
{
   if(!init) { perror(MISSING_INIT); return; }
   for(uint i=0;i<N_CHANNELS;i++){
      channels[i] = readChannel(i,voltage_reference);
   }
   memcpy(channels,vec,sizeof(channels));
}


int MCP3k8::spi_rw(uchar *data, uint length){

   struct spi_ioc_transfer spi_transfer[length];
   int ret = -1;

   for (uint i = 0 ; i < length ; i++){
      memcpy(&spi_transfer[i],&spi_type_ioc,sz_ioc);
      spi_transfer[i].tx_buf = (unsigned long)(data + i);
      spi_transfer[i].rx_buf = (unsigned long)(data + i);
      spi_transfer[i].len = sizeof(*(data + i)) ;
   }

   ret = ioctl (spi_fd, SPI_IOC_MESSAGE(length), &spi_transfer) ;
   if(ret<0){perror("_MCP3008: Couldn't read form device.\n"); }
   return ret;
}
