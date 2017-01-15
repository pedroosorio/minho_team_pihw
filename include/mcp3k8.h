/* This librarie interfaces SPI 4-CHannel ADC MCP3008 using wiringPi's SPI */
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifndef mcp3k8_h
#define mcp3k8_h

typedef unsigned int uint;
typedef unsigned char uchar;

typedef enum SPI_DEV {SPI_DEV_0=0,SPI_DEV_1} SPI_DEV;
/* Configuration defines */

#define N_CHANNELS 8 //Number of channels of MCP3008
#define MISSING_INIT "error: init_mcp3k8() should be called before any other operation.\n"

class MCP3k8{
   public:
   /// \brief Constructor of class
   /// \return - -1 on error. linux file descriptor (>0) on success.
   /// \param channel - SPI driver channel (0 or 1)
   /// \param fclk - SPI Clock speed in Hertz 
   /// \param mode - SPI Mode (0,1,2,3) depending on clock phase and polarity
   /// \param bytesWord - number of bits per word
   /// \param vref - conversion reference voltage
   MCP3k8(SPI_DEV channel, uint fclk, uchar mode, uchar bitsWord, float vref);

   /// \brief opens SPI interface
   /// \return - <0 for error on closing.
   int openMCP3k8();

   /// \brief closes SPI interface
   /// \return - <0 for error on closing.
   int closeMCP3k8();

   // \brief function that reads a channel
   /// \param channel - number of the channel to be read
   float readChannel(char channel);

   /// \brief function that reads all channels and
   /// stores the values in channels
   /// \param vec - pointer to vector where data will be stored
   void readAll(float *vec);

   private:
   /// \brief function that reads and writes data to SPI bus
   /// \brief data - pointer to in and out data
   /// \brief length - size of data
   /// \return - <0 for error on read/write.
   int spi_rw(uchar *data, uint length);

   /// \brief vector to hold values read from each channel
   float channels[N_CHANNELS];
   /// \brief boolean value to define wether init function has been
   /// already called or not
   bool init;
   /// \brief spi file descriptor
   int spi_fd;
   /// \brief device name
   char spi_dev_name[15];
   /// \brief spi interface channel
   uchar spi_channel;
   /// \brief spi clock frequency
   uint spi_fclk;
   /// \brief spi operation mode
   uchar spi_mode;
   /// \brief number of bit per word
   uchar spi_bitsWord;
   /// \brief standard data spi exchange structure
   struct spi_ioc_transfer spi_type_ioc;
   /// \brief size of spi_ioc_transfer struct
   int sz_ioc;
   /// \brief standard data array for mcp3k8
   uchar data_out[3];
   /// \brief standard data array size
   int sz_dout;
   /// \brief defined voltage reference in mcp3008 chip
   float voltage_reference;
};

#endif
