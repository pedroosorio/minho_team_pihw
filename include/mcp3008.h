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
#include <stdbool.h>
#include <string.h>

typedef unsigned int uint;
typedef unsigned char uchar;

typedef enum SPI_DEV {SPI_DEV_0=0,SPI_DEV_1} SPI_DEV;
/* Configuration defines */

#define N_CHANNELS 8 //Number of channels of MCP3008
#define MISSING_INIT "error: init_mcp3k8() should be called before any other operation.\n"

/// \brief vector to hold values read from each channel
extern float channels[N_CHANNELS];
/// \brief boolean value to define wether init function has been
/// already called or not
extern bool init;
/// \brief spi file descriptor
extern int spi_fd;
/// \brief device name
extern char spi_dev_name[15];
/// \brief spi interface channel
extern uchar spi_channel;
/// \brief spi clock frequency
extern uint spi_fclk;
/// \brief spi operation mode
extern uchar spi_mode;
/// \brief number of bit per word
extern uchar spi_bitsWord;
/// \brief standard data spi exchange structure
extern struct spi_ioc_transfer spi_type_ioc;
/// \brief size of spi_ioc_transfer struct
extern int sz_ioc;
/// \brief standard data array for mcp3k8
extern uchar data_out[3];
/// \brief standard data array size
extern int sz_dout;

// \brief function that reads a channel
/// \param channel - number of the channel to be read
/// \param vref - conversion reference voltage
float read_channel_mcp3k8(float vref, char channel);

/// \brief function that reads all channels and
/// stores the values in channels
/// \param vec - pointer to vector where data will be stored
/// \param vred - conversion reference voltage
void read_all_mcp3k8(float vref, float *vec);

/// \brief init function
/// \return - -1 on error. linux file descriptor (>0) on success.
/// \param channel - SPI driver channel (0 or 1)
/// \param fclk - SPI Clock speed in Hertz 
/// \param mode - SPI Mode (0,1,2,3) depending on clock phase and polarity
/// \param bytesWord - number of bits per word
int init_mcp3k8(SPI_DEV channel, uint fclk, uchar mode, uchar bitsWord);

/// \brief closes SPI interface
/// \param fd - file descriptor of the interface to be closes
/// \return - <0 for error on closing.
int close_mcp3k8(int fd);

/// \brief function that reads and writes data to SPI bus
/// \brief data - pointer to in and out data
/// \brief length - size of data
/// \return - <0 for error on read/write.
int spi_rw(uchar *data, uint length);
