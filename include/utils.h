/*
   This header contains utility functions
*/

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <signal.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <unistd.h>
#include <linux/if_ether.h>
#include <sstream>

/// \brief function to return ipv4 address of eth0 interface
/// \param ifname - interface name to be read
static inline std::string getInterfaceIp(std::string ifname)
{
   int   fd;
   struct ifreq if_info;
   int if_index;
   memset(&if_info, 0, sizeof(if_info));
   strncpy(if_info.ifr_name, ifname.c_str(), IFNAMSIZ-1);

   if ((fd=socket(AF_INET, SOCK_DGRAM, 0)) == -1)
   {
      return "10.42.0.64";
   }
   if (ioctl(fd, SIOCGIFINDEX, &if_info) == -1)
   {
      close(fd);
      return "10.42.0.64";
   }
   if_index = if_info.ifr_ifindex;

   if (ioctl(fd, SIOCGIFADDR, &if_info) == -1)
   {
      close(fd);
      return "10.42.0.64";
   }
   
   close(fd);
   std::stringstream ip; 
   ip << (int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[2] << "." << (int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[3] << "." << (int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[4] << "." << (int) ((unsigned char *) if_info.ifr_hwaddr.sa_data)[5]; 
   
   return ip.str();
}
