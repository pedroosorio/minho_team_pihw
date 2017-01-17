#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/timerfd.h>
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

typedef struct periodic_info
{
	int timer_fd;
	unsigned long long wakeups_missed;
   unsigned long int period_us;
   int id;

}periodic_info;

static inline int make_periodic (unsigned int period, periodic_info *info)
{
	int ret;
	unsigned int ns;
	unsigned int sec;
	int fd;
	struct itimerspec itval;

	/* Create the timer */
	fd = timerfd_create (CLOCK_REALTIME, 0);
	info->wakeups_missed = 0;
	info->timer_fd = fd;
	if (fd == -1)
		return fd;

	/* Make the timer periodic */
	sec = period/1000000;
	ns = (period - (sec * 1000000)) * 1000;
	itval.it_interval.tv_sec = sec;
	itval.it_interval.tv_nsec = ns;
	itval.it_value.tv_sec = sec;
	itval.it_value.tv_nsec = ns;
	ret = timerfd_settime (fd, 0, &itval, NULL);
	return ret;
}

static inline void wait_period (periodic_info *info)
{
	unsigned long long missed;
	int ret;

	/* Wait for the next timer event. If we have missed any the
	   number is written to "missed" */
	ret = read (info->timer_fd, &missed, sizeof (missed));
	if (ret == -1){
		perror ("read timer");
		return;
	}

	/* "missed" should always be >= 1, but just to be sure, check it is not 0 anyway */
	if (missed > 0)
		info->wakeups_missed += (missed - 1);
}

static inline std::string getInterfaceIp()
{
   int   fd;
   struct ifreq if_info;
   int if_index;
   std::string ifname = "eth0";
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
