#include <iostream>
#include "rttimer.h"

pthread_t thread1,thread2;
periodic_info th1,th2;

void *func1(void *per_info)
{
   periodic_info *info = (periodic_info *)(per_info);
   make_periodic(info->period_us,info);
   
   struct timeval now,past;
   gettimeofday(&now,0);
   past = now;
   while(1){
      gettimeofday(&now,0);
      std::cout << "Hi from thread " << info->id << " time " << ((float)((past.tv_sec-now.tv_sec)*1000000LL + past.tv_usec-now.tv_usec))/1000000.0 << std::endl;
      if(info->id==1)usleep(900000);
      past = now;
      wait_period(info);
   }
}

int main(int argc, char**argv)
{
   th1.period_us = 1000000; th1.id = 1;
   th2.period_us = 22000; th2.id = 2;
   pthread_create(&thread1, NULL, func1, &th1);
   pthread_create(&thread2, NULL, func1, &th2);

	pthread_join(thread1, NULL);
	//pthread_join(thread2, NULL);

   return 0;
}
