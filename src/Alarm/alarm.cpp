#include "alarm.h"

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

/// \brief function that sets up pin and wiringPi's interface for
/// alarm buzzer
void setup_alarm()
{
   wiringPiSetup();
   softToneCreate(ALARM_PIN); 
   pullUpDnControl(ALARM_PIN, PUD_OFF);  
}

/// \brief instanciates a thread to play an alarm
/// in the Buzzer
/// \param type - type of alarm to be thrown
void throw_alarm(void *type)
{
   ALARM *alarm = (ALARM *)type;
   
   pthread_mutex_lock(&mutex);
   switch(*alarm){
   
   case MAIN:{
      for(int i=0;i<15;i++){
         softToneWrite(ALARM_PIN,BASE);
         usleep(4*TIMESTP);
         softToneWrite(ALARM_PIN,0);
         usleep(2*TIMESTP); 
      }
      break;
   }
   case PC:{
      for(int i=0;i<10;i++){
         softToneWrite(ALARM_PIN,BASE);
         usleep(2*TIMESTP);
         softToneWrite(ALARM_PIN,0);
         usleep(1*TIMESTP);
         softToneWrite(ALARM_PIN,BASE);
         usleep(2*TIMESTP);
         softToneWrite(ALARM_PIN,0);
         usleep(4*TIMESTP);
      }
      break;
   }
   case CAM:{
      for(int i=0;i<6;i++){
         softToneWrite(ALARM_PIN,BASE);
         usleep(2*TIMESTP);
         softToneWrite(ALARM_PIN,0);
         usleep(1*TIMESTP);
         softToneWrite(ALARM_PIN,BASE);
         usleep(2*TIMESTP);
         softToneWrite(ALARM_PIN,0);
         usleep(1*TIMESTP);
         softToneWrite(ALARM_PIN,BASE);
         usleep(2*TIMESTP);
         softToneWrite(ALARM_PIN,0);
         usleep(8*TIMESTP);
      }
      break;
   }
   case TELE_ON:{
      softToneWrite(ALARM_PIN,BASE);
      usleep(5*TIMESTP);
      softToneWrite(ALARM_PIN,BASE+200);
      usleep(7*TIMESTP);
      softToneWrite(ALARM_PIN,0);
      break;
   }
   case TELE_OFF:{
      softToneWrite(ALARM_PIN,BASE+200);
      usleep(5*TIMESTP);
      softToneWrite(ALARM_PIN,BASE);
      usleep(7*TIMESTP);
      softToneWrite(ALARM_PIN,0);
      break;
   }
   
   case FW_ON:{
      softToneWrite(ALARM_PIN,BASE+200);
      usleep(10*TIMESTP);
      softToneWrite(ALARM_PIN,0);
      break;
   }
   case FW_OFF:{
      softToneWrite(ALARM_PIN,BASE-200);
      usleep(10*TIMESTP);
      softToneWrite(ALARM_PIN,0);
      break;
   }

   }
   
   pthread_mutex_unlock(&mutex);  
   delete((ALARM*)type);  
   return;   
}

/// \brief plays ready sound
void throw_alarm_ready()
{
   pthread_mutex_lock(&mutex);
   softToneWrite(ALARM_PIN,BASE);
   usleep(4*TIMESTP);
   softToneWrite(ALARM_PIN,0);
   usleep(TIMESTP);
   softToneWrite(ALARM_PIN,BASE);
   usleep(4*TIMESTP);
   softToneWrite(ALARM_PIN,0);
   usleep(TIMESTP);
   softToneWrite(ALARM_PIN,BASE);
   usleep(TIMESTP);
   softToneWrite(ALARM_PIN,0);
   usleep(4*TIMESTP);
   softToneWrite(ALARM_PIN,BASE);
   usleep(4*TIMESTP);
   softToneWrite(ALARM_PIN,0);
   usleep(TIMESTP);
   softToneWrite(ALARM_PIN,BASE);
   usleep(4*TIMESTP);
   softToneWrite(ALARM_PIN,0);
   usleep(TIMESTP);
   softToneWrite(ALARM_PIN,BASE);
   usleep(4*TIMESTP);
   softToneWrite(ALARM_PIN,0);
   usleep(TIMESTP);
   pthread_mutex_unlock(&mutex);
}
