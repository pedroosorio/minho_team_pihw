/* Here some pins shall be defined */
#define ALARM_PIN 7 //GPIO4 - GPI0_GCLK - BOT PIN 4
#define BASE 1000
#define TIMESTP 50000
typedef enum ALARM{MAIN=0,PC,CAM,TELE_ON,TELE_OFF} ALARM;

#include <wiringPi.h>
#include <softTone.h>

/// \brief function that sets up pin and wiringPi's interface for
/// alarm buzzer
inline void setup_alarm()
{
   wiringPiSetup();
   softToneCreate(ALARM_PIN);   
}

/// \brief instanciates a thread to play an alarm
/// in the Buzzer
/// \param type - type of alarm to be thrown
inline void* throw_alarm(void *type)
{
   ALARM *alarm = (ALARM *)type;
   switch(*alarm){
   
   case MAIN:{
      for(int i=0;i<30;i++){
         softToneWrite(ALARM_PIN,BASE);
         usleep(4*TIMESTP);
         softToneWrite(ALARM_PIN,0);
         usleep(2*TIMESTP); 
      }
      break;
   }
   case PC:{
      for(int i=0;i<20;i++){
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
      for(int i=0;i<15;i++){
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
   
   }

   return NULL;   
}

/// \brief plays ready sound
inline void throw_alarm_ready()
{
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
}
