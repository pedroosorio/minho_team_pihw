/* Here some pins shall be defined */
#define ALARM_PIN 7 //GPIO4 - GPI0_GCLK - BOT PIN 4
#define BASE 1000
#define TIMESTP 50000
typedef enum ALARM{MAIN=0,PC,CAM,TELE_ON,TELE_OFF,FW_ON,FW_OFF} ALARM;

#include <wiringPi.h>
#include <softTone.h>

pthread_mutex_t *mutex;

/// \brief function that sets up pin and wiringPi's interface for
/// alarm buzzer
inline void setup_alarm(pthread_mutex_t *mt)
{
   mutex = mt;
   wiringPiSetup();
   softToneCreate(ALARM_PIN);   
}

/// \brief instanciates a thread to play an alarm
/// in the Buzzer
/// \param type - type of alarm to be thrown
inline void throw_alarm(void *type)
{
   ALARM *alarm = (ALARM *)type;
   printf("Launching alarm %d\n",*alarm);
   switch(*alarm){
   
   printf("%lx\n",mutex);  
   case MAIN:{
      printf("Main alarm\n");
      for(int i=0;i<20;i++){
         softToneWrite(ALARM_PIN,BASE);
         usleep(4*TIMESTP);
         softToneWrite(ALARM_PIN,0);
         usleep(2*TIMESTP); 
      }
      break;
   }
   case PC:{
      printf("PC alarm\n");
      for(int i=0;i<5;i++){
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
      printf("Cam alarm\n");
      for(int i=0;i<4;i++){
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
      printf("Teleop on\n");
      softToneWrite(ALARM_PIN,BASE);
      usleep(5*TIMESTP);
      softToneWrite(ALARM_PIN,BASE+200);
      usleep(7*TIMESTP);
      softToneWrite(ALARM_PIN,0);
      break;
   }
   case TELE_OFF:{
      printf("Teleop off\n");
      softToneWrite(ALARM_PIN,BASE+200);
      usleep(5*TIMESTP);
      softToneWrite(ALARM_PIN,BASE);
      usleep(7*TIMESTP);
      softToneWrite(ALARM_PIN,0);
      break;
   }
   
   case FW_ON:{
      printf("FREE WHEEL ON\n");
      softToneWrite(ALARM_PIN,BASE+200);
      usleep(5*TIMESTP);
      softToneWrite(ALARM_PIN,0);
      break;
   }
   case FW_OFF:{
      printf("FREE WHEEL OFF\n");
      softToneWrite(ALARM_PIN,BASE-200);
      usleep(5*TIMESTP);
      softToneWrite(ALARM_PIN,0);
      break;
   }

   }

   return;   
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
