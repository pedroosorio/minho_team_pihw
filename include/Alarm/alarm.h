/* Here some pins shall be defined */
#define ALARM_PIN 7 //GPIO4 - GPI0_GCLK - BOT PIN 4
#define BASE 1000
#define TIMESTP 50000

typedef enum ALARM{MAIN=0,PC,CAM,TELE_ON,TELE_OFF,FW_ON,FW_OFF} ALARM;

#include <wiringPi.h>
#include <softTone.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <stdio.h>

extern pthread_mutex_t *mutex;

/// \brief function that sets up pin and wiringPi's interface for
/// alarm buzzer
void setup_alarm(pthread_mutex_t *mt);

/// \brief instanciates a thread to play an alarm
/// in the Buzzer
/// \param type - type of alarm to be thrown
void throw_alarm(void *type);

/// \brief plays ready sound
void throw_alarm_ready();
