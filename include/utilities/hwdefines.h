/* This header defines voltage references, offsets and
   critical voltages for measurements.
*/

// PC
#define PC_REF_VOLT 15.0
#define PC_CRIT_VOLT 10.5
#define PC_OFF_VOLT 0.0

// CAM
#define CAM_REF_VOLT 8.0
#define CAM_CRIT_VOLT 5.6
#define CAM_OFF_VOLT 0.0

// MAIN
#define MAIN_CRIT_VOLT 21.0
#define MAIN_OFF_VOLT -4.0

/* This header defines KICK pin and kick signal properties */
#define KICK_PIN 4
#define KICK_MAX_DURATION 25000
#define KICK_STR_CONV KICK_MAX_DURATION/100
#define PASS_MAX_DURATION 10000
#define PASS_STR_CONV PASS_MAX_DURATION/100

#define PC_GPIO 2
#define CAM_GPIO 0
#define GRABBER_GPIO 3
#define BALLSENS_GPIO 7
#define FREE_WHEEL_GPIO 5
