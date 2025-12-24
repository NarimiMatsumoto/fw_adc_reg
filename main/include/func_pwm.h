#ifndef INCLUDE_FUNC_PWM_H
#define INCLUDE_FUNC_PWM_H
// *****************************************************************************
// Include File
// *****************************************************************************
#include "sdkconfig.h"
#include "driver/ledc.h"
#include "func_common.h"
//#include "func_gpio.h"
#include "func_pcnt.h"
#include <math.h>
//#include "func_gptimer.h"

// *****************************************************************************
// Parameter
// *****************************************************************************
static const char *TAG_PWM = "PWM";
#define PWM_MODE       LEDC_LOW_SPEED_MODE
#define PWM_FAST_TIMER LEDC_TIMER_0
#define PWM_SLOW_TIMER LEDC_TIMER_1
#define PWM_CHANNEL    LEDC_CHANNEL_0
//#define PWM_DUTY_RES   LEDC_TIMER_13_BIT
// #define FAST_FREQUENCY 2000
//#define FAST_FREQUENCY 1000
// #define FAST_TRIGNUM 1
//#define FAST_TRIGNUM 16
// #define SLOW_FREQUENCY 250
#define SLOW_FREQUENCY 500
// #define SLOW_TRIGNUM 1
#define SLOW_TRIGNUM 16
extern SemaphoreHandle_t pcnt_reach_sem;
// *****************************************************************************
// Prototypes
// *****************************************************************************
void PWM_Initialize(int gpio_num, uint32_t freq_hz);
uint32_t PWM_GetFastFreq();
void PWM_SetFastFreq(uint32_t freq_hz);
void PWM_TrigLoop(int trig_per_frame);
// *****************************************************************************
// Macro
// *****************************************************************************
#define PWM_DUTY(rate) (pow(2, duty_res) * rate / 100)
#endif // INCLUDE_FUNC_PWM_H