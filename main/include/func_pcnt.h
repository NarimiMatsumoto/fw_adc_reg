#ifndef INCLUDE_FUNC_PCNT_H
#define INCLUDE_FUNC_PCNT_H
// *****************************************************************************
// Include File
// *****************************************************************************
#include "sdkconfig.h"
#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "func_common.h"
//#include "func_gpio.h"

// *****************************************************************************
// Parameter
// *****************************************************************************
static const char *TAG_PCNT = "PCNT";
// #define CTL_PIN TRIG
#define PCNT_HIGH_LIMIT 100
#define PCNT_LOW_LIMIT  -100
extern int cur_watch_point;
// *****************************************************************************
// Prototypes
// *****************************************************************************
void PCNT_Initialize(void);
void PCNT_SetTrigNum(int trig_num);
void PCNT_CntClr(void);
void PCNT_Get(int *pcnt);
// *****************************************************************************
// Macro
// *****************************************************************************
#endif // INCLUDE_FUNC_PCNt_H