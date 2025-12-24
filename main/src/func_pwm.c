#include "func_pwm.h"
static ledc_timer_config_t fast_timer;
static ledc_timer_config_t slow_timer;
static ledc_channel_config_t ledc_channel;

static uint32_t duty_res;

void PWM_Initialize(int gpio_num, uint32_t freq_hz)
{
    ESP_LOGI(TAG_PWM, "PWM initialization");

    duty_res = ledc_find_suitable_duty_resolution(40000000, freq_hz);

    //Timer Settings
    fast_timer = (ledc_timer_config_t){
        fast_timer.speed_mode       = PWM_MODE,
        fast_timer.duty_resolution  = duty_res,
        fast_timer.timer_num        = PWM_FAST_TIMER,
        fast_timer.freq_hz          = freq_hz,
        fast_timer.clk_cfg          = LEDC_USE_XTAL_CLK,
        fast_timer.deconfigure      = false
    };
    ESP_ERROR_CHECK(ledc_timer_config(&fast_timer));

    /*
    slow_timer = (ledc_timer_config_t){
        .speed_mode       = PWM_MODE,
        .duty_resolution  = PWM_DUTY_RES,
        .timer_num        = PWM_SLOW_TIMER,
        .freq_hz          = SLOW_FREQUENCY,
        .clk_cfg          = LEDC_USE_XTAL_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&slow_timer));
    */

    //Pin Settings
    ledc_channel = (ledc_channel_config_t){
        .speed_mode     = PWM_MODE,
        .channel        = PWM_CHANNEL,
        .timer_sel      = PWM_FAST_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = gpio_num,
        // .duty           = PWM_DUTY(25),
        .duty           = PWM_DUTY(50),
        // .hpoint         = pow(2, PWM_DUTY_RES) - 1
        .hpoint         = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ESP_ERROR_CHECK(ledc_timer_pause(PWM_MODE, PWM_FAST_TIMER));
    ESP_ERROR_CHECK(ledc_timer_rst(PWM_MODE, PWM_FAST_TIMER));
    // ESP_LOGI(TAG_PWM, "Fast Duty max reso:%d", ledc_find_suitable_duty_resolution(40000000, 2000));
    // ESP_LOGI(TAG_PWM, "Slow Duty max reso:%d", ledc_find_suitable_duty_resolution(40000000, 250));
    uint32_t act_freq = ledc_get_freq(PWM_MODE, PWM_FAST_TIMER);
    ESP_LOGI(TAG_PWM, "PWM initialization DONE, Freq = %ld Hz (%f ms), Resolution %ld", act_freq, 1000.0 / act_freq, duty_res);
    return;

}

// static void PWM_ChgDuty(int rate) {
//     ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL, PWM_DUTY(rate)));
//     ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL));
//     return;
// }

static void PWM_ChgTimer(ledc_timer_t timer_sel) {
    ESP_ERROR_CHECK(ledc_bind_channel_timer(LEDC_LOW_SPEED_MODE, PWM_CHANNEL, timer_sel));
    ESP_ERROR_CHECK(ledc_timer_rst(LEDC_LOW_SPEED_MODE, timer_sel));
    return;
}

void PWM_StartFastTrig() {
    PWM_ChgTimer(PWM_FAST_TIMER);
    //Start FAST Triger
    // on_reach = false;
    // ESP_LOGI(TAG_PWM, "PWM Resume. (FastTrig)!");
    ESP_ERROR_CHECK(ledc_timer_resume(PWM_MODE, PWM_FAST_TIMER));
    // while(!on_reach); //wait pulse count.
    if (xSemaphoreTake(pcnt_reach_sem, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG_PWM, "PCNT watch point not reached in time (FastTrig)!");
    }
    //Stop FAST Triger
    ESP_ERROR_CHECK(ledc_timer_pause(PWM_MODE, PWM_FAST_TIMER));
    // ESP_LOGI(TAG_PWM, "PWM Pause. (FastTrig)!");
    PCNT_CntClr();
    return;
}

void PWM_StartSlowTrig() {
    PWM_ChgTimer(PWM_SLOW_TIMER);
    //Start SLOW Triger
    // on_reach = false;
    ESP_ERROR_CHECK(ledc_timer_resume(PWM_MODE, PWM_SLOW_TIMER));
    // while(!on_reach); //wait pulse count.
    if (xSemaphoreTake(pcnt_reach_sem, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG_PWM, "PCNT watch point not reached in time (SlowTrig)!");
    }
    //Stop SLOW Triger
    ESP_ERROR_CHECK(ledc_timer_pause(PWM_MODE, PWM_SLOW_TIMER));
    PCNT_CntClr();
    return;
}

uint32_t PWM_GetFastFreq() {
    return ledc_get_freq(PWM_MODE, PWM_FAST_TIMER);
}

void PWM_SetFastFreq(uint32_t freq_hz) {
    ESP_ERROR_CHECK(ledc_set_freq(PWM_MODE, PWM_FAST_TIMER, freq_hz));
}

void PWM_TrigLoop(int trig_per_frame) {
    PCNT_SetTrigNum(trig_per_frame);
    PWM_StartFastTrig();
    // GPT_DelayUs(500);
    // PCNT_SetTrigNum(SLOW_TRIGNUM);
    // PWM_StartSlowTrig();
    return;
}
