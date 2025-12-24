#include "func_pcnt.h"
static pcnt_unit_handle_t pcnt_unit;
// *****************************************************************************
// Callback Function
// *****************************************************************************
// volatile bool on_reach = false;
// static bool pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
//     // PWM_StopTrig(PWM_FAST_TIMER);
//     bool *flag = (bool *)user_ctx;
//     *flag = true;
//     return true;
// }
SemaphoreHandle_t pcnt_reach_sem = NULL;
static bool pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(pcnt_reach_sem, &xHigherPriorityTaskWoken);
    return (xHigherPriorityTaskWoken == pdTRUE);
}
// *****************************************************************************
// Main Function
// *****************************************************************************
int cur_watch_point = 0;
void PCNT_Initialize(void) {
    ESP_LOGI(TAG_PCNT, "PCNT initialization");
    //Limit Settings
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit  = PCNT_LOW_LIMIT,
    };
    pcnt_unit = (pcnt_unit_handle_t) NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    //Glitch Filter Settings
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    //Channel Settings
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = CTL_PIN,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_HOLD, PCNT_CHANNEL_EDGE_ACTION_INCREASE));

    cur_watch_point = 17;
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, cur_watch_point));
    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach,
    };
    // ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, &on_reach));
    pcnt_reach_sem = xSemaphoreCreateBinary();
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, NULL));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    ESP_LOGI(TAG_PCNT, "PCNT initialization DONE");
    return;
}

void PCNT_SetTrigNum(int trig_num) {
    ESP_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit, cur_watch_point));
    cur_watch_point = trig_num;
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, cur_watch_point));
    return;
}

void PCNT_CntClr(void) {
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    return;
}

void PCNT_Get(int *pcnt) {
    pcnt_unit_get_count(pcnt_unit, pcnt);
}