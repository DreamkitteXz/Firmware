
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/pcnt.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "Communication/libs/data_queue.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"

#define PULSE_COUNTER_PIN 26

void pcnt_contador(void * params)
{
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = PULSE_COUNTER_PIN,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = 32767,
        .counter_l_lim = -32767
    };
    //pcnt_glitch_filter_config_t filter_config = {
     //   .max_glitch_ns = 12.5,
    //};
    pcnt_unit_t pcnt_unit = PCNT_UNIT_0;
    pcnt_channel_t pcnt_channel = PCNT_CHANNEL_0;

    //ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config));
    ESP_ERROR_CHECK(pcnt_counter_pause(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_counter_clear(pcnt_unit));
    //ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
    pcnt_counter_resume(pcnt_unit);
    rtc_gpio_isolate(26);


    while (1)
    {
        int16_t pulse_count;
        pcnt_get_counter_value(pcnt_unit, &pulse_count);
        //printf("Contagem de pulsos: %d\n", pulse_count);
        sensor_data.pulses = pulse_count;
   vTaskDelay(500/ portTICK_PERIOD_MS);
    }
}
void pulse_counter_suply(void *params)
{
  esp_rom_gpio_pad_select_gpio(33);
  gpio_set_direction(33, GPIO_MODE_OUTPUT);
  int ON = 0;
while(true)
{
   ON = !ON;
   gpio_set_level(33, ON);
   vTaskDelay(100 / portTICK_PERIOD_MS);
}
}
void pulse_counter_start(void)
{
    xTaskCreate(&pcnt_contador, "Codigo de Contar", 2048, NULL, 1, NULL);
    xTaskCreate(&pulse_counter_suply, "Codigo de gerar pulsos", 2048, NULL, 1, NULL);
}