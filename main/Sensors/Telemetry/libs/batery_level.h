#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "Communication/libs/data_queue.h"

static esp_adc_cal_characteristics_t adc1_chars;

void ler_nivel_bateria(void *params);

void ler_nivel_bateria_start();

#endif /* CURRENT_SENSOR_H */
