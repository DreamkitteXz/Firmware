/*
*  Código da medição da Corrente gerada pela placa (PAYLOAD)
*  Tópico: Sensor de Corrente
*  Project: OBSAT-2023
*  Autor: Kayque Amado
*  Data: 29 de Junho de 2023
*/

#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "Communication/libs/data_queue.h"

static esp_adc_cal_characteristics_t adc1_chars;


void ler_nivel_bateria(void*params)
{

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);

    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);

    while (1) 
    {
        int adc_value = adc1_get_raw(ADC1_CHANNEL_7);
        float adc_value_voltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_CHANNEL_7), &adc1_chars);
        sensor_data.batery = adc_value;
        printf("Valor ADC bateria: %d", adc_value); 
        vTaskDelay(2000/ portTICK_PERIOD_MS);
    }
}

void ler_nivel_bateria_start()
{
    xTaskCreate(&ler_nivel_bateria,"Leitura de Nível Bateria", 2048, NULL, 1, NULL);
}