/*
*  Código com a Lógica principal do Sistema
*  Tópico: Código Principal
*  Project: OBSAT-2023
*  Autor: Kayque Amado
*  Data: 29 de Junho de 2023
*/
//=======================================================
// --- Bibliotecas ---
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h" 
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"

//Wifi e servidor
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_client.h"

//Dados coletados
#include "Sensors/Payload/libs/plate_voltage.h" //Lib da task de ler tensão da placa

// Novo
#include "Sensors/Payload/libs/plate_temperature.h" // DS18B20
#include "Communication/libs/wifi.h"
#include "Communication/libs/http_request.h"
#include "Sensors/Payload/libs/pulse_counter.h"
#include "Communication/libs/data_queue.h"
#include "esp_sleep.h"
#include "i2cdev.h"
#include "Sensors/Telemetry/libs/i2c.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "esp_vfs_fat.h"
#include "Communication/libs/memory.h"
#include "Sensors/Telemetry/libs/batery_level.h"
//==================================
//teste 

//=======================================================
// --- Deep Sleep ---
void configure_deep_sleep() {
    const int deep_sleep_sec = 15; // 3 minutos e 40 segundos
    esp_sleep_enable_timer_wakeup(deep_sleep_sec * 1000000); // Microssegundos
    ESP_LOGI("DEEP SLEEP", "Antes de entrar na função do deep sleep");
    esp_deep_sleep_start();
}

//=======================================================
// --- Função Principal ---

void app_main(void)
{ 
    pulse_counter_start();
    initialize_i2c_semaphores();
    // --- inicializar o NVS ---
    ESP_LOGI("TESTANDO", "CHEGUEI AQUI");
    
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_LOGI("TESTANDO", "CHEGUEI AQUIa");
        printf("HELLO");
    // --- I2C ---
    bmp180_init_task();
    vTaskDelay( 2000 / portTICK_PERIOD_MS);
    //printf("Pressure: %ld Pa", sensor_data.pressure);
    mpu6050_init_task();
    // --- ONEWIRE ---
    ler_temp_ds18b20_start();
    //printf("Plate Temperature: %f\n", sensor_data.plate_temp);
    
     // --- PAYLOAD ---
    //
    //ler_tensao_placa_start(); // Inicia a task de ler_tensao_placa
    //printf("Voltage: %d\n", sensor_data.voltage);

   // printf("Pulse Counter: %d\n", sensor_data.pulses);
   //ler_nivel_bateria_start();

    
    
    // --- Wifi ---
    wifi_start();
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // --- HTTP POST ---
    vTaskDelay(15000 / portTICK_PERIOD_MS); //Aguarda a conexão do wifi
    http_request();
    //printf("Voltage: %d\n", sensor_data.voltage);
    //printf("Plate Temperature: %f\n", sensor_data.plate_temp);
    //printf("Pulse Counter: %d\n", sensor_data.pulses);

    // --- Prints de DEBUG ---
    /*
    while (1)
    {
        printf("Pressure: %ld Pa\n", sensor_data.pressure);
                vTaskDelay( 1000 / portTICK_PERIOD_MS);

        printf("Temperature: %f\n", sensor_data.temperature);
                vTaskDelay( 1000 / portTICK_PERIOD_MS);

        printf("Voltage: %d\n", sensor_data.voltage);
                vTaskDelay( 1000 / portTICK_PERIOD_MS);

        printf("Plate Temperature: %f\n", sensor_data.plate_temp);
                vTaskDelay( 1000 / portTICK_PERIOD_MS);

        printf("Pulse Counter: %d\n", sensor_data.pulses);
                vTaskDelay( 1000 / portTICK_PERIOD_MS);

        printf("accel_x: %d, accel_y: %d, accel_z: %d", sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z);
        printf("gyro_x: %d, gyro_y: %d, gyro_z: %d", sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);
        vTaskDelay( 1000 / portTICK_PERIOD_MS);

    }
    */
    
    // Configurar e iniciar deep sleep

    saveData();
    vTaskDelay(2000 / portTICK_PERIOD_MS); //Aguarda a conexão do wifi

     configure_deep_sleep();
    //===================================================TESTE    
}
//=======================================================
