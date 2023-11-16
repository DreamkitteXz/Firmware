#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "cJSON.h"
#include "Communication/libs/data_queue.h"

#define TAG "HTTP"


esp_err_t _http_event_handle(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER");
            printf("%.*s", evt->data_len, (char*)evt->data);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                printf("%.*s", evt->data_len, (char*)evt->data);
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGI(TAG, "HTTP_EVENT_REDIRECT");
            break;
    }
    return ESP_OK;
}
// Função para criar uma string JSON a partir dos dados da estrutura SensorData
char* create_json_string(SensorData data) {
    char json_buffer[256];  // Tamanho suficiente para a string JSON
    data.voltage = 2340;
    data.batery = 4095;
snprintf(json_buffer, sizeof(json_buffer),
         "{\"equipe\": 4,\"bateria\":%d,\"temperatura\":%.1f,\"pressao\":%ld,\"gyroscope\":{\"x\":%d,\"y\":%d,\"z\":%d},\"acelerometro\":{\"x\":%d,\"y\":%d,\"z\":%d},\"payload\":{\"contador\":%d,\"tensao\":%d,\"temp\":%.1f}}",
          data.batery, data.temperature, data.pressure, data.gyro_x, data.gyro_y, data.gyro_z, data.accel_x, data.accel_y, data.accel_z, data.pulses, data.voltage, data.plate_temp);
          
    char *json_string = strdup(json_buffer);  // Aloque dinamicamente a string JSON

    return json_string;
}

// Você pode usar a função create_json_string para criar a string JSON e adicioná-la ao seu código de envio HTTP.
void http_request() {
    // Obtenha a string JSON a partir dos dados da estrutura SensorData
    char *post_data = create_json_string(sensor_data);

    esp_http_client_config_t config_post = {
        .url = "http://192.168.4.1/",
        .method = HTTP_METHOD_POST,
        .cert_pem = NULL,
        .event_handler = _http_event_handle};
        
    esp_http_client_handle_t client = esp_http_client_init(&config_post);

    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    esp_http_client_set_header(client, "Content-Type", "application/json");

    esp_http_client_perform(client);

    // Libere a memória alocada para a string JSON
    free(post_data);

    esp_http_client_cleanup(client);
}