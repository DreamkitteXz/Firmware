// sd_card.h

#ifndef SD_CARD_H
#define SD_CARD_H

#include "Communication/libs/data_queue.h"  // Certifique-se de incluir o cabeçalho correto para a sua estrutura SensorData

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

// Funções de leitura e escrita no arquivo
esp_err_t sd_card_write_data(const char *path, SensorData *data);
esp_err_t sd_card_read_data(const char *path);
void get_unique_filename(char *filename);

// Função principal para salvar dados no cartão SD
void saveData();

#ifdef __cplusplus
}
#endif

#endif  // SD_CARD_H
