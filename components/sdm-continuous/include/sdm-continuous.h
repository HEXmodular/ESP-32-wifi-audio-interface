#include <stdint.h>

void sdm_dac_init(void);

// для записи буфера семплов в циклический буфер sdm-каналов
void sdm_continuous_write_to_channels(int16_t *payload);