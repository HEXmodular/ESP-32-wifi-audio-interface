#include "esp_adc/adc_continuous.h"

void adc_continuous_init(void);
void adc_continuous_register_buffer_ready_callback(void (*callback)(uint8_t* samples));
void adc_continuous_adc_deinit(adc_continuous_handle_t handle);