#include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_mac.h"
// #include "esp_wifi.h"
// #include "esp_event.h"
#include "esp_log.h"
// #include "nvs_flash.h"
#include "wifi_ap.h"
#include "web_server.h"
#include "adc-continuous.h"
#include "sdm-continuous.h"

#include "lwip/err.h"
#include "lwip/sys.h"

// получен буфер с данными от АЦП, отправляем его веб-серверу
void adc_continuous_buffer_ready_callback(uint8_t* samples){
    web_server_send_samples_to_client(samples);
}

// получен буфер с данными от веб-сервера, отправляем его в sdm-каналы
void web_server_ws_recv_frame_callback(uint16_t* samples){
    sdm_continuous_write_to_channels(samples);
}

void app_main(void)
{
    // //Initialize NVS
    // esp_err_t ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    //   ESP_ERROR_CHECK(nvs_flash_erase());
    //   ret = nvs_flash_init();
    // }
    // ESP_ERROR_CHECK(ret);

    // ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();
    web_server_init();
    // регистрируем коллбек для получения данных из веб-сокета
    web_server_register_ws_recv_frame_callback(&web_server_ws_recv_frame_callback);
    
    adc_continuous_init();
    // данные получены из АЦП, регистрируем коллбек
    adc_continuous_register_buffer_ready_callback(&adc_continuous_buffer_ready_callback);   

}
