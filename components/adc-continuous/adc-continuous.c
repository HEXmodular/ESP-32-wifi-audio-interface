/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"

#define ADC_UNIT ADC_UNIT_1
#define ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH

#define READ_LEN 256

#if CONFIG_IDF_TARGET_ESP32
static adc_channel_t channel[2] = {ADC_CHANNEL_6, ADC_CHANNEL_7};
#else
static adc_channel_t channel[2] = {ADC_CHANNEL_2, ADC_CHANNEL_3};
#endif

static TaskHandle_t s_task_handle;
static const char *TAG = "ADC-CONTINUOUS";

// Callback function pointer
static void (*buffer_ready_callback)(uint8_t* samples) = NULL;

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    // BaseType_t mustYield = pdFALSE;
    // Notify that ADC continuous driver has done enough number of conversions
    // vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    // return (mustYield == pdTRUE);

    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[READ_LEN] = {0};
    memset(result, 0xcc, READ_LEN);

    ret = adc_continuous_read(handle, result, READ_LEN, &ret_num, 0);
    if (ret == ESP_OK)
    {
        // ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);

        adc_continuous_data_t parsed_data[ret_num / SOC_ADC_DIGI_RESULT_BYTES];
        uint32_t num_parsed_samples = 0;
        // тут возможно не нужно парсить, а отдавать сырые данные сразу клиенту
        // потому что уже упакован канал и сырые данные
        buffer_ready_callback(result);

        // esp_err_t parse_ret = adc_continuous_parse_data(handle, result, ret_num, parsed_data, &num_parsed_samples);
        // if (parse_ret == ESP_OK)
        // {
        //     for (int i = 0; i < num_parsed_samples; i++)
        //     {
        //         if (parsed_data[i].valid)
        //         {
        //             ESP_LOGI(TAG, "ADC%d, Channel: %d, Value: %" PRIu32,
        //                      parsed_data[i].unit + 1,
        //                      parsed_data[i].channel,
        //                      parsed_data[i].raw_data);
        //         }
        //         else
        //         {
        //             ESP_LOGW(TAG, "Invalid data [ADC%d_Ch%d_%" PRIu32 "]",
        //                      parsed_data[i].unit + 1,
        //                      parsed_data[i].channel,
        //                      parsed_data[i].raw_data);
        //         }
        //     }
        // }
        // else
        // {
        //     ESP_LOGE(TAG, "Data parsing failed: %s", esp_err_to_name(parse_ret));
        // }
    }
    else if (ret == ESP_ERR_TIMEOUT)
    {
        // We try to read `READ_LEN` until API returns timeout, which means there's no available data
        ESP_LOGW(TAG, "Data parsing timeout: %s", esp_err_to_name(ret));
    }


    return true;
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20 * 1000,
        .conv_mode = ADC_CONV_MODE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++)
    {
        adc_pattern[i].atten = ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = ADC_UNIT;
        adc_pattern[i].bit_width = ADC_BIT_WIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

// output_register_buffer_ready_callback(&send_samples_to_client);

// Function to register buffer ready callback
void adc_continuous_register_buffer_ready_callback(void (*callback)(uint8_t* samples))
{
    ESP_LOGI(TAG, "Registering buffer ready callback");
    buffer_ready_callback = callback;
}

void adc_continuous_init(void)
{
    // esp_err_t ret;
    // uint32_t ret_num = 0;
    // uint8_t result[READ_LEN] = {0};
    // memset(result, 0xcc, READ_LEN);

    // s_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    // while (1) {

    //     /**
    //      * This is to show you the way to use the ADC continuous mode driver event callback.
    //      * This `ulTaskNotifyTake` will block when the data processing in the task is fast.
    //      * However in this example, the data processing (print) is slow, so you barely block here.
    //      *
    //      * Without using this event callback (to notify this task), you can still just call
    //      * `adc_continuous_read()` here in a loop, with/without a certain block timeout.
    //      */
    //     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    //     while (1) {
    //         ret = adc_continuous_read(handle, result, READ_LEN, &ret_num, 0);
    //         if (ret == ESP_OK) {
    //             ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);

    //             adc_continuous_data_t parsed_data[ret_num / SOC_ADC_DIGI_RESULT_BYTES];
    //             uint32_t num_parsed_samples = 0;

    //             esp_err_t parse_ret = adc_continuous_parse_data(handle, result, ret_num, parsed_data, &num_parsed_samples);
    //             if (parse_ret == ESP_OK) {
    //                 for (int i = 0; i < num_parsed_samples; i++) {
    //                     if (parsed_data[i].valid) {
    //                         ESP_LOGI(TAG, "ADC%d, Channel: %d, Value: %"PRIu32,
    //                                  parsed_data[i].unit + 1,
    //                                  parsed_data[i].channel,
    //                                  parsed_data[i].raw_data);
    //                     } else {
    //                         ESP_LOGW(TAG, "Invalid data [ADC%d_Ch%d_%"PRIu32"]",
    //                                  parsed_data[i].unit + 1,
    //                                  parsed_data[i].channel,
    //                                  parsed_data[i].raw_data);
    //                     }
    //                 }
    //             } else {
    //                 ESP_LOGE(TAG, "Data parsing failed: %s", esp_err_to_name(parse_ret));
    //             }

    //             /**
    //              * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
    //              * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
    //              * usually you don't need this delay (as this task will block for a while).
    //              */
    //             vTaskDelay(1);
    //         } else if (ret == ESP_ERR_TIMEOUT) {
    //             //We try to read `READ_LEN` until API returns timeout, which means there's no available data
    //             break;
    //         }
    //     }
    // }
}

void adc_continuous_adc_deinit(adc_continuous_handle_t handle)
{
    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}