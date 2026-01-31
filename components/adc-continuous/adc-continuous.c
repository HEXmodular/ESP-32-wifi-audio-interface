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
#define ADC_ATTEN ADC_ATTEN_DB_0 // ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH

#define READ_LEN 1024 //(8192)

#if CONFIG_IDF_TARGET_ESP32
static adc_channel_t channel[2] = {ADC_CHANNEL_6, ADC_CHANNEL_4};
#else
static adc_channel_t channel[2] = {ADC_CHANNEL_2, ADC_CHANNEL_3};
#endif

static TaskHandle_t s_task_handle;
static const char *TAG = "ADC-CONTINUOUS";
static uint8_t adc_data[READ_LEN / 2];


// Callback function pointer
static void (*buffer_ready_callback)(uint8_t *samples) = NULL;

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    // Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = READ_LEN * 4,
        .conv_frame_size = READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 48 * 1000,
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
void adc_continuous_register_buffer_ready_callback(void (*callback)(uint8_t *samples))
{
    ESP_LOGI(TAG, "Registering buffer ready callback");
    buffer_ready_callback = callback;
}

// 2. Основная задача по обработке данных
void adc_processing_task(void *pvParameters)
{
    adc_continuous_handle_t handle = (adc_continuous_handle_t)pvParameters;
    uint8_t result[READ_LEN] = {0};
    uint32_t ret_num = 0;

    while (1)
    {
        // Ждем уведомления от прерывания (ждем бесконечно)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Читаем данные. Поскольку callback уже сказал, что данные есть,
        // adc_continuous_read отработает мгновенно.
        esp_err_t ret = adc_continuous_read(handle, result, READ_LEN, &ret_num, 0);

        if (ret == ESP_OK)
        {
            // Здесь ваша логика обработки: фильтрация, FFT или вывод
            // ESP_LOGI("ADC", "Read %"PRIu32" bytes", ret_num);

            // Пример доступа к первому значению
            // adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[0];
            // printf("Val: %d\n", p->type1.data);

            adc_continuous_data_t parsed_data[ret_num / SOC_ADC_DIGI_RESULT_BYTES];
            uint32_t num_parsed_samples = 0;

            esp_err_t parse_ret = adc_continuous_parse_data(handle, result, ret_num, parsed_data, &num_parsed_samples);
            if (parse_ret == ESP_OK)
            {

                // uint8_t raw_data[num_parsed_samples];
                uint8_t *raw_data = calloc(1, num_parsed_samples);
                for (int i = 0; i < num_parsed_samples / 2; i++)
                {
                    raw_data[i] = (uint8_t)(parsed_data[i * 2].raw_data >> 4);

                    raw_data[i + num_parsed_samples / 2] = (uint8_t)(parsed_data[i * 2 + 1].raw_data >> 4);
                }
                // 4095

                // for (int i = 0; i < num_parsed_samples; i++)

                // ESP_LOGI(TAG, "Samples: %d ADC%d, Channel: %d, Value: %" PRIu32 " Raw: %d",
                //          num_parsed_samples,
                //          parsed_data[2].unit + 1,
                //          parsed_data[2].channel,
                //          //  raw_data[0]);
                //          parsed_data[2].raw_data,
                //          (uint8_t)(parsed_data[2].raw_data >> 4));
                // ESP_LOGI(TAG, "ADC%d, Channel: %d, Value: %" PRIu32 " Raw: %d",
                //          parsed_data[3].unit + 1,
                //          parsed_data[3].channel,
                //          parsed_data[3].raw_data,
                //          (uint8_t)(parsed_data[3].raw_data >> 4));

                memcpy(adc_data, raw_data, num_parsed_samples);
                buffer_ready_callback(adc_data);
                free(raw_data);
            }
            else
            {
                ESP_LOGE(TAG, "Data parsing failed: %s", esp_err_to_name(parse_ret));
            }
        }
    }
}

void adc_continuous_init(void)
{
    // esp_err_t ret;
    // uint32_t ret_num = 0;
    // uint8_t result[READ_LEN] = {0};
    // memset(result, 0xcc, READ_LEN);

    s_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));

    xTaskCreate(adc_processing_task, "adc_task", 4096 * 4, handle, 5, &s_task_handle);

    ESP_ERROR_CHECK(adc_continuous_start(handle));
}

void adc_continuous_adc_deinit(adc_continuous_handle_t handle)
{
    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}