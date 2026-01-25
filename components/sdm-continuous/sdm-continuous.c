/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/sdm.h"
#include "driver/gptimer.h"

#define KHZ (1000)
#define MHZ (1000 * 1000)
#define CONST_PI (3.1416f)                     // Constant of PI, used for calculating the sine wave
#define SIGMA_DELTA_GPIO_NUMS {23, 22, 21, 19} // Select GPIOs for sigma-delta output pins
#define SIGMA_DELTA_GPIO_LEN (4)
#define OVER_SAMPLE_RATE (80 * MHZ) // 10 MHz over sample rate (частота для SDM канала)
#define TIMER_RESOLUTION (48 * KHZ) // timer counting resolution
#define CALLBACK_INTERVAL_US (1000000 / TIMER_RESOLUTION)  // 100 us interval of each timer callback
#define ALARM_COUNT 1            //(CALLBACK_INTERVAL_US * (TIMER_RESOLUTION / MHZ)) // настройка таймера

#define FULL_BUF (1 * 256 * 4 * 2) // x2 two half ping-pong buffers
#define HALF_BUF (FULL_BUF / 2)
#define PER_CHANNEL_BUF (HALF_BUF / 4)

static const char *TAG = "SDM-DAC";

// пинг понг буфер для семплов
static int16_t pp_buffer[FULL_BUF] = {0};
static uint8_t active_read_half = 0; // Какую половину сейчас читаем (0 или 1)

// Чтение по ОДНОМУ значению
static int8_t read_next_sample(int channel_num)
{
    static uint16_t read_ptr = 0; // Индекс для чтения по одному
    int8_t sample = pp_buffer[read_ptr + (active_read_half * HALF_BUF) + (channel_num * PER_CHANNEL_BUF)];

    if (channel_num == SIGMA_DELTA_GPIO_LEN - 1)
    {
        read_ptr++;
    }

    // Если вычитали всю текущую половину — переключаемся
    if (read_ptr >= PER_CHANNEL_BUF)
    {
        read_ptr = 0;
        active_read_half = !active_read_half; // Инвертируем 0 -> 1 или 1 -> 0
    }

    // NB Приведение int16_t к int8_t с усечением старших бит
    // возможно в будущем будет повышение битности
    return (int8_t)sample;
}

static bool IRAM_ATTR timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    // static uint32_t cnt = 0;
    sdm_channel_handle_t *sdm_chans = (sdm_channel_handle_t *)user_ctx;
    /* Set the pulse density */
    // Assuming SIGMA_DELTA_GPIO_NUMS defines the number of channels
    // This needs to be a fixed size or passed dynamically
    for (int i = 0; i < SIGMA_DELTA_GPIO_LEN; i++)
    {
        sdm_channel_set_pulse_density(sdm_chans[i], read_next_sample(i));
    }
    return false;
}

static gptimer_handle_t init_gptimer(void *args)
{
    /* Allocate GPTimer handle */
    gptimer_handle_t timer_handle;
    gptimer_config_t timer_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_cfg, &timer_handle));
    ESP_LOGI(TAG, "Timer allocated with resolution %d Hz", TIMER_RESOLUTION);

    /* Set the timer alarm configuration */
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = ALARM_COUNT,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_handle, &alarm_cfg));

    /* Register the alarm callback */
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_handle, &cbs, args));
    ESP_LOGI(TAG, "Timer callback registered, interval %d us", CALLBACK_INTERVAL_US);

    /* Clear the timer raw count value, make sure it'll count from 0 */
    ESP_ERROR_CHECK(gptimer_set_raw_count(timer_handle, 0));
    /* Enable the timer */
    ESP_ERROR_CHECK(gptimer_enable(timer_handle));

    ESP_LOGI(TAG, "Timer enabled");

    return timer_handle;
}

static sdm_channel_handle_t *init_sdm(void)
{
    int sdm_channels[] = SIGMA_DELTA_GPIO_NUMS;
    static sdm_channel_handle_t sdm_chans[SIGMA_DELTA_GPIO_LEN];

    for (int i = 0; i < SIGMA_DELTA_GPIO_LEN; i++)
    {
        // ESP_LOGI(TAG, "SDM Channel %d on GPIO %d", i, sdm_channels[i]);
        /* Allocate sdm channel handle */
        sdm_config_t config = {
            .clk_src = SDM_CLK_SRC_DEFAULT,
            .gpio_num = sdm_channels[i],
            .sample_rate_hz = OVER_SAMPLE_RATE,
        };
        ESP_ERROR_CHECK(sdm_new_channel(&config, &sdm_chans[i]));
        /* Enable the sdm channel */
        ESP_ERROR_CHECK(sdm_channel_enable(sdm_chans[i]));

        ESP_LOGI(TAG, "Sigma-delta output is attached to GPIO %d", sdm_channels[i]);
    }

    return sdm_chans;
}

// Запись сразу ПОЛОВИНЫ данных (1024 семпла)
void sdm_continuous_write_to_channels(int16_t *samples)
{
    // Пишем в ту половину, которая СЕЙЧАС НЕ читается
    int16_t write_offset = (active_read_half == 0) ? HALF_BUF : 0;
    memcpy(&pp_buffer[write_offset], samples, HALF_BUF * sizeof(int16_t));
}

void sdm_dac_init(void)
{
    /* Initialize sigma-delta modulation on the specific GPIO */
    sdm_channel_handle_t *sdm_chans = init_sdm();
    /* Initialize GPTimer and register the timer alarm callback */
    gptimer_handle_t timer_handle = init_gptimer(sdm_chans);
    /* Start the GPTimer */
    ESP_LOGI(TAG, "Output start");
    ESP_ERROR_CHECK(gptimer_start(timer_handle));
}