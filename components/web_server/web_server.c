#include "web_server.h"
// #include "api_registry.h"
// #include "oscillator_handler.h"
// #include "output.h"

#include <string.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_https_server.h"
#include "mdns.h"
#include "cJSON.h"
#include "esp_timer.h"

#define OUTPUT_SAMPLE_BUFFER_SIZE (1024 / 2) // размер буфера с семплами для отправки клиенту
#define MAX_WS_PAYLOAD (1024)
#define CONFIG_HTTPD_MAX_OPEN_SOCKETS (2)

static const char *TAG = "web_server";
static httpd_handle_t server = NULL;
// static httpd_req_t *ws_req = NULL;
static httpd_handle_t ws_req_hd = NULL;
static int ws_req_fd = 0;
static bool ws_connected = false;     // Add connection state tracking
static uint32_t last_ws_activity = 0; // Track last successful WebSocket activity
// Callback function pointer
static void (*ws_recv_frame_callback)(int16_t *samples) = NULL;

// todo переименовать
static int16_t dst[MAX_WS_PAYLOAD / 2];

// lunette.local to connect to the web server
void start_mdns_service()
{
    ESP_LOGI(TAG, "Starting MDNS service");

    // Initialize MDNS service
    esp_err_t err = mdns_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize MDNS: %s", esp_err_to_name(err));
        return;
    }

    ESP_ERROR_CHECK(mdns_hostname_set("lunette"));

    // Set MDNS instance name
    err = mdns_instance_name_set("LUNETTE");
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set MDNS instance name: %s", esp_err_to_name(err));
        return;
    }

    // Add service for HTTPS
    err = mdns_service_add(NULL, "_https", "_tcp", 443, NULL, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add HTTPS service: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "MDNS service started successfully");
}





// static uint8_t shared_buffer[512];
// static _Atomic bool is_busy = false; // Атомарный флаг для защиты

// void fast_broadcast_worker(void *arg) {
//     httpd_handle_t server = (httpd_handle_t)arg;
//     int fds[CONFIG_HTTPD_MAX_OPEN_SOCKETS];
//     size_t clients = CONFIG_HTTPD_MAX_OPEN_SOCKETS;

//     if (httpd_get_client_list(server, &clients, fds) == ESP_OK) {
//         for (size_t i = 0; i < clients; i++) {
//             if (httpd_ws_get_fd_info(server, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
//                 httpd_ws_frame_t ws_pkt = {
//                     .payload = shared_buffer,
//                     .len = 512,
//                     .type = HTTPD_WS_TYPE_BINARY,
//                 };
//                 httpd_ws_send_frame_async(server, fds[i], &ws_pkt);
//             }
//         }
//     }
//     is_busy = false; // Освобождаем буфер
// }

// Вызов из высокочастотного потока
// void send_data_fast(httpd_handle_t server, uint8_t *new_data) {
//     if (is_busy) return; // Пропускаем кадр, если сервер еще занят

//     is_busy = true;
//     memcpy(shared_buffer, new_data, 512);
//     // free(new_data);
    
//     if (httpd_queue_work(server, fast_broadcast_worker, server) != ESP_OK) {
//         is_busy = false;
//     }
// }



// отправляет буфер с выходными значениями на клиента
void web_server_send_samples_to_client(uint8_t *payload)
{
    // if (!ws_connected || !ws_req_hd || ws_req_fd <= 0)
    // {
    //     ESP_LOGD(TAG, "WebSocket not connected, skipping send");
    //     return;
    // }

    // // Check if connection is still valid
    // if (httpd_ws_get_fd_info(ws_req_hd, ws_req_fd) != HTTPD_WS_CLIENT_WEBSOCKET)
    // {
    //     ESP_LOGD(TAG, "WebSocket connection no longer valid");
    //     ws_connected = false;
    //     ws_req_hd = NULL;
    //     ws_req_fd = 0;
    //     return;
    // }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;
    ws_pkt.payload = payload; //(uint8_t *)samples;
    ws_pkt.len = OUTPUT_SAMPLE_BUFFER_SIZE;
    // ws_pkt.final = true;


    httpd_ws_send_frame_async(ws_req_hd, ws_req_fd, &ws_pkt);

    // if (err != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "Failed to send WebSocket frame: %d", err);
    //     if (err == ESP_ERR_HTTPD_INVALID_REQ)
    //     {
    //         ws_connected = false;
    //         ws_req_hd = NULL;
    //         ws_req_fd = 0;
    //     }
    // }
    // else
    // {
    //     last_ws_activity = esp_timer_get_time() / 1000; // Update last activity timestamp
    // }

    // send_data_fast(server, payload);
}

void web_server_register_ws_recv_frame_callback(void (*callback)(int16_t *))
{
    ESP_LOGI(TAG, "Registering WebSocket receive frame callback");
    ws_recv_frame_callback = callback;
}

// WebSocket handler
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        // Clean up any existing connection
        if (ws_connected)
        {
            ESP_LOGI(TAG, "Cleaning up existing WebSocket connection");
            ws_connected = false;
            ws_req_hd = NULL;
            ws_req_fd = 0;
        }

        httpd_ws_frame_t ws_pkt;
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
        ws_pkt.type = HTTPD_WS_TYPE_TEXT;

        ws_req_hd = req->handle;
        ws_req_fd = httpd_req_to_sockfd(req);

        // Verify the connection is valid before marking as connected
        if (httpd_ws_get_fd_info(ws_req_hd, ws_req_fd) == HTTPD_WS_CLIENT_WEBSOCKET)
        {
            ws_connected = true;
            last_ws_activity = esp_timer_get_time() / 1000;
            ESP_LOGI(TAG, "Handshake done, the new connection was opened");
            ESP_LOGI(TAG, "Buffer ready callback registered");
        }
        else
        {
            ESP_LOGE(TAG, "Invalid WebSocket connection");
            ws_req_hd = NULL;
            ws_req_fd = 0;
            return ESP_FAIL;
        }

        return ESP_OK;
    }

    // Handle WebSocket close
    // if (req->method == HTTP_DELETE)
    // {
    // ESP_LOGI(TAG, "WebSocket connection closed");
    // ws_connected = false;
    // ws_req_hd = NULL;
    // ws_req_fd = 0;
    //     return ESP_OK;
    // }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    // ESP_LOGI(TAG, "ws_recv_frame_callback");

    // 1. Сначала узнаем только длину и тип кадра
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get frame len: %d", ret);
        return ret;
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_PONG)
    {
        ESP_LOGI(TAG, "HTTPD_WS_TYPE_PONG");
        return ESP_OK; // Игнорируем или логируем
    }

    if (ws_pkt.len > 0)
    {
        uint8_t *buf = calloc(1, ws_pkt.len);
        if (ws_pkt.len != MAX_WS_PAYLOAD)
        {
            ESP_LOGW(TAG, "WebSocket payload length %d exceeds maximum %d", ws_pkt.len, MAX_WS_PAYLOAD);
        }

        ws_pkt.payload = buf;
        // 2. Читаем сами данные
        httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);

        if (ws_pkt.type == HTTPD_WS_TYPE_BINARY)
        {
            // преобразование потока байт в массив int16_t
            for (int i = 0; i < ws_pkt.len / 2; i++)
            {
                //[i*2] - старший байт, [i*2+1] - младший
                dst[i] = ((int16_t)(ws_pkt.payload[i * 2] << 8)) | ws_pkt.payload[i * 2 + 1];
            }
            ws_recv_frame_callback(dst);
        }
        free(buf);
    }

    return ESP_OK;
}

// Helper function to send file content
static esp_err_t send_file_content(httpd_req_t *req, const char *file_path)
{
    esp_err_t err = ESP_OK;

    return err;
}

// Обработчик для OPTIONS запросов к корневому URL
// возможно не нужен
static esp_err_t root_options_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_send(req, NULL, 0); // Отправляем пустой ответ с кодом 200 OK
    return ESP_OK;
}

// Handler for static files
static esp_err_t static_handler(httpd_req_t *req)
{
    const char *uri = req->uri;
    const char *file_path = uri + 1; // Skip leading '/'
    ESP_LOGD(TAG, "GET %s", uri);
    ESP_LOGD(TAG, "Request headers:");

    // Set content type based on file extension
    const char *content_type = "text/plain";
    if (strstr(file_path, ".js"))
    {
        content_type = "application/javascript";
    }
    else if (strstr(file_path, ".css"))
    {
        content_type = "text/css";
    }
    else if (strstr(file_path, ".html"))
    {
        content_type = "text/html";
    }
    else if (strstr(file_path, ".png"))
    {
        content_type = "image/png";
    }
    else if (strstr(file_path, ".jpg") || strstr(file_path, ".jpeg"))
    {
        content_type = "image/jpeg";
    }
    ESP_LOGD(TAG, "Content-Type: %s", content_type);

    // Set response headers
    httpd_resp_set_type(req, content_type);
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=31536000");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type, Authorization");

    // Send file
    ESP_LOGD(TAG, "Sending file: %s", file_path);
    esp_err_t err = send_file_content(req, file_path);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send file %s: %s", file_path, esp_err_to_name(err));
    }
    return err;
}

// URI matching function with logging
static bool uri_match_fn(const char *reference_uri, const char *uri_to_match, size_t match_upto)
{
    // Handle wildcard pattern
    const char *wildcard = strchr(reference_uri, '*');
    if (wildcard != NULL)
    {
        size_t prefix_len = wildcard - reference_uri;
        // If we have a length limit, use it
        if (match_upto > 0)
        {
            prefix_len = (prefix_len < match_upto) ? prefix_len : match_upto;
        }
        // Match the part before wildcard
        bool match = (strncmp(reference_uri, uri_to_match, prefix_len) == 0);
        ESP_LOGD(TAG, "Wildcard match - Prefix length: %zu, Result: %s", prefix_len, match ? "true" : "false");
        return match;
    }

    // No wildcard, do normal matching
    bool match = false;
    if (match_upto == 0)
    {
        match = (strcmp(reference_uri, uri_to_match) == 0);
    }
    else
    {
        match = (strncmp(reference_uri, uri_to_match, match_upto) == 0);
    }

    ESP_LOGD(TAG, "URI Match Result: %s", match ? "true" : "false");
    return match;
}

// Start web server with custom configuration
httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = uri_match_fn; // Set our custom URI matching function
    config.max_open_sockets = 1;        // Allow multiple WebSocket connections

    esp_err_t err;

    // Regular HTTP server
    err = httpd_start(&server, &config);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error starting server!");
        return NULL;
    }

    // URI handler for WebSocket endpoint
    httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .is_websocket = true,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &ws_uri);

    ESP_LOGI(TAG, "Server started successfully");
    return server;
}

// Initialize and start web server
esp_err_t web_server_init(void)
{
    ESP_LOGD(TAG, "Initializing web server...");

    // Set log level to DEBUG to enable LOGD messages
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    ESP_LOGD(TAG, "Debug logging enabled");

    if (server != NULL)
    {
        ESP_LOGW(TAG, "Web server already running");
        return ESP_OK;
    }

    // Start mDNS service
    start_mdns_service();

    // Start web server with configuration
    server = start_webserver();
    if (server == NULL)
    {
        ESP_LOGE(TAG, "Failed to start web server");
        return ESP_FAIL;
    }

    return ESP_OK;
}

// Stop and deinitialize web server
esp_err_t web_server_deinit(void)
{
    ESP_LOGD(TAG, "Deinitializing web server...");
    if (server == NULL)
    {
        ESP_LOGW(TAG, "Web server not running");
        return ESP_OK;
    }

    esp_err_t err = stop_webserver(server);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to stop server: %s", esp_err_to_name(err));
        return err;
    }

    server = NULL;
    ESP_LOGD(TAG, "Web server stopped successfully");
    return ESP_OK;
}
