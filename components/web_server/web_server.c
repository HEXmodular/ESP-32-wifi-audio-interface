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

#define OUTPUT_SAMPLE_BUFFER_SIZE 256

static const char *TAG = "web_server";
static httpd_handle_t server = NULL;
// static httpd_req_t *ws_req = NULL;
static httpd_handle_t ws_req_hd = NULL;
static int ws_req_fd = 0;
static bool ws_connected = false;     // Add connection state tracking
static uint32_t last_ws_activity = 0; // Track last successful WebSocket activity
// Callback function pointer
static void (*ws_recv_frame_callback)(uint16_t* samples) = NULL;
// WebSocket frame receive buffer
// #define WS_BUFFER_SIZE 1024
// static uint8_t ws_buffer[WS_BUFFER_SIZE];

// static output_handle_t output = NULL;

// execute_buffer_ready_callback

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

// отправляет буфер с выходными значениями на клиента
void web_server_send_samples_to_client(uint8_t *payload)
{
    if (!ws_connected || !ws_req_hd || ws_req_fd <= 0)
    {
        ESP_LOGD(TAG, "WebSocket not connected, skipping send");
        return;
    }

    // Check if connection is still valid
    if (httpd_ws_get_fd_info(ws_req_hd, ws_req_fd) != HTTPD_WS_CLIENT_WEBSOCKET)
    {
        ESP_LOGD(TAG, "WebSocket connection no longer valid");
        ws_connected = false;
        ws_req_hd = NULL;
        ws_req_fd = 0;
        return;
    }

    // if (!output)
    // {
    //     ESP_LOGE(TAG, "Output instance not found");
    //     return;
    // }

    // if (!output_samples_ready(output))
    // {
    //     ESP_LOGD(TAG, "Samples are not ready");
    //     return;
    // }

    // int8_t samples[OUTPUT_SAMPLE_BUFFER_SIZE];
    // esp_err_t err = output_get_samples(output, samples, OUTPUT_SAMPLE_BUFFER_SIZE);
    // if (err != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "Failed to get samples: %d", err);
    //     return;
    // }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;
    ws_pkt.payload = payload; //(uint8_t *)samples;
    ws_pkt.len = OUTPUT_SAMPLE_BUFFER_SIZE;
    ws_pkt.final = true;

    // err =
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
}

void web_server_register_ws_recv_frame_callback(void (*callback)(uint16_t *))
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

            // output = output_get_instance();
            // if (!output)
            // {
            //     ESP_LOGE(TAG, "Failed to get output instance");
            //     ws_connected = false;
            //     ws_req_hd = NULL;
            //     ws_req_fd = 0;
            //     return ESP_FAIL;
            // }

            // output_register_buffer_ready_callback(&send_samples_to_client);
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
    if (req->method == HTTP_DELETE)
    {
        ESP_LOGI(TAG, "WebSocket connection closed");
        ws_connected = false;
        ws_req_hd = NULL;
        ws_req_fd = 0;
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

    // 1. Получаем длину кадра --------------------------------------------------------------------------------------------------------------
    httpd_ws_recv_frame(req, &ws_pkt, 0);

    if (ws_pkt.len > 0)
    {
        uint16_t *buf = calloc(1, ws_pkt.len);
        ws_pkt.payload = buf;
        // 2. Читаем сами данные
        httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);

        if (ws_pkt.type == HTTPD_WS_TYPE_BINARY)
        {
            // Работа с бинарными данными в buf
            ws_recv_frame_callback(ws_pkt.payload);
        }
        free(buf);
    }

    return ESP_OK;
}

// Helper function to send file content
static esp_err_t send_file_content(httpd_req_t *req, const char *file_path)
{
    ESP_LOGD(TAG, "Opening file: %s", file_path);

    // extern const unsigned char index_html_start[] asm("_binary_index_html_start");
    // extern const unsigned char index_html_end[] asm("_binary_index_html_end");
    // extern const unsigned char index_js_start[] asm("_binary_index_js_start");
    // extern const unsigned char index_js_end[] asm("_binary_index_js_end");
    // extern const unsigned char index_css_start[] asm("_binary_index_css_start");
    // extern const unsigned char index_css_end[] asm("_binary_index_css_end");
    // extern const unsigned char audio_worklet_js_start[] asm("_binary_audio_worklet_js_start");
    // extern const unsigned char audio_worklet_js_end[] asm("_binary_audio_worklet_js_end");
    // extern const unsigned char vite_svg_start[] asm("_binary_vite_svg_start");
    // extern const unsigned char vite_svg_end[] asm("_binary_vite_svg_end");

    // const unsigned char *start = NULL;
    // const unsigned char *end = NULL;
    esp_err_t err = ESP_OK;

    // // Determine which embedded file to serve
    // if (strcmp(file_path, "index.html") == 0)
    // {
    //     start = index_html_start;
    //     end = index_html_end;
    //     httpd_resp_set_type(req, "text/html");
    // }
    // else if (strcmp(file_path, "index.js") == 0)
    // {
    //     start = index_js_start;
    //     end = index_js_end;
    //     httpd_resp_set_type(req, "application/javascript");
    // }
    // else if (strcmp(file_path, "index.css") == 0)
    // {
    //     start = index_css_start;
    //     end = index_css_end;
    //     httpd_resp_set_type(req, "text/css");
    // }
    // else if (strcmp(file_path, "audio-worklet.js") == 0)
    // {
    //     start = audio_worklet_js_start;
    //     end = audio_worklet_js_end;
    //     httpd_resp_set_type(req, "application/javascript");
    // }
    // else if (strcmp(file_path, "vite.svg") == 0)
    // {
    //     start = vite_svg_start;
    //     end = vite_svg_end;
    //     httpd_resp_set_type(req, "image/svg+xml");
    // }
    // else
    // {
    //     ESP_LOGE(TAG, "File not found: %s", file_path);
    //     return ESP_FAIL;
    // }
    // // Send the file content
    // ESP_LOGD(TAG, "File found");
    // if (start && end)
    // {
    //     size_t file_size = end - start;
    //     ESP_LOGD(TAG, "File size: %zu bytes", file_size);
    //     err = httpd_resp_send(req, (const char *)start, file_size);
    //     if (err != ESP_OK)
    //     {
    //         ESP_LOGE(TAG, "Error sending file: %s", esp_err_to_name(err));
    //     }
    // }
    // else
    // {
    //     ESP_LOGE(TAG, "File content not found");
    //     err = ESP_FAIL;
    // }

    return err;
}

// Handler for root path
// static esp_err_t root_handler(httpd_req_t *req)
// {
//     ESP_LOGD(TAG, "GET /");
//     ESP_LOGD(TAG, "Request headers:");

//     // Set response headers
//     httpd_resp_set_type(req, "text/html");
//     httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
//     httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
//     httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
//     httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type, Authorization");

//     // Send index.html
//     ESP_LOGD(TAG, "Sending index.html");
//     esp_err_t err = send_file_content(req, "index.html");
//     if (err != ESP_OK)
//     {
//         ESP_LOGE(TAG, "Failed to send index.html: %s", esp_err_to_name(err));
//     }
//     return err;
// }

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
    // httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    // config.lru_purge_enable = true;
    // config.uri_match_fn = uri_match_fn;  // Set our custom URI matching function
    // config.max_open_sockets = 4;  // Allow multiple WebSocket connections

    // ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);

    esp_err_t err;

    httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();
    // conf.httpd.max_open_sockets = max_clients;
    // conf.httpd.global_user_ctx = keep_alive;
    // conf.httpd.open_fn = wss_open_fd;
    // conf.httpd.close_fn = wss_close_fd;
    conf.httpd.uri_match_fn = uri_match_fn; // Set our custom URI matching function
    conf.httpd.max_uri_handlers = 16;       // Increase maximum number of URI handlers
    conf.httpd.max_open_sockets = 7;        // Increase maximum number of open sockets

    extern const unsigned char certificate_pem_start[] asm("_binary_certificate_pem_start");
    extern const unsigned char certificate_pem_end[] asm("_binary_certificate_pem_end");
    size_t cert_len = certificate_pem_end - certificate_pem_start;
    conf.servercert = certificate_pem_start;
    conf.servercert_len = cert_len;

    extern const unsigned char private_key_pem_start[] asm("_binary_private_key_pem_start");
    extern const unsigned char private_key_pem_end[] asm("_binary_private_key_pem_end");
    conf.prvtkey_pem = private_key_pem_start;
    conf.prvtkey_len = private_key_pem_end - private_key_pem_start;

    esp_err_t ret = httpd_ssl_start(&server, &conf);
    if (ESP_OK != ret)
    {
        ESP_LOGI(TAG, "Error starting server!");
        return NULL;
    }

    // Regular HTTP server
    // err = httpd_start(&server, &config);

    // if (err != ESP_OK) {
    //     ESP_LOGE(TAG, "Error starting server!");
    //     return NULL;
    // }

    // URI handler for root path
    // httpd_uri_t root_uri = {
    //     .uri = "/",
    //     .method = HTTP_GET,
    //     .handler = root_handler,
    //     .user_ctx = NULL};
    // httpd_register_uri_handler(server, &root_uri);

    // // Структура для регистрации OPTIONS обработчика
    // httpd_uri_t root_options_uri = {
    //     .uri = "/",
    //     .method = HTTP_OPTIONS,
    //     .handler = root_options_handler,
    //     .user_ctx = NULL};
    // httpd_register_uri_handler(server, &root_options_uri);

    // URI handler for WebSocket endpoint
    httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .is_websocket = true,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &ws_uri);

    // Initialize API registry
    // err = api_registry_init(server);
    // if (err != ESP_OK)
    // {
    //     ESP_LOGE(TAG, "Error initializing API registry!");
    //     httpd_stop(server);
    //     return NULL;
    // }

    // URI handler for static files
    // httpd_uri_t static_uri = {
    //     .uri = "/*",
    //     .method = HTTP_GET,
    //     .handler = static_handler,
    //     .user_ctx = NULL};
    // httpd_register_uri_handler(server, &static_uri);

    ESP_LOGI(TAG, "Server started successfully");
    return server;
}

// Add periodic connection check task
static void check_ws_connection(void *arg)
{
    while (1)
    {
        if (ws_connected && ws_req_hd && ws_req_fd > 0)
        {
            uint32_t current_time = esp_timer_get_time() / 1000;
            // If no activity for more than 5 seconds, consider connection dead
            if (current_time - last_ws_activity > 5000)
            {
                ESP_LOGI(TAG, "WebSocket connection timeout");
                ws_connected = false;
                ws_req_hd = NULL;
                ws_req_fd = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }
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

    // Create task to check WebSocket connection
    xTaskCreate(check_ws_connection, "ws_check", 2048, NULL, 5, NULL);

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
