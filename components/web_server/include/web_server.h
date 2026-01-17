#pragma once

#include <esp_http_server.h>
#include "esp_err.h"


// TODO удалить эту порнографию нафиг

// /**
//  * @brief Configuration structure for the web server
//  */
// typedef struct {
//     uint16_t port;                    /*!< Port to listen on */
//     uint16_t max_uri_handlers;        /*!< Maximum number of URI handlers */
//     uint16_t max_resp_headers;        /*!< Maximum number of response headers */
//     uint16_t max_req_headers_len;     /*!< Maximum length of request headers */
//     uint16_t max_uri_len;             /*!< Maximum length of URI */
//     uint16_t max_post_len;            /*!< Maximum length of POST data */
//     uint16_t max_out_headers;         /*!< Maximum number of output headers */
//     uint16_t max_sess_ctx;            /*!< Maximum number of session contexts */
//     uint16_t max_open_sockets;        /*!< Maximum number of open sockets */
//     uint16_t backlog_conn;            /*!< Number of backlog connections */
//     bool lru_purge_enable;            /*!< Enable LRU purge */
//     uint16_t recv_wait_timeout;       /*!< Timeout for receiving data */
//     uint16_t send_wait_timeout;       /*!< Timeout for sending data */
//     bool use_ssl;                     /*!< Enable SSL/TLS */
//     const char *cert_pem;             /*!< Server certificate in PEM format */
//     const char *private_key_pem;      /*!< Server private key in PEM format */
//     bool use_global_ca_store;         /*!< Use global CA store for client verification */
//     const char *client_cert_pem;      /*!< Client certificate in PEM format for mutual auth */
//     const char *client_key_pem;       /*!< Client private key in PEM format for mutual auth */
// } web_server_config_t;


// #define SCRATCH_BUFSIZE (10240)

// typedef struct rest_server_context {
//     char scratch[SCRATCH_BUFSIZE];
// } rest_server_context_t;

/**
 * @brief Start the web server with custom configuration
 * 
 * @param config Server configuration
 * @return httpd_handle_t Server handle on success, NULL on failure
 */
httpd_handle_t start_webserver(void);

/**
 * @brief Stop the web server
 * 
 * @param server Server handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t stop_webserver(httpd_handle_t server);

/**
 * @brief Initialize and start the web server
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t web_server_init(void);

/**
 * @brief Stop and deinitialize the web server
 * 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t web_server_deinit(void);

// отправляет буфер с выходными значениями на клиента
void web_server_send_samples_to_client(uint8_t *payload);

void web_server_register_ws_recv_frame_callback(void (*callback)(uint16_t *));
