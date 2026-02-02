// rmds_wifi.c

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_client.h"

#include "rmds_wifi.h"

#define WIFI_TAG "RMDS_WIFI"

// ***** EDIT THESE FOR YOUR SETUP *****
#define RMDS_WIFI_SSID     "UMBC Visitor"
#define RMDS_WIFI_PASS     ""

// Your backend / API endpoint that talks to MongoDB Atlas
// (for testing you can point this at a mock HTTP server)
#define RMDS_CLOUD_URL     "https://your-backend.example.com/api/sensor"

// Optional API key header for your backend (leave empty if not used)
#define RMDS_CLOUD_API_KEY ""  // e.g. "my-secret-key"

// Event bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static const int MAX_RETRY = 5;

// --------------------
// Wi-Fi event handler
// --------------------
static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(WIFI_TAG, "Retrying Wi-Fi connection (%d/%d)",
                     s_retry_num, MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT &&
               event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(WIFI_TAG, "Got IP address");
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// --------------------
// Wi-Fi init / connect
// --------------------
void rmds_wifi_init(void)
{
    esp_err_t ret;

    // Initialize NVS (required for Wi-Fi)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &wifi_event_handler,
        NULL,
        NULL));

    wifi_config_t wifi_config = { 0 };
    snprintf((char *)wifi_config.sta.ssid,
             sizeof(wifi_config.sta.ssid), "%s", RMDS_WIFI_SSID);
    snprintf((char *)wifi_config.sta.password,
             sizeof(wifi_config.sta.password), "%s", RMDS_WIFI_PASS);

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(WIFI_TAG,
             "Wi-Fi init done. Connecting to SSID \"%s\"...",
             RMDS_WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(WIFI_TAG, "Connected to Wi-Fi, ready for cloud traffic");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(WIFI_TAG, "Failed to connect to Wi-Fi");
    } else {
        ESP_LOGE(WIFI_TAG, "Unexpected Wi-Fi event bits: 0x%02lx", (unsigned long)bits);
    }
}

// ---------------------
// HTTP POST helper
// ---------------------
static esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ON_DATA:
        // You could print response here if you care
        break;
    default:
        break;
    }
    return ESP_OK;
}

// ---------------------
// Public cloud API
// ---------------------
void send_frame_to_cloud(const char *payload)
{
    if (!payload || payload[0] == '\0') {
        ESP_LOGW(WIFI_TAG, "send_frame_to_cloud: empty payload, skipping");
        return;
    }

    // Build a small JSON body wrapping the LoRa payload
    // NOTE: If your payload can contain quotes or special chars,
    // you'd want to escape them properly.
    char json_body[512];
    int written = snprintf(json_body, sizeof(json_body),
                           "{\"raw\":\"%s\"}", payload);
    if (written <= 0 || written >= (int)sizeof(json_body)) {
        ESP_LOGE(WIFI_TAG, "JSON body too long or error");
        return;
    }

    esp_http_client_config_t config = {
        .url = RMDS_CLOUD_URL,
        .method = HTTP_METHOD_POST,
        .event_handler = _http_event_handler,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(WIFI_TAG, "Failed to init HTTP client");
        return;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");

    if (strlen(RMDS_CLOUD_API_KEY) > 0) {
        esp_http_client_set_header(client, "X-API-Key", RMDS_CLOUD_API_KEY);
    }

    esp_http_client_set_post_field(client, json_body, strlen(json_body));

    ESP_LOGI(WIFI_TAG, "Sending payload to cloud: %s", json_body);

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        ESP_LOGI(WIFI_TAG, "HTTP POST done, status = %d", status);
    } else {
        ESP_LOGE(WIFI_TAG, "HTTP POST failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}
