#include "rmds_wifi.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#define WIFI_TAG "RMDS_WIFI"

#define RMDS_WIFI_SSID "UMBC Visitor"
#define RMDS_WIFI_PASS ""

#define GOOGLE_SHEETS_URL "https://script.google.com/macros/s/AKfycbyW9pfPW8bOA1K0cMi4BbOknBrk4bzfjMWjIWcDrxjGn9i3XlunYHBt1oUDk8eg-ueJ4g/exec"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define RMDS_WIFI_CLOUD_QUEUE_LEN 64
#define RMDS_WIFI_RETRY_DELAY_MS  2000

static EventGroupHandle_t s_wifi_event_group;
static QueueHandle_t s_cloud_queue;
static esp_event_handler_instance_t s_wifi_any_id_handler;
static esp_event_handler_instance_t s_wifi_got_ip_handler;
static int s_retry_num = 0;
static const int MAX_RETRY = 5;
static bool s_wifi_ready = false;
static bool s_wifi_stack_initialized = false;

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_ready = false;

        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;

            ESP_LOGW(WIFI_TAG,
                     "Retrying Wi-Fi connection (%d/%d)",
                     s_retry_num,
                     MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(WIFI_TAG, "Got IP address");
        s_retry_num = 0;
        s_wifi_ready = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t rmds_wifi_http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ON_DATA:
        if (evt->data_len > 0 && evt->data != NULL) {
            ESP_LOGI(WIFI_TAG,
                     "HTTP_EVENT_ON_DATA len=%d: %.*s",
                     evt->data_len,
                     evt->data_len,
                     (const char *)evt->data);
        }
        break;

    case HTTP_EVENT_ON_FINISH:
        ESP_LOGI(WIFI_TAG, "HTTP_EVENT_ON_FINISH");
        break;

    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGW(WIFI_TAG, "HTTP_EVENT_DISCONNECTED");
        break;

    default:
        break;
    }

    return ESP_OK;
}

static bool rmds_wifi_send_frame_to_cloud(const rmds_wifi_cloud_frame_t *frame)
{
    char json_body[512];
    int written;
    esp_http_client_config_t config;
    esp_http_client_handle_t client;
    esp_err_t err;
    bool success = false;

    if (frame == NULL) {
        return false;
    }

    written = snprintf(
        json_body,
        sizeof(json_body),
        "{"
        "\"node_id\":\"Node_%u\","
        "\"network_seq\":%" PRIu32 ","
        "\"concentration\":%" PRIu32 ","
        "\"faults\":%" PRIu32 ","
        "\"temperature\":%.1f,"
        "\"sensor_crc\":\"%08" PRIx32 "\","
        "\"sensor_crc_inv\":\"%08" PRIx32 "\","
        "\"rssi_dbm\":%d,"
        "\"snr_db\":%.2f"
        "}",
        (unsigned int)frame->node_id,
        frame->network_seq,
        frame->concentration_ppm,
        frame->faults,
        frame->temperature_k,
        frame->sensor_crc,
        frame->sensor_crc_inv,
        frame->rssi_dbm,
        frame->snr_db);

    if (written <= 0 || written >= (int)sizeof(json_body)) {
        ESP_LOGE(WIFI_TAG, "JSON body too long or error");
        return false;
    }

    memset(&config, 0, sizeof(config));
    config.url = GOOGLE_SHEETS_URL;
    config.method = HTTP_METHOD_POST;
    config.event_handler = rmds_wifi_http_event_handler;
    config.timeout_ms = 20000;
    config.keep_alive_enable = true;

    client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(WIFI_TAG, "Failed to init HTTP client");
        return false;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "User-Agent", "RMDS/1.0 esp-idf");
    esp_http_client_set_post_field(client, json_body, strlen(json_body));

    ESP_LOGI(WIFI_TAG, "Sending to Google Sheets: %s", json_body);
    err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);

        ESP_LOGI(WIFI_TAG, "HTTP POST status = %d", status);
        if (status == 200 || status == 302) {
            ESP_LOGI(WIFI_TAG, "Data logged to Google Sheets successfully");
            success = true;
        } else {
            ESP_LOGW(WIFI_TAG, "Unexpected status code: %d", status);
        }
    } else {
        ESP_LOGE(WIFI_TAG, "HTTP POST failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return success;
}

static void rmds_wifi_cloud_task(void *pvParameters)
{
    rmds_wifi_cloud_frame_t frame;

    (void)pvParameters;

    while (1) {
        if (xQueueReceive(s_cloud_queue, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (!s_wifi_ready) {
            ESP_LOGW(WIFI_TAG,
                     "Wi-Fi not ready; retrying frame node=%u seq=%" PRIu32,
                     (unsigned int)frame.node_id,
                     frame.network_seq);
            vTaskDelay(pdMS_TO_TICKS(RMDS_WIFI_RETRY_DELAY_MS));
            xQueueSendToFront(s_cloud_queue, &frame, pdMS_TO_TICKS(100));
            continue;
        }

        if (!rmds_wifi_send_frame_to_cloud(&frame)) {
            ESP_LOGW(WIFI_TAG,
                     "Cloud send failed; requeueing node=%u seq=%" PRIu32,
                     (unsigned int)frame.node_id,
                     frame.network_seq);
            vTaskDelay(pdMS_TO_TICKS(RMDS_WIFI_RETRY_DELAY_MS));
            xQueueSendToFront(s_cloud_queue, &frame, pdMS_TO_TICKS(100));
        }
    }
}

void rmds_wifi_init(void)
{
    esp_err_t ret;
    EventBits_t bits;
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifi_config = { 0 };

    if (s_wifi_stack_initialized) {
        return;
    }

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        ESP_LOGE(WIFI_TAG, "Failed to create Wi-Fi event group");
        return;
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        &s_wifi_any_id_handler));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &wifi_event_handler,
        NULL,
        &s_wifi_got_ip_handler));

    snprintf((char *)wifi_config.sta.ssid,
             sizeof(wifi_config.sta.ssid),
             "%s",
             RMDS_WIFI_SSID);
    snprintf((char *)wifi_config.sta.password,
             sizeof(wifi_config.sta.password),
             "%s",
             RMDS_WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(WIFI_TAG, "Wi-Fi init done. Connecting to SSID \"%s\"...", RMDS_WIFI_SSID);

    bits = xEventGroupWaitBits(
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

    s_cloud_queue = xQueueCreate(RMDS_WIFI_CLOUD_QUEUE_LEN, sizeof(rmds_wifi_cloud_frame_t));
    if (s_cloud_queue == NULL) {
        ESP_LOGE(WIFI_TAG, "Failed to create cloud queue");
        return;
    }

    if (xTaskCreate(rmds_wifi_cloud_task,
                    "rmds_wifi_cloud_task",
                    6144,
                    NULL,
                    4,
                    NULL) != pdPASS) {
        ESP_LOGE(WIFI_TAG, "Failed to create cloud upload task");
        return;
    }

    s_wifi_stack_initialized = true;
}

bool rmds_wifi_enqueue_frame(const rmds_wifi_cloud_frame_t *frame)
{
    if (s_cloud_queue == NULL || frame == NULL) {
        return false;
    }

    if (xQueueSend(s_cloud_queue, frame, 0) != pdTRUE) {
        ESP_LOGW(WIFI_TAG,
                 "Cloud queue full, dropping frame from node=%u seq=%" PRIu32,
                 (unsigned int)frame->node_id,
                 frame->network_seq);
        return false;
    }

    return true;
}

bool rmds_wifi_is_ready(void)
{
    return s_wifi_ready;
}