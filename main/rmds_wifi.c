#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
//#include "esp_crt_bundle.h"
#include "rmds_wifi.h"

#define WIFI_TAG "RMDS_WIFI"

#define RMDS_WIFI_SSID     "UMBC Visitor"
#define RMDS_WIFI_PASS     ""

#define GOOGLE_SHEETS_URL  "https://script.google.com/macros/s/AKfycbyW9pfPW8bOA1K0cMi4BbOknBrk4bzfjMWjIWcDrxjGn9i3XlunYHBt1oUDk8eg-ueJ4g/exec"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static const int MAX_RETRY = 5;

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_START) {

        esp_wifi_connect();
    }

    else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {

        if (s_retry_num < MAX_RETRY) {

            esp_wifi_connect();
            s_retry_num++;

            ESP_LOGW(WIFI_TAG,
                     "Retrying Wi-Fi connection (%d/%d)",
                     s_retry_num,
                     MAX_RETRY);

        } else {

            xEventGroupSetBits(
                s_wifi_event_group,
                WIFI_FAIL_BIT);
        }
    }

    else if (event_base == IP_EVENT &&
             event_id == IP_EVENT_STA_GOT_IP) {

        ESP_LOGI(WIFI_TAG,
                 "Got IP address");

        s_retry_num = 0;

        xEventGroupSetBits(
            s_wifi_event_group,
            WIFI_CONNECTED_BIT);
    }
}

void rmds_wifi_init(void)
{
    esp_err_t ret;

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

    wifi_init_config_t cfg =
        WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(
        esp_wifi_init(&cfg));

    s_wifi_event_group =
        xEventGroupCreate();

    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &wifi_event_handler,
            NULL,
            NULL));

    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &wifi_event_handler,
            NULL,
            NULL));

    wifi_config_t wifi_config = { 0 };

    snprintf(
        (char *)wifi_config.sta.ssid,
        sizeof(wifi_config.sta.ssid),
        "%s",
        RMDS_WIFI_SSID);

    snprintf(
        (char *)wifi_config.sta.password,
        sizeof(wifi_config.sta.password),
        "%s",
        RMDS_WIFI_PASS);

    wifi_config.sta.threshold.authmode =
        WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(
        esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(
        esp_wifi_set_config(
            WIFI_IF_STA,
            &wifi_config));

    ESP_ERROR_CHECK(
        esp_wifi_start());

    ESP_LOGI(WIFI_TAG,
             "Wi-Fi init done. Connecting to SSID \"%s\"...",
             RMDS_WIFI_SSID);

    EventBits_t bits =
        xEventGroupWaitBits(
            s_wifi_event_group,
            WIFI_CONNECTED_BIT |
            WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {

        ESP_LOGI(WIFI_TAG,
                 "Connected to Wi-Fi, ready for cloud traffic");

    }
    else if (bits & WIFI_FAIL_BIT) {

        ESP_LOGE(WIFI_TAG,
                 "Failed to connect to Wi-Fi");

    }
    else {

        ESP_LOGE(WIFI_TAG,
                 "Unexpected Wi-Fi event bits: 0x%02lx",
                 (unsigned long)bits);
    }
}

static esp_err_t _http_event_handler(
    esp_http_client_event_t *evt)
{
    switch (evt->event_id) {

    case HTTP_EVENT_ON_DATA:
        if (evt->data_len > 0 && evt->data) {
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

void send_frame_to_cloud(
    const char *payload)
{
    if (!payload ||
        payload[0] == '\0') {

        ESP_LOGW(WIFI_TAG,
                 "send_frame_to_cloud: empty payload, skipping");

        return;
    }

    uint32_t concentration = 0;
    uint32_t faults = 0;
    float temperature = 0.0;

    char crc_str[20] = "N/A";

    const char* dataPtr = strstr(payload, "Concentration=");
    if (dataPtr) {
        sscanf(
            dataPtr,
            "Concentration=%" PRIu32 "ppm, Faults=%" PRIu32 ", Sensor Temp=%fK",
            &concentration,
            &faults,
            &temperature);
    }

    const char *crc_pos =
        strstr(payload, "CRC=");

    if (crc_pos) {

        sscanf(
            crc_pos,
            "CRC=%19s",
            crc_str);
    }

    char json_body[512];

    int written =
        snprintf(
            json_body,
            sizeof(json_body),

            "{"
            "\"node_id\":\"Node_TX\","
            "\"concentration\":%" PRIu32 ","
            "\"faults\":%" PRIu32 ","
            "\"temperature\":%.1f,"
            "\"crc\":\"%s\""
            "}",

            concentration,
            faults,
            temperature,
            crc_str);

    if (written <= 0 ||
        written >= (int)sizeof(json_body)) {

        ESP_LOGE(WIFI_TAG,
                 "JSON body too long or error");

        return;
    }

    esp_http_client_config_t config = {

        .url = GOOGLE_SHEETS_URL,
        .method = HTTP_METHOD_POST,
        .event_handler = _http_event_handler,
        .timeout_ms = 20000,
        //.crt_bundle_attach = esp_crt_bundle_attach, /*Ignored for simplicity*/
        .keep_alive_enable = true,
    };

    esp_http_client_handle_t client =
        esp_http_client_init(&config);

    if (!client) {

        ESP_LOGE(WIFI_TAG,
                 "Failed to init HTTP client");

        return;
    }

    esp_http_client_set_header(
        client,
        "Content-Type",
        "application/json");

    /* Add common headers to help the server and for debugging */
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "User-Agent", "RMDS/1.0 esp-idf");

    esp_http_client_set_post_field(
        client,
        json_body,
        strlen(json_body));

    ESP_LOGI(WIFI_TAG,
             "Sending to Google Sheets: %s",
             json_body);

    esp_err_t err =
        esp_http_client_perform(client);

    if (err == ESP_OK) {

        int status =
            esp_http_client_get_status_code(client);

        ESP_LOGI(WIFI_TAG,
                 "HTTP POST status = %d",
                 status);

        if (status == 200 ||
            status == 302) {

            ESP_LOGI(WIFI_TAG,
                     "Data logged to Google Sheets successfully");

        } else {

            ESP_LOGW(WIFI_TAG,
                     "Unexpected status code: %d",
                     status);
        }

    } else {

        ESP_LOGE(WIFI_TAG,
                 "HTTP POST failed: %s",
                 esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}