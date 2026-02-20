#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_sleep.h"
#include "esp_pm.h"

static const char *TAG = "POWER";

// Save data across sleep
RTC_DATA_ATTR uint32_t boot_count = 0;

// Your LoRa pins
#define LORA_DIO0_PIN  GPIO_NUM_26  // Wake pin (must be RTC-capable)

// ================================================================
// 1. MODEM SLEEP - CPU on, WiFi/BT off (20-25 mA)
// ================================================================
void enter_modem_sleep(void)
{
    // Turn off WiFi and Bluetooth
    esp_wifi_stop();
    esp_wifi_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    
    // Set CPU to 80 MHz
    esp_pm_config_esp32_t pm_config = {
        .max_freq_mhz = 80,
        .min_freq_mhz = 80,
        .light_sleep_enable = false
    };
    esp_pm_configure(&pm_config);
    
    ESP_LOGI(TAG, "Modem sleep: 20-25 mA");
}

// ================================================================
// 2. DEEP SLEEP - Everything off (~10 µA)
// ================================================================
void enter_deep_sleep(uint64_t seconds)
{
    boot_count++;
    
    // Wake on timer
    esp_sleep_enable_timer_wakeup(seconds * 1000000);
    
    // Wake on LoRa interrupt (DIO0)
    esp_sleep_enable_ext0_wakeup(LORA_DIO0_PIN, 1);
    
    ESP_LOGI(TAG, "Deep sleep: ~10 µA for %llu seconds", seconds);
    esp_deep_sleep_start();
}

// ================================================================
// 3. HIBERNATION - Maximum savings (10 µA)
// ================================================================
void enter_hibernation(uint64_t seconds)
{
    // Only timer wake
    esp_sleep_enable_timer_wakeup(seconds * 1000000);
    
    // Power down everything except RTC memory
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    
    ESP_LOGI(TAG, "Hibernation: 10 µA for %llu seconds", seconds);
    esp_deep_sleep_start();
}

// ================================================================
// Check why we woke up
// ================================================================
void check_wake_reason(void)
{
    switch(esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_EXT0:
            ESP_LOGI(TAG, "Wake: LoRa message (boot #%lu)", boot_count);
            break;
        case ESP_SLEEP_WAKEUP_TIMER:
            ESP_LOGI(TAG, "Wake: Timer (boot #%lu)", boot_count);
            break;
        default:
            ESP_LOGI(TAG, "Wake: Power on");
            boot_count = 0;
            break;
    }
}
