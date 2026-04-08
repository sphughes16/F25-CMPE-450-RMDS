#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RMDS_LORA_MAX_PACKET_LEN 128

typedef struct {
    long frequency_hz;
    long bandwidth_hz;
    int spreading_factor;
    int coding_rate;
    int preamble_len;
    int sync_word;
    bool enable_crc;
} rmds_lora_config_t;

const rmds_lora_config_t *rmds_lora_default_config(void);
bool rmds_lora_init(const rmds_lora_config_t *config);
bool rmds_lora_send(const void *data, size_t len);
int rmds_lora_receive(uint8_t *buf, size_t buf_len);
void rmds_lora_start_listening(void);
void rmds_lora_sleep_radio(void);
bool rmds_lora_is_initialized(void);
int rmds_lora_last_packet_rssi(void);
float rmds_lora_last_packet_snr(void);

#ifdef __cplusplus
}
#endif
