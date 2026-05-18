#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RMDS_LORA_MAX_PACKET_LEN 128

/* Configuration for the LoRa radio: frequency, bandwidth, spreading
 * factor, coding rate, preamble length, sync word, and CRC enable flag. */
typedef struct {
    long frequency_hz;
    long bandwidth_hz;
    int spreading_factor;
    int coding_rate;
    int preamble_len;
    int sync_word;
    bool enable_crc;
} rmds_lora_config_t;

/* Return pointer to the default LoRa configuration. */
const rmds_lora_config_t *rmds_lora_default_config(void);
/* Initialize LoRa radio with optional config, uses defaults if NULL. */
bool rmds_lora_init(const rmds_lora_config_t *config);
/* Send len bytes from data over LoRa, returns true on success. */
bool rmds_lora_send(const void *data, size_t len);
/* Receive a LoRa packet into buf up to buf_len, returns length or 0. */
int rmds_lora_receive(uint8_t *buf, size_t buf_len);
/* Put radio into receive/listen mode. */
void rmds_lora_start_listening(void);
/* Put radio in low-power sleep mode. */
void rmds_lora_sleep_radio(void);
/* Return whether the LoRa driver has been initialized. */
bool rmds_lora_is_initialized(void);
/* Return RSSI (dBm) of last received packet, 0 if unavailable. */
int rmds_lora_last_packet_rssi(void);
/* Return SNR (dB) of last received packet, 0.0 if unavailable. */
float rmds_lora_last_packet_snr(void);

#ifdef __cplusplus
}
#endif
