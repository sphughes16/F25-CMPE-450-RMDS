#ifndef POWER_H
#define POWER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void enter_modem_sleep(void);

void enter_deep_sleep(uint64_t seconds);

void enter_hibernation(uint64_t seconds);

void check_wake_reason(void);

#ifdef __cplusplus
}
#endif

#endif