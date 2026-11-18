#ifndef TEST_TUSB_H
#define TEST_TUSB_H

#include <stdbool.h>
#include <stdint.h>

bool tud_cdc_connected(void);
bool tud_suspended(void);
bool tud_cdc_write_clear(void);
uint32_t tud_cdc_available(void);
uint32_t tud_cdc_read(void *buffer, uint32_t count);
uint32_t tud_cdc_write_available(void);
uint32_t tud_cdc_write(void const *buffer, uint32_t count);
uint32_t tud_cdc_write_flush(void);

#endif
