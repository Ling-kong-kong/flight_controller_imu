#ifndef MAG_H
#define MAG_H
#include <stdbool.h>
#include <stdint.h>
#include "bmm150.h"
#define MAG_FRAME_SIZE 72u
#define MAG_VALID 1u
#define MAG_READY 2u
#define MAG_TRIM_OK 4u
#define MAG_CONFIGURED 8u
#define MAG_DIAGNOSTIC 16u
/* SWD-visible diagnostic snapshot, never consumed by attitude fusion. */
typedef struct {
    uint32_t sequence, bus_errors, invalid_samples, not_ready, tx_overwrites;
    uint32_t samples, retries, read_us;
    uint8_t address, chip_id, state, error, rep_xy, rep_z;
    bool trim_ok, configured;
} mag_diagnostics_t;
extern volatile mag_diagnostics_t g_mag_diagnostics;
extern struct bmm150_trim_registers g_mag_trim;
/* Pure helper: rejects sentinel raw values, zero Hall and unusable trim. */
bool mag_compensate(const uint8_t data[8], const struct bmm150_trim_registers *trim,
                    int16_t raw[3], uint16_t *hall, float ut[3]);
void mag_task(void);
#endif
