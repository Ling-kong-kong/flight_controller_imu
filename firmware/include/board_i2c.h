#ifndef BOARD_I2C_H
#define BOARD_I2C_H
#include <stdbool.h>
#include <stdint.h>
/* Single main-context owner. Buffer must outlive transaction. No polling loops.
 * Result: 0 pending, 1 completed, -1 error (controller reset, bounded 5ms).
 * Only this controller is recovered; no bus-wide GPIO pulses or slave reset. */
void board_i2c_init(void);
bool board_i2c_start(uint8_t address, uint8_t reg, uint8_t *data, uint8_t size, bool read);
int board_i2c_poll(void);
#endif
