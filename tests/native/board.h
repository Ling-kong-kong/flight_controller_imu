#ifndef TEST_BOARD_H
#define TEST_BOARD_H

#include <stdbool.h>
#include <stdint.h>

/* Single-thread host harness: model PRIMASK preservation, not ARM IRQ timing. */
extern uint32_t mock_primask;
static inline uint32_t __get_PRIMASK(void) { return mock_primask; }
static inline void __disable_irq(void) { mock_primask = 1; }
static inline void __set_PRIMASK(uint32_t value) { mock_primask = value; }
static inline void __DMB(void) { __atomic_thread_fence(__ATOMIC_SEQ_CST); }

uint64_t board_micros(void);
_Noreturn void board_fatal(unsigned code);

#endif
