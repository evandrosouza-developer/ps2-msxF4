#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void tim2_setup(void);

void delay_usec(uint16_t, void next_step (void));

void prepares_capture(uint32_t timer_peripheral);

#ifdef __cplusplus
}
#endif
