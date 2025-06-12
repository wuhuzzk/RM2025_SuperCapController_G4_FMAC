#ifndef _STATUS_H_
#define _STATUS_H_

#include "stm32g4xx_hal.h"
extern uint8_t led_state[4];
extern uint16_t led_cnt[4];

void status_flash (void);


#endif
