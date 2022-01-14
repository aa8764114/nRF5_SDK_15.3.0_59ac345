#ifndef CUSTOM_BOARD_US1_JP1_H__
#define CUSTOM_BOARD_US1_JP1_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

// LEDs definitions for US1 / JP1
#define LEDS_NUMBER    1

#define LED_START      12
#define LED_1          12
#define LED_STOP       12

#define LEDS_ACTIVE_STATE 0

#define LEDS_INV_MASK  LEDS_MASK

#define LEDS_LIST { LED_1 }

#define BSP_LED_0      LED_1

#define BUTTONS_NUMBER 1

#define BUTTON_START   21
#define BUTTON_1       21
#if (BUTTONS_NUMBER == 1)
#define BUTTON_STOP    21
#define BUTTONS_LIST { BUTTON_1 }
#elif (BUTTONS_NUMBER == 2)
#define BUTTON_2       12
#define BUTTON_STOP    12
#define BUTTONS_LIST { BUTTON_1, BUTTON_2 }
#define BSP_BUTTON_1   BUTTON_2
#else
#error "check button definition"
#endif
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0


#define BSP_BUTTON_0   BUTTON_1

#ifdef __cplusplus
}
#endif

#endif
