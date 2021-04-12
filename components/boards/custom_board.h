/*
 * @Description: -
 * @Version: -
 * @Author: Fox_benjiaming
 * @Date: 2021-04-07 15:32:31
 * @LastEditors: Fox_benjiaming
 * @LastEditTime: 2021-04-12 15:09:26
 */
#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

#define LEDS_NUMBER    4

#define LED_START      26
#define LED_1          26
#define LED_2          27
#define LED_3          28
#define LED_4          29
#define LED_STOP       29

#define LEDS_ACTIVE_STATE 0

#define LEDS_INV_MASK  LEDS_MASK

#define LEDS_LIST { LED_1, LED_2, LED_3, LED_4 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2
#define BSP_LED_2      LED_3
#define BSP_LED_3      LED_4

#define BUTTONS_NUMBER 1

#define BUTTON_START   11
#define BUTTON_1       11
#define BUTTON_STOP    11
#define BUTTON_PULL    NRF_GPIO_PIN_PULLDOWN

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1

#define RX_PIN_NUMBER  13
#define TX_PIN_NUMBER  12
#define CTS_PIN_NUMBER 7
#define RTS_PIN_NUMBER 5
#define HWFC           false

#endif
