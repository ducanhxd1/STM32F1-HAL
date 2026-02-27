/*
 * LED7.h
 *
 *  Created on: Feb 27, 2026
 *      Author: DucAnh
 */

#ifndef _LED7_H_
#define _LED7_H_

#include "stm32f1xx_hal.h"

/*
 * LED 7 Doan
 */

typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
} LED7_GPIO_t;

typedef enum {
	LED_ANODE,
	LED_CATHODE
} LED7_Type_t;

typedef struct {
	LED7_GPIO_t seg[7];
	LED7_Type_t type;
} LED7_Handle_t;

void LED7_Init(LED7_Handle_t *hLed7,
		LED7_GPIO_t seg_a,
		LED7_GPIO_t seg_b,
		LED7_GPIO_t seg_c,
		LED7_GPIO_t seg_d,
		LED7_GPIO_t seg_e,
		LED7_GPIO_t seg_f,
		LED7_GPIO_t seg_g,
		LED7_Type_t type);

void LED7_Display(LED7_Handle_t *hLed7, uint8_t number);



#endif /* _LED7_H_ */










