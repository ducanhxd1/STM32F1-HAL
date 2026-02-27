/*
 * LED7.c
 *
 *  Created on: Feb 27, 2026
 *      Author: DucAnh
 */

#include "LED7.h"

// NUMBER SEGMENT CODE
static uint8_t segment_code[10] = {
		0x3F,	// 0
		0x06,	// 1
		0x5B,	// 2
		0x4F,	// 3
		0x66,	// 4
		0x6D,	// 5
		0x7D,	// 6
		0x07,	// 7
		0x7F,	// 8
		0x6F,	// 9
};

void LED7_Init(LED7_Handle_t *hLed7,
		LED7_GPIO_t seg_a,
		LED7_GPIO_t seg_b,
		LED7_GPIO_t seg_c,
		LED7_GPIO_t seg_d,
		LED7_GPIO_t seg_e,
		LED7_GPIO_t seg_f,
		LED7_GPIO_t seg_g,
		LED7_Type_t type)
{
	hLed7->type = type;
	hLed7->seg[0] = seg_a;
	hLed7->seg[1] = seg_b;
	hLed7->seg[2] = seg_c;
	hLed7->seg[3] = seg_d;
	hLed7->seg[4] = seg_e;
	hLed7->seg[5] = seg_f;
	hLed7->seg[6] = seg_g;
}

void LED7_Display(LED7_Handle_t *hLed7, uint8_t number)
{
	if (number > 9)
		return;

	uint8_t data = segment_code[number];

	for(int i = 0; i < 7; i++) {
		uint8_t bit = (data >> i) & (0x01);

		GPIO_PinState state;
		if (hLed7 -> type == LED_ANODE) {
			state = bit ? 0 : 1;
		} else {
			state = bit ? 1 : 0;
		}

		HAL_GPIO_WritePin(hLed7->seg[i].port, hLed7->seg[i].pin, state);
	}
}




