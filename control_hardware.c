/*
 * for Gas Range
 */

#include "main.h"
#include "control_hardware.h"
#define led_ring_rgb(red, green, blue) ((uint32_t)((uint32_t) blue | (uint32_t) red << 16 | (uint32_t) green << 8))

void led_ring_bit(uint8_t x) {
	if (x) {
		LED_RING_GPIO_Port->BSRR = LED_RING_Pin;
		NS800
		;
		LED_RING_GPIO_Port->BRR = LED_RING_Pin;
	} else {
		LED_RING_GPIO_Port->BSRR = LED_RING_Pin;
		NS400;
		LED_RING_GPIO_Port->BRR = LED_RING_Pin;
	}
}

void led_ring_update(uint32_t *data_24bit) {
	uint16_t i;
	uint8_t data_bit[288];

	for (i = 0; i < 288; i++) {
		data_bit[i] = data_24bit[i / 24] >> (23 - (i % 24)) & 1;
	}

	for (i = 0; i < 288; i++) {
		led_ring_bit(data_bit[i]);
	}


	HAL_Delay(1);

}

