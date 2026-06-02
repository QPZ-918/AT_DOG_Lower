#ifndef __WS2812_H__
#define __WS2812_H__

#include <stdint.h>

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);
void WS2812_SPI_Send_IRQ();
void WS2812_Init();


#endif