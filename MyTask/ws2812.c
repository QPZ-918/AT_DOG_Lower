#include "ws2812.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define WS2812_LowLevel    0xC0     // 0Ты
#define WS2812_HighLevel   0xF0     // 1Ты

static QueueHandle_t kWs2812_spi_send_semphr;

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[24];
    uint8_t res = 0;
    for (int i = 0; i < 8; i++)
    {
        txbuf[7-i]  = (((g>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[15-i] = (((r>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[23-i] = (((b>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
    }
    HAL_SPI_Transmit_IT(&hspi6, &res, 0);
    xSemaphoreTake(kWs2812_spi_send_semphr,2);
    while (hspi6.State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit_IT(&hspi6, txbuf, 24);
    xSemaphoreTake(kWs2812_spi_send_semphr,2);
    for (int i = 0; i < 100; i++)
    {
        HAL_SPI_Transmit_IT(&hspi6, &res, 1);
        xSemaphoreTake(kWs2812_spi_send_semphr,2);
    }
}

void WS2812_Init()
{
	kWs2812_spi_send_semphr=xSemaphoreCreateBinary();
    xSemaphoreTake(kWs2812_spi_send_semphr,0);
}

void WS2812_SPI_Send_IRQ()
{
    BaseType_t temp;
    xSemaphoreGiveFromISR(kWs2812_spi_send_semphr,&temp);
    portYIELD_FROM_ISR(temp);
}