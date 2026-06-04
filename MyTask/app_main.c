#include "app_main.h"
#include "485_bus.h"
#include "ws2812.h"
#include "usart.h"
#include "tim.h"
#include "spi.h"
#include "run.h"
#include "freertos.h"


extern RS485_t rs485bus;
extern RS485_t rs485bus2;
TaskHandle_t usb_send_task_handle;
TaskHandle_t usb_recv_task_handle;
//TaskHandle_t motor_control_task_handle;
TaskHandle_t unitree_front_task_handle;
TaskHandle_t unitree_back_task_handle;

QueueHandle_t remote_semaphore;

extern uint8_t data[11];

uint8_t dma1_send_buf[sizeof(GOMotor_SendPack_t)] __attribute__((section("RAM_D2_485"), aligned(32)));
uint8_t dma1_recv_buf[sizeof(GOMotor_ReceivePack_t)] __attribute__((section("RAM_D2_485"), aligned(32)));
uint8_t dma2_send_buf[sizeof(GOMotor_SendPack_t)] __attribute__((section("RAM_D2_485"), aligned(32)));
uint8_t dma2_recv_buf[sizeof(GOMotor_ReceivePack_t)] __attribute__((section("RAM_D2_485"), aligned(32)));

void Wheel(uint8_t pos, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if(pos < 85)
    {
        *r = (255 - pos * 3) * 0.1;
        *g = (pos * 3) * 0.1;
        *b = 0;
    }
    else if(pos < 170)
    {
        pos -= 85;
        *r = 0;
        *g = (255 - pos * 3) * 0.1;
        *b = (pos * 3) * 0.1;
    }
    else
    {
        pos -= 170;
        *r = (pos * 3) * 0.1;
        *g = 0;
        *b = (255 - pos * 3) * 0.1;
    }
}

uint8_t hue = 0;


uint8_t r,g,b;

void app_main()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);

	/*
	*/
	RS485Init(&rs485bus, &huart2, NULL, NULL,dma1_send_buf,dma1_recv_buf);	//该串口拥有硬件流控制脚，所以写NULL表示使用硬件流控制脚
	RS485Init(&rs485bus2, &huart3, NULL, NULL,dma2_send_buf,dma2_recv_buf);
	remote_semaphore =	xSemaphoreCreateBinary();
	xTaskCreate(MotorControlTask_Front,"MotorComm_Front",256,NULL,6,&unitree_front_task_handle);
	xTaskCreate(MotorControlTask_Back,"MotorComm_Back",256,NULL,6,&unitree_back_task_handle);
	xTaskCreate(MotorSendTask,"MotorSend",256,NULL,4,&usb_send_task_handle);
	xTaskCreate(MotorRecvTask,"MotorRecv",128,NULL,5,&usb_recv_task_handle);
	WS2812_Init();
	while(1)
	{
		Wheel(hue, &r, &g, &b);
		WS2812_Ctrl(r,g,b);
		hue++;
		vTaskDelay(20);
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance==SPI6)
	{
		WS2812_SPI_Send_IRQ();
	}
}
