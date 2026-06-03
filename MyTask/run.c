#include "run.h"
#include "usart.h"
#include "usb_trans.h"
#include <string.h>
#include "usbd_cdc_if.h"
#include "bezier.h"
#include "imu_temp_ctrl.h"
#define test 0

#define FRONT_LEFT 0
#define FRONT_RIGHT 1
#define BACK_LEFT 2
#define BACK_RIGHT 3

extern uint8_t UserTxBufferHS[APP_TX_DATA_SIZE];
extern TIM_HandleTypeDef htim13;

#include "crc_ccitt.h"
#define MOTOR_NUM 12

BMI088_data IMU_data;


uint32_t error_cnt = 0;
uint32_t error_cnt2 = 0;
uint32_t success_cnt = 0;
uint32_t reast_cnt = 0;

// 添加错误标志和重启接收标志
uint32_t last_error_time = 0;

RS485_t rs485bus;
QueueHandle_t cdc_recv_semphr;

MotorTargetPack_t legs_target = {.pack_type = 0x04};
MotorStatePack_t legs_state = {.pack_type = 0x00};
Leg_t leg[4] = {

    // 右前腿
    {.joint[0] = {.motor = {.motor_id = 0x04, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 3.968367f},
     .joint[1] = {.motor = {.motor_id = 0x05, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 6.73052f},
     .joint[2] = {.motor = {.motor_id = 0x06, .rs485 = &rs485bus}, .inv_motor = -0.5, .pos_offset = 13.056006f}
     },
    // 左前腿
    {.joint[0] = {.motor = {.motor_id = 0x01, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = -4.292375f},
     .joint[1] = {.motor = {.motor_id = 0x02, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = -6.875717f},
     .joint[2] = {.motor = {.motor_id = 0x03, .rs485 = &rs485bus}, .inv_motor = -0.5, .pos_offset = -13.226964f}
     },

    // 右后腿
    {.joint[0] = {.motor = {.motor_id = 0x0A, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = -4.234103f},
     .joint[1] = {.motor = {.motor_id = 0x0B, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 6.392829f},
     .joint[2] = {.motor = {.motor_id = 0x0C, .rs485 = &rs485bus}, .inv_motor = -0.5, .pos_offset = 12.753429f}
     },
    // 左后腿
    {.joint[0] = {.motor = {.motor_id = 0x07, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 4.089883f},
     .joint[1] = {.motor = {.motor_id = 0x08, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = -6.996269f},
     .joint[2] = {.motor = {.motor_id = 0x09, .rs485 = &rs485bus}, .inv_motor = -0.5, .pos_offset = -13.355066f}
     }
};

float setup_offset[4][3]; // 上电启动时的电机角度
uint32_t first_run = 5;
// uint32_t watch_dog_id[24];
uint32_t reset_uart;
uint32_t bad_Motor = 0;
uint64_t uart_reast = 0;
int err_check = 0;


void MotorControlTask(void *param) // 将数据发送到电机，并从电机接收数据
{
    TickType_t last_wake_time = xTaskGetTickCount();
    HAL_TIM_Base_Start_IT(&htim13);
    while (1)
    {
        err_check = 0;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {

              GoMotorSend(&leg[i].joint[j].motor, leg[i].joint[j].exp_torque / 6.33f * leg[i].joint[j].inv_motor,
                           leg[i].joint[j].exp_omega * 6.33f / leg[i].joint[j].inv_motor,
                           leg[i].joint[j].exp_rad * 6.33f / leg[i].joint[j].inv_motor + leg[i].joint[j].pos_offset + setup_offset[i][j],
                           leg[i].joint[j].Kp / (6.33f * 6.33f / (leg[i].joint[j].inv_motor * leg[i].joint[j].inv_motor)),
                           leg[i].joint[j].Kd / (6.33f * 6.33f / (leg[i].joint[j].inv_motor * leg[i].joint[j].inv_motor)));
				

                int ret = GoMotorRecv(&leg[i].joint[j].motor);
                if (ret)
                {
                    bad_Motor &= (~(0x0001 << (i * 3 + j)));
                    err_check++;
                    legs_state.watch_dog = legs_state.watch_dog & (~(0x0001 << (i * 3 + j)));
                }
                else
                    bad_Motor |= (0x0001 << (i * 3 + j));
            }
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(3));
        if (reset_uart)
        {
            HAL_UART_DMAStop(&huart2);
            vTaskDelay(2);
            HAL_UART_DeInit(&huart2);

            // 3. 外设寄存器硬复位
            __HAL_RCC_USART2_FORCE_RESET();
            __HAL_RCC_USART2_RELEASE_RESET();

            // 4. 重新初始化 UART + DMA，MspInit 会自动执行
            MX_USART2_UART_Init();
            reset_uart = 0;
            reast_cnt++;
        }

        if (err_check == 12 && first_run)
            first_run--;
    }
}


uint32_t current_size = 0;
uint32_t cnt = 0;
void CDC_Recv_Cb(uint8_t *src, uint16_t size)
{
    if (size == sizeof(MotorTargetPack_t) && ((MotorTargetPack_t *)src)->pack_type == 0x04)
    {

        memcpy(&legs_target, src, sizeof(MotorTargetPack_t));
        xSemaphoreGive(cdc_recv_semphr);

    }
    cnt++;
    current_size = size;

}


uint16_t count = 0;
uint8_t allow_send = 0;

uint8_t flag_check = 1;
void MotorSendTask(void *param) // 将电机的数据发送到PC上
{
    USB_CDC_Init(CDC_Recv_Cb, NULL, NULL);
    TickType_t last_wake_time = xTaskGetTickCount();
    uint16_t len = sizeof(legs_state);

    while (1)
    {
        for (int i = 0; i < 4; i++) // 填写数据并发送到PC
        {
            for (int j = 0; j < 3; j++)
            {
                legs_state.leg[i].joint[j].rad = (leg[i].joint[j].motor.state.rad - leg[i].joint[j].pos_offset - setup_offset[i][j]) / 6.33f * leg[i].joint[j].inv_motor;
                legs_state.leg[i].joint[j].omega = (leg[i].joint[j].motor.state.velocity) / 6.33f * leg[i].joint[j].inv_motor;
                legs_state.leg[i].joint[j].torque = (leg[i].joint[j].motor.state.torque) * 6.33f / leg[i].joint[j].inv_motor;
            }
            legs_state.leg[i].wheel.omega = leg[i].wheel.inv_wheel * leg[i].wheel.wheel_.velocity;
            legs_state.leg[i].wheel.torque = leg[i].wheel.inv_wheel * leg[i].wheel.wheel_.torque;
						legs_state.tim_p = leg[i].tim_p;
            // TODO:根据反馈计算真实力矩
        }


        legs_state.imu_data.Angle.q0 = IMU_data.q[0];
        legs_state.imu_data.Angle.q1 = IMU_data.q[1];
        legs_state.imu_data.Angle.q2 = IMU_data.q[2];
        legs_state.imu_data.Angle.q3 = IMU_data.q[3];
        legs_state.imu_data.AngularVelocity.X = IMU_data.Gyro[0];
        legs_state.imu_data.AngularVelocity.Y = IMU_data.Gyro[1];
        legs_state.imu_data.AngularVelocity.Z = IMU_data.Gyro[2];

        if (allow_send) // 电机数据准备好再发
            CDC_Transmit_HS((uint8_t *)&legs_state, len);

        if (legs_state.watch_dog != 0x0000)
            count++;

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(3));
    }
}

void MotorRecvTask(void *param) // 从PC接收电机的期望值
{
    cdc_recv_semphr = xSemaphoreCreateBinary();
    xSemaphoreTake(cdc_recv_semphr, 0);
    vTaskDelay(1000);
    while (first_run) // 等待电机数据准备好
        vTaskDelay(1);

    for (int i = 0; i < 4; i++)
    {
        setup_offset[i][0] = leg[i].joint[0].motor.state.rad;
        setup_offset[i][1] = leg[i].joint[1].motor.state.rad;
        setup_offset[i][2] = leg[i].joint[2].motor.state.rad;
    }
    allow_send = 1; // 允许发送数据
    // xSemaphoreTake(cdc_recv_semphr, portMAX_DELAY); // 等待第一个数据帧到来
    while (1)
    {
        if (xSemaphoreTake(cdc_recv_semphr, pdMS_TO_TICKS(50)) != pdPASS) // 发生超时，说明通讯断开
        {
            // TODO:通过LED显示，清零所有力矩，电机进入低阻尼模式，整狗进入安全模式
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    leg[i].joint[j].exp_omega = 0.0f;
                    leg[i].joint[j].exp_torque = 0.0f;
                    leg[i].joint[j].Kp = 0.0f * (6.33f * 6.33f / (leg[i].joint[j].inv_motor * leg[i].joint[j].inv_motor));
                    leg[i].joint[j].Kd = 0.1f * (6.33f * 6.33f / (leg[i].joint[j].inv_motor * leg[i].joint[j].inv_motor));
                }
            }
            continue;
        }

        // TODO:安全限幅并给到电机期望
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                leg[i].joint[j].exp_rad = legs_target.leg[i].joint[j].rad;
                leg[i].joint[j].exp_omega = legs_target.leg[i].joint[j].omega;
                leg[i].joint[j].exp_torque = legs_target.leg[i].joint[j].torque;
                leg[i].joint[j].Kp = legs_target.leg[i].joint[j].kp;
                leg[i].joint[j].Kd = legs_target.leg[i].joint[j].kd;
            }
			leg[i].tim_p = legs_target.tim_p;
        }
				
    }
}
uint32_t cnt_imu = 0;
void BMI088_task(void *param)
{

    INS_Init();
    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;)
    {
		//cnt_imu++;
        INS_Task(&IMU_data);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
}





void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        RS485SendIRQ_Handler(&rs485bus, huart);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == USART2)
    {
        RS485RecvIRQ_Handler(&rs485bus, huart, size);
        success_cnt++;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        __HAL_UART_CLEAR_FLAG(huart,
                              UART_CLEAR_OREF |
                                  UART_CLEAR_FEF |
                                  UART_CLEAR_NEF |
                                  UART_CLEAR_PEF);

        __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);

        error_cnt++;
    }
}
