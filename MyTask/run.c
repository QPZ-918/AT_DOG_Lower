#include "run.h"
#include "usart.h"
#include "usb_trans.h"
#include <string.h>
#include "usbd_cdc_if.h"
#include "tim.h"

/*************************
 * 测试模式开关
 * 1 - 开启测试模式
 * 0 - 关闭测试模式
 ************************/
#define test 0

/**************************
 * 狗腿编号定义
 * 0 - 左前腿 (Front Left)
 * 1 - 右前腿 (Front Right)
 * 2 - 左后腿 (Back Left)
 * 3 - 右后腿 (Back Right)
 **************************/
#define FRONT_LEFT 0
#define FRONT_RIGHT 1
#define BACK_LEFT 2
#define BACK_RIGHT 3

/********************
 * 全局变量 - 通讯统计
 ********************/
uint32_t error_cnt = 0;       // USART2(485_1) 通讯错误次数
uint32_t error_cnt2 = 0;      // USART3(485_2) 通讯错误次数
uint32_t success_cnt = 0;     // USART2(485_1) 通讯成功次数
uint32_t success_cnt2 = 0;    // USART3(485_2) 通讯成功次数
uint32_t reast_cnt_front = 0; // USART2(485_1) 复位次数
uint32_t reast_cnt_back = 0;  // USART3(485_2) 复位次数

uint32_t cur_size_USB_Re = 0; // USB 当前接收数据长度
uint32_t cnt_USB_Re = 0;      // USB 接收总次数

QueueHandle_t cdc_recv_semphr; // USB CDC 接收信号量（二进制）

/********************
 * 全局变量 - 系统状态
 ********************/
uint32_t first_run_front = 5;  // 上电初始化计数，归零后允许 USB 上传数据
uint32_t first_run_back = 5;   // 上电初始化计数，归零后允许 USB 上传数据
uint8_t allow_send = 0;        // 数据发送允许标志：1-允许，0-禁止
uint32_t watch_dog_id[24];     // 看门狗监测的电机 ID 数组（24 个电机，含长时掉线检测）
uint16_t count_Watch_dog = 0;  // 看门狗掉线电机数量统计
uint32_t reset_uart_front = 0; // 485 总线复位标志位
uint32_t reset_uart_back = 0;  // 485 总线复位标志位
uint32_t bad_Motor = 0;        // 电机故障标志位（短时掉线，bit 位对应电机索引）
int err_check_front = 0;       // 每轮通讯检查：6 个电机的接收成功个数
int err_check_back = 0;        // 每轮通讯检查：6 个电机的接收成功个数


uint8_t usb_recv_timeout = 0;

/************************
 * 电机系统数据结构
 ************************/
float setup_offset[4][3]; // 上电初始角度偏移 [4 条腿][3 个关节]，用于零点校准

RS485_t rs485bus;  // RS485 总线 1 句柄（连接 6 个电机）
RS485_t rs485bus2; // RS485 总线 2 句柄（连接 6 个电机）

MotorTargetPack_t legs_target = {.pack_type = 0x04}; // 电机目标值数据包（从 PC 接收）
MotorStatePack_t legs_state = {.pack_type = 0x03};   // 电机状态数据包（上传到 PC）

// 四足机器狗腿部配置表
// 每条腿包含 3 个关节电机 + 1 个轮子电机
// 字段说明：
//   motor_id   - 电机在 485 总线上的 ID(16 进制)
//   rs485      - 所属 485 总线指针
//   inv_motor  - 电机反转标志：1-正转，-1-反转
//   pos_offset - 角度零点偏移量 (rad)
//   hcan       - CAN 总线句柄
//   id         - 轮子在 CAN 总线上的 ID
//   inv_wheel  - 轮子反转标志
Leg_t leg[4] = {
    // 左前腿 (Leg 0)
    {
        .joint[0] = {.motor = {.motor_id = 0x04, .rs485 = &rs485bus}, .inv_motor = 1.0f, .pos_offset = -0.919476f},
        .joint[1] = {.motor = {.motor_id = 0x05, .rs485 = &rs485bus}, .inv_motor = 1.0f, .pos_offset = -10.30426f},
        .joint[2] = {.motor = {.motor_id = 0x06, .rs485 = &rs485bus}, .inv_motor = 2.0f/3.0f, .pos_offset = 15.35149f}
    },

    // 右前腿 (Leg 1)
    {
			    .joint[0] = {.motor = {.motor_id = 0x01, .rs485 = &rs485bus}, .inv_motor = 1.0f, .pos_offset = 0.710782f},
					.joint[1] = {.motor = {.motor_id = 0x02, .rs485 = &rs485bus}, .inv_motor = 1.0f, .pos_offset =  10.213802f},
					.joint[2] = {.motor = {.motor_id = 0x03, .rs485 = &rs485bus}, .inv_motor = 2.0f/3.0f, .pos_offset = -15.55437f}
    },

    // 左后腿 (Leg 2)
    {
        .joint[0] = {.motor = {.motor_id = 0x0A, .rs485 = &rs485bus2}, .inv_motor = 1.0f, .pos_offset =0.828497f},
        .joint[1] = {.motor = {.motor_id = 0x0B, .rs485 = &rs485bus2}, .inv_motor = 1.0f, .pos_offset = 9.060572f},
        .joint[2] = {.motor = {.motor_id = 0x0C, .rs485 = &rs485bus2}, .inv_motor = 2.0f/3.0f, .pos_offset = 15.892584f}
    },

    // 右后腿 (Leg 3)
    {
        .joint[0] = {.motor = {.motor_id = 0x07, .rs485 = &rs485bus2}, .inv_motor = 1.0f, .pos_offset =  -0.89011f},
        .joint[1] = {.motor = {.motor_id = 0x08, .rs485 = &rs485bus2}, .inv_motor = 1.0f, .pos_offset = -9.162765f},
        .joint[2] = {.motor = {.motor_id = 0x09, .rs485 = &rs485bus2}, .inv_motor = 2.0f/3.0f, .pos_offset = -16.010522f}

		}};

void MotorControlTask_Front(void *param) // 将数据发送到电机，并从电机接收数据
{
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        err_check_front = 0;
        for (int i = 0; i < 2; i++)
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
                    taskENTER_CRITICAL();
                    bad_Motor &= (~(0x0001 << (i * 3 + j)));
                    err_check_front++;
                    taskEXIT_CRITICAL();
                    // legs_state.watch_dog = legs_state.watch_dog & (~(0x0001 << (i * 3 + j)));
                }
                else
                {
                    taskENTER_CRITICAL();
                    bad_Motor |= (0x0001 << (i * 3 + j));
                    taskEXIT_CRITICAL();
                }
            }
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(3));
        if (reset_uart_front)
        {

            HAL_UART_DMAStop(&huart2);
            vTaskDelay(2);
            HAL_UART_DeInit(&huart2);

            // 3. 外设寄存器硬复位
            __HAL_RCC_USART2_FORCE_RESET();
            __HAL_RCC_USART2_RELEASE_RESET();

            // 4. 重新初始化 UART + DMA，MspInit 会自动执行
            MX_USART2_UART_Init();
            reast_cnt_front++;
            reset_uart_front = 0;
        }

        if (err_check_front == 6 && first_run_front)
            first_run_front--;
    }
}

void MotorControlTask_Back(void *param) // 将数据发送到电机，并从电机接收数据
{
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1)
    {
        err_check_back = 0;
        for (int i = 2; i < 4; i++)
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
                    taskENTER_CRITICAL();
                    bad_Motor &= (~(0x0001 << (i * 3 + j)));
                    err_check_back++;
                    taskEXIT_CRITICAL();
                    // legs_state.watch_dog = legs_state.watch_dog & (~(0x0001 << (i * 3 + j)));
                }
                else
                {
                    taskENTER_CRITICAL();
                    bad_Motor |= (0x0001 << (i * 3 + j));
                    taskEXIT_CRITICAL();
                }
            }
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(3));
        if (reset_uart_back)
        {
            HAL_UART_DMAStop(&huart3);
            vTaskDelay(2);
            HAL_UART_DeInit(&huart3);

            // 3. 外设寄存器硬复位
            __HAL_RCC_USART3_FORCE_RESET();
            __HAL_RCC_USART3_RELEASE_RESET();

            // 4. 重新初始化 UART + DMA，MspInit 会自动执行
            MX_USART3_UART_Init();
            reast_cnt_back++;
            reset_uart_back = 0;
        }

        if (err_check_back == 6 && first_run_back)
            first_run_back--;
    }
}

int size_recve_mot = 0;
static uint32_t cdc_suc = 0;
int size_usb = 0;
void CDC_Recv_Cb(uint8_t *src, uint16_t size)
{
//    if (usb_recv_timeout)
//        return;
    size_usb = size;
    size_recve_mot = sizeof(MotorTargetPack_t);
    if (size == sizeof(MotorTargetPack_t))
    {
        if (((MotorTargetPack_t *)src)->pack_type == 0x04)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            memcpy(&legs_target, src, sizeof(MotorTargetPack_t));
            xSemaphoreGive(cdc_recv_semphr);
					if(!cdc_suc)
						cdc_suc++;
        }
    }
    cnt_USB_Re++;
    cur_size_USB_Re = size;
}

void MotorSendTask(void *param) // 将电机的数据发送到PC上
{
    USB_CDC_Init(CDC_Recv_Cb, NULL, NULL);
    TickType_t last_wake_time = xTaskGetTickCount();
    uint16_t len = sizeof(legs_state);
    // vTaskDelay(500);
    while (1)
    {
        for (int i = 0; i < 4; i++) // 填写数据并发送到PC
        {
            for (int j = 0; j < 3; j++)
            {
                legs_state.leg[i].joint[j].rad = (leg[i].joint[j].motor.state.rad - leg[i].joint[j].pos_offset - setup_offset[i][j]) / 6.33f * leg[i].joint[j].inv_motor;
                legs_state.leg[i].joint[j].omega = (leg[i].joint[j].motor.state.velocity) / 6.33f * leg[i].joint[j].inv_motor;
                legs_state.leg[i].joint[j].torque = (leg[i].joint[j].motor.state.torque) * 6.33f / leg[i].joint[j].inv_motor;
                if(leg[i].joint[j].motor.state.error)
                    legs_state.motor_error = 0x10;
            }
            legs_state.timestamp = leg[i].timestamp;
            // TODO:根据反馈计算真实力矩
        }

        //        uint8_t flag_check = 1;//检测陀螺仪数据是否正常，1：正常 ，0：异常
        //        JY61_Typedef_ JY61_2 = {};
        //        JY61_2.AngularVelocity.X = JY61.AngularVelocity.X * 3.1415926 / 180.0f;
        //        JY61_2.AngularVelocity.Y = JY61.AngularVelocity.Y * 3.1415926 / 180.0f;
        //        JY61_2.AngularVelocity.Z = JY61.AngularVelocity.Z * 3.1415926 / 180.0f;

        //        JY61_2.Angle.Roll = JY61.Angle.Roll * 3.1415926 / 180.0f;
        //        JY61_2.Angle.Pitch = JY61.Angle.Pitch * 3.1415926 / 180.0f;
        //        JY61_2.Angle.Yaw = JY61.Angle.Yaw * 3.1415926 / 180.0f;

        //        if (JY61_2.AngularVelocity.X > 2.0f || JY61_2.AngularVelocity.X < -2.0f)
        //            flag_check = 0;
        //        else if (JY61_2.AngularVelocity.Y > 2.0f || JY61_2.AngularVelocity.Y < -2.0f)
        //            flag_check = 0;
        //        else if (JY61_2.AngularVelocity.Z > 2.0f || JY61_2.AngularVelocity.Z < -2.0f)
        //            flag_check = 0;
        //        else if (JY61_2.Angle.Roll > 1.0f || JY61_2.Angle.Roll < -1.0f)
        //            flag_check = 0;
        //        else if (JY61_2.Angle.Pitch > 1.0f || JY61_2.Angle.Pitch < -1.0f)
        //            flag_check = 0;

        //        if (flag_check)
        //            legs_state.JY61_ = JY61_2;
        if (allow_send) // 电机数据准备好再发
            CDC_Transmit_HS((uint8_t *)&legs_state, len);
        if (legs_state.watch_dog != 0x0000)
            count_Watch_dog++;

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(3));
    }
}

void MotorRecvTask(void *param) // 从PC接收电机的期望值
{
    cdc_recv_semphr = xSemaphoreCreateBinary();
    xSemaphoreTake(cdc_recv_semphr, 0);
    vTaskDelay(1000);
		HAL_TIM_Base_Start_IT(&htim13);
    while (first_run_back != 0 || first_run_front != 0) // 等待电机数据准备好
        vTaskDelay(1);

    for (int i = 0; i < 4; i++)
    {
        setup_offset[i][0] = leg[i].joint[0].motor.state.rad;
        setup_offset[i][1] = leg[i].joint[1].motor.state.rad;
        setup_offset[i][2] = leg[i].joint[2].motor.state.rad;
    }
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) && !cdc_suc)
    {
        vTaskDelay(50);
    }
    allow_send = 1;
    vTaskDelay(1000);
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            // usb_recv_timeout = 1;
            leg[i].joint[j].exp_omega = 0.0f;
            leg[i].joint[j].exp_torque = 0.0f;
            leg[i].joint[j].Kp = 0.0f * (6.33f * 6.33f / (leg[i].joint[j].inv_motor * leg[i].joint[j].inv_motor));
            leg[i].joint[j].Kd = 0.1f * (6.33f * 6.33f / (leg[i].joint[j].inv_motor * leg[i].joint[j].inv_motor));
        }
    }
    // 允许发送数据
    xSemaphoreTake(cdc_recv_semphr, portMAX_DELAY); // 等待第一个数据帧到来
    while (1)
    {
        if (xSemaphoreTake(cdc_recv_semphr, pdMS_TO_TICKS(100)) != pdPASS) // 发生超时，说明通讯断开
        {
            // TODO:通过LED显示，清零所有力矩，电机进入低阻尼模式，整狗进入安全模式
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    usb_recv_timeout = 1;
                    leg[i].joint[j].exp_omega = 0.0f;
                    leg[i].joint[j].exp_torque = 0.0f;
                    leg[i].joint[j].Kp = 0.0f * (6.33f * 6.33f / (leg[i].joint[j].inv_motor * leg[i].joint[j].inv_motor));
                    leg[i].joint[j].Kd = 0.1f * (6.33f * 6.33f / (leg[i].joint[j].inv_motor * leg[i].joint[j].inv_motor));
                }
            }
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15))
            {
                vTaskDelay(10);
                if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15))
                {
                    usb_recv_timeout = 0;
                    xSemaphoreTake(cdc_recv_semphr, portMAX_DELAY);
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
            leg[i].timestamp = legs_target.timestamp;
        }
    }
}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        RS485SendIRQ_Handler(&rs485bus, huart);
    }
    else if (huart->Instance == USART3)
    {
        RS485SendIRQ_Handler(&rs485bus2, huart);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    
     if (huart->Instance == USART2)
    {
        RS485RecvIRQ_Handler(&rs485bus, huart, size);
        success_cnt++;
    }
    else if (huart->Instance == USART3)
    {
        RS485RecvIRQ_Handler(&rs485bus2, huart, size);
        success_cnt2++;
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


    else if (huart->Instance == USART3)
    {
        __HAL_UART_CLEAR_FLAG(huart,
                              UART_CLEAR_OREF |
                                  UART_CLEAR_FEF |
                                  UART_CLEAR_NEF |
                                  UART_CLEAR_PEF);

        __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);

        error_cnt2++;
    }
}

