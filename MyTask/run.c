#include "run.h"
#include "usart.h"
#include "mylist.h"
#include "usb_trans.h"
#include "WatchDog2.h"
#include <string.h>
#include "usbd_cdc_if.h"
#include "JY61.h"
#include "bezier.h"
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

/**********************
 * 陀螺仪数据 (JY61)
 **********************/
JY61_Typedef JY61;                                                      // 陀螺仪数据结构体
uint8_t data[11] __attribute__((section("RAM_D2_OTHER"), aligned(32))); // DMA 接收缓冲区（32 字节对齐）
extern DMA_HandleTypeDef hdma_usart10_rx;                               // USART10 RX DMA 句柄

/*********************
 * 遥控器数据处理
 *********************/
static BezierLine bezier = { // 摇杆贝塞尔曲线参数（用于非线性映射）
    .p1_x = 0.660634f,
    .p1_y = 0.131222f,
    .p2_x = 0.846154f,
    .p2_y = 0.556561f};

uint8_t remote_control_buf[12] __attribute__((section("RAM_D2_OTHER"), aligned(32))); // 遥控接收缓冲区（32 字节对齐）

float filter_gate = 1.0f;                    // 摇杆数据滤波系数
float last_v0, last_v1, last_omega, last_v3; // 各通道上次滤波值
static const float filter_alpha = 0.2f;      // 一阶低通滤波系数

float max_omega = 120.0f;        // 最大自转角速度 (deg/s)
float max_forword_speed = 1.0f;  // 最大前进速度 (m/s)
float max_backward_speed = 0.5f; // 最大后退速度 (m/s)
float max_speed = 0.4f;          // 最大横向速度 (m/s)

float cur_dir = 0.0f;     // 当前运动方向角 (rad)
float key1 = 0, key2 = 0; // 遥控器按键状态

RemotePack_t remotedata;               // 遥控器数据包结构体
extern QueueHandle_t remote_semaphore; // 遥控器信号量（ISR 触发）

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
        .joint[2] = {.motor = {.motor_id = 0x06, .rs485 = &rs485bus}, .inv_motor = 2.0f/3.0f, .pos_offset = 15.35149f},
        .wheel    = {.wheel_ = {.hcan = &hfdcan1, .id = 0x02}, .inv_wheel = 1}

//        .joint[0] = {.motor = {.motor_id = 0x04, .rs485 = &rs485bus}, .inv_motor = 1.0f, .pos_offset = -0.4193f},
//        .joint[1] = {.motor = {.motor_id = 0x05, .rs485 = &rs485bus}, .inv_motor = 1.0f, .pos_offset = -10.1582f},
//        .joint[2] = {.motor = {.motor_id = 0x06, .rs485 = &rs485bus}, .inv_motor = 2.0f / 3.0f, .pos_offset = 15.0628004f},
//        .wheel = {.wheel_ = {.hcan = &hfdcan1, .id = 0x02}, .inv_wheel = 1}
        //        .joint[0] = {.motor = {.motor_id = 0x01, .rs485 = &rs485bus}, .inv_motor = 1.0f, .pos_offset = 0.3928f},
        //        .joint[1] = {.motor = {.motor_id = 0x02, .rs485 = &rs485bus}, .inv_motor = 1.0f, .pos_offset = 9.8292f},
        //        .joint[2] = {.motor = {.motor_id = 0x03, .rs485 = &rs485bus}, .inv_motor = 2.0f/3.0f, .pos_offset = -14.2399998f},
        //        .wheel    = {.wheel_ = {.hcan = &hfdcan1, .id = 0x01}, .inv_wheel = 1}
    },

    // 右前腿 (Leg 1)
    {

//        .joint[0] = {.motor = {.motor_id = 0x01, .rs485 = &rs485bus}, .inv_motor = 1.0f, .pos_offset = 0.3928f},
//        .joint[1] = {.motor = {.motor_id = 0x02, .rs485 = &rs485bus}, .inv_motor = 1.0f, .pos_offset = 9.8292f},
//        .joint[2] = {.motor = {.motor_id = 0x03, .rs485 = &rs485bus}, .inv_motor = 2.0f / 3.0f, .pos_offset = -14.2399998f},
//        .wheel = {.wheel_ = {.hcan = &hfdcan1, .id = 0x01}, .inv_wheel = 1}
			    .joint[0] = {.motor = {.motor_id = 0x01, .rs485 = &rs485bus}, .inv_motor = 1.0f, .pos_offset = 0.710782f},
					.joint[1] = {.motor = {.motor_id = 0x02, .rs485 = &rs485bus}, .inv_motor = 1.0f, .pos_offset =  10.213802f},
					.joint[2] = {.motor = {.motor_id = 0x03, .rs485 = &rs485bus}, .inv_motor = 2.0f/3.0f, .pos_offset = -15.55437f},
					.wheel    = {.wheel_ = {.hcan = &hfdcan1, .id = 0x01}, .inv_wheel = 1}
        //        .joint[0] = {.motor = {.motor_id = 0x04, .rs485 = &rs485bus}, .inv_motor = 1.0f, .pos_offset = -0.4193f},
        //        .joint[1] = {.motor = {.motor_id = 0x05, .rs485 = &rs485bus}, .inv_motor = 1.0f, .pos_offset = -10.1582f},
        //        .joint[2] = {.motor = {.motor_id = 0x06, .rs485 = &rs485bus}, .inv_motor = 2.0f/3.0f, .pos_offset = 15.0628004f},
        //        .wheel    = {.wheel_ = {.hcan = &hfdcan1, .id = 0x02}, .inv_wheel = 1}
    },

    // 左后腿 (Leg 2)
    {
//        .joint[0] = {.motor = {.motor_id = 0x0A, .rs485 = &rs485bus2}, .inv_motor = 1.0f, .pos_offset = 0.1762f},
//        .joint[1] = {.motor = {.motor_id = 0x0B, .rs485 = &rs485bus2}, .inv_motor = 1.0f, .pos_offset = 10.101f},
//        .joint[2] = {.motor = {.motor_id = 0x0C, .rs485 = &rs485bus2}, .inv_motor = 2.0f / 3.0f, .pos_offset = 15.3891392f},
//        .wheel = {.wheel_ = {.hcan = &hfdcan1, .id = 0x04}, .inv_wheel = 1}
			
        .joint[0] = {.motor = {.motor_id = 0x0A, .rs485 = &rs485bus2}, .inv_motor = 1.0f, .pos_offset =0.828497f},
        .joint[1] = {.motor = {.motor_id = 0x0B, .rs485 = &rs485bus2}, .inv_motor = 1.0f, .pos_offset = 9.060572f},
        .joint[2] = {.motor = {.motor_id = 0x0C, .rs485 = &rs485bus2}, .inv_motor = 2.0f/3.0f, .pos_offset = 15.892584f},
        .wheel    = {.wheel_ = {.hcan = &hfdcan1, .id = 0x04}, .inv_wheel = 1}


        //        .joint[0] = {.motor = {.motor_id = 0x07, .rs485 = &rs485bus2}, .inv_motor = 1.0f, .pos_offset = -0.0811f},
        //        .joint[1] = {.motor = {.motor_id = 0x08, .rs485 = &rs485bus2}, .inv_motor = 1.0f, .pos_offset = -10.0143f},
        //        .joint[2] = {.motor = {.motor_id = 0x09, .rs485 = &rs485bus2}, .inv_motor = 2.0f/3.0f, .pos_offset = -15.0216503f},
        //        .wheel    = {.wheel_ = {.hcan = &hfdcan1, .id = 0x03}, .inv_wheel = 1}
    },

    // 右后腿 (Leg 3)
    {
        //        .joint[0] = {.motor = {.motor_id = 0x0A, .rs485 = &rs485bus2}, .inv_motor = 1.0f, .pos_offset = 0.1762f},
        //        .joint[1] = {.motor = {.motor_id = 0x0B, .rs485 = &rs485bus2}, .inv_motor = 1.0f, .pos_offset = 10.101f},
        //        .joint[2] = {.motor = {.motor_id = 0x0C, .rs485 = &rs485bus2}, .inv_motor = 2.0f/3.0f, .pos_offset = 15.3891392f},
        //        .wheel    = {.wheel_ = {.hcan = &hfdcan1, .id = 0x04}, .inv_wheel = 1}
        .joint[0] = {.motor = {.motor_id = 0x07, .rs485 = &rs485bus2}, .inv_motor = 1.0f, .pos_offset =  -0.89011f},
        .joint[1] = {.motor = {.motor_id = 0x08, .rs485 = &rs485bus2}, .inv_motor = 1.0f, .pos_offset = -9.162765f},
        .joint[2] = {.motor = {.motor_id = 0x09, .rs485 = &rs485bus2}, .inv_motor = 2.0f/3.0f, .pos_offset = -16.010522f},
        .wheel    = {.wheel_ = {.hcan = &hfdcan1, .id = 0x03}, .inv_wheel = 1}

//        .joint[0] = {.motor = {.motor_id = 0x07, .rs485 = &rs485bus2}, .inv_motor = 1.0f, .pos_offset = -0.0811f},
//        .joint[1] = {.motor = {.motor_id = 0x08, .rs485 = &rs485bus2}, .inv_motor = 1.0f, .pos_offset = -10.0143f},
//        .joint[2] = {.motor = {.motor_id = 0x09, .rs485 = &rs485bus2}, .inv_motor = 2.0f / 3.0f, .pos_offset = -15.0216503f},
//        .wheel = {.wheel_ = {.hcan = &hfdcan1, .id = 0x03}, .inv_wheel = 1}
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

float wheel_Kd = 0.50f;

#if (test)

DMH6215_t DM_motor_ = {.hcan = &hfdcan1, .id = 0x01};

float wheel_exp_torque = 0.0f;
float wheel_exp_omega = 0.0f;
void WheelControlTask(void *param)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    for (uint8_t i = 0; i < 4; i++)
    {
        DMH6215_Enable(&leg[i].wheel.wheel_);
    }

    while (1)
    {

        DMH6215_MIT_Control(&DM_motor_, wheel_exp_rad,
                            wheel_exp_omega, wheel_exp_torque,
                            wheel_Kp, wheel_Kd);

        vTaskDelayUntil(&last_wake_time, 2);
    }
}

#endif

#if (!test)
void WheelControlTask(void *param)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    for (uint8_t i = 0; i < 4; i++)
    {
        DMH6215_Enable(&leg[i].wheel.wheel_);
    }

    while (1)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            DMH6215_MIT_Control(&leg[i].wheel.wheel_, 0.0f,
                                leg[i].wheel.inv_wheel * leg[i].wheel.exp_omega, leg[i].wheel.inv_wheel * leg[i].wheel.exp_torque,
                                0.0f, wheel_Kd);
        }

        vTaskDelayUntil(&last_wake_time, 2);
    }
}
#endif

int size_recve_mot = 0;
int size_usb = 0;
void CDC_Recv_Cb(uint8_t *src, uint16_t size)
{
    if (usb_recv_timeout)
        return;
    size_usb = size;
    size_recve_mot = sizeof(MotorTargetPack_t);
    if (size == sizeof(MotorTargetPack_t))
    {
        if (((MotorTargetPack_t *)src)->pack_type == 0x04)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            memcpy(&legs_target, src, sizeof(MotorTargetPack_t));
            xSemaphoreGive(cdc_recv_semphr);
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
            }
            legs_state.leg[i].wheel.omega = leg[i].wheel.inv_wheel * leg[i].wheel.wheel_.velocity;
            legs_state.leg[i].wheel.torque = leg[i].wheel.inv_wheel * leg[i].wheel.wheel_.torque;
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
    while (first_run_back != 0 || first_run_front != 0) // 等待电机数据准备好
        vTaskDelay(1);
    // TODO:上电时电机角度在极点附近的处理
    //        if(leg[0].joint[0].motor.state.rad>4.0f)
    //            leg[0].joint[0].pos_offset=leg[0].joint[0].pos_offset+6.2831853f;
    //        if(leg[1].joint[2].motor.state.rad<3.0f)
    //            leg[1].joint[2].pos_offset=leg[1].joint[2].pos_offset-6.2831853f;
    //        if(leg[2].joint[2].motor.state.rad<3.0f)
    //            leg[2].joint[2].pos_offset=leg[2].joint[2].pos_offset-6.2831853f;
    //        if(leg[3].joint[1].motor.state.rad<3.0f)
    //            leg[3].joint[1].pos_offset=leg[3].joint[1].pos_offset-6.2831853f;
    for (int i = 0; i < 4; i++)
    {
        setup_offset[i][0] = leg[i].joint[0].motor.state.rad;
        setup_offset[i][1] = leg[i].joint[1].motor.state.rad;
        setup_offset[i][2] = leg[i].joint[2].motor.state.rad;
    }
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15))
    {
        vTaskDelay(50);
    }
    allow_send = 1;
    vTaskDelay(1000);
    HAL_TIM_Base_Start_IT(&htim13);
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
        leg[i].wheel.exp_omega = 0.0f;
        leg[i].wheel.exp_torque = 0.0f;
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
                leg[i].wheel.exp_omega = 0.0f;
                leg[i].wheel.exp_torque = 0.0f;
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
            leg[i].wheel.exp_omega = legs_target.leg[i].wheel.omega;
            leg[i].wheel.exp_torque = legs_target.leg[i].wheel.torque;
            leg[i].timestamp = legs_target.timestamp;
        }
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    uint8_t buf[8];
    if (hfdcan->Instance == FDCAN1 && (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {

        uint32_t id = CAN_Receive_DataFrame(hfdcan, buf);
#if (!test)
        DMH6215_Recv_Handle(&leg[0].wheel.wheel_, hfdcan, id, buf);
        DMH6215_Recv_Handle(&leg[1].wheel.wheel_, hfdcan, id, buf);
        DMH6215_Recv_Handle(&leg[2].wheel.wheel_, hfdcan, id, buf);
        DMH6215_Recv_Handle(&leg[3].wheel.wheel_, hfdcan, id, buf);
#endif

#if (test)

        DMH6215_Recv_Handle(&DM_motor_, hfdcan, id, buf);
#endif
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
    if (huart->Instance == USART10)
    {
        JY61_Receive(&JY61, data, size);
        HAL_UART_DMAStop(&huart10);
        __HAL_UART_CLEAR_IDLEFLAG(&huart10);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, data, sizeof(data));
        __HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);
    }
    else if (huart->Instance == UART7)
    {

        if (remote_control_buf[0] == 0xAA)
        {
            memcpy(&remotedata, remote_control_buf, 12);

            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(remote_semaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, remote_control_buf, sizeof(remote_control_buf));
    }
    else if (huart->Instance == USART2)
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

    else if (huart->Instance == USART10)
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        HAL_UART_DMAStop(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, data, sizeof(data));
    }
    else if (huart->Instance == UART7)
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        HAL_UART_DMAStop(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, remote_control_buf, sizeof(remote_control_buf));
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

// void UART7_RemotecontrolTask(void *param)
//{

//    TickType_t xLastWakeTime = xTaskGetTickCount(); // 获取当前 Tick
//    const TickType_t xFrequency = pdMS_TO_TICKS(100);

//    while (1)
//    {

//        float vel[3];
//        // 阻塞等待 ISR 释放的信号量
//        if (xSemaphoreTake(remote_semaphore, pdMS_TO_TICKS(200)) != pdTRUE)
//        { // 200ms未收到遥控器的数据，复位摇杆
//            remotedata.rocker[0] = 0;
//            remotedata.rocker[1] = 0;
//            remotedata.rocker[2] = 0;
//            remotedata.rocker[3] = 0;
//        }

//        __disable_irq();
//        last_v0 = filter_gate * (((float)(remotedata.rocker[0])) / 2047.0f) + (1.0f - filter_gate) * last_v0;
//        vel[0] = last_v0;
//        last_v1 = filter_gate * (((float)(remotedata.rocker[1])) / 2047.0f) + (1.0f - filter_gate) * last_v1;
//        vel[1] = last_v1;
//        last_omega = (((float)(remotedata.rocker[2])) / 2047.0f) * max_omega * filter_gate + (1.0f - filter_gate) * last_omega;
//        key1 = remotedata.key1;

//        __enable_irq();

//        legs_state.remote_cmd.omega = BezierTransform(last_omega, bezier); // 计算自转角速度
//        legs_state.remote_cmd.omega = legs_state.remote_cmd.omega * M_PI / 180.0f;

//        float rocker_val = (float)remotedata.rocker[3] / 2047.0f;
//        last_v3 = filter_alpha * rocker_val + (1.0f - filter_alpha) * last_v3;
//        vel[2] = last_v3 * 1.0f;

//        // 归一化第四个摇杆值

//        float model = sqrtf(vel[0] * vel[0] + vel[1] * vel[1]);
//        if (model != 0.0f)
//            cur_dir = atan2f(vel[1], vel[0]);

//        model = BezierTransform(model, bezier);
//        vel[1] = model * sinf(cur_dir);

//        if (vel[1] >= 0)
//        {
//            vel[1] = vel[1] * max_forword_speed;
//        }
//        else if (vel[1] < 0)
//        {
//            vel[1] = vel[1] * max_backward_speed;
//        };

//        vel[0] = model * cosf(cur_dir) * max_speed;

//        legs_state.remote_cmd.vy = vel[0];
//        legs_state.remote_cmd.vx = vel[1];
//        legs_state.remote_cmd.wheel_v = vel[2];

//        vTaskDelayUntil(&xLastWakeTime, xFrequency);
//    }
//}
