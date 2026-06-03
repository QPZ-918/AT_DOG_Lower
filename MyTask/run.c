#include "run.h"
#include "usart.h"
#include "usb_trans.h"
#include <cstdint>
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

// JY61_Typedef JY61;
// 添加错误统计结构
// typedef struct
// {
//     uint32_t total;
//     uint32_t overrun;
//     uint32_t frame;
//     uint32_t noise;
//     uint32_t parity;
//     uint32_t last_error_time;
//     uint32_t continuous_errors;
//     uint32_t recovery_attempts;
//     uint32_t last_recovery_time;
// } ErrorStats_t;
// ErrorStats_t error_stats = {0};
uint32_t error_cnt = 0;
uint32_t error_cnt2 = 0;
uint32_t success_cnt = 0;
uint32_t reast_cnt = 0;
// uint8_t data[11] __attribute__((section("RAM_D2_OTHER"),aligned(32)));
// uint32_t req_stop_transmit;

//******有关遥控器需要的数据定义********

// static BezierLine bezier = {.p1_x = 0.660634f, .p1_y = 0.131222f, .p2_x = 0.846154f, .p2_y = 0.556561f}; // 摇杆贝塞尔曲线参数
// uint8_t remote_control_buf[12] __attribute__((section("RAM_D2_OTHER"),aligned(32)));
// float filter_gate = 1.0f, last_v0, last_v1, last_omega, last_v3;
// static const float filter_alpha = 0.2f;
// float max_omega = 120.0f;
// float max_forword_speed = 1.0f, max_backward_speed = 0.5f, max_speed = 0.4f;
// float cur_dir = 0.0f;
// float key1 = 0, key2 = 0;

// RemotePack_t remotedata; // 接收遥控数据的结构体

// extern QueueHandle_t remote_semaphore;

//**************************************

// extern DMA_HandleTypeDef hdma_usart10_rx;
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
     .joint[2] = {.motor = {.motor_id = 0x06, .rs485 = &rs485bus}, .inv_motor = -0.5, .pos_offset = 13.056006f},
     .wheel = {.wheel_ = {.hcan = &hfdcan1, .id = 0x03}, .inv_wheel = 1}},
    // 左前腿
    {.joint[0] = {.motor = {.motor_id = 0x01, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = -4.292375f},
     .joint[1] = {.motor = {.motor_id = 0x02, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = -6.875717f},
     .joint[2] = {.motor = {.motor_id = 0x03, .rs485 = &rs485bus}, .inv_motor = -0.5, .pos_offset = -13.226964f},
     .wheel = {.wheel_ = {.hcan = &hfdcan1, .id = 0x01}, .inv_wheel = 1}},

    // 右后腿
    {.joint[0] = {.motor = {.motor_id = 0x0A, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = -4.234103f},
     .joint[1] = {.motor = {.motor_id = 0x0B, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 6.392829f},
     .joint[2] = {.motor = {.motor_id = 0x0C, .rs485 = &rs485bus}, .inv_motor = -0.5, .pos_offset = 12.753429f},
     .wheel = {.wheel_ = {.hcan = &hfdcan1, .id = 0x04}, .inv_wheel = 1}},
    // 左后腿
    {.joint[0] = {.motor = {.motor_id = 0x07, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 4.089883f},
     .joint[1] = {.motor = {.motor_id = 0x08, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = -6.996269f},
     .joint[2] = {.motor = {.motor_id = 0x09, .rs485 = &rs485bus}, .inv_motor = -0.5, .pos_offset = -13.355066f},
     .wheel = {.wheel_ = {.hcan = &hfdcan1, .id = 0x02}, .inv_wheel = 1}}
};

float setup_offset[4][3]; // 上电启动时的电机角度
uint32_t first_run = 5;
// uint32_t watch_dog_id[24];
uint32_t reset_uart;
uint32_t bad_Motor = 0;
uint64_t uart_reast = 0;
int err_check = 0;

float KPP[12];
float KDD[12];

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
							KPP[i*3+j] = leg[i].joint[j].Kp / (6.33f * 6.33f / (leg[i].joint[j].inv_motor * leg[i].joint[j].inv_motor));
							KDD[i*3+j] = leg[i].joint[j].Kd / (6.33f * 6.33f / (leg[i].joint[j].inv_motor * leg[i].joint[j].inv_motor));

//               GoMotorSend(&leg[i].joint[j].motor, leg[i].joint[j].exp_torque / 6.33f * leg[i].joint[j].inv_motor,
//                            leg[i].joint[j].exp_omega * 6.33f / leg[i].joint[j].inv_motor,
//                            leg[i].joint[j].exp_rad * 6.33f / leg[i].joint[j].inv_motor + leg[i].joint[j].pos_offset + setup_offset[i][j],
//                            leg[i].joint[j].Kp / (6.33f * 6.33f / (leg[i].joint[j].inv_motor * leg[i].joint[j].inv_motor)),
//                            leg[i].joint[j].Kd / (6.33f * 6.33f / (leg[i].joint[j].inv_motor * leg[i].joint[j].inv_motor)));
							GoMotorSend(&leg[i].joint[j].motor, leg[i].joint[j].exp_torque / 6.33f * leg[i].joint[j].inv_motor,
                            leg[i].joint[j].exp_omega * 6.33f / leg[i].joint[j].inv_motor,
                            leg[i].joint[j].exp_rad * 6.33f / leg[i].joint[j].inv_motor + leg[i].joint[j].pos_offset + setup_offset[i][j],
                            KPP[i*3+j],
                            KDD[i*3+j]);
                // err_check+=GoMotorRecv(&leg[i].joint[j].motor);
                // FakeMotor_SendReply(leg[i].joint[j].motor.motor_id, leg[i].joint[j].exp_torque,leg[i].joint[j].exp_omega,leg[i].joint[j].exp_rad);

                int ret = GoMotorRecv(&leg[i].joint[j].motor);
                if (ret)
                {
                    bad_Motor &= (~(0x0001 << (i * 3 + j)));
                    // FeedDog(watch_dog_id[i * 3 + j]);
                    // FeedDog(watch_dog_id[i * 3 + j + 12]);
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

// kp 应为0.00f
float wheel_Kp = 0.00f;
float wheel_exp_rad = 0.0f;
float wheel_Kd = 0.20f;

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
    vTaskDelay(1000);
    for (uint8_t j = 0; j < 3; j++)
    {
        for (uint8_t i = 0; i < 2; i++)
        {
            DMH6215_Enable(&leg[i].wheel.wheel_);
        }
        vTaskDelay(2);
        for (uint8_t i = 2; i < 4; i++)
        {
            DMH6215_Enable(&leg[i].wheel.wheel_);
        }

        vTaskDelay(2);
    }
    static uint16_t wheel_enable = 0;
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1)
    {
        vTaskDelayUntil(&last_wake_time, 2);
        for (uint8_t i = 0; i < 2; i++)
        {
            DMH6215_MIT_Control(&leg[i].wheel.wheel_, wheel_exp_rad,
                                leg[i].wheel.inv_wheel * leg[i].wheel.exp_omega, leg[i].wheel.inv_wheel * leg[i].wheel.exp_torque,
                                wheel_Kp, wheel_Kd);
        }
        vTaskDelayUntil(&last_wake_time, 2);
        for (uint8_t i = 2; i < 4; i++)
        {
            DMH6215_MIT_Control(&leg[i].wheel.wheel_, wheel_exp_rad,
                                leg[i].wheel.inv_wheel * leg[i].wheel.exp_omega, leg[i].wheel.inv_wheel * leg[i].wheel.exp_torque,
                                wheel_Kp, wheel_Kd);
        }
        wheel_enable++;
        wheel_enable %= 125;
        if(!wheel_enable)
        {
            for (uint8_t i = 0; i < 2; i++)
            {
                DMH6215_Enable(&leg[i].wheel.wheel_);
            }
            vTaskDelay(2);
            for (uint8_t i = 2; i < 4; i++)
            {
                DMH6215_Enable(&leg[i].wheel.wheel_);
            }
        }
    }
}
#endif

uint32_t current_size = 0;
uint32_t cnt = 0;
void CDC_Recv_Cb(uint8_t *src, uint16_t size)
{
    if (size == sizeof(MotorTargetPack_t) && ((MotorTargetPack_t *)src)->pack_type == 0x04)
    {
        // SCB_InvalidateDCache_by_Addr((uint32_t*)src, CACHE_ALIGN(sizeof(MotorTargetPack_t)));
        //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        memcpy(&legs_target, src, sizeof(MotorTargetPack_t));
        xSemaphoreGive(cdc_recv_semphr);
    }
    cnt++;
    current_size = size;
    // HAL_UART_Transmit_DMA(&huart3, src, size);
}

// PID2 wheel_vel_pid[4];
// float wheel_exp_vel[4], wheel_exp_torque[4];
// int16_t can_send_buf[4];
// float inv_wheel[4] = {-1.0f, 1.0f, -1.0f, 1.0f};
// void WheelControlTask(void *param)
// {
//     TickType_t last_wake_time = xTaskGetTickCount();
//     leg[0].wheel.vel_pid.Kp = 0.55f;
//     leg[0].wheel.vel_pid.Ki = 0.04f;
//     leg[0].wheel.vel_pid.limit = 300.0f;
//     leg[0].wheel.vel_pid.output_limit = 4.0f;
//     leg[3].wheel.vel_pid = leg[2].wheel.vel_pid = leg[1].wheel.vel_pid = leg[0].wheel.vel_pid;
//     while (1)
//     {
//         for (int i = 0; i < 4; i++)
//         {
//             PID_Control2(leg[i].wheel.motor.Speed * 3.14159265f * 2.0f / 60.0f / 19.0f, wheel_exp_vel[i] * inv_wheel[i], &leg[i].wheel.vel_pid);
//             float out_temp = ((leg[i].wheel.vel_pid.pid_out + wheel_exp_torque[i] * inv_wheel[i]) / 0.3f * (16384.0f / 20.0f / 0.3f));
//             if (out_temp > 16384)
//                 out_temp = 16384;
//             else if (out_temp < -16384)
//                 out_temp = -16384;
//             can_send_buf[i] = (int16_t)out_temp;
//         }
//         MotorSend(&hcan1, 0x200, can_send_buf);
//         vTaskDelayUntil(&last_wake_time, 2);
//     }
// }
uint16_t count = 0;
uint8_t allow_send = 0;
// JY61_Typedef_ JY61_2;
uint8_t flag_check = 1;
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
						legs_state.tim_p = leg[i].tim_p;
            // TODO:根据反馈计算真实力矩
        }

        // JY61_2.AngularVelocity.X = JY61.AngularVelocity.X * 3.1415926 / 180.0f;
        // JY61_2.AngularVelocity.Y = JY61.AngularVelocity.Y * 3.1415926 / 180.0f;
        // JY61_2.AngularVelocity.Z = JY61.AngularVelocity.Z * 3.1415926 / 180.0f;

        // JY61_2.Angle.Roll = JY61.Angle.Roll * 3.1415926 / 180.0f;
        // JY61_2.Angle.Pitch = JY61.Angle.Pitch * 3.1415926 / 180.0f;
        // JY61_2.Angle.Yaw = JY61.Angle.Yaw * 3.1415926 / 180.0f;

        // if (JY61_2.AngularVelocity.X > 2.0f || JY61_2.AngularVelocity.X < -2.0f)
        //     flag_check = 0;
        // else if (JY61_2.AngularVelocity.Y > 2.0f || JY61_2.AngularVelocity.Y < -2.0f)
        //     flag_check = 0;
        // else if (JY61_2.AngularVelocity.Z > 2.0f || JY61_2.AngularVelocity.Z < -2.0f)
        //     flag_check = 0;
        // else if (JY61_2.Angle.Roll > 1.0f || JY61_2.Angle.Roll < -1.0f)
        //     flag_check = 0;
        // else if (JY61_2.Angle.Pitch > 1.0f || JY61_2.Angle.Pitch < -1.0f)
        //     flag_check = 0;

        // if (flag_check)
        // {
        legs_state.imu_data.Angle.q0 = IMU_data.q[0];
        legs_state.imu_data.Angle.q1 = IMU_data.q[1];
        legs_state.imu_data.Angle.q2 = IMU_data.q[2];
        legs_state.imu_data.Angle.q3 = IMU_data.q[3];
        legs_state.imu_data.AngularVelocity.X = IMU_data.Gyro[0];
        legs_state.imu_data.AngularVelocity.Y = IMU_data.Gyro[1];
        legs_state.imu_data.AngularVelocity.Z = IMU_data.Gyro[2];
        // }
        // else
        //     flag_check = 1;
        if (allow_send) // 电机数据准备好再发
                        // memcpy(UserTxBufferHS, &legs_state, sizeof(legs_state));
                        // SCB_CleanDCache_by_Addr((uint32_t*)UserTxBufferHS, CACHE_ALIGN(sizeof(legs_state)));
                        // CDC_Transmit_HS(UserTxBufferHS, sizeof(legs_state));
            CDC_Transmit_HS((uint8_t *)&legs_state, len);

        // CDC_Transmit_HS((uint8_t *)"HELLO\r\n", 7);
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
                leg[i].wheel.exp_omega = 0.0f;
                leg[i].wheel.exp_torque = 0.0f;
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
				cnt_imu++;
        INS_Task(&IMU_data);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
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

uint32_t Fack_Motor_TX = 0;

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
