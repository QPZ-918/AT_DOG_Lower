
#ifndef __RUN_H__
#define __RUN_H__

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "motorEx.h"
#include "go_motor.h"
/**
 * @brief 关节结构体 - 单个关节电机的完整控制参数
 */
typedef struct {
    GO_MotorHandle_t motor;    /**< 电机句柄（包含 ID、RS485 总线指针等） */
    float pos_offset;          /**< 角度零点偏移量 (rad)，用于校准机械安装误差 */
    float inv_motor;          /**< 电机方向标志：1-正转，-1-反转 */

    float exp_rad;             /**< 期望角度 (rad) */
    float exp_omega;           /**< 期望角速度 (rad/s) */
    float exp_torque;          /**< 期望力矩 (N·m) */

    float Kp;                  /**< 位置环比例增益 */
    float Kd;                  /**< 位置环微分增益 */
} Joint_t;


/**
 * @brief 单条腿结构体 - 包含 3 个关节电机和 1 个轮子电机
 */
typedef struct {
    Joint_t joint[3];          /**< 3 个关节电机（髋关节、大腿、小腿） */
    uint32_t timestamp;
} Leg_t;


#pragma pack(1)                /**< 1 字节对齐，确保数据包紧凑传输，避免内存空洞 */

/* ==================== 下行数据包（PC → 下位机）==================== */

/**
 * @brief 单个关节电机的目标值
 */
typedef struct {
    float rad;                 /**< 目标角度 (rad) */
    float omega;               /**< 目标角速度 (rad/s) */
    float torque;              /**< 目标力矩 (N·m) */
    float kp;                  /**< 位置环比例增益 */
    float kd;                  /**< 位置环微分增益 */
} MotorTarget_t;


/**
 * @brief 单条腿的目标值集合
 */
typedef struct {
    MotorTarget_t joint[3];    /**< 3 个关节的目标结构体 */
} LegTarget_t;



/**
 * @brief 完整的电机控制数据包（下行 - 从 PC 到控制器）
 */
typedef struct {
    int pack_type;             /**< 数据包类型标识：0x00 表示电机控制包 */
    LegTarget_t leg[4];        /**< 4 条腿的目标结构体 */
		uint32_t timestamp;
} MotorTargetPack_t;


/* ==================== 上行数据包（下位机 → PC）==================== */

/**
 * @brief 三维向量结构体 - 用于表示角速度、加速度等
 */
typedef struct {
    float X, Y, Z;             /**< 三轴分量（机体坐标系） */
} Vector3D_Typedef_;


 typedef struct {
   Vector3D_Typedef_ AngularVelocity;
   struct {
     float q0, q1, q2 ,q3;
   } Angle;
 } IMU_Typedef_;

/**
 * @brief 单个关节电机的状态反馈
 */
typedef struct {
    float rad;                 /**< 实际角度 (rad) */
    float omega;               /**< 实际角速度 (rad/s) */
    float torque;              /**< 实际力矩 (N·m)（估算值） */
} MotorState_t;

/**
 * @brief 单条腿的状态反馈集合
 */
typedef struct {
    MotorState_t joint[3];     /**< 3 个关节的状态反馈 */
} LegState_t;

/**
 * @brief 完整的状态反馈数据包（上行 - 从控制器到 PC）
 */
typedef struct {
    int pack_type;             /**< 数据包类型标识：0x00 表示状态反馈包 */
    LegState_t leg[4];         /**< 4 条腿的状态反馈 */
		IMU_Typedef_ imu_data;
    uint16_t watch_dog;        /**< 看门狗标志位：bit 位为 1 表示对应电机掉线 */
	uint32_t timestamp;
} MotorStatePack_t;

#pragma pack()


void MotorControlTask_Front(void* param);
void MotorControlTask_Back(void* param);
void MotorSendTask(void* param);
void MotorRecvTask(void* param);
void BMI088_task(void *param);
#endif
