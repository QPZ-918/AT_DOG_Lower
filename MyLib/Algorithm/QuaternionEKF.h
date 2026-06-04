/**
 ******************************************************************************
 * @file    QuaternionEKF.h
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _QUAT_EKF_H
#define _QUAT_EKF_H
#include "kalman_filter.h"

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif




typedef struct
{
    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float q[4];        // 四元数估计值
    float q_prev[4];   // 前一时刻的四元数，用于计算被滤波的角速度
    float GyroBias[3]; // 陀螺仪零偏估计值

    float Gyro[3];     // 去偏置后的角速度(中间量)
    float GyroFiltered[3]; // EKF完整滤波后的角速度（基于后验四元数导数）
    float GyroPrev[3]; // 上一次的滤波角速度，用于计算角加速度
    float GyroAccel[3]; // 角加速度状态估计值（参与观测更新）

    KalmanFilter_t GyroAccelKF; // 角加速度状态滤波器: x=[ax,ay,az], z=[ax_meas,ay_meas,az_meas]

    float Accel[3];

    float OrientationCosine[3];

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float Roll;
    float Pitch;
    float Yaw;

    float YawTotalAngle;

    float Q1; // 四元数更新过程噪声
    float Q2; // 陀螺仪零偏过程噪声
    float R;  // 加速度计量测噪声

    float dt; // 姿态更新周期
    mat ChiSquare;
    float ChiSquare_Data[1];      // 卡方检验检测函数
    float ChiSquareTestThreshold; // 卡方检验阈值
    float lambda;                 // 渐消因子

    int16_t YawRoundCount;

    float YawAngleLast;
} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;
extern float chiSquare;
extern float ChiSquareTestThreshold;
void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda, float dt, float lpf);
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az);
void IMU_QuaternionEKF_Reset(void);

float Get_Pitch(void);//get pitch
float Get_Roll(void);//get roll
float Get_Yaw(void);//get yaw

// 获取融合后的数据
void Get_Quaternion(float q[4]);
void Get_AngularVelocity(float gyro[3]); // 返回EKF完整滤波后的角速度
void Get_FilteredAngularVelocity(float gyro[3]); // 获取EKF完整滤波后的角速度
void Get_AngularAcceleration(float gyro_accel[3]); // 返回由滤波角速度求导的角加速度
#endif
