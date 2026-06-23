#ifndef __RUN_H__
#define __RUN_H__

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "motorEx.h"
#include "go_motor.h"
#include "dm_h6215.h"
typedef struct{
    GO_MotorHandle_t motor;
    float pos_offset;
    float inv_motor;

    float exp_rad;
    float exp_omega;
    float exp_torque;

    float Kp;
    float Kd;
}Joint_t;

typedef struct{
    DMH6215_t wheel_;
    int8_t inv_wheel;
    float exp_omega;
    float exp_torque;
}DMH6215_t_;

typedef struct{
    Joint_t joint[3];
    DMH6215_t_ wheel;
	  uint32_t tim_p;
}Leg_t;


#pragma pack(1)


typedef struct{
    float rad;
    float omega;
    float torque;
    float kp;
    float kd;
}MotorTarget_t;

typedef struct{
    float omega;
    float torque;
}WheelTarget_t;

typedef struct{
    MotorTarget_t joint[3];
    WheelTarget_t wheel;
}LegTarget_t;

typedef struct{
    int pack_type;
    LegTarget_t leg[4];
	uint32_t tim_p;
}MotorTargetPack_t;



 typedef struct {
     float X, Y, Z;
 } Vector3D_Typedef_;



 typedef struct {
   Vector3D_Typedef_ AngularVelocity;
   struct {
     float q0, q1, q2 ,q3;
   } Angle;
 } IMU_Typedef_;

typedef struct{
    float rad;
    float omega;
    float torque;
}MotorState_t;

typedef struct{
    float omega;
    float torque;
}WheelState_t;

typedef struct{
    MotorState_t joint[3];
    WheelState_t wheel;
}LegState_t;

typedef struct{
    int pack_type;
    LegState_t leg[4];
    IMU_Typedef_ imu_data;
    uint16_t watch_dog;
	uint32_t tim_p;
}MotorStatePack_t;

#pragma pack()


void MotorControlTask(void* param);
void MotorSendTask(void* param);
void MotorRecvTask(void* param);
void BMI088_task(void *param);
#endif