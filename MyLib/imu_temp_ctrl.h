#ifndef IMU_TEMP_CTRL_H
#define IMU_TEMP_CTRL_H
void IMU_task(void * argument);

typedef struct
{
  float q[4];        // 四元数
  float Gyro[3];     // 角速度（rad/s）
  float Accel[3];    // 角加速度（rad/s^2）
} BMI088_data;


void INS_Init(void);
void INS_Task(BMI088_data *data);

#endif // IMU_TEMP_CTRL_H
