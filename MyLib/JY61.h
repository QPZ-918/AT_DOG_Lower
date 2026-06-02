#ifndef _JY61_H_
#define _JY61_H_

#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif



typedef struct {
    uint8_t head;
    uint8_t ID;
    int16_t X, Y, Z, Temp;
    uint8_t sum;
} Angle_Pack_Typedef;


typedef struct {
    float X, Y, Z;
} Vector3D_Typedef;

typedef struct {
  Vector3D_Typedef Acceleration;
  Vector3D_Typedef AngularVelocity;
  struct {
    float Yaw, Pitch, Roll;
    float lastYaw;
    int32_t rand;
    float Multiturn;
  } Angle;
  float Temp;
} JY61_Typedef;



void JY61_Receive(JY61_Typedef* Gyro, uint8_t *data, uint8_t len);


#ifdef __cplusplus
}
#endif


#endif
