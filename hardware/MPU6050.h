#ifndef __MPU6050_H
#define __MPU6050_H

void MPU6050_Init(void);
void MPU6050_Update(float dt);   // 统一更新函数（10ms 调一次）

float MPU6050_GetAccTotal(void);
float MPU6050_GetGyroTotal(void);
float MPU6050_GetPitch(void);
float MPU6050_GetRoll(void);

#endif
