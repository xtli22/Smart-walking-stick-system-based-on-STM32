#ifndef __KALMAN_H
#define __KALMAN_H

typedef struct
{
    float angle;	// 当前估计角度
    float bias;		// 陀螺仪零偏
    float P[2][2];	// 协方差矩阵
} Kalman_t;

void Kalman_Init(Kalman_t *kalman);
float Kalman_Update(Kalman_t *kalman, float acc_angle, float gyro_rate, float dt);

#endif
