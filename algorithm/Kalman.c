#include "Kalman.h"

/*
 * 一维卡尔曼滤波
 * 状态变量：
 *   angle —— 角度
 *   bias  —— 陀螺仪零偏
 *
 * 测量值：加速度计算得到的角度
 * 输入值：陀螺仪角速度
 */

void Kalman_Init(Kalman_t *kalman)
{
    kalman->angle = 0.0f;
    kalman->bias  = 0.0f;

    kalman->P[0][0] = 1.0f;
    kalman->P[0][1] = 0.0f;
    kalman->P[1][0] = 0.0f;
    kalman->P[1][1] = 1.0f;
}

/*
 * 卡尔曼滤波更新
 * acc_angle : 由加速度计算得到的角度（°）
 * gyro_rate : 陀螺仪角速度（°/s）
 * dt        : 采样周期（s）
 */
float Kalman_Update(Kalman_t *kalman, float acc_angle, float gyro_rate, float dt)
{
    // 可调参数
    const float Q_angle   = 0.001f;   // 角度过程噪声
    const float Q_bias    = 0.003f;   // 偏置过程噪声
    const float R_measure = 0.03f;    // 测量噪声

    /* ---------- 预测阶段 ---------- */
    // 状态预测
    kalman->angle += (gyro_rate - kalman->bias) * dt;

    // 协方差预测
    kalman->P[0][0] += dt * (dt * kalman->P[1][1]
                            - kalman->P[0][1]
                            - kalman->P[1][0]
                            + Q_angle);

    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += Q_bias * dt;

    /* ---------- 更新阶段 ---------- */
    // 计算卡尔曼增益
    float S  = kalman->P[0][0] + R_measure;
    float K0 = kalman->P[0][0] / S;
    float K1 = kalman->P[1][0] / S;

    // 测量残差
    float y = acc_angle - kalman->angle;

    // 状态更新
    kalman->angle += K0 * y;
    kalman->bias  += K1 * y;

    // 协方差更新
    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];

    kalman->P[0][0] -= K0 * P00_temp;
    kalman->P[0][1] -= K0 * P01_temp;
    kalman->P[1][0] -= K1 * P00_temp;
    kalman->P[1][1] -= K1 * P01_temp;

    return kalman->angle;
}
