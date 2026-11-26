#ifndef __HR_SPO2_H
#define __HR_SPO2_H

#include "stm32f10x.h"

/* 初始化 */
void HRSpO2_Init(void);

/* 10ms周期调用 */
void HRSpO2_10msHandler(void);

/* 1秒周期调用 */
void HRSpO2_1sHandler(void);

/* 状态获取 */
uint8_t HRSpO2_IsFingerPresent(void);
uint8_t HRSpO2_IsReady(void);

/* 数据获取 */
int32_t HRSpO2_GetHR(void);
int32_t HRSpO2_GetSpO2(void);

/* 心率血氧播报任务 */
void Hr_O2_Speak(uint8_t rx);

#endif
