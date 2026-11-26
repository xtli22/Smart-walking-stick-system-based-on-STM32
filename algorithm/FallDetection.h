#ifndef __FALLDETECTION_H
#define __FALLDETECTION_H


void FallDetection_Init(void);
void FallDetection_Recover(void);
uint8_t FallDetection_Update(uint32_t dt);

#endif

