#ifndef __USART2_H
#define __USART2_H

#include <stdio.h>

#define USART2_REC_LEN  512        // 定义接收缓冲区大小

extern char    USART2_RxPacket[USART2_REC_LEN];
extern uint8_t USART2_RxFlag;

void usart2_Init(void);
void usart2_SendByte(uint8_t Byte);
void usart2_SendString(char *String);
void usart2_SendHexString(char *hex);
void usart2_Printf(char *format, ...);
void usart2_ClearPacket(void);

#endif
