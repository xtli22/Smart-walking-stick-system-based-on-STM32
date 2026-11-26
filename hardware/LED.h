#ifndef __LED_H
#define __LED_H

void LED_Init(void);

void LED1_ON(void);//点亮PA5口LED
void LED1_OFF(void);//熄灭PA5口LED
void LED1_Turn(void);//PA1口LED翻转

void LED_Task(uint8_t rx);

#endif
