#include "stm32f10x.h"                  // Device header
#include "LightSensor.h"
#include "Serial.h"

extern uint8_t KeyNum;

static uint8_t led_state = 0;	// 灯状态 0=关 1=开
static uint8_t Last_Light_State = 0; // 记录上一次的光敏状态

#define LED_1	GPIO_Pin_1	//PinA1

void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin=LED_1;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA,LED_1);
	
}

void LED1_ON(void)//点亮LED1
{
	GPIO_ResetBits(GPIOA,LED_1);
}

void LED1_OFF(void)//熄灭PA1口LED
{
	GPIO_SetBits(GPIOA,LED_1);
}

void LED1_Turn(void)//PA1口LED翻转
{
	if(GPIO_ReadOutputDataBit(GPIOA,LED_1)==0)
		GPIO_SetBits(GPIOA,LED_1);
	else 
		GPIO_ResetBits(GPIOA,LED_1);
}

/* LED任务 */
void LED_Task(uint8_t rx)
{
	// 1. 获取当前环境状态
    uint8_t Current_Light_State = LightSensor_Get(); // 1暗, 0亮
	
	// 2. 环境触发逻辑：仅在“由亮变暗”或“由暗变亮”的瞬间动作
    if (Current_Light_State == 1 && Last_Light_State == 0) // 由亮变暗
    {
        led_state = 1; 
    }
    else if (Current_Light_State == 0 && Last_Light_State == 1) // 由暗变亮
    {
        led_state = 0;
    }
    Last_Light_State = Current_Light_State; // 更新历史状态

    // 3. 按键触发逻辑：按下按键3翻转状态
    if (KeyNum == 3) 
    {
        led_state = !led_state; 
    }

    // 4. 串口触发逻辑：收到01开，02关
    if (rx)
    {
        if (Serial_RxPacket[0] == 1)
		{			
			led_state = 1;
			Serial_SendNumber(8,3);
		}
        else if (Serial_RxPacket[0] == 2) led_state = 0;
    }

    // 5. 最终输出硬件控制
    if (led_state) LED1_ON();
    else           LED1_OFF();

}

