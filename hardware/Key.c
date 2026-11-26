#include "stm32f10x.h"                  // Device header
#include "Delay.h"

#define key1 GPIO_Pin_0
#define key2 GPIO_Pin_4
#define key3 GPIO_Pin_6

void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin=key1|key2|key3;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
}

uint8_t Key_GetNum(void)	//uint8_t==unsigned char
{
	uint8_t KeyNum=0;
	
	if(GPIO_ReadInputDataBit(GPIOA,key1)==0)
	{
		Delay_ms(20);
		while(GPIO_ReadInputDataBit(GPIOA,key1)==0);
		Delay_ms(20);
		KeyNum=1;
	}
	
	if(GPIO_ReadInputDataBit(GPIOA,key2)==0)
	{
		Delay_ms(20);
		while(GPIO_ReadInputDataBit(GPIOA,key2)==0);
		Delay_ms(20);
		KeyNum=2;
	}
	
	if(GPIO_ReadInputDataBit(GPIOA,key3)==0)
	{
		Delay_ms(20);
		while(GPIO_ReadInputDataBit(GPIOA,key3)==0);
		Delay_ms(20);
		KeyNum=3;
	}
	
	return KeyNum;
}

