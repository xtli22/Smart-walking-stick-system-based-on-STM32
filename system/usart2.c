#include "stm32f10x.h"                  // Device header
#include "usart2.h"
#include <stdarg.h>
#include <string.h>

char    USART2_RxPacket[USART2_REC_LEN]; // 接收缓冲区
uint16_t USART2_RxPos = 0;               // 当前接收位置
uint8_t  USART2_RxFlag = 0;              // 接收完成标志

void usart2_Init(void)
{
	/* 1. 时钟开启 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	/* 2. GPIO配置 */
	GPIO_InitTypeDef GPIO_InitStructure;
	// PA2 - TX: 复用推挽输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// PA3 - RX: 浮空输入 (适配 Air780E 强推挽输出)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* 3. USART2配置 */
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200; // Air780E 默认波特率
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &USART_InitStructure);
	
	/* 4. 中断配置 */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // 开启接收中断
    // 若追求更高稳定性，可在此开启 USART_IT_IDLE 空闲中断
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	/* 5. 使能 */
	USART_Cmd(USART2, ENABLE);
}

void usart2_SendByte(uint8_t Byte)
{
	USART_SendData(USART2, Byte);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

void usart2_SendString(char *String)
{
	for (uint16_t i = 0; String[i] != '\0'; i++)
	{
		usart2_SendByte(String[i]);
	}
}

void usart2_SendHexString(char *hex)
{
    uint8_t high, low;

    while(*hex && *(hex+1))
    {
        // 高4位
        if(*hex >= '0' && *hex <= '9') high = *hex - '0';
        else if(*hex >= 'A' && *hex <= 'F') high = *hex - 'A' + 10;
        else if(*hex >= 'a' && *hex <= 'f') high = *hex - 'a' + 10;
        else return;
        hex++;

        // 低4位
        if(*hex >= '0' && *hex <= '9') low = *hex - '0';
        else if(*hex >= 'A' && *hex <= 'F') low = *hex - 'A' + 10;
        else if(*hex >= 'a' && *hex <= 'f') low = *hex - 'a' + 10;
        else return;
        hex++;

        usart2_SendByte((high<<4) | low);
    }
}

void usart2_Printf(char *format, ...)
{
	char String[256];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	usart2_SendString(String);
}

/**
  * 清除接收缓冲区
  */
void usart2_ClearPacket(void)
{
	memset(USART2_RxPacket, 0, USART2_REC_LEN);
	USART2_RxPos = 0;
	USART2_RxFlag = 0;
}

/**
  * USART2 中断服务函数
  */
void USART2_IRQHandler(void)
{
	// 错误处理：检测并清除溢出错误 (ORE)
	if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) == SET)
	{
		USART_ReceiveData(USART2); // 读DR寄存器可清除ORE标志
	}

	// 正常接收中断
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		uint8_t RxData = USART_ReceiveData(USART2);
		
		// 边界保护：防止缓冲区溢出
		if (USART2_RxPos < USART2_REC_LEN - 1)
		{
			USART2_RxPacket[USART2_RxPos++] = RxData;
			
			// 简单的逻辑判断：检测到 \n 则认为一条指令结束（针对 AT 指令回复）
			if (RxData == '\n') 
			{
				USART2_RxFlag = 1;
			}
		}
        // 注意：RXNE 标志位通过读 DR 寄存器已自动清除
	}
}
