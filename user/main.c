#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "MyI2C.h"
#include "MPU6050.h"
#include "FallDetection.h"
#include "Timer.h"
#include "Buzzer.h"
#include "Key.h"
#include "LightSensor.h"
#include "LED.h"
#include "hr_spo2.h"
#include <stdlib.h>
#include "usart.h"
#include "GPS.h"
#include "Serial.h"
#include "usart2.h"
#include <string.h>
#include "Air780E.h"

char number[] = "+8619211110111";

/* 定时器标志 */
volatile uint8_t flag_10ms = 0;
volatile uint32_t tick_10ms = 0;

static uint32_t last_upload_tick = 0;

uint8_t sms_lock = 0;

uint8_t KeyNum;
int32_t hr,spo2;

uint8_t rx;

volatile uint8_t sms_request = 0;
volatile uint8_t sms_safe = 0;


/* ==================== OLED任务 ==================== */
static void OLED_Task(void)
{
    static uint8_t div = 0;
    if (++div < 30) return; 
    div = 0;

    // 第一行：心率
    OLED_ShowCH(1, 1, 0); // 心
    OLED_ShowCH(1, 3, 1); // 率
    OLED_ShowChar(1, 5, ':');
    if (!HRSpO2_IsFingerPresent()) {
        OLED_ShowString(1, 6, "--- ");
    } else {
        hr = HRSpO2_GetHR();
        OLED_ShowNum(1, 6, hr, 3);
    }
    OLED_ShowString(1, 10, "bpm");

    // 第二行：血氧
    OLED_ShowCH(2, 1, 2); // 血
    OLED_ShowCH(2, 3, 3); // 氧
    OLED_ShowChar(2, 5, ':');
    if (!HRSpO2_IsFingerPresent()) {
        OLED_ShowString(2, 6, "--- ");
    } else {
        spo2 = HRSpO2_GetSpO2();
        OLED_ShowNum(2, 6, (uint32_t)spo2, 3);
    }
    OLED_ShowChar(2, 10, '%');

    // 第三行：状态显示
	OLED_ShowCH(3, 1, 10); // 跌 
	OLED_ShowCH(3, 3, 11); // 倒
	OLED_ShowChar(3, 5, ':');
	if (sms_lock == 1) {
		OLED_ShowCH(3, 6, 6); // 跌 
		OLED_ShowCH(3, 8, 7); // 倒 
	} else {
		OLED_ShowCH(3, 6, 4); // 安
		OLED_ShowCH(3, 8, 5); // 全
	}
}

/* ==================== 主函数 ==================== */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    MyI2C_Init();
    OLED_Init();
    MPU6050_Init();
    Timer_Init();
    FallDetection_Init();
    Buzzer_Init();
    Key_Init();
    LightSensor_Init();
    LED_Init();
    HRSpO2_Init();
	Serial_Init();
	Air780E_Init();

    OLED_ShowCH(2, 6, 8); // 开
	OLED_ShowCH(2, 8, 9); // 机
    Delay_ms(800);
	
	uart_init(9600);	 //串口初始化为9600
	
	clrStruct();
	
	// 发送握手信号
	usart2_Printf("AT\r\n");
	
	while(MQTT_Connect());		//连接服务器
	
	OLED_Clear();
	
    while (1)
    {
		KeyNum=Key_GetNum();
		
		parseGpsBuffer();
		
        if (flag_10ms)
        {
            flag_10ms = 0;

            MPU6050_Update(0.01f);
            HRSpO2_10msHandler();
			OLED_Task();
			
			//触发警报
			 if ((FallDetection_Update(10)||KeyNum==1)&& sms_lock == 0)     	// 10 ms
            {
				sms_lock = 1;
				Buzzer_ON();					//蜂鸣器打开
				sms_request = 1;		//发送一条包含求救信息的短信
            }
			//安全确认
			if (KeyNum==2 && sms_lock==1)
			{
				sms_lock = 0;
				Buzzer_OFF();
				sms_safe=1;				//发送安全的短信
			}
			// 只有在 GPS 解析成功且需要打印时才操作
			if (Save_Data.isParseData) {
				printGpsBuffer(); 
				Save_Data.isParseData = false;
			}
			
        }
        if (tick_10ms % 100 == 0)
		{
            HRSpO2_1sHandler();
		}
		
		// 短信只在需要时发送一次，发完立刻清零标志位
		if(sms_request)				//发送短信
		{
			Send_Fall_Location_SMS();
			sms_request = 0;

		}
		if(sms_safe)				//发送短信
		{
			Send_Safe_SMS();
			sms_safe = 0;
		}
		
		//MQTT上传
		if(tick_10ms - last_upload_tick >= 3000)
		{	
			last_upload_tick = tick_10ms;
			
			Air780E_MQTT_GPS_Upload();
			Air780E_MQTT_Upload();   // 30s上传一次
			
		}
		
		//串口指令处理(语音播报)
		rx = Serial_GetRxFlag();
		
		LED_Task(rx);				//led_cmd
		Hr_O2_Speak(rx);			//hr_O2播放
		Voice_Send(rx);				//语音发送短信
		
	}
}


/* ==================== 定时器中断 ==================== */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update))
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        flag_10ms = 1;
        tick_10ms++;
    }
}
