#include "hr_spo2.h"
#include "max30102.h"
#include "algorithm.h"
#include "Delay.h"
#include <string.h>
#include <stdlib.h>
#include "Serial.h"

/* ==================== 宏定义 ==================== */
#define HR_BUFFER_SIZE        250
#define IR_FINGER_THRESHOLD   50000

/* ==================== 数据缓存 ==================== */
static uint32_t ir_buf[HR_BUFFER_SIZE];
static uint32_t red_buf[HR_BUFFER_SIZE];

static uint16_t hr_index = 0;
static uint8_t hr_full = 0;

/* 算法输出 */
static int32_t n_heart_rate = 0;
static int32_t n_spo2 = 0;
static int8_t ch_hr_valid = 0;
static int8_t ch_spo2_valid = 0;

/* 稳定显示值 */
static int32_t disp_hr = 0;
static int32_t disp_spo2 = 0;

/* 手指检测 */
static uint8_t finger_present = 0;
static uint8_t finger_cnt = 0;


/* ==================== 私有函数 ==================== */
static void sample_once(void)
{
    uint32_t red, ir;

    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) == 0)
    {
        maxim_max30102_read_fifo(&red, &ir);

        /* 手指检测 */
        if (ir > IR_FINGER_THRESHOLD)
        {
            if (finger_cnt < 5) finger_cnt++;
        }
        else
        {
            finger_cnt = 0;
        }

        finger_present = (finger_cnt >= 3);

        if (finger_present)
        {
            red_buf[hr_index] = red;
            ir_buf[hr_index]  = ir;

            hr_index++;
            if (hr_index >= HR_BUFFER_SIZE)
            {
                hr_index = 0;
                hr_full = 1;
            }
        }
        else
        {
            hr_index = 0;
            hr_full  = 0;
            disp_hr = 0;
            disp_spo2 = 0;
            ch_hr_valid = 0;
            ch_spo2_valid = 0;
        }
    }
}

static void calculate(void)
{
    static uint32_t ir_copy[HR_BUFFER_SIZE];
    static uint32_t red_copy[HR_BUFFER_SIZE];

    __disable_irq();
    memcpy(ir_copy,  ir_buf,  sizeof(ir_copy));
    memcpy(red_copy, red_buf, sizeof(red_copy));
    __enable_irq();

    maxim_heart_rate_and_oxygen_saturation(
        ir_copy,
        HR_BUFFER_SIZE,
        red_copy,
        &n_spo2,
        &ch_spo2_valid,
        &n_heart_rate,
        &ch_hr_valid
    );

    if (hr_full && ch_hr_valid)
    {
        if (n_heart_rate > 45 && n_heart_rate < 160)
        {
            if (disp_hr == 0 || abs(n_heart_rate - disp_hr) < 20)
                disp_hr = n_heart_rate;
        }
    }

    if (hr_full && ch_spo2_valid)
    {
        if (n_spo2 >= 90 && n_spo2 <= 100)
            disp_spo2 = n_spo2;
    }
}


/* ==================== 对外接口 ==================== */

void HRSpO2_Init(void)
{
    maxim_max30102_reset();
    Delay_ms(50);
    maxim_max30102_init();
}

void HRSpO2_10msHandler(void)
{
    static uint8_t div = 0;

    div++;
    if (div >= 2)
    {
        sample_once();
        div = 0;
    }
}

void HRSpO2_1sHandler(void)
{
    if (finger_present && hr_full)
        calculate();
}

uint8_t HRSpO2_IsFingerPresent(void)
{
    return finger_present;
}

uint8_t HRSpO2_IsReady(void)
{
    return hr_full;
}

int32_t HRSpO2_GetHR(void)
{
    return disp_hr;
}

int32_t HRSpO2_GetSpO2(void)
{
    return disp_spo2;
}

/* 心率血氧播报任务 */
void Hr_O2_Speak(uint8_t rx)
{

	if(!rx) return;

    if(!HRSpO2_IsReady()) return;

    if (Serial_RxPacket[0] == 4)
        Serial_SendNumber(disp_hr,3);
	
    else if (Serial_RxPacket[0] == 5)
        Serial_SendNumber(disp_spo2,3);
	
}
