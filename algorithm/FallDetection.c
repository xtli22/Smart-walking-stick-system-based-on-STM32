#include "stm32f10x.h"                  // Device header
#include "MPU6050.h"
#include "math.h"

/* ================= 参数阈值 ================= */
/* 加速度阈值（单位：g） */
#define FREE_FALL_TH      0.7f    // 失重阈值
#define IMPACT_TH         2.0f    // 碰撞阈值

/* 角速度阈值（单位：°/s） */
#define STILL_GYRO_TH     10.0f   // 静止判定角速度

/* 姿态变化阈值（单位：°） */
#define ANGLE_CHANGE_TH  60.0f   // 拐杖由竖直到倒地的姿态变化
#define RECOVER_GYRO_TH       50.0f   // 事后大动作角速度阈值（°/s）

/* 时间阈值（单位：ms） */
#define STILL_TIME_TH    1500     // 碰撞后静止持续时间
#define LONG_STILL_TIME_TH     10000   // 倒地后持续静止时间（ms）

typedef enum				 //跌倒状态机
{
    FALL_STATE_NORMAL = 0,   // 正常使用
    FALL_STATE_FREE,         // 失重阶段
    FALL_STATE_IMPACT,       // 碰撞阶段
    FALL_STATE_STILL,        // 静止确认阶段
	 FALL_STATE_CONFIRM    	 // 长时间确认阶段
} FallState_t;

//内部变量
static FallState_t fall_state;

static float pitch_start;    // 失重开始时的姿态
static float roll_start;

static uint32_t still_time;  // 静止计时
static uint32_t confirm_time;	//长时间静止确认


void FallDetection_Init(void)
{
	MPU6050_Init();
    fall_state = FALL_STATE_NORMAL;
    still_time = 0;
	confirm_time = 0;
}

void FallDetection_Recover(void)
{
    fall_state = FALL_STATE_NORMAL;
    still_time = 0;
	confirm_time = 0;
}

/* =========================================================
 * 函数名：FallDetection_Update
 * 功能  ：拐杖跌倒检测核心逻辑
 * 说明  ：10ms~20ms 调用一次
 * 返回值：1 - 检测到跌倒
 *         0 - 正常
 * ========================================================= */
uint8_t FallDetection_Update(uint32_t dt)
{
    float acc_total  = MPU6050_GetAccTotal();   // 合加速度
    float gyro_total = MPU6050_GetGyroTotal();  // 合角速度
    float pitch      = MPU6050_GetPitch();      // 滤波后 Pitch
    float roll       = MPU6050_GetRoll();       // 滤波后 Roll

    switch(fall_state)
    {
        case FALL_STATE_NORMAL:			//正常状态
            if(acc_total < FREE_FALL_TH)
            {
                //拐杖出现明显失重，可能开始跌倒
                pitch_start = pitch;
                roll_start  = roll;
                fall_state  = FALL_STATE_FREE;
				
				
            }
            break;

        case FALL_STATE_FREE:		//失重阶段
            if(acc_total > IMPACT_TH)
            {
                //与地面发生明显碰撞
                still_time = 0;
                fall_state = FALL_STATE_IMPACT;
				
				
            }
            else if((acc_total > 0.9f)&(acc_total < IMPACT_TH))
            {
                //失重后快速恢复，判定为滑动或误触
                fall_state = FALL_STATE_NORMAL;
				
            }
            break;

        case FALL_STATE_IMPACT: 	//碰撞阶段
            if(gyro_total < STILL_GYRO_TH)
            {
                //拐杖逐渐进入静止状态
                still_time += dt;
                if(still_time >= STILL_TIME_TH)
                {
                    fall_state = FALL_STATE_STILL;
					
                }
            }
            else
            {
                //仍在运动，重新计时
                still_time = 0;
            }
            break;

        case FALL_STATE_STILL:	// 静止 + 姿态确认
        {
            float pitch_diff = fabs(pitch - pitch_start);
            float roll_diff  = fabs(roll  - roll_start);

            if(pitch_diff > ANGLE_CHANGE_TH || roll_diff > ANGLE_CHANGE_TH)//与初始拐杖竖直状态差别过大
            {
				confirm_time = 0; 	//姿态已明显改变，进入长时间确认阶段
				fall_state = FALL_STATE_CONFIRM;
				
            }
            else
            {
                fall_state = FALL_STATE_NORMAL; 	//拐杖滑倒，但人未倒
            }
            break;
        }
		
		case FALL_STATE_CONFIRM:	//长时间确认阶段
			if(gyro_total > RECOVER_GYRO_TH)	//如果出现明显运动，认为使用者已恢复或误触
			{
				fall_state = FALL_STATE_NORMAL;
				
			}
			else
			{
				confirm_time += dt;
				if(confirm_time >= LONG_STILL_TIME_TH)
				{
					fall_state = FALL_STATE_NORMAL;	//长时间静止 + 姿态异常，确认跌倒
					return 1;
				}
			}
			break;

        default:
            fall_state = FALL_STATE_NORMAL;
            break;
    }

    return 0;
}
