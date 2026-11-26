#include "stm32f10x.h"                  // Device header                   
#include "MyI2C.h"
#include "MPU6050_Reg.h"                
#include "Kalman.h"                     
#include "math.h"                       

#define MPU6050_ADDRESS  0xD0			//MPU6050 I2C 从机地址

// 加速度计、陀螺仪原始数据（16 位有符号） 
static int16_t AX, AY, AZ;               // 加速度原始数据
static int16_t GX, GY, GZ;               // 陀螺仪原始数据

// 转换后的物理量
static float acc_x, acc_y, acc_z;        // 加速度（单位：g）
static float gyro_x, gyro_y, gyro_z;     // 角速度（单位：°/s）

static float acc_total;                  // 合加速度（单位：g）
static float gyro_total;                 // 合角速度（单位：m/s^2）
static float pitch, roll;                // 姿态角（单位：°）

static float acc_pitch;   				 // 未滤波 Pitch（加速度解算）
static float acc_roll;   				 // 未滤波 Roll

//卡尔曼滤波器结构体实例
static Kalman_t Kalman_Pitch;            // Pitch 角卡尔曼滤波器
static Kalman_t Kalman_Roll;             // Roll 角卡尔曼滤波器


/* =====================================================
 * 函    数：MPU6050_WriteReg
 * 功    能：向 MPU6050 指定寄存器写入 1 字节数据
 * 参    数：RegAddress - 寄存器地址
 *           Data       - 写入的数据
 * 返 回 值：无
 * ===================================================== */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{	
	MyI2C_Start();                        // I2C 起始信号
    MyI2C_SendByte(MPU6050_ADDRESS);      // 发送从机地址（写）
    MyI2C_ReceiveAck();                   // 等待应答
    MyI2C_SendByte(RegAddress);           // 发送寄存器地址
    MyI2C_ReceiveAck();
    MyI2C_SendByte(Data);                 // 发送数据
    MyI2C_ReceiveAck();
    MyI2C_Stop();                         // I2C 停止信号
	
}

/* =====================================================
 * 函    数：MPU6050_ReadReg
 * 功    能：读取 MPU6050 指定寄存器的数据
 * 参    数：RegAddress - 寄存器地址
 * 返 回 值：寄存器数据
 * ===================================================== */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;
	
	MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);      // 写模式
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();

    MyI2C_Start();                        // 重复起始
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01);// 读模式
    MyI2C_ReceiveAck();
    Data = MyI2C_ReceiveByte();           // 读取数据
    MyI2C_SendAck(1);                     // NACK
    MyI2C_Stop();
	
	return Data;
	
}

/* =====================================================
 * 函    数：MPU6050_Init
 * 功    能：MPU6050 初始化配置
 * 返 回 值：无
 * ===================================================== */
void MPU6050_Init(void)
{
    MyI2C_Init();                         // 初始化软件 I2C
	
    /* ----------- MPU6050 基本配置 ----------- */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);   // 退出睡眠，使用 X 轴陀螺仪时钟
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);   // 所有轴正常工作
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);  // 采样率 100Hz
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);      // DLPF = 6，降低高频噪声
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00); // ±250°/s
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);// ±2g

    /* ----------- 卡尔曼滤波器初始化 ----------- */
    Kalman_Init(&Kalman_Pitch);
    Kalman_Init(&Kalman_Roll);
}

/* =====================================================
 * 函    数：MPU6050_GetRawData
 * 功    能：连续读取 MPU6050 加速度与陀螺仪原始数据
 * 说    明：使用连续读方式，提高 I2C 读取效率
 * ===================================================== */
static void MPU6050_GetRawData(void)
{
    uint8_t buf[14];
    uint8_t i;
	
	MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(MPU6050_ACCEL_XOUT_H); // 从加速度 X 高字节开始读取
    MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
    MyI2C_ReceiveAck();

    for(i = 0; i < 13; i++)
    {
        buf[i] = MyI2C_ReceiveByte();
        MyI2C_SendAck(0);                 // ACK
    }
    buf[13] = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);                     // NACK
    MyI2C_Stop();
	

    //16 位原始数据 
    AX = (buf[0] << 8) | buf[1];
    AY = (buf[2] << 8) | buf[3];
    AZ = (buf[4] << 8) | buf[5];

    GX = (buf[8] << 8) | buf[9];
    GY = (buf[10] << 8) | buf[11];
    GZ = (buf[12] << 8) | buf[13];
}

/* =====================================================
 * 函    数：MPU6050_Update
 * 功    能：更新 MPU6050 数据并计算姿态角
 * 参    数：dt - 采样周期（单位：秒）
 * 说    明：建议在 10ms 定时中断中调用
 * ===================================================== */
void MPU6050_Update(float dt)
{
    
    MPU6050_GetRawData();		//读取原始数据 

    acc_x  = AX / 16384.0f;		//原始数据转换为物理量
    acc_y  = AY / 16384.0f;
    acc_z  = AZ / 16384.0f;

    gyro_x = GX / 131.0f;
    gyro_y = GY / 131.0f;
    gyro_z = GZ / 131.0f;

    acc_total = sqrt(acc_x * acc_x + 		//计算合加速度
						acc_y * acc_y +
						acc_z * acc_z);
	
	gyro_total = fabs(gyro_x) + fabs(gyro_y) + fabs(gyro_z); //计算合角速度

							
    acc_pitch = atan2(acc_x,		//由加速度计算姿态角
                      sqrt(acc_y * acc_y + acc_z * acc_z)) * 57.3f;

    acc_roll  = atan2(acc_y,
                      sqrt(acc_x * acc_x + acc_z * acc_z)) * 57.3f;

    //卡尔曼滤波融合加速度与陀螺仪
    pitch = Kalman_Update(&Kalman_Pitch, acc_pitch, gyro_y, dt);
    roll  = Kalman_Update(&Kalman_Roll,  acc_roll,  gyro_x, dt);
}

float MPU6050_GetAccTotal(void)		//获取当前合加速度
{
    return acc_total;
}

float MPU6050_GetGyroTotal(void)	//获取当前合角速度
{
    return gyro_total;
}

float MPU6050_GetPitch(void)	//滤波后的 Pitch 角
{
    return pitch;
}

float MPU6050_GetRoll(void)		//滤波后的 Roll 角
{
    return roll;
}
