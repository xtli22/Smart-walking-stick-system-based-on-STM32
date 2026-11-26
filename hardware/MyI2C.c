#include "stm32f10x.h"                  // Device header
#include "Delay.h"

#define SCL GPIO_Pin_0
#define SDA GPIO_Pin_1
#define MyI2C_GPIO GPIOB

/*引脚配置层*/

/**
  * 函    数：I2C写SCL引脚电平
  * 参    数：BitValue 协议层传入的当前需要写入SCL的电平，范围0~1
  * 返 回 值：无
  * 注意事项：此函数需要用户实现内容，当BitValue为0时，需要置SCL为低电平，当BitValue为1时，需要置SCL为高电平
  */
void MyI2C_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(MyI2C_GPIO, SCL, (BitAction)BitValue);		//根据BitValue，设置SCL引脚的电平
	Delay_us(10);												//延时10us，防止时序频率超过要求
}

/**
  * 函    数：I2C写SDA引脚电平
  * 参    数：BitValue 协议层传入的当前需要写入SDA的电平，范围0~1
  * 返 回 值：无
  * 注意事项：此函数需要用户实现内容，当BitValue为0时，需要置SDA为低电平，当BitValue为1时，需要置SDA为高电平
  */
void MyI2C_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(MyI2C_GPIO, SDA, (BitAction)BitValue);		//根据BitValue，设置SDA引脚的电平，BitValue要实现非0即1的特性
	Delay_us(10);												//延时10us，防止时序频率超过要求
}

/**
  * 函    数：I2C读SDA引脚电平
  * 参    数：无
  * 返 回 值：协议层需要得到的当前SDA的电平，范围0~1
  * 注意事项：此函数需要用户实现内容，当前SDA为低电平时，返回0，当前SDA为高电平时，返回1
  */
uint8_t MyI2C_R_SDA(void)
{
	uint8_t BitValue;
	BitValue = GPIO_ReadInputDataBit(MyI2C_GPIO, SDA);		//读取SDA电平
	Delay_us(10);												//延时10us，防止时序频率超过要求
	return BitValue;											//返回SDA电平
}

/**
  * 函    数：I2C初始化
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数需要用户实现内容，实现SCL和SDA引脚的初始化
  */
void MyI2C_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//开启GPIOB的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin = SCL | SDA;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MyI2C_GPIO, &GPIO_InitStructure);					//将PB10和PB11引脚初始化为开漏输出
	
	/*设置默认电平*/
	GPIO_SetBits(MyI2C_GPIO, SCL | SDA);			//设置PB10和PB11引脚初始化后默认为高电平（释放总线状态）
}

/*协议层*/

/**
  * 函    数：I2C起始
  * 参    数：无
  * 返 回 值：无
  */
void MyI2C_Start(void)
{
	MyI2C_W_SDA(1);							//释放SDA，确保SDA为高电平
	MyI2C_W_SCL(1);							//释放SCL，确保SCL为高电平
	MyI2C_W_SDA(0);							//在SCL高电平期间，拉低SDA，产生起始信号
	MyI2C_W_SCL(0);							//起始后把SCL也拉低，即为了占用总线，也为了方便总线时序的拼接
}

/**
  * 函    数：I2C终止
  * 参    数：无
  * 返 回 值：无
  */
void MyI2C_Stop(void)
{
	MyI2C_W_SDA(0);							//拉低SDA，确保SDA为低电平
	MyI2C_W_SCL(1);							//释放SCL，使SCL呈现高电平
	MyI2C_W_SDA(1);							//在SCL高电平期间，释放SDA，产生终止信号
}

/**
  * 函    数：I2C发送一个字节
  * 参    数：Byte 要发送的一个字节数据，范围：0x00~0xFF
  * 返 回 值：无
  */
void MyI2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i ++)				//循环8次，主机依次发送数据的每一位
	{
		/*两个!可以对数据进行两次逻辑取反，作用是把非0值统一转换为1，即：!!(0) = 0，!!(非0) = 1*/
		MyI2C_W_SDA(!!(Byte & (0x80 >> i)));//使用掩码的方式取出Byte的指定一位数据并写入到SDA线
		MyI2C_W_SCL(1);						//释放SCL，从机在SCL高电平期间读取SDA
		MyI2C_W_SCL(0);						//拉低SCL，主机开始发送下一位数据
	}
}

/**
  * 函    数：I2C接收一个字节
  * 参    数：无
  * 返 回 值：接收到的一个字节数据，范围：0x00~0xFF
  */
uint8_t MyI2C_ReceiveByte(void)
{
    uint8_t i, Byte = 0x00;
    uint32_t Timeout; // 定义超时计数器

    MyI2C_W_SDA(1); // 接收前释放SDA
    
    for (i = 0; i < 8; i++)
    {
        MyI2C_W_SCL(1); // 释放SCL
        
        // --- 增加超时逻辑 ---
        // 在实际应用中，如果从机支持 Clock Stretching，这里会有一个 while(Read_SCL == 0);
        // 如果没有时钟拉伸，这里仅作为防御性代码
        Timeout = 10000; // 设置超时初值（根据 CPU 频率调整）
        while (/* 这里可以放入检测条件，例如检测 SCL 是否真的变高了 */ 0) 
        {
            Timeout--;
            if (Timeout == 0) return 0xFF; // 超时跳出，返回错误标志
        }
        
        // 读取数据
        if (MyI2C_R_SDA())
        {
            Byte |= (0x80 >> i);
        }
        
        MyI2C_W_SCL(0); // 拉低SCL
    }
    
    return Byte;
}


/**
  * 函    数：I2C发送应答位
  * 参    数：Byte 要发送的应答位，范围：0~1，0表示应答，1表示非应答
  * 返 回 值：无
  */
void MyI2C_SendAck(uint8_t AckBit)
{
	MyI2C_W_SDA(AckBit);					//主机把应答位数据放到SDA线
	MyI2C_W_SCL(1);							//释放SCL，从机在SCL高电平期间，读取应答位
	MyI2C_W_SCL(0);							//拉低SCL，开始下一个时序模块
}


/**
  * 函    数：I2C接收应答位（带超时功能）
  * 参    数：无
  * 返 回 值：接收到的应答位，0为应答，1为非应答，0xFF为总线异常/超时
  */
uint8_t MyI2C_ReceiveAck(void)
{
    uint8_t AckBit;
    uint32_t Timeout = 10000; // 超时阈值，根据 CPU 主频调整

    MyI2C_W_SDA(1); // 释放SDA，准备接收
    MyI2C_W_SCL(1); // 释放SCL

    // 防御性逻辑：等待 SCL 真正变高（防止从机拉低 SCL 进行时钟拉伸）
    // 注意：如果你的硬件不支持读取 SCL 引脚，可以删除此段
    while (/* 如果能读取SCL引脚状态: MyI2C_R_SCL() == 0 */ 0)
    {
        Timeout--;
        if (Timeout == 0) return 0xFF; // SCL被死锁，返回错误标志
    }

    // 在 SCL 高电平期间读取 SDA
    AckBit = MyI2C_R_SDA(); 

    MyI2C_W_SCL(0); // 拉低SCL，结束 ACK 时序
    
    return AckBit;
}
