#include "stm32f10x.h"                  // Device header
#include "usart.h"
#include <stdlib.h>
#include <string.h>

void errorLog(int num)
{
	while (1)
	{
	  	printf("ERROR%d\r\n",num);
	}
}

// 内部辅助函数：跳过逗号
char* Get_Nmea_Field(char *str, uint8_t field_idx) {
    char *p = str;
    while (field_idx--) {
        p = strchr(p, ',');
        if (!p) return NULL;
        p++;
    }
    return p;
}

void parseGpsBuffer(void)
{
    // 1. 检查是否有新行到达
    if (!Save_Data.isGetData) return;

    // 2. 局部缓冲区快照（防止解析时中断修改数据导致字符串断裂）
    char tempBuffer[USART_REC_LEN];
    __disable_irq(); // 关中断，拷贝数据
    memcpy(tempBuffer, USART_RX_BUF, USART_REC_LEN);
    Save_Data.isGetData = false; 
    point1 = 0;      // 处理完后重置索引，准备接收下一行
    __enable_irq();  // 开中断

    // 3. 寻找 RMC 语句 (推荐：RMC 包含经纬度、速度和有效性判定)
    char *p_rmc = strstr(tempBuffer, "RMC");
    if (p_rmc == NULL) return;

    char *field;
    
    // 字段2: 定位状态 (A=有效, V=无效)
    field = Get_Nmea_Field(p_rmc, 2);
    if (field && *field == 'A') {
        Save_Data.isUsefull = true;
    } else {
        Save_Data.isUsefull = false;
        return; // 若无效则不再解析经纬度
    }

    // 字段3 & 4: 纬度
    field = Get_Nmea_Field(p_rmc, 3);
    if (field) {
        strncpy(Save_Data.latitude, field, 10);
        Save_Data.latitude[10] = '\0';
    }
    field = Get_Nmea_Field(p_rmc, 4);
    if (field) Save_Data.N_S[0] = *field;

    // 字段5 & 6: 经度
    field = Get_Nmea_Field(p_rmc, 5);
    if (field) {
        strncpy(Save_Data.longitude, field, 11);
        Save_Data.longitude[11] = '\0';
    }
    field = Get_Nmea_Field(p_rmc, 6);
    if (field) Save_Data.E_W[0] = *field;

    // 字段1: UTC 时间 (hhmmss.ss)
    field = Get_Nmea_Field(p_rmc, 1);
    if (field) {
        strncpy(Save_Data.UTCTime, field, 6); // 只取前6位时分秒
        Save_Data.UTCTime[6] = '\0';
    }

    Save_Data.isParseData = true; // 标记解析成功，可用于打印或发送
}


void printGpsBuffer()
{
	if (!Save_Data.isParseData) return;

    Save_Data.isParseData = false;

    printf("UTC:%s\r\n", Save_Data.UTCTime);

    if (Save_Data.isUsefull)
    {
        printf("Lat:%s NS:%s\r\n", Save_Data.latitude, Save_Data.N_S);
        printf("Lon:%s EW:%s\r\n", Save_Data.longitude, Save_Data.E_W);
    }
    else
    {
        printf("GPS invalid\r\n");
    }
	
}

// 将 GPS 原始字符串格式 ddmm.mmmm 转换为十进制度数 dd.dddddd
double ConvertToDecimalDegrees(const char* ddmm) {
    if (ddmm == NULL || strlen(ddmm) < 4) return 0.0;
    
    char *dot_ptr = strchr(ddmm, '.');
    if (dot_ptr == NULL) return 0.0;
    
    int dot_index = dot_ptr - ddmm;
    
    // 1. 提取“分”：从点前面两位开始提取 (例如 14.5000)
    double minutes = atof(dot_ptr - 2);
    
    // 2. 提取“度”：点前面两位之前的所有内容
    // 我们直接创建一个临时缓冲区来处理“度”的部分
    char degree_part[8] = {0};
    int deg_len = dot_index - 2; // 度部分的长度
    if (deg_len > 0 && deg_len < 7) {
        strncpy(degree_part, ddmm, deg_len);
        degree_part[deg_len] = '\0';
    }
    double degrees = atof(degree_part);
    
    // 3. 计算结果：度 + 分/60
    return degrees + (minutes / 60.0);
}
