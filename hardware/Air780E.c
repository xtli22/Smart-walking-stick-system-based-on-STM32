#include "stm32f10x.h"                  // Device header
#include "usart2.h"
#include "Delay.h"
#include <string.h>
#include "usart.h"
#include "GPS.h"
#include <stdlib.h>
#include "Serial.h"

char final_hex_msg[512]; // 增加缓冲区，防止溢出
char tmp_hex[256]; 

extern uint8_t sms_lock;
extern int32_t hr,spo2;
extern char number[];

double dec_lon,dec_lat;

void Air780E_Init(void)
{
	usart2_Init();
}

uint8_t WaitReply(char *ok, char *err, uint16_t timeout)
{
    uint16_t t = 0;

    while(t < timeout)
    {
        if(strstr(USART2_RxPacket, ok)) return 1;
        if(err && strstr(USART2_RxPacket, err)) return 0;

        Delay_ms(10);
        t += 10;
    }
    return 0;
}

/**
  * 函  数：发送报警短信
  * 参  数：PhoneNum 电话号码，Message 内容
  */
uint8_t Air780E_SendSMS(char *PhoneNum, char *Message)
{
    usart2_ClearPacket();

    /* 文本模式 */
    usart2_ClearPacket();
    usart2_Printf("AT+CMGF=1\r\n");
    if(!WaitReply("OK","ERROR",300)) return 0;
	
	/* 文本模式 */
    usart2_ClearPacket();
    usart2_Printf("AT+CSMP=17,167,0,8\r\n");
    if(!WaitReply("OK","ERROR",300)) return 0;

    /* 输入号码 */
    usart2_ClearPacket();
    usart2_Printf("AT+CMGS=\"%s\"\r\n", PhoneNum);
    if(!WaitReply(">","ERROR",600)) return 0;

    /* 输入正文 */
    usart2_ClearPacket();
    usart2_SendHexString(Message);
	
    usart2_SendByte(0x1A);

    /* 等待发送结果 */
    if(!WaitReply("+CMGS","ERROR",800)) return 0;
    if(!WaitReply("OK","ERROR",600)) return 0;

    return 1;
}


/**
 * @brief 将 int 数值转换为短信所需的 UCS2 十六进制字符串
 * @param val: 输入的整数 (如 98)
 * @param dest: 接收转换结果的缓冲区 (建议大小至少 24 字节)
 */
void IntToUcs2Hex(int val, char* dest)
{
    char str[12];
    sprintf(str, "%d", val); // 1. 先将 int 转为普通字符串 "98"
    
    char *ptr = str;
    while (*ptr) {
        // 2. 将每个字符转为 4 位十六进制 (如 '9' -> "0039")
        sprintf(dest, "00%02X", (unsigned char)*ptr);
        ptr++;
        dest += 4; // 移动到下一个存放位置
    }
    *dest = '\0'; // 添加结束符
}

// 将 ASCII 字符串转换为 UCS2 16进制字符串
void StrToUcs2Hex(char* src, char* dest) 
{
    while (*src) {
        sprintf(dest, "00%02X", (unsigned char)*src);
        src += 1;
        dest += 4;
    }
    *dest = '\0';
}


/* ==================== 短信 ==================== */
void Send_Fall_Location_SMS(void) {
	
    static char str_coords[32];
    static char lbs_lon[32] = {0}, lbs_lat[32] = {0}; // 统一存储变量
    int code;
    
    // 初始化清空
	memset(lbs_lon, 0, sizeof(lbs_lon));
	memset(lbs_lat, 0, sizeof(lbs_lat));
    memset(final_hex_msg, 0, sizeof(final_hex_msg));
    
    // A. 抬头：【跌倒警报】 (UCS2: 30108DCC50128B6662A53011)
    strcpy(final_hex_msg, "30108DCC50128B6662A53011");
    strcat(final_hex_msg, "000A");
	
    // B. 拼接心率：心率：
    strcat(final_hex_msg, "5FC37387FF1A"); 
    IntToUcs2Hex((int)hr, tmp_hex); 
    strcat(final_hex_msg, tmp_hex);
    strcat(final_hex_msg, "000A");
    
    // C. 拼接血氧：血氧：
    strcat(final_hex_msg, "88406C27FF1A"); 
    IntToUcs2Hex((int)spo2, tmp_hex);
    strcat(final_hex_msg, tmp_hex);
    strcat(final_hex_msg, "0025000A");
    

    if (Save_Data.isUsefull) {	
   // D. 拼接位置抬头：位置：
		strcat(final_hex_msg, "4F4D7F6EFF1A");
		
        // --- GPS 定位有效 ---
        dec_lon = ConvertToDecimalDegrees(Save_Data.longitude);
        dec_lat = ConvertToDecimalDegrees(Save_Data.latitude);
        
        sprintf(str_coords, "%.7f", dec_lon);
        StrToUcs2Hex(str_coords, tmp_hex);
        strcat(final_hex_msg, tmp_hex);
        
        strcat(final_hex_msg, "002C"); // ,
        
        sprintf(str_coords, "%.7f", dec_lat);
        StrToUcs2Hex(str_coords, tmp_hex);
        strcat(final_hex_msg, tmp_hex);
        
    }
    else {
        // --- GPS失效，尝试基站定位 (LBS) ---
        usart2_ClearPacket();
        usart2_Printf("AT+CIPGSMLOC=1,1\r\n");
		
        if(WaitReply("+CIPGSMLOC:", "ERROR", 5000)){
            char *ptr = strstr(USART2_RxPacket, "+CIPGSMLOC:");
			
			
            if(ptr) {
                // 解析格式：code,纬度,经度,日期,时间
				
                // 手动解析：跳过 "+CIPGSMLOC: " 这 11 个字符
                char *p = ptr + 11; 
                
                // 1. 获取 code
                code = atoi(p); 
                
                // 2. 找到第一个逗号，后面是 Latitude
                p = strchr(p, ',');
                if(p && code == 0) {
                    p++; // 跳过逗号
                    char *next_comma = strchr(p, ',');
                    if(next_comma) {
                        int len = next_comma - p;
                        if(len < 20) {
                            memcpy(lbs_lat, p, len);
                            lbs_lat[len] = '\0'; // 提取出纬度
                        }
                        
                        // 3. 找到第二个逗号，后面是 Longitude
                        p = next_comma + 1;
                        next_comma = strchr(p, ',');
                        if(next_comma) {
                            len = next_comma - p;
                            if(len < 20) {
                                memcpy(lbs_lon, p, len);
                                lbs_lon[len] = '\0'; // 提取出经度
                            }

                            // --- 执行拼接逻辑 ---
                            strcat(final_hex_msg, "4FE153F75F31FF0C57FA7AD95B9A4F4DFF1A"); // 基站定位抬头
                            strcat(final_hex_msg, "000A");
                            
                            StrToUcs2Hex(lbs_lat, tmp_hex); 
                            strcat(final_hex_msg, tmp_hex);
                            strcat(final_hex_msg, "002C"); // ,
                            StrToUcs2Hex(lbs_lon, tmp_hex); 
                            strcat(final_hex_msg, tmp_hex);
                        }
                    }
                } else if (code != 0) {
                    strcat(final_hex_msg, "65E06CD583B7"); // 定位失败
                }
            }
        }
    }   

    // --- 最后统一发送 ---
    Delay_ms(200);
    usart2_ClearPacket(); 
    Air780E_SendSMS(number, final_hex_msg);
	
}

/* ================ 安全短信 ================= */
void Send_Safe_SMS(void)
{
	
    static char str_coords[20];
    static char lbs_lon[32] = {0}, lbs_lat[32] = {0}; // 统一存储变量
    int code;
    
    // 初始化清空
	memset(lbs_lon, 0, sizeof(lbs_lon));
	memset(lbs_lat, 0, sizeof(lbs_lat));
    memset(final_hex_msg, 0, sizeof(final_hex_msg));
    
    // 【安全通知】
    strcpy(final_hex_msg, "30105B895168901A77E53011");
    strcat(final_hex_msg, "000A");

    // 状态：已确认安全
    strcat(final_hex_msg, "72B66001FF1A"); 
    strcat(final_hex_msg, "5DF2786E8BA45B895168");
    strcat(final_hex_msg, "000A");
    

    if (Save_Data.isUsefull) {	
   // D. 拼接位置抬头：位置：
		strcat(final_hex_msg, "4F4D7F6EFF1A");
		
        // --- GPS 定位有效 ---
        dec_lon = ConvertToDecimalDegrees(Save_Data.longitude);
        dec_lat = ConvertToDecimalDegrees(Save_Data.latitude);
        
        sprintf(str_coords, "%.7f", dec_lon);
        StrToUcs2Hex(str_coords, tmp_hex);
        strcat(final_hex_msg, tmp_hex);
        
        strcat(final_hex_msg, "002C"); // ,
        
        sprintf(str_coords, "%.7f", dec_lat);
        StrToUcs2Hex(str_coords, tmp_hex);
        strcat(final_hex_msg, tmp_hex);
        
    }
    else {
        // --- GPS失效，尝试基站定位 (LBS) ---
        usart2_ClearPacket();
        usart2_Printf("AT+CIPGSMLOC=1,1\r\n");
		
        if(WaitReply("+CIPGSMLOC:", "ERROR", 5000)){
            char *ptr = strstr(USART2_RxPacket, "+CIPGSMLOC:");
			
			
            if(ptr) {
                // 解析格式：code,纬度,经度,日期,时间
				
                // 手动解析：跳过 "+CIPGSMLOC: " 这 11 个字符
                char *p = ptr + 11; 
                
                // 1. 获取 code
                code = atoi(p); 
                
                // 2. 找到第一个逗号，后面是 Latitude
                p = strchr(p, ',');
                if(p && code == 0) {
                    p++; // 跳过逗号
                    char *next_comma = strchr(p, ',');
                    if(next_comma) {
                        int len = next_comma - p;
                        if(len < 20) {
                            memcpy(lbs_lat, p, len);
                            lbs_lat[len] = '\0'; // 提取出纬度
                        }
                        
                        // 3. 找到第二个逗号，后面是 Longitude
                        p = next_comma + 1;
                        next_comma = strchr(p, ',');
                        if(next_comma) {
                            len = next_comma - p;
                            if(len < 20) {
                                memcpy(lbs_lon, p, len);
                                lbs_lon[len] = '\0'; // 提取出经度
                            }

                            // --- 执行拼接逻辑 ---
                            strcat(final_hex_msg, "4FE153F75F31FF0C57FA7AD95B9A4F4DFF1A"); // 基站定位抬头
                            strcat(final_hex_msg, "000A");
                            
                            StrToUcs2Hex(lbs_lat, tmp_hex); 
                            strcat(final_hex_msg, tmp_hex);
                            strcat(final_hex_msg, "002C"); // ,
                            StrToUcs2Hex(lbs_lon, tmp_hex); 
                            strcat(final_hex_msg, tmp_hex);
                        }
                    }
                } else if (code != 0) {
                    strcat(final_hex_msg, "65E06CD583B7"); // 定位失败
                }
            }
        }
    }   

    // --- 最后统一发送 ---
    Delay_ms(200);
    usart2_ClearPacket(); 
    Air780E_SendSMS(number, final_hex_msg);
	
}


void Voice_Send(uint8_t rx)
{
	if (rx)
    {
        if (Serial_RxPacket[0] == 3)
		{			
			Send_Fall_Location_SMS();
		}
    }
}

/* MQTT连接 */
uint8_t MQTT_Connect(void)
{
    usart2_ClearPacket();
    usart2_Printf("AT+CGATT?\r\n");
    if(!WaitReply("OK","ERROR",1000)) return 0;
	
    usart2_ClearPacket();
    usart2_Printf("AT+MCONFIG=\"xxxl\",\"pUd70U826Z\",\"version=2018-10-31&res=products%%2FpUd70U826Z%%2Fdevices%%2Fxxxl&et=1803546780&method=md5&sign=W3H1H6SqhAvQRDL%%2F8TZeYQ%%3D%%3D\"\r\n");
    if(!WaitReply("OK","ERROR",2000)) return 0;

	
    usart2_ClearPacket();
    usart2_Printf("AT+MIPSTART=\"mqtts.heclouds.com\",1883\r\n");
    if(!WaitReply("OK","ERROR",3000)) return 0;
	
    usart2_ClearPacket();
    usart2_Printf("AT+MCONNECT=1,120\r\n");
    if(!WaitReply("OK","ERROR",2000)) return 0;
	
	usart2_ClearPacket();
    usart2_Printf("AT+MSUB=\"$sys/pUd70U826Z/xxxl/thing/property/post/reply\",0\r\n");
    if(!WaitReply("OK","ERROR",2000)) return 0;

    return 1;
}

/**
 * @brief  上传心率、血氧和位置数据到 OneNET (MQTT 协议)
 * @return 1: 成功, 0: 失败
 */
uint8_t Air780E_MQTT_Upload(void)
{
    static char json[256];
	
	memset(json,0,sizeof(json));
	
    /* ---------- 生成JSON ---------- */
    sprintf(json,
        "{\\22id\\22:\\22123\\22,\\22params\\22:{"
		"\\22command\\22:{\\22value\\22:%d},"
        "\\22hr\\22:{\\22value\\22:%d},"
        "\\22spo2\\22:{\\22value\\22:%d}"
        "}}",
		sms_lock,
        hr,
        spo2
	);
	
    /* ---------- 发布 ---------- */
    usart2_ClearPacket();
    usart2_Printf("AT+MPUB=\"$sys/pUd70U826Z/xxxl/thing/property/post\",0,0,\"%s\"\r\n",json);
    if(!WaitReply("OK","ERROR",500)) return 0;
	
    return 1;
}

uint8_t Air780E_MQTT_GPS_Upload(void)
{
    static char json[256];
	
	memset(json,0,sizeof(json));
	
    double lon = 0;
    double lat = 0;
    uint8_t useLBS = 0;

    /* ---------- 判断定位来源 ---------- */
    if(Save_Data.isUsefull)
    {
        lon = ConvertToDecimalDegrees(Save_Data.longitude);
        lat = ConvertToDecimalDegrees(Save_Data.latitude);
    }
    else
    {
        char lbs_lon[24]={0};
		char lbs_lat[24]={0};
		int code;

		usart2_ClearPacket();
		usart2_Printf("AT+CIPGSMLOC=1,1\r\n");

		if(WaitReply("OK", "ERROR", 3000))
		{
			char *ptr = strstr(USART2_RxPacket, "+CIPGSMLOC:");
			if(ptr)
			{
				/* 找冒号（比固定+11安全） */
				char *p = strchr(ptr,':');
				if(!p) goto lbs_fail;
				p++;

				/* 跳过空格 */
				while(*p==' ') p++;

				/* code */
				code = atoi(p);
				if(code!=0) goto lbs_fail;

				/* 找第一个逗号 → 纬度 */
				p = strchr(p,',');
				if(!p) goto lbs_fail;
				p++;

				char *comma = strchr(p,',');
				if(!comma) goto lbs_fail;

				int len = comma - p;
				if(len<=0 || len>20) goto lbs_fail;

				memcpy(lbs_lat,p,len);
				lbs_lat[len]=0;

				/* 找第二个逗号 → 经度 */
				p = comma+1;
				comma = strchr(p,',');
				if(!comma) goto lbs_fail;

				len = comma - p;
				if(len<=0 || len>20) goto lbs_fail;

				memcpy(lbs_lon,p,len);
				lbs_lon[len]=0;

				/* 转double */
				lat = atof(lbs_lat);
				lon = atof(lbs_lon);

				/* 合法性校验（关键） */
				if(lat<-90 || lat>90) goto lbs_fail;
				if(lon<-180 || lon>180) goto lbs_fail;

				useLBS = 1;
                
            }
        }
    }

	lbs_fail:;
	
	int32_t lon_i = (int)(lon * 10000000);
	int32_t lat_i = (int)(lat * 10000000);
	
    /* ---------- 生成JSON ---------- */
	sprintf(json,
        "{\\22id\\22:\\22123\\22,\\22params\\22:{"
        "\\22gps\\22:{\\22value\\22:{\\22lon\\22:%d,\\22lat\\22:%d,\\22src\\22:\\22%s\\22}}"
        "}}",
        lon_i,
        lat_i,
        useLBS ? "LBS" : "GPS"
	);
	
    /* ---------- 发布 ---------- */
    usart2_ClearPacket();
    usart2_Printf("AT+MPUB=\"$sys/pUd70U826Z/xxxl/thing/property/post\",0,0,\"%s\"\r\n",json);
    if(!WaitReply("OK","ERROR",500)) return 0;

    return 1;
}


