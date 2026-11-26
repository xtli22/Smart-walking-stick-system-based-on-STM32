#ifndef __AIR780E_H
#define __AIR780E_H

void Air780E_Init(void);
uint8_t WaitReply(char *ok, char *err, uint16_t timeout);
uint8_t Air780E_SendSMS(char *PhoneNum, char *Message);
void IntToUcs2Hex(int val, char* dest);
void StrToUcs2Hex(char* src, char* dest);

void Send_Fall_Location_SMS(void);
void Send_Safe_SMS(void);
void Voice_Send(uint8_t rx);
uint8_t MQTT_Connect(void);
uint8_t Air780E_MQTT_Upload(void);
uint8_t Air780E_MQTT_GPS_Upload(void);
	
#endif
