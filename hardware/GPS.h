#ifndef __GPS_H
#define __GPS_H

void errorLog(int num);
void parseGpsBuffer(void);
void printGpsBuffer(void);
double ConvertToDecimalDegrees(const char* ddmm);

#endif
