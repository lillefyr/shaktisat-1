#ifndef DS3231_H
#define DS3231_H

#define  DS3231M_ADDRESS            0x68       // Fixed I2C Address for DS3231M
#define  DS3231M_RTCSEC             0x00       // DS3231 RTCSEC      Register Address
#define  DS3231M_RTCMIN             0x01       // DS3231 RTCMIN      Register Address
#define  DS3231M_RTCHOUR            0x02       // DS3231 RTCHOUR     Register Address
#define  DS3231M_RTCWKDAY           0x03       // DS3231 RTCWKDAY    Register Address
#define  DS3231M_RTCDATE            0x04       // DS3231 RTCDATE     Register Address
#define  DS3231M_RTCMTH             0x05       // DS3231 RTCMTH      Register Address
#define  DS3231M_RTCYEAR            0x06       // DS3231 RTCYEAR     Register Address
#define  DS3231M_ALM1SEC            0x07       // DS3231 ALM1SEC     Register Address
#define  DS3231M_ALM1MIN            0x08       // DS3231 ALM1MIN     Register Address
#define  DS3231M_ALM1HOUR           0x09       // DS3231 ALM1HOUR    Register Address
#define  DS3231M_ALM1DATE           0x0A       // DS3231 ALM1DATE    Register Address
#define  DS3231M_ALM2MIN            0x0B       // DS3231 ALM2SEC     Register Address
#define  DS3231M_ALM2HOUR           0x0C       // DS3231 ALM2HOUR    Register Address
#define  DS3231M_ALM2DATE           0x0D       // DS3231 ALM2DATE    Register Address
#define  DS3231M_CONTROL            0x0E       // DS3231 CONTROL     Register Address
#define  DS3231M_STATUS             0x0F       // DS3231 STATUS      Register Address
#define  DS3231M_AGING              0x10       // DS3231 AGING       Register Address
#define  DS3231M_TEMPERATURE        0x11       // DS3231 TEMPERATURE Register Address

int DS3231_init();
int readDS3231(char * read_buf);
int updateDS3231Time( unsigned int hour, unsigned int minute, unsigned int second,
                      unsigned int date, unsigned int month, unsigned int year);
void setAlarmEveryMinute();  // Max 60 seconds to next alarm
void pinAlarm(); // do not know which pin
#endif
