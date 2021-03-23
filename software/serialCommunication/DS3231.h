#ifndef DS3231_H
#define DS3231_H
int initDS3231();
int readDS3231(char * read_buf);
int updateDS3231Time( unsigned int hour, unsigned int minute, unsigned int second,
                      unsigned int date, unsigned int month, unsigned int year);
#endif
