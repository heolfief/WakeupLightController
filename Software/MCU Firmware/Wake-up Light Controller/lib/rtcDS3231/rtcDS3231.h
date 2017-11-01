/*
* rtcDS3231.h
* 
* Library for the DS3231 real time clock
*
* Created: 10/04/2017 15:15:46
* Author : Heol
*
* This software is released under the Creative Commons Attribution Share-Alike 4.0 License
*
*/ 

#include <avr/io.h>

#include "I2C-master-lib-master/i2c_master.h"

#ifndef RTC_DS3231
#define RTC_DS3231

// Time setting functions
void rtc_set_time_24h(uint8_t hour, uint8_t min, uint8_t sec);
void rtc_set_time_12h(uint8_t hour, uint8_t min, uint8_t sec, uint8_t am_pm);

// Date setting function
void rtc_set_date(uint16_t year, uint8_t month, uint8_t date);

// Time getting functions
void rtc_get_time_24h(uint8_t *hour, uint8_t *min, uint8_t *sec);
void rtc_get_time_12h(uint8_t *hour, uint8_t *min, uint8_t *sec, uint8_t *am_pm);

// Date getting functions
void rtc_get_date(uint16_t *year, uint8_t *month, uint8_t *date);
uint8_t rtc_get_day();

// Formating functions
uint8_t bcdtodec(uint8_t bcd);
uint8_t dectobcd(uint8_t dec);

#endif