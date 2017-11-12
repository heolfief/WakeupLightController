/*
* rtcDS3231.c
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

#include "rtcDS3231.h"

#define RTC_ADDR 0x68 // I2C address

// Binairy coded decimal to decimal
uint8_t bcdtodec(uint8_t bcd)
{
	return ((bcd>>4)*10) + (bcd & 0x0F);
}

// Decimal to binary coded decimal
uint8_t dectobcd(uint8_t dec)
{
	return ((dec/10)<<4) + (dec % 10);
}

// Set time in 24h format (if used, only use rtc_get_time_24h function to get time)
void rtc_set_time_24h(uint8_t hour, uint8_t min, uint8_t sec)
{
	i2c_start((RTC_ADDR<<1) | I2C_WRITE); 		// Start I2C at RTC adress, write mode
	i2c_write(0x00);							// Point to register adress 0x00

	i2c_write(dectobcd(sec));					// Write seconds to register
	i2c_write(dectobcd(min));					// Write minutes to register
	i2c_write(dectobcd(hour));					// Write hours to register

	i2c_stop();									// End I2C communication
}

// Set time in 12h format (if used, only use rtc_get_time_12h function to get time)
void rtc_set_time_12h(uint8_t hour, uint8_t min, uint8_t sec, uint8_t am_pm)
{
	uint8_t temphour = 0;

	if (hour >= 12) return; 					// Reject 24h setting

	if (am_pm)									// if PM
		temphour = 0x40 | 0x20 | dectobcd(hour);// Set pm bit in register
	else       									// if AM
		temphour = 0x40 | dectobcd(hour);		// clear pm bit in register

	i2c_start((RTC_ADDR<<1) | I2C_WRITE); 		// Start I2C at RTC adress, write mode
	i2c_write(0x00);							// Point to register adress 0x00
	i2c_write(dectobcd(sec));					// Write seconds to register
	i2c_write(dectobcd(min));					// Write minutes to register
	i2c_write(temphour);						// Write hours to register

	i2c_stop();									// End I2C communication
}

void rtc_set_date(uint16_t year, uint8_t month, uint8_t date)
{
	uint8_t century = 0;

	if((year / 100) == 20)						// If year is 20xx
	{
		year = year - 2000;						// only keep 2 last digits
		century = 1;							// Set century bit
	}
	else
	{
		if((year / 100) == 19)					// If year is 19xx
		{
			year = year - 1900;					// only keep 2 last digits
			century = 0;						// Clear century bit
		}
	}

	i2c_start((RTC_ADDR<<1) | I2C_WRITE); 		// Start I2C at RTC adress, write mode
	i2c_write(0x04);							// Point to register adress 0x04

	i2c_write(dectobcd(date));					// Write date to register
	i2c_write((century << 7) | dectobcd(month));// Write century bit and month to register
	i2c_write(dectobcd(year));					// Write year to register

	i2c_stop();									// End I2C communication
}

void rtc_set_day(uint8_t day)
{
	i2c_start((RTC_ADDR<<1) | I2C_WRITE); 		// Start I2C at RTC adress, write mode
	i2c_write(0x03);							// Point to register adress 0x03
	i2c_write(dectobcd(day));					// Write day to register
	i2c_stop();									// End I2C communication
}

// Set time in 24h format (only use if rtc_get_time_24h function has been called once to program RTC to work in 24h mode)
void rtc_get_time_24h(uint8_t *hour, uint8_t *min, uint8_t *sec)
{
	i2c_start((RTC_ADDR<<1) | I2C_WRITE); 		// Start I2C at RTC adress, write mode
	i2c_write(0x00);							// Point to register adress 0x00
	i2c_stop();									// End I2C communication

	i2c_start((RTC_ADDR<<1) | I2C_READ); 		// Start I2C at RTC adress, read mode

	*sec = bcdtodec(i2c_read_ack());			// Read seconds from register
	*min = bcdtodec(i2c_read_ack());			// Read minutes from register
	*hour = bcdtodec(i2c_read_nack() & 0x3F);	// Read hours from register

	i2c_stop();									// End I2C communication
}

// Set time in 12h format (only use if rtc_get_time_12h function has been called once to program RTC to work in 12h mode)
void rtc_get_time_12h(uint8_t *hour, uint8_t *min, uint8_t *sec, uint8_t *am_pm)
{
	uint8_t temphour;

	i2c_start((RTC_ADDR<<1) | I2C_WRITE); 		// Start I2C at RTC adress, write mode
	i2c_write(0x00);							// Point to register adress 0x00
	i2c_stop();									// End I2C communication

	i2c_start((RTC_ADDR<<1) | I2C_READ); 		// Start I2C at RTC adress, read mode

	*sec = bcdtodec(i2c_read_ack());			// Read seconds from register
	*min = bcdtodec(i2c_read_ack());			// Read minutes from register
	temphour = i2c_read_nack();					// Read hours from register

	i2c_stop();									// End I2C communication

	*hour =  bcdtodec(temphour & 0x1F);			// Format hours

	if (temphour & 0x20)						// if PM
		*am_pm = 1;								// Set am_pm bit to 1 (PM)
	else										// if AM
		*am_pm = 0;								// Set am_pm bit to 0 (AM)
}

void rtc_get_date(uint16_t *year, uint8_t *month, uint8_t *date)
{
	uint8_t tempmonth = 0, tempyear = 0;

	i2c_start((RTC_ADDR<<1) | I2C_WRITE); 		// Start I2C at RTC adress, write mode
	i2c_write(0x04);							// Point to register adress 0x04
	i2c_stop();									// End I2C communication

	i2c_start((RTC_ADDR<<1) | I2C_READ); 		// Start I2C at RTC adress, read mode

	*date = bcdtodec(i2c_read_ack());			// Read date from register
	tempmonth = i2c_read_ack();					// Read month from register
	tempyear = bcdtodec(i2c_read_nack());		// Read year from register

	i2c_stop();									// End I2C communication

	*month = bcdtodec(tempmonth & 0x1F);		// Format month

	if(tempmonth & 0x80)						// If year is 20xx
		*year = tempyear + 2000;				// Add 2000 to year
	else 										// If year is 19xx
		*year = tempyear + 1900;				// Add 1900 to year
}

uint8_t rtc_get_day()
{
	uint8_t day;

	i2c_start((RTC_ADDR<<1) | I2C_WRITE); 		// Start I2C at RTC adress, write mode
	i2c_write(0x03);							// Point to register adress 0x03
	i2c_stop();									// End I2C communication

	i2c_start((RTC_ADDR<<1) | I2C_READ); 		// Start I2C at RTC adress, read mode
	day = bcdtodec(i2c_read_nack() & 0x07);		// Read day from register
	i2c_stop();									// End I2C communication

	return day;
}
