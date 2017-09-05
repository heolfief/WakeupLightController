/*
* Wake-up Light Controller code
*
* Designed to be run on Atmel ATmega328P MCU
* 
* More informations and hardware on :
* https://github.com/heolfief/WakeupLightController
* 
* Created: 12/08/2017 15:57:40
* Author : Heol Fief
* 
* This hardware and software are released under the Creative Commons Attribution Share-Alike 4.0 License
* https://creativecommons.org/licenses/by-sa/4.0/
*/
 
// Hardware related constants//////////////////////////////
#define F_CPU		16000000UL							// 16MHz external oscillator
#define BT_PWR		PD4									// ...
#define RTC_PWR		PC1									// ...
#define AT_MODE		PD6									// ...
#define ALRM_BTN	PD2									// ...
#define PAIR_BTN	PD3									// ...
#define PAIR_INFO	PC1									// ... Define IO

// Software related constants//////////////////////////////
#define ALRM_EN		1									// ...
#define ALRM_DUR	2									// ...
#define ALRM_HR		3									// ...
#define ALRM_MIN	4									// ...
#define SET_HR		5									// ...
#define SET_MIN		6									// ... Define bluetooth data IDs
#define BAUDRATE	38400								// Baud rate for bluetooth module USART communication


// User definable constants///////////////////////////////
#define PAIR_NAME	"Wake up light"						// Name of the bluetooth device, change it to whatever you like (Max 20 characters)
#define PAIR_PIN	"1234"								// Password of the bluetooth device pairing, change it to whatever you like
#define OFFTHRESHOLD 2									// Threshold (10 bit value) for potentiometer to be read as 0 (lamp off) to compensate for noise

// Libraries used/////////////////////////////////////////
#include <avr/io.h>										// Inputs / Output library
#include <util/delay.h>									// Delay function library
#include <avr/interrupt.h>								// Interrupts library
#include <math.h>										// Library for pow() function
#include <avr/eeprom.h>									// EEPROM library
#include "lib/rtcDS3231/rtcDS3231.h"					// Include RTC library

// Functions declarations/////////////////////////////////
void IO_init();
void read_eeprom_settings();
void write_eeprom_settings();
void PWM_init();
void PWM_on();
void PWM_off();
void ADC_init();
void UART_init();
void UART_transmit(char txdata);
void UART_transmit_String(char *txdata);
char UART_receive();
void alarm_start_time_calculation();
void BT_init();
void update_state_machine();

// Variables declarations/////////////////////////////////
enum statemachine {INIT, STANDBY, LAMP, ALARM, PAIRING} state = INIT; // Machine states

uint8_t alrm_EN[7] = {0};								// Alarm enable array
uint8_t alrm_duration = 0;								// Alarm duration
uint8_t alrm_hr[7] = {0};								// Alarm hour array (actual alarm time)
uint8_t alrm_min[7] = {0};								// Alarm minute array (actual alarm time)
uint8_t alrm_start_hr[7] = {0};							// Alarm start hour array (calculated progressive start alarm time)
uint8_t alrm_start_min[7] = {0};						// Alarm start minute array (calculated progressive start alarm time)
uint8_t id;												// Global variable for bluetooth id data
uint8_t day;											// Global variable for bluetooth day data
uint8_t data;											// Global variable for bluetooth data
uint8_t actual_hour;                                    // Global variable for hours from RTC
uint8_t actual_min;                                     // Global variable for minutes from RTC
uint8_t actual_sec;                                     // Global variable for seconds from RTC
uint8_t actual_day;                                     // Global variable for day from RTC

uint16_t slider = 0;									// Slider pot value

uint8_t pair_request_flag = 0;							// Flag set when pushing pair button
uint8_t is_first_byte_received = 1;						// Flag set when first UART byte is received
uint8_t is_paired = 0;									// Flag set when bluetooth is paired
uint8_t alarm_in_process = 0;							// Flag set when an alarm is in process
uint8_t stop_alrm = 0;									// Flag set when pushing stop button

uint8_t tmr1ovf = 0;                                    // Timer 1 overflow counter
uint8_t duty_cycle_increments;							// Increments used for PWM fade-in during the alarm

// Main programm//////////////////////////////////////////
int main(void)
{
	
	while (1)
	{
		update_state_machine();
	}
}

// Functions//////////////////////////////////////////////
void IO_init()
{
	DDRC |= (1<<RTC_PWR);								// RTC_PWR as output
	DDRC &= ~(1<<PAIR_INFO);							// PAIR_INFO as input
	DDRD |= (1<<BT_PWR) | (1<<AT_MODE);					// BT_PWR and AT_MODE as outputs
	DDRD &= ~((1<<ALRM_BTN) | (1<<PAIR_BTN));			// Alarm and pair buttons as inputs
	
	PORTC &= ~(1<<RTC_PWR);								// Low level (RTC off)
	PORTD |= (1<<BT_PWR);								// High level (BT module off)
	PORTD &= ~(1<<AT_MODE);								// Low level
	
	PORTD |= (1<<ALRM_BTN) | (1<<PAIR_BTN);				// Enable (weak) internals pullups
	
	PCICR |= (1<<PCIE2);								// Enable pin change interrupts
	PCMSK2 |= (1<<PCINT21);								// Enable pin change interrupt 21 (connected to pairing pin of bluetooth module)
	
	EIMSK |= (1<<INT1);									// Enable external interrupt INT1
	EICRA |= (1<<ISC11);								// ...
	EICRA &= ~(1<<ISC10);								// ... Enable interrupt on falling edge of INT1
}

void read_eeprom_settings()								// Read previously stored settings
{
	uint8_t i;											// Cursor
	for (i=1;i<8;i++)									// For each day of the week (start at address 0x01 to avoid unsafe address 0x00)
	{
		alrm_EN[i-1] = eeprom_read_byte((uint8_t*) i);	// Get stored settings for alrm_EN
	}
	for (i=1;i<8;i++)									// For each day of the week
	{
		alrm_hr[i-1] = eeprom_read_byte((uint8_t*) i+7);// Get stored settings for alarm_hr
	}
	for (i=1;i<8;i++)									// For each day of the week
	{
		alrm_min[i-1] = eeprom_read_byte((uint8_t*) i+14);// Get stored settings for alrm_min
	}
	alrm_duration = eeprom_read_byte((uint8_t*) 22);	// Get stored settings for alrm_duration
}

void write_eeprom_settings()							// Store settings
{
	eeprom_update_block(alrm_EN, 0x01, 7);				// Write 7 bytes of alrm_EN data to address 1
	eeprom_update_block(alrm_hr, 0x08, 7);				// Write 7 bytes of alrm_hr data to address 8
	eeprom_update_block(alrm_min, 0x0F, 7);				// Write 7 bytes of alrm_min data to address 15
	eeprom_update_byte(0x16, alrm_duration);			// Write 1 bytes of alrm_duration data to address 22
}

void PWM_init()
{
	TCCR1A |= (1<<COM1A1);								// ...
	TCCR1A &= ~(1<<COM1A0);								// ... Non-inverting mode on OC1A
	
	TCCR1A |=(1<<WGM11);								// ...
	TCCR1A &= ~(1<<WGM10);								// ...
	TCCR1B |= (1<<WGM13);								// ...
	TCCR1B |= (1<<WGM12);								// Fast PWM mode 14, ICR1 as TOP
	
	TCCR1B |= (1<<CS10);								// ...
	TCCR1B &= ~((1<<CS12) | (1<<CS11));					// No prescaler
	
	TIMSK1 |= (1<<TOIE1);								// Enable interrupt on timer 1 overflow
	
	ICR1 = 0xFFFF;										// TOP set to max value -> 244Hz
	
	OCR1A = 0;											// Duty cycle to 0% (as default)
}

void PWM_on()
{
	TCCR1A |= (1<<COM1A1);								// ...
	TCCR1A &= ~(1<<COM1A0);								// ... Non-inverting mode on OC1A
	
	TCCR1A &= ~(1<<WGM11);								// ...
	TCCR1A &= ~(1<<WGM10);								// ...
	TCCR1B &= ~(1<<WGM13);								// ...
	TCCR1B &= ~(1<<WGM12);								// ... Normal mode, no PWM
	
	TCCR1C |= (1<<FOC1A);								// Force output compare (to remove PWM spike glitch)
	
	TCCR1A |=(1<<WGM11);								// ...
	TCCR1A &= ~(1<<WGM10);								// ...
	TCCR1B |= (1<<WGM13);								// ...
	TCCR1B |= (1<<WGM12);								// ... Fast PWM mode 14, ICR1 as TOP
}

void PWM_off()
{
	TCCR1A &= ~(1<<COM1A1);								// ...
	TCCR1A &= ~(1<<COM1A0);								// ... Normal mode, OC1A disconnected
}

void ADC_init()
{
	ADMUX &= ~((0<<REFS1) | (1<<REFS0));				// VCC used as Voltage Reference
	ADMUX &= ~(1<<ADLAR);								// ADC Right Adjust Result
	ADMUX &= ~((1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0)); // ... Select ADC0
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);		// ADC clock prescaler /128
	ADCSRA |= (1<<ADIE) | (1<<ADEN);					// Enable ADC and enable interrupt
	ADCSRA |= (1<<ADSC);								// Start ADC conversion
}

void UART_init()
{
	UBRR0 = F_CPU/16/BAUDRATE-1;						// Set baud rate
	UCSR0A &= ~(1<<U2X0);								// Normal speed mode
	UCSR0B |= ((1<<RXEN0) | (1<<TXEN0));				// Enable RX and TX
	UCSR0C &= ~((1<<UMSEL01) | (1<<UMSEL00));			// Asynchronous USART
	UCSR0C &= ~((1<<UPM01) | (1<<UPM00));				// Parity check disabled
	UCSR0C &= ~(1<<USBS0);								// 1 stop bit
	
	UCSR0B &= ~(1<<UCSZ02);								// ...
	UCSR0C |= ((1<<UCSZ01) | (1<<UCSZ00));				// ... 8 bits character size
}

void UART_transmit(char txdata)
{
	loop_until_bit_is_set(UCSR0A, UDRE0);				// Wait for transmit register to be free
	UDR0 = txdata;										// load data and transmit
}

void UART_transmit_String(char *txdata)
{
	while(*txdata) UART_transmit(*(txdata++));			// While end of string not reached, transmit each characters one by one
}

char UART_receive()
{
	loop_until_bit_is_set(UCSR0A, RXC0);				// Wait while data is being received
	return UDR0;										// Return received data
}

void alarm_start_time_calculation()						// Process the start alarm time based on fading alarm duration
{
	uint8_t days_of_week;
	for (days_of_week=0;days_of_week<7;days_of_week++)
	{
		if (alrm_min[actual_day] < alrm_duration)
		{
			alrm_start_hr[actual_day] = alrm_hr[actual_day] - 1;		// Recalculate the hour
			alrm_start_min[actual_day] = 60 - (alrm_duration - alrm_min[actual_day]); // Recalculate the minutes
		}
		else
		{
			alrm_start_hr[actual_day] = alrm_hr[actual_day];
			alrm_start_min[actual_day] = alrm_min[actual_day] - alrm_duration;
		}
	}
	
}

void BT_init()
{
	char buffer[6];										// Buffer for itoa function
	cli();												// Global interrupts disable
	PORTD &= ~(1<<BT_PWR);								// Give power to bluetooth module
	PORTD |= (1<<AT_MODE);								// Enter configuration (AT) mode
	
	UART_transmit_String("AT\r\n");						// Send test command to bluetooth module
	
	if((UART_receive() != 'O') || (UART_receive() != 'K'))// If bluetooth module doesn't answer "OK" then reset bluetooth module
	{
		PORTD |= (1<<BT_PWR);							// Cut power to bluetooth module
		_delay_ms(500);									// Wait 500ms
		PORTD &= ~(1<<BT_PWR);							// Give power to bluetooth module
		PORTD |= (1<<AT_MODE);							// Enter configuration (AT) mode
	}
	itoa(BAUDRATE, buffer, 10);							// Convert BAUDRATE value to a string
	UART_transmit_String("AT+UART=");					// ...	
	UART_transmit_String(buffer);						// ... Set bluetooth module baud rate to 38400 (previously converted from int to string)
	UART_transmit_String(",1,0\r\n");					// One stop bit, no parity check
	
	UART_transmit_String("AT+ROLE=0\r\n");				// Set bluetooth module role to slave
	
	UART_transmit_String("AT+POLAR=0,1\r\n");			// Low drive LED and high drive PAIR_INFO
	
	UART_transmit_String("AT+NAME=");					// ...
	UART_transmit_String(PAIR_NAME);					// ... Change the name of the bluetooth module
	UART_transmit_String("\r\n");						// Send carriage return and new line
	
	UART_transmit_String("AT+PSWD=");					// ...
	UART_transmit_String(PAIR_PIN);						// ... Change the bluetooth pairing password
	UART_transmit_String("\r\n");						// Send carriage return and new line
	
	_delay_ms(200);										// Wait for bluetooth module to stop communicate
	PORTD &= ~(1<<AT_MODE);								// Exit configuration (AT) mode
	PORTD |= (1<<BT_PWR);								// Cut power to bluetooth module
	sei();												// Global interrupts enable
}

void update_state_machine()								// State machine
{
	switch (state)
	{
		case INIT :
		IO_init();
		read_eeprom_settings();
		PWM_init();
		ADC_init();
		UART_init();
		BT_init();
		sei();											// Global interrupts enable
		state = STANDBY;
		break;

		case STANDBY :
		PWM_off();										// Turn off PWM output
		if(slider > OFFTHRESHOLD) state = LAMP;			// If slider isn't on 0, turn on the Lamp
		if(pair_request_flag) state = PAIRING;			// If pair button is pressed, go to pair state
		if(alarm_in_process) state = ALARM;				// If an alarm is reached go to alarm state
		break;

		case LAMP :
		PWM_on();										// Turn on PWM output
		if(slider < OFFTHRESHOLD) state = STANDBY;		// If slider is on 0, turn off the Lamp
		if(pair_request_flag) state = PAIRING;			// If pair button is pressed, got to pair mode
		break;

		case PAIRING :
		PORTD &= ~(1<<BT_PWR);							// Give power to bluetooth module
		sei();											// Enable global interrupts
		UCSR0B |= ((1<<RXCIE0) | (1<<TXCIE0));			// Enable RX and TX interrupts
		while (is_paired);								// Wait for bluetooth communication complete
		UCSR0B &= ~((1<<RXCIE0) | (1<<TXCIE0));			// Disable RX and TX interrupts
		pair_request_flag = 0;							// Pairing complete
		PORTD |= (1<<BT_PWR);							// Cut power to bluetooth module
		break;

		case ALARM :
		ADCSRA &= ~(1<<ADIE);							// Disable ADC interrupt, therefore stop ADC		
		break;

		default : state = INIT;							// In case of a fault, get back to INIT state
	}
}

// Interrupt vectors//////////////////////////////////////

ISR(INT1_vect)											// When pair / stop button is pressed
{
	if (alarm_in_process)								// If alarm is in process
		alarm_in_process = 0;							// Stop the alarm
	else												// If no alarm is in process
		pair_request_flag = 1;							// Go to bluetooth pair state
}

ISR(PCINT2_vect)										// Interrupt on bluetooth pairing status changed
{
	if (bit_is_set(PORTD, PAIR_INFO))					// If bluetooth is paired
	{
		is_paired = 1;									// Flag set
	} 
	else												// If bluetooth is not paired
	{
		is_paired = 0;									// Flag reset
	}
}

ISR(TIMER1_OVF_vect)									// Interrupt on PWM timer overflow, each 4.096ms (244Hz)
{
	tmr1ovf ++;											// Overflow counter
	if(tmr1ovf == 122)									// Each 0.5sec (122*4.096ms = 0.5s)
	{
		tmr1ovf = 0;									// Reset the overflow counter
		PORTC |= (1<<RTC_PWR);							// Give power to RTC
		rtc_get_time_24h(&actual_hour, &actual_min, &actual_sec); // Get time from RTC
		actual_day = rtc_get_day();						// Get day from RTC
		PORTC &= ~(1<<RTC_PWR);							// Cut power to RTC
	}
	if(!alarm_in_process && (actual_hour == alrm_start_hr[actual_day]) && (actual_min == alrm_start_min[actual_day])) // If alarm time is reached and no alarm is in process
	{
		alarm_in_process = 1;							// Set the flag
		OCR1A = 0;										// Start PWM fade with a duty cycle of 0%
		duty_cycle_increments = (float)alrm_duration*60/65536/0.004096; // Calculate OCR1A increments based on alrm_duration
	}
	if (alarm_in_process)								// If alarm is in process
	{
		if (stop_alrm)									// If Pair/stop button is pressed
		{
			alarm_in_process = 0;						// Stop the alarm
		}
		if ((OCR1A + duty_cycle_increments) < 65535)	// Avoid 16bit value overflow
			OCR1A = OCR1A + duty_cycle_increments;		// Increase duty cycle progressively 
		else OCR1A = 65535;								// Set duty cycle to 100%
	}
	else ADCSRA |= (1<<ADSC);							// Start ADC conversion if no alarm is in process
}

ISR(ADC_vect)											// Interrupt on ADC conversion complete
{
	slider = ADC;										// Read ADC value
	OCR1A = pow(2,(float)slider/64)-1;					// Converts linear response to "anti-log" (to compensate for eye brightness perception)
														// And Set PWM duty cycle to converted ADC value
}

ISR(USART_RX_vect)										// Interrupt on UART reception complete
{
	uint8_t rxdata = UDR0;								// Write received data to rxdata
	if(is_first_byte_received)							// If received byte is the first, as two byte are received by bluetooth each time
	{
		id = rxdata >> 3;								// Get only identification variable of received data
		day = rxdata & 0x07;							// Get only day variable of received data
		is_first_byte_received = 0;
	}
	else
	{
		data = rxdata;									// Get data
		is_first_byte_received = 1;
	}
	switch(id)
	{
		case ALRM_EN : alrm_EN[day] = (data & 0x01);		// Assign alarm enable data to alrm_EN array
		break;
		case ALRM_DUR : alrm_duration = data;				// Assign data to alarm duration variable
		break;
		case ALRM_HR : alrm_hr[day] = data;					// Assign data to alarm hour array
		break;
		case ALRM_MIN : alrm_min[day] = data;				// Assign data to alarm min array
		break;
		case SET_HR : rtc_set_time_24h(data, actual_min, actual_sec); // Set hour
		break;
		case SET_MIN : rtc_set_time_24h(actual_hour, data, actual_sec);	// Set minute
		break;
	}
	write_eeprom_settings();							// Write settings to EEPROM
	
}