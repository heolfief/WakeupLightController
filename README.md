# WakeupLightController
**[THIS IS A WORK IN PROGRESS]**

This is a small board designed to control LEDs (for example LED strip) as a wake-up light.

## What it does
The controller function in two modes :
* **Fade lamp**, LED intensity is controlled by an external potentiometer
* Parametric Progressive **wake-up light**

Wake-up times can be set by the dedicated **bluetooth android app**, and can be set for the week.
For example you can set the light to wake you up at half past 8 on Monday and at 12 on Sunday.

You can enable or disable alarm for each day with the app, or you can manually disable all alarms with an external switch.

When a alarm time approaches, the lamp will start to fade-in linearly and will be at full intensity when alarm time is reached.
The duration of the fade-in can be set via the app.

The board will keep alarm settings until new one are set, even if power goes down.


## How it works

The board is based on an AVR microcontroller (ATmega328p), a bluetooth serial module (HC-05), and a real time clock (DS3231).
The MCU is programmed in C within the Atmel Studio 7 environment.

The real time clock is extremely acurate and will only drift few second per year. It is used to keep track of time to trigger alarms.

The bluetooth module is used to pair an android device with the board, and to send and get settings from a user.

The programm is composed of a main state machine and run by interrupts.

The LED intensity is controlled by a 16bits PWM. As the ADC is only 10bits, when using the board as a lamp with a potentiommeter, LED intensity will only be controlled with a 10 bit resolution. That mean it as 1024 intensity steps which is enough for most applications. However, when using the wake-up function, the LED intensity will be controlled by a 16bit resolution (65536 steps) which is good for a very slow fade.

Brightness intensity is automaticaly compensated to be seen as linear by the human eye.

Settings are kept even if power goes off by writing them to the EEPROM.

## Wiring

#### What you will need to connect to the board :
* A 12v power source capable to deliver current for your LED setup + 200mA (max) for the board.
* LEDs, for example an LED strip. The board is thermally capable of delivering up to 4A of currents.
* A potentiometer (1 to 100k linear). You can use a rotary or a linear one.
* A momentary button for bluetooth pairing.  
* A switch if you want to add a way to manualy disable all alarms (even if they are enable by the app).
