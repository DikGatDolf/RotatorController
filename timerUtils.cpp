/******************************************************************************
Project:    Outdoor Rotator
Module:     timers.c
Purpose:    This file contains the timer routines (because delay() sucks!)
Author:     Rudolph van Niekerk
Processor:  Arduino Uno Rev3 (ATmega328)
Compiler:	Arduino AVR Compiler


This implements timers which has to be polled to check for expiry.
The timer will act as a one-shot timer in normal operation.
To make the timer behave as a recurring timer, reload the interval and start
the timer once it has expired (using TimerStart()).

The General Timer (ST_MS_TIMER type) - 1 kHz granularity
The general Timers enables the program to create a downcounter with a
preloaded value. This timer will then decrement every 1 ms until it has
expired.
Usage:
The module making use of the timers must host a ST_MS_TIMER structure in RAM and
add it to the linked list (TimerAdd) to ensure that it is maintained.
Removing it from the linked list (TimerRemove) will  make it dormant.
The Timer must be polled (TimerPoll) to see when it has expired

 ******************************************************************************/

#define __NOT_EXTERN__
#include "timerUtils.h"
#undef __NOT_EXTERN__

#include "Arduino.h"
#include "stdUtils.h"

/*******************************************************************************
local defines
 *******************************************************************************/

/*******************************************************************************
local variables
 *******************************************************************************/
unsigned long timerUtils::time_units;
void (*timerUtils::func)();
volatile unsigned long timerUtils::count;
volatile char timerUtils::overflowing;
volatile unsigned int timerUtils::tcnt2;

/*******************************************************************************
local functions
 *******************************************************************************/
/*******************************************************************************

Constructor - Initializes the timers.

 *******************************************************************************/
/*timerUtils::timerUtils(void)
{
	//Nothing to do here....
	time_units = 0;
	func = NULL;
}*/


/*******************************************************************************

Sets the timers interval and starts it.

 *******************************************************************************/

//void timerUtils::usTimerInit(unsigned long ms, void (*f)()) {
//	timerUtils::usTimerInit(ms, 0.001, f);
//}
/*******************************************************************************

 * @param resolution
 *   0.001 implies a 1 ms (1kHz) resolution.
 *   0.0005 implies a 0.5 ms or 500us  (2kHz) resolution.
 *   0.000005 implies a 5us  (200kHz) resolution.
 *   For a 9600 baud transmission, the bit period is 104.17us, i.e. 0.00010417

 *******************************************************************************/
void timerUtils::usTimerInit(unsigned long units, double resolution, void (*f)()) {
	float prescaler = 0.0;

	if (units == 0)
		time_units = 1;
	else
		time_units = units;

	func = f;

	TIMSK2 &= ~(1<<TOIE2);
	TCCR2A &= ~((1<<WGM21) | (1<<WGM20));
	TCCR2B &= ~(1<<WGM22);
	ASSR &= ~(1<<AS2);
	TIMSK2 &= ~(1<<OCIE2A);

	if ((F_CPU >= 1000000UL) && (F_CPU <= 16000000UL)) {	// prescaler set to 64
		TCCR2B |= (1<<CS22);
		TCCR2B &= ~((1<<CS21) | (1<<CS20));
		prescaler = 64.0;
	} else if (F_CPU < 1000000UL) {	// prescaler set to 8
		TCCR2B |= (1<<CS21);
		TCCR2B &= ~((1<<CS22) | (1<<CS20));
		prescaler = 8.0;
	} else { // F_CPU > 16Mhz, prescaler set to 128
		TCCR2B |= ((1<<CS22) | (1<<CS20));
		TCCR2B &= ~(1<<CS21);
		prescaler = 128.0;
	}

	tcnt2 = 256 - (int)((float)F_CPU * resolution / prescaler);
	//	9600 baud: 		tcnt2 = 256 - (int)((float)16000000 * 0.00010417 / 64); = 229.9575 = 229
	//	19200 baud: 	tcnt2 = 256 - (int)((float)16000000 * 0.000052083 / 64); = 242.97925 = 242
	//	38400 baud: 	tcnt2 = 256 - (int)((float)16000000 * 0.0000260417 / 64); = 249.489575 = 249
	//	57600 baud: 	tcnt2 = 256 - (int)((float)16000000 * 0.000017361 / 64); = 251.65975 = 251
	//	115200 baud:	tcnt2 = 256 - (int)((float)16000000 * 0.0000086806 / 64); = 253.82985 = 253

}

void timerUtils::usTimerStart() {
	count = 0;
	overflowing = 0;
	TCNT2 = tcnt2;
	TIMSK2 |= (1<<TOIE2);
}

void timerUtils::usTimerStop() {
	TIMSK2 &= ~(1<<TOIE2);
}

void timerUtils::usTimer_overflow() {
	count += 1;

	if (count >= time_units && !overflowing) {
		overflowing = 1;
		count = count - time_units; // subtract time_uints to catch missed overflows
					// set to 0 if you don't want this.
		(*func)();
		overflowing = 0;
	}
}

ISR(TIMER2_OVF_vect) {
	TCNT2 = timerUtils::tcnt2;
	timerUtils::usTimer_overflow();
}


/*******************************************************************************

Sets the timers interval and starts it. In order for the timer to be maintained
it must be added to the linked list with TimerAdd().

 *******************************************************************************/
void timerUtils::msTimerStart(ST_MS_TIMER *t, unsigned long interval)
{

	t->Enabled = false;
	t->msExpire = millis() + interval;
	t->msPeriod = interval;
	t->Expired = false;
	t->Enabled = true;
}

/*******************************************************************************

Sets the timers interval and starts it. In order for the timer to be maintained
it must be added to the linked list with TimerAdd().

 *******************************************************************************/
bool timerUtils::msTimerReset(ST_MS_TIMER *t)
{
	if (t->msPeriod > 0)
	{
		t->Enabled = false;
		t->msExpire = millis() + t->msPeriod;
		t->Expired = false;
		t->Enabled = true;
	}
	return t->Enabled;
}
/*******************************************************************************

Stops the timer. DO NOT POLL A ST_MS_TIMER YOU HAVE STOPPED.
When polling a stopped timer, it will appear like it has not expired and it
never will expire (the Poll will return false ad infinitum) .

 *******************************************************************************/
void timerUtils::msTimerStop(ST_MS_TIMER *t)       //Pointer to the Timer Struct being stopped
{
	t->Enabled = false;
	//  t->Expired = true;
}


/*******************************************************************************

Returns true if the timer has expired, but only if it is still enabled. If a
timer has been stopped, it will always return false when polled, appearing as if
it doesn't expire.

 *******************************************************************************/
bool timerUtils::msTimerPoll(ST_MS_TIMER *t)   //Pointer to the Timer Struct which you want to poll
{
	unsigned long now_ms = millis();

	//Is the timer enabled?
	if (t->Enabled == false)
		return false;

	//Has the timer expired?
	if (!(t->Expired))
	{
		//have we moved beyond the expiry time?
		if (now_ms >= t->msExpire)
		{
			//NOW it has expired
			t->Expired = true;
		}
	}

	return(t->Expired);
}

/*******************************************************************************

Returns true if the timer is enabled.

 *******************************************************************************/
bool timerUtils::msTimerEnabled(ST_MS_TIMER *t)   //Pointer to the Timer Struct which you want to check
{
	return t->Enabled;
}

/*******************************************************************************

Returns the number of seconds since System Startup

*******************************************************************************/
unsigned long timerUtils::SecondCount(void)
{
  return (millis()/1000);
}

#undef EXT
/*************************** END OF FILE *************************************/
