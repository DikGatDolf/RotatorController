/******************************************************************************
Project:    Outdoor Rotator
Module:     utils.c
Purpose:    This file contains the std utilited
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

/*******************************************************************************
includes
 *******************************************************************************/
#define __NOT_EXTERN__
#include "appWaveGen.h"
#undef __NOT_EXTERN__

#include "stdUtils.h"
#include "timerUtils.h"
#include "devConsole.h"
#include "devMotorControl.h"

/*******************************************************************************
local defines
 *******************************************************************************/
#ifdef USE_WAV_GEN

/*******************************************************************************
local variables
 *******************************************************************************/

#ifdef CONSOLE_MENU
	ST_CONSOLE_LIST_ITEM devMenuItem_WaveGen = {NULL, "wavegen", appWaveGen::menuCmd,	"Provides access to the Wave Generator"};
#endif /* CONSOLE_MENU */
ST_WAVE_GEN waveGen;
ST_MS_TIMER waveTimer;
bool appWaveGen_InitOK = false;

/*******************************************************************************
local functions
 *******************************************************************************/
bool appWaveGen::Init(void)
{
	if (!appWaveGen_InitOK)
	{
		appWaveGen_InitOK = true;
#ifdef CONSOLE_MENU
		devConsole::addMenuItem(&devMenuItem_WaveGen);
#endif /* CONSOLE_MENU */
		//Disable the Waveform Generator
		waveGen.Type = 0;
		waveGen.Amplitude = 100.0;
		waveGen.SamplesPerPeriod = 200;	//0.5 Hz
		waveGen.SampleCount = 0;
		waveGen.Direction = true;
		timerUtils::msTimerStop(&waveTimer);
	}
	return appWaveGen_InitOK;
}

bool appWaveGen::Enabled(void)
{
	return (waveGen.Type == 0)? false : true;
}

/*******************************************************************************

 Starts The PID process

 *******************************************************************************/
void appWaveGen::Process(void) {
	if (!timerUtils::msTimerPoll(&waveTimer))
		return;

	timerUtils::msTimerReset (&waveTimer);

	//Let's do some wave generation here....
	float time_now = ((float) (millis())) / 1000.0;
	float levelDACoutput_f;
	int levelDACoutput_i;
	float position = devMotorControl::GetPosition();
	float dacSpeed = devMotorControl::GetSpeed_DAC();
	float encSpeed = devMotorControl::GetSpeed_ENC();
	float avgSpeed = devMotorControl::GetSpeed_AVG();
	switch (waveGen.Type) {
	case 1: 	//#### Sine waveform ####
		levelDACoutput_f = (0.5
				- (0.5
						* cos(
								2 * PI * waveGen.SampleCount
										/ waveGen.SamplesPerPeriod)));
		break;

	case 2:		//#### Square waveform ####
		levelDACoutput_f =
				(waveGen.SampleCount >= (waveGen.SamplesPerPeriod / 2)) ?
						0.0 : 1.0;
		break;

	case 3: 	//#### Triangle waveform ####
		levelDACoutput_f =
				1.0
						- (1
								* abs(
										1
												- ((((float )waveGen.SampleCount)
														* 2)
														/ ((float )waveGen.SamplesPerPeriod))));
		break;

	case 4:		//#### Sawtooth - Rising ####
		levelDACoutput_f = 1.0 * (((float) waveGen.SampleCount))
				/ ((float) waveGen.SamplesPerPeriod);
		break;

	case 5:		//#### Sawtooth - Falling ####
		levelDACoutput_f = 1.0
				- (1.0
						* ((((float) waveGen.SampleCount))
								/ ((float) waveGen.SamplesPerPeriod)));
		break;

	default: {
		//#### Error ####
		levelDACoutput_f = 0;
		timerUtils::msTimerStop(&waveTimer);
	}
		break;
	}

	levelDACoutput_f = waveGen.Amplitude
			* ((waveGen.Direction == MOTOR_FWD) ? 1.0 : -1.0);
	levelDACoutput_i =
			(int) (roundf(levelDACoutput_f * (((float)TLC5615_MAX_OUTPUT_VAL)/100.0) ));

	//iPrintF(trWAVEFORM, "Banner,Time,Out%%,DAC,SpdDac,SpdEnc,SpdAvg,Pos\n");
	iPrintF(trWAVEFORM, "[WAV],%s", stdUtils::floatToStr(time_now, 3));
	iPrintF(trWAVEFORM, ",%s",
			stdUtils::floatToStr(levelDACoutput_f * 100.0, 3));
	iPrintF(trWAVEFORM, ",%u", levelDACoutput_i);
	iPrintF(trWAVEFORM, ",%s", stdUtils::floatToStr(dacSpeed, 3));
	iPrintF(trWAVEFORM, ",%s", stdUtils::floatToStr(encSpeed, 3));
	iPrintF(trWAVEFORM, ",%s", stdUtils::floatToStr(avgSpeed, 3));
	iPrintF(trWAVEFORM, ",%s", stdUtils::floatToStr(position, 3));
	iPrintF(trWAVEFORM, "\n");

	devMotorControl::SetSpeed_abs(levelDACoutput_i);

	waveGen.SampleCount = (waveGen.SampleCount + 1) % waveGen.SamplesPerPeriod;
}

/*******************************************************************************

 Provides access to the Wave Generator variables through the console
 "wavegen ?" to see the options.

 *******************************************************************************/
void appWaveGen::menuCmd(void) {
	float tmpFloatVal = 0.0;
//bool returnDeg = false;
	char *valueStr1;
	char *valueStr2;
	char *valueStr3;
	int paramIndex = -1;
	bool startFlag = false;
	int retVal;
	char * paramStr;
	//PrintF("PID %s called with \"%\"\n", paramStr);

	//Get pointers to all the arguments
	paramStr = devConsole::getParam(0);
	valueStr1 = devConsole::getParam(1);
	valueStr2 = devConsole::getParam(2);
	valueStr3 = devConsole::getParam(3);

	//PrintF("PID called with \"%s\" and \"%s\"\n", paramStr, ((valueStr)? valueStr : ""));

	// Is th user "SET"ting a parameter?
	if (strcasecmp(paramStr, "Off") == NULL) {
		paramStr = "Type";
		valueStr1 = "Off";
	}

	if (strcasecmp(paramStr, "Dir") == NULL) {
		if (strcasecmp(valueStr1, "fwd") == NULL)
			waveGen.Direction = true;

		else if (strcasecmp(valueStr1, "rev") == NULL)
			waveGen.Direction = false;

		else
			PrintF(" Options for \"Dir\": [FWD|REV]\n");

		PrintF(" Dir = % 7s s\n", (waveGen.Direction) ? "FWD" : "REV");
	}

	if (strcasecmp(paramStr, "Type") == NULL) {
		if (strcasecmp(valueStr1, "Sine") == NULL)
			paramIndex = 1;
		else if (strcasecmp(valueStr1, "SQUARE") == NULL)
			paramIndex = 2;
		else if (strcasecmp(valueStr1, "Triangle") == NULL)
			paramIndex = 3;
		else if (strcasecmp(valueStr1, "SAW-Rise") == NULL)
			paramIndex = 4;
		else if (strcasecmp(valueStr1, "SAW-Fall") == NULL)
			paramIndex = 5;
		else if (strcasecmp(valueStr1, "Off") == NULL)
			paramIndex = 0;
		//else paramIndex = -1;

		if (paramIndex == 0) {
			if (waveGen.Type != 0)
				devMotorControl::SetSpeed_abs(0);

			waveGen.Type = paramIndex;
			//Stop the Waveform generator
			timerUtils::msTimerStop (&waveTimer);
			iPrintF(trWAVEFORM, "Waveform Generator Stopped\n");
			return;
		} else if (paramIndex > 0) {
			waveGen.Type = paramIndex;
			//We have a 2nd parameter value (freq)
			retVal = 0;
			if (valueStr2) {
				//We have a 2nd parameter value (freq)
				retVal = stdUtils::setFloatParam("Period", "Period", valueStr2,
						&tmpFloatVal, 0.5, 30.0);
				if (retVal == -2)
					return;
				else if (retVal >= 0) {
					waveGen.SamplesPerPeriod = (int) ((tmpFloatVal * 100.0));
					//The Frequency value is all good
					if (valueStr3) {
						//We have a 3rd parameter value (Amplitude)
						retVal = stdUtils::setFloatParam("Amp", "Amp",
								valueStr3, &waveGen.Amplitude,
								1 / ((float) TLC5615_MAX_OUTPUT_VAL), 100.0);
						if (retVal == -2)
							return;
						else if (retVal < 0)
							paramIndex = -1; //Something went wrong
						//else  The Amplitude value is all good
					}

				} else
					paramIndex = -1; //Something went wrong

			}
			if ((retVal >= 0) && (paramIndex > 0)) {
				//Start the count at 0
				waveGen.SampleCount = 0;
				//This means we can start the waveform generator with our current parameters
				iPrintF(trWAVEFORM,
						"Banner,Time,Out%%,DAC,SpdDac,SpdEnc,SpdAvg,Pos\n");
				timerUtils::msTimerStart(&waveTimer, 10ul);	//Adjust the DAC output level every 10ms
				return;
			}

		}
	}

	retVal = stdUtils::setFloatParam("Period", paramStr, valueStr1,
			&tmpFloatVal, 0.5, 30.0);
	if (retVal == -2)
		return;
	else if (retVal >= 0) {
		waveGen.SamplesPerPeriod = (int) ((tmpFloatVal * 100.0));
		paramIndex = 1;
		return;
	}

	retVal = stdUtils::setFloatParam("Amp", paramStr, valueStr1, &tmpFloatVal,
			1 / ((float) TLC5615_MAX_OUTPUT_VAL), 100.0);
	if (retVal == -2)
		return;
	else if (retVal >= 0) {
		waveGen.Amplitude = tmpFloatVal;
		paramIndex = 1;
		return;
	}

	if (strcasecmp(paramStr, "ALL") == NULL) {
		PrintF("The Waveform Generator paramaters values are:\n");
		PrintF("    Amp   : % 5s%%\n",stdUtils::floatToStr(waveGen.Amplitude, 1));
		PrintF("    Dir   : % 5s\n", (waveGen.Direction) ? "FWD" : "REV");
		PrintF("    Period: % 5s s\n",	stdUtils::floatToStr(((float )waveGen.SamplesPerPeriod) / 100.0, 2));
		PrintF("    Type  : %s \n",
				(waveGen.Type == 0) ? "OFF" : (waveGen.Type == 1) ? "SINE" :
				(waveGen.Type == 2) ? "SQUARE" :
				(waveGen.Type == 3) ? "TRIANGLE" :
				(waveGen.Type == 4) ? "SAW-RISE" :
				(waveGen.Type == 5) ? "SAW-FALL" : "ERROR");
		PrintF("           Options are: [OFF|SINE|SQUARE|TRI|SAW-RISE|SAW-FALL] \n");
/*	} else if ((paramIndex > 0) && (paramIndex <= 10)) {
		//This should just print the changed parameter
		PrintF(" *");

		//This function call is VERY dependant on what the order of the variable declerations in the struct are.
		stdUtils::setFloatParam(paramStr, paramStr, NULL,
				(float*) ((unsigned long) (&pidControl)
						+ (unsigned long) ((((unsigned long) paramIndex - 1)
								* ((unsigned long) sizeof(float))))));
*/	} else {
		PrintF("Valid commands:\n");
		PrintF("   All       - Prints the values for all parameters\n");
		PrintF("   Amp       - Amplitude % of the waveform\n");
		PrintF("   Dir       - Direction of rotation for the motor. Options: [FWD|REV]\n");
		PrintF("   Period    - Time Period of the waveform (0.5 to 30 seconds)\n");
		PrintF("   Type      - The type of waveform to generate (Rd/Wr)\n");
		PrintF("                Options: [OFF|SINE|SQUARE|TRI|SAW-RISE|SAW-FALL] \n");
	}

	PrintF("\n");
}
#endif /* USE_WAV_GEN */

#undef EXT
/*************************** END OF FILE *************************************/
