/******************************************************************************
Project:    Outdoor Rotator
Module:     utils.c
Purpose:    This file contains the std utilited
Author:     Rudolph van Niekerk
Processor:  Arduino Uno Rev3 (ATmega328)
Compiler:	Arduino AVR Compiler


Before getting your feet wet in C (sea?)
I assume you are reading the signals from some kind of analogue to digital
converter. If not then you would have to simulate the signal as an input.
If using Standard form we have,

Assuming the the loop running time is small enough (a slow process), we can
use the following function for calculating output,

output = Kp * err + (Ki * int * dt) + (Kd * der /dt);

where
	Kp = Proptional Constant.
	Ki = Integral Constant.
	Kd = Derivative Constant.
	err = Expected Output - Actual Output ie. error;
	int  = int from previous loop + err; ( i.e. integral error )
	der  = err - err from previous loop; ( i.e. differential error)
	dt = execution time of loop.

where initially 'der' and 'int' would be zero. If you use a delay function in
code to tune the loop frequency to say 1 KHz then your dt would be 0.001
seconds.

 ******************************************************************************/

/*******************************************************************************
includes
 *******************************************************************************/
#define __NOT_EXTERN__
#include "appPidControl.h"
#undef __NOT_EXTERN__

#include "stdUtils.h"
#include "timerUtils.h"
#include "devMotorControl.h"
#ifdef CONSOLE_MENU
	#include "devConsole.h"
#else
	#include "devComms.h"
#endif

/*******************************************************************************
local defines
 *******************************************************************************/
//#define PID_CSV_STREAM

#ifdef PID_CSV_STREAM
	#define PrintCsvHeaders()	iPrintF(trPIDCTRL, "[PID],Time,Pos,PosErr,Spd,dSpd,eSpd,aSpd,Bound,SpdErr,spdOut1,spdOut2,spdOut,deltaSpd\n")
#endif /* #ifdef PID_CSV_STREAM */

/*******************************************************************************
local variables
 *******************************************************************************/
#ifdef CONSOLE_MENU
ST_CONSOLE_LIST_ITEM devMenuItem_PIDCmds = {NULL, "pid", 	 	 appPidControl::menuCmd,	"Provides access to the PID control variables"};
#endif /* CONSOLE_MENU */
//ST_PID pidControl;
ST_MS_TIMER pidTimer;


#ifndef PID_CSV_STREAM
	ST_MS_TIMER pidUpdatePosTimer;
#endif /* #ifdef PID_CSV_STREAM */

const ST_PID pidControlDefault = {
		80.0,	// Kp
		0.4, 	// Ki
		2.0, 	// Kd
		0.01, 	// dt
		0, 		// Bias
		0, 		// Target
		0, 		// Position
		9.0, 	// maxAccel
		MOTOR_SPD_ABS_MAX, 	// maxSpd
		1.5, 	// minSpd
		0.0, 	// Speed
		0, 		// error
		0, 		// Int-Error
		0, 		// Der-Error
		false,	// enable
		0.0, 	// startPos
		0l, 	// startTime
		false,	// aiming
};
bool appPidControl_initOK = false;

/*******************************************************************************
local functions
 *******************************************************************************/

/*******************************************************************************

Initializes all the PID variables

 *******************************************************************************/
bool appPidControl::Init(void)
{
	if (!appPidControl_initOK)
	{

#ifdef CONSOLE_MENU
		devConsole::addMenuItem(&devMenuItem_PIDCmds);
#endif /* CONSOLE_MENU */

		timerUtils::msTimerStop(&pidTimer);
#ifndef PID_CSV_STREAM
		timerUtils::msTimerStop(&pidUpdatePosTimer);
#endif /* #ifdef PID_CSV_STREAM */
		//pidControl.Enable = true;
		memcpy(&pidSettings, &pidControlDefault, sizeof(ST_PID));
		//When we start up we can read the current position of the encoder and return to zero if we are not there.
		pidSettings.Target = 0.0;
		//We are very dependant on the Motor Controller working properly.
		appPidControl_initOK |= devMotorControl::Init();

		ControlState = stateIDLE;
	}

	return appPidControl_initOK;
}
/*******************************************************************************

Starts The PID process

 *******************************************************************************/
bool appPidControl::Enabled(void)
{
	return pidSettings.Enable;
}
/*******************************************************************************

Starts The PID process

 *******************************************************************************/
void appPidControl::Start(void)
{
	timerUtils::msTimerStart(&pidTimer, (unsigned long)(pidSettings.Period * 1000.0));
	pidSettings.startPos = devMotorControl::GetPosition();
	pidSettings.startTime = millis();
	pidSettings.aiming = true;
	pidSettings.Speed = devMotorControl::GetSpeed_DAC();
	pidSettings.intError = 0;
	pidSettings.derError = 0;
	pidSettings.error = 0;
#ifdef PID_CSV_STREAM
	PrintCsvHeaders();
#else
	timerUtils::msTimerStart(&pidUpdatePosTimer, (unsigned long)(500.0));
#endif /* #ifdef PID_CSV_STREAM */
	stdUtils::SetStatus(statusPID_BUSY);
	stdUtils::ClearStatus(statusPID_DONE);
	pidSettings.Enable = true;
}


/*******************************************************************************

Starts The PID process

 *******************************************************************************/
void appPidControl::Stop(void)
{
	pidSettings.aiming = false;
	pidSettings.Speed = devMotorControl::GetSpeed_DAC();
	stdUtils::ClearStatus(statusPID_BUSY);
	stdUtils::ClearStatus(statusPID_DONE);
	pidSettings.Enable = false;
	devMotorControl::Stop();
}

/*******************************************************************************

Starts the calibration routine

 *******************************************************************************/
bool appPidControl::GotoPos(float newPos)
{
int retVal;

	//Are we busy calibrating?
	if (ControlState != stateIDLE)
		return false;

	//Can we set the new target value
	if (stdUtils::setFloatParam(&pidSettings.Target, newPos, MOTOR_POS_WRAP_MIN, MOTOR_POS_WRAP_MAX) != 0)
		return false;

	pidSettings.Target = newPos;
	Start();

	return true;
}
/*******************************************************************************

The PID process... needs to be called repeatedly as often as possible.

 *******************************************************************************/
bool appPidControl::PID_Process(void)
{
float posError;
float spdError;
float spdBound;
float spdOutput;
float dacSpeed;
float encSpeed;
float avgSpeed;
float thisAccel;
float timeTaken;
/*

What information is really available to us?
1) Our position
2) Our set speed on the DAC (the actual speed may be too inaccurate and the average speed is delayed).

*/

//RVN - Check if the target is different from the current position by more than one pulse width. If so, turn it on???

	// Anything to do?
	if (!pidSettings.Enable)
		return false; //Nope

	pidSettings.Position = devMotorControl::GetPosition();
	pidSettings.TimeToTarget = ((float)(millis() - pidSettings.startTime))/1000.0;
	posError = pidSettings.Target - pidSettings.Position;
	dacSpeed = devMotorControl::GetSpeed_DAC();
	encSpeed = devMotorControl::GetSpeed_ENC();
	avgSpeed = devMotorControl::GetSpeed_AVG();

	if (abs(posError) < MOTOR_POS_INCREMENT_DEG) //We are within 1 full Pulse width of our target
	{
		//if ((abs(pidControl.Speed) / pidControl.Period) <= pidControl.MaxAccel)
		if (abs(encSpeed) <= pidSettings.MinSpeed)
		{
			//Woohoo, target reached!
			devMotorControl::Stop();

			if (pidSettings.aiming)
			{
				//Let the user know we are at the target!

#ifndef PID_CSV_STREAM
				iPrintF(trPIDCTRL,  "[PID] Position: %s\n",			stdUtils::floatToStr(pidSettings.Position, 3));
				timerUtils::msTimerStop(&pidUpdatePosTimer);
#endif /* #ifdef PID_CSV_STREAM */
				//Print time taken to reach destination, with offset degrees moved
				iPrintF(trALWAYS | trPIDCTRL,  "[PID]%s degs ",
						stdUtils::floatToStr(pidSettings.Target - pidSettings.startPos, 2));
				iPrintF(trALWAYS | trPIDCTRL,  "in %s s ",
						stdUtils::floatToStr(pidSettings.TimeToTarget, 3));
				iPrintF(trALWAYS | trPIDCTRL,  "(%s deg/s)\n",
						stdUtils::floatToStr(abs(pidSettings.Target - pidSettings.startPos)/pidSettings.TimeToTarget, 3));

				iPrintF(trALWAYS | trPIDCTRL,  "[PID]Stopped from %s deg/s\n",
						stdUtils::floatToStr(encSpeed, 3));

				//RVN... there is slight possibility that we have overshot our target by 1 bit...
				//Just do a check and then move back that single bit.

				/*
				iPrintF(trALWAYS | trPIDCTRL,  "Kp: %s, ",
						stdUtils::floatToStr(pidControl.Kp, 3));
				iPrintF(trALWAYS | trPIDCTRL,  "Ki: %s, ",
						stdUtils::floatToStr(pidControl.Ki, 3));
				iPrintF(trALWAYS | trPIDCTRL,  "Kd: %s\n",
						stdUtils::floatToStr(pidControl.Kd, 3));
				 */

				stdUtils::ClearStatus(statusPID_BUSY);
				stdUtils::SetStatus(statusPID_DONE);
				pidSettings.aiming = false;
				pidSettings.Enable = false;
				//devComms::readSetting("status");
			}

			//Reset the integral (accumalating) error
			pidSettings.intError = 0;

			return pidSettings.Enable;
		}
	}

	if(!timerUtils::msTimerPoll(&pidTimer))
		return pidSettings.Enable;

	timerUtils::msTimerReset(&pidTimer);

	spdBound = sqrtf(2*pidSettings.MaxAccel*abs(posError)) * sign_f(posError);
	spdError = spdBound - pidSettings.Speed;

	pidSettings.intError += (spdError * pidSettings.Period);
	pidSettings.derError = ((spdError - pidSettings.error)/pidSettings.Period);

	spdOutput = (pidSettings.Kp * spdError) +
				(pidSettings.Ki * pidSettings.intError) +
				(pidSettings.Kd * pidSettings.derError) +
				 pidSettings.bias;

	//PrintF("[PID],Time,Pos,PosErr,Spd,Bound,SpdErr,Kp,Ki,Kd,bias,Output,SpdOut\n");


#ifdef PID_CSV_STREAM
 	iPrintF(trPIDCTRL,  "[PID],%s",		stdUtils::floatToStr(pidSettings.TimeToTarget, 2));
	iPrintF(trPIDCTRL,  ",%s",			stdUtils::floatToStr(pidSettings.Position, 3));
	iPrintF(trPIDCTRL,  ",%s",			stdUtils::floatToStr(posError, 3));
	iPrintF(trPIDCTRL,  ",%s",			stdUtils::floatToStr(pidSettings.Speed, 3));
	iPrintF(trPIDCTRL,  ",%s",			stdUtils::floatToStr(dacSpeed, 3));
	iPrintF(trPIDCTRL,  ",%s",			stdUtils::floatToStr(encSpeed, 3));
	iPrintF(trPIDCTRL,  ",%s",			stdUtils::floatToStr(avgSpeed, 3));
	iPrintF(trPIDCTRL,  ",%s",			stdUtils::floatToStr(spdBound, 3));
	iPrintF(trPIDCTRL,  ",%s",			stdUtils::floatToStr(spdError, 3));
	//iPrintF(trPIDCTRL,  ",%s",		stdUtils::floatToStr(pidControl.Kp, 2));
	//iPrintF(trPIDCTRL,  ",%s",		stdUtils::floatToStr(pidControl.Ki, 2));
	//iPrintF(trPIDCTRL,  ",%s",		stdUtils::floatToStr(pidControl.Kd, 2));
	//iPrintF(trPIDCTRL,  ",%s",		stdUtils::floatToStr(pidControl.bias, 1));
	iPrintF(trPIDCTRL,  ",%s",			stdUtils::floatToStr(spdOutput, 3));

	//iPrintF(trPIDCTRL,  "[PID]% 7s: ", stdUtils::floatToStr((float)(millis()-pidControl.startTime)/1000.0, 3));
	//iPrintF(trPIDCTRL,  "% 7s\n", stdUtils::floatToStr(pidControl.Position, 3));
#else
	if(timerUtils::msTimerPoll(&pidUpdatePosTimer))
	{
		iPrintF(trPIDCTRL,  "[PID] Position: %s\n",			stdUtils::floatToStr(pidSettings.Position, 3));
		timerUtils::msTimerReset(&pidUpdatePosTimer);
	}
#endif /* #ifdef PID_CSV_STREAM */

	pidSettings.error = spdError;

	//Now we need to adjust our DAC according to the output value, but we are not allowed to exceed the abs max speed and accell
	//curSpeed = MotorControl.GetSpeed_ENC();

	//Limit the acceleration to our max accell value.
	thisAccel = ((spdOutput - pidSettings.Speed) / pidSettings.Period);
	if (abs(thisAccel)  > pidSettings.MaxAccel)
		spdOutput  = pidSettings.Speed + (sign_f(thisAccel) * pidSettings.MaxAccel * pidSettings.Period);

#ifdef PID_CSV_STREAM
	iPrintF(trPIDCTRL,  ",%s",	stdUtils::floatToStr(spdOutput, 3));
#endif /* #ifdef PID_CSV_STREAM */

	//Limit the Speed to our max speed value.
	if (abs(spdOutput) > pidSettings.MaxSpeed)
		spdOutput  = sign_f(spdOutput) * (pidSettings.MaxSpeed);

#ifdef PID_CSV_STREAM
	iPrintF(trPIDCTRL,  ",%s",	stdUtils::floatToStr(spdOutput, 3));
	iPrintF(trPIDCTRL,  ",%s",	stdUtils::floatToStr((spdOutput - pidSettings.Speed), 3));

	iPrintF(trPIDCTRL,  "\n");
#endif /* #ifdef PID_CSV_STREAM */

	pidSettings.aiming = true;
	pidSettings.Speed = spdOutput;

	devMotorControl::SetSpeed_degs(spdOutput);

	return pidSettings.Enable;

}

/*******************************************************************************

 Display program version info

 *******************************************************************************/
bool appPidControl::ControlStateHandler(void)
{

//float realPos, pos, offset, spd;
float offset;

	switch (ControlState)
	{
		case stateIDLE:
			//Chill out... or search for your target... whatever, dude.
			break;

		case stateCAL_SEARCH:
			// We are busy searching for that ever elusive zero pulse.
			//realPos = devMotorControl::GetRealPosition();
			offset = devMotorControl::GetZeroOffset();
			if (abs(offset) < 361.0)
			{
				//spd = devMotorControl::GetSpeed_ENC();
				//pos = devMotorControl::GetPosition();
				//PrintF("CAL - Go to 0 (%s) from ", stdUtils::floatToStr(offset, 2));
				//PrintF("%s (", stdUtils::floatToStr(realPos, 2));
				//PrintF("%s) ", stdUtils::floatToStr(pos, 2));
				//PrintF(". Current Speed: %s deg/s!\n", stdUtils::floatToStr(spd , 2));
				//If we know where the real 0 is.. we rather want to move in that direction
				appPidControl::pidSettings.Target = offset;//realPos * (-1.0);
				ControlState = stateCAL_GOTO_0;
			}
			break;

		case stateCAL_GOTO_0:
			//Are we there yet?
			if ((stdUtils::GetStatus(statusPID_DONE)) && (!stdUtils::GetStatus(statusPID_BUSY)))
			{
				//PrintF("CAL - Zero reached?: ");
				//Are we really really there yet... We should be seeing a high on the "zero" pin
			    if (devMotorControl::IsAtRealZero())
			    {
			    	//Whoop-Whoop... we can return back to normal.
			    	devMotorControl::SetPosition(0.0);
					stdUtils::ClearStatus(statusCALIB_BUSY);
					ControlState = stateIDLE;
					//PrintF("Yes!\n");
					//devComms::readSetting("status");
			    }
			    else
			    {
					//realPos = devMotorControl::GetRealPosition();
			    	//If we are not at the real zero... something is not completely right... let's start again?
					//PrintF("No (%s)!\n", stdUtils::floatToStr(realPos, 2));
					appPidControl::pidSettings.Target = 360.0;
					ControlState = stateCAL_SEARCH;
					appPidControl::Start();
			    }
			}
			//else //We keep going until we reach the target.

			break;
		default:
			break;
	}

	//Do the PID dance
	return appPidControl::PID_Process();
}

/*******************************************************************************

Starts the calibration routine

 *******************************************************************************/
void appPidControl::StartCalibration(void)
{
	//PrintF("CAL - Start!\n");

	ControlState = stateCAL_SEARCH;
	stdUtils::SetStatus(statusCALIB_BUSY);
	appPidControl::pidSettings.Target = 360.0;
	appPidControl::Start();
	//devComms::readSetting("status");
}
/*******************************************************************************

Provides access to the PID controller variables/constantsthrough the console
"pid ?" to see the options.

 *******************************************************************************/
#ifdef CONSOLE_MENU
void appPidControl::menuCmd(void)
{
char * paramStr;
char *valueStr;
int paramIndex = -1;
int retVal;

//PrintF("PID %s called with \"%\"\n", paramStr);

	//PrintF("PID called with \"%s\" and \"%s\"\n", paramStr, ((valueStr)? valueStr : ""));

	// Is th user "SET"ting a parameter?
	paramStr = devConsole::getParam(0);
	valueStr = devConsole::getParam(1);

	if (strcasecmp(paramStr, "ON") == NULL)
	{
		Start();
		return;
	}

	if (strcasecmp(paramStr, "OFF") == NULL)
	{
		paramIndex = 11;
		if (pidSettings.Enable)
		{
			pidSettings.aiming = false;
			pidSettings.Speed = devMotorControl::GetSpeed_DAC();
			pidSettings.intError = 0;
			pidSettings.derError = 0;
			pidSettings.error = 0;
			stdUtils::ClearStatus(statusPID_BUSY);
			stdUtils::ClearStatus(statusPID_DONE);
			pidSettings.Enable = false;
			devMotorControl::Stop();
		}
		paramStr = "ALL";
	}

	if (strcasecmp(paramStr, "Default") == NULL)
	{
		if (!valueStr)
		{
			memcpy(&pidSettings, &pidControlDefault, sizeof(ST_PID));
			paramStr = "ALL";
		}
	}

	if (strcasecmp(paramStr, "Target") == NULL)
	{
		retVal = stdUtils::setFloatParam("Target", paramStr, valueStr, &pidSettings.Target, MOTOR_POS_WRAP_MIN, MOTOR_POS_WRAP_MAX);
		if (retVal == -2) 		return;
		else if (retVal >= 0)
		{
			PrintF(" * Target : % 7s deg\n", stdUtils::floatToStr(pidSettings.Target, 2));
			Start();
			return;
		}
	}

	retVal = stdUtils::setFloatParam("Kp", paramStr, valueStr, &pidSettings.Kp);
	if (retVal == -2) 		return;
	else if (retVal >= 0)	paramIndex = 1;

	retVal = stdUtils::setFloatParam("Ki", paramStr, valueStr, &pidSettings.Ki);
	if (retVal == -2) 		return;
	else if (retVal >= 0)	paramIndex = 2;

	retVal = stdUtils::setFloatParam("Kd", paramStr, valueStr, &pidSettings.Kd);
	if (retVal == -2) 		return;
	else if (retVal >= 0)	paramIndex = 3;

	retVal = stdUtils::setFloatParam("Period", paramStr, valueStr, &pidSettings.Period, 0.01, 1.0);
	if (retVal == -2) 		return;
	else if (retVal >= 0)	paramIndex = 4;

	retVal = stdUtils::setFloatParam("MaxAcc", paramStr, valueStr, &pidSettings.MaxAccel, 0.0, 72.0);
	if (retVal == -2) 		return;
	else if (retVal >= 0)	paramIndex = 8;

	retVal = stdUtils::setFloatParam("MaxSpd", paramStr, valueStr, &pidSettings.MaxSpeed, 0.0, MOTOR_SPD_ABS_MAX);
	if (retVal == -2) 		return;
	else if (retVal >= 0)	paramIndex = 9;

	retVal = stdUtils::setFloatParam("MinSpd", paramStr, valueStr, &pidSettings.MinSpeed, MOTOR_SPD_ABS_MIN, pidSettings.MaxSpeed);
	if (retVal == -2) 		return;
	else if (retVal >= 0)	paramIndex = 10;

	retVal = stdUtils::setFloatParam("Bias", paramStr, valueStr, &pidSettings.bias, 0.0, 1.0);
	if (retVal == -2) 		return;
	else if (retVal >= 0)	paramIndex = 5;


	if (strcasecmp(paramStr, "ALL") == NULL)
	{
		PrintF("The PID paramater values are:\n");
		PrintF(" Kp    : % 7s\n", stdUtils::floatToStr(pidSettings.Kp, 3));
		PrintF(" Ki    : % 7s\n", stdUtils::floatToStr(pidSettings.Ki, 3));
		PrintF(" Kd    : % 7s\n", stdUtils::floatToStr(pidSettings.Kd, 3));
		PrintF(" dt    : % 7s s\n", stdUtils::floatToStr(pidSettings.Period, 3));
		PrintF(" Bias  : % 7s\n", stdUtils::floatToStr(pidSettings.bias, 3));
		PrintF(" Target: % 7s degs\n", stdUtils::floatToStr(pidSettings.Target, 3));
		PrintF(" Pos   : % 7s degs\n", stdUtils::floatToStr(devMotorControl::GetPosition(), 3));
		PrintF(" MinSpd: % 7s deg/s\n", stdUtils::floatToStr(pidSettings.MinSpeed, 3));
		PrintF(" MaxSpd: % 7s deg/s\n", stdUtils::floatToStr(pidSettings.MaxSpeed, 3));
		PrintF(" MaxAcc: % 7s deg/s/s\n", stdUtils::floatToStr(pidSettings.MaxAccel, 3));
		PrintF(" State : % 7s\n", (pidSettings.Enable)? "ON" : "OFF");
	}
	else if ((paramIndex > 0) && (paramIndex <= 10))
	{
		//This should just print the changed parameter
		PrintF(" *");

		//This function call is VERY dependant on what the order of the variable declerations in the struct are.
		stdUtils::setFloatParam(paramStr, paramStr, NULL, (float*)((unsigned long)(&pidSettings) + (unsigned long)((((unsigned long)paramIndex-1)*((unsigned long)sizeof(float))))));
	}
	else if (paramIndex == 11)
	{
		PrintF(" * Enable : % 7s\n", (pidSettings.Enable)? "ON" : "OFF");
	}
	else
	{
		PrintF("Valid commands:\n");
		PrintF("    Kp     - Proptional Constant\n");
		PrintF("    Ki     - Integral Constant\n");
		PrintF("    Kd     - Derivative Constant\n");
		PrintF("    dt     - Execution Period\n");
		PrintF("    Bias   - Small biasing constant (<1.0)\n");
		PrintF("    Target - The target position\n");
		PrintF("    MinSpd - Min absolute speed deg/s\n");
		PrintF("    MaxSpd - Max absolute speed deg/s\n");
		PrintF("    MaxAcc - Max absolute acceleration\n");
		PrintF("    ON/OFF - Enable/Disable\n");
		PrintF("    Default- Set Default Values\n");
	}

	PrintF("\n");
}
#endif /* CONSOLE_MENU */


#undef EXT
/*************************** END OF FILE *************************************/

