/*******************************************************************************

Project:    Outdoor Rotator
Module:     devMotorSpeedDrive.cpp
Purpose:    This file contains the Motor Speed Drive Controller
Author:     Rudolph van Niekerk
Processor:  Arduino Uno Rev3 (ATmega328)
Compiler:	Arduino AVR Compiler

This series of speed controller is a low voltage DC four-quadrant regeneration
pulse width controller adopts special single-chip intelligent control system and
has rapid response speed, steady operation, reliable work status and multiple
protection functions.

Description of control terminal's functions:

EN (& EN_COM) - Connection of EN to EN_COM enables operation of the motor
		This wilhl be controlled by switching a relay.

DIR (& DIR_COM) - Connection of DIR to DIR_COM reverses direction of the motor
		This will be controlled by switching a relay.

C & E - When over-current is detected on the controller, over-current signal will
		be immediately sent to diode of opto-coupler for break over and then
		transmitted to C, E ports.

S1, S2, S3 - Analog control of Motor speed. This will be fed controlled through
 	 	 a DAC (TLC5615) which output is connected to S2 and S3 to ground.
 	 	 The output of te TLC5615 is set via commands over an SPI bus.

POSITION - The motor final output drive's angular position is given by an
		absolute encoder (AMT203) which provides 1024 ppr output on
		quadrature lines (A, B and Zero Point) or alternatively a 12-bit
		resolution position (4006 ppr) can be read over the SPI bus.

		Quadrature signals with index showing counter-clockwise rotation:
			 	 	 __
		X	________|  |_____________________________
	  	  	  ____      ____      ____      ____
		A	_|    |____|    |____|    |____|    |____
	     	 	 ____      ____      ____      ____
		B	____|    |____|    |____|    |____|    |_

		A leads B by 90 degrees phase angle for CCW rotation (viewed from front)
		X = Zero Offset (once per rotation) pulse width is 1/2 pulse width of
		A & B Outputs A/B (and Z) can be used for tracking. These outputs are
		operational up to 8000 RPM without speed error.

		The maximum speed of final output drive is 6RPM (36 deg/s)

		---------------------------------------------------------------
		NB!!! : The AMT203's SPI could not be successfully implemented.
		---------------------------------------------------------------

 The angular speed of the motor should be controlled according to the following states
 ^ Speed
 |				   __________________________
 |				 /!							 !\
 |				/ !							 ! \
 |			   /  !							 !  \
 |			  /	  !							 !	 \
 |			 /	  !							 !	  \
 |			/	  !							 !	   \
 |		   /	  !							 !		\
 |		  /		  !							 !		 \
 |		 /		  !							 !		  \
 |______/_________!__________________________!_________\_________ Time
		!         !                          !         !\_/!     !
		!         !                          !         !   !     !
STATE 0	!    1    !            2             !    3    ! 4 !  0  !

	0: IDLE - No movement
	1: ACCELERATE - Incremental acceleration up to Max Speed
	2: CONSTANT - Constant (max) speed maintained
	3: DECELARATE - Incremental decelaration to zero speed
	4: OVERSHOOT - Slight reverse to target in case of overshoot.

 *******************************************************************************/

/*******************************************************************************
includes
 *******************************************************************************/
#define __NOT_EXTERN__
#include "devMotorControl.h"
#undef __NOT_EXTERN__

#include "Arduino.h"
#include "stdUtils.h"
#ifdef CONSOLE_MENU
	#include "devConsole.h"
#else
	#include "devComms.h"
#endif
#include "version.h"
#include "timerUtils.h"

/*******************************************************************************
local defines
 *******************************************************************************/
#define AMT203_DEBOUNCE_PERIOD		0.0002 /* 200us */
#define AMT203_PULSEWIDTH_ARR_CNT	20	/* The size of the Array for averaging the speed */
#define AMT203_DEBOUNCE_CNT			2	/* This will ensure that we have a
											delay of no more than 2 x Debounce_Period */
#define UNKNOWNPOS		-11377 /* as close to -999.9 deg as we can get */


/*
 * These equations were calculted through extensive experimentation
 * (see the "OTHER" folder)
 *  The linearity is denoted by the equation y = Mx + C and
 *  it is slightly different for the forward and reverse rotation.
 *  Turning Direction	Transfer Equation		Inverse (Adjustment)		MIN Limit		MAX Limit
 *  Clockwise:			y = 1.1598x - 1.1071	x = (y + 1.1071)/1.1598		0.954561131		31.99439559
 *  Counter-Clockwise:	y = 1.1417x + 1.3754	x = (y - 1.3754)/1.1417)	-1.204694753	-32.73662083
*/

#define ROTATE_NONE		0	/* No Rotation determined */


//#define SetOffsetWithinLimit(z)	((z >= (AMT203_QUAD_PPR + AMT203_QUAD_PPR_NEG_OFFSET))? (z - AMT203_QUAD_PPR) : ((z < AMT203_QUAD_PPR_NEG_OFFSET)? (z + AMT203_QUAD_PPR) : z))
#define SetOffsetWithinLimit(z)	((z < AMT203_QUAD_PPR_NEG_OFFSET)? (z + AMT203_QUAD_PPR) : ((z >= (AMT203_QUAD_PPR + AMT203_QUAD_PPR_NEG_OFFSET))? (z - AMT203_QUAD_PPR) : z ))

/*******************************************************************************
local function prototypes
 *******************************************************************************/

/*******************************************************************************
local structure
 *******************************************************************************/
#ifdef CONSOLE_MENU
ST_CONSOLE_LIST_ITEM devMenuItem_speed 		= {NULL, "motor", 	devMotorControl::menuCmd,	"GET/SET motor control variables."};
#endif /* CONSOLE_MENU */

/*******************************************************************************
local variables
 *******************************************************************************/
volatile bool debugPin2Level;
volatile bool debugPin3Level;

ST_MS_TIMER printTraceTmr;

bool devMotorControl_initOK = false;
const PROGMEM int _EN = 7;
const PROGMEM int _REV = 8;
const PROGMEM int _OC = 9;
const PROGMEM int _A = 2;
const PROGMEM int _B = 4;
const PROGMEM int _X = 3;
char devMotorControl_tag[6];

//************ Variables for compensation of Non-linearity of Motor ************

bool _reversing;
bool _enabled;

//************ Variables for AMT203's input/calculations ************
ST_PIN_DEBOUNCE debounceInput_A;
ST_PIN_DEBOUNCE debounceInput_B;
ST_PIN_DEBOUNCE debounceInput_X;

volatile int _direction;
volatile int _zero_offset;
volatile int _position;
volatile bool _ZeroOffsetIsKnown;

volatile unsigned long _lastEdge_us;
volatile unsigned long _now_us;
volatile unsigned long _pulsePeriod_us;	//Will roll over if the pulse width is more than 71 minutes

volatile unsigned long _pulsePeriod_us_arr[AMT203_PULSEWIDTH_ARR_CNT];
volatile unsigned long _pulsePeriod_us_cnt;

/*******************************************************************************
functions
 *******************************************************************************/

/*******************************************************************************

Initialises the Motor Controller

 *******************************************************************************/
bool devMotorControl::Init(void)
{
	if (devMotorControl_initOK)
		return true;

	strncpy(devMotorControl_tag, "[MOT]", 5);
	devMotorControl_tag[5] = 0;

    //Set the Pin configurations (Input/Output and Interrupt driven)
	pinMode(_EN, OUTPUT);
	pinMode(_REV, OUTPUT);
	pinMode(_OC, INPUT);

	//Might as well start in a DISABLED state...
	stdUtils::ClearStatus(statusMOVING);
	stdUtils::quickPinToggle(_EN, LOW);
    //onState = false;
    _enabled = false;

	//Start in a forward direction...
	stdUtils::ClearStatus(statusDIRECTION);
	stdUtils::quickPinToggle(_REV, LOW);
    _reversing = false;

	pinMode(_A, INPUT);
	pinMode(_B, INPUT);
	pinMode(_X, INPUT);

	stdUtils::quickPinToggle(_A, HIGH);	// activate internal pull-up resistor (when set up as input)
	//digitalWrite(_A, HIGH);	// activate internal pull-up resistor (when set up as input)
	stdUtils::quickPinToggle(_B, HIGH);	// activate internal pull-up resistor (when set up as input)
	//digitalWrite(_B, HIGH);	// activate internal pull-up resistor (when set up as input)
	stdUtils::quickPinToggle(_X, HIGH);	// activate internal pull-up resistor (when set up as input)
	//digitalWrite(_X, HIGH);	// activate internal pull-up resistor (when set up as input)

    //Save the startup states of these pins so we can start debouncing immediately.
	debounceInput_A.DebounceCount = 0;
	debounceInput_B.DebounceCount = 0;
	debounceInput_X.DebounceCount = 0;

	debounceInput_A.CurrentState = LOW;
	debounceInput_B.CurrentState = LOW;
	debounceInput_X.CurrentState = LOW;

	Xfer.Pos.M = XFER_EQ_POS_M;
	Xfer.Pos.C = XFER_EQ_POS_C;
	Xfer.Neg.M = XFER_EQ_NEG_M;
	Xfer.Neg.C = XFER_EQ_NEG_C;

	debugPin2Level = false;
	debugPin3Level = false;
	stdUtils::quickPinToggle(pinDEBUG_2, debugPin2Level);
	stdUtils::quickPinToggle(pinDEBUG_3, debugPin3Level);

	// initialize the DAC
	devMotorControl_initOK |= halTLC5615::Init();

	if (!devMotorControl_initOK)
	{
		iPrintF(trALWAYS | trMOTOR, "DAC init Failed\n", devMotorControl_tag);
		return false;
	}

	//We are going to check the input pins (_A, _B and _X) at 200us interval (5 kHz)
	// in a Timer Interrupt, because we need to debounce the signal.
	timerUtils::usTimerInit(0.0002, AMT203_DEBOUNCE_PERIOD, devMotorControl::TimerInterruptCallback);

	//Now we can start checking our input signals...
	timerUtils::usTimerStart();

	// Wait for startup time (100ms) required by the encoder...
	delay(100);

	//Assume we start at position 0
	_position = 0;
	_zero_offset = 0;

	//If we are at 0, then all is well...
	if (stdUtils::quickPinRead(_X) == HIGH)
		_ZeroOffsetIsKnown = true;
	else //if not, then we do not really have any idea what our offset is.... yet
		_ZeroOffsetIsKnown = false;

#ifdef CONSOLE_MENU
	devConsole::addMenuItem(devMotorControl_tag, &devMenuItem_speed);
#endif /* CONSOLE_MENU */

	//Start at 0, wherever we are now.
	devMotorControl::SetPosition(0.0);
	devMotorControl::Stop();

	// Initialize the use of our custom-made timers (running on Timer1)
	timerUtils::msTimerStart(&printTraceTmr, 1000L);	//Print the Speed and Position every 1000ms

	iPrintF(trMOTOR | trALWAYS, "%sInit OK\n", devMotorControl_tag);

	return devMotorControl_initOK;
}

/*******************************************************************************

	/* The speed output is not perfectly linear, so we are going to apply a conversion
	 * here:
	 * 				  Output (deg/s)
	 * 						 |
	 * 						 |           /
	 * 						 |          /
	 * 						 |         /
	 * 						 |        /
	 * 						 |       /
	 * 						 |      /
	 * 						 |     /
	 *  					 |    /
	 *  					 |   /
	 * ----------------------0-------------------- Input (deg/s)
	 *  				 /   |
	 *   			    /    |
	 * 				   /     |
	 * 				  /      |
	 * 			     /       |
	 * 			    /	     |
	 * 			   /   	     |
	 * 			  /      	 |
	 * 			 /      	 |
	 * 						 |
	 *
	 *
	 * These equations were calculted through extensive experimentation
	 * (see the "OTHER" folder)
	 *  The the linearity is denoted by the equation y = Mx + C and
	 *  it is slightly different for the forward and reverse rotation.
	 *  Turning Direction	Transfer Equation		Inverse (Adjustment)		MIN Limit		MAX Limit
	 *  Positive:			y = 1.1598x - 1.1071	x = (y + 1.1071)/1.1598		0.954561131		31.99439559
	 *  Negative:			y = 1.1417x + 1.3754	x = (y - 1.3754)/1.1417)	-1.204694753	-32.73662083
	 *
 *******************************************************************************/
float devMotorControl::ConvertSpeed_WR_degs(float spdDegreePerSecond)
{
float spdDegreePerSecond_adjust;

	if (spdDegreePerSecond > 0.0)
		spdDegreePerSecond_adjust = (spdDegreePerSecond - Xfer.Pos.C) / Xfer.Pos.M;

	else if (spdDegreePerSecond < 0.0)
		spdDegreePerSecond_adjust = (spdDegreePerSecond - Xfer.Neg.C) / Xfer.Neg.M;

	else
		spdDegreePerSecond_adjust = spdDegreePerSecond;
/*
	iPrintF(trMOTOR, "%sWR Speed Xfer %s => ",
			_tag,
			stdUtils::floatToStr(spdDegreePerSecond, 3));
	iPrintF(trMOTOR, "%s deg/s\n",
			stdUtils::floatToStr(spdDegreePerSecond_adjust, 3));
*/
	return spdDegreePerSecond_adjust;
}

/*******************************************************************************
	/* The speed output is not perfectly linear, so we are going to apply a conversion
	 * here:
	 * 				  Output (deg/s)
	 * 						 |
	 * 						 |           /
	 * 						 |          /
	 * 						 |         /
	 * 						 |        /
	 * 						 |       /
	 * 						 |      /
	 * 						 |     /
	 *  					 |    /
	 *  					 |   /
	 * ----------------------0-------------------- Input (deg/s)
	 *  				 /   |
	 *   			    /    |
	 * 				   /     |
	 * 				  /      |
	 * 			     /       |
	 * 			    /	     |
	 * 			   /   	     |
	 * 			  /      	 |
	 * 			 /      	 |
	 * 						 |
	 *
	 *
 *******************************************************************************/
float devMotorControl::ConvertSpeed_RD_degs(float spdDegreePerSecond)
{
float spdDegreePerSecond_adjust;

	if (spdDegreePerSecond > 0.0)
		spdDegreePerSecond_adjust = (spdDegreePerSecond * Xfer.Pos.M) + Xfer.Pos.C;

	else if (spdDegreePerSecond < 0.0)
		spdDegreePerSecond_adjust = (spdDegreePerSecond * Xfer.Neg.M) + Xfer.Neg.C;

	else
		spdDegreePerSecond_adjust = spdDegreePerSecond;

	return spdDegreePerSecond_adjust;

}
/*******************************************************************************

Sets the output level of the the DAC to 0. Do not call this function if the
speed of the motor is too high. If it is turning a heavy object the angular
momentum could cause damage.

 *******************************************************************************/
void devMotorControl::Stop(void)
{
	devMotorControl::SetSpeed_abs(0);
	devMotorControl::ResetSpeedParams();
	iPrintF(trMOTOR, "%sMotor Stopped\n", devMotorControl_tag);
	stdUtils::ClearStatus(statusMOVING);
}
/*******************************************************************************

Resets all the speed variables of the motor controller. This should be done
before starting rotation, or when we are sure the rotation has stopped.

 *******************************************************************************/
void devMotorControl::ResetSpeedParams(void)
{
    _direction = ROTATE_NONE;//This could be dodgy... should only be done if the speed is 0
    _now_us = micros();
    _lastEdge_us = _now_us;
    _pulsePeriod_us = 0l; //This insures that the startup speed is 0

    //_pulsePeriod_us_arr[AMT203_PULSEWIDTH_AVG];
    _pulsePeriod_us_cnt = 0;

    for (int i = 0; i < AMT203_PULSEWIDTH_ARR_CNT; i++)
    	_pulsePeriod_us_arr[i] = 0l;

}
/*******************************************************************************

Takes the speed as input (-36.0 to 36.0)
Sets the output level of the the DAC as well as the reverse
input (if the speed percentage is negative)

 *******************************************************************************/
float devMotorControl::SetSpeed_degs(float spdDegreePerSecond)
{
float spdDegreePerSecond_adjust;

	if (spdDegreePerSecond == 0.0)
		return (float)devMotorControl::SetSpeed_abs(0);

	else if (abs(spdDegreePerSecond) > MOTOR_SPD_ABS_MAX)
		spdDegreePerSecond = (spdDegreePerSecond < 0)? -MOTOR_SPD_ABS_MAX : MOTOR_SPD_ABS_MAX;

	else if (abs(spdDegreePerSecond) < MOTOR_SPD_ABS_MIN)
		spdDegreePerSecond = (spdDegreePerSecond < 0)? -MOTOR_SPD_ABS_MIN : MOTOR_SPD_ABS_MIN;

	return (float)devMotorControl::SetSpeed_abs(
			(int)(ConvertSpeed_WR_degs(spdDegreePerSecond) / MOTOR_SPD_INCREMENT_FLT)
			) * MOTOR_SPD_INCREMENT_FLT;
}

/*******************************************************************************

Takes the absolute DAC level as input
Sets the output level of the the DAC as well as the reverse output

 *******************************************************************************/
int devMotorControl::SetSpeed_abs(int spdAbsolute)
{

	if (abs(spdAbsolute) > TLC5615_MAX_OUTPUT_VAL)
		spdAbsolute = (spdAbsolute < 0)? -TLC5615_MAX_OUTPUT_VAL : TLC5615_MAX_OUTPUT_VAL;

	//Enable the motor if we are going to make it work!
	_enabled = (spdAbsolute != 0)? true : false;

	stdUtils::ToggleStatus(statusMOVING, _enabled);
	stdUtils::quickPinToggle(_EN, (_enabled)? HIGH : LOW);

	//Set the direction output in the correct state (if it isn't in the correct state already).
	_reversing = (spdAbsolute < 0)? MOTOR_REV : MOTOR_FWD;

	//iPrintF(trMOTOR, "%sSet Abs: %d (Reversing: %s)\n", _tag, spdAbsolute, ((_reversing)? "Y" : "N"));

	stdUtils::ToggleStatus(statusDIRECTION, _reversing);
	stdUtils::quickPinToggle(_REV, (_reversing)? HIGH : LOW);

	//Now set the DAC output to the correct value.
	return (int)halTLC5615::SetLevel((unsigned int)abs(spdAbsolute)) * ((_reversing)? ROTATE_FORWARD : ROTATE_BACKWARD);

}

/*******************************************************************************

Sets the Enable Output pin LOW (called from interrupt)
Should also send a flag to tell the system that the motor was stopped (from the
interrupt)

 *******************************************************************************/
void devMotorControl::KillMotor(void)
{
	stdUtils::ClearStatus(statusMOVING);
	stdUtils::quickPinToggle(_EN, LOW);
}

/*******************************************************************************

Returns the motor speed in deg/s as read from the shaft encoder over the last
10 pulses received from the encoder

 *******************************************************************************/
float devMotorControl::GetSpeed_AVG(void)
{
	float tmpOmega = 0.0;

	// NOTE: The recorded pulse period could be in the order of 585937 us long (@ 0.1 RPM)

	//Have we picked up any speed?
	if (_pulsePeriod_us > 0l)
	{
		//We need to convert the pulse width of the quadrature input(s) into
		// angular velocity here. We know that we have 1024 pulses per rotation.
		// Omega = (60 seconds x 10^6 x 360)/(Pulse_count_per_rotation x Tp_us x 60)
		// Omega = (360 x 10^6)/(Pulse_count_per_rotation x Tp_us)

		//averageFloat(_pulsePeriod_us_arr, AMT203_PULSEWIDTH_AVG);
	  //tmpOmega = (360000000.0/AMT203_QUAD_PPR)/_pulsePeriod_us;
		tmpOmega = (360000000.0/AMT203_QUAD_PPR)/(stdUtils::avgULong(_pulsePeriod_us_arr, AMT203_PULSEWIDTH_ARR_CNT));

	}
	return tmpOmega * ((_direction == ROTATE_BACKWARD)? ROTATE_BACKWARD : ROTATE_FORWARD);
}

/*******************************************************************************

Returns the motor speed in deg/s as read from the shaft encoder

 *******************************************************************************/
float devMotorControl::GetSpeed_ENC(void)
{
float tmpOmega = 0.0;

	// NOTE: The recorded pulse period could be in the order of 585937 us long (@ 0.1 RPM or 0.6 deg/s)

	//Have we picked up any speed?
	if (_pulsePeriod_us > 0l)
	{
		//We need to convert the pulse width of the quadrature input(s) into
		// angular velocity here. We know that we have 1024 pulses per rotation.
		// Omega = (60 seconds x 10^6 x 360)/(Pulse_count_per_rotation x Tp_us x 60)
		// Omega = (360 x 10^6)/(Pulse_count_per_rotation x Tp_us)

		tmpOmega = (360000000.0/AMT203_QUAD_PPR)/_pulsePeriod_us;

	}

	return tmpOmega * ((_direction == ROTATE_BACKWARD)? ROTATE_BACKWARD : ROTATE_FORWARD);
}

/*******************************************************************************

Returns the motor speed in deg/s as set on the DAC

 *******************************************************************************/
float devMotorControl::GetSpeed_DAC(void)
{
	return ConvertSpeed_RD_degs(((float)halTLC5615::GetLevel_abs()) *  MOTOR_SPD_INCREMENT_FLT * ((_reversing)? ROTATE_BACKWARD : ROTATE_FORWARD));
}

/*******************************************************************************

Returns the motor position in degs

 *******************************************************************************/
float devMotorControl::GetPosition(void)
{
	return (((float)_position) * MOTOR_POS_INCREMENT_DEG);
}

/*******************************************************************************

Sets the motor Postion in deg as set on the DAC

 *******************************************************************************/
float devMotorControl::SetPosition(float newPos)
{
int realPosition;
	if (devMotorControl::GetSpeed_DAC() != 0)
	{
		iPrintF(trMOTOR | trALWAYS,
				"%sCannot set Position when the Motor Speed (%s) != 0 deg/s\n",
				devMotorControl_tag,
				stdUtils::floatToStr(devMotorControl::GetSpeed_DAC(), 2));
	}
	else if (_ZeroOffsetIsKnown)
	{
		//Now we need to set our new position and recalculate our offset

		//Our "real" position (wrt the correct 0) is RealPos = X_current - Z_current
		realPosition = (_position - _zero_offset)%AMT203_QUAD_PPR;

		//But we want to keep our real position in the range of -180 to 180 degrees.
		realPosition = SetOffsetWithinLimit(realPosition);

		//We know our "real" position... so we can set our "new" position
		_position = (int)(roundf(newPos/MOTOR_POS_INCREMENT_DEG));
		//_position = (int)(newPos/MOTOR_POS_INCREMENT_DEG);

		//A "new" position will have a "new" offset, but that should remain
		// at the same "real" position: i.e. RealPos = X_new - Z_new
		// Thus: Z_new = X_new - RealPos
		_zero_offset = (_position - realPosition)%AMT203_QUAD_PPR;

		//Now we want our new offset to be somewhere between -180 and 180 degrees.
		_zero_offset = SetOffsetWithinLimit(_zero_offset);
	}
	else //if (!_ZeroOffsetIsKnown)
	{
		//Great, we can just set it...
		_position = (int)(newPos/MOTOR_POS_INCREMENT_DEG);
	}

	return (((float)_position) * MOTOR_POS_INCREMENT_DEG);;
}
/*******************************************************************************

Returns the motor speed in deg/s as set on the DAC

 *******************************************************************************/
float devMotorControl::GetRealPosition(void)
{
int tmpPos = UNKNOWNPOS;
	//return _position;
	if (_ZeroOffsetIsKnown)
		tmpPos = _position - _zero_offset;

	return (((float)tmpPos) * MOTOR_POS_INCREMENT_DEG);
}

/*******************************************************************************

Returns the motor speed in deg/s as set on the DAC

 *******************************************************************************/
float devMotorControl::GetZeroOffset(void)
{
int tmpPos = UNKNOWNPOS;
	//return _position;
	if (_ZeroOffsetIsKnown)
		tmpPos = _zero_offset;

	return (((float)tmpPos) * MOTOR_POS_INCREMENT_DEG);
}
/*******************************************************************************

Returns the motor speed in deg/s as set on the DAC

 *******************************************************************************/
bool devMotorControl::IsAtRealZero(void)
{
	//If we are at 0, then all is well...
	return (stdUtils::quickPinRead(_X) == HIGH) ? true : false;
}

/*******************************************************************************

Timer Interrupt callback for the Encoder Input Trigger
This callback happens about everty 100us, so do not f@#$ about in here....

... get you sh!t done and get out.

And for all that is good in this earth, do not call PrintF in here!!!!

A and B both have 1024 PPR with a 50% duty cycle and a 90 degree phase offset
between A and B.
             __
X	________|  |_____________________________
	  ____      ____      ____      ____
A	_|    |____|    |____|    |____|    |____
	     ____      ____      ____      ____
B	____|    |____|    |____|    |____|    |_

This checks for rising AND falling edges on A AND B which effectively gives us a
resolution of 4096

 *******************************************************************************/
void devMotorControl::TimerInterruptCallback()
{
//int pinA_lvl, pinB_lvl, pinX_lvl;
int pinA_edge, pinB_edge, pinX_edge;

/*	we are toggling debug pin 0 to get an idea of how long the timer interrupt
	takes to execute */
	stdUtils::quickPinToggle(pinDEBUG_0, true);

	//int pinA_lvl, pinB_lvl, pinX_lvl;


	pinA_edge = stdUtils::debounceInput(&debounceInput_A, stdUtils::quickPinRead(_A), AMT203_DEBOUNCE_CNT);
	pinB_edge = stdUtils::debounceInput(&debounceInput_B, stdUtils::quickPinRead(_B), AMT203_DEBOUNCE_CNT);
	pinX_edge = stdUtils::debounceInput(&debounceInput_X, stdUtils::quickPinRead(_X), AMT203_DEBOUNCE_CNT);

	//Rising or falling edge on A and B should now be handled
	if (pinA_edge || pinB_edge) //&& _PositionIsKnown)
	{
		//Get the time of the edge on input A
		//I know I am not supposed to call this here, but my interrupt is very
		// short and the effect of enabling the interrupts in the micros() function
		// *should* be negligable
		_now_us = micros();

		//Toggle debug pin 1 according to A XOR B (debounced)
		stdUtils::quickPinToggle(pinDEBUG_1, (debounceInput_A.CurrentState == debounceInput_B.CurrentState)? false : true);

		if (((pinA_edge) && (debounceInput_A.CurrentState == debounceInput_B.CurrentState)) ||
			((pinB_edge) && (debounceInput_A.CurrentState != debounceInput_B.CurrentState)))
			_direction = ROTATE_BACKWARD; //ROTATE_FORWARD;//

		if (((pinA_edge) && (debounceInput_A.CurrentState != debounceInput_B.CurrentState)) ||
			((pinB_edge) && (debounceInput_A.CurrentState == debounceInput_B.CurrentState)))
			_direction = ROTATE_FORWARD; //ROTATE_BACKWARD;//

		//Calculate the pulse period on the fly
		_pulsePeriod_us = (_now_us - _lastEdge_us);

		//Should we perhaps consider averaging the pulse period over a couple of pulses?
		//By adding the pulse period to the previous one and then dividing the result by 2 I'm
		// actually trying to "average" the variances out a bit. This means that the delta cannot
		// be more than 50% for any given time
		//_pulsePeriod_us += (_now_us - _lastEdge_us);
		//_pulsePeriod_us /= 2;

		//This is an attempt to average the pulse period over AMT203_PULSEWIDTH_ARR_CNT counts
		_pulsePeriod_us_arr[_pulsePeriod_us_cnt] = _pulsePeriod_us;
	    _pulsePeriod_us_cnt = (_pulsePeriod_us_cnt + 1)%AMT203_PULSEWIDTH_ARR_CNT;

	    //Save the micro second counter for the next rising edge
	    _lastEdge_us = _now_us;

		//Step the position on....
    	_position +=  _direction;

    	//It is very dangerous to WRAP here... The system should allow for some overshoot on the
    	// edges of the boundary

    	//We need to wrap at -360 and +720 degrees
		//if (_position > AMT203_QUAD_PPR_WRAP_MAX)	// If we reach > 540 degrees (3 x 180)
		//	_position -= AMT203_QUAD_PPR_RANGE;		// Wrap to -180
		//else if (_position <= AMT203_QUAD_PPR_WRAP_MIN)	// Or if we reach < -180 degrees
		//	_position += AMT203_QUAD_PPR_RANGE; 		// Wrap to 540

		//Ideally, if we are at the Target Position and the speed is low enough,
		// we want to stop right here?
		// But then we will need viibility into the PID control here?
		// Perhaps we can do the speed control here?
	}
	// else if ((!pinA_edge) && (!pinB_edge))

	//If we get no edge for how long, then do we assume we are standing still?

	//Do we have an edge on the zero line.
	if (pinX_edge)
	{
		//Set debug pin 2 HIGH While we are at the "real" zero (debounced)
		stdUtils::quickPinToggle(pinDEBUG_2, (debounceInput_X.CurrentState == HIGH)? true : false);
		//We can now determine what our offset from "actual" zero is
		_ZeroOffsetIsKnown = true;

		//On a rising edge we are at real position = 0;
		if (debounceInput_X.CurrentState == HIGH)
			_zero_offset = _position % AMT203_QUAD_PPR;

		//I suspect checking the falling edge of the zero is messing me around a bit

		//On a falling edge we are at -1 (going backward) or +1 (going forward)
		//else //if (debounceInput_X.CurrentState == LOW)
		//	_zero_offset = (_position - _direction) % AMT203_QUAD_PPR;

		//Now our offset must be somewhere between -360 and 360 degrees.
		_zero_offset = SetOffsetWithinLimit(_zero_offset);

		//If our offset is equal to or above 180 degrees, move it 360 degrees back.
		//if (_zero_offset >= (AMT203_QUAD_PPR/2))
		//	_zero_offset -= AMT203_QUAD_PPR;
		//Or If our offset is below 180 degrees, move it 360 degrees forward.
		//else if (_zero_offset < (-AMT203_QUAD_PPR/2))
		//	_zero_offset += AMT203_QUAD_PPR;

		//Now our offset can only be between -180 and 180 degrees, no?

		//By redoing this calculation on every"real" zero pulse we can see if our offset
		// will changes over time (as we miss or doubl-count pulses from the Encoder).

		//IMPORTAN: This could be useful to check if our debouncing is sufficient...
	}
	stdUtils::quickPinToggle(pinDEBUG_0, false);
}


#ifdef CONSOLE_MENU
/*******************************************************************************

Returns the motor speed in deg/s as set on the DAC

 *******************************************************************************/
void devMotorControl::PrintSpeedAndPosition(void)
{
	if (timerUtils::msTimerPoll(&printTraceTmr))
	{
		iPrintF(trMOTOR, "%s", devMotorControl_tag);
		iPrintF(trMOTOR, "P: % 8s | ", stdUtils::floatToStr(devMotorControl::GetPosition(), 3));
		iPrintF(trMOTOR, "R: % 8s | ", stdUtils::floatToStr(devMotorControl::GetRealPosition(), 3));//GetZeroOffset(), 3));
		iPrintF(trMOTOR, "E: % 7s | ", stdUtils::floatToStr(devMotorControl::GetSpeed_ENC() , 2));
		iPrintF(trMOTOR, "A: % 7s | ", stdUtils::floatToStr(devMotorControl::GetSpeed_AVG() , 2));
		iPrintF(trMOTOR, "D: % 7s\n",  stdUtils::floatToStr(devMotorControl::GetSpeed_DAC() , 2));
		timerUtils::msTimerReset(&printTraceTmr);
	}
}

/*******************************************************************************

Provides Motor settings functionality through the console
"Motor ?" to see the options.

 *******************************************************************************/
void devMotorControl::menuCmd(void)
{
float fltValue;
float motorSpeed_set;
int motorSpeed_set_i;
//int retVal;
char tmpStr1[10];
bool motorStop = false;
bool motorSpdSet = true;
char *paramStr;
//char *valueStr;

	//PrintF("\"Speed\" called with \"%s\"\n", paramStr);

	paramStr = devConsole::getParam(0);
	if (strcasecmp(paramStr, "stop") == NULL)
	{
		devMotorControl::Stop();
		return;
	}


	if (strcasecmp(paramStr, "Speed") == NULL)
	{
		fltValue = devMotorControl::GetSpeed_DAC();
		if (stdUtils::setFloatParam(paramStr, paramStr, devConsole::getParam(1), &fltValue) == 0)
		{
			motorSpeed_set = devMotorControl::SetSpeed_degs(fltValue);
			if (abs(motorSpeed_set) < MOTOR_SPD_INCREMENT_FLT)
				devMotorControl::Stop();
		}
		//No need for error message here... it would already be printed
		return;
	}

	if (strcasecmp(paramStr, "xfer+") == NULL)
	{
		if (strcasecmp(devConsole::getParam(1), "C") == NULL)
			stdUtils::setFloatParam("Xfer+ C", "Xfer+ C", devConsole::getParam(2), &Xfer.Pos.C);

		if (strcasecmp(devConsole::getParam(1), "M") == NULL)
			stdUtils::setFloatParam("Xfer+ M", "Xfer+ M", devConsole::getParam(2), &Xfer.Pos.M);

		PrintF(" Xfer+ : Y = %7sX %s ", stdUtils::floatToStr(Xfer.Pos.M, 3), (Xfer.Pos.C >= 0.0)? "+" : "-");
		PrintF(                      "%7s\n", stdUtils::floatToStr(abs(Xfer.Pos.C), 3));
		return;
	}

	if (strcasecmp(paramStr, "xfer-") == NULL)
	{
		if (strcasecmp(devConsole::getParam(1), "C") == NULL)
			stdUtils::setFloatParam("Xfer- C", "Xfer- C", devConsole::getParam(2), &Xfer.Neg.C);

		if (strcasecmp(devConsole::getParam(1), "M") == NULL)
			stdUtils::setFloatParam("Xfer- M", "Xfer- M", devConsole::getParam(2), &Xfer.Neg.M);

		PrintF(" Xfer- : Y = %sX %s ", stdUtils::floatToStr(Xfer.Neg.M, 3), (Xfer.Neg.C >= 0.0)? "+" : "-");
		PrintF(                      "%s\n", stdUtils::floatToStr(abs(Xfer.Neg.C), 3));
		return;
	}

	if (strcasecmp(paramStr, "Period") == NULL)
	{
		if (devConsole::paramCnt() == 2)
		{
			fltValue = 0.0;
			if (atof(devConsole::getParam(1)) == 0.0)
			{
				timerUtils::msTimerStop(&printTraceTmr);
				PrintF("%sTraces Stopped\n", devMotorControl_tag);
				return;
			}
			else if (stdUtils::setFloatParam("Period", paramStr, devConsole::getParam(1), &fltValue, 0.5, 10) == 0)
			{
				//fltValue now contains the frequency... which must be converted to period in ms
				printTraceTmr.msPeriod = (unsigned long)(1000.0 * fltValue);
				timerUtils::msTimerReset(&printTraceTmr);
			}
			else
				return;
		}
		PrintF("%sTrace Period: %s s\n",
				devMotorControl_tag,
				((printTraceTmr.Enabled)? stdUtils::floatToStr((((float)printTraceTmr.msPeriod)/1000.0), 2) : "OFF"));
		//No need for error message here... it would already be printed
		return;
	}
	if (strcasecmp(paramStr, "Pos") == NULL)
	{
		if (devConsole::paramCnt() == 2)
		{
			if (stdUtils::setFloatParam(paramStr, paramStr, devConsole::getParam(1), &fltValue, 0.0, 360.0) == 0)
			{
				//fltValue now contains the new position
				devMotorControl::SetPosition(fltValue);
			}
			else
				return;
		}
		PrintF("%sPosition: % 6s deg ",
				devMotorControl_tag,
				stdUtils::floatToStr(devMotorControl::GetPosition(), 2));
		PrintF("(= %s + ",
				stdUtils::floatToStr(devMotorControl::GetRealPosition(), 2));
		PrintF("%s)\n",
				stdUtils::floatToStr(devMotorControl::GetZeroOffset(), 2));
		return;
	}

	if (strcasecmp(paramStr, "ALL") == NULL)
	{
		PrintF("%sThe Motor Controller paramaters are:\n", devMotorControl_tag);
		PrintF(" Period: % 6s \n", stdUtils::floatToStr((((float)printTraceTmr.msPeriod)/1000.0), 2));
		PrintF(" Pos   : % 7s degs\n", stdUtils::floatToStr(devMotorControl::GetPosition(), 3));
		PrintF(" ActPos: % 7s degs\n", stdUtils::floatToStr(devMotorControl::GetRealPosition(), 3));
		PrintF(" Offset: % 7s degs\n", stdUtils::floatToStr(devMotorControl::GetZeroOffset(), 3));
		PrintF(" Speed : % 7s degs (ENC) \n", stdUtils::floatToStr(devMotorControl::GetSpeed_ENC(), 3));
		PrintF(" Speed : % 7s degs (AVG) \n", stdUtils::floatToStr(devMotorControl::GetSpeed_AVG(), 3));
		PrintF(" Speed : % 7s degs (SET) \n", stdUtils::floatToStr(devMotorControl::GetSpeed_DAC(), 3));
		PrintF(" Xfer+ : Y = %sX %s ", stdUtils::floatToStr(Xfer.Pos.M, 3), (Xfer.Pos.C >= 0.0)? "+" : "-");
		PrintF(                      "%s\n", stdUtils::floatToStr(abs(Xfer.Pos.C), 3));
		PrintF(" Xfer- : Y = %sX %s ", stdUtils::floatToStr(Xfer.Neg.M, 3), (Xfer.Neg.C >= 0.0)? "+" : "-");
		PrintF(                      "%s\n", stdUtils::floatToStr(abs(Xfer.Neg.C), 3));
		return;
	}

	PrintF("Valid commands:\n");
	PrintF("   All       - Prints the values for all parameters\n");
	PrintF("   Pos       - motor Position (Rd/Wr) -360.0 to 360.0\n");
	PrintF("   Speed     - motor speed -36.0 to 36.0\n");
	//PrintF("   DAC       - DAC absolute value (WO) -1023 to 1023\n");
	PrintF("   Stop      - Stops the motor\n");
	PrintF("   Period    - Trace Frequency (Rd/Wr) 0.5 to 10 s (0 to disable)\n");
	PrintF("   Xfer<+/-> - Pos/Neg Xfer function constants (Rd/Wr)\n");
	PrintF("\n");
}
#endif /* CONSOLE_MENU */

#undef EXT
/*************************** END OF FILE *************************************/
