/*******************************************************************************

 Project:    Outdoor Rotator
 Purpose:    This file contains the controlling app for the Motor control of the
 Outdoor Rotator
 Author:     Rudolph van Niekerk
 Processor:  Arduino Uno Rev3 (ATmega328)
 Compiler:	Arduino AVR Compiler

 The Motor speed and direction is controlled by a KES2420BL-4Q02 Motor Controller
 supplied along with the 24Vdc motor. Position feedback is provided by an absolute
 rotary encoder with a zero point.

 The included classes are:

 ** devConsole **
 A class which exposes the serial port for debugging purposes and maintains a
 linked list of menu functions for the other classes

 ** halTLC5156 **
 A class which handles interaction with the TI TLC5156 Digital to Analogue
 Controller (DAC).

 ** devMotorControl **
 A class which exposes the functionality of the KES2402BL-4Q02 Motor Speed
 Controller as well as the interfacing from the AMT203 rotary encoder.

 This class also uses the devTLC5156 class as it uses the 10 bit DAC
 to control the speed of the motor.

 The max output speed (100%) is 6 RPM = 36 degrees/s
 Our encoder output is 2x 1024 ppr signals offset by 90% wrt each other.... thus we get an
 effective resolution of 4096 ppr with direction.

 We are thinking of using a PI control loop to manage our speed and position.
 There are some good info here:
 https://softwareengineering.stackexchange.com/questions/186124/programming-pid-loops-in-c
 https://www.csimn.com/CSI_pages/PIDforDummies.html

 //TODO
 * 1 - Recalculate the Xfer equations for the Motor.... carefully.
 * 2 - At startup, do a quick forwad-backward pulse, just to let the user know
 * 		we are online.
 * 3 - Write interface for Matlab Comms/Driver
 * 4 - Write Matlab Comms/Driver
 *******************************************************************************/

 /*******************************************************************************
  includes
  *******************************************************************************/
#include <avr/pgmspace.h>
#include <SPI.h>
#include "defines.h"

#include "devMotorControl.h"
  //#include "halAMT203.h"
#include "halTLC5615.h"
//#include "paramCtrl.h"
#include "stdUtils.h"
#include "timerUtils.h"
#include "appPidControl.h"
#include "appWaveGen.h"
#include "version.h"
#ifdef CONSOLE_MENU
#include "devConsole.h"
#else
#include "devComms.h"
#endif
/*******************************************************************************
 local defines
 *******************************************************************************/
#define SYS_STATE_INIT			0	/* */
#define SYS_STATE_FIND_ZERO		1	/* */
#define SYS_STATE_STOPPED		2	/* */
#define SYS_STATE_MOVING		3	/* */
#define SYS_STATE_ERROR			9	/* */

 /*******************************************************************************
  Function prototypes
  *******************************************************************************/
void(*resetFunc)(void) = 0; //declare reset function at address 0

//Menu Commands (without any owners)
#ifdef MAIN_DEBUG
#ifdef CONSOLE_MENU
void menuTime(void);
void menuReset(void);
void menuVersion(void);
#endif /* CONSOLE_MENU */
#endif /* MAIN_DEBUG */

/*******************************************************************************
 local structure
 *******************************************************************************/

 /*******************************************************************************
  local variables
  *******************************************************************************/
  //The help menu structures
#ifdef MAIN_DEBUG
#ifdef CONSOLE_MENU
ST_CONSOLE_LIST_ITEM devMenuItem_time = { NULL, "time", menuTime, "Returns the system time" };
ST_CONSOLE_LIST_ITEM devMenuItem_reset = { NULL, "reset", menuReset, "Resets the system" };
ST_CONSOLE_LIST_ITEM devMenuItem_version = { NULL, "version", menuVersion, "Display firmware version" };
#endif /* CONSOLE_MENU */
#endif /* MAIN_DEBUG */
//int SystemState;		/* Movement State Machine variable */

const char BuildTimeData[] = { __TIME__ " " __DATE__ }; /* Used in our startup Banner*/


/*******************************************************************************
 Functions
 *******************************************************************************/

void setup() {


	//	ProgramVersion = 0x11;

		// put your setup code here, to run once:

	SystemStatus = 0;

	//Initialize serial communication on the standard port with the USB-2-RS232 converter
#ifdef CONSOLE_MENU
	devConsole::Init(115200, SERIAL_8N1);
#else
	devComms::Init(115200, SERIAL_8N1);
#endif
	// print our sign-on banner
	PrintF("\n");
	PrintF("=====================================================\n");
	PrintF("Outdoor Rotator - Control Program\n");
	PrintF("[c] 2017 Alaris Antennas (Pty) Ltd\n");
	PrintF("Version   %s.\n", PROG_VERSIONSTRING);
	PrintF("BuildInfo %s.\n", BuildTimeData);
	PrintF("Arduino Uno R3 (Clock %lu MHz)\n", F_CPU / 1000000L);
	PrintF("=====================================================\n");


	// initialize the Motor Controller (which will also check that the DAC and ENCODER is good).
	if (devMotorControl::Init() == false) {
		iPrintF(trMAIN | trALWAYS, "%sMotor Controller Init Failed\n", "MAIN");
		while (1); //we might as well chill right here doing nothing!
	}

	//Set PID controller to default values.
	if (appPidControl::Init() == false) {
		iPrintF(trMAIN | trALWAYS, "%sPID Controller Init Failed\n", "MAIN");
		while (1); //we might as well chill right here doing nothing!
	}

#ifdef USE_WAV_GEN
	appWaveGen::Init();
#endif /* USE_WAV_GEN */

	//Add the Menu Items which has no owners
#ifdef MAIN_DEBUG
	devConsole::addMenuItem(&devMenuItem_reset);
	devConsole::addMenuItem(&devMenuItem_time);
	devConsole::addMenuItem(&devMenuItem_version);
#endif /* MAIN_DEBUG */


#ifdef CONSOLE_MENU
	//Set the print traces we want...
	devConsole::SetTrace(trALL);
	devConsole::ClearTrace(devConsole::GetTraceIndex("Console"));
	devConsole::ClearTrace(devConsole::GetTraceIndex("Motor"));
	devConsole::ClearTrace(devConsole::GetTraceIndex("Encoder"));
	devConsole::ClearTrace(devConsole::GetTraceIndex("Var0"));
#else
	stdUtils::SetStatus(statusOK);
#endif /* CONSOLE_MENU */

	/*
	PrintF("13.9543 to %d decimal points: %s\n", 0,stdUtils::floatToStr(13.9543, 0));
	PrintF("13.9543 to %d decimal points: %s\n", 1,stdUtils::floatToStr(13.9543, 1));
	PrintF("13.9543 to %d decimal points: %s\n", 2,stdUtils::floatToStr(13.9543, 2));
	PrintF("13.9543 to %d decimal points: %s\n", 3,stdUtils::floatToStr(13.9543, 3));
	PrintF("13.9543 to %d decimal points: %s\n", 4,stdUtils::floatToStr(13.9543, 4));
	*/
}

void loop() {
	// put your main code here, to run repeatedly:

	//Check for anything coming in on the Serial Port and process it
#ifdef CONSOLE_MENU
	devConsole::Read();
#else
	devComms::Read();
#endif

	//Do the Position controlling (if enabled and required).
	appPidControl::ControlStateHandler();

	//We only consider using the waveform generator if the PID is not active.
#ifdef USE_WAV_GEN
	else if (appWaveGen::Enabled()) {
	appWaveGen::Process();
	}
#endif /* USE_WAV_GEN */
}

/*******************************************************************************

 Display program version info

 *******************************************************************************/
#ifdef MAIN_DEBUG
void menuVersion(void)
{
	PrintF("VER %s\n", PROG_VERSIONSTRING);
}

/*******************************************************************************

 Reads or Sets the system time.

 *******************************************************************************/
void menuTime(void)
{
	PrintF("Running for %lu s\n", ((long)millis() / 1000));
}

/*******************************************************************************

 Resets the system

 *******************************************************************************/
void menuReset(void)
{
	char *paramStr;
	// if no parameter passed then just open the gate
	paramStr = devConsole::getParam(0);

	if (!paramStr)
	{
		PrintF("Now type 'reset Y', IF YOU ARE SURE.\n");
		return;
	}
	if (*(char *)paramStr != 'Y')
	{
		PrintF("'reset Y' expected. Start over.\n");
		return;
	}
	// ok, do the reset.
	PrintF("Resetting. Goodbye, cruel world!\n");
	Serial.flush();

	resetFunc();//devMCUReset();
}
#endif /* MAIN_DEBUG */


/*************************** END OF FILE *************************************/
