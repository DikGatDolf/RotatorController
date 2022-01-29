/*******************************************************************************

Project:    Outdoor Rotator
Module:     devConsole.cpp
Purpose:    This file contains the console interface
Author:     Rudolph van Niekerk
Processor:  Arduino Uno Rev3 (ATmega328)
Compiler:	Arduino AVR Compiler

The console device will parse all received input strings (terminated with a
carriage return & newline) on the debug port for functional data. The first
part of the input string is matched in the local structured array to check for a
matching command. All additional data will be treated as parameters.

NOTE TO THE PROGRAMMER:
To expand on the functions of the console, add your unique command, function
pointer and help text to the Console Menu structure array. Then add your
function (remember the prototype) to the bottom of the file. The pointer to
the start of the parameters (if any) will be passed as the parameter to your
function. NULL means no parameters followed the command.

 *******************************************************************************/

/*******************************************************************************
includes
 *******************************************************************************/
#define __NOT_EXTERN__
#include "devConsole.h"
#undef __NOT_EXTERN__

#include "devMotorControl.h"

#include "halTLC5615.h"
#include "version.h"
#include "stdUtils.h"

#ifdef CONSOLE_MENU

/*******************************************************************************
local defines
 *******************************************************************************/
//map PrintF() to SerialPrintf()  (Can't use printf() as the IDE uses it to include stdio.h and then our define would create problems. )
//#define PrintF SerialPrintf

static ST_CONSOLE_LIST_ITEM * FirstMenuItem;
static ST_CONSOLE_LIST_ITEM * LastMenuItem;

static unsigned long Dev_DumpAddr = 0x10000L;
static unsigned int Dev_DumpLen = 256;
//static unsigned int Dev_DumpDev = 0;

/*******************************************************************************
local variables
 *******************************************************************************/

/*******************************************************************************
local function prototypes
 *******************************************************************************/

/*******************************************************************************
local structure
 *******************************************************************************/
#ifdef MAIN_DEBUG
typedef struct
{
	int Pin[6];
	bool State[6];
}ST_DEBUG_PORT;
#endif /*MAIN_DEBUG*/


/*******************************************************************************
local variables
 *******************************************************************************/
char devConsole_tag[6];
ST_CONSOLE    stDev_Console;
bool devConsole_initOK = false;

const char u8Dev_BackSpaceEcho[] = {0x08, 0x20, 0x08, 0x00};

byte traceMask = trALL;

ST_PRINT_FLAG_ITEM traceFlags[8] 	 = {
		{trDAC, 		"DAC"},		/* 1 - 0x01 */
		{trCONSOLE, 	"Console"},	/* 2 - 0x02 */
		{trMOTOR, 		"Motor"},	/* 3 - 0x04 */
		{trPIDCTRL, 	"PID"},		/* 4 - 0x08 */
		{trWAVEFORM, 	"Waveform"},/* 5 - 0x10 */
		{trENC, 		"Encoder"},	/* 6 - 0x20 */
		{trMAIN, 		"Main"},	/* 7 - 0x40 */
		{trALWAYS, 		"ALWAYS"}	/* 8 - 0x80 */
};

#ifdef MAIN_DEBUG
ST_DEBUG_PORT debugPort;
#endif /*MAIN_DEBUG*/

char * _paramStr[5];  // We can keep pointers to up to 5 parameters in here...
int _paramCnt;

//ST_CONSOLE_LIST_ITEM devMenuItem_help 		= {NULL, "read", devConsole::menuPrintHelp,	"Duh!"};
//ST_CONSOLE_LIST_ITEM devMenuItem_dump 		= {NULL, "write", devConsole::menuDumpMem,	"Dump memory (as bytes). Dump <ADDR(hex)> <LEN(dec)>"};

ST_CONSOLE_LIST_ITEM devMenuItem_help 		= {NULL, "help", devConsole::menuPrintHelp,	"Duh!"};
ST_CONSOLE_LIST_ITEM devMenuItem_dump 		= {NULL, "dump", devConsole::menuDumpMem,	"Dump memory (as bytes). Dump <ADDR(hex)> <LEN(dec)>"};
ST_CONSOLE_LIST_ITEM devMenuItem_Print 		= {NULL, "trace",devConsole::menuTogglePrintFlags,	"Toggle print flags"};
#ifdef MAIN_DEBUG
ST_CONSOLE_LIST_ITEM devMenuItem_DbgPin 	= {NULL, "pin",  devConsole::menuTogglePin, "Toggle Debug pins A(0 to 5)"};
#endif /* MAIN_DEBUG */

/*******************************************************************************
Function prototypes
*******************************************************************************/

extern "C" {
	int serialputc(char c, FILE *fp)
  	{
		if(c == '\n')
			Serial.write('\r');
		return Serial.write(c);
	}
}

/*******************************************************************************

Replaces the printf I am so used to.
Override to accommodate flags (to turn certain things on or off).

 *******************************************************************************/
void devConsole::_SerialPrintf(int traceflags, const char *fmt, ...)
{
FILE stdiostr;
va_list ap;

	if ((traceMask & traceflags) == trNONE)
		return;

	fdev_setup_stream(&stdiostr, serialputc, NULL, _FDEV_SETUP_WRITE);

	va_start(ap, fmt);
	vfprintf_P(&stdiostr, fmt, ap);
	va_end(ap);
}

/*******************************************************************************

Replaces the printf I am so used to.

 *******************************************************************************/
void devConsole::_SerialPrintf(const char *fmt, ...)
{
FILE stdiostr;
va_list ap;

  fdev_setup_stream(&stdiostr, serialputc, NULL, _FDEV_SETUP_WRITE);

  va_start(ap, fmt);
  vfprintf_P(&stdiostr, fmt, ap);
  va_end(ap);
}

/*******************************************************************************

Initialises the Console

 *******************************************************************************/
bool devConsole::Init(unsigned long baud, byte config)
{
	if (devConsole_initOK)
		return true;

	devConsole_initOK = false;

    // The Console will run on the Debug port
	//Serial.begin(115200, SERIAL_8N1);
	Serial.begin(baud, config);
	Serial.flush();

    FirstMenuItem = NULL;
    LastMenuItem = NULL;

	devConsole_initOK = true;

	strncpy(devConsole_tag, "[CON]", 5);
	devConsole_tag[5] = 0;


	//Assign the "private"
	addMenuItem(devConsole_tag, &devMenuItem_help);
	addMenuItem(devConsole_tag, &devMenuItem_dump);
	addMenuItem(devConsole_tag, &devMenuItem_Print);
#ifdef MAIN_DEBUG
	addMenuItem(&devMenuItem_DbgPin);
#endif /* MAIN_DEBUG */

#ifdef MAIN_DEBUG
	debugPort.Pin[0] = pinDEBUG_0;
	debugPort.Pin[1] = pinDEBUG_1;
	debugPort.Pin[2] = pinDEBUG_2;
	debugPort.Pin[3] = pinDEBUG_3;
	debugPort.Pin[4] = pinDEBUG_4;
	debugPort.Pin[5] = pinDEBUG_5;
	for (int i = 0; i < 6; i++)
	{
		debugPort.State[i] = false;
		pinMode(debugPort.Pin[i], OUTPUT);
		digitalWrite(debugPort.Pin[i], LOW);
		//iPrintF(trCONSOLE, "\n%sDebug Pin %d INIT OK\n", devConsole_tag, i);
	}
#endif /*MAIN_DEBUG*/

	iPrintF(trCONSOLE, "\n%sInit OK\n", devConsole_tag);

	return true;
}
/*******************************************************************************

Sets selected trace Flags

 *******************************************************************************/
void devConsole::SetTrace(int flagIndex)
{
	if (flagIndex < 7)
	{
		traceMask |= (0x01 << flagIndex);
	}
	else if (flagIndex == trALL)
	{
		for (int i = 0; i < 7; i++)
			traceMask |= (0x01 << i);
	}
}

/*******************************************************************************

Sets selected print Flags

 *******************************************************************************/
void devConsole::ClearTrace(int flagIndex)
{
	if (flagIndex < 7)
	{
		traceMask &= (~traceFlags[flagIndex].flagMask);
		traceMask |= trALWAYS; //Keep the ALWAYS flag on.
	}
	else if (flagIndex == trALL)
	{
		for (int i = 0; i < 7; i++)
			traceMask &= (~(0x01 << i));
		traceMask |= trALWAYS; //Keep the ALWAYS flag on.
	}
}

/*******************************************************************************

Sets selected print Flags

 *******************************************************************************/
bool devConsole::GetTrace(int flagIndex)
{
	if (flagIndex < 7)
	{
		return ((traceMask >> flagIndex) & 0x01)? true : false;
	}
	return false;
}

int devConsole::GetTraceIndex(char * traceName)
{
	for (int i = 0; i < 8; i++)
	{
		if ( strcasecmp(traceName, traceFlags[i].Name) == 0)
			return i;
	}
	return 9;
}

char * devConsole::GetTraceName(int flagIndex)
{
	if (flagIndex < 7)
	{
		return traceFlags[flagIndex].Name;
	}
	return "";
}

/*******************************************************************************

Reads the data on the serial port and parses the line when a carriage return is
encountered.

 *******************************************************************************/
bool devConsole::Read(void)
{
	//UINT16 rxBytes, cnt;
	//UINT16 rxCnt;
	byte rxData;
	bool retVal = false;

	if (devConsole_initOK == false)
	{
		iPrintF(trCONSOLE | trALWAYS, "%sNot Initialized yet\n", devConsole_tag);
		return false;
	}

	while (Serial.available() > 0)
	{
		// read the incoming byte:
		rxData = Serial.read();
		//printf("Received: %c (0x%02X)  - Inptr @ %d\n", rxData, rxData, stDev_Console.InPtr);
		retVal = true;

		// Skip Newline characters
		if (rxData == '\n')
			continue;

		// Lines are terminated on Carriage returns.
		switch (rxData)
		{
		// ****** Newline ******
		case '\n':
			// Skip Newline characters
			continue;

			// ****** Carriage Return ******
		case '\r':
			PrintF("\n");
			//Serial.write('\n');
			// Now parse the line if it is Valid
			if (stDev_Console.InPtr > 0)
			{
				//printf("Parseline() called\n");
				devConsole::ParseLine();
			}
			// Start a new line now....
			stDev_Console.InPtr = 0;      // Reset index pointer
			break;

			// ****** Backspace ******
		case 0x08:
			// Move one char back in buffer
			if (stDev_Console.InPtr)
			{
				stDev_Console.InPtr--;
				Serial.print(u8Dev_BackSpaceEcho);
			}
			break;

			// ****** Escape ******
		case 0x1B:
			// Clear the line...
			while (stDev_Console.InPtr)
			{
				Serial.print(u8Dev_BackSpaceEcho);
				stDev_Console.InPtr--;
			}
			stDev_Console.InPtr = 0;
			break;

			// ****** All other chars ******
		default:
			// Must we echo the data on the console?
			Serial.write(rxData);

			// Do we still have space in the rx buffer?
			if (stDev_Console.InPtr < CONSOLE_RX_BUFF) //Wrap index?
			{
				// Add char and Null Terminate string
				stDev_Console.RxBuff[stDev_Console.InPtr++] = rxData;
			}
			else  // The index pointer now wraps, our data is invalid.
			{
				stDev_Console.InPtr     = 0;      // Reset the index pointer
			}
			break;
		}

		// Always NULL terminate whatever is in the buffer.
		stDev_Console.RxBuff[stDev_Console.InPtr] = 0;  // Null Terminate
	}

	//This just prints the speed and position trace if enabled
	devMotorControl::PrintSpeedAndPosition();

	return retVal;
}

/*******************************************************************************

Parses the line received on the Debug port.

 *******************************************************************************/
void devConsole::ParseLine(void)
{
	int index = 0;
//	char *ownerStr;
	char *commandStr;
	char *paramStr;
	ST_CONSOLE_LIST_ITEM *pt;

	commandStr = stDev_Console.RxBuff;
	// Find the start of the parameters (the first space in the Line)
	paramStr = stdUtils::nextWord(commandStr, true);


/*
 	paramStr = strchr(commandStr, ' ');

	//printf("Parseline() called\n");
	if (paramStr != NULL)
	{
		// NULL terminate the command string
		*paramStr = 0;

		//Check for characters after the space...
		paramStr++;
		// Skip additional space if necessary
		while (*paramStr == ' ')
		{
			paramStr++;
		}

		// The parameters start at the next character after the space
		if (*paramStr == 0)
		{
			paramStr = NULL;  //Reset paramStr pointer to nothing
		}
	}
	//else  paramStr == NULL
*/


	// Convert the command to lower case...
	strlwr(commandStr);

	//PrintF("commandStr: \"%s\"\n", commandStr);
	//PrintF("paramStr: \"%s\"\n", paramStr);
	// check first if the item is in the list

	//If this is a "?", then we can just pop out the help menu.
	if (strcasecmp("?", commandStr) == NULL)
	{
		devConsole::menuPrintHelp();
		return;
	}

	pt = FirstMenuItem;

	while (pt)
	{
		//PrintF("Comparing: \"%s\"\n", pt->Command);
		if (strcasecmp(pt->Command, commandStr) == NULL)
		{
			//Let's seperate all the parameters for the guys to use further on...
			devConsole::paramsParse(paramStr, true);

			//Debugging....

			//PrintF("Matched with: \"%s\"\n", pt->Command);
			iPrintF(trCONSOLE, "%s\"%s\" called with %d parameters:\n", devConsole_tag, commandStr, devConsole::paramCnt());
			for (int i = 0; i < devConsole::paramCnt(); i++)
				iPrintF(trCONSOLE, "%s    %s \n", devConsole_tag, devConsole::getParam(i));

			//pt->Func(pt->OwnerObj);
			pt->Func();
			return;
		}
		pt = (ST_CONSOLE_LIST_ITEM *)pt->Next;
	}
	return;
}

/*******************************************************************************

Adds a Menu Item Structure (with a specific callback function pointer) on the
Console Menu linked list.

 *******************************************************************************/
bool devConsole::addMenuItem(char * ptrTag, ST_CONSOLE_LIST_ITEM * menuItem)
{
	//Do not bother if we have not been initialized
	if (devConsole_initOK == false)
	{
		iPrintF(trCONSOLE | trALWAYS, "%sNot Initialized yet\n", devConsole_tag);
		return false;
	}

	//PrintF("CON0: 0x%08lX - %ld | 0x%08lX - %ld\n", *(unsigned long *)ownerObject, (unsigned long *)ownerObject, (unsigned long *)ownerObject);

	//Assign the owner to the structure
	menuItem->Tag = ptrTag;

	return addMenuItem(menuItem);
}

/*******************************************************************************

Adds a Menu Item Structure (with a specific callback function pointer) on the
Console Menu linked list.

 *******************************************************************************/
bool devConsole::addMenuItem(ST_CONSOLE_LIST_ITEM * menuItem)
{
	ST_CONSOLE_LIST_ITEM *pt;

	//Do not bother if we have not been initialized
	if (devConsole_initOK == false)
	{
		iPrintF(trCONSOLE | trALWAYS, "%sNot Initialized yet\n", devConsole_tag);
		return false;
	}
	// check first if the item is not already in the list
	pt = FirstMenuItem;

	//PrintF("Add %s to the Menu Item List\n", menuItem->Command);

	while (pt)
	{
		//PrintF("Checking list for presence of %s (%s)\n", menuItem->Command, pt->Command);
		if (pt == menuItem)
		{
			iPrintF(trCONSOLE, "\n%s\"%s\" already exists in the Menu List\n", devConsole_tag, menuItem->Command);
			break;
		}
		pt = (ST_CONSOLE_LIST_ITEM *)pt->Next;
	}

	if (!pt)
	{
		// append item to linked list
		menuItem->Prev = (void *)LastMenuItem;
		menuItem->Next = NULL;
		if (!FirstMenuItem)
		{
			//Adding menuItem->Command as the first Item in the list
			FirstMenuItem = menuItem;
		}
		else
		{
			//Adding menuItem->Command to end of list
			LastMenuItem->Next = (void *)menuItem;
		}

		LastMenuItem = menuItem;
	}
	return true;
}
/*******************************************************************************

Finds the parameters in the passed string (seperated by spaces)
Returns the number found.

 *******************************************************************************/
int devConsole::paramsParse(char * paramStr, bool terminate)
{
	//Start by clearing all the previously saved pointers...
	for (int i = 0; i < 5; i++)
		_paramStr[i] = NULL;

	//PrintF("%s called with %d parameters:\n", commandStr, this->paramCnt());

	//Let's start off by checking if we have at least 1 parameter
	_paramStr[0] = paramStr;
	_paramCnt = 0;
	while ((_paramStr[_paramCnt]) && (_paramCnt < 5))
	{
		//This Paramater is valid...
		_paramCnt++;;
		//Let's start looking at the next one
		_paramStr[_paramCnt] = stdUtils::nextWord(_paramStr[_paramCnt-1], true);
	}

	//How many did we get?
	return _paramCnt;
}
/*******************************************************************************

Returns the number of paramaeters found.

 *******************************************************************************/
int devConsole::paramCnt(void)
{
	return _paramCnt;
}

/*******************************************************************************

Returns the parameter at the specifed index...0 = first parameter, 4 = last

 *******************************************************************************/
char * devConsole::getParam(int index)
{
	if ((index < _paramCnt) && (index >= 0))
		return _paramStr[index];
	else
		return NULL;
}
/*******************************************************************************

Pops the next paramater out of the "stack"

 *******************************************************************************/
char * devConsole::getNextParam(void)
{
char * tmpVal = _paramStr[0];

	for (int i = 0; i < 4; i++)
		_paramStr[i] = _paramStr[i + 1];
	_paramStr[4] = NULL;

	return tmpVal;
}


/*******************************************************************************

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

      Add the Console Command functions from here on down in the file.

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

 *******************************************************************************/

/*******************************************************************************

Prints the help string for either the command in question, or for the entire
list of commands.

 *******************************************************************************/
void devConsole::menuPrintHelp(void)
{
	int index = 0;

	//Serial.println(F("The list of available commands are:\n\n"));
	PrintF("The list of available commands are:\n\n");

	ST_CONSOLE_LIST_ITEM * pt;

	// Run through the list and print all the Help strings...
	pt = FirstMenuItem;
	while (pt)
	{
		//PrintF("% 12s - % 5s%s.\n", pt->Command, ((pt->Tag) ? pt->Tag : ""), pt->HelpStr);
		PrintF("% 12s - %s.\n", pt->Command, pt->HelpStr);
		pt = (ST_CONSOLE_LIST_ITEM *)pt->Next;
	}
	PrintF("\nNOTE: Enter Parameters after the command, following a space.\n");
	PrintF("       Command strings longer than %d chars are invalid.\n", CONSOLE_RX_BUFF);
}

/*******************************************************************************

Dumps a block of memory in Byte Access

*******************************************************************************/
void
devConsole::menuDumpMem(void)
{
char *plen;

// get the address from the commandline, if any
  if(devConsole::getParam(0))
  {
    sscanf(strupr(devConsole::getParam(0)), "%lX", &Dev_DumpAddr);
// get the length from the commandline, if any

    if(devConsole::getParam(1))
      sscanf(devConsole::getParam(0), "%d", &Dev_DumpLen);
  }

// OK, dump
  devConsole::DumpMem(trCONSOLE | trALWAYS, (void *)Dev_DumpAddr, (u32)Dev_DumpAddr, Dev_DumpLen);
// add len to addr, so the next "Dump" without params just continues...
  Dev_DumpAddr += Dev_DumpLen;
}

/*******************************************************************************


 *******************************************************************************/
void devConsole::menuTogglePrintFlags(void)
{
char *argStr_1;
char *argStr_2;
//int argCnt = 0;
byte tmpFlags;
bool flagFound = false;
int affectedIndex = -1;

	//Get pointers to all the arguments
	argStr_1 = devConsole::getParam(0);//paramStr;

	if (argStr_1)
	{
		// Check if the user passed the "ALL" or "NONE" keywords
		if (strcasecmp(argStr_1, "ALL") == NULL)
		{
			SetTrace(trALL);
			affectedIndex = 7;
		}
		else if (strcasecmp(argStr_1, "NONE") == NULL)
		{
			ClearTrace(trALL);
			affectedIndex = 7;
		}
		else //Perhaps the user passed a flag name
		{
			argStr_2 = devConsole::getParam(1);//paramStr;
			for (int i = 0; i < 7; i++)
			{
				if (strcasecmp(argStr_1, GetTraceName(i)) == NULL)
				{
					affectedIndex = i;
					if (devConsole::paramCnt() > 1)
					{
						//Allright, we found a match
						if (strcasecmp(argStr_2, "ON") == NULL)
						{
							SetTrace(i);
						}
						else if (strcasecmp(argStr_2, "OFF") == NULL)
						{
							//PrintF("Turn the %s (%d) traces OFF\n", stdUtils::GetPrintFlagsName(i), i);
							ClearTrace(i);
						}
						else
						{
							affectedIndex = -1;
						}
					}
					break; //... from FOR loop
				}
			}
		}
	}
	else //No arguments passed
	{
		affectedIndex = 7;
	}
	if (affectedIndex == 7)
	{
		PrintF("Traces: \n");
		for (int i = 0; i < 7; i++)
			PrintF("% 10s : %s \n", GetTraceName(i), (GetTrace(i))? "ON" : "OFF");
	}
	else if (affectedIndex >= 0)
	{
		PrintF("% 10s : %s \n", GetTraceName(affectedIndex), (GetTrace(affectedIndex))? "ON" : "OFF");
	}
	else
	{
		//PrintF("Debug prints: \n");
		//for (int i = 0; i < 7; i++)
		//	PrintF("% 10s : %s \n", stdUtils::GetPrintFlagsName(i), (stdUtils::GetPrintFlagsBit(i))? "ON" : "OFF");

		PrintF("Valid commands:\n");
		PrintF(" \"ALL\"             - Turns ALL traces ON\n");
		PrintF(" \"NONE\"            - Turns ALL traces OFF\n");
		PrintF(" \"<NAME>\"          - Shows state of traces for <NAME>\n");
		PrintF(" \"<NAME> <ON/OFF>\" - Turns <NAME> traces ON or OFF\n");
	}
	PrintF("\n");
}
/*******************************************************************************

Provides access to the debug pins through the console
"pin ?" to see the options.

 *******************************************************************************/
#ifdef MAIN_DEBUG
void devConsole::menuTogglePin(void)
{
char * paramStr;
char *argStr_1;
char *argStr_2;
//char *argStr_3;
int argCnt = 0;

int debugPinNum;

paramStr = devConsole::getParam(0);
	//Get pointers to all the arguments
	argStr_1 = stdUtils::nextWord(paramStr, true);
	if (argStr_1)
	{
		argCnt++;;
		argStr_2 = stdUtils::nextWord(argStr_1, true);
		if (argStr_2)
		{
			argCnt++;;
		}
	}

	//PrintF("Toggle %s called with \"%s\"\n", paramStr, ((argCnt >= 1)? argStr_1 : ""));

	// if no parameter passed then assume the user wanted RPM
	if ((paramStr) && (isdigit(*paramStr)))
	{
		debugPinNum = (int)((*paramStr) - '0');

		//PrintF("Pin %d identified\n", debugPinNum);

		if ((debugPinNum >= 0) && (debugPinNum <= 5))
		{
			if (argCnt == 1)
			{
				//PrintF("Param = %s\n", argStr_1);

				//we are expecting a "hi" or a "lo"
				if (strcasecmp(argStr_1, "hi") == NULL)
					debugPort.State[debugPinNum] = true;
				else if (strcasecmp(argStr_1, "lo") == NULL)
					debugPort.State[debugPinNum] = false;
				else
					argCnt = 8;
			}
			if (argCnt == 0)
			{
				//PrintF("No Param, just toggle\n");
				//Just toggle the pin
				debugPort.State[debugPinNum] = !debugPort.State[debugPinNum];
			}
			if (argCnt < 2)
			{
				//if (debug.State[debugPinNum])
				//	PORTC |= _BV(debugPinNum);
				//else
				//	PORTC &= ~_BV(debugPinNum);

				stdUtils::quickPinToggle(debugPort.Pin[debugPinNum], debugPort.State[debugPinNum]);
				//digitalWrite(debug.Port[debugPinNum], (debug.State[debugPinNum])? HIGH : LOW);
				PrintF("Debug Pin A%d -> %s \n", debugPinNum , debugPort.State[debugPinNum]? "HI" : "LO");
				PrintF("\n");
				return;
			}
		}
	}

	PrintF("Valid Toggle commands are:\n");
	PrintF(" \"<#> <HI/LO>\" - Sets the debug pin A(0 to 5) hi or lo\n");
	PrintF(" \"<#>\" - Toggles the debug pin A(0 to 5)\n");
	PrintF("\n");
}
#endif /* MAIN_DEBUG */

/*void stdUtils::DumpMem(int Flags,
void * Src,           // ptr to memory to dump
unsigned long Address,       // address to display
unsigned short Len)               // nr of bytes to dump
*/
void devConsole::DumpMem(int Flags, void * Src, unsigned long Address, int Len)
{
u8 *s;
u8 x;
u16 cnt;
static char temp[16];

	// Are we even bothering to print this?
	if ((traceMask & Flags) == trNONE)
		return;

	s = (u8 *)Src;

	while(Len)
	{
		// print offset
		iPrintF(Flags, "%06lX : ", Address);
		// print hex data
		for(x = 0; x < 16; x++)
		{
			if (x < Len)
				iPrintF(Flags, "%02X%c", (byte)s[x], (byte)((x == 7) ? '-' : ' '));
			else
				iPrintF(Flags, "  %c", (byte)((x == 7) ? '-' : ' '));

		}
		// print ASCII data
		iPrintF(Flags, " ");
		for(x = 0; x < 16; x++)
		{
			if (x < Len)
				iPrintF(Flags, "%c", (byte)(((s[x] >= 0x20) && (s[x] <= 0x7f))? s[x] : '.'));
			else
				break;
		}
		// goto next line
		cnt = (Len > 16) ? 16 : Len;
		s       += cnt;
		Len     -= cnt;
		iPrintF(Flags, "\n");
		Address += 16;
	}

//  iprintf(Flags, "\n");
}

#endif /* CONSOLE_MENU */

#undef EXT
/*************************** END OF FILE *************************************/
