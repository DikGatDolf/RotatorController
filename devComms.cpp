/*******************************************************************************

Project:    Outdoor Rotator
Module:     devComms.cpp
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
#include "devComms.h"
#undef __NOT_EXTERN__

#include "Arduino.h"
#include "stdUtils.h"

#include "devMotorControl.h"
#include "appPidControl.h"

#include "halTLC5615.h"
#include "version.h"


#ifndef CONSOLE_MENU

/* RVN - This is for debug purposes only... we are not doing any CRC checking...
 * simply parsing what we have right after we receive the tail indicater.
 * Replace the correct line "return msCRC_H; when done with debugging.*/
//#define DEBUG_COMMS

/*******************************************************************************
local defines
 *******************************************************************************/
//map PrintF() to SerialPrintf()  (Can't use printf() as the IDE uses it to include stdio.h and then our define would create problems. )
//#define PrintF SerialPrintf

//Message States
#define msIDLE			0
#define msHEADER		6
#define msMESSAGE		7
#define msCRC_H			8
#define msCRC_L			9

#define TMP_STR_BUFF_SIZE 80

/*
ST_ERROR_MSG errOverflow 		= {900, "OVERFLOW (%d)"};
ST_ERROR_MSG errCrc 			= {901, "CRC (rx: %02X != %02X :calc)"};
ST_ERROR_MSG errHexChar 		= {902, "HEX (rx: %c in CRC)"};
ST_ERROR_MSG errTerminateEarly 	= {903, "MSG TERMINATED EARLY (%d)"};
ST_ERROR_MSG errNoCmd 			= {904, "UNKNOWN CMD: \"%s\""};
ST_ERROR_MSG errNoParam 		= {905, "UNKNOWN PARAM: \"%s\""};
ST_ERROR_MSG errNoRead 			= {906, "CANNOT READ: \"%s\""};
ST_ERROR_MSG errNoWrite 		= {907, "CANNOT WRITE: \"%s\""};
*/
/*******************************************************************************
local variables
 *******************************************************************************/
char TxPayloadBuff[TMP_STR_BUFF_SIZE + 1];  // The tmp buff for copying data closer to home (into SRAM)

/*******************************************************************************
local function prototypes
 *******************************************************************************/

/*******************************************************************************
local structure
 *******************************************************************************/

/*******************************************************************************
local variables
 *******************************************************************************/
const char *  CmdMsgHeader = "[ROT-C]";
const char *  ResponseMsgHeader = "[ROT-R]";
//const char *  DelimeterStr = {PARAM_DELIMETER, VALUE_DELIMETER, NULL };

ST_COMMS stDev_Comms;

const ST_SETTING_ITEM SettingsArray[] = {
		// 0  RO Status Word
		{"status",		(PARAM_READABLE),  NULL, NULL, NULL},

		// 1  RO The Actual current position relative to the startup point (startup zero)
		{"position",	(PARAM_READABLE|PARAM_WRITABLE),  MOTOR_POS_WRAP_MIN_STR, MOTOR_POS_WRAP_MAX_STR, NULL},
//		  12345678
		// 2  WO The Target for the PID controller to reach... setting this to a value different than position will activate the rotation and PID control
		{"target",		(PARAM_READABLE|PARAM_WRITABLE),  MOTOR_POS_WRAP_MIN_STR, MOTOR_POS_WRAP_MAX_STR, NULL},

		// 3  RW The maximum speed of rotation allowed on the final drive
		{"maxspd",		(PARAM_READABLE|PARAM_WRITABLE),  "2.0", MOTOR_SPD_ABS_MAX_STR, NULL},

		// 4  RW The maximum speed from which the the final drivewill be allowed to "hard stop"
		{"minspd",		(PARAM_READABLE|PARAM_WRITABLE),  MOTOR_SPD_ABS_MIN_STR, "5.0", "1.5"},

		// 5  RW The maximum acceleration allowed on the final drive
		{"maxaccel",	(PARAM_READABLE|PARAM_WRITABLE),  "36.0", "1.0", "9.0"},

		// 6  RW PID proportional constant
		{"kp",			(PARAM_READABLE|PARAM_WRITABLE),  "0.0", "1000.0", "80.0"},

		// 7  RW PID integral constant
		{"ki",			(PARAM_READABLE|PARAM_WRITABLE),  "0.0", "10.0", "0.4"},

		// 8  RW PID differential constant
		{"kd",			(PARAM_READABLE|PARAM_WRITABLE),  "0.0", "100.0", "2.0"},

		// 9 RW PID interval period
		{"period",		(PARAM_READABLE|PARAM_WRITABLE),  "0.01", "1.0", "0.01"},

		// 10 RW PID bias
		{"bias",		(PARAM_READABLE|PARAM_WRITABLE),  "0.0", "10.0", "0.0"},

		// 11 RO The "distance-to-target" for the last rotation of the PID controller
		{"dtt",			(PARAM_READABLE),  NULL, NULL, NULL},

		// 12 RO The "time-to-target" for the last rotation of the PID controller
		{"ttt",			(PARAM_READABLE),  NULL, NULL, NULL},

		// 13 RW Offset from actual zero... only known once the zero point has been passed
		{"offset",		(PARAM_READABLE),  "-360.0", "360.0", NULL},

		// 14 RO Real position from zero point
		{"realpos",		(PARAM_READABLE),  NULL, NULL, NULL},

		// 15 RO Speed as measured on Encoder (final shaft drive)
		{"speed",		(PARAM_READABLE),  NULL, NULL, NULL},

		// 16 RO Average speed as calculated over the last 10 pulses rx'd from Encoded (final shaft drive)
		{"speed_avg",	(PARAM_READABLE),  NULL, NULL, NULL},

		// 17 RO Speed as set on the DAC
		{"speed_dac",	(PARAM_READABLE),  NULL, NULL, NULL},
//		  123456789
		// 18 RW Positive quadrant Transfer function M-value (Don't f*ck around with this value unless you know what you are doing)
		{"xfer+M",		(PARAM_READABLE|PARAM_WRITABLE),  "0.5", "10.0", XFER_EQ_POS_M_STR},

		// 19 RW Positive quadrant Transfer function C-value (Don't f*ck around with this value unless you know what you are doing)
		{"xfer+C",		(PARAM_READABLE|PARAM_WRITABLE),  "0.5", "10.0", XFER_EQ_POS_C_STR},

		// 20 RW Negative quadrant Transfer function M-value (Don't f*ck around with this value unless you know what you are doing)
		{"xfer-M",		(PARAM_READABLE|PARAM_WRITABLE), /* true, true, */"0.5", "10.0", XFER_EQ_NEG_M_STR},

		// 21 RW Negative quadrant Transfer function C-value (Don't f*ck around with this value unless you know what you are doing)
		{"xfer-C",		(PARAM_READABLE|PARAM_WRITABLE),  "0.5", "10.0", XFER_EQ_NEG_C_STR},
		{NULL, 			NULL, /* false, false,*/ NULL, NULL, NULL}
};


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

void devComms::DoNothing(int traceflags, const char *fmt, ...)	{	/* Yep... Just don't do ANYTHING */	}

/*******************************************************************************

Replaces the printf I am so used to.

 *******************************************************************************/
void devComms::_SerialPrintf(const char *fmt, ...)
{
FILE stdiostr;
va_list ap;

  fdev_setup_stream(&stdiostr, serialputc, NULL, _FDEV_SETUP_WRITE);

  va_start(ap, fmt);
  vfprintf_P(&stdiostr, fmt, ap);
  va_end(ap);
}

/*******************************************************************************

Wrapper for the Error Prints

 *******************************************************************************/
void devComms::CmdResponseError(int errCode, const char *msg)
{
	byte crc = 0;
	crc = stdUtils::crc8_str(ResponseMsgHeader);
	crc = stdUtils::crc8_str(crc, "ERR ");
	crc = stdUtils::crc8_str(crc, stdUtils::TmpStrPrintf("%03d", errCode));
	if (*msg)
	{
		crc = stdUtils::crc8_str(crc, "");
		crc = stdUtils::crc8_str(crc, msg);
	}

	PrintF("%sERR %03d", ResponseMsgHeader, errCode);
	if (*msg != 0x00) PrintF(" ");
	PrintF("%s|%02X\n", msg, crc);
	//PrintF("[%s]ERR %03d%s%s\n", ResponseMsgHeader, errCode, ((*msg == 0x00)? "": " "), msg);
	//stdUtils::TmpStrPrintf();
}

/*******************************************************************************

Wrapper for the OK Prints

 *******************************************************************************/
void devComms::CmdResponseOK(const char * msg)
{
	byte crc = 0;
	crc = stdUtils::crc8_str(ResponseMsgHeader);
	crc = stdUtils::crc8_str(crc, "OK");
	if (*msg)
		crc = stdUtils::crc8_str(crc, " ");
	crc = stdUtils::crc8_str(crc, msg);

	PrintF("%sOK%s%s|%02X\n", ResponseMsgHeader, ((*msg == 0x00)? "": " "), msg, crc);
}
/*******************************************************************************

Initialises the Console

 *******************************************************************************/
bool devComms::Init(unsigned long baud, byte config)
{
    // The Console will run on the Debug port
	//Serial.begin(115200, SERIAL_8N1);
	Serial.begin(baud, config);
	Serial.flush();

	return true;
}
/*******************************************************************************

Sets selected trace Flags

 *******************************************************************************/

/*******************************************************************************

Reads the data on the serial port and parses the line when a carriage return is
encountered.

 *******************************************************************************/
void devComms::Read(void)
{
	//UINT16 rxBytes, cnt;
	//UINT16 rxCnt;
	byte rxData;

	while (Serial.available() > 0)
	{
		// read the incoming byte:
		rxData = Serial.read();
		//printf("Received: %c (0x%02X)  - Inptr @ %d\n", rxData, rxData, stDev_Console.InPtr);

		Serial.write((byte)rxData);
		//PrintF("%c", rxData);// print(u8Dev_BackSpaceEcho);

		// Skip Newline characters
		if (rxData == '\n')
			continue;

		//Parse the Rx'd byte based on the current status of the received message

		//PrintF(" RX: %c % 2d % 3d %02X\n", rxData, stDev_Comms.MsgState, stDev_Comms.InPtr, stDev_Comms.CRC_calc);

		//First off... a carriage return is to be used as the end of line/msg...
		if (rxData == '\r')
		{
			PrintF("\n");
			//The only time a carriage return is expected is when the message state is msIDLE
			if (stDev_Comms.MsgState >= msMESSAGE)
			{
				/* RVN - This is for debug purposes only... we are not doing any CRC checking...
				 * simply parsing what we have right after we receive the tail indicater or \r.
				 * Replace the correct line "return msCRC_H; when done with debugging.*/
#ifdef DEBUG_COMMS
				ParseLine();
#else
				CmdResponseError(903, stdUtils::TmpStrPrintf(TxPayloadBuff, TMP_STR_BUFF_SIZE, "%d", stDev_Comms.MsgState));
#endif /* DEBUG_COMMS */
			}
			stDev_Comms.InPtr = 0;
			stDev_Comms.MsgState = msIDLE;
			continue;
		}
		//Are we waiting for the start of a message or are we busy receiving a message header
		else if (stDev_Comms.MsgState < msMESSAGE)
		{
			stDev_Comms.MsgState = devComms::ParseByteForHeader(rxData);
		}
		//Are we busy receiving a message payload
		else if (stDev_Comms.MsgState == msMESSAGE)
		{
			stDev_Comms.MsgState = devComms::ParseByteForPayload(rxData);
		}
		//Are we busy receiving a message tail
		else //if (stDev_Comms.MsgState <= msCRC_L)
		{
			stDev_Comms.MsgState = devComms::ParseByteForTail(rxData);
		}

		// Always NULL terminate whatever is in the buffer.
		stDev_Comms.RxBuff[stDev_Comms.InPtr] = 0;  // Null Terminate
	}
}

/*******************************************************************************

Parses a byte for while we are expecting the Header section of the mesage.

 *******************************************************************************/
int devComms::ParseByteForHeader(byte rxData)
{
	stDev_Comms.InPtr = 0;

	//Is this the expected byte....
	if (rxData == CmdMsgHeader[stDev_Comms.MsgState])
	{
		//Yes... add this to our CRC calculation
		stDev_Comms.CRC_calc = stdUtils::crc8((stDev_Comms.MsgState == msIDLE)? 0: stDev_Comms.CRC_calc, rxData);
		//Wait for process the next byte
		return (stDev_Comms.MsgState + 1);
	}
	//No, but Just check if this is not the start of ythe new message.
	else if (rxData == CmdMsgHeader[0])
	{
		//Yes, it is... add this to our CRC calculation
		stDev_Comms.CRC_calc = stdUtils::crc8(0, rxData);
		return (msIDLE + 1);
	}
	//Nope... we are not expecting this... go back to the IDLE state
	else
	{
		return msIDLE;
	}

}
/*******************************************************************************

Parses a byte while we are expecting the Message section of the mesage.

 *******************************************************************************/
int devComms::ParseByteForPayload(byte rxData)
{
	//A pipe (|) signifies the CRC to follow, which was calculated up to (excluding) the pipe char itself.
	if (rxData == '|')
	{
		/* RVN - This is for debug purposes only... we are not doing any CRC checking...
		 * simply parsing what we have right after we receive the tail indicater.
		 * Replace the correct line "return msCRC_H; when done with debugging.*/
#ifdef DEBUG_COMMS
		PrintF("\n");
		ParseLine();
		return msIDLE;
#else
		return msCRC_H;
#endif /* DEBUG_COMMS */
		}

	if (stDev_Comms.InPtr >= COMMS_RX_BUFF_LEN)
	{
		CmdResponseError(900, stdUtils::TmpStrPrintf(TxPayloadBuff, TMP_STR_BUFF_SIZE, "%d", COMMS_RX_BUFF_LEN));
	}

	// Add this char to the buffer and Null Terminate string
	stDev_Comms.RxBuff[stDev_Comms.InPtr++] = rxData;

	//Remember to add the rx'd byte to the crc calc
	stDev_Comms.CRC_calc = stdUtils::crc8(stDev_Comms.CRC_calc, rxData);

	return msMESSAGE;
}

/*******************************************************************************

Parses a byte while we are expecting the Message section of the mesage.

 *******************************************************************************/
int devComms::ParseByteForTail(byte rxData)
{
	//This should be a ASCII HEX byte
	if (isxdigit (rxData) == 0)
	{
		//this is not a valid CRC character
		CmdResponseError(902, stdUtils::TmpStrPrintf(TxPayloadBuff, TMP_STR_BUFF_SIZE, "%c", rxData));
		return msIDLE;
	}

	// This character can only be '0'->'9', 'a'->'f', or 'A'->'F'

	else if (stDev_Comms.MsgState == msCRC_H)
	{
		//This is the High Nibble
		stDev_Comms.CRC_rx = stdUtils::charToNibble(rxData);
		stDev_Comms.CRC_rx <<= 4;
		return (stDev_Comms.MsgState + 1);
	}

	//else  if (stDev_Console.MsgState == msCRC_L)
	//This is the Low Nibble
	stDev_Comms.CRC_rx |= stdUtils::charToNibble(rxData);

	PrintF("\n");

	//OK, we've reached the end of our message tail... Do the CRC check now
	if (stDev_Comms.CRC_rx != stDev_Comms.CRC_calc)
	{
		//Oops.. CRC failed
		CmdResponseError(901, stdUtils::TmpStrPrintf(TxPayloadBuff, TMP_STR_BUFF_SIZE, "%02X:%02X", stDev_Comms.CRC_rx, stDev_Comms.CRC_calc));
	}
	else
	{
		//CRC is good. Parse the message now
		ParseLine();
	}
	return msIDLE;
}

/*******************************************************************************

Parses the line received on the Debug port.

 *******************************************************************************/
void devComms::ParseLine(void)
{
	int index = 0;
//	char *ownerStr;
	char *commandStr;
	char *paramStr;

	commandStr = stDev_Comms.RxBuff;
	// Find the start of the parameters (the first space in the Line)
	paramStr = stdUtils::nextWord(commandStr, true);

	// Convert the command to lower case...
	strlwr(commandStr);

	//PrintF("commandStr: \"%s\"\n", commandStr);
	//PrintF("paramStr: \"%s\"\n", paramStr);
	// check first if the item is in the list

	//The user basically has the following options here:
	//	"get"			Get a parameter value
	//  "seta"			Set a parameter to an absolute value
	//  "setr"			Adjust a parameter to a relative value
	//  "calibrate"		Start the calibration procedure
	//  "kill"			EMERGENCY STOP - (USE WITH CAUTION)

	if (strcasecmp("get", commandStr) == NULL)
	{
		readSetting(paramStr);
	}
	else if (strcasecmp("seta", commandStr) == NULL)
	{
		writeSetting(paramStr, true);
	}
	else if (strcasecmp("setr", commandStr) == NULL)
	{
		writeSetting(paramStr, false);
	}
	else if (strcasecmp("calibrate", commandStr) == NULL)
	{
		//RVN, find tha absolute zero and set that as the point of reference (relative zero).
		appPidControl::StartCalibration();
		devComms::readSetting("status");
	}
	else if (strcasecmp("kill", commandStr) == NULL)
	{
		//just kill everything.
		//Stop the motor... HARSHLY
		devMotorControl::KillMotor();
		//We should also stop the PID controller from taking over again.
		appPidControl::Stop();
	}
	else
	{
		CmdResponseError(904, stdUtils::TmpStrPrintf(TxPayloadBuff, TMP_STR_BUFF_SIZE, "%s", commandStr));
	}
}
/*******************************************************************************

Does the "GET" functionality of the settings
We can expect the following type of messages:
	get <param>
	get <param_1>,<param_2>,...,<param_n>

 *******************************************************************************/
void devComms::readSetting(char * paramStr)
{
	int paramIndex = 0;
	char * thisParam = paramStr;
	char * nextParam;
	int paramCount = countParams(paramStr);
	int strLen = 0;
	char * valStr;

	//PrintF("GET %d params: \"%s\"\n", paramCount, paramStr);

	//CmdResponseOK(stdUtils::TmpStrPrintf("GET \"%s\"\n", thisParam));

	// Are all the parameters valid?
	if (!isRdSettingsValid(paramStr, false))
	{
		//Invalid parameters will allready be dealt with.. we can just exit silently at this point.
		return;
	}

	//OK, now we KNOW every parameter is good to go.... we just need to build up our response string

	while (thisParam)
	{
		//Decrement the paramater count
		paramCount--;

		//This call will find the next parameter's start as well as null-terminate this parameter
		nextParam = stdUtils::nextWord(thisParam, true, PARAM_DELIMETER);

		//Is this parameter referenced by index or by name
		paramIndex = (stdUtils::isNaturalNumberStr(thisParam))? atoi(thisParam) : getParamIndex(thisParam);

		//Add every parameter value (comma delimted) in the string.
		valStr = getParamValueStr(paramIndex);
		stdUtils::TmpStrPrintf(TxPayloadBuff + strLen, TMP_STR_BUFF_SIZE - strLen, "%s%c%s%s", thisParam, (byte)VALUE_DELIMETER, valStr, (paramCount>0)? ",": "");

		//Get the updated string length
		strLen = strlen(TxPayloadBuff);
		//PrintF("%02d Value of \"%s\" (%d) = %s\n", paramCount, thisParam, paramIndex, valStr);

		//If we are dealing with multiple parameters, move on to the next one.
		thisParam = nextParam;
	}

	CmdResponseOK(TxPayloadBuff);
}
/*******************************************************************************

Does the "SET" functionality of the settings
Checks

 *******************************************************************************/
char * devComms::getParamValueStr(int paramIndex)
{
	char * retVal;

	switch (paramIndex)
	{
		case 0:		retVal = stdUtils::SatusWordBinStr();/*stdUtils::TmpStrPrintf("%d", SystemStatus);*/break;// status (word)
		case 1:		retVal = stdUtils::floatToStr(devMotorControl::GetPosition(), 1);		break;// position
		case 2:		retVal = stdUtils::floatToStr(appPidControl::pidSettings.Target, 1);	break;// target
		case 3:		retVal = stdUtils::floatToStr(appPidControl::pidSettings.MaxSpeed, 2);	break;// maxspd
		case 4:		retVal = stdUtils::floatToStr(appPidControl::pidSettings.MinSpeed, 2);	break;// minspd
		case 5:		retVal = stdUtils::floatToStr(appPidControl::pidSettings.MaxAccel, 2);	break;// maxaccel
		case 6:		retVal = stdUtils::floatToStr(appPidControl::pidSettings.Kp, 3);		break;// kp
		case 7:		retVal = stdUtils::floatToStr(appPidControl::pidSettings.Ki, 3);		break;// ki
		case 8:		retVal = stdUtils::floatToStr(appPidControl::pidSettings.Kd, 3);		break;// kd
		case 9:		retVal = stdUtils::floatToStr(appPidControl::pidSettings.Period, 2);	break;// Period
		case 10:	retVal = stdUtils::floatToStr(appPidControl::pidSettings.bias, 3);		break;// Bias
		case 11:	retVal = stdUtils::floatToStr(abs(appPidControl::pidSettings.Target-appPidControl::pidSettings.startPos), 1);		break;// Distance-To-Target
		case 12:	retVal = stdUtils::floatToStr(appPidControl::pidSettings.TimeToTarget, 1);	break;// Time-To-Target
		case 13:	retVal = stdUtils::floatToStr(devMotorControl::GetZeroOffset(), 2);		break;// zeroOffset
		case 14:	retVal = stdUtils::floatToStr(devMotorControl::GetRealPosition(), 2);	break;// RealPos
		case 15:	retVal = stdUtils::floatToStr(devMotorControl::GetSpeed_ENC(), 2);		break;// speed
		case 16:	retVal = stdUtils::floatToStr(devMotorControl::GetSpeed_AVG(), 2);		break;// speed_avg
		case 17:	retVal = stdUtils::floatToStr(devMotorControl::GetSpeed_DAC(), 2);		break;// speed_dac
		case 18:	retVal = stdUtils::floatToStr(devMotorControl::Xfer.Pos.M, 3);			break;// Xfer_Pos_M
		case 19:	retVal = stdUtils::floatToStr(devMotorControl::Xfer.Pos.C, 3); break; // Xfer_Pos_C
		case 20:	retVal = stdUtils::floatToStr(devMotorControl::Xfer.Neg.M, 3);			break;// Xfer_Neg_M
		case 21:	retVal = stdUtils::floatToStr(devMotorControl::Xfer.Neg.C, 3); break; // Xfer_Neg_C
		default:	retVal = NULL;
		break;
	}
	return retVal;
}

/*******************************************************************************

Does the "SET" functionality of the settings
We can expect the following type of messages:
	set <param>=<value>
	set <param_1>=<value_1>,<param_2>=<value_2>,...,<param_n>=<value_n>

 *******************************************************************************/
void devComms::writeSetting(char * paramStr, bool absolute)
{

	int paramIndex = 0;
	char * thisParam = paramStr;
	char * thisValue;
	char * nextParam;
	int paramCount = countParams(paramStr);
	int strLen = 0;
	char * valStr;
	float val_current;
	float val_math;

	//PrintF("SET %d params: \"%s\"\n", paramCount, paramStr);
	//CmdResponseOK(stdUtils::TmpStrPrintf(TxPayloadBuff, TMP_STR_BUFF_SIZE, "SET \"%s\"\n", paramStr));

	//CmdResponseOK(stdUtils::TmpStrPrintf("GET \"%s\"\n", thisParam));

	// Are all the parameters valid?

	if (!isWrSettingsValid(paramStr, absolute))
	{
		//Invalid parameters will allready be dealt with.. we can just exit silently at this point.
		return;
	}

	//OK, now we KNOW every parameter is good to go.... we just need to build up our response string
	//We can now alter the data in the RX buffer

	while (thisParam)
	{
		//Decrement the paramater count
		paramCount--;

		//This call will find the next parameter's start as well as null-terminate this parameter
		nextParam = stdUtils::nextWord(thisParam, true, PARAM_DELIMETER);

		//If this parameter-value pair, it contains a ":" , we can null terminate there.
		thisValue = stdUtils::nextWord(thisParam, true, VALUE_DELIMETER);

		//Is this parameter referenced by index or by name
		paramIndex = (stdUtils::isNaturalNumberStr(thisParam))? atoi(thisParam) : getParamIndex(thisParam);

		//Determine the final value and set it to the correct parameter
		val_current = atof((const char *)getParamValueStr(paramIndex));
		val_math = stdUtils::FloatMathStr(thisValue, val_current, absolute);
		valStr = setParamValueStr(paramIndex, val_math);
		//PrintF("WRITE \"%s\" (%d) %s= \"%s\" (", thisParam, paramIndex, ((absolute)? "": "+"), thisValue);
		//PrintF("%s -> ", stdUtils::floatToStr(val_current, 3));
		//PrintF("%s", stdUtils::floatToStr(val_math, 3));
		//PrintF(")\n");

		//Add every parameter & final value pair (comma delimted) in the string.
		stdUtils::TmpStrPrintf(TxPayloadBuff + strLen, TMP_STR_BUFF_SIZE - strLen, "%s:%s%s", thisParam, valStr, (paramCount>0)? ",": "");

		//Get the updated string length
		strLen = strlen(TxPayloadBuff);
		//PrintF("%02d Value of \"%s\" (%d) = %s\n", paramCount, thisParam, paramIndex, valStr);

		//If we are dealing with multiple parameters, move on to the next one.
		thisParam = nextParam;
	}

	CmdResponseOK(TxPayloadBuff);
}

/*******************************************************************************

Does the "SET" functionality of the settings
Checks

 *******************************************************************************/
char * devComms::setParamValueStr(int paramIndex, float finalValue)
{
float * dst = NULL;
char * retVal = NULL;

//	PrintF("   Set %s to %d\n", stdUtils::floatToStr(finalValue, 3), paramIndex);

	switch (paramIndex)
	{
		case 1:		retVal = stdUtils::floatToStr(devMotorControl::SetPosition(finalValue), 3);	break;// position
		/*The target is a special case since it should start a bunch of actions.... or should it?
		 * Why can't we have the PID on PERMANENTLY?*/
		case 2:
			appPidControl::GotoPos(finalValue);
			retVal = stdUtils::floatToStr(appPidControl::pidSettings.Target, 1);
			//dst = &appPidControl::pidSettings.Target;
			break;// target
		case 3:		dst = &appPidControl::pidSettings.MaxSpeed;	break;// maxspd
		case 4:		dst = &appPidControl::pidSettings.MinSpeed;	break;// minspd
		case 5:		dst = &appPidControl::pidSettings.MaxAccel;	break;// maxaccel
		case 6:		dst = &appPidControl::pidSettings.Kp;		break;// kp
		case 7:		dst = &appPidControl::pidSettings.Ki;		break;// ki
		case 8:		dst = &appPidControl::pidSettings.Kd;		break;// kd
		case 9:		dst = &appPidControl::pidSettings.Period;	break;// Period
		case 10:	dst = &appPidControl::pidSettings.bias;		break;// Bias

		case 18:	dst = &devMotorControl::Xfer.Pos.M;			break;// Xfer_Pos_M
		case 19:	dst = &devMotorControl::Xfer.Pos.C; 		break; // Xfer_Pos_C
		case 20:	dst = &devMotorControl::Xfer.Neg.M;			break;// Xfer_Neg_M
		case 21:	dst = &devMotorControl::Xfer.Neg.C; 		break; // Xfer_Neg_C
		default:	retVal = NULL; /* These are not writable */ break;
	}

	//One of two things happened... either we have written the value already or we have been given a pointer to the value to write.
	if ((retVal == NULL) && (dst != NULL))
	{
		//We have a pointer to a float, but not one to a string yet
		*dst = finalValue;
		retVal = stdUtils::floatToStr(*dst, 2);
	}

	return retVal;
}

/*******************************************************************************

We want to check the validity of every parameter/index passed as well as the
values to be written (if applicable) and only respond if we are certain that
they are all within the array and readable/writable (as required).

This will require that the name of the parameter is copied into a tmp variable
for it to be compared

	get <param_1>,<param_2>,...,<param_n>
	set <param_1>=<value_1>,<param_2>=<value_2>,...,<param_n>=<value_n>

Returns true if all is good.
Will respond with an error message upond the first invalid parameter found
and return false

 *******************************************************************************/
bool devComms::isRdSettingsValid(char * paramString, bool absolute)
{
	int paramIndex = 0;
	char * thisParam= paramString;
	char curParam[21];  // The tmp buff for copying data closer to home (into SRAM)

	while (thisParam)
	{
		//PrintF("Checking \"%s\" (%02X)\n", paramString, rd_wr);
		//PrintF("Validate GET \"%s\"\n", curParam);

		//We need to work with the current parameter in the tmp buffer.
		StrCopyToChar(curParam, 20, thisParam, PARAM_DELIMETER);

		//Is this parameter referenced by index or by name
		paramIndex = (stdUtils::isNaturalNumberStr(curParam))? atoi(curParam) : getParamIndex(curParam);

		//Is this name/index valid (within the scope of the array)?
		if ((paramIndex + 1) >= (sizeof(SettingsArray)/sizeof(ST_SETTING_ITEM)))
		{
			//Nope, we ran past the end of our settings array.
			CmdResponseError(905, stdUtils::TmpStrPrintf(TxPayloadBuff, TMP_STR_BUFF_SIZE, "%s", curParam));
			return false;
		}

		//Is this parameter readable (PARAM_READABLE)
		if ((SettingsArray[paramIndex].rdwr & PARAM_READABLE) != PARAM_READABLE)
		{
			//No, respond with an error
			CmdResponseError(906, stdUtils::TmpStrPrintf(TxPayloadBuff, TMP_STR_BUFF_SIZE, "%s", curParam));
			return false;
		}

		//Move on to the next one. This call will find the next parameter's start without ammending the string
		thisParam = stdUtils::nextWord(thisParam, false, PARAM_DELIMETER);
	}

	//No issues picked up... we are all good.
	return true;

}


/*******************************************************************************

We want to check the validity of every parameter/index passed as well as the
values to be written (if applicable) and only respond if we are certain that
they are all within the array and readable/writable (as required).

This will require that the name of the parameter is copied into a tmp variable
for it to be compared

	get <param_1>,<param_2>,...,<param_n>
	set <param_1>=<value_1>,<param_2>=<value_2>,...,<param_n>=<value_n>

Returns true if all is good.
Will respond with an error message upond the first invalid parameter found
and return false

 *******************************************************************************/
bool devComms::isWrSettingsValid(char * paramString,bool absolute)
{
	int paramIndex = 0;
	char * thisParam= paramString;
	float min, max, val;
	char curParam[21];  // The tmp buff for copying data closer to home (into SRAM)
	char * curValue;  // The tmp buff for copying data closer to home (into SRAM)

	while (thisParam)
	{
		//PrintF("Checking \"%s\" (%02X)\n", paramString, rd_wr);

		//We need to work with the current parameter in the tmp buffer.
		StrCopyToChar(curParam, 20, thisParam, PARAM_DELIMETER);

		//If this parameter-value pair, it contains a ":" , we can null terminate there.
		//We work with the current parameter value directly from the tmp buffer.
		curValue = stdUtils::nextWord(curParam, true, VALUE_DELIMETER);
		if (curValue == NULL)
		{
			//No value to set??
			CmdResponseError(907, stdUtils::TmpStrPrintf(TxPayloadBuff, TMP_STR_BUFF_SIZE, "%s", curParam));
			return false;
		}

		//Is this parameter referenced by index or by name
		paramIndex = (stdUtils::isNaturalNumberStr(curParam))? atoi(curParam) : getParamIndex(curParam);

		//Is this name/index valid (within the scope of the array)?
		if ((paramIndex + 1) >= (sizeof(SettingsArray)/sizeof(ST_SETTING_ITEM)))
		{
			//Nope, we ran past the end of our settings array.
			CmdResponseError(908, stdUtils::TmpStrPrintf(TxPayloadBuff, TMP_STR_BUFF_SIZE, "%s", curParam));
			return false;
		}

		//PrintF("Validate SET%s \"%s\" <- \"%s\"\n", ((absolute)? "A" : "R"), curParam, curValue);

		//Is this parameter writable (PARAM_WRITABLE)
		if ((SettingsArray[paramIndex].rdwr&PARAM_WRITABLE) != PARAM_WRITABLE)
		{
			//No, respond with an error
			CmdResponseError(909, stdUtils::TmpStrPrintf(TxPayloadBuff, TMP_STR_BUFF_SIZE, "%s", curParam));
			return false;
		}
		//Type check... the good news is that all the writable parameters are floats.
		if (!stdUtils::isFloatStr(curValue))
		{
			//Not a float str, respond with an error
			CmdResponseError(910, stdUtils::TmpStrPrintf(TxPayloadBuff, TMP_STR_BUFF_SIZE, "%s:%s", curParam, curValue));
			return false;
		}
		val = atof((const char *)getParamValueStr(paramIndex));
		val = stdUtils::FloatMathStr(curValue, val, absolute);

		//Is the value within the min an max ranges
		if ((SettingsArray[paramIndex].Min != NULL) && (val < stdUtils::FloatMathStr(SettingsArray[paramIndex].Min, 0, true)))
		{
			CmdResponseError(911, stdUtils::TmpStrPrintf(TxPayloadBuff, TMP_STR_BUFF_SIZE, "%s:%s:%s", curParam, SettingsArray[paramIndex].Min, stdUtils::floatToStr(val, 3)));
			return false;
		}
		if ((SettingsArray[paramIndex].Max != NULL) && (val > stdUtils::FloatMathStr(SettingsArray[paramIndex].Max, 0, true)))
		{
			CmdResponseError(912, stdUtils::TmpStrPrintf(TxPayloadBuff, TMP_STR_BUFF_SIZE, "%s:%s:%s", curParam, SettingsArray[paramIndex].Max, stdUtils::floatToStr(val, 3)));
			return false;
		}

		//Move on to the next one. This call will find the next parameter's start without ammending the string
		thisParam = stdUtils::nextWord(thisParam, false, PARAM_DELIMETER);
	}

	//No issues picked up... we are all good.
	return true;

}

/*******************************************************************************

Copies a string from the src to the dst until any delimeter or a null pointer
has been reached. Max string length to copy is also specified.
Returns the number of characters copied

 *******************************************************************************/
/*
int devComms::StrCopyToDelim(char *dst, int len, const char *src)
{
	char * isdelim;
	int cnt = 0;

	while ((src[cnt]) && (cnt < len))
	{
		isdelim = strchr(DelimeterStr, src[cnt]);
		if (isdelim)
			return cnt;
		dst[cnt] = src[cnt];
		dst[++cnt] = 0;
	}
	return cnt;
}
*/
/*******************************************************************************

Copies a string from the src to the dst until a specified char or a null pointer
has been reached. Max string length to copy is also specified.
Returns the number of characters copied

 *******************************************************************************/
int devComms::StrCopyToChar(char *dst, int len, const char *src, char delim)
{
	int cnt = 0;

	while ((src[cnt] != NULL) && (cnt < len) && (src[cnt] != delim))
	{
		dst[cnt] = src[cnt];
		dst[++cnt] = 0;
	}
	return cnt;
}
/*******************************************************************************

Returns the index in the settings array of the parameter passed to the function.
The parameter must be null-terminated.

 *******************************************************************************/
int devComms::getParamIndex(char * thisParam)
{
	//Start at the beginning of the settings array.
	int paramIndex = 0;

	//Check each and every parameter name in the settings array.
	while (SettingsArray[paramIndex].Name != NULL)
	{
		//PrintF("Comparing %s with %s", SettingsArray[paramIndex].Name, thisParam);
		if (strcasecmp(SettingsArray[paramIndex].Name, thisParam) == NULL)
		{
			//PrintF("....Success!");
			//We found the setting we want, now you can return this index;
			return paramIndex;
		}
		//PrintF("....Fail");

		//Try the next parameter
		paramIndex++;
	}

	return paramIndex;
}
/*******************************************************************************

Returns the number of parameeters found starting at the passed pointer, which
is considered to be the first parameter.

 *******************************************************************************/
int devComms::countParams(char * thisParam)
{
	int cnt = 0;

	//While we have a valid pointer...
	while (thisParam)
	{
		//Start counting...
		cnt++;
		//Move on to the next parameter.
		thisParam = stdUtils::nextWord(thisParam, false, PARAM_DELIMETER);
	}

	return cnt;
}

/*******************************************************************************

Returns the parameter string at the specifed index...0-based index

 *******************************************************************************/
char * devComms::getParamAtIndex(char * thisParam, int index)
{
	int cnt = 0;

	//While we have a valid pointer...
	while (thisParam)
	{
		//is this the index we have to return?
		if (cnt == index)
			return thisParam; //Yes

		//NO, so we move on to the next parameter.
		thisParam = stdUtils::nextWord(thisParam, false, PARAM_DELIMETER);
		//Remember to increment your counter...
		cnt++;
	}

	return NULL;
}

/*******************************************************************************

Returns the parameter following the one that has been passed

 *******************************************************************************/
char * devComms::getNextParam(char * thisParam)
{
	return stdUtils::nextWord(thisParam, false, PARAM_DELIMETER);
}


/*******************************************************************************

Finds the parameters in the passed string (seperated by spaces)
Returns the number found.

 *******************************************************************************/
/*
int devComms::paramsParse(char * paramStr, bool terminate)
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
*/
/*******************************************************************************

Pops the next paramater out of the "stack"

 *******************************************************************************/
/*
char * devComms::getNextParam(void)
{
char * tmpVal = _paramStr[0];

	for (int i = 0; i < 4; i++)
		_paramStr[i] = _paramStr[i + 1];
	_paramStr[4] = NULL;

	return tmpVal;
}
*/

#endif /* CONSOLE_MENU */

#undef EXT
/*************************** END OF FILE *************************************/
