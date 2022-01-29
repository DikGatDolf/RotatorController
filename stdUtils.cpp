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
#include "Arduino.h"
#include <stdarg.h>

#define __NOT_EXTERN__
#include "stdUtils.h"
#undef __NOT_EXTERN__

#ifdef CONSOLE_MENU
	#include "devConsole.h"
#else
	#include "devComms.h"
#endif

/*******************************************************************************
local defines
 *******************************************************************************/
#define FMT_TO_STR_BUFF_SIZE 20

/*******************************************************************************
local variables
 *******************************************************************************/
//char tmpBuff[TMP_BUF_SIZE + 1];  // The tmp buff for copying data closer to home (into SRAM)
char convToStringBuff[FMT_TO_STR_BUFF_SIZE + 1];  // The tmp buff for copying data closer to home (into SRAM)


/*******************************************************************************

Finds the start of the next word after this one.
Returns NULL if nothing is found.

 *******************************************************************************/
char * stdUtils::nextWord(char * curWord, bool terminate)
{
char *nxtWord;
	//Let's start by checking if there is a space after this word?
	nxtWord = strchr(curWord, ' ');
	if (nxtWord)
	{
		if (terminate)
			*nxtWord = 0;

		//Great... now we skip the spaces until we find something else
		do
		{
			nxtWord++;
		}
		while (*nxtWord == ' ');

		if (*nxtWord == NULL)
			return NULL;
	}

	return nxtWord;
}

/*******************************************************************************

Finds the start of the next word after this one.
Returns NULL if nothing is found.

 *******************************************************************************/
char * stdUtils::nextWord(char * curWord, bool terminate, char delim)
{
char *nxtWord;
	//Let's start by checking if there is a space after this word?
	nxtWord = strchr(curWord, delim);
	if (nxtWord)
	{
		if (terminate)
			*nxtWord = 0;

		//Great... now we skip the spaces until we find something else
		do
		{
			nxtWord++;
		}
		while (*nxtWord == ' ');

		if (*nxtWord == NULL)
			return NULL;
	}

	return nxtWord;
}
/*******************************************************************************

Returns the byte value (lower nibble only) represented by a single HEX character

 *******************************************************************************/
byte stdUtils::charToNibble(char hexDigit)
{
	byte retVal = 0x00;
	if ((hexDigit >= '0') && (hexDigit <= '9'))
		retVal = (hexDigit - '0');
	else if ((hexDigit >= 'A') && (hexDigit <= 'F'))
		retVal = 0x0A + (hexDigit - 'A');
	else if ((hexDigit >= 'a') && (hexDigit <= 'f'))
		retVal = 0x0A + (hexDigit - 'a');

	return retVal;
}

byte stdUtils::hexToByte(char * hexDigits)
{
	byte retVal;

	retVal = charToNibble(*hexDigits);
	retVal <<= 4;
	hexDigits++;
	retVal |= charToNibble(*hexDigits);

	return retVal;
}



int stdUtils::setFloatParam(float * param, float value, float min_Limit, float max_Limit)
{

	//within min/max limits?
	if ((value < min_Limit) || (value > max_Limit))
		return -2;

	//Excellent
	*param = value;
	return 0;
}



#ifdef CONSOLE_MENU
int stdUtils::setFloatParam(char * name, char * paramStr, char * valueStr, float * param, float min_Limit, float max_Limit)
{
float tmpFloat;
int retVal;

	//Correct Name?
	if (strcasecmp(paramStr, name) != NULL)
	return -1;

	//Value passed?
	if (valueStr == NULL)
	{
		//PrintF(" &%s = 0x%08lX\n", name,  (unsigned long)param);
		PrintF(" %s = % 7s\n\n", name,  stdUtils::floatToStr(*param, 3));
		return -2;
	}

	//tmpFloat = atof(valueStr);
	tmpFloat = FloatMathStr(valueStr, *param);

	//RVN - TODO - I am not convinced this setFloatParam() in the IF is correct....

	//within min/max limits?
	if ((tmpFloat < min_Limit) || (tmpFloat > max_Limit))
	{
		PrintF("Invalid value for %s - Value must be between %s", name,  stdUtils::floatToStr(min_Limit, 3));
		PrintF(" and %s\n\n", stdUtils::floatToStr(max_Limit, 3));
		return -3;
	}

	//Excellent
	*param = tmpFloat;
	return 0;
}

int stdUtils::setFloatParam(char * name, char * paramStr, char * valueStr, float * param)
{
float tmpFloat;

	//Correct Name?
	if (strcasecmp(paramStr, name) != NULL)
		return -1;

	//Value passed?
	if (valueStr == NULL)
	{
		//PrintF(" &%s = 0x%08lX\n", name,  (unsigned long)param);
		PrintF(" %s = % 7s\n\n", name,  stdUtils::floatToStr(*param, 3));
		return -2;
	}

	//Excellent
	//*param = atof(valueStr);//tmpFloat;
	*param = FloatMathStr(valueStr, *param);
	return 0;
}

float stdUtils::FloatMathStr(char * Src, float value)
{
	//First we want to check if our string is preceded by a '+', '-' or '=' sign
	if (Src[0] == '+')
		return (value + atof((const char *)&Src[1]));
	else if (Src[0] == '-')
		return (value - atof((const char *)&Src[1]));
	else if (Src[0] == '=')
	{
		if (Src[1] == '-')
			return (0 - atof((const char *)&Src[2]));
		else
			return atof((const char *)&Src[1]);
	}
	else if (Src[0] == NULL)
		return value;
	else
		return atof(Src);
}
#endif /* CONSOLE_MENU */

float stdUtils::FloatMathStr(char * Src, float value, bool absolute)
{
	//First we want to check if our string is preceded by a '+', '-' or '=' sign
	if (Src[0] == '-')
		return (((absolute)? 0 : value) - atof((const char *)&Src[1]));
	else if (Src[0] == NULL)
		return value;
	else if (Src[0] == '+')
		return (((absolute)? 0 : value) + atof((const char *)&Src[1]));
	else
		return (((absolute)? 0 : value) + atof((const char *)&Src[0]));
}
/*******************************************************************************

A dedicated sprintF for floats... to be used in all the other console prints

 *******************************************************************************/
//char * stdUtils::floatToStr(float fVal, int precision, char * dst, int maxlen)
char * stdUtils::floatToStr(float fVal, int decimalpoints)
{
	long int  multiplier_divider = 1;
	long int  workingVal;
	long int mod_delta = 0;
	char __fmt[FMT_TO_STR_BUFF_SIZE + 1];
	long int intVal;
	long int decVal;
	char negSign[2];

	//Start with an empty format string.
	__fmt[0] = NULL;
	negSign[0] = NULL;
	//signVal[1] = NULL;

	//Limit the requsted decimal points between 0 and 4
	if (decimalpoints < 0)
		decimalpoints = 0;	//"9.99999999" -> "10"
	else if (decimalpoints > 4)
		decimalpoints = 4;	//"9.99999999" -> "9.9999"

	//We need to get our multiplier right.
	for (int i = 0; i < decimalpoints; i++)
		multiplier_divider *= 10;

	// Calculate the 10x product
	workingVal = (long int)(fVal * 10 * ((float)multiplier_divider));

	//Do we need to round up or down?
	mod_delta = workingVal%10;
	if (mod_delta >= 5)
		//For values 5, 6, 7, 8, and 9, we round up
		workingVal += (10 - mod_delta);
	else //if (mod_delta >= 0)
		//For values 0, 1, 2, 3, and 4 we round down
		workingVal -= mod_delta;

	// Calculate the final product
	workingVal /= 10;

	__fmt[FMT_TO_STR_BUFF_SIZE] = NULL;
	//We are going to build up our format string as we go along
	strlcat(__fmt, "%s%ld", FMT_TO_STR_BUFF_SIZE);
	//				01234...
	//Are we planning on having a decimal point?
	if (decimalpoints > 0)
	{
		if (decimalpoints == 1)
		{
			strlcat(__fmt, ".%ld", FMT_TO_STR_BUFF_SIZE);
			//		   	 ...567
		}
		else
		{
			strlcat(__fmt, ".%0_ld", FMT_TO_STR_BUFF_SIZE);
			//		   	 ...56789A
			__fmt[8] = '0' + decimalpoints;
		}
	}

	//Get the whole number
	intVal = abs(workingVal / multiplier_divider);
	//Get the decimal number
	decVal = abs(workingVal % multiplier_divider);
	//determine if we need a "-" sign
	if (((intVal > 0) || (decVal > 0)) && (fVal < 0))
		strlcat(negSign, "-", 2);

	//And now we print our buffer?
	snprintf(convToStringBuff, FMT_TO_STR_BUFF_SIZE, (const char*)__fmt, negSign, intVal, decVal);

	return convToStringBuff;
}
/*******************************************************************************

Prints the status word in binary string format

 *******************************************************************************/
char * stdUtils::SatusWordBinStr()
{
	strncpy(convToStringBuff, "--", FMT_TO_STR_BUFF_SIZE);
	strlcat(convToStringBuff, (GetStatus(statusCALIB_BUSY)? "C": "-"), FMT_TO_STR_BUFF_SIZE);
	strlcat(convToStringBuff, (GetStatus(statusPID_DONE)? 	"D": "-"), FMT_TO_STR_BUFF_SIZE);
	strlcat(convToStringBuff, (GetStatus(statusPID_BUSY)? 	"B": "-"), FMT_TO_STR_BUFF_SIZE);
	strlcat(convToStringBuff, (GetStatus(statusDIRECTION)? 	"R": "-"), FMT_TO_STR_BUFF_SIZE);
	strlcat(convToStringBuff, (GetStatus(statusMOVING)? 	"M": "-"), FMT_TO_STR_BUFF_SIZE);
	strlcat(convToStringBuff, (GetStatus(statusOK)? 		"O": "-"), FMT_TO_STR_BUFF_SIZE);

	return convToStringBuff;
}
/*******************************************************************************

Provides the ability to create "dynamic" string up to a maximum of len characters.
Returns a pointer to a specified buffer with a max length of len characters

 *******************************************************************************/
char * stdUtils::TmpStrPrintf(char *dst, int len, const char *fmt, ...)
{
FILE stdiostr;
va_list ap;

	va_start(ap, fmt);
	//vsprintf(TmpStringBuff, fmt, ap);
	vsnprintf(dst /*TmpStringBuff*/, len, fmt, ap);
	va_end(ap);
	return dst;
}

/*******************************************************************************

Provides the ability to create "dynamic" string up to a maximum of 20 characters.
Returns a pointer to the default buffer with a max length of 20 characters

 *******************************************************************************/
char * stdUtils::TmpStrPrintf(const char *fmt, ...)
{
FILE stdiostr;
va_list ap;

	va_start(ap, fmt);
	//vsprintf(TmpStringBuff, fmt, ap);
	vsnprintf(convToStringBuff, FMT_TO_STR_BUFF_SIZE, fmt, ap);
	va_end(ap);
	return convToStringBuff;
}

/*******************************************************************************

Checks if the string is a decimal number (i.e. -1.04, 10.7, 200.8, -23.5....

 *******************************************************************************/
bool stdUtils::isFloatStr(char * str)
{
	int cnt = 0;
	bool decimal = false; //Only allowed 1 decimal point;

	//Check every character until we reach a delimeter or a null-terminator

	//Allow a + or - only in the first position
	if ((str[cnt] == '-') || (str[cnt] == '+'))
		cnt++;

	while (str[cnt])
	{
		if (isdigit(str[cnt]) == 0)
		{
			//Not a digit... could be a decimal point
			if ((str[cnt] == '.') && (!decimal))
			{
				//Ok... let's keep on looking...
				decimal = true;
			}
			else
			{
				//Either a second decimal point or something else completely
				return false;
			}
		}
		cnt++;

	}
	return true;
}
/*******************************************************************************

Checks if the string is a natural number (i.e. 0, 1, 2,....

 *******************************************************************************/
bool stdUtils::isNaturalNumberStr(char * str)
{
	int cnt = 0;

	//Check every character until we reach a delimeter or a null-terminator
	while (*str)
	{
		if (isdigit(*str) == 0)
			return false;
		str++;

	}
	return true;
}
/*******************************************************************************

A dedicated sprintF for floats... to be used in all the other console prints

 *******************************************************************************/
/*char * stdUtils::floatToStr(float fVal, int precision, char * dst, int maxlen)
{
long int intVal;
long int decVal;
long int dividerVal = 1;
char * signVal;
char * __fmt = "%s%ld.%0_ld";
//			    0123456789A

	if ((precision < 1) || (precision > 4))
	{
		PrintF("Could not convert \"%d.%02d...\" to string\n",
				(int)(fVal),
				abs(((int)(fVal * 100)%100)));
		*dst = NULL;
		//*convToStringBuff = NULL;
	}
	else
	{
		__fmt[8] = '0' + precision;
		for (int i = 0; i < precision; i++)
			dividerVal *= 10l;

		intVal = abs((long int)(fVal));
		decVal = abs(((long int)(fVal * dividerVal)%dividerVal));
		//signVal = (((intVal > 0) || (decVal > 0)) && (fVal < 0))? "-" : "";
		if (((intVal > 0) || (decVal > 0)) && (fVal < 0))
			snprintf(dst, maxlen, (const char*)__fmt, "-", intVal, decVal);
		else
			snprintf(dst, maxlen, (const char*)__fmt, "", intVal, decVal);
	}
	return dst;
}
*/
/*******************************************************************************

An attempt to manage my outpts without the use of digitalWrite and a subsequent
f@#$ing about with clearing the interrupt flag

 *******************************************************************************/
void stdUtils::quickPinToggle(uint8_t pin, bool state)
{
//byte port;

	//PrintF("Toggle pin %d to %s\n", pin, (state)? "HI" : "LO");

	//Assign the correct port....
	if (pin < 8)
	{
		//port = PORTD;
		if (state)
			PORTD |= _BV(pin);
		else
			PORTD &= ~_BV(pin);
	}
	else if (pin < 14)
	{
		//port = PORTB;
		if (state)
			PORTB |= _BV(pin%8);
		else
			PORTB &= ~_BV(pin%8);
	}
	else if (pin < 19)
	{
		//port = PORTC;
		if (state)
			PORTC |= _BV(pin%14);
		else
			PORTC &= ~_BV(pin%14);
	}
}

/*******************************************************************************

An attempt to manage my outputs without the use of digitalWrite and a subsequent
f@#$ing about with clearing the interrupt flag

 *******************************************************************************/
int stdUtils::quickPinRead(uint8_t pin)
{
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);

	//if (port == NOT_A_PIN) return LOW;

	if (*portInputRegister(port) & bit) return HIGH;
	return LOW;
}

/*******************************************************************************

Returns one of the follwoing EDGE states:
#define INPUT_EDGE_NONE		0
#define INPUT_EDGE_FALLING	1
#define INPUT_EDGE_RISING	2

 *******************************************************************************/
int stdUtils::debounceInput(ST_PIN_DEBOUNCE * input, int level, int count)
{
	//Is this level the same as what was saved previously?
	if (input->CurrentState == level)
	{
		//Yup... nothing to debounce.
		input->DebounceCount = 0;
		return INPUT_EDGE_NONE;
	}

	//This means we have seen a change in level from our current "debounced" level
	input->DebounceCount++;

	//Has it been debounce "count" times?
	if (input->DebounceCount < count)
	{
		//We are still debouncing... no change to report.
		return INPUT_EDGE_NONE;
	}

	//Now we have a stable state... assign it to our current state.
	input->CurrentState = level;

	//A HIGH level implies a rising edge....
	// ...A LOW level implies a falling edge.
	return (level == HIGH)?
			INPUT_EDGE_RISING : INPUT_EDGE_FALLING;
}

/*******************************************************************************

Returns the average of all the values in the passed array

 *******************************************************************************/
unsigned long stdUtils::avgULong(volatile unsigned long * arr, int cnt)
{
unsigned long retVal = 0L;

	for (int i = 0; i < cnt; i++)
	{
		retVal += arr[i];
	}
	retVal /= cnt;

	return retVal;
}

/*******************************************************************************

Reports the amount of space available between the stack and the heap.
Calls to this function from different locations in the code will return a
different result.

 *******************************************************************************/
int stdUtils::freeRam ()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

/*******************************************************************************

Calculate the CRC for a null terminated string.

 *******************************************************************************/
byte stdUtils::crc8_str(const char *str) {
	char * data = str;
	byte crc = 0x00; //Start with a blank CRC.
	while (*data){
		byte extract = *data++;
		for (byte tempI = 8; tempI; tempI--) {
			byte sum = (crc ^ extract) & 0x01;
			crc >>= 1;
			if (sum) {
				crc ^= CRC_POLYNOMIAL;
			}
			extract >>= 1;
		}
	}
	return crc;
}

/*******************************************************************************

Calculate the CRC for a null terminated string with a starting seed CRC value

 *******************************************************************************/
byte stdUtils::crc8_str(byte crc_start, const char *str) {
	char * data = str;
	byte crc = crc_start; //Start with the seed CRC.
	while (*data){
		byte extract = *data++;
		for (byte tempI = 8; tempI; tempI--) {
			byte sum = (crc ^ extract) & 0x01;
			crc >>= 1;
			if (sum) {
				crc ^= CRC_POLYNOMIAL;
			}
			extract >>= 1;
		}
	}
	return crc;
}

/*******************************************************************************

Calculate the CRC for an N number of bytes.

 *******************************************************************************/
byte stdUtils::crc8_str_n(const byte *data, byte len) {
	byte crc = 0x00; //Start with a blank CRC.
	while (len--) {
		byte extract = *data++;
		for (byte tempI = 8; tempI; tempI--) {
			byte sum = (crc ^ extract) & 0x01;
			crc >>= 1;
			if (sum) {
				crc ^= CRC_POLYNOMIAL;
			}
			extract >>= 1;
		}
	}
	return crc;
}

/*******************************************************************************

Calculate the CRC for an N number of bytes with a starting seed CRC value

 *******************************************************************************/
byte stdUtils::crc8_str_n(byte crc_start, const byte *data, byte len) {
	byte crc = crc_start; //Start with a seed CRC.
	while (len--) {
		byte extract = *data++;
		for (byte tempI = 8; tempI; tempI--) {
			byte sum = (crc ^ extract) & 0x01;
			crc >>= 1;
			if (sum) {
				crc ^= CRC_POLYNOMIAL;
			}
			extract >>= 1;
		}
	}
	return crc;
}

/*******************************************************************************

Calculate the CRC for a single bytes
Why only a single byte CRC. Because a single byte will serve its purpose (of
ensuring the integrity of the message) very well while still allowing us not
to have to worry about byte-endianness

 *******************************************************************************/
byte stdUtils::crc8(byte crc_start, byte data) {
	byte crc = crc_start; //Start with a blank CRC.
	for (byte tempI = 8; tempI; tempI--) {
		byte sum = (crc ^ data) & 0x01;
		crc >>= 1;
		if (sum) {
			crc ^= CRC_POLYNOMIAL;
		}
		data >>= 1;
	}
	return crc;
}
/*******************************************************************************

Sets selected flag(s) mask

 *******************************************************************************/
void stdUtils::SetStatus(byte statusMask)
{
	SystemStatus |= statusMask;
}

/*******************************************************************************

Clears selected Flag(s) mask

 *******************************************************************************/
void stdUtils::ClearStatus(byte statusMask)
{
	SystemStatus &= (~statusMask);
}

/*******************************************************************************

Toggles the selected status Flag(s) mask based on the boolean value of the flag

 *******************************************************************************/
void stdUtils::ToggleStatus(byte statusMask, bool flag)
{
	if (flag)
		SystemStatus |= statusMask;
	else
		SystemStatus &= (~statusMask);
}
/*******************************************************************************

Returns true or false based on the flag status

 *******************************************************************************/
bool stdUtils::GetStatus(byte statusMask)
{
	return (SystemStatus & statusMask)? true : false;
}

/*******************************************************************************

Copies a string from PROGMEM to a RAM buffer for use in functions.
The validity of the data in the buffer is by no means guaranteed once the function 
has returned 

 *******************************************************************************/
/*const char * _FAR(const char *sptr)
{
	if (strlen_P(sptr) > TMP_BUF_SIZE)
	{
		PrintF("TMP Buffer Overrun!!\n");
		strncpy_P(tmpBuff, sptr, TMP_BUF_SIZE);
	}
	else
	{
		strcpy_P(tmpBuff, sptr);
	}
	return tmpBuff;
}*/

//#define Printf SerialPrintf

/*
 * To use it for debugging define something like this:
 */
//#define Debug SerialPrintf

/*
 * And to disable it (will eliminate all the Debug() print calls)
 */
//#define Debug NullPrintf

#undef EXT
/*************************** END OF FILE *************************************/
