/*****************************************************************************

devComms.h

Include file for devConsole.c

******************************************************************************/
#ifndef __DEVCOMMS_H__
#define __DEVCOMMS_H__


/******************************************************************************
includes
******************************************************************************/
#include "defines.h"


/******************************************************************************
definitions
******************************************************************************/
#ifdef __NOT_EXTERN__
#define EXT
#else
#define EXT extern
#endif /* __NOT_EXTERN__ */

#ifndef CONSOLE_MENU

#define PrintF(fmt, ...) devComms::_SerialPrintf(PSTR(fmt), ##__VA_ARGS__)
#define iPrintF(traceflags, fmt, ...) devComms::DoNothing(traceflags, PSTR(fmt), ##__VA_ARGS__) /* {	}  */

#define COMMS_RX_BUFF_LEN     80 /* must be able to store the max size of string. */

#define PARAM_READABLE		0x01
#define PARAM_WRITABLE		0x02

#define PARAM_DELIMETER		','
#define VALUE_DELIMETER		':'

/******************************************************************************
Macros
******************************************************************************/

/******************************************************************************
Struct & Unions
******************************************************************************/
typedef struct
{
  char RxBuff[COMMS_RX_BUFF_LEN + 1];  // The rx buff for data for the console.
  int InPtr;                        // The index into the rx buff.
  byte CRC_rx;
  byte CRC_calc;
  int MsgState;		/* msIDLE, msHEADER, msMESSAGE, msCHECKSUM_H, msCHECKSUM_L, msEND */
}ST_COMMS;

typedef struct
{
	PROGMEM char * Name;
	byte rdwr;
	PROGMEM char * Min;
	PROGMEM char * Max;
	PROGMEM char * Default;
}ST_SETTING_ITEM;

typedef struct
{
	int ErrCode;
	PROGMEM char *  MsgStr;
}ST_ERROR_MSG;

/******************************************************************************
variables
******************************************************************************/
//EXT STATIC ST_TIMER stDev_Print_Timer;


/******************************************************************************
Class definition
******************************************************************************/

namespace devComms
{
    bool Init(unsigned long baud, byte config);
	void _SerialPrintf(const char *fmt, ...);

	void DoNothing(int traceflags, const char *fmt, ...);

	void Read(void);
	int ParseByteForHeader(byte rxData);
	int ParseByteForPayload(byte rxData);
	int ParseByteForTail(byte rxData);
	void ParseLine(void);

	void CmdResponseOK(const char * msg);
	void CmdResponseError(int errCode, const char * msg);

	void readSetting(char * paramStr);
	void writeSetting(char * paramStr, bool absolute);

	char * getParamValueStr(int paramIndex);
	char * setParamValueStr(int paramIndex, float finalValue);

	bool isRdSettingsValid(char * paramString, bool absolute);
	bool isWrSettingsValid(char * paramString, bool absolute);
	//bool isSettingsValid(char * paramString, byte rd_wr, bool absolute);
	//int StrCopyToDelim(char *dst, int len, const char *src);
	int StrCopyToChar(char *dst, int len, const char *src, char delim);

	int countParams(char * thisParam);
	int getParamIndex(char * thisParam);
	char * getParamAtIndex(char * thisParam, int index);
	char * getNextParam(char * thisParam);

};

#endif /* CONSOLE_MENU */

#endif /* __DEVCOMMS_H__ */

/****************************** END OF FILE **********************************/
