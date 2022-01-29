/*****************************************************************************

devConsole.h

Include file for devConsole.c

******************************************************************************/
#ifndef __DEVCONSOLE_H__
#define __DEVCONSOLE_H__


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

#ifdef CONSOLE_MENU

#define CONSOLE_RX_BUFF     80 /* must be able to store the max size of string. */

#define iPrintF(traceflags, fmt, ...) devConsole::_SerialPrintf(traceflags, PSTR(fmt), ##__VA_ARGS__)
#define PrintF(fmt, ...) devConsole::_SerialPrintf(PSTR(fmt), ##__VA_ARGS__)

/******************************************************************************
Macros
******************************************************************************/

/******************************************************************************
Struct & Unions
******************************************************************************/
typedef struct
{
  char RxBuff[CONSOLE_RX_BUFF + 1];  // The rx buff for data for the console.
  int InPtr;                        // The index into the rx buff.
  //bool Overflow;
}ST_CONSOLE;

typedef struct
{
//	void * OwnerObj;
	char * Tag;
	char * Command;
	void (*Func)(void);
	PROGMEM char *  HelpStr;
	void * Prev;
	void * Next;
}ST_CONSOLE_LIST_ITEM;

/******************************************************************************
variables
******************************************************************************/
//EXT STATIC ST_TIMER stDev_Print_Timer;

//EXT BYTE DoMutex;

/******************************************************************************
Class definition
******************************************************************************/

namespace devConsole
{
    bool Init(unsigned long baud, byte config);

    void _SerialPrintf(const char *fmt, ...);
	void _SerialPrintf(int traceflags, const char *fmt, ...);

	bool addMenuItem(char * ptrTag, ST_CONSOLE_LIST_ITEM * menuItem);;
	bool addMenuItem(ST_CONSOLE_LIST_ITEM * menuItem);

	bool Read(void);
	void ParseLine(void);

	int paramsParse(char * paramStr, bool terminate);
    int paramCnt(void);
    char * getParam(int index);
    char * getNextParam(void);

	//void _SerialPrintf(int flags, const char *fmt, ...);
	//void DoNothing(int flags, const char *fmt, ...);
	bool GetTrace(int flagIndex);
	int GetTraceIndex(char * traceName);
	void SetTrace(int flagIndex);
	void ClearTrace(int flagIndex);
	char * GetTraceName(int flagIndex);
    void menuPrintHelp();
    void menuDumpMem();
    void menuTogglePrintFlags(void);
	void DumpMem(int Flags, void * Src, unsigned long Address, int Len);
#ifdef MAIN_DEBUG
	void menuTogglePin(void);
#endif /* MAIN_DEBUG */

};

#endif /* CONSOLE_MENU */

#endif /* __DEVCONSOLE_H__ */

/****************************** END OF FILE **********************************/
