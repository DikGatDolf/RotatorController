/*****************************************************************************

timers.h

Include file for timers.c

******************************************************************************/
#ifndef __TIMERUTILS_H__
#define __TIMERUTILS_H__


/******************************************************************************
includes
******************************************************************************/
#include "Arduino.h"

/******************************************************************************
definitions
******************************************************************************/
#ifdef __NOT_EXTERN__
#define EXT
#else
#define EXT extern
#endif /* __NOT_EXTERN__ */

/******************************************************************************
Macros
******************************************************************************/

/******************************************************************************
Struct & Unions
******************************************************************************/
// structs
typedef struct
{
  unsigned long msExpire;   // 1ms ~ 49 days.
  unsigned long msPeriod;   // 1ms ~ 49 days.
  bool Enabled;
  bool Expired;
} ST_MS_TIMER;

/******************************************************************************
variables
******************************************************************************/

/******************************************************************************
functions
******************************************************************************/
/******************************************************************************
Class definition
******************************************************************************/

namespace timerUtils
{
//  public:
	//devTimers(void);
    void msTimerStart(ST_MS_TIMER *t, unsigned long interval);
    void msTimerStop(ST_MS_TIMER *t);
    bool msTimerReset(ST_MS_TIMER *t);
    bool msTimerPoll(ST_MS_TIMER *t);
    bool msTimerEnabled(ST_MS_TIMER *t);
    unsigned long SecondCount(void);

//    void usTimerInit(unsigned long, void (*f)());
    void usTimerInit(unsigned long, double, void (*f)());
    void usTimerStart();
    void usTimerStop();
	void usTimer_overflow();

	extern unsigned long time_units;
	extern void (*func)();
	extern volatile unsigned long count;
	extern volatile char overflowing;
	extern volatile unsigned int tcnt2;
};


#endif /* __TIMERUTILS_H__ */

/****************************** END OF FILE **********************************/
