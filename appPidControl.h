/*****************************************************************************

appPidControl.h

Include file for appPidControl.c

******************************************************************************/
#ifndef __APPPIDCONTROL_H__
#define __APPPIDCONTROL_H__


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

/******************************************************************************
Macros
******************************************************************************/

/******************************************************************************
Struct & Unions
******************************************************************************/

typedef struct
{
	float Kp /* = 80.0 */;	// 1 Proportional Constant.
	float Ki /* =  0.4 */;		// 2 Integral Constant.
	float Kd /* = 2.0 */;		// 3 Derivative Constant.
	float Period /* = 0.01 */;// 4 execution time of PID loop in s

	float bias /* = 0 */;		// 5 bias - optional, but small value prevents the output to be 0;

	float Target /* = 0 */;	/* 6 The point where we want to move to (in degrees) */
	float Position;		/* 7 The point where we are now (in degrees) */

	float MaxAccel /*= 18.0*/;/* 8 The maximum acceleration (degrees/s/s) we want to reach */
	float MaxSpeed /* = MOTOR_SPD_ABS_MAX*/;		/* 9 The maximum absolute speed (degrees/second) we want to reach
	 	 	 	 	 	 	 This value is limited to +/-36 degrees/second */

	float MinSpeed /* = 1.0*/;	/* 10 The Min absolute speed (degrees/second) from which
								we can stop the Motor
								This value is limited to MaxSpeed degrees/second */

	float Speed;		/* The Speed set on the DAC by the PID */

	float error;		// err = Expected Output - Actual Output ie. error;
	float intError;		// int from previous loop + err; ( i.e. integral error )
	float derError;		// err - err from previous loop; ( i.e. differential error)


	bool  Enable;		// Proptional Constant.

	float startPos;
	unsigned long startTime;
	bool aiming;

	float TimeToTarget;
}ST_PID;

/******************************************************************************
variables
******************************************************************************/

/******************************************************************************
functions
******************************************************************************/
namespace appPidControl
{
	EXT ST_PID pidSettings;
	EXT byte ControlState;

	bool Init(void);
	bool Enabled(void);
	bool GotoPos(float newPos);
	void Start(void);
	void Stop(void);
	bool PID_Process(void);
	bool ControlStateHandler(void);
	void StartCalibration(void);
	void menuCmd(void);
}
#endif /* __APPPIDCONTROL_H__ */

/****************************** END OF FILE **********************************/
