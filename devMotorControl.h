/*
/*****************************************************************************

devMotorSpeedDrive.h

Include file for devMotorSpeedDrive.c

******************************************************************************/
#ifndef __DEVMOTORSPEEDDRIVE_H__
#define __DEVMOTORSPEEDDRIVE_H__

/******************************************************************************
Includes
******************************************************************************/
#include "Arduino.h"
#include "defines.h"

//#include "halAMT203.h"
#include "halTLC5615.h"

/******************************************************************************
definitions
******************************************************************************/
#ifdef __NOT_EXTERN__
#define EXT
#else
#define EXT extern
#endif /* __NOT_EXTERN__ */

/* IMPORTANT: When you need to turn the rotation direction of the motor around,
 * this "MOTOR_CONTROL_REV_INPUT_STATE" is where you are going to change it
 * You should interpret this Flag as follows:
 * The State of the REVERSE input signal on the Motor Controller determines
 * what denotes "forward" rotation... i.e.
 *
 * 		a TRUE means the REV Signal input on the motor controller
 * 			must be ACTIVE for Forward rotation
 *
 * 		a FALSE means the REV Signal input on the motor controller
 * 			must be INACTIVE for Forward rotation  */
#define MOTOR_CONTROL_REV_INPUT_STATE	false

#define MOTOR_FWD	MOTOR_CONTROL_REV_INPUT_STATE
#define MOTOR_REV	(!MOTOR_CONTROL_REV_INPUT_STATE)

#define MOTOR_SPD_ABS_MAX 		 36.0	/* degrees per second */
#define MOTOR_SPD_ABS_MAX_STR 	"36.0"	/* degrees per second */

#define MOTOR_SPD_ABS_MIN 		 0.5	/* degrees per second */
#define MOTOR_SPD_ABS_MIN_STR 	"0.5"	/* degrees per second */

#define AMT203_QUAD_PPR		4096 /* We are counting both edges, So two
									1024 ppr signals offset by 90 degrees
									results in 4096 edges.*/

#define AMT203_QUAD_PPR_RANGE		(2*4096) /* This MUST equate to a multiple of 360 degrees */

#define AMT203_QUAD_PPR_NEG_OFFSET 	(-2048) /* Half a rotation (180 degrees) is only 2048 pulses*/

#define AMT203_QUAD_PPR_WRAP_MAX	(AMT203_QUAD_PPR_RANGE + AMT203_QUAD_PPR_NEG_OFFSET)
#define AMT203_QUAD_PPR_WRAP_MIN	(AMT203_QUAD_PPR_NEG_OFFSET)

#define MOTOR_POS_WRAP_MAX			( 540.0) /* degrees */
#define MOTOR_POS_WRAP_MAX_STR		 "540.0" /* degrees */
#define MOTOR_POS_WRAP_MIN			(-540.0) /* degrees */
#define MOTOR_POS_WRAP_MIN_STR		"-540.0" /* degrees */

#define MOTOR_POS_INCREMENT_DEG 	(360.0/AMT203_QUAD_PPR) /* degrees */
#define MOTOR_SPD_INCREMENT_FLT		(((float)MOTOR_SPD_ABS_MAX)/((float)TLC5615_MAX_OUTPUT_VAL))

#define XFER_EQ_CLOCKWISE_M			( 1.1598)
#define XFER_EQ_CLOCKWISE_M_STR		 "1.1598"
#define XFER_EQ_CLOCKWISE_C			( 1.1071)
#define XFER_EQ_CLOCKWISE_C_STR		 "1.1071"
#define XFER_EQ_ANTICLOCK_M			( 1.1417)
#define XFER_EQ_ANTICLOCK_M_STR		 "1.1417"
#define XFER_EQ_ANTICLOCK_C			( 1.3754)
#define XFER_EQ_ANTICLOCK_C_STR		 "1.3754"

#if MOTOR_CONTROL_REV_INPUT_STATE
	#define ROTATE_FORWARD 		(-1 )	/* Counter-clockwise rotation */
	#define ROTATE_BACKWARD		( 1 )	/* Clockwise rotation */

	#define XFER_EQ_POS_M		( XFER_EQ_CLOCKWISE_M )
	#define XFER_EQ_POS_C		(-XFER_EQ_CLOCKWISE_C )
	#define XFER_EQ_NEG_M		( XFER_EQ_ANTICLOCK_M )
	#define XFER_EQ_NEG_C		( XFER_EQ_ANTICLOCK_C )
	#define XFER_EQ_POS_M_STR	( XFER_EQ_CLOCKWISE_M_STR )
	#define XFER_EQ_POS_C_STR	("-"XFER_EQ_CLOCKWISE_C_STR )
	#define XFER_EQ_NEG_M_STR	( XFER_EQ_ANTICLOCK_M_STR )
	#define XFER_EQ_NEG_C_STR	( XFER_EQ_ANTICLOCK_C_STR )
#else
	#define ROTATE_FORWARD 		( 1 )	/* Counter-clockwise rotation */
	#define ROTATE_BACKWARD		(-1 )	/* Clockwise rotation */

	#define XFER_EQ_POS_M		( XFER_EQ_ANTICLOCK_M )
	#define XFER_EQ_POS_C		(-XFER_EQ_ANTICLOCK_C )
	#define XFER_EQ_NEG_M		( XFER_EQ_CLOCKWISE_M )
	#define XFER_EQ_NEG_C		( XFER_EQ_CLOCKWISE_C )
	#define XFER_EQ_POS_M_STR	( XFER_EQ_ANTICLOCK_M_STR )
	#define XFER_EQ_POS_C_STR	("-"XFER_EQ_ANTICLOCK_C_STR )
	#define XFER_EQ_NEG_M_STR	( XFER_EQ_CLOCKWISE_M_STR )
	#define XFER_EQ_NEG_C_STR	( XFER_EQ_CLOCKWISE_C_STR )
#endif

typedef struct
{
	float M;
	float C;
}ST_LINEAR;

typedef struct
{
	ST_LINEAR Pos;
	ST_LINEAR Neg;
}ST_XFER;

/******************************************************************************
Macros
******************************************************************************/

/******************************************************************************
Struct & Unions
******************************************************************************/

/******************************************************************************
variables
******************************************************************************/

/******************************************************************************
Class definition
******************************************************************************/

namespace devMotorControl
{
	EXT ST_XFER Xfer;

	bool Init(void);
    float ConvertSpeed_WR_degs(float spdDegreePerSecond);
    float ConvertSpeed_RD_degs(float spdDegreePerSecond);
    void Stop(void);
    void ResetSpeedParams(void);
    void KillMotor(void);
    float SetSpeed_degs(float);
    int SetSpeed_abs(int spdAbsolute);
    float GetSpeed_ENC(void);
    float GetSpeed_DAC(void);
    float GetSpeed_AVG(void);
    float GetPosition(void);
    float SetPosition(float newPos);
    float GetRealPosition(void);
    float GetZeroOffset(void);
    bool IsAtRealZero(void);
    void TimerInterruptCallback(void);

#ifdef CONSOLE_MENU
    void PrintSpeedAndPosition(void);
    void menuCmd(void);
#endif /* CONSOLE_MENU */
};

#endif /* __DEVMOTORSPEEDDRIVE_H__ */

/****************************** END OF FILE **********************************/
