/*
 * defaults.h
 *
 *  Created on: Apr 19, 2015
 *      Author: Kevin
 */

#ifndef DEFAULTS_H_
#define DEFAULTS_H_

// 10:44 AM
//#define DEFAULT_STEER_P 			(0.9f)
//#define DEFAULT_STEER_D 			(0.05f)
//#define DEFAULT_STEER_I 			(0.01f)
//
//#define DEFAULT_SPEED_MULT 			(0.0015f)
//#define DEFAULT_SPEED_CONT			(false)
//
//#define DEFAULT_THRESHOLD			(0.65f)
//
//#define DEFAULT_EDGECLIP_COUNT 		(5)
//
//#define DEFAULT_BE_STILL 			(false)
//
//#define DEFAULT_MIN_SPEED			(0.25f)
//
//#define DEFAULT_SPEEDCONT_POLICY	(SPEEDCONT_AVG_INVSQ)
//#define DEFAULT_STRATEGY			(WEIGHTED)
//
//#define DEFAULT_SS_PID              (true)
//#define DEFAULT_SS_P_COEFF          (0.05f)
//#define DEFAULT_SS_I_COEFF          (0.0f)
//#define DEFAULT_SS_D_COEFF          (0.04f)
//
//#define DEFAULT_TURN_THRESH         (0.05f)
//#define DEFAULT_LEFT_SETPOINT       (0.2f)
//#define DEFAULT_RIGHT_SETPOINT      (0.8f)
//#define DEFAULT_SETPOINT_DECAY      (0.995f)
//
//#define DEFAULT_SPEEDBOOST_DECAY    (0.9f)

// 10:44 AM
#define DEFAULT_STEER_P 			(1.5f)
#define DEFAULT_STEER_D 			(0.035f)
#define DEFAULT_STEER_I 			(0.0f)

#define DEFAULT_SPEED_MULT 			(0.5f)
#define DEFAULT_SPEED_CONT			(false)

#define DEFAULT_THRESHOLD			(0.30f)

#define DEFAULT_EDGECLIP_COUNT 		(5)

#define DEFAULT_BE_STILL 			(false)

#define DEFAULT_MIN_SPEED			(0.45f)

#define DEFAULT_SPEEDCONT_POLICY	(SPEEDCONT_WEIGHTED_LINEAR)
#define DEFAULT_STRATEGY			(SAFE)

#define DEFAULT_SS_PID              (true)
#define DEFAULT_SS_P_COEFF          (0.00f)
#define DEFAULT_SS_I_COEFF          (0.0f)
#define DEFAULT_SS_D_COEFF          (0.00f)

#define DEFAULT_TURN_THRESH         (0.15f)
#define DEFAULT_LEFT_SETPOINT       (0.5f)
#define DEFAULT_RIGHT_SETPOINT      (0.5f)
#define DEFAULT_SETPOINT_DECAY      (0.93f)

#define DEFAULT_SPEEDBOOST_DECAY    (0.8f)

#endif /* DEFAULTS_H_ */
