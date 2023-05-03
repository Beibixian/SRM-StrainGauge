/**
 * @file KlabVector.h
 * @brief This is a direct-quadrature-zero transfer function for SRM.
 * @warning CAN ONLY BE USED FOR SRM.
 * @details
 * Rotational speed of the reference frame is twice faster than normal AC motors.
 * For more information about the SRM vector gateControl, please refer to the paper below.
 * @see N. Nakao and K. Akatsu, “Vector gateControl specialized for switched reluctance motor drives�?
 * @author Shou Qiu
 * @version 1.0
 * @date 2022-08-10
 *
 * @copyright Copyright (c) 2022 Tokyo Institute of Technology, Chiba and Kiyota Lab
 */

#ifndef KLAB_VECTOR_h_
#define KLAB_VECTOR_h_
#include <mwio4.h>

#define SQRT_2OVER3 ((FLOAT32)0.816496580927726)
#define SQRT_1OVER2 ((FLOAT32)0.707106781186547)
#define SQRT_1OVER3 ((FLOAT32)0.577350269189625)
#define PI_4 ((DOUBLE64)12.566370614359172)
#define PI_2 ((DOUBLE64)6.283185307179586)
#define PI_2OVER3 ((DOUBLE64)2.094395102393195)
#define PI_4OVER3 ((DOUBLE64)4.188790204786390)
#define DEG2RADCOEFF ((DOUBLE64)0.01745329251994)


/** Structure contains six electrical paremeters {u,v,w,d,q,zero}. */
typedef struct
{
    FLOAT32 u, v, w;    /** Electric values on uvw coordinate */
    FLOAT32 d, q, zero; /** Electric values on dq0 coordinate */
} ELECTRIC_VALUE;

/** Structure contains seven rotation paremeters {abz,theta,sin,cos,timer,omega,rpm} */
typedef struct
{
    INT32 abz;     /** Count data of ABZ counter, integer in 0-4095 [INT32] */
    FLOAT32 theta; /** Electric angle [rad] */
    FLOAT32 sin;   /** sin(theta) */
    FLOAT32 cos;   /** cos(theta) */
    INT32 timer;   /** Count data of free run counter on PEV board, integer in 0-1e9 [INT32] */
    FLOAT32 omega; /** Electric rotational speed [rad/s] */
    FLOAT32 rpm;   /** Mechanical rotational speed [revolution per min] */
} ROTATE_VALUE;

/** Structure contains swichting duty {dutyU,dutyV,dutyW,armU,armV,armW} */
typedef struct
{
    FLOAT32 dutyU, dutyV, dutyW; /** Duty of uvw phase */
    FLOAT32 armU, armV, armW;    /** The state of negative arm for uvw phase */
} SWITCH_DUTY;

extern void KLAB_uvw2dq0(ELECTRIC_VALUE *elecValue, ROTATE_VALUE *rotValue);
extern void KLAB_dq02uvw(ELECTRIC_VALUE *elecValue, ROTATE_VALUE *rotValue);
extern void KLAB_speedCalc_update_speed(ROTATE_VALUE *rotValue, INT32 timerCount);
extern void KLAB_inverter_output_duty(ELECTRIC_VALUE *volValue, SWITCH_DUTY *armDuty, INT32 dcVoltage);

#endif /* KLAB_VECTOR_h_ */
