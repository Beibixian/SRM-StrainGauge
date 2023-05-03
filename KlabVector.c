/**
 * @file KlabVector.c
 * @brief This is a direct-quadrature-zero transfer function for SRM.
 * @warning CAN ONLY BE USED FOR SRM.
 * @details
 * Rotational speed of the reference frame is twice faster than normal AC motors.
 * For more information about the SRM vector gateControl, please refer to the paper below.
 * @see N. Nakao and K. Akatsu, “Vector gateControl specialized for switched reluctance motor drives”
 * @author Shou Qiu
 * @version 1.0
 * @date 2022-08-10
 *
 * @copyright Copyright (c) 2022 Tokyo Institute of Technology, Chiba and Kiyota Lab
 */

#include "KlabVector.h"

/**
 * @brief Transform electric value from uvw to dq0.
 * @details
 * An electric value structure {u,v,w,d,q,zero} is passed into this function,
 * it keeps the uvw value and updates the dq0 value using angle parameter theta.
 * Using this function, {u,v,w,*,*,*} need to be assigned in advance.
 * An example: KLAB_uvw2dq0(&Voltage_n, &rotateValue) will update its dq0 value.
 *
 * @param elecValue Structure contains six electrical paremeters {u,v,w,d,q,zero}.
 * @param rotValue Rotate value contains angle position of rotor.
 */
void KLAB_uvw2dq0(ELECTRIC_VALUE *elecValue, ROTATE_VALUE *rotValue)
{
	FLOAT32 alpha = SQRT_2OVER3 * (elecValue->u - 0.5 * elecValue->v - 0.5 * elecValue->w);
	FLOAT32 beta = SQRT_1OVER2 * elecValue->v - SQRT_1OVER2 * elecValue->w;
	FLOAT32 zero_ab = SQRT_1OVER3 * (elecValue->u + elecValue->v + elecValue->w);

	elecValue->d = alpha * rotValue->cos + beta * rotValue->sin;
	elecValue->q = -alpha * rotValue->sin + beta * rotValue->cos;
	elecValue->zero = zero_ab;
}

/**
 * @brief Transform electric value from dq0 to uvw.
 * @details
 * An electric value structure {u,v,w,d,q,zero} is passed into this function,
 * it keeps the dq0 value and updates the uvw value using angle parameter theta.
 * Using this function, {*,*,*,d,q,zero} need to be assigned in advance.
 * An example: KLAB_dq02uvw(&Voltage_n, &rotateValue) will update its uvw value.
 *
 * @param elecValue Structure contains six electrical paremeters {u,v,w,d,q,zero}.
 * @param rotValue Rotate value contains angle position of rotor.
 */
void KLAB_dq02uvw(ELECTRIC_VALUE *elecValue, ROTATE_VALUE *rotValue)
{
	FLOAT32 alpha = elecValue->d * rotValue->cos - elecValue->q * rotValue->sin;
	FLOAT32 beta = elecValue->d * rotValue->sin + elecValue->q * rotValue->cos;
	FLOAT32 zero_ab = elecValue->zero;

	elecValue->u = SQRT_2OVER3 * alpha + SQRT_1OVER3 * zero_ab;
	elecValue->v = -SQRT_2OVER3 * 0.5 * alpha + SQRT_1OVER2 * beta + SQRT_1OVER3 * zero_ab;
	elecValue->w = -SQRT_2OVER3 * 0.5 * alpha - SQRT_1OVER2 * beta + SQRT_1OVER3 * zero_ab;
}

/**
 * @brief It is used to calculate the rotational speed.
 * @note This speed calculate function is excuted every 1/3 mechanical rotor revolution.
 * @attention Take care to prevent data overflow, timer counter counts up to 1e9, it's very close to the max value for a int32 value.
 * @attention This method cannot detect rotational direction.
 *!@warning If the rotational speed is lower than 1 rpm, it will give a wrong speed.
 * @details
 * The SRM in S3.101 is equipped with a resolver. (Note the difference from an encoder!)
 * This resolver will give abz count number in 0-4095 & a Z-pluse for every 1/3 mechanical rotor revolution.
 * This function is executed for each time a Z-pluse is received.
 * It means that, for every revolution of the rotor, the speed will be calculated 3 times.
 * To measure the past time for each revolution, the free run counter (time counter) on MWPE4-PEV Board is used.
 * This time counter counts up 1 for every 20ns, and will be cleared for every 20s (Maximum count is set as 1e9 in the main function).
 * Therefore, if time past 20s and the rotor dosn't rotate 1/3 revolution, this function will give a wrong speed result.
 *
 * @param rotValue Rotational data structure contains all needed speed data.
 * @param timerCount Value of the free run counter.
 */
void KLAB_speedCalc_update_speed(ROTATE_VALUE *rotValue, INT32 timerCount)
{
	INT32 timerDiff = 0;

	if (timerCount > rotValue->timer)
	{
		timerDiff = timerCount - rotValue->timer;
	}
	else
	{
		timerDiff = timerCount - rotValue->timer + (INT32)1e9;
	}

	FLOAT32 floatTime = timerDiff * 2e-8; /** Past time per 1/3 mechanical revolution, only for the Chiba lab JFE-SRM in S3.101. */
	rotValue->rpm = 20 / floatTime;	  // = (1 / floatTime ) / 3 * 60
	rotValue->omega = rotValue->rpm * PI_2 / 5;	  // = rpm / 60 * 12 * 2pi
	rotValue->timer = timerCount;
}

/**
 * @brief This function is used to decide duty value for Myway function PEV_inverter_set_uvw().
 *!@warning 2.33 = 2*(Ts/(Ts-2Td)) in this function, please change this value according to your dead time and switching frequency.
 * @details
 * There is still some hard work to get the inverter to output desired voltage, especially considering the effects of dead time.
 * Myway company has already done most of the work for us.
 * However, it is still necessary to calculate the duty of each inverter switch based on desired output voltage.
 * The m value passing to myway function is calculated using this equation below.
 * For possitive voltage:
 * m = 2*(Ts/(Ts-2Td))*(outVoltage/DCvoltage) - 1;
 * With setting the switch arm on the negative side to alway off.
 * For negative voltage:
 * m = 2*(Ts/(Ts-2Td)) + 2*(Ts/(Ts-2Td))*(outVoltage/DCvoltage);
 * With setting the switch arm on the negative side to alway on.
 * @see You can check the m graph of Myway function PEV_inverter_set_uvw().
 *
 * @param volValue Desired output voltege.
 * @param armDuty Duty for each inverter switch.
 * @param dcVoltage DC voltage of inverter.
 */
void KLAB_inverter_output_duty(ELECTRIC_VALUE *volValue, SWITCH_DUTY *armDuty, INT32 dcVoltage)
{
	if (volValue->u >= 0)
	{
		armDuty->dutyU = 2.33f * volValue->u / dcVoltage - 1;
		armDuty->armU = -1.5f;
	}
	else
	{
		armDuty->dutyU = 1.33f + 2.33f * volValue->u / dcVoltage;
		armDuty->armU = 1.5f;
	}

	if (volValue->v >= 0)
	{
		armDuty->dutyV = 2.33f * volValue->v / dcVoltage - 1;
		armDuty->armV = -1.5f;
	}
	else
	{
		armDuty->dutyV = 1.33f + 2.33f * volValue->v / dcVoltage;
		armDuty->armV = 1.5f;
	}

	if (volValue->w >= 0)
	{
		armDuty->dutyW = 2.33f * volValue->w / dcVoltage - 1;
		armDuty->armW = -1.5f;
	}
	else
	{
		armDuty->dutyW = 1.33f + 2.33f * volValue->w / dcVoltage;
		armDuty->armW = 1.5f;
	}
}
