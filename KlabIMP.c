/**
 * @file KlabIMP.c
 * @brief This file is for the Internal Model Principle (IMP) current controller.
 * @details
 * The main job of this program is to generate needed voltage by comparing the difference between feedback current and reference current.
 * This program seems complex but can be regarded as two parts: _update_ssFunc() & _output_outVol().
 * Actually, functions _generate_outputY() and _update_stateX() are belong to _output_outVol(),
 * Because it has several very similar computation steps, thus the duplicate tasks are splited into two new functions to make the code more concise and readable.
 * @author Shou Qiu
 * @version 1.0
 * @date 2022-08-10
 *
 * @copyright Copyright (c) 2022 Tokyo Institute of Technology, Chiba and Kiyota Lab
 */

#include "KlabIMP.h"
#include <stdio.h>
#include <string.h>
/**
 * This is a 3x12 dimension matrix used for current controller. It can be regarded as a matrix of 3x12 PID parameters.
 * Calculated using MATLAB LQR function.
 */
const FED_GAIN fedGain = {
    86.155174, 4.216277, -12.499961, -8.927197, 7.373827, -41.854034, 3.707355, -0.761033, 7.422151, 2.561384, -6.097105, 10.051089,
    -1.707397, 97.380074, -9.769536, -4.001380, 0.707206, -7.860053, -9.136024, 1.460092, -43.989056, -0.722510, -0.125424, -0.731251,
    31.286140, -13.007730, 87.726738, -2.072227, -2.622387, -11.167322, 1.669906, 0.333093, 2.655671, -9.639361, 6.327781, -42.675632};

/**
 * Two variables to store temporary state data.
 * Because they contain values that will be used in the next time when the function is executed, it is declared outside of any function.
 * User (from the main() function side) don't care these parameters, so they are not shown in the main function but declared here.
 */
DISCRETE_FUNC_VALUE ssFunc_states_X;
DISCRETE_FUNC_VALUE ssFunc_output_Y;

/**
 * @brief This function will update the state-space function of IMP current controller for target rotational speed rotOmega.
 * @note If not update the matrices A,B,C,D, the stability of the system will not change, but it cannot eliminate the harmonics current.
 * @attention Initialize the state-space function using rated speed.
 *!@warning DO NOT USE THIS FUNNCTION FOR EVERY STEEP, only use it when target rotational speed is changed.
 * @details
 * It is a discrete system transformed from continous system using Tustin Transform method.
 * Parameters in state space matrix A are used for elimination of current harmonics, and help to gateControl current component at specific frequency.
 * For more information about the Discretization Method, please refer to this document below.
 * @see 'MULTIVARIABLE CONTROL SYSTEMS', Prof. Alexandre Megretski. MIT Course Number 6.245.
 *      https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-245-multivariable-gateControl-systems-spring-2004/lecture-notes/lec11_6245_2004.pdf
 *
 * @param ssFunc Discrete state space function contains A,B,C,D matrix.
 * @param rotOmega Target rotational speed in rad/s.
 * @param fs Sampling frequency for discrete system.
 */
void KLAB_curCtrl_update_ssFunc(DISCRETE_STATE_SPACE *ssFunc, FLOAT32 rotOmega, FLOAT32 fs)
{
  /**
   * Please note the difference between rotOmega and omega0.
   * Where rotOmega is the target rotational speed, omega0 is the Tustin transform frequency.
   * Use temporary values to reduce calculation times, save DSP execution time.
   */
  FLOAT32 omega0 = fs * 2;
  FLOAT32 tmp_0 = omega0 * omega0;
  FLOAT32 tmp_1 = 36 * rotOmega * rotOmega;
  FLOAT32 tmp01 = 6 * omega0 * rotOmega;
  FLOAT32 sigma1 = tmp_0 + tmp_1;
  FLOAT32 sigma2 = tmp_0 - tmp_1;
  FLOAT32 sigma3 = tmp01 + tmp_1;
  FLOAT32 sigma4 = tmp01 - tmp_1;
  FLOAT32 sigma5 = 2 * sqrtf(fs);

  /**
   * Equations for the original continuous system:
   * Ac: [0 0 0; 0 0 -6*rotOmega; 0 6*rotOmega 0];
   * Bc: [rotOmega;-6*rotOmega;6*rotOmega];
   * Cc: [1 0 0; 0 1 0; 0 0 1];
   * Dc: [0;0;0];
   *
   * Equations for the transformed discrete system using Tustin transform:
   * Ad: (omega0*I + Ac)*(omega0*I - Ac)^-1;
   * Bd: sqrt(2*omega0)*(omega0*I - Ac)^-1*Bc;
   * Cd: sqrt(2*omega0)*Cc*(omega0*I - Ac)^-1;
   * Dd: Dc - Cc*(omega0*I - Ac)^-1*Bc;
   * where I is identity matrix.
   */
  ssFunc->A[0][0] = 1;
  ssFunc->A[1][1] = sigma2 / sigma1;
  ssFunc->A[1][2] = -2 * tmp01 / sigma1;
  ssFunc->A[2][1] = 2 * tmp01 / sigma1;
  ssFunc->A[2][2] = sigma2 / sigma1;

  ssFunc->B[0] = sigma5 * rotOmega / omega0;
  ssFunc->B[1] = -sigma5 * sigma3 / sigma1;
  ssFunc->B[2] = sigma5 * sigma4 / sigma1;

  ssFunc->C[0][0] = sigma5 / omega0;
  ssFunc->C[1][1] = sigma5 * omega0 / sigma1;
  ssFunc->C[1][2] = -sigma5 * 6 * rotOmega / sigma1;
  ssFunc->C[2][1] = sigma5 * 6 * rotOmega / sigma1;
  ssFunc->C[2][2] = sigma5 * omega0 / sigma1;

  ssFunc->D[0] = -rotOmega / omega0;
  ssFunc->D[1] = sigma3 / sigma1;
  ssFunc->D[2] = -sigma4 / sigma1;
}

/**
 * @brief This function calculates output Y for discrete state space func: Y = C*X + D*u.
 * @note This function belongs to KLAB_curCtrl_output_outVol(). Use 'inline' to save DSP execution time.
 *
 * @param ssFunc Discrete state space function contains A,B,C,D matrix.
 * @param stateX State value Xd(or q,0) in 3x1 dimensional. Here, it is a current controller state for d,q or zero(3x1 for each).
 * @param outputY Output value Yd(or q,0) in 3x1 dimensional. Here, it is a current controller output for d,q or zero(3x1 for each).
 * @param inputU Input value ud(or q,0) in 1x1 dimensional. Here, it is the current error for d,q or zero(1x1 for each).
 */
inline void KLAB_curCtrl_generate_outputY(DISCRETE_STATE_SPACE *ssFunc, FLOAT32 *stateX, FLOAT32 *outputY, FLOAT32 inputU)
{
  outputY[0] = ssFunc->C[0][0] * stateX[0] + ssFunc->D[0] * inputU;
  outputY[1] = ssFunc->C[1][1] * stateX[1] + ssFunc->C[1][2] * stateX[2] + ssFunc->D[1] * inputU;
  outputY[2] = ssFunc->C[2][1] * stateX[1] + ssFunc->C[2][2] * stateX[2] + ssFunc->D[2] * inputU;
}

/**
 * @brief This function updates the state values of current controller: X(n+1) = A*X(n) + B*u.
 * @note This function belongs to KLAB_curCtrl_output_outVol(). Use 'inline' to save DSP execution time.
 *!@warning Call this function after the generation of output Y.
 *
 * @param ssFunc Discrete state space function contains A,B,C,D matrix.
 * @param stateX State value Xd(or q,0) in 3x1 dimensional. Here, it is a current controller state for d,q or zero(3x1 for each).
 * @param inputU Input value ud(or q,0) in 1x1 dimensional. Here, it is the current error for d,q or zero(1x1 for each).
 */
inline void KLAB_curCtrl_update_stateX(DISCRETE_STATE_SPACE *ssFunc, FLOAT32 *stateX, FLOAT32 inputU)
{
  FLOAT32 tmp_0 = ssFunc->A[0][0] * stateX[0] + ssFunc->B[0] * inputU;
  FLOAT32 tmp_1 = ssFunc->A[1][1] * stateX[1] + ssFunc->A[1][2] * stateX[2] + ssFunc->B[1] * inputU;
  FLOAT32 tmp_2 = ssFunc->A[2][1] * stateX[1] + ssFunc->A[2][2] * stateX[2] + ssFunc->B[2] * inputU;

  stateX[0] = tmp_0, stateX[1] = tmp_1, stateX[2] = tmp_2;
}

/**
 * @brief It takes the reference current and measured current, calculates the tracking error and generates the reference voltage.
 * @note The main function of this .c file.
 * @details
 * Typical discrete state space equation has the form:
 * X[n+1] = A*X[n] + B*U[n];
 * Y[n]   = C*X[n] + D*U[n];
 * Here, X[n] is ssFunc_states_X, Y[n] is ssFunc_output_Y, U[n] is the tracking error.
 *
 * @param ssFunc Discrete state space function contains A,B,C,D matrix.
 * @param refC Reference current id,iq,i0 on dq0 frame.
 * @param fedC Feedback current id,iq,i0 on dq0 frame.
 * @param outV Generated reference voltage vd,vq,v0 on dq0 frame.
 */
void KLAB_curCtrl_output_outVol(DISCRETE_STATE_SPACE *ssFunc, ELECTRIC_VALUE *refC, ELECTRIC_VALUE *fedC, ELECTRIC_VALUE *outV)
{
  FLOAT32 err_d = refC->d - fedC->d;
  FLOAT32 err_q = refC->q - fedC->q;
  FLOAT32 err_0 = refC->zero - fedC->zero;

  KLAB_curCtrl_generate_outputY(ssFunc, ssFunc_states_X.d, ssFunc_output_Y.d, err_d);
  KLAB_curCtrl_generate_outputY(ssFunc, ssFunc_states_X.q, ssFunc_output_Y.q, err_q);
  KLAB_curCtrl_generate_outputY(ssFunc, ssFunc_states_X.zero, ssFunc_output_Y.zero, err_0);

  KLAB_curCtrl_update_stateX(ssFunc, ssFunc_states_X.d, err_d);
  KLAB_curCtrl_update_stateX(ssFunc, ssFunc_states_X.q, err_q);
  KLAB_curCtrl_update_stateX(ssFunc, ssFunc_states_X.zero, err_0);

  /** This combines two vectors as one [feedback current, IMP output Y]([1x3,1x9]). */
  FLOAT32 fedCur_outpuY[12];
  (void)memcpy(fedCur_outpuY, &fedC->d, sizeof(FLOAT32) * 3);
  (void)memcpy(fedCur_outpuY + 3, &ssFunc_output_Y.d, sizeof(FLOAT32) * 9);

  /**
   * Calculates the reference voltage using matrix multiplication.
   * Feedback gain matrix * IMP controller output vector: ([vd;vq;v0] = [3x12]*[12x1]).
   */
  FLOAT32 tmp_outV[3];
  int i = 0, j = 0;
  for (; i < 3; i++)
  {
    tmp_outV[i] = 0;
    j = 0;
    for (; j < 12; j++)
    {
      tmp_outV[i] -= fedGain.Matrix[i][j] * fedCur_outpuY[j];
    }
  }
  (void)memcpy(&(outV->d), tmp_outV, sizeof(FLOAT32) * 3);
}

void proposed_phase_deg2rad(FLOAT32 *p1_deg, FLOAT32 *p2_deg, FLOAT32 *p3_deg, FLOAT32 *p1_rad, FLOAT32 *p2_rad, FLOAT32 *p3_rad)
{
  // if(*p1_rad = *p1_deg * DEG2RADCOEFF){
  // return;
  //}
  *p1_rad = *p1_deg * DEG2RADCOEFF;
  *p2_rad = *p2_deg * DEG2RADCOEFF;
  *p3_rad = *p3_deg * DEG2RADCOEFF;
}

FLOAT32 combineFourierSeries(FLOAT32 i0, FLOAT32 i1, FLOAT32 i2, FLOAT32 i3, FLOAT32 p1, FLOAT32 p2, FLOAT32 p3, FLOAT32 theta)
{
  FLOAT32 proposedcurrent;
  proposedcurrent = (i0 + (i1 * mwsin((theta) + p1)) + (i2 * mwsin((2 * (theta)) + p2)) + (i3 * mwsin((3 * (theta)) + p3)));
  if (proposedcurrent < 0)
  {
    proposedcurrent = 0;
  }
  return (proposedcurrent);
}

void generate_proposed_reference(ELECTRIC_VALUE *refCurrent, FLOAT32 *i0, FLOAT32 *i1, FLOAT32 *i2, FLOAT32 *i3, FLOAT32 *p1, FLOAT32 *p2, FLOAT32 *p3, ROTATE_VALUE rotateValue)
{
  refCurrent->u = combineFourierSeries(*i0, *i1, *i2, *i3, *p1, *p2, *p3, rotateValue.theta);
  refCurrent->v = combineFourierSeries(*i0, *i1, *i2, *i3, *p1, *p2, *p3, rotateValue.theta - PI_2OVER3);
  refCurrent->w = combineFourierSeries(*i0, *i1, *i2, *i3, *p1, *p2, *p3, rotateValue.theta - PI_4OVER3);
}

INT16 generate_phase_gateSignalSequence(FLOAT32 refCurrent, FLOAT32 fedCurrent, FLOAT32 hysterisis_limit)
{
  INT16 phase_sequence = 15; // 0b1111
  if (fedCurrent > refCurrent - hysterisis_limit && fedCurrent < refCurrent + hysterisis_limit)
  {
    phase_sequence = 14; // 0b1110;
  }
  else if (fedCurrent < refCurrent - hysterisis_limit)
  {
    phase_sequence = 10; // 0b1010;
  }
  else if (fedCurrent > refCurrent + hysterisis_limit)
  {
    phase_sequence = 15; // 0b1111;
  }
  else
  {
    phase_sequence = 14; // 0b1110;
  }
  // if(fedCurrent < refCurrent){
  //       phase_sequence = 10;  //0b1010;
  // }else if(fedCurrent > refCurrent){
  //       phase_sequence = 15;  //0b1111;
  // }else{
  //       phase_sequence = 15;  //0b1111;
  // }
  return phase_sequence;
}

void generate_gateSignalSequence_Hysterisis(ELECTRIC_VALUE *refCurrent, ELECTRIC_VALUE *fedCurrent, FLOAT32 hysterisis_limit, INT16 *gateSignalSequence, INT16 *hysFLogIu)
{
  INT16 u = generate_phase_gateSignalSequence(refCurrent->u, fedCurrent->u, hysterisis_limit);
  // if (u == 15){
  //   *hysFLogIu = -1;
  // }else if(u == 10){
  //   *hysFLogIu = 1;
  // }else if(u==14){
  //   *hysFLogIu = 0;
  // }
  // INT16 u = 15;
  // INT16 v = 15;
  // INT16 w = 15;
  INT16 v = generate_phase_gateSignalSequence(refCurrent->v, fedCurrent->v, hysterisis_limit);
  INT16 w = generate_phase_gateSignalSequence(refCurrent->w, fedCurrent->w, hysterisis_limit);
  *gateSignalSequence = u + v * 16 + w * 16 * 16;
}

FLOAT32 generate_phase_square_reference(FLOAT32 theta_on, FLOAT32 theta_off, FLOAT32 peak, FLOAT32 theta)
{
  if ((theta) < 0.0)
  {
    (theta) = (theta) + PI_2;
  }
  FLOAT32 refCurrent = 0;
  if (theta_on < theta_off)
  {
    if ((theta) > theta_on && (theta) < theta_off)
    {
      refCurrent = peak;
    }
    else
    {
      refCurrent = 0;
    }
  }
  else if (theta_on > theta_off)
  {
    if ((theta) > theta_on || (theta) < theta_off)
    {
      refCurrent = peak;
    }
    else
    {
      refCurrent = 0;
    }
  }
  else
  {
    refCurrent = 0;
  }
  // return peak;
  return refCurrent;
}

void generate_square_reference(ELECTRIC_VALUE *refCurrent, FLOAT32 theta_on, FLOAT32 theta_off, FLOAT32 peak, FLOAT32 MAX_PHASE_CURRENT, ROTATE_VALUE rotateValue, FLOAT32 *compensation)
{
  /***------compensate delay between currrent and sg_output------***/
  INT32 abz_compensation = 0;
  abz_compensation = rotateValue.abz + 30;
  if (abz_compensation > 1023)
  {
    (abz_compensation) = (abz_compensation)-1023;
  }
  /***-----------------------------------------------------------***/

  refCurrent->u = 0;
  refCurrent->v = generate_phase_square_reference(theta_on, theta_off, peak, rotateValue.theta - PI_2OVER3) + compensation[rotateValue.abz]; // + compensation[abz_compensation]
  refCurrent->w = 0;
  // refCurrent->u = generate_phase_square_reference(theta_on, theta_off, peak, rotateValue.theta);
  // refCurrent->v = generate_phase_square_reference(theta_on, theta_off, peak, rotateValue.theta - PI_2OVER3);
  // refCurrent->w = generate_phase_square_reference(theta_on, theta_off, peak, rotateValue.theta - PI_4OVER3);

  if (refCurrent->u < 0)
  {
    refCurrent->u = 0;
  }
  if (refCurrent->v < 0)
  {
    refCurrent->v = 0;
  }
  if (refCurrent->w < 0)
  {
    refCurrent->w = 0;
  }
  if (refCurrent->u > MAX_PHASE_CURRENT)
  {
    refCurrent->u = MAX_PHASE_CURRENT;
  }
  if (refCurrent->v > MAX_PHASE_CURRENT)
  {
    refCurrent->v = MAX_PHASE_CURRENT;
  }
  if (refCurrent->w > MAX_PHASE_CURRENT)
  {
    refCurrent->w = MAX_PHASE_CURRENT;
  }
  // refCurrent->u = 0;
  // refCurrent->v = peak;
  // refCurrent->w = 0;
}

void square_phase_deg2rad(FLOAT32 *theta_on_deg, FLOAT32 *theta_off_deg, FLOAT32 *theta_on_rad, FLOAT32 *theta_off_rad)
{
  *theta_on_rad = *theta_on_deg * DEG2RADCOEFF;
  *theta_off_rad = *theta_off_deg * DEG2RADCOEFF;
}