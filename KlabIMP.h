/**
 * @file KlabIMP.h
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

#ifndef KLAB_IMP_h_
#define KLAB_IMP_h_
#include <mwio4.h>
#include <string.h>
#include "KlabVector.h"

/** Structure to save discete state space function {A[3][3], B[3], C[3][3], D[3]} */
typedef struct
{
  FLOAT32 A[3][3]; /** State (or system) matrix */
  FLOAT32 B[3];    /** Input matrix */
  FLOAT32 C[3][3]; /** Output matrix */
  FLOAT32 D[3];    /** Feedthrough (or feedforward) matrix */
} DISCRETE_STATE_SPACE;

/** Structure to save discete state space variables {d[3], q[3], zero[3]} */
typedef struct
{
  FLOAT32 d[3];    /** 3 states for d-phase. */
  FLOAT32 q[3];    /** 3 states for q-phase. */
  FLOAT32 zero[3]; /** 3 states for zero-phase. */
} DISCRETE_FUNC_VALUE;

/** Matrix to save feedback gain {Matrix[3][12]} */
typedef struct
{
  FLOAT32 Matrix[3][12];
} FED_GAIN;

extern const FED_GAIN fedGain;
extern DISCRETE_FUNC_VALUE ssFunc_states_X;
extern DISCRETE_FUNC_VALUE ssFunc_output_Y;

extern void KLAB_curCtrl_update_ssFunc(DISCRETE_STATE_SPACE *ssFunc, FLOAT32 rotOmega, FLOAT32 fs);
extern void KLAB_curCtrl_generate_outputY(DISCRETE_STATE_SPACE *ssFunc, FLOAT32 *stateX, FLOAT32 *outputY, FLOAT32 inputU);
extern void KLAB_curCtrl_update_stateX(DISCRETE_STATE_SPACE *ssFunc, FLOAT32 *stateX, FLOAT32 inputU);
extern void KLAB_curCtrl_output_outVol(DISCRETE_STATE_SPACE *ssFunc, ELECTRIC_VALUE *refC, ELECTRIC_VALUE *fedC, ELECTRIC_VALUE *outV);
extern void proposed_phase_deg2rad(FLOAT32 *p1_deg, FLOAT32 *p2_deg, FLOAT32 *p3_deg, FLOAT32 *p1_rad, FLOAT32 *p2_rad, FLOAT32 *p3_rad);
extern void generate_proposed_reference(ELECTRIC_VALUE *refCurrent, FLOAT32 *i0, FLOAT32 *i1, FLOAT32 *i2, FLOAT32 *i3, FLOAT32 *p1, FLOAT32 *p2, FLOAT32 *p3, ROTATE_VALUE rotateValue);
extern void generate_gateSignalSequence_Hysterisis(ELECTRIC_VALUE *refCurrent, ELECTRIC_VALUE *fedCurrent, FLOAT32 hysterisis_limit, INT16 *gateSignalSequence, INT16 *hysFLogIu);
extern void generate_square_reference(ELECTRIC_VALUE *refCurrent, FLOAT32 theta_on, FLOAT32 theta_off, FLOAT32 peak, FLOAT32 MAX_PHASE_CURRENT, ROTATE_VALUE rotateValue, FLOAT32 *compensation);
extern void square_phase_deg2rad(FLOAT32 *theta_on_deg, FLOAT32 *theta_off_deg, FLOAT32 *theta_on_rad, FLOAT32 *theta_off_rad);

#endif /* KLAB_IMP_h_ */
