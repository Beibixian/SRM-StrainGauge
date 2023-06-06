/**
 * @file BasicSRM.c
 * @brief Control program for the Switched Reluctance Motor in S3.101 for Chiba Lab.
 * @details
 * In order to distinguish the functions of Kiyota Lab from the Myway library functions,
 * all the functions made in this lab are named in the following rules:
 * KLAB_<Function name>_<Operation type>_<Operation target>
 * For example: KLAB_curCtrl_output_outVol, KLAB_<current controller>_<output>_<output voltage>.
 * All the global variables are named in lower camel case: <value purpose><Physical Quantity>
 * For example: refCurrent, <reference><Current>
 * @author Shou Qiu -> Xiang
 * @version 2.0
 * @date 2023-05-03
 *
 * @copyright Copyright (c) 2022 Tokyo Institute of Technology, Chiba and Kiyota Lab
 */

#include <math.h>	/** Standard Math Library */
#include <mwio4.h>	/** IO Library for Myway PE-Xpert4 */
#include <string.h> /** To use memset() and memcpy() */
#include <stdio.h>
#include <complex.h>
#include "KlabVector.h" /** Used for uvw2dq0 coordinate conversion and speed calculation */
#include "KlabIMP.h"	/** Internal model principle current controller */

#define PEV_BDN 0			   // Board number for PEV board
#define ADC_BDN 1			   // Board number for AD Converter board
#define FS 20000.0			   // Switching frequency
#define DEADT 3500			   // Dead time, at least 3500ns for MWINV-2022B
#define DCVOLTAGE 50		   // DC Voltage of the inverter supply
#define DIR 0				   // Rotation direction 1 = FWD, 0 = REV
#define ENCODER_MAX_COUNT 1023 // the reset threshold of encouder count
#define PI 3.14159265358	   // PI
#define N_FFT 32			   // FFTデータ個数
// #define avg_max_count 50 // the count of averaging method

FLOAT32 RPM_SET = 300;	   // rpm set by IPM 200/15=13.333hz
INT16 avg_max_count = 200; // the count of averaging method
FLOAT32 value_cutoff = 0;  // the count of averaging method
FLOAT32 AdjustTorque = 0.25;
FLOAT32 AdjustFre = 100;
FLOAT32 Proposed_Method_On = 1;

// FLOAT32 adc_range_BND0[8] = {31.25, 31.25, 31.25, 31.25, 312.5, 312.5, 312.5, 0};						 /** For MWINV-1044-SiC, last 3 values are for LEM HO50-S */
// FLOAT32 adc_offset_BND0[8] = {0.07, 0.02, 0.105, 0.0, -156.9, -157.4, -157.3, 0.0};						 /** For MWINV-1044-SiC, Ser.No. xx-0022.(0.0 means not tested), last 3 values are for LEM HO50-S*/
// FLOAT32 adc_range_BND1[12] = {50.0, 500.0, 250.0, 250.0, 50.0, 312.5, 312.5, 312.5, 0.0, 0.0, 0.0, 0.0}; /** For MWINV-1044-SiC, last 3 values are for LEM HO50-S */
// FLOAT32 adc_offset_BND1[12] = {0.0, 0.0, 0.34, 0.58, 0.0, -156.9, -157.4, -157.3, 0.0, 0.0, 0.0, 0.0};	 /** For MWINV-1044-SiC, Ser.No. xx-0022.(0.0 means not tested), last 3 values are for LEM HO50-S*/
FLOAT32 adc_range_BND0[8] = {50, 50, 50, 50, 312.5, 312.5, 312.5, 0};									 /** For MWINV-9R122C, last 3 values are for LEM HO50-S */
FLOAT32 adc_offset_BND0[8] = {0.07, 0.02, 0.105, 0.0, -156.9, -157.4, -157.3, 0.0};						 /** For MWINV-9R122C, Ser.No. xx-0022.(0.0 means not tested), last 3 values are for LEM HO50-S*/
FLOAT32 adc_range_BND1[12] = {50.0, 400.0, 250.0, 250.0, 50.0, 312.5, 312.5, 312.5, 0.0, 0.0, 0.0, 0.0}; /** For MWINV-9R122C, last 3 values are for LEM HO50-S */
FLOAT32 adc_offset_BND1[12] = {0.0, 0.0, 0.34, 0.58, 0.0, -156.9, -157.4, -157.3, 0.0, 0.0, 0.0, 0.0};	 /** For MWINV-9R122C, Ser.No. xx-0022.(0.0 means not tested), last 3 values are for LEM HO50-S*/

// FLOAT32 strain_record[(ENCODER_MAX_COUNT + 1) * avg_max_count] = {0}; // Record data of SG output and use it to calculate average.
FLOAT32 avg_record_times[ENCODER_MAX_COUNT + 1] = {0}; // Average times (real-time).
FLOAT32 strain_avg[ENCODER_MAX_COUNT + 1] = {0};	   // Average data of SG output.
FLOAT32 strain_ref[ENCODER_MAX_COUNT + 1] = {
	0.089582141, 0.099175219, 0.107612321, 0.115616792, 0.121867346, 0.134995376, 0.143879305, 0.146976975,
	0.158717466, 0.167480296, 0.177856983, 0.185020222, 0.193782584, 0.201525057, 0.209224701, 0.222122934,
	0.226610622, 0.238858663, 0.249043318, 0.257343658, 0.262876793, 0.2769934, 0.283625804, 0.299789158,
	0.299252441, 0.315289292, 0.321006568, 0.336776175, 0.338349073, 0.353892834, 0.354776859, 0.370322224,
	0.385589038, 0.390123268, 0.397681418, 0.409673833, 0.41862636, 0.421588462, 0.426586565, 0.44438044,
	0.446800253, 0.447287436, 0.465195902, 0.46701272, 0.471459494, 0.489127792, 0.492761578, 0.501288597,
	0.502266238, 0.522327461, 0.519120943, 0.531651483, 0.539429486, 0.539552572, 0.558697757, 0.558659846,
	0.560812781, 0.577547348, 0.575272487, 0.591275048, 0.590271368, 0.607998767, 0.606802459, 0.616634118,
	0.627047734, 0.630218178, 0.63963175, 0.641765747, 0.651201737, 0.659021742, 0.661087274, 0.675812786,
	0.68563089, 0.687496595, 0.689246487, 0.707263648, 0.713624899, 0.715197202, 0.718446764, 0.738563704,
	0.739311322, 0.7399381, 0.753511633, 0.76216163, 0.758424764, 0.770826672, 0.780088691, 0.779581089,
	0.792085878, 0.800692403, 0.799917048, 0.80642909, 0.820891829, 0.819868567, 0.81996968, 0.83905916,
	0.836941088, 0.83679286, 0.854697245, 0.852213101, 0.862882655, 0.865346962, 0.86257087, 0.883987717,
	0.885240527, 0.88639504, 0.888994183, 0.900849202, 0.911205284, 0.901815161, 0.915254509, 0.926393182,
	0.926251688, 0.921149512, 0.944483805, 0.94493875, 0.944470454, 0.943902421, 0.962675096, 0.961999828,
	0.956136127, 0.974041808, 0.972258964, 0.971237669, 0.983166286, 0.991649837, 0.98626432, 0.985822862,
	1.00667873, 1.004337977, 0.996504151, 1.017494973, 1.015817742, 1.013133186, 1.023237365, 1.027266143,
	1.02994283, 1.027708412, 1.028832656, 1.041227383, 1.041328087, 1.05282938, 1.052838166, 1.041844412,
	1.05424007, 1.054075608, 1.06555675, 1.059421819, 1.053206896, 1.071523903, 1.070109112, 1.069622809,
	1.06480564, 1.076208295, 1.086670786, 1.086003401, 1.085275791, 1.083469199, 1.082619839, 1.082735708,
	1.093096769, 1.09292539, 1.09188439, 1.090785508, 1.106360293, 1.104132179, 1.09763338, 1.106890542,
	1.110863504, 1.105079382, 1.103641835, 1.118225547, 1.118903074, 1.107671018, 1.120123136, 1.119578585,
	1.109197444, 1.131486152, 1.11538211, 1.127829605, 1.12707133, 1.116323347, 1.138760548, 1.137884773,
	1.13248195, 1.125903046, 1.143967207, 1.138404767, 1.13843947, 1.137297382, 1.136108421, 1.147433162,
	1.147322189, 1.140277606, 1.13891855, 1.157211724, 1.145365958, 1.143895437, 1.159864425, 1.159505716,
	1.146215743, 1.156312978, 1.166463913, 1.149515456, 1.159595035, 1.169737862, 1.170427099, 1.169881979,
	1.159702479, 1.163859914, 1.176468408, 1.164886114, 1.171559425, 1.172009993, 1.16026643, 1.182607747,
	1.17193509, 1.169795443, 1.182390588, 1.170311884, 1.188068155, 1.177103549, 1.174783698, 1.186134773,
	1.175003987, 1.185083926, 1.18387757, 1.195257805, 1.183877784, 1.187891591, 1.194181088, 1.182648654,
	1.191428059, 1.191276115, 1.183416581, 1.199883053, 1.18806115, 1.204888036, 1.192954129, 1.190039251,
	1.208800733, 1.198919569, 1.20073918, 1.201215665, 1.214841956, 1.204673569, 1.21053913, 1.213537856,
	1.201804015, 1.219469483, 1.222402468, 1.211795921, 1.217345414, 1.224255602, 1.208296652, 1.232883999,
	1.221958391, 1.221942753, 1.234290607, 1.23007964, 1.217541123, 1.242352019, 1.231041974, 1.231154145,
	1.244922203, 1.233439862, 1.231731252, 1.245488312, 1.235239843, 1.233421616, 1.245754125, 1.228633922,
	1.252350747, 1.241825299, 1.240181813, 1.254094895, 1.243564355, 1.230165736, 1.242575872, 1.244939855,
	1.227799728, 1.238753432, 1.242517955, 1.228906888, 1.239865542, 1.243599422, 1.232774581, 1.230648361,
	1.244539807, 1.229969682, 1.232161322, 1.235808346, 1.221904177, 1.2328186, 1.236435467, 1.225350709,
	1.22305192, 1.231783927, 1.222095184, 1.219742631, 1.232124509, 1.216451032, 1.214040874, 1.227875792,
	1.216545149, 1.203748378, 1.222772404, 1.206933863, 1.204469685, 1.219181652, 1.199786272, 1.198178973,
	1.202482129, 1.180238742, 1.200463, 1.181147852, 1.180997417, 1.183787519, 1.163101678, 1.174651672,
	1.168620784, 1.156755726, 1.171217889, 1.144500075, 1.155966026, 1.149947919, 1.138128523, 1.152489897,
	1.133385921, 1.127394302, 1.128634841, 1.1281792, 1.114973359, 1.114755565, 1.115977064, 1.109992574,
	1.098280847, 1.106659442, 1.08637702, 1.087560891, 1.097095104, 1.079705927, 1.080927302, 1.064085595,
	1.068496814, 1.051777472, 1.043564566, 1.047930343, 1.028883229, 1.033218067, 1.0209668, 1.012909079,
	1.019951739, 0.992845091, 0.997100633, 0.994574612, 0.977239949, 0.984377947, 0.969823352, 0.962025534,
	0.959563805, 0.942604038, 0.946743912, 0.929924402, 0.931426988, 0.913699408, 0.917775243, 0.914106346,
	0.897643778, 0.892757027, 0.899329383, 0.874218715, 0.881982243, 0.859643941, 0.867545258, 0.855312649,
	0.848591366, 0.845839314, 0.830877026, 0.828164581, 0.81221125, 0.808347598, 0.790809169, 0.794082767,
	0.783238428, 0.772491497, 0.774542297, 0.75128814, 0.749898292, 0.742842698, 0.732468173, 0.726017364,
	0.719078449, 0.716620466, 0.702031795, 0.702902095, 0.68094248, 0.679656805, 0.677293445, 0.662653465,
	0.667730747, 0.645480068, 0.648395717, 0.628606555, 0.628414441, 0.621105363, 0.606844093, 0.602679843,
	0.603919206, 0.586018687, 0.589783243, 0.571328021, 0.575123218, 0.556971248, 0.553167236, 0.549379555,
	0.547469136, 0.532101492, 0.528414588, 0.525651068, 0.513897608, 0.513867229, 0.496135064, 0.496117174,
	0.49347272, 0.482548653, 0.479100861, 0.469732304, 0.464668079, 0.462143207, 0.448880785, 0.443969392,
	0.440714551, 0.431048063, 0.430569839, 0.421051833, 0.413225032, 0.413233587, 0.397094134, 0.400158331,
	0.391087669, 0.386612161, 0.380983073, 0.370738863, 0.367142861, 0.36428162, 0.362149321, 0.349428913,
	0.346664525, 0.343913992, 0.335692699, 0.329205762, 0.326553145, 0.326586659, 0.312079379, 0.312125092,
	0.309566871, 0.29744458, 0.29749561, 0.286528468, 0.289064376, 0.281114894, 0.272702437, 0.269779761,
	0.266878055, 0.267528812, 0.253625063, 0.253697333, 0.248293258, 0.245550814, 0.24282866, 0.234636761,
	0.237444914, 0.22457002, 0.226779962, 0.216860479, 0.216617987, 0.214118348, 0.20405086, 0.204070111,
	0.19961969, 0.19232602, 0.192351638, 0.185704447, 0.182918273, 0.183127181, 0.171574412, 0.173917723,
	0.1716793, 0.164984587, 0.158020373, 0.155926343, 0.155985763, 0.147348122, 0.145352776, 0.142968843,
	0.140611583, 0.138678709, 0.136369843, 0.130223993, 0.124250436, 0.122106847, 0.122318154, 0.120197552,
	0.116309745, 0.110413399, 0.110153662, 0.110234837, 0.104548594, 0.099042955, 0.098922488, 0.098688056,
	0.093389441, 0.088609316, 0.08694819, 0.085308129, 0.0854494, 0.0806562, 0.079096504, 0.075977275,
	0.074478249, 0.072999189, 0.071539954, 0.069839017, 0.065617898, 0.061307593, 0.060006708, 0.059978882,
	0.05846206, 0.057202387, 0.055732462, 0.051626943, 0.050465941, 0.049322448, 0.049038034, 0.04524844,
	0.043047358, 0.041823221, 0.040621946, 0.040570173, 0.037016379, 0.035905613, 0.032767711, 0.031739081,
	0.031545466, 0.031390574, 0.03039201, 0.027404971, 0.026479051, 0.024003488, 0.023151443, 0.022184784,
	0.021374694, 0.021389139, 0.02047118, 0.019578283, 0.018831253, 0.016482251, 0.014941096, 0.014413214,
	0.013693192, 0.012995741, 0.012822759, 0.010371352, 0.010421633, 0.009609203, 0.00930245, 0.007103612,
	0.006395479, 0.005823187, 0.004833938, 0.00429604, 0.004331504, 0.003835271, 0.003380463, 0.003296155,
	0.002174269, 0.001870488, 0.001562933, 0.001176881, 0.001167176, 0.000971465, 0.001117708, 0.000594722,
	0.000473611, 0.000357968, 0.000273803, 0.000157151, 0.000104879, 0.000109343, 6.93108E-05, 4.06422E-05,
	2.13282E-05, 9.4338E-06, 0.000003096, 5.256E-07, 0, 0.000000684, 6.768E-07, 0.000003447,
	0.000010899, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 1.13724E-05, 0.000011367, 4.5882E-06, 3.6828E-06, 3.6846E-06,
	9.342E-07, 7.02E-08, 1.08E-08, 2.232E-07, 6.822E-07, 4.788E-07, 0.000002358, 4.446E-07,
	2.2572E-06, 2.8026E-06, 7.2882E-06, 1.53918E-05, 2.74554E-05, 2.69334E-05, 4.38768E-05, 6.67602E-05,
	6.02406E-05, 0.000088119, 5.76216E-05, 8.36172E-05, 0.000116458, 0.000113715, 0.000147524, 0.00015086,
	0.000180783, 0.000212139, 0.000246031, 0.000283972, 0.00032306, 0.000362997, 0.000358022, 0.000401839,
	0.000446695, 0.000428933, 0.000475857, 0.000525352, 0.000460492, 0.000507247, 0.000619711, 0.000674154,
	0.000597935, 0.000651514, 0.000705121, 0.000760995, 0.000760217, 0.000818402, 0.000878891, 0.000927277,
	0.000926584, 0.000988351, 0.001052275, 0.001118367, 0.001183471, 0.001250584, 0.001319648, 0.001301956,
	0.001372379, 0.001444797, 0.001515638, 0.001588334, 0.001662896, 0.001739333, 0.001817658, 0.00189788,
	0.001872884, 0.001950388, 0.00203123, 0.002113506, 0.002197753, 0.0022795, 0.002367653, 0.002453152,
	0.002540504, 0.002607851, 0.002698506, 0.002786024, 0.002875268, 0.002966256, 0.003059001, 0.003153517,
	0.003244318, 0.003497722, 0.003600171, 0.003692642, 0.003987077, 0.004085453, 0.004185254, 0.004286587,
	0.004389464, 0.004487303, 0.00443039, 0.004535582, 0.004635603, 0.004737028, 0.004839863, 0.004937116,
	0.005042713, 0.005142569, 0.005224819, 0.005707535, 0.005818316, 0.005968316, 0.006120508, 0.00652219,
	0.006675183, 0.006830127, 0.006943106, 0.006887268, 0.007488707, 0.007645613, 0.007804224, 0.008022872,
	0.008175987, 0.008340574, 0.008496684, 0.009126238, 0.009281848, 0.009449572, 0.009876407, 0.010039477,
	0.010542839, 0.010701041, 0.010799689, 0.011316384, 0.011218207, 0.011380669, 0.011531282, 0.012295843,
	0.012454873, 0.012614501, 0.013022221, 0.01356593, 0.013436001, 0.013602116, 0.013803084, 0.015581669,
	0.017872646, 0.019958053, 0.021851114, 0.024013364, 0.025568914, 0.026885822, 0.028369046, 0.031578617,
	0.033921662, 0.035903167, 0.038308505, 0.039754795, 0.041163444, 0.044927032, 0.046323821, 0.047679473,
	0.049858317, 0.053729717, 0.056286261, 0.057512459, 0.059423747, 0.064104799, 0.065197526, 0.066231596,
	0.071553791, 0.074063759, 0.074995081, 0.076720928, 0.081591856, 0.082370615, 0.087096213, 0.087679615}; // reference of SG output.
FLOAT32 strain_avg_temp[ENCODER_MAX_COUNT + 1] = {0};														 // Record average data of SG output.
FLOAT32 strain_avg_ffted[ENCODER_MAX_COUNT + 1] = {0};														 // SG value after phase compensation.
FLOAT32 compensation[ENCODER_MAX_COUNT + 1] = {0};															 // compensation of current reference
FLOAT32 strain_offset = 0;																					 // offset of SG output.
FLOAT32 hy_band = 0.1;
FLOAT32 current_step = 0; // 0.2;

FLOAT32 data[8];

FLOAT32 gateControl_interval;
FLOAT32 StrainGaugeRead_interval;
FLOAT32 scope_interval;
INT16 A_aligned_encoder_count;
FLOAT32 hysterisis_limit;
INT inverter_on;
INT16 gateSignalSequence;
INT16 virtual_frequency;

INT32 abz_prev = 1024;
INT32 rotate_period_count_cal = 0; // Used in compensation calculation
INT32 factor_cal_time = 1;		   // Used in compensation calculation
INT32 n_com = 0;				   // Used in compensation calculation
INT32 n_offset = 0;				   // Used in offset calculation

// FLOAT32 i0 = 0, i1 = 0, i2 = 0, i3 = 0, p1 = 0, p2 = 0, p3 = 0;
FLOAT32 i0, i1, i2, i3, p1_deg, p2_deg, p3_deg, p1_rad, p2_rad, p3_rad;
FLOAT32 theta_on_deg, theta_off_deg, theta_on_rad, theta_off_rad, square_peak, MAX_PHASE_CURRENT;
ROTATE_VALUE rotateValue;						   /** A data structure that contains all needed rotation-related data */
ELECTRIC_VALUE refCurrent, fedCurrent, outVoltage; /** Electric values on uvw&dq0, ref: reference, fed: feedback(measured), out: output */
// DISCRETE_STATE_SPACE ssFunction;				   /** State-space function of IMP controller */
// SWITCH_DUTY switchDuty;							   /** Duty of each inverter switch */

/**
 * @brief To inspect variables using PE-ViewX Inspector/WAVE function.
 * @details
 * Myway PE-ViewX Inspector/WAVE function can display public variables.
 * However, it can not display the user defined data structures, even they are global variables.
 * This function is used to pass the variables you want to check in the WAVE window.
 * @note
 * This part of code can be replaced by modifying the .def file in your Myway PE-ViewX build folder after compilation.
 * For example, add [refCurrent.u 11f0xxxx FLOAT 1] at the end of the .def file.
 * The variable adderss can be found in .map file in the same folder.
 * It is highly recommended to do this, as it saves RAM memory, and avoids wasting useless DSP execution time.
 * The python command file Address.py can help you finish this work. You can double click this file after compilation in Myway PE-ViewX.
 *?If you have trouble understanding this, you can also uncomment this section and add the variables that you wish to observe.
 *
 * @param SCOPE_XXX All parameters are name as SCOPE_XXX to indicate that they are only used for data observation.
 */
// FLOAT32 SCOPE_fedId, SCOPE_fedIq, SCOPE_fedI0;
FLOAT32 SCOPE_fedIu, SCOPE_fedIv, SCOPE_fedIw;
FLOAT32 SCOPE_outVu, SCOPE_outVv, SCOPE_outVw;
FLOAT32 SCOPE_abz, SCOPE_theta, SCOPE_omega, SCOPE_rpm;
FLOAT32 SCOPE_strain_avg;
FLOAT32 SCOPE_strain_ref;
FLOAT32 SCOPE_compensation;
FLOAT32 SCOPE_phase_fft;
FLOAT32 SCOPE_magnitude_fft;
FLOAT32 SCOPE_strain_avg_ffted;
// FLOAT32 SCOPE_refI0;
FLOAT32 SCOPE_torque;
FLOAT32 SCOPE_refIu, SCOPE_refIv, SCOPE_refIw;
FLOAT32 Vdc;
FLOAT32 Idc;
FLOAT32 adjustFedIu;
INT16 hysFLagIu;
FLOAT32 strain;
// INT16 n_fft, k_fft;
INT16 i_fft;
FLOAT32 R[N_FFT], phase[N_FFT], magnitude[N_FFT];
FLOAT32 x_fft[N_FFT], y_fft[N_FFT], a_fft[N_FFT], b_fft[N_FFT];						// 离散傅里叶变换实部虚部
FLOAT32 a_fft_new[N_FFT], b_fft_new[N_FFT], x_fft_new[N_FFT], y_fft_new[N_FFT];		// 补偿后的离散傅里叶变换实部虚部
FLOAT32 a_fft_neww[N_FFT], b_fft_neww[N_FFT], x_fft_neww[N_FFT], y_fft_neww[N_FFT]; // 补偿后的离散傅里叶变换实部虚部
double complex Z;
double complex phase_test = I * PI / 2;
// typedef struct
// {
// 	FLOAT32 r;
// 	FLOAT32 i;
// } complex_define;
// complex_define x[N_FFT], X_FFT;

void scope(void)
{

	SCOPE_fedIu = fedCurrent.u;
	SCOPE_fedIv = fedCurrent.v;
	SCOPE_fedIw = fedCurrent.w;

	SCOPE_outVu = outVoltage.u;
	SCOPE_outVv = outVoltage.v;
	SCOPE_outVw = outVoltage.w;

	SCOPE_abz = rotateValue.abz;
	SCOPE_theta = rotateValue.theta;
	SCOPE_omega = rotateValue.omega;
	SCOPE_rpm = rotateValue.rpm;

	// SCOPE_refI0 = refCurrent.zero;

	SCOPE_refIu = refCurrent.u;
	SCOPE_refIv = refCurrent.v;
	SCOPE_refIw = refCurrent.w;

	SCOPE_strain_avg = (strain_avg[rotateValue.abz] - strain_offset);
	SCOPE_strain_ref = strain_ref[rotateValue.abz] * AdjustTorque;
	SCOPE_compensation = compensation[rotateValue.abz];
	SCOPE_phase_fft = phase[rotateValue.abz];
	SCOPE_magnitude_fft = magnitude[rotateValue.abz];
	SCOPE_strain_avg_ffted = strain_avg_ffted[rotateValue.abz];
}

// interruput handler for the interrupt triggered by timer0
interrupt void gateControl(void)
{
	int3_ack();

	// /** Read the value from current sensor on MWINV-2022B. (Defult, Please comment out one of these current measurement code) */
	if (PEV_ad_in_grp(PEV_BDN, data) != 1)
	{
		fedCurrent.u = data[0];
		fedCurrent.v = -data[1];
		fedCurrent.w = data[2];
		Idc = data[3];
	}
	// // /** Read electrical angle of the rotor. */
	// rotateValue.abz = (PEV_abz_read(PEV_BDN) + ENCODER_MAX_COUNT + 1 - A_aligned_encoder_count) % (ENCODER_MAX_COUNT + 1); // compensate abz-count to A-aligned position to count:zero
	// rotateValue.theta = PI_2 * rotateValue.abz / ENCODER_MAX_COUNT;														   // convert abz-count to electric angle in radians

	proposed_phase_deg2rad(&p1_deg, &p2_deg, &p3_deg, &p1_rad, &p2_rad, &p3_rad);
	square_phase_deg2rad(&theta_on_deg, &theta_off_deg, &theta_on_rad, &theta_off_rad);
	// generate_proposed_reference(&refCurrent, &i0, &i1, &i2, &i3, &p1_rad, &p2_rad, &p3_rad, rotateValue);

	generate_square_reference(&refCurrent, theta_on_rad, theta_off_rad, square_peak, MAX_PHASE_CURRENT, rotateValue, compensation); // 在这里加个补偿参数,注意数组变量
	generate_gateSignalSequence_Hysterisis(&refCurrent, &fedCurrent, hysterisis_limit, &gateSignalSequence, &hysFLagIu);
	if (inverter_on == 1)
	{
		PEV_inverter_control_gate(PEV_BDN, gateSignalSequence);
	}
	else
	{
		gateSignalSequence = 4095;
		PEV_inverter_control_gate(PEV_BDN, gateSignalSequence);
	}
	scope();
}

/* 离散傅里叶变换及逆变换
	x-存放要变换数据的实部
	y-存放要变换数据的虚部
	a-存放变换结果的实部
	b-存放变换结果的虚部
	n-数据长度N
	sign-为1时执行DFT，为-1时执行IDFT
*/
void dft(FLOAT32 *x, FLOAT32 *y, FLOAT32 *a, FLOAT32 *b, INT16 n)
{
	INT16 i, k;
	i = k = 0;
	FLOAT32 c, d, q, w, s;
	c = d = q = w = s = 0;
	q = 6.28318530718 / n;
	for (k = 0; k < n; k++) // 这行n改为12
	{
		w = k * q;
		a[k] = b[k] = 0.0;
		for (i = 0; i < n; i++)
		{
			d = i * w;
			c = cos(d);
			s = sin(d);
			a[k] += c * x[i] + s * y[i];
			b[k] += c * y[i] - s * x[i];
		}
	}
}

void idft(FLOAT32 *x, FLOAT32 *y, FLOAT32 *a, FLOAT32 *b, INT16 n)
{
	INT16 i, k;
	i = k = 0;
	FLOAT32 c, d, q, w, s;
	c = d = q = w = s = 0;
	q = 6.28318530718 / n;
	for (k = 0; k < n; k++) // 这行n改为12
	{
		w = k * q;
		a[k] = b[k] = 0.0;
		for (i = 0; i < n; i++)
		{
			d = i * w;
			c = cos(d);
			s = sin(d) * (-1);
			a[k] += c * x[i] + s * y[i];
			b[k] += c * y[i] - s * x[i];
		}
	}
	c = 1.0 / n;
	for (k = 0; k < n; k++) // 这行n改为12
	{
		a[k] = c * a[k];
		b[k] = c * b[k];
	}
}

// StrainGaugeRead interruput handler for the interrupt triggered by timer1
interrupt void StrainGaugeRead(void)
{
	// /** Read electrical angle of the rotor. */
	rotateValue.abz = (PEV_abz_read(PEV_BDN) + ENCODER_MAX_COUNT + 1 - A_aligned_encoder_count) % (ENCODER_MAX_COUNT + 1); // compensate abz-count to A-aligned position to count:zero
	rotateValue.theta = PI_2 * rotateValue.abz / (ENCODER_MAX_COUNT + 1);												   // convert abz-count to electric angle in radians

	if (abz_prev == rotateValue.abz)
	{
		return;
	}
	FLOAT32 data_ADC[12];

	if (ADC_in_grp(ADC_BDN, data_ADC) != 1)
	{
		strain = data_ADC[0];
	}

	avg_record_times[rotateValue.abz]++;
	strain_avg_temp[rotateValue.abz] = strain_avg_temp[rotateValue.abz] + (strain - strain_avg_temp[rotateValue.abz]) / avg_record_times[rotateValue.abz];
	// if (abz_prev != rotateValue.abz)
	// {
	// 	avg_record_times[rotateValue.abz]++;
	// 	strain_avg_temp[rotateValue.abz] = strain_avg_temp[rotateValue.abz] + (strain - strain_avg_temp[rotateValue.abz]) / avg_record_times[rotateValue.abz];
	// }
	if (avg_record_times[rotateValue.abz] > avg_max_count)
	{

		INT16 i;
		for (i = 0; i < ENCODER_MAX_COUNT + 1; i++)
		{
			strain_avg[i] = strain_avg_temp[i];
			// strain_avg_temp[i] = 0;
			// avg_record_times[i] = 0;
		}
		memset(strain_avg_temp, 0, sizeof(strain_avg_temp));
		memset(avg_record_times, 0, sizeof(avg_record_times));
	}

	strain_offset = 0;
	// generate_offset(); 使用abz800-1000范围内的sg输出值来计算当前offset
	for (n_offset = 801; n_offset < 1001; n_offset++)
	{
		strain_offset = strain_offset + strain_avg[n_offset];
	}
	strain_offset = strain_offset / 200;

	if (abz_prev - rotateValue.abz > 500)
	{
		rotate_period_count_cal++;
		if (rotate_period_count_cal > avg_max_count * factor_cal_time)
		{
			// FFT function()
			for (i_fft = 0; i_fft < N_FFT; i_fft++)
			{
				// x_fft[i_fft] = strain_ref[i_fft * 1024 / N_FFT] * AdjustTorque;
				// x_fft[i_fft] = sin(i_fft / N_FFT * 2 * PI * AdjustFre) * AdjustTorque;
				x_fft[i_fft] = strain_avg[i_fft * 1024 / N_FFT] - strain_offset;
				y_fft[i_fft] = 0;
			}
			dft(x_fft, y_fft, a_fft, b_fft, N_FFT); // DFT变换
			for (i_fft = 0; i_fft < N_FFT; i_fft++)
			{
				Z = a_fft[i_fft] + I * b_fft[i_fft];
				// Z = Z * exp(phase_test);
				phase[i_fft] = carg(Z);		// 计算相位
				magnitude[i_fft] = cabs(Z); // 计算幅度
				// (代办)phase_compensation(); // 利用hammer test数据补偿相位
				// a_fft_new[i_fft] = magnitude[i_fft] * cos(phase[i_fft]);
				// b_fft_new[i_fft] = magnitude[i_fft] * sin(phase[i_fft]);
				// phase[i_fft] = phase[i_fft] - fmod(0.001419 * (double)i_fft * RPM_SET / 5 + 0.04958, 2 * PI);
				// phase[i_fft] = phase[i_fft] - 0.001419 * (double)i_fft * RPM_SET / 5 - 0.04958; // 补偿相位
				// phase[i_fft] = phase[i_fft] - 0.001419 * (double)i_fft * RPM_SET / 5 - 0.04958;
				if (i_fft > N_FFT / 2)
				{
					phase[i_fft] = phase[i_fft] - 0.001419 * (N_FFT - (double)i_fft) * RPM_SET / 5;
				}
				else
				{
					phase[i_fft] = phase[i_fft] + 0.001419 * (double)i_fft * RPM_SET / 5;
				}
				a_fft_neww[i_fft] = magnitude[i_fft] * cos(phase[i_fft]);
				b_fft_neww[i_fft] = magnitude[i_fft] * sin(phase[i_fft]);
				// a_fft_neww[i_fft] = creal(Z);
				// b_fft_neww[i_fft] = cimag(Z);
			}

			// idft(a_fft_new, b_fft_new, x_fft_new, y_fft_new, N_FFT);	 // IDFT变换 其中y_fft_new应为0
			idft(a_fft_neww, b_fft_neww, x_fft_neww, y_fft_neww, N_FFT); // IDFT变换 其中y_fft_new应为0

			// interpolation();在N_FFT中插值,扩展为1024长度
			INT16 mm, nn, kk, mmm;
			FLOAT32 nnn, kkk;
			for (i_fft = 0; i_fft < 1024; i_fft++)
			{
				kk = 1024 / N_FFT;
				mm = i_fft / kk;
				nn = i_fft % kk;
				kkk = (double)kk;
				nnn = (double)nn;
				// strain_avg_ffted[i_fft] = x_fft_new[mm] * ((kkk - nnn) / kkk) + x_fft_new[mm + 1] * (nnn / kkk);
				mmm = mm + 1;
				if (mmm == N_FFT)
				{
					mmm = 0;
				}
				strain_avg_ffted[i_fft] = x_fft_neww[mm] * ((kkk - nnn) / kkk) + x_fft_neww[mmm] * (nnn / kkk);
			}

			// generate_compensation(); 参数 1.补偿(输出) 2.SG参考 3.SG反馈 SG_ave 4.hy band 5.current_step 6.offset
			for (n_com = 0; n_com < ENCODER_MAX_COUNT + 1; n_com++)
			{
				// if ((strain_avg[n_com] - strain_offset) > (strain_ref[n_com] * AdjustTorque + hy_band))
				if (strain_avg_ffted[n_com] > (strain_ref[n_com] * AdjustTorque + hy_band))
				{
					compensation[n_com] = (compensation[n_com] - current_step) * Proposed_Method_On;
				}
				// if ((strain_avg[n_com] - strain_offset) < (strain_ref[n_com] * AdjustTorque - hy_band))
				if (strain_avg_ffted[n_com] < (strain_ref[n_com] * AdjustTorque - hy_band))
				{
					compensation[n_com] = (compensation[n_com] + current_step) * Proposed_Method_On;
				}
				if (strain_ref[n_com] < value_cutoff)
				{
					compensation[n_com] = -100;
				}
			}
			rotate_period_count_cal = 0;
		}
	}
	abz_prev = rotateValue.abz;
	scope();
}

interrupt void scope_interrupt(void)
{
	C6657_timer1_clear_eventflag();
	FLOAT32 data[8];
	if (PEV_ad_in_grp(PEV_BDN, data) != 1)
	{
		fedCurrent.u = data[0];
		fedCurrent.v = -data[1];
		fedCurrent.w = data[2];
		Idc = data[3];
	}
	scope();
}

int MW_main(void)
{
	// for (i_fft = 0; i_fft < 1024; i_fft++)
	// {
	// 	// x_fft[i_fft] = strain_ref[i_fft * 1024 / N_FFT] * AdjustTorque;
	// 	strain_ref[i_fft] = sin((double)i_fft / 1024 * 2 * PI);
	// 	// x_fft[i_fft] = strain_avg[i_fft * 1024 / N_FFT] - strain_offset;
	// }

	// rotateValue.theta = 0;
	i0 = i1 = i2 = i3 = p1_deg = p2_deg = p3_deg = theta_on_deg = theta_off_deg = square_peak = 0;
	MAX_PHASE_CURRENT = 13;
	inverter_on = 0;
	gateSignalSequence = 4095;

	// gateControl_interval = mwdiv(1.0, FS) * 1e6;
	gateControl_interval = 1 / FS * 1e6;
	StrainGaugeRead_interval = 1 / FS * 1e6 / 5;
	scope_interval = 1 / FS * 1e6 / 5;
	A_aligned_encoder_count = 5;
	hysterisis_limit = 1; // in [A]

	adjustFedIu = 5;
	hysFLagIu = 1;
	virtual_frequency = 100;
	/** Set initial reference current to 0A. */
	// refId = 0, refIq = 0, refI0 = 0;
	// refI0ac = 0, refPhi = 0;

	/** Initialize the IMP controller for 800rpm.(800/60*2pi*6 = 502.65) */
	// IMPOmega = 502.65f;
	// KLAB_curCtrl_update_ssFunc(&ssFunction, 502.65f, FS);
	// (void)memset(&ssFunc_states_X.d, 0, sizeof(FLOAT32) * 9);
	// (void)memset(&ssFunc_output_Y.d, 0, sizeof(FLOAT32) * 9);

	/** All the codes below are from Myway sample program [test_pev_sync_inverter01_ad] with few changes. */
	int_disable();

	// C6657_timer0_init (gateControl_interval);									/* for speed calculation */
	// C6657_timer0_init_vector (gateControl, (CSL_IntcVectId)5);
	// C6657_timer0_enable_int();
	// C6657_timer0_start ();

	// C6657_timer1_init (scope_interval);									/* for speed calculation */
	// C6657_timer1_init_vector (scope_interrupt, (CSL_IntcVectId)6);
	// C6657_timer1_enable_int();
	// C6657_timer1_start ();

	C6657_timer1_init(StrainGaugeRead_interval); /* for Strain Gauge Sampling */
	C6657_timer1_init_vector(StrainGaugeRead, (CSL_IntcVectId)9);
	C6657_timer1_enable_int();
	C6657_timer1_start();

	PEV_init(PEV_BDN);
	PEV_int_init(PEV_BDN, 0, 0, 0, 2, 0, 0, 0, 0);
	int3_init_vector(gateControl, (CSL_IntcVectId)4, FALSE);
	int3_enable_int();

	// PEV_sync_ad_out(PEV_BDN,4);
	//  int2_init_vector(speed, (CSL_IntcVectId)4, FALSE);
	//  int2_enable_int();
	//  int7_init_vector(gateControl, (CSL_IntcVectId)5, FALSE);
	//  int7_enable_int();

	/** Initialization should be placed in front of PEV_ad_set_offset(), or the offset will not be enabled. */
	PEV_sync_ad_init(PEV_BDN, 1, 4);
	ADC_init(ADC_BDN); // TODO check the necessary settings to implement an ADC

	PEV_ad_set_range(PEV_BDN, adc_range_BND0);
	PEV_ad_set_offset(PEV_BDN, adc_offset_BND0);
	ADC_set_range(ADC_BDN, adc_range_BND0);
	ADC_set_offset(ADC_BDN, adc_offset_BND1);

	PEV_ad_set_mode(PEV_BDN, 1);

	PEV_sync_ad_out(PEV_BDN, 4); // TODO check how master-slave is done between PEV and ADC
	ADC_set_mode(ADC_BDN, 3, 4); // second param "3" set ADC to be used in sync mode, third param assigns the in-port in master-slave relation

	PEV_inverter_init(PEV_BDN, FS, DEADT);
	PEV_inverter_set_uvw(PEV_BDN, 0, 0, 0, 0);
	PEV_inverter_enable_int(PEV_BDN);
	PEV_inverter_init_adtrig_timing(PEV_BDN, 0, 200); // AD conversion need 200ns to complete

	/** Get initial value of the resolver. */
	PEV_abz_set_mode(PEV_BDN, 5, DIR);
	PEV_abz_init_maxcount(PEV_BDN, ENCODER_MAX_COUNT);
	PEV_abz_clear(PEV_BDN);
	PEV_abz_clear_resolver_err(PEV_BDN);
	INT16 count_init = PEV_abz_get_resolver_pos(PEV_BDN);
	PEV_abz_write(PEV_BDN, count_init);

	/** Initialize the free run counter (time counter) on PEV, it counts up to 20 secounds (1e9*20ns) and returns to 0. */

	int_enable();
	wait(200);
	PEV_ad_start(PEV_BDN);
	PEV_inverter_start_pwm(PEV_BDN);

	return 0;
}
