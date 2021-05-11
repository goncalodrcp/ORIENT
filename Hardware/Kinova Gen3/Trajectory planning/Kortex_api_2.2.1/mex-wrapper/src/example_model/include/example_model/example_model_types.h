//
// File: example_model_types.h
//
// Code generated for Simulink model 'example_model'.
//
// Model version                  : 1.174
// Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
// C/C++ source code generated on : Tue Apr 13 16:36:20 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_example_model_types_h_
#define RTW_HEADER_example_model_types_h_
#include "rtwtypes.h"
#include "kortex_wrapper_data.h"

// Model Code Variants
#ifndef DEFINED_TYPEDEF_FOR_enumCmd_
#define DEFINED_TYPEDEF_FOR_enumCmd_

typedef int32_T enumCmd;

// enum enumCmd
const enumCmd undefined = 0;           // Default value
const enumCmd reboot_arm = 1;
const enumCmd emergency_stop = 2;
const enumCmd clear_faults = 3;
const enumCmd stop_action = 4;
const enumCmd pause_action = 5;
const enumCmd resume_action = 6;
const enumCmd precomputed_joint_trj = 101;
const enumCmd joint_reach = 200;
const enumCmd cartesian_reach = 300;
const enumCmd tool_reach = 400;
const enumCmd tool_speed = 401;

#endif

#ifndef DEFINED_TYPEDEF_FOR_enumToolMode_
#define DEFINED_TYPEDEF_FOR_enumToolMode_

typedef int32_T enumToolMode;

// enum enumToolMode
const enumToolMode toolMode_force = 1; // Default value
const enumToolMode toolMode_speed = 2;
const enumToolMode toolMode_reach = 3;

#endif

// Custom Type definition for MATLABSystem: '<Root>/MATLAB System'
#include "coder_posix_time.h"
#include "coder_posix_time.h"
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_real_T

#ifndef typedef_emxArray_real_T_example_model_T
#define typedef_emxArray_real_T_example_model_T

typedef struct emxArray_real_T emxArray_real_T_example_model_T;

#endif                                 //typedef_emxArray_real_T_example_model_T

#ifndef struct_tag_ssBunkPBGdZ0PgRklMjQ8gF
#define struct_tag_ssBunkPBGdZ0PgRklMjQ8gF

struct tag_ssBunkPBGdZ0PgRklMjQ8gF
{
  enumCmd cmd;
  real_T wayPoint;
  real_T constraint;
};

#endif                                 //struct_tag_ssBunkPBGdZ0PgRklMjQ8gF

#ifndef typedef_ssBunkPBGdZ0PgRklMjQ8gF_examp_T
#define typedef_ssBunkPBGdZ0PgRklMjQ8gF_examp_T

typedef struct tag_ssBunkPBGdZ0PgRklMjQ8gF ssBunkPBGdZ0PgRklMjQ8gF_examp_T;

#endif                                 //typedef_ssBunkPBGdZ0PgRklMjQ8gF_examp_T

#ifndef struct_tag_sfFtoaHdl8GQkNOkMLCauKG
#define struct_tag_sfFtoaHdl8GQkNOkMLCauKG

struct tag_sfFtoaHdl8GQkNOkMLCauKG
{
  enumCmd cmd;
  real_T wayPoint[6];
  real_T constraint[4];
};

#endif                                 //struct_tag_sfFtoaHdl8GQkNOkMLCauKG

#ifndef typedef_sfFtoaHdl8GQkNOkMLCauKG_examp_T
#define typedef_sfFtoaHdl8GQkNOkMLCauKG_examp_T

typedef struct tag_sfFtoaHdl8GQkNOkMLCauKG sfFtoaHdl8GQkNOkMLCauKG_examp_T;

#endif                                 //typedef_sfFtoaHdl8GQkNOkMLCauKG_examp_T

#ifndef struct_tag_s6EcqkLXKIeUpFCqeScqVGD
#define struct_tag_s6EcqkLXKIeUpFCqeScqVGD

struct tag_s6EcqkLXKIeUpFCqeScqVGD
{
  enumCmd cmd;
  real_T wayPoint[7];
  real_T constraint[2];
};

#endif                                 //struct_tag_s6EcqkLXKIeUpFCqeScqVGD

#ifndef typedef_s6EcqkLXKIeUpFCqeScqVGD_examp_T
#define typedef_s6EcqkLXKIeUpFCqeScqVGD_examp_T

typedef struct tag_s6EcqkLXKIeUpFCqeScqVGD s6EcqkLXKIeUpFCqeScqVGD_examp_T;

#endif                                 //typedef_s6EcqkLXKIeUpFCqeScqVGD_examp_T

#ifndef struct_tag_sQQJATZdbj9jSCddUdGiNLC
#define struct_tag_sQQJATZdbj9jSCddUdGiNLC

struct tag_sQQJATZdbj9jSCddUdGiNLC
{
  enumCmd cmd;
  real_T wayPoint[725000];
  real_T count;
  boolean_T convertToDeg;
};

#endif                                 //struct_tag_sQQJATZdbj9jSCddUdGiNLC

#ifndef typedef_sQQJATZdbj9jSCddUdGiNLC_examp_T
#define typedef_sQQJATZdbj9jSCddUdGiNLC_examp_T

typedef struct tag_sQQJATZdbj9jSCddUdGiNLC sQQJATZdbj9jSCddUdGiNLC_examp_T;

#endif                                 //typedef_sQQJATZdbj9jSCddUdGiNLC_examp_T

#ifndef struct_tag_fzD9VCXPF871MrbeHWYqEH
#define struct_tag_fzD9VCXPF871MrbeHWYqEH

struct tag_fzD9VCXPF871MrbeHWYqEH
{
  boolean_T matlabCodegenIsDeleted;
  boolean_T isSetupComplete;
  boolean_T in_nextCmdReadyToProcess;
  real_T guide_list_max_size;
  uint32_T guide_list[100];
  boolean_T simulation_ended;
  real_T currentGuideIndex;
  real_T currentCartIndex;
  real_T currentAngIndex;
  real_T currentPrecompIndex;
  real_T currentToolReachIndex;
  real_T currentToolSpeedIndex;
  uint32_T nextWayPointType;
  real_T cart_list_size;
  sfFtoaHdl8GQkNOkMLCauKG_examp_T trjCartWayPoints[10];
  real_T ang_list_size;
  s6EcqkLXKIeUpFCqeScqVGD_examp_T trjAngWayPoints[10];
  ssBunkPBGdZ0PgRklMjQ8gF_examp_T trjToolReachWayPoints[10];
  ssBunkPBGdZ0PgRklMjQ8gF_examp_T trjToolSpeedWayPoints[10];
  real_T precomp_list_size;
  sQQJATZdbj9jSCddUdGiNLC_examp_T trjPreCompWayPoints[10];
};

#endif                                 //struct_tag_fzD9VCXPF871MrbeHWYqEH

#ifndef typedef_kObjTrajectoryFeeder_example__T
#define typedef_kObjTrajectoryFeeder_example__T

typedef struct tag_fzD9VCXPF871MrbeHWYqEH kObjTrajectoryFeeder_example__T;

#endif                                 //typedef_kObjTrajectoryFeeder_example__T

// Parameters (default storage)
typedef struct P_example_model_T_ P_example_model_T;

// Forward declaration for rtModel
typedef struct tag_RTM_example_model_T RT_MODEL_example_model_T;

#endif                                 // RTW_HEADER_example_model_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
