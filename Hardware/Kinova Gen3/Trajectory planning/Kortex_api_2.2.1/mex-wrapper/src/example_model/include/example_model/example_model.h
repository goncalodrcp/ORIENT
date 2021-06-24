//
// File: example_model.h
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
#ifndef RTW_HEADER_example_model_h_
#define RTW_HEADER_example_model_h_
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include "rtwtypes.h"
#include "kortex_wrapper_data.h"
#include "kortexApiWrapper.h"
#include "example_model_types.h"
#include "MW_target_hardware_resources.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef struct_tag_LcMXxTokLCELhQ06jYvrfB
#define struct_tag_LcMXxTokLCELhQ06jYvrfB

struct tag_LcMXxTokLCELhQ06jYvrfB
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  boolean_T CacheInputSizes;
  uint32_T apiHandle;
  real_T nbrMaxJointActuators;
  real_T nbrMaxToolActuators;
  boolean_T kortex_isInitialized;
  boolean_T isCmdInProgress;
  enumCmd idCmdInProgress;
  real_T cmdToProcess;
  boolean_T processCmdSignal;
  emxArray_real_T_example_model_T *precompute_trj_position;
  emxArray_real_T_example_model_T *precompute_trj_velocity;
  emxArray_real_T_example_model_T *precompute_trj_acceleration;
  emxArray_real_T_example_model_T *precompute_trj_timestamp;
  real_T precompute_count;
  real_T cartesian_constraint[4];
  real_T cartesian_cmd[6];
  real_T joint_constraint[2];
  real_T joint_cmd[7];
  real_T tool_constraint;
  real_T tool_cmd;
  BaseFeedback baseFeedback;
  ActuatorsFeedback actuatorFeedback;
  InterconnectFeedback toolFeedback;
  IntrinsicParameters intrinsic_info;
  ExtrinsicParameters extrinsic_info;
  SensorSettings sensor_settings;
  boolean_T property_validation;
};

#endif                                 //struct_tag_LcMXxTokLCELhQ06jYvrfB

#ifndef typedef_kortex_example_model_T
#define typedef_kortex_example_model_T

typedef struct tag_LcMXxTokLCELhQ06jYvrfB kortex_example_model_T;

#endif                                 //typedef_kortex_example_model_T

// Block signals (default storage)
typedef struct {
  sQQJATZdbj9jSCddUdGiNLC_examp_T s;
  sQQJATZdbj9jSCddUdGiNLC_examp_T s_m;
  sQQJATZdbj9jSCddUdGiNLC_examp_T s_c;
  sQQJATZdbj9jSCddUdGiNLC_examp_T s_k;
  sQQJATZdbj9jSCddUdGiNLC_examp_T s_cx;
  sQQJATZdbj9jSCddUdGiNLC_examp_T s_b;
  sQQJATZdbj9jSCddUdGiNLC_examp_T s_p;
  sQQJATZdbj9jSCddUdGiNLC_examp_T s_cv;
  sQQJATZdbj9jSCddUdGiNLC_examp_T s_f;
  sQQJATZdbj9jSCddUdGiNLC_examp_T s_g;
  real_T b_varargout_5[175000];
  real_T b_varargout_4[175000];
  real_T b_varargout_3[175000];
  real_T MATLABSystem1_o6[25000];      // '<Root>/MATLAB System1'
  InterconnectFeedback b_varargout_6;
  ActuatorsFeedback b_varargout_4_g;
  BaseFeedback b_varargout_5_m;
  BaseFeedback tempBaseFeedback;
  sfFtoaHdl8GQkNOkMLCauKG_examp_T s_n;
  sfFtoaHdl8GQkNOkMLCauKG_examp_T s_pp;
  sfFtoaHdl8GQkNOkMLCauKG_examp_T s_l;
  sfFtoaHdl8GQkNOkMLCauKG_examp_T s_j;
  sfFtoaHdl8GQkNOkMLCauKG_examp_T s_d;
  sfFtoaHdl8GQkNOkMLCauKG_examp_T s_gu;
  sfFtoaHdl8GQkNOkMLCauKG_examp_T s_ld;
  sfFtoaHdl8GQkNOkMLCauKG_examp_T s_dh;
  sfFtoaHdl8GQkNOkMLCauKG_examp_T s_dy;
  sfFtoaHdl8GQkNOkMLCauKG_examp_T s_lx;
  IntrinsicParameters b_varargout_7;
  s6EcqkLXKIeUpFCqeScqVGD_examp_T s_o;
  s6EcqkLXKIeUpFCqeScqVGD_examp_T s_bj;
  s6EcqkLXKIeUpFCqeScqVGD_examp_T s_nu;
  s6EcqkLXKIeUpFCqeScqVGD_examp_T s_bs;
  s6EcqkLXKIeUpFCqeScqVGD_examp_T s_ln;
  s6EcqkLXKIeUpFCqeScqVGD_examp_T s_h;
  s6EcqkLXKIeUpFCqeScqVGD_examp_T s_bn;
  s6EcqkLXKIeUpFCqeScqVGD_examp_T s_da;
  s6EcqkLXKIeUpFCqeScqVGD_examp_T s_e;
  s6EcqkLXKIeUpFCqeScqVGD_examp_T s_bjv;
  real_T b_varargout_9[7];
  real_T joint_cmd[7];
  real_T b_varargout_11[6];
  ExtrinsicParameters b_varargout_8;
  real_T b_varargout_10[4];
  emxArray_real_T_example_model_T b_varargout_3_j;
  emxArray_real_T_example_model_T b_varargout_4_f;
  emxArray_real_T_example_model_T b_varargout_5_a;
  emxArray_real_T_example_model_T rtb_MATLABSystem1_o6_j;
  real_T translationCmd[3];
  real_T orientationCmd[3];
  SensorSettings b_varargout_9_j;
  real_T b_varargout_8_o[2];
  real_T b_varargout_13;
  real_T b_varargout_12;
  real_T b_varargout_7_n;
  real_T b_varargout_1;
  real_T tool_cmd;
  int32_T iv[2];
  int32_T iv1[2];
  int32_T iv2[2];
  int32_T MATLABSystem1_DIMS6[2];
  uint32_T b_varargout_3_i;
  uint32_T b_varargout_2;
  uint32_T result;
} B_example_model_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  kObjTrajectoryFeeder_example__T obj; // '<Root>/MATLAB System1'
  kortex_example_model_T obj_p;        // '<Root>/MATLAB System'
  int32_T MATLABSystem1_DIMS3[2];      // '<Root>/MATLAB System1'
  int32_T MATLABSystem1_DIMS4[2];      // '<Root>/MATLAB System1'
  int32_T MATLABSystem1_DIMS5[2];      // '<Root>/MATLAB System1'
  int32_T MATLABSystem1_DIMS6[2];      // '<Root>/MATLAB System1'
  boolean_T Memory_PreviousInput;      // '<Root>/Memory'
} DW_example_model_T;

// Parameters (default storage)
struct P_example_model_T_ {
  boolean_T Memory_InitialCondition;   // Expression: false
                                          //  Referenced by: '<Root>/Memory'

};

// Real-time Model Data Structure
struct tag_RTM_example_model_T {
  const char_T *errorStatus;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    boolean_T stopRequestedFlag;
  } Timing;
};

// Class declaration for model example_model
class example_modelModelClass {
  // public data and function members
 public:
  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  example_modelModelClass();

  // Destructor
  ~example_modelModelClass();

  // Real-Time Model get method
  RT_MODEL_example_model_T * getRTM();

  // private data and function members
 private:
  // Tunable parameters
  static P_example_model_T example_model_P;

  // Block signals
  B_example_model_T example_model_B;

  // Block states
  DW_example_model_T example_model_DW;

  // Real-Time Model
  RT_MODEL_example_model_T example_model_M;

  // private member function(s) for subsystem '<Root>'
  void e_kObjTrajectoryFeeder_stepImpl(kObjTrajectoryFeeder_example__T *obj,
    boolean_T varargin_1, real_T *varargout_1, boolean_T *varargout_2, real_T
    varargout_3[175000], real_T varargout_4[175000], real_T varargout_5[175000],
    real_T varargout_6[25000], real_T *varargout_7, real_T varargout_8[2],
    real_T varargout_9[7], real_T varargout_10[4], real_T varargout_11[6],
    real_T *varargout_12, real_T *varargout_13, boolean_T *varargout_14);
  void exampl_emxEnsureCapacity_real_T(emxArray_real_T_example_model_T *emxArray,
    int32_T oldNumel);
  void example_model_emxInit_real_T(emxArray_real_T_example_model_T **pEmxArray,
    int32_T numDimensions);
  void example_model_emxFree_real_T(emxArray_real_T_example_model_T **pEmxArray);
  void example_model_kortex_stepImpl(kortex_example_model_T *obj, real_T
    varargin_1, boolean_T varargin_2, const emxArray_real_T_example_model_T
    *varargin_3, const emxArray_real_T_example_model_T *varargin_4, const
    emxArray_real_T_example_model_T *varargin_5, const
    emxArray_real_T_example_model_T *varargin_6, real_T varargin_7, const real_T
    varargin_8[2], const real_T varargin_9[7], const real_T varargin_10[4],
    const real_T varargin_11[6], real_T varargin_12, real_T varargin_13,
    boolean_T *varargout_1, uint32_T *varargout_2, uint32_T *varargout_3,
    ActuatorsFeedback *varargout_4, BaseFeedback *varargout_5,
    InterconnectFeedback *varargout_6, IntrinsicParameters *varargout_7,
    ExtrinsicParameters *varargout_8, SensorSettings *varargout_9, boolean_T
    *varargout_10);
  void matlabCodegenHandle_matlabCod_h(kObjTrajectoryFeeder_example__T *obj);
  void example_mode_SystemCore_release(const kortex_example_model_T *obj);
  void example_model_SystemCore_delete(const kortex_example_model_T *obj);
  void matlabCodegenHandle_matlabCodeg(kortex_example_model_T *obj);
  void example_mo_emxFreeStruct_kortex(kortex_example_model_T *pStruct);
  kObjTrajectoryFeeder_example__T *kObjTrajectoryFeeder_kObjTrajec
    (kObjTrajectoryFeeder_example__T *obj);
  void kObjTrajectoryFeeder_AddWaypoin(kObjTrajectoryFeeder_example__T *obj);
  void example_mode_SystemCore_setup_h(kObjTrajectoryFeeder_example__T *obj);
  void example_mo_emxInitStruct_kortex(kortex_example_model_T *pStruct);
  kortex_example_model_T *example_model_kortex_kortex(kortex_example_model_T
    *obj);
  void example_model_SystemCore_setup(kortex_example_model_T *obj);
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Scope' : Unused code path elimination
//  Block '<Root>/Scope1' : Unused code path elimination
//  Block '<Root>/Scope2' : Unused code path elimination
//  Block '<Root>/Scope3' : Unused code path elimination
//  Block '<Root>/Scope7' : Unused code path elimination


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'example_model'

#endif                                 // RTW_HEADER_example_model_h_

//
// File trailer for generated code.
//
// [EOF]
//
