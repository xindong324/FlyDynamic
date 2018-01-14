#ifndef __c1_Quan_h__
#define __c1_Quan_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc1_QuanInstanceStruct
#define typedef_SFc1_QuanInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_Quan;
  real_T *c1_X;
  real_T *c1_d_you;
  real_T *c1_Y;
  real_T *c1_Z;
  real_T *c1_L;
  real_T *c1_M;
  real_T *c1_N;
  real_T (*c1_state)[12];
  real_T *c1_d_v;
  real_T *c1_d_w;
  real_T *c1_d_p;
  real_T *c1_d_q;
  real_T *c1_d_r;
  real_T *c1_d_fai;
  real_T *c1_d_theta;
  real_T *c1_d_psi;
  real_T *c1_d_xe;
  real_T *c1_d_ye;
  real_T *c1_d_ze;
} SFc1_QuanInstanceStruct;

#endif                                 /*typedef_SFc1_QuanInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c1_Quan_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_Quan_get_check_sum(mxArray *plhs[]);
extern void c1_Quan_method_dispatcher(SimStruct *S, int_T method, void *data);

#endif
