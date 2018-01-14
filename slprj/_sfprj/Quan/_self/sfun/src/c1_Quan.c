/* Include files */

#include "Quan_sfun.h"
#include "c1_Quan.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "Quan_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c_with_debugger(S, sfGlobalDebugInstanceStruct);

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization);
static void chart_debug_initialize_data_addresses(SimStruct *S);
static const mxArray* sf_opaque_get_hover_data_for_msg(void *chartInstance,
  int32_T msgSSID);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c1_debug_family_names[51] = { "you", "v", "w", "p", "q", "r",
  "fai", "theta", "psi", "ze", "H", "V", "R", "g0", "m", "Ix", "Iy", "Iz", "Ixz",
  "Isum", "c1", "c2", "c3", "c4", "c5", "c6", "c7", "c8", "c9", "g", "nargin",
  "nargout", "X", "Y", "Z", "L", "M", "N", "state", "d_you", "d_v", "d_w", "d_p",
  "d_q", "d_r", "d_fai", "d_theta", "d_psi", "d_xe", "d_ye", "d_ze" };

/* Function Declarations */
static void initialize_c1_Quan(SFc1_QuanInstanceStruct *chartInstance);
static void initialize_params_c1_Quan(SFc1_QuanInstanceStruct *chartInstance);
static void enable_c1_Quan(SFc1_QuanInstanceStruct *chartInstance);
static void disable_c1_Quan(SFc1_QuanInstanceStruct *chartInstance);
static void c1_update_debugger_state_c1_Quan(SFc1_QuanInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c1_Quan(SFc1_QuanInstanceStruct
  *chartInstance);
static void set_sim_state_c1_Quan(SFc1_QuanInstanceStruct *chartInstance, const
  mxArray *c1_st);
static void finalize_c1_Quan(SFc1_QuanInstanceStruct *chartInstance);
static void sf_gateway_c1_Quan(SFc1_QuanInstanceStruct *chartInstance);
static void mdl_start_c1_Quan(SFc1_QuanInstanceStruct *chartInstance);
static void c1_chartstep_c1_Quan(SFc1_QuanInstanceStruct *chartInstance);
static void initSimStructsc1_Quan(SFc1_QuanInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static real_T c1_emlrt_marshallIn(SFc1_QuanInstanceStruct *chartInstance, const
  mxArray *c1_b_d_ze, const char_T *c1_identifier);
static real_T c1_b_emlrt_marshallIn(SFc1_QuanInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_mpower(SFc1_QuanInstanceStruct *chartInstance, real_T c1_a);
static void c1_scalarEg(SFc1_QuanInstanceStruct *chartInstance);
static void c1_dimagree(SFc1_QuanInstanceStruct *chartInstance);
static void c1_error(SFc1_QuanInstanceStruct *chartInstance);
static real_T c1_sqrt(SFc1_QuanInstanceStruct *chartInstance, real_T c1_x);
static void c1_b_error(SFc1_QuanInstanceStruct *chartInstance);
static real_T c1_sin(SFc1_QuanInstanceStruct *chartInstance, real_T c1_x);
static real_T c1_cos(SFc1_QuanInstanceStruct *chartInstance, real_T c1_x);
static real_T c1_tan(SFc1_QuanInstanceStruct *chartInstance, real_T c1_x);
static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_c_emlrt_marshallIn(SFc1_QuanInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static uint8_T c1_d_emlrt_marshallIn(SFc1_QuanInstanceStruct *chartInstance,
  const mxArray *c1_b_is_active_c1_Quan, const char_T *c1_identifier);
static uint8_T c1_e_emlrt_marshallIn(SFc1_QuanInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_sqrt(SFc1_QuanInstanceStruct *chartInstance, real_T *c1_x);
static void c1_b_sin(SFc1_QuanInstanceStruct *chartInstance, real_T *c1_x);
static void c1_b_cos(SFc1_QuanInstanceStruct *chartInstance, real_T *c1_x);
static void c1_b_tan(SFc1_QuanInstanceStruct *chartInstance, real_T *c1_x);
static void init_dsm_address_info(SFc1_QuanInstanceStruct *chartInstance);
static void init_simulink_io_address(SFc1_QuanInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c1_Quan(SFc1_QuanInstanceStruct *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc1_Quan(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c1_is_active_c1_Quan = 0U;
}

static void initialize_params_c1_Quan(SFc1_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c1_Quan(SFc1_QuanInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c1_Quan(SFc1_QuanInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c1_update_debugger_state_c1_Quan(SFc1_QuanInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c1_Quan(SFc1_QuanInstanceStruct
  *chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_y = NULL;
  real_T c1_hoistedGlobal;
  real_T c1_u;
  const mxArray *c1_b_y = NULL;
  real_T c1_b_hoistedGlobal;
  real_T c1_b_u;
  const mxArray *c1_c_y = NULL;
  real_T c1_c_hoistedGlobal;
  real_T c1_c_u;
  const mxArray *c1_d_y = NULL;
  real_T c1_d_hoistedGlobal;
  real_T c1_d_u;
  const mxArray *c1_e_y = NULL;
  real_T c1_e_hoistedGlobal;
  real_T c1_e_u;
  const mxArray *c1_f_y = NULL;
  real_T c1_f_hoistedGlobal;
  real_T c1_f_u;
  const mxArray *c1_g_y = NULL;
  real_T c1_g_hoistedGlobal;
  real_T c1_g_u;
  const mxArray *c1_h_y = NULL;
  real_T c1_h_hoistedGlobal;
  real_T c1_h_u;
  const mxArray *c1_i_y = NULL;
  real_T c1_i_hoistedGlobal;
  real_T c1_i_u;
  const mxArray *c1_j_y = NULL;
  real_T c1_j_hoistedGlobal;
  real_T c1_j_u;
  const mxArray *c1_k_y = NULL;
  real_T c1_k_hoistedGlobal;
  real_T c1_k_u;
  const mxArray *c1_l_y = NULL;
  real_T c1_l_hoistedGlobal;
  real_T c1_l_u;
  const mxArray *c1_m_y = NULL;
  uint8_T c1_m_hoistedGlobal;
  uint8_T c1_m_u;
  const mxArray *c1_n_y = NULL;
  c1_st = NULL;
  c1_st = NULL;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_createcellmatrix(13, 1), false);
  c1_hoistedGlobal = *chartInstance->c1_d_fai;
  c1_u = c1_hoistedGlobal;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 0, c1_b_y);
  c1_b_hoistedGlobal = *chartInstance->c1_d_p;
  c1_b_u = c1_b_hoistedGlobal;
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", &c1_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 1, c1_c_y);
  c1_c_hoistedGlobal = *chartInstance->c1_d_psi;
  c1_c_u = c1_c_hoistedGlobal;
  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", &c1_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 2, c1_d_y);
  c1_d_hoistedGlobal = *chartInstance->c1_d_q;
  c1_d_u = c1_d_hoistedGlobal;
  c1_e_y = NULL;
  sf_mex_assign(&c1_e_y, sf_mex_create("y", &c1_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 3, c1_e_y);
  c1_e_hoistedGlobal = *chartInstance->c1_d_r;
  c1_e_u = c1_e_hoistedGlobal;
  c1_f_y = NULL;
  sf_mex_assign(&c1_f_y, sf_mex_create("y", &c1_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 4, c1_f_y);
  c1_f_hoistedGlobal = *chartInstance->c1_d_theta;
  c1_f_u = c1_f_hoistedGlobal;
  c1_g_y = NULL;
  sf_mex_assign(&c1_g_y, sf_mex_create("y", &c1_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 5, c1_g_y);
  c1_g_hoistedGlobal = *chartInstance->c1_d_v;
  c1_g_u = c1_g_hoistedGlobal;
  c1_h_y = NULL;
  sf_mex_assign(&c1_h_y, sf_mex_create("y", &c1_g_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 6, c1_h_y);
  c1_h_hoistedGlobal = *chartInstance->c1_d_w;
  c1_h_u = c1_h_hoistedGlobal;
  c1_i_y = NULL;
  sf_mex_assign(&c1_i_y, sf_mex_create("y", &c1_h_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 7, c1_i_y);
  c1_i_hoistedGlobal = *chartInstance->c1_d_xe;
  c1_i_u = c1_i_hoistedGlobal;
  c1_j_y = NULL;
  sf_mex_assign(&c1_j_y, sf_mex_create("y", &c1_i_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 8, c1_j_y);
  c1_j_hoistedGlobal = *chartInstance->c1_d_ye;
  c1_j_u = c1_j_hoistedGlobal;
  c1_k_y = NULL;
  sf_mex_assign(&c1_k_y, sf_mex_create("y", &c1_j_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 9, c1_k_y);
  c1_k_hoistedGlobal = *chartInstance->c1_d_you;
  c1_k_u = c1_k_hoistedGlobal;
  c1_l_y = NULL;
  sf_mex_assign(&c1_l_y, sf_mex_create("y", &c1_k_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 10, c1_l_y);
  c1_l_hoistedGlobal = *chartInstance->c1_d_ze;
  c1_l_u = c1_l_hoistedGlobal;
  c1_m_y = NULL;
  sf_mex_assign(&c1_m_y, sf_mex_create("y", &c1_l_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 11, c1_m_y);
  c1_m_hoistedGlobal = chartInstance->c1_is_active_c1_Quan;
  c1_m_u = c1_m_hoistedGlobal;
  c1_n_y = NULL;
  sf_mex_assign(&c1_n_y, sf_mex_create("y", &c1_m_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c1_y, 12, c1_n_y);
  sf_mex_assign(&c1_st, c1_y, false);
  return c1_st;
}

static void set_sim_state_c1_Quan(SFc1_QuanInstanceStruct *chartInstance, const
  mxArray *c1_st)
{
  const mxArray *c1_u;
  chartInstance->c1_doneDoubleBufferReInit = true;
  c1_u = sf_mex_dup(c1_st);
  *chartInstance->c1_d_fai = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("d_fai", c1_u, 0)), "d_fai");
  *chartInstance->c1_d_p = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("d_p", c1_u, 1)), "d_p");
  *chartInstance->c1_d_psi = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("d_psi", c1_u, 2)), "d_psi");
  *chartInstance->c1_d_q = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("d_q", c1_u, 3)), "d_q");
  *chartInstance->c1_d_r = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("d_r", c1_u, 4)), "d_r");
  *chartInstance->c1_d_theta = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("d_theta", c1_u, 5)), "d_theta");
  *chartInstance->c1_d_v = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("d_v", c1_u, 6)), "d_v");
  *chartInstance->c1_d_w = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("d_w", c1_u, 7)), "d_w");
  *chartInstance->c1_d_xe = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("d_xe", c1_u, 8)), "d_xe");
  *chartInstance->c1_d_ye = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("d_ye", c1_u, 9)), "d_ye");
  *chartInstance->c1_d_you = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("d_you", c1_u, 10)), "d_you");
  *chartInstance->c1_d_ze = c1_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("d_ze", c1_u, 11)), "d_ze");
  chartInstance->c1_is_active_c1_Quan = c1_d_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell("is_active_c1_Quan", c1_u, 12)),
    "is_active_c1_Quan");
  sf_mex_destroy(&c1_u);
  c1_update_debugger_state_c1_Quan(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void finalize_c1_Quan(SFc1_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c1_Quan(SFc1_QuanInstanceStruct *chartInstance)
{
  int32_T c1_i0;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  for (c1_i0 = 0; c1_i0 < 12; c1_i0++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_state)[c1_i0], 6U, 1U, 0U,
                          chartInstance->c1_sfEvent, false);
  }

  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_N, 5U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_M, 4U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_L, 3U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_Z, 2U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_Y, 1U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_X, 0U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  chartInstance->c1_sfEvent = CALL_EVENT;
  c1_chartstep_c1_Quan(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_QuanMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_d_you, 7U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_d_v, 8U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_d_w, 9U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_d_p, 10U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_d_q, 11U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_d_r, 12U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_d_fai, 13U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_d_theta, 14U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_d_psi, 15U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_d_xe, 16U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_d_ye, 17U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_d_ze, 18U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
}

static void mdl_start_c1_Quan(SFc1_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c1_chartstep_c1_Quan(SFc1_QuanInstanceStruct *chartInstance)
{
  real_T c1_hoistedGlobal;
  real_T c1_b_hoistedGlobal;
  real_T c1_c_hoistedGlobal;
  real_T c1_d_hoistedGlobal;
  real_T c1_e_hoistedGlobal;
  real_T c1_f_hoistedGlobal;
  real_T c1_b_X;
  real_T c1_b_Y;
  real_T c1_b_Z;
  real_T c1_b_L;
  real_T c1_b_M;
  real_T c1_b_N;
  int32_T c1_i1;
  uint32_T c1_debug_family_var_map[51];
  real_T c1_b_state[12];
  real_T c1_you;
  real_T c1_v;
  real_T c1_w;
  real_T c1_p;
  real_T c1_q;
  real_T c1_r;
  real_T c1_fai;
  real_T c1_theta;
  real_T c1_psi;
  real_T c1_ze;
  real_T c1_H;
  real_T c1_V;
  real_T c1_R;
  real_T c1_g0;
  real_T c1_m;
  real_T c1_Ix;
  real_T c1_Iy;
  real_T c1_Iz;
  real_T c1_Ixz;
  real_T c1_Isum;
  real_T c1_c1;
  real_T c1_c2;
  real_T c1_c3;
  real_T c1_c4;
  real_T c1_c5;
  real_T c1_c6;
  real_T c1_c7;
  real_T c1_c8;
  real_T c1_c9;
  real_T c1_g;
  real_T c1_nargin = 7.0;
  real_T c1_nargout = 12.0;
  real_T c1_b_d_you;
  real_T c1_b_d_v;
  real_T c1_b_d_w;
  real_T c1_b_d_p;
  real_T c1_b_d_q;
  real_T c1_b_d_r;
  real_T c1_b_d_fai;
  real_T c1_b_d_theta;
  real_T c1_b_d_psi;
  real_T c1_b_d_xe;
  real_T c1_b_d_ye;
  real_T c1_b_d_ze;
  real_T c1_d0;
  real_T c1_A;
  real_T c1_x;
  real_T c1_b_x;
  real_T c1_y;
  real_T c1_d1;
  real_T c1_b_A;
  real_T c1_c_x;
  real_T c1_d_x;
  real_T c1_b_y;
  real_T c1_d2;
  real_T c1_d3;
  real_T c1_c_A;
  real_T c1_e_x;
  real_T c1_f_x;
  real_T c1_c_y;
  real_T c1_d4;
  real_T c1_d5;
  real_T c1_d6;
  real_T c1_d7;
  real_T c1_d8;
  real_T c1_d9;
  real_T c1_d10;
  real_T c1_d11;
  real_T c1_d12;
  real_T c1_d_A;
  real_T c1_B;
  real_T c1_g_x;
  real_T c1_d_y;
  real_T c1_h_x;
  real_T c1_e_y;
  real_T c1_i_x;
  real_T c1_j_x;
  real_T c1_k_x;
  real_T c1_l_x;
  real_T c1_m_x;
  real_T c1_n_x;
  real_T c1_o_x;
  real_T c1_p_x;
  real_T c1_q_x;
  real_T c1_r_x;
  real_T c1_s_x;
  real_T c1_t_x;
  real_T c1_u_x;
  real_T c1_v_x;
  real_T c1_w_x;
  real_T c1_x_x;
  real_T c1_y_x;
  real_T c1_ab_x;
  real_T c1_bb_x;
  real_T c1_cb_x;
  real_T c1_db_x;
  real_T c1_eb_x;
  real_T c1_fb_x;
  real_T c1_gb_x;
  real_T c1_hb_x;
  real_T c1_ib_x;
  real_T c1_jb_x;
  real_T c1_kb_x;
  real_T c1_lb_x;
  real_T c1_mb_x;
  real_T c1_nb_x;
  real_T c1_ob_x;
  real_T c1_pb_x;
  real_T c1_qb_x;
  real_T c1_rb_x;
  real_T c1_sb_x;
  real_T c1_tb_x;
  real_T c1_ub_x;
  real_T c1_vb_x;
  real_T c1_wb_x;
  real_T c1_xb_x;
  real_T c1_yb_x;
  real_T c1_ac_x;
  real_T c1_bc_x;
  real_T c1_cc_x;
  real_T c1_dc_x;
  real_T c1_ec_x;
  real_T c1_fc_x;
  real_T c1_gc_x;
  real_T c1_hc_x;
  real_T c1_ic_x;
  real_T c1_jc_x;
  real_T c1_kc_x;
  real_T c1_lc_x;
  real_T c1_mc_x;
  real_T c1_nc_x;
  real_T c1_oc_x;
  real_T c1_pc_x;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  c1_hoistedGlobal = *chartInstance->c1_X;
  c1_b_hoistedGlobal = *chartInstance->c1_Y;
  c1_c_hoistedGlobal = *chartInstance->c1_Z;
  c1_d_hoistedGlobal = *chartInstance->c1_L;
  c1_e_hoistedGlobal = *chartInstance->c1_M;
  c1_f_hoistedGlobal = *chartInstance->c1_N;
  c1_b_X = c1_hoistedGlobal;
  c1_b_Y = c1_b_hoistedGlobal;
  c1_b_Z = c1_c_hoistedGlobal;
  c1_b_L = c1_d_hoistedGlobal;
  c1_b_M = c1_e_hoistedGlobal;
  c1_b_N = c1_f_hoistedGlobal;
  for (c1_i1 = 0; c1_i1 < 12; c1_i1++) {
    c1_b_state[c1_i1] = (*chartInstance->c1_state)[c1_i1];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 51U, 51U, c1_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_you, 0U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_v, 1U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_w, 2U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_p, 3U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_q, 4U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_r, 5U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_fai, 6U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_theta, 7U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_psi, 8U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_ze, 9U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_H, 10U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_V, 11U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_R, 12U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_g0, 13U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_m, 14U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_Ix, 15U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_Iy, 16U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_Iz, 17U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_Ixz, 18U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_Isum, 19U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c1, 20U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c2, 21U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c3, 22U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c4, 23U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c5, 24U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c6, 25U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c7, 26U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c8, 27U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_c9, 28U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_g, 29U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 30U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 31U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_X, 32U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_Y, 33U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_Z, 34U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_L, 35U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_M, 36U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_N, 37U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_b_state, 38U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_d_you, 39U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_d_v, 40U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_d_w, 41U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_d_p, 42U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_d_q, 43U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_d_r, 44U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_d_fai, 45U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_d_theta, 46U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_d_psi, 47U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_d_xe, 48U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_d_ye, 49U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_b_d_ze, 50U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 4);
  c1_you = c1_b_state[0];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 5);
  c1_v = c1_b_state[1];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 6);
  c1_w = c1_b_state[2];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 7);
  c1_p = c1_b_state[3];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 8);
  c1_q = c1_b_state[4];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 9);
  c1_r = c1_b_state[5];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 10);
  c1_fai = c1_b_state[6];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 11);
  c1_theta = c1_b_state[7];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 12);
  c1_psi = c1_b_state[8];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 13);
  c1_ze = c1_b_state[11];
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 15);
  c1_H = 10000.0 - c1_ze;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 16);
  c1_d0 = (c1_mpower(chartInstance, c1_you) + c1_mpower(chartInstance, c1_v)) +
    c1_mpower(chartInstance, c1_w);
  c1_b_sqrt(chartInstance, &c1_d0);
  c1_V = c1_d0;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 19);
  c1_R = 6.356766E+6;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 19);
  c1_g0 = 9.80665;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 22);
  c1_m = 9298.643585;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 23);
  c1_Ix = 12874.8446;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 23);
  c1_Iy = 75673.6077;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 23);
  c1_Iz = 85552.0953;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 23);
  c1_Ixz = 1331.413;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 23);
  c1_Isum = 1.0996972716153214E+9;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 26);
  c1_c1 = -0.77011919099121517;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 27);
  c1_c2 = 0.027547656128945643;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 28);
  c1_c3 = 7.7796042154705343E-5;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 29);
  c1_c4 = 1.2107086507947013E-6;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 30);
  c1_c5 = 0.96040420047265718;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 31);
  c1_c6 = 0.017594152577979971;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 32);
  c1_c7 = 1.3214646828579841E-5;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 33);
  c1_c8 = -0.73361249157518171;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 34);
  c1_c9 = 1.1707626209791737E-5;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 37);
  c1_g = 9.7605319838536264;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 43);
  c1_A = c1_b_X;
  c1_x = c1_A;
  c1_b_x = c1_x;
  c1_y = c1_b_x / 9298.643585;
  c1_d1 = c1_theta;
  c1_b_sin(chartInstance, &c1_d1);
  c1_b_d_you = ((c1_y - c1_g * c1_d1) - c1_q * c1_w) + c1_r * c1_v;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 44);
  c1_b_A = c1_b_Y;
  c1_c_x = c1_b_A;
  c1_d_x = c1_c_x;
  c1_b_y = c1_d_x / 9298.643585;
  c1_d2 = c1_theta;
  c1_b_cos(chartInstance, &c1_d2);
  c1_d3 = c1_fai;
  c1_b_sin(chartInstance, &c1_d3);
  c1_b_d_v = ((c1_b_y + c1_g * c1_d2 * c1_d3) - c1_r * c1_you) + c1_p * c1_w;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 45);
  c1_c_A = c1_b_Z;
  c1_e_x = c1_c_A;
  c1_f_x = c1_e_x;
  c1_c_y = c1_f_x / 9298.643585;
  c1_d4 = c1_theta;
  c1_b_cos(chartInstance, &c1_d4);
  c1_d5 = c1_fai;
  c1_b_cos(chartInstance, &c1_d5);
  c1_b_d_w = ((c1_c_y + c1_g * c1_d4 * c1_d5) - c1_p * c1_v) + c1_q * c1_you;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 47);
  c1_b_d_p = ((c1_c1 * c1_r + c1_c2 * c1_p) * c1_q + c1_c3 * c1_b_L) + c1_c4 *
    c1_b_N;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 48);
  c1_b_d_q = (c1_c5 * c1_p * c1_r - c1_c6 * (c1_mpower(chartInstance, c1_p) -
    c1_mpower(chartInstance, c1_q))) + c1_c7 * c1_b_M;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 49);
  c1_b_d_r = ((c1_c8 * c1_p - c1_c2 * c1_r) * c1_q + c1_c4 * c1_b_L) + c1_c9 *
    c1_b_N;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 51);
  c1_d6 = c1_fai;
  c1_b_sin(chartInstance, &c1_d6);
  c1_d7 = c1_fai;
  c1_b_cos(chartInstance, &c1_d7);
  c1_d8 = c1_theta;
  c1_b_tan(chartInstance, &c1_d8);
  c1_b_d_fai = c1_p + (c1_q * c1_d6 + c1_r * c1_d7) * c1_d8;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 52);
  c1_d9 = c1_fai;
  c1_b_cos(chartInstance, &c1_d9);
  c1_d10 = c1_fai;
  c1_b_sin(chartInstance, &c1_d10);
  c1_b_d_theta = c1_q * c1_d9 - c1_r * c1_d10;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 53);
  c1_d11 = c1_fai;
  c1_b_sin(chartInstance, &c1_d11);
  c1_d12 = c1_fai;
  c1_b_cos(chartInstance, &c1_d12);
  c1_d_A = c1_q * c1_d11 + c1_r * c1_d12;
  c1_B = c1_theta;
  c1_b_cos(chartInstance, &c1_B);
  c1_g_x = c1_d_A;
  c1_d_y = c1_B;
  c1_h_x = c1_g_x;
  c1_e_y = c1_d_y;
  c1_b_d_psi = c1_h_x / c1_e_y;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 55);
  c1_i_x = c1_psi;
  c1_j_x = c1_i_x;
  c1_j_x = muDoubleScalarCos(c1_j_x);
  c1_k_x = c1_theta;
  c1_l_x = c1_k_x;
  c1_l_x = muDoubleScalarCos(c1_l_x);
  c1_m_x = c1_psi;
  c1_n_x = c1_m_x;
  c1_n_x = muDoubleScalarCos(c1_n_x);
  c1_o_x = c1_theta;
  c1_p_x = c1_o_x;
  c1_p_x = muDoubleScalarSin(c1_p_x);
  c1_q_x = c1_fai;
  c1_r_x = c1_q_x;
  c1_r_x = muDoubleScalarSin(c1_r_x);
  c1_s_x = c1_psi;
  c1_t_x = c1_s_x;
  c1_t_x = muDoubleScalarSin(c1_t_x);
  c1_u_x = c1_fai;
  c1_v_x = c1_u_x;
  c1_v_x = muDoubleScalarCos(c1_v_x);
  c1_w_x = c1_psi;
  c1_x_x = c1_w_x;
  c1_x_x = muDoubleScalarCos(c1_x_x);
  c1_y_x = c1_theta;
  c1_ab_x = c1_y_x;
  c1_ab_x = muDoubleScalarSin(c1_ab_x);
  c1_bb_x = c1_fai;
  c1_cb_x = c1_bb_x;
  c1_cb_x = muDoubleScalarCos(c1_cb_x);
  c1_db_x = c1_psi;
  c1_eb_x = c1_db_x;
  c1_eb_x = muDoubleScalarSin(c1_eb_x);
  c1_fb_x = c1_fai;
  c1_gb_x = c1_fb_x;
  c1_gb_x = muDoubleScalarSin(c1_gb_x);
  c1_b_d_xe = (c1_you * c1_j_x * c1_l_x + c1_v * (c1_n_x * c1_p_x * c1_r_x -
    c1_t_x * c1_v_x)) + c1_w * (c1_x_x * c1_ab_x * c1_cb_x + c1_eb_x * c1_gb_x);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 56);
  c1_hb_x = c1_psi;
  c1_ib_x = c1_hb_x;
  c1_ib_x = muDoubleScalarSin(c1_ib_x);
  c1_jb_x = c1_theta;
  c1_kb_x = c1_jb_x;
  c1_kb_x = muDoubleScalarCos(c1_kb_x);
  c1_lb_x = c1_psi;
  c1_mb_x = c1_lb_x;
  c1_mb_x = muDoubleScalarSin(c1_mb_x);
  c1_nb_x = c1_theta;
  c1_ob_x = c1_nb_x;
  c1_ob_x = muDoubleScalarSin(c1_ob_x);
  c1_pb_x = c1_fai;
  c1_qb_x = c1_pb_x;
  c1_qb_x = muDoubleScalarSin(c1_qb_x);
  c1_rb_x = c1_psi;
  c1_sb_x = c1_rb_x;
  c1_sb_x = muDoubleScalarCos(c1_sb_x);
  c1_tb_x = c1_fai;
  c1_ub_x = c1_tb_x;
  c1_ub_x = muDoubleScalarCos(c1_ub_x);
  c1_vb_x = c1_psi;
  c1_wb_x = c1_vb_x;
  c1_wb_x = muDoubleScalarSin(c1_wb_x);
  c1_xb_x = c1_theta;
  c1_yb_x = c1_xb_x;
  c1_yb_x = muDoubleScalarSin(c1_yb_x);
  c1_ac_x = c1_fai;
  c1_bc_x = c1_ac_x;
  c1_bc_x = muDoubleScalarCos(c1_bc_x);
  c1_cc_x = c1_psi;
  c1_dc_x = c1_cc_x;
  c1_dc_x = muDoubleScalarCos(c1_dc_x);
  c1_ec_x = c1_fai;
  c1_fc_x = c1_ec_x;
  c1_fc_x = muDoubleScalarSin(c1_fc_x);
  c1_b_d_ye = (c1_you * c1_ib_x * c1_kb_x + c1_v * (c1_mb_x * c1_ob_x * c1_qb_x
    + c1_sb_x * c1_ub_x)) + c1_w * (c1_wb_x * c1_yb_x * c1_bc_x - c1_dc_x *
    c1_fc_x);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 57);
  c1_gc_x = c1_theta;
  c1_hc_x = c1_gc_x;
  c1_hc_x = muDoubleScalarSin(c1_hc_x);
  c1_ic_x = c1_theta;
  c1_jc_x = c1_ic_x;
  c1_jc_x = muDoubleScalarCos(c1_jc_x);
  c1_kc_x = c1_fai;
  c1_lc_x = c1_kc_x;
  c1_lc_x = muDoubleScalarSin(c1_lc_x);
  c1_mc_x = c1_theta;
  c1_nc_x = c1_mc_x;
  c1_nc_x = muDoubleScalarCos(c1_nc_x);
  c1_oc_x = c1_fai;
  c1_pc_x = c1_oc_x;
  c1_pc_x = muDoubleScalarCos(c1_pc_x);
  c1_b_d_ze = (-c1_you * c1_hc_x + c1_v * c1_jc_x * c1_lc_x) + c1_w * c1_nc_x *
    c1_pc_x;
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -57);
  _SFD_SYMBOL_SCOPE_POP();
  *chartInstance->c1_d_you = c1_b_d_you;
  *chartInstance->c1_d_v = c1_b_d_v;
  *chartInstance->c1_d_w = c1_b_d_w;
  *chartInstance->c1_d_p = c1_b_d_p;
  *chartInstance->c1_d_q = c1_b_d_q;
  *chartInstance->c1_d_r = c1_b_d_r;
  *chartInstance->c1_d_fai = c1_b_d_fai;
  *chartInstance->c1_d_theta = c1_b_d_theta;
  *chartInstance->c1_d_psi = c1_b_d_psi;
  *chartInstance->c1_d_xe = c1_b_d_xe;
  *chartInstance->c1_d_ye = c1_b_d_ye;
  *chartInstance->c1_d_ze = c1_b_d_ze;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
}

static void initSimStructsc1_Quan(SFc1_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber)
{
  (void)c1_machineNumber;
  (void)c1_chartNumber;
  (void)c1_instanceNumber;
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  real_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuanInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static real_T c1_emlrt_marshallIn(SFc1_QuanInstanceStruct *chartInstance, const
  mxArray *c1_b_d_ze, const char_T *c1_identifier)
{
  real_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_d_ze), &c1_thisId);
  sf_mex_destroy(&c1_b_d_ze);
  return c1_y;
}

static real_T c1_b_emlrt_marshallIn(SFc1_QuanInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_y;
  real_T c1_d13;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d13, 1, 0, 0U, 0, 0U, 0);
  c1_y = c1_d13;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_d_ze;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_y;
  SFc1_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuanInstanceStruct *)chartInstanceVoid;
  c1_b_d_ze = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_d_ze), &c1_thisId);
  sf_mex_destroy(&c1_b_d_ze);
  *(real_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_i2;
  const mxArray *c1_y = NULL;
  real_T c1_u[12];
  SFc1_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuanInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  for (c1_i2 = 0; c1_i2 < 12; c1_i2++) {
    c1_u[c1_i2] = (*(real_T (*)[12])c1_inData)[c1_i2];
  }

  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 1, 12), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

const mxArray *sf_c1_Quan_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  sf_mex_assign(&c1_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c1_nameCaptureInfo;
}

static real_T c1_mpower(SFc1_QuanInstanceStruct *chartInstance, real_T c1_a)
{
  real_T c1_c;
  real_T c1_b_a;
  real_T c1_c_a;
  real_T c1_x;
  real_T c1_d_a;
  boolean_T c1_p;
  c1_b_a = c1_a;
  c1_c_a = c1_b_a;
  c1_x = c1_c_a;
  c1_d_a = c1_x;
  c1_c = c1_d_a * c1_d_a;
  c1_p = false;
  if (c1_p) {
    c1_error(chartInstance);
  }

  return c1_c;
}

static void c1_scalarEg(SFc1_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c1_dimagree(SFc1_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c1_error(SFc1_QuanInstanceStruct *chartInstance)
{
  const mxArray *c1_y = NULL;
  static char_T c1_u[31] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'p', 'o', 'w', 'e', 'r', '_', 'd', 'o', 'm', 'a', 'i',
    'n', 'E', 'r', 'r', 'o', 'r' };

  (void)chartInstance;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 31), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c1_y));
}

static real_T c1_sqrt(SFc1_QuanInstanceStruct *chartInstance, real_T c1_x)
{
  real_T c1_b_x;
  c1_b_x = c1_x;
  c1_b_sqrt(chartInstance, &c1_b_x);
  return c1_b_x;
}

static void c1_b_error(SFc1_QuanInstanceStruct *chartInstance)
{
  const mxArray *c1_y = NULL;
  static char_T c1_u[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  const mxArray *c1_b_y = NULL;
  static char_T c1_b_u[4] = { 's', 'q', 'r', 't' };

  (void)chartInstance;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", c1_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c1_y, 14, c1_b_y));
}

static real_T c1_sin(SFc1_QuanInstanceStruct *chartInstance, real_T c1_x)
{
  real_T c1_b_x;
  c1_b_x = c1_x;
  c1_b_sin(chartInstance, &c1_b_x);
  return c1_b_x;
}

static real_T c1_cos(SFc1_QuanInstanceStruct *chartInstance, real_T c1_x)
{
  real_T c1_b_x;
  c1_b_x = c1_x;
  c1_b_cos(chartInstance, &c1_b_x);
  return c1_b_x;
}

static real_T c1_tan(SFc1_QuanInstanceStruct *chartInstance, real_T c1_x)
{
  real_T c1_b_x;
  c1_b_x = c1_x;
  c1_b_tan(chartInstance, &c1_b_x);
  return c1_b_x;
}

static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData = NULL;
  int32_T c1_u;
  const mxArray *c1_y = NULL;
  SFc1_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuanInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_y = NULL;
  sf_mex_assign(&c1_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_y, false);
  return c1_mxArrayOutData;
}

static int32_T c1_c_emlrt_marshallIn(SFc1_QuanInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_y;
  int32_T c1_i3;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i3, 1, 6, 0U, 0, 0U, 0);
  c1_y = c1_i3;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_y;
  SFc1_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuanInstanceStruct *)chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static uint8_T c1_d_emlrt_marshallIn(SFc1_QuanInstanceStruct *chartInstance,
  const mxArray *c1_b_is_active_c1_Quan, const char_T *c1_identifier)
{
  uint8_T c1_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_y = c1_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_is_active_c1_Quan),
    &c1_thisId);
  sf_mex_destroy(&c1_b_is_active_c1_Quan);
  return c1_y;
}

static uint8_T c1_e_emlrt_marshallIn(SFc1_QuanInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_y;
  uint8_T c1_u0;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u0, 1, 3, 0U, 0, 0U, 0);
  c1_y = c1_u0;
  sf_mex_destroy(&c1_u);
  return c1_y;
}

static void c1_b_sqrt(SFc1_QuanInstanceStruct *chartInstance, real_T *c1_x)
{
  real_T c1_b_x;
  boolean_T c1_b0;
  boolean_T c1_p;
  c1_b_x = *c1_x;
  c1_b0 = (c1_b_x < 0.0);
  c1_p = c1_b0;
  if (c1_p) {
    c1_b_error(chartInstance);
  }

  *c1_x = muDoubleScalarSqrt(*c1_x);
}

static void c1_b_sin(SFc1_QuanInstanceStruct *chartInstance, real_T *c1_x)
{
  (void)chartInstance;
  *c1_x = muDoubleScalarSin(*c1_x);
}

static void c1_b_cos(SFc1_QuanInstanceStruct *chartInstance, real_T *c1_x)
{
  (void)chartInstance;
  *c1_x = muDoubleScalarCos(*c1_x);
}

static void c1_b_tan(SFc1_QuanInstanceStruct *chartInstance, real_T *c1_x)
{
  (void)chartInstance;
  *c1_x = muDoubleScalarTan(*c1_x);
}

static void init_dsm_address_info(SFc1_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc1_QuanInstanceStruct *chartInstance)
{
  chartInstance->c1_X = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    0);
  chartInstance->c1_d_you = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c1_Y = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    1);
  chartInstance->c1_Z = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    2);
  chartInstance->c1_L = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    3);
  chartInstance->c1_M = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    4);
  chartInstance->c1_N = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    5);
  chartInstance->c1_state = (real_T (*)[12])ssGetInputPortSignal_wrapper
    (chartInstance->S, 6);
  chartInstance->c1_d_v = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c1_d_w = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c1_d_p = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c1_d_q = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 5);
  chartInstance->c1_d_r = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 6);
  chartInstance->c1_d_fai = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 7);
  chartInstance->c1_d_theta = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 8);
  chartInstance->c1_d_psi = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 9);
  chartInstance->c1_d_xe = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 10);
  chartInstance->c1_d_ye = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 11);
  chartInstance->c1_d_ze = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 12);
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c1_Quan_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1297292808U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2560300875U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(4276802100U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3205853190U);
}

mxArray* sf_c1_Quan_get_post_codegen_info(void);
mxArray *sf_c1_Quan_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("LrMLJYWs4o7rIqs40O8AdD");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,7,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,1,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(12);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,12,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,7,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,7,"type",mxType);
    }

    mxSetField(mxData,7,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,8,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,8,"type",mxType);
    }

    mxSetField(mxData,8,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,9,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,9,"type",mxType);
    }

    mxSetField(mxData,9,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,10,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,10,"type",mxType);
    }

    mxSetField(mxData,10,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,11,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,11,"type",mxType);
    }

    mxSetField(mxData,11,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c1_Quan_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c1_Quan_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c1_Quan_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("pre");
  mxArray *fallbackReason = mxCreateString("hasBreakpoints");
  mxArray *hiddenFallbackType = mxCreateString("none");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c1_Quan_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c1_Quan_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c1_Quan(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[17],T\"d_fai\",},{M[1],M[14],T\"d_p\",},{M[1],M[19],T\"d_psi\",},{M[1],M[15],T\"d_q\",},{M[1],M[16],T\"d_r\",},{M[1],M[18],T\"d_theta\",},{M[1],M[12],T\"d_v\",},{M[1],M[13],T\"d_w\",},{M[1],M[20],T\"d_xe\",},{M[1],M[21],T\"d_ye\",}}",
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[5],T\"d_you\",},{M[1],M[22],T\"d_ze\",},{M[8],M[0],T\"is_active_c1_Quan\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 13, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_Quan_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_QuanInstanceStruct *chartInstance = (SFc1_QuanInstanceStruct *)
      sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _QuanMachineNumber_,
           1,
           1,
           1,
           0,
           19,
           0,
           0,
           0,
           0,
           0,
           &chartInstance->chartNumber,
           &chartInstance->instanceNumber,
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation(_QuanMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_QuanMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _QuanMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"X");
          _SFD_SET_DATA_PROPS(1,1,1,0,"Y");
          _SFD_SET_DATA_PROPS(2,1,1,0,"Z");
          _SFD_SET_DATA_PROPS(3,1,1,0,"L");
          _SFD_SET_DATA_PROPS(4,1,1,0,"M");
          _SFD_SET_DATA_PROPS(5,1,1,0,"N");
          _SFD_SET_DATA_PROPS(6,1,1,0,"state");
          _SFD_SET_DATA_PROPS(7,2,0,1,"d_you");
          _SFD_SET_DATA_PROPS(8,2,0,1,"d_v");
          _SFD_SET_DATA_PROPS(9,2,0,1,"d_w");
          _SFD_SET_DATA_PROPS(10,2,0,1,"d_p");
          _SFD_SET_DATA_PROPS(11,2,0,1,"d_q");
          _SFD_SET_DATA_PROPS(12,2,0,1,"d_r");
          _SFD_SET_DATA_PROPS(13,2,0,1,"d_fai");
          _SFD_SET_DATA_PROPS(14,2,0,1,"d_theta");
          _SFD_SET_DATA_PROPS(15,2,0,1,"d_psi");
          _SFD_SET_DATA_PROPS(16,2,0,1,"d_xe");
          _SFD_SET_DATA_PROPS(17,2,0,1,"d_ye");
          _SFD_SET_DATA_PROPS(18,2,0,1,"d_ze");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1681);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 12U;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(13,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(14,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(15,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(16,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(17,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(18,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)c1_sf_marshallIn);
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _QuanMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static void chart_debug_initialize_data_addresses(SimStruct *S)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_QuanInstanceStruct *chartInstance = (SFc1_QuanInstanceStruct *)
      sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c1_X);
        _SFD_SET_DATA_VALUE_PTR(7U, chartInstance->c1_d_you);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c1_Y);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c1_Z);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c1_L);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c1_M);
        _SFD_SET_DATA_VALUE_PTR(5U, chartInstance->c1_N);
        _SFD_SET_DATA_VALUE_PTR(6U, *chartInstance->c1_state);
        _SFD_SET_DATA_VALUE_PTR(8U, chartInstance->c1_d_v);
        _SFD_SET_DATA_VALUE_PTR(9U, chartInstance->c1_d_w);
        _SFD_SET_DATA_VALUE_PTR(10U, chartInstance->c1_d_p);
        _SFD_SET_DATA_VALUE_PTR(11U, chartInstance->c1_d_q);
        _SFD_SET_DATA_VALUE_PTR(12U, chartInstance->c1_d_r);
        _SFD_SET_DATA_VALUE_PTR(13U, chartInstance->c1_d_fai);
        _SFD_SET_DATA_VALUE_PTR(14U, chartInstance->c1_d_theta);
        _SFD_SET_DATA_VALUE_PTR(15U, chartInstance->c1_d_psi);
        _SFD_SET_DATA_VALUE_PTR(16U, chartInstance->c1_d_xe);
        _SFD_SET_DATA_VALUE_PTR(17U, chartInstance->c1_d_ye);
        _SFD_SET_DATA_VALUE_PTR(18U, chartInstance->c1_d_ze);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "shBYzAGjqL9VykECenzKjME";
}

static void sf_opaque_initialize_c1_Quan(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_QuanInstanceStruct*) chartInstanceVar)->S,0);
  initialize_params_c1_Quan((SFc1_QuanInstanceStruct*) chartInstanceVar);
  initialize_c1_Quan((SFc1_QuanInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c1_Quan(void *chartInstanceVar)
{
  enable_c1_Quan((SFc1_QuanInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c1_Quan(void *chartInstanceVar)
{
  disable_c1_Quan((SFc1_QuanInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c1_Quan(void *chartInstanceVar)
{
  sf_gateway_c1_Quan((SFc1_QuanInstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c1_Quan(SimStruct* S)
{
  return get_sim_state_c1_Quan((SFc1_QuanInstanceStruct *)
    sf_get_chart_instance_ptr(S));     /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c1_Quan(SimStruct* S, const mxArray *st)
{
  set_sim_state_c1_Quan((SFc1_QuanInstanceStruct*)sf_get_chart_instance_ptr(S),
                        st);
}

static void sf_opaque_terminate_c1_Quan(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_QuanInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_Quan_optimization_info();
    }

    finalize_c1_Quan((SFc1_QuanInstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_Quan((SFc1_QuanInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_Quan(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c1_Quan((SFc1_QuanInstanceStruct*)
      sf_get_chart_instance_ptr(S));
  }
}

static void mdlSetWorkWidths_c1_Quan(SimStruct *S)
{
  /* Set overwritable ports for inplace optimization */
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_Quan_optimization_info(sim_mode_is_rtw_gen(S),
      sim_mode_is_modelref_sim(S), sim_mode_is_external(S));
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,1,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_set_chart_accesses_machine_info(S, sf_get_instance_specialization(),
      infoStruct, 1);
    sf_update_buildInfo(S, sf_get_instance_specialization(),infoStruct,1);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 6, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,7);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,12);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=12; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 7; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2050421179U));
  ssSetChecksum1(S,(44561438U));
  ssSetChecksum2(S,(1078790347U));
  ssSetChecksum3(S,(1968086975U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSetStateSemanticsClassicAndSynchronous(S, true);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c1_Quan(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_Quan(SimStruct *S)
{
  SFc1_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc1_QuanInstanceStruct *)utMalloc(sizeof
    (SFc1_QuanInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc1_QuanInstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c1_Quan;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c1_Quan;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c1_Quan;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c1_Quan;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c1_Quan;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c1_Quan;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c1_Quan;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c1_Quan;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_Quan;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_Quan;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c1_Quan;
  chartInstance->chartInfo.callGetHoverDataForMsg = NULL;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  sf_init_ChartRunTimeInfo(S, &(chartInstance->chartInfo), false, 0);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  chart_debug_initialization(S,1);
  mdl_start_c1_Quan(chartInstance);
}

void c1_Quan_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_Quan(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_Quan(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_Quan(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_Quan_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
