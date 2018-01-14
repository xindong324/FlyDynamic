/* Include files */

#include "Quan_sfun.h"
#include "c2_Quan.h"
#include <math.h>
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
static const char * c2_debug_family_names[101] = { "you", "v", "w", "p", "q",
  "r", "fai", "theta", "psi", "ze", "H", "V", "R", "c_", "S", "b", "r2d", "Vv",
  "alpha", "beta", "r2dalpha", "r2dbeta", "Ix", "Iy", "Iz", "Ixz", "Isum",
  "given_Alpha", "given_Elevator", "given_Cz", "given_Cx", "given_Cm",
  "given_Cl", "given_Cn", "given_Cl_delta_a", "given_Cl_delta_r",
  "given_Cn_delta_a", "given_Cn_delta_r", "given_throtte_000",
  "given_throtte_077", "given_throtte_100", "given_Cxq", "given_Cyr",
  "given_Cyp", "given_Czq", "given_Clr", "given_Clp", "given_Cmq", "given_Cnr",
  "given_Cnp", "given_delta_beta", "given_beta", "given_height", "given_mah",
  "rou0", "h1", "h2", "h_p", "WW", "T", "rou", "a", "Mah", "Cx_q", "Cy_r",
  "Cy_p", "Cz_q", "Cl_r", "Cl_p", "Cm_q", "Cn_r", "Cn_p", "Interp_throtte_000",
  "Interp_throtte_077", "Interp_throtte_100", "matrix_interp", "throtte_value",
  "X_thrust", "Cx", "Cy", "Cz", "Cl_delta_a", "Cl_delta_r", "Cl", "Cm",
  "Cn_delta_a", "Cn_delta_r", "Cn", "nargin", "nargout", "delta_e", "delta_r",
  "delta_a", "delta_T", "state", "X", "Y", "Z", "L", "M", "N" };

/* Function Declarations */
static void initialize_c2_Quan(SFc2_QuanInstanceStruct *chartInstance);
static void initialize_params_c2_Quan(SFc2_QuanInstanceStruct *chartInstance);
static void enable_c2_Quan(SFc2_QuanInstanceStruct *chartInstance);
static void disable_c2_Quan(SFc2_QuanInstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_Quan(SFc2_QuanInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c2_Quan(SFc2_QuanInstanceStruct
  *chartInstance);
static void set_sim_state_c2_Quan(SFc2_QuanInstanceStruct *chartInstance, const
  mxArray *c2_st);
static void finalize_c2_Quan(SFc2_QuanInstanceStruct *chartInstance);
static void sf_gateway_c2_Quan(SFc2_QuanInstanceStruct *chartInstance);
static void mdl_start_c2_Quan(SFc2_QuanInstanceStruct *chartInstance);
static void c2_chartstep_c2_Quan(SFc2_QuanInstanceStruct *chartInstance);
static void initSimStructsc2_Quan(SFc2_QuanInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static real_T c2_emlrt_marshallIn(SFc2_QuanInstanceStruct *chartInstance, const
  mxArray *c2_b_N, const char_T *c2_identifier);
static real_T c2_b_emlrt_marshallIn(SFc2_QuanInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_c_emlrt_marshallIn(SFc2_QuanInstanceStruct *chartInstance, const
  mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[3]);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_k_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_mpower(SFc2_QuanInstanceStruct *chartInstance, real_T c2_a);
static void c2_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static void c2_dimagree(SFc2_QuanInstanceStruct *chartInstance);
static boolean_T c2_fltpower_domain_error(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_a, real_T c2_b);
static void c2_error(SFc2_QuanInstanceStruct *chartInstance);
static real_T c2_sqrt(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x);
static void c2_b_error(SFc2_QuanInstanceStruct *chartInstance);
static real_T c2_atan(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x);
static real_T c2_asin(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x);
static void c2_c_error(SFc2_QuanInstanceStruct *chartInstance);
static real_T c2_b_mpower(SFc2_QuanInstanceStruct *chartInstance, real_T c2_a);
static real_T c2_exp(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x);
static real_T c2_c_mpower(SFc2_QuanInstanceStruct *chartInstance, real_T c2_a);
static real_T c2_d_mpower(SFc2_QuanInstanceStruct *chartInstance, real_T c2_a);
static void c2_b_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static void c2_StringToMethodID(SFc2_QuanInstanceStruct *chartInstance);
static void c2_d_error(SFc2_QuanInstanceStruct *chartInstance);
static void c2_e_error(SFc2_QuanInstanceStruct *chartInstance);
static real_T c2_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_3);
static real_T c2_ppval(SFc2_QuanInstanceStruct *chartInstance,
  c2_sSYDZjSrzjRAmypAou1DUJH *c2_pp, real_T c2_x);
static int32_T c2_bsearch(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x[12],
  real_T c2_xi);
static real_T c2_b_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_3);
static real_T c2_c_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_3);
static real_T c2_d_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_3);
static real_T c2_e_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_3);
static real_T c2_f_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_3);
static real_T c2_g_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_3);
static real_T c2_h_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_3);
static real_T c2_i_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_3);
static real_T c2_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5);
static void c2_b_StringToMethodID(SFc2_QuanInstanceStruct *chartInstance);
static void c2_interp2_validate(SFc2_QuanInstanceStruct *chartInstance, uint8_T
  c2_METHOD);
static void c2_c_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static boolean_T c2_isplaid(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_1, real_T c2_varargin_2);
static void c2_d_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static real_T c2_TensorInterp23(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5);
static real_T c2_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4, real_T c2_varargin_5);
static void c2_splinepp(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[36],
  c2_s5aHNKucFmPykI7NAl3JfH *c2_pp);
static void c2_chckxy(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[36]);
static void c2_e_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static void c2_pwchcore(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x[6],
  real_T c2_y[36], real_T c2_s[36], real_T c2_dx[5], real_T c2_divdif[30],
  c2_s5aHNKucFmPykI7NAl3JfH *c2_pp);
static void c2_f_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static void c2_b_ppval(SFc2_QuanInstanceStruct *chartInstance,
  c2_s5aHNKucFmPykI7NAl3JfH *c2_pp, real_T c2_x, real_T c2_v[6]);
static int32_T c2_b_bsearch(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x
  [6], real_T c2_xi);
static void c2_b_splinepp(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[6],
  c2_syzabYTcbAiThYrkWkxvAFC *c2_pp);
static void c2_b_chckxy(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[6]);
static void c2_g_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static void c2_b_pwchcore(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x[6],
  real_T c2_y[6], real_T c2_s[6], real_T c2_dx[5], real_T c2_divdif[5],
  c2_syzabYTcbAiThYrkWkxvAFC *c2_pp);
static void c2_h_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static real_T c2_c_ppval(SFc2_QuanInstanceStruct *chartInstance,
  c2_syzabYTcbAiThYrkWkxvAFC *c2_pp, real_T c2_x);
static void c2_unique_vector(SFc2_QuanInstanceStruct *chartInstance, real_T c2_a,
  real_T c2_b_data[], int32_T c2_b_sizes[2], int32_T c2_ndx_data[], int32_T
  *c2_ndx_sizes, int32_T *c2_pos);
static int32_T c2_sortIdx(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x);
static void c2_i_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static void c2_count_nonfinites(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_b_data[], int32_T c2_b_sizes[2], int32_T c2_nb, int32_T *c2_nMInf, int32_T *
  c2_nFinite, int32_T *c2_nPInf, int32_T *c2_nNaN);
static void c2_check_forloop_overflow_error(SFc2_QuanInstanceStruct
  *chartInstance, boolean_T c2_overflow);
static real_T c2_abs(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x);
static void c2_output_size(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_1_data[], int32_T c2_varargin_1_sizes[2], real_T
  c2_varargin_2_data[], int32_T c2_varargin_2_sizes[2], real_T c2_sz[2]);
static boolean_T c2_iscolumn(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_x_data[], int32_T c2_x_sizes[2]);
static void c2_spline(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y_data[],
                      int32_T c2_y_sizes[2], c2_sca5AIK8nC7yf5Qh9WghmhF
                      *c2_output_data, c2_sca5AIK8nC7yf5Qh9WghmhF_size
                      *c2_output_elems_sizes);
static void c2_c_chckxy(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_y_data[], int32_T c2_y_sizes[2]);
static boolean_T c2_anyIsNaN(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_y_data[], int32_T c2_y_sizes[2]);
static void c2_j_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static void c2_k_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static void c2_c_pwchcore(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x[6],
  real_T c2_y_data[], int32_T c2_y_sizes[2], int32_T c2_yoffset, real_T
  c2_s_data[], int32_T c2_s_sizes[2], real_T c2_dx[5], real_T c2_divdif_data[],
  int32_T c2_divdif_sizes[2], c2_sca5AIK8nC7yf5Qh9WghmhF *c2_pp_data,
  c2_sca5AIK8nC7yf5Qh9WghmhF_size *c2_pp_elems_sizes);
static void c2_d_ppval(SFc2_QuanInstanceStruct *chartInstance,
  c2_sca5AIK8nC7yf5Qh9WghmhF *c2_pp_data, c2_sca5AIK8nC7yf5Qh9WghmhF_size
  c2_pp_elems_sizes, real_T c2_x, real_T c2_v_data[], int32_T *c2_v_sizes);
static void c2_l_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static real_T c2_b_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5);
static real_T c2_b_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_3[36], real_T c2_varargin_4, real_T c2_varargin_5);
static real_T c2_b_TensorInterp23(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5);
static real_T c2_c_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5);
static real_T c2_c_TensorInterp23(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5);
static real_T c2_j_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_1[3], real_T c2_varargin_2[3], real_T c2_varargin_3);
static void c2_m_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static void c2_c_StringToMethodID(SFc2_QuanInstanceStruct *chartInstance);
static real_T c2_d_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5);
static void c2_b_interp2_validate(SFc2_QuanInstanceStruct *chartInstance,
  uint8_T c2_METHOD);
static void c2_n_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static void c2_o_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static real_T c2_c_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4, real_T c2_varargin_5);
static void c2_b_spline(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[60],
  c2_samQD8b77pHdwkX3iY74akB *c2_output);
static void c2_c_splinepp(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[60],
  c2_samQD8b77pHdwkX3iY74akB *c2_pp);
static void c2_d_chckxy(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[60]);
static void c2_p_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static void c2_d_pwchcore(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x[5],
  real_T c2_y[60], real_T c2_s[60], real_T c2_dx[4], real_T c2_divdif[48],
  c2_samQD8b77pHdwkX3iY74akB *c2_pp);
static void c2_q_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static void c2_e_ppval(SFc2_QuanInstanceStruct *chartInstance,
  c2_samQD8b77pHdwkX3iY74akB *c2_pp, real_T c2_x, real_T c2_v[12]);
static void c2_c_spline(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[12],
  c2_sSYDZjSrzjRAmypAou1DUJH *c2_output);
static void c2_d_splinepp(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[12],
  c2_sSYDZjSrzjRAmypAou1DUJH *c2_pp);
static void c2_e_chckxy(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[12]);
static void c2_r_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static void c2_e_pwchcore(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x[12],
  real_T c2_y[12], real_T c2_s[12], real_T c2_dx[11], real_T c2_divdif[11],
  c2_sSYDZjSrzjRAmypAou1DUJH *c2_pp);
static void c2_s_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static void c2_d_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4_data[], int32_T c2_varargin_4_sizes[2], real_T
  c2_varargin_5_data[], int32_T c2_varargin_5_sizes[2], real_T c2_yi_data[],
  int32_T c2_yi_sizes[2]);
static void c2_d_spline(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_y_data[], int32_T c2_y_sizes[2], c2_sL3Gi7vZqpbfvDjMPafkGnG *c2_output_data,
  c2_sL3Gi7vZqpbfvDjMPafkGnG_size *c2_output_elems_sizes);
static void c2_f_chckxy(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_y_data[], int32_T c2_y_sizes[2]);
static void c2_t_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static void c2_u_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static void c2_f_pwchcore(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x[12],
  real_T c2_y_data[], int32_T c2_y_sizes[2], int32_T c2_yoffset, real_T
  c2_s_data[], int32_T c2_s_sizes[2], real_T c2_dx[11], real_T c2_divdif_data[],
  int32_T c2_divdif_sizes[2], c2_sL3Gi7vZqpbfvDjMPafkGnG *c2_pp_data,
  c2_sL3Gi7vZqpbfvDjMPafkGnG_size *c2_pp_elems_sizes);
static void c2_f_ppval(SFc2_QuanInstanceStruct *chartInstance,
  c2_sL3Gi7vZqpbfvDjMPafkGnG *c2_pp_data, c2_sL3Gi7vZqpbfvDjMPafkGnG_size
  c2_pp_elems_sizes, real_T c2_x, real_T c2_v_data[], int32_T *c2_v_sizes);
static void c2_v_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static real_T c2_k_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_3);
static real_T c2_e_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5);
static void c2_c_interp2_validate(SFc2_QuanInstanceStruct *chartInstance,
  uint8_T c2_METHOD);
static void c2_w_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static void c2_x_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static boolean_T c2_b_anyIsNaN(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_y[84]);
static real_T c2_interp2_dispatch(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_Xq, real_T c2_Yq);
static real_T c2_e_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4, real_T c2_varargin_5);
static void c2_e_spline(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[84],
  c2_sPMb9Nq525A1JP3SLksK2fC *c2_output);
static void c2_e_splinepp(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[84],
  c2_sPMb9Nq525A1JP3SLksK2fC *c2_pp);
static void c2_g_chckxy(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[84]);
static void c2_y_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static void c2_g_pwchcore(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x[7],
  real_T c2_y[84], real_T c2_s[84], real_T c2_dx[6], real_T c2_divdif[72],
  c2_sPMb9Nq525A1JP3SLksK2fC *c2_pp);
static void c2_ab_scalarEg(SFc2_QuanInstanceStruct *chartInstance);
static void c2_g_ppval(SFc2_QuanInstanceStruct *chartInstance,
  c2_sPMb9Nq525A1JP3SLksK2fC *c2_pp, real_T c2_x, real_T c2_v[12]);
static void c2_f_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4_data[], int32_T c2_varargin_4_sizes[2], real_T
  c2_varargin_5_data[], int32_T c2_varargin_5_sizes[2], real_T c2_yi_data[],
  int32_T c2_yi_sizes[2]);
static void c2_intermediate_size(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_b_data[], int32_T c2_b_sizes[2], real_T c2_sz[2]);
static real_T c2_f_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5);
static real_T c2_g_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_3[84], real_T c2_varargin_4, real_T c2_varargin_5);
static real_T c2_b_interp2_dispatch(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_Xq, real_T c2_Yq);
static void c2_h_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4_data[], int32_T c2_varargin_4_sizes[2], real_T
  c2_varargin_5_data[], int32_T c2_varargin_5_sizes[2], real_T c2_yi_data[],
  int32_T c2_yi_sizes[2]);
static real_T c2_sign(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x);
static real_T c2_g_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5);
static real_T c2_c_interp2_dispatch(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_Xq, real_T c2_Yq);
static real_T c2_i_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4, real_T c2_varargin_5);
static void c2_f_spline(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[84],
  c2_sPMb9Nq525A1JP3SLksK2fC *c2_output);
static void c2_f_splinepp(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[84],
  c2_sPMb9Nq525A1JP3SLksK2fC *c2_pp);
static void c2_h_chckxy(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[84]);
static void c2_j_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4_data[], int32_T c2_varargin_4_sizes[2], real_T
  c2_varargin_5_data[], int32_T c2_varargin_5_sizes[2], real_T c2_yi_data[],
  int32_T c2_yi_sizes[2]);
static real_T c2_h_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5);
static real_T c2_k_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_3[60], real_T c2_varargin_4, real_T c2_varargin_5);
static void c2_l_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4_data[], int32_T c2_varargin_4_sizes[2], real_T
  c2_varargin_5_data[], int32_T c2_varargin_5_sizes[2], real_T c2_yi_data[],
  int32_T c2_yi_sizes[2]);
static real_T c2_i_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5);
static real_T c2_d_interp2_dispatch(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_Xq, real_T c2_Yq);
static void c2_m_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4_data[], int32_T c2_varargin_4_sizes[2], real_T
  c2_varargin_5_data[], int32_T c2_varargin_5_sizes[2], real_T c2_yi_data[],
  int32_T c2_yi_sizes[2]);
static real_T c2_j_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5);
static real_T c2_e_interp2_dispatch(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_Xq, real_T c2_Yq);
static void c2_n_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4_data[], int32_T c2_varargin_4_sizes[2], real_T
  c2_varargin_5_data[], int32_T c2_varargin_5_sizes[2], real_T c2_yi_data[],
  int32_T c2_yi_sizes[2]);
static real_T c2_k_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5);
static real_T c2_o_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_3[84], real_T c2_varargin_4, real_T c2_varargin_5);
static real_T c2_f_interp2_dispatch(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_Xq, real_T c2_Yq);
static void c2_p_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4_data[], int32_T c2_varargin_4_sizes[2], real_T
  c2_varargin_5_data[], int32_T c2_varargin_5_sizes[2], real_T c2_yi_data[],
  int32_T c2_yi_sizes[2]);
static const mxArray *c2_l_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_d_emlrt_marshallIn(SFc2_QuanInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_e_emlrt_marshallIn(SFc2_QuanInstanceStruct *chartInstance,
  const mxArray *c2_b_is_active_c2_Quan, const char_T *c2_identifier);
static uint8_T c2_f_emlrt_marshallIn(SFc2_QuanInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sqrt(SFc2_QuanInstanceStruct *chartInstance, real_T *c2_x);
static void c2_b_atan(SFc2_QuanInstanceStruct *chartInstance, real_T *c2_x);
static void c2_b_asin(SFc2_QuanInstanceStruct *chartInstance, real_T *c2_x);
static void c2_b_exp(SFc2_QuanInstanceStruct *chartInstance, real_T *c2_x);
static void c2_b_sign(SFc2_QuanInstanceStruct *chartInstance, real_T *c2_x);
static void init_dsm_address_info(SFc2_QuanInstanceStruct *chartInstance);
static void init_simulink_io_address(SFc2_QuanInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c2_Quan(SFc2_QuanInstanceStruct *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc2_Quan(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c2_is_active_c2_Quan = 0U;
}

static void initialize_params_c2_Quan(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c2_Quan(SFc2_QuanInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c2_Quan(SFc2_QuanInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c2_update_debugger_state_c2_Quan(SFc2_QuanInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c2_Quan(SFc2_QuanInstanceStruct
  *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  real_T c2_hoistedGlobal;
  real_T c2_u;
  const mxArray *c2_b_y = NULL;
  real_T c2_b_hoistedGlobal;
  real_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  real_T c2_c_hoistedGlobal;
  real_T c2_c_u;
  const mxArray *c2_d_y = NULL;
  real_T c2_d_hoistedGlobal;
  real_T c2_d_u;
  const mxArray *c2_e_y = NULL;
  real_T c2_e_hoistedGlobal;
  real_T c2_e_u;
  const mxArray *c2_f_y = NULL;
  real_T c2_f_hoistedGlobal;
  real_T c2_f_u;
  const mxArray *c2_g_y = NULL;
  uint8_T c2_g_hoistedGlobal;
  uint8_T c2_g_u;
  const mxArray *c2_h_y = NULL;
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellmatrix(7, 1), false);
  c2_hoistedGlobal = *chartInstance->c2_L;
  c2_u = c2_hoistedGlobal;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_b_hoistedGlobal = *chartInstance->c2_M;
  c2_b_u = c2_b_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_c_hoistedGlobal = *chartInstance->c2_N;
  c2_c_u = c2_c_hoistedGlobal;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 2, c2_d_y);
  c2_d_hoistedGlobal = *chartInstance->c2_X;
  c2_d_u = c2_d_hoistedGlobal;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 3, c2_e_y);
  c2_e_hoistedGlobal = *chartInstance->c2_Y;
  c2_e_u = c2_e_hoistedGlobal;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 4, c2_f_y);
  c2_f_hoistedGlobal = *chartInstance->c2_Z;
  c2_f_u = c2_f_hoistedGlobal;
  c2_g_y = NULL;
  sf_mex_assign(&c2_g_y, sf_mex_create("y", &c2_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 5, c2_g_y);
  c2_g_hoistedGlobal = chartInstance->c2_is_active_c2_Quan;
  c2_g_u = c2_g_hoistedGlobal;
  c2_h_y = NULL;
  sf_mex_assign(&c2_h_y, sf_mex_create("y", &c2_g_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 6, c2_h_y);
  sf_mex_assign(&c2_st, c2_y, false);
  return c2_st;
}

static void set_sim_state_c2_Quan(SFc2_QuanInstanceStruct *chartInstance, const
  mxArray *c2_st)
{
  const mxArray *c2_u;
  chartInstance->c2_doneDoubleBufferReInit = true;
  c2_u = sf_mex_dup(c2_st);
  *chartInstance->c2_L = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("L", c2_u, 0)), "L");
  *chartInstance->c2_M = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("M", c2_u, 1)), "M");
  *chartInstance->c2_N = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("N", c2_u, 2)), "N");
  *chartInstance->c2_X = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("X", c2_u, 3)), "X");
  *chartInstance->c2_Y = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("Y", c2_u, 4)), "Y");
  *chartInstance->c2_Z = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("Z", c2_u, 5)), "Z");
  chartInstance->c2_is_active_c2_Quan = c2_e_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell("is_active_c2_Quan", c2_u, 6)),
    "is_active_c2_Quan");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_Quan(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_Quan(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c2_Quan(SFc2_QuanInstanceStruct *chartInstance)
{
  int32_T c2_i0;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  for (c2_i0 = 0; c2_i0 < 12; c2_i0++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_state)[c2_i0], 4U, 1U, 0U,
                          chartInstance->c2_sfEvent, false);
  }

  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_delta_T, 3U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_delta_a, 2U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_delta_r, 1U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_delta_e, 0U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_Quan(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_QuanMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_X, 5U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_Y, 6U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_Z, 7U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_L, 8U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_M, 9U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_N, 10U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
}

static void mdl_start_c2_Quan(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_chartstep_c2_Quan(SFc2_QuanInstanceStruct *chartInstance)
{
  real_T c2_hoistedGlobal;
  real_T c2_b_hoistedGlobal;
  real_T c2_c_hoistedGlobal;
  real_T c2_d_hoistedGlobal;
  real_T c2_b_delta_e;
  real_T c2_b_delta_r;
  real_T c2_b_delta_a;
  real_T c2_b_delta_T;
  int32_T c2_i1;
  uint32_T c2_debug_family_var_map[101];
  real_T c2_b_state[12];
  real_T c2_you;
  real_T c2_v;
  real_T c2_w;
  real_T c2_p;
  real_T c2_q;
  real_T c2_r;
  real_T c2_fai;
  real_T c2_theta;
  real_T c2_psi;
  real_T c2_ze;
  real_T c2_H;
  real_T c2_V;
  real_T c2_R;
  real_T c2_c_;
  real_T c2_S;
  real_T c2_b;
  real_T c2_r2d;
  real_T c2_Vv;
  real_T c2_alpha;
  real_T c2_beta;
  real_T c2_r2dalpha;
  real_T c2_r2dbeta;
  real_T c2_Ix;
  real_T c2_Iy;
  real_T c2_Iz;
  real_T c2_Ixz;
  real_T c2_Isum;
  real_T c2_given_Alpha[12];
  real_T c2_given_Elevator[5];
  real_T c2_given_Cz[12];
  real_T c2_given_Cx[60];
  real_T c2_given_Cm[60];
  real_T c2_given_Cl[84];
  real_T c2_given_Cn[84];
  real_T c2_given_Cl_delta_a[84];
  real_T c2_given_Cl_delta_r[84];
  real_T c2_given_Cn_delta_a[84];
  real_T c2_given_Cn_delta_r[84];
  real_T c2_given_throtte_000[36];
  real_T c2_given_throtte_077[36];
  real_T c2_given_throtte_100[36];
  real_T c2_given_Cxq[12];
  real_T c2_given_Cyr[12];
  real_T c2_given_Cyp[12];
  real_T c2_given_Czq[12];
  real_T c2_given_Clr[12];
  real_T c2_given_Clp[12];
  real_T c2_given_Cmq[12];
  real_T c2_given_Cnr[12];
  real_T c2_given_Cnp[12];
  real_T c2_given_delta_beta[7];
  real_T c2_given_beta[7];
  real_T c2_given_height[6];
  real_T c2_given_mah[6];
  real_T c2_rou0;
  real_T c2_h1;
  real_T c2_h2;
  real_T c2_h_p;
  real_T c2_WW;
  real_T c2_T;
  real_T c2_rou;
  real_T c2_a;
  real_T c2_Mah;
  real_T c2_Cx_q;
  real_T c2_Cy_r;
  real_T c2_Cy_p;
  real_T c2_Cz_q;
  real_T c2_Cl_r;
  real_T c2_Cl_p;
  real_T c2_Cm_q;
  real_T c2_Cn_r;
  real_T c2_Cn_p;
  real_T c2_Interp_throtte_000;
  real_T c2_Interp_throtte_077;
  real_T c2_Interp_throtte_100;
  real_T c2_matrix_interp[3];
  real_T c2_throtte_value[3];
  real_T c2_X_thrust;
  real_T c2_Cx;
  real_T c2_Cy;
  real_T c2_Cz;
  real_T c2_Cl_delta_a;
  real_T c2_Cl_delta_r;
  real_T c2_Cl;
  real_T c2_Cm;
  real_T c2_Cn_delta_a;
  real_T c2_Cn_delta_r;
  real_T c2_Cn;
  real_T c2_nargin = 5.0;
  real_T c2_nargout = 6.0;
  real_T c2_b_X;
  real_T c2_b_Y;
  real_T c2_b_Z;
  real_T c2_b_L;
  real_T c2_b_M;
  real_T c2_b_N;
  real_T c2_d0;
  real_T c2_A;
  real_T c2_B;
  real_T c2_x;
  real_T c2_y;
  real_T c2_b_x;
  real_T c2_b_y;
  real_T c2_c_y;
  real_T c2_b_A;
  real_T c2_b_B;
  real_T c2_c_x;
  real_T c2_d_y;
  real_T c2_d_x;
  real_T c2_e_y;
  real_T c2_f_y;
  int32_T c2_i2;
  int32_T c2_i3;
  int32_T c2_i4;
  static real_T c2_dv0[12] = { 0.77, 0.241, -0.1, -0.416, -0.731, -1.053, -1.366,
    -1.646, -1.917, -2.12, -2.248, -2.229 };

  int32_T c2_i5;
  static real_T c2_dv1[60] = { -0.099, -0.081, -0.081, -0.063, -0.025, 0.044,
    0.097, 0.113, 0.145, 0.167, 0.174, 0.166, -0.048, -0.038, -0.04, -0.021,
    0.016, 0.083, 0.127, 0.137, 0.162, 0.177, 0.179, 0.167, -0.022, -0.02,
    -0.021, -0.004, 0.032, 0.094, 0.128, 0.13, 0.154, 0.161, 0.155, 0.138, -0.04,
    -0.038, -0.039, -0.025, 0.006, 0.062, 0.087, 0.085, 0.1, 0.11, 0.104, 0.091,
    -0.083, -0.073, -0.076, -0.072, -0.046, 0.012, 0.024, 0.025, 0.043, 0.053,
    0.047, 0.04 };

  int32_T c2_i6;
  static real_T c2_dv2[60] = { 0.205, 0.168, 0.186, 0.196, 0.213, 0.251, 0.245,
    0.238, 0.252, 0.231, 0.198, 0.192, 0.081, 0.077, 0.107, 0.11, 0.11, 0.141,
    0.127, 0.119, 0.133, 0.108, 0.081, 0.093, -0.046, -0.02, -0.009, -0.005,
    -0.006, 0.01, 0.006, -0.001, 0.014, 0.0, -0.013, 0.032, -0.174, -0.145,
    -0.121, -0.127, -0.129, -0.102, -0.097, -0.113, -0.087, -0.084, -0.069,
    -0.006, -0.259, -0.202, -0.184, -0.193, -0.199, -0.15, -0.16, -0.167, -0.104,
    -0.076, -0.041, -0.005 };

  int32_T c2_i7;
  static real_T c2_dv3[84] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.001, -0.004, -0.008, -0.012, -0.016, -0.019, -0.02, -0.02,
    -0.015, -0.008, -0.013, -0.015, -0.003, -0.009, -0.017, -0.024, -0.03,
    -0.034, -0.04, -0.037, -0.016, -0.002, -0.1, -0.19, -0.001, -0.01, -0.02,
    -0.03, -0.039, -0.044, -0.05, -0.049, -0.023, -0.006, -0.014, -0.027, 0.0,
    -0.01, -0.022, -0.034, -0.047, -0.046, -0.059, -0.061, -0.033, -0.036,
    -0.035, -0.035, 0.07, -0.01, -0.023, -0.034, -0.049, -0.046, -0.068, -0.071,
    -0.06, -0.058, -0.062, -0.059, 0.009, -0.011, -0.023, -0.037, -0.05, -0.047,
    -0.074, -0.079, -0.091, -0.076, -0.077, -0.076 };

  int32_T c2_i8;
  static real_T c2_dv4[84] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.018, 0.019, 0.018, 0.019, 0.019, 0.018, 0.013, 0.007, 0.004,
    -0.014, -0.017, -0.033, 0.038, 0.042, 0.042, 0.042, 0.043, 0.039, 0.03,
    0.017, 0.004, -0.035, -0.047, -0.057, 0.056, 0.057, 0.059, 0.058, 0.058,
    0.053, 0.032, 0.012, 0.002, -0.046, -0.071, -0.073, 0.064, 0.077, 0.076,
    0.074, 0.073, 0.057, 0.029, 0.007, 0.012, -0.034, -0.065, -0.041, 0.074,
    0.086, 0.093, 0.089, 0.08, 0.062, 0.049, 0.022, 0.028, -0.012, -0.002,
    -0.013, 0.079, 0.09, 0.106, 0.106, 0.096, 0.08, 0.068, 0.03, 0.064, 0.015,
    0.011, -0.001 };

  int32_T c2_i9;
  static real_T c2_dv5[84] = { -0.041, -0.052, -0.053, -0.056, -0.05, -0.056,
    -0.082, -0.059, -0.042, -0.038, -0.027, -0.017, -0.041, -0.053, -0.053,
    -0.053, -0.05, -0.051, -0.066, -0.043, -0.038, -0.027, -0.023, -0.016,
    -0.042, -0.053, -0.052, -0.051, -0.049, -0.049, -0.043, -0.035, -0.026,
    -0.016, -0.018, -0.014, -0.04, -0.052, -0.051, -0.052, -0.048, -0.048,
    -0.042, -0.037, -0.031, -0.026, -0.017, -0.012, -0.043, -0.049, -0.048,
    -0.049, -0.043, -0.042, -0.042, -0.036, -0.025, -0.021, -0.016, -0.011,
    -0.044, -0.048, -0.048, -0.047, -0.042, -0.041, -0.02, -0.028, -0.013,
    -0.014, -0.011, -0.01, -0.043, -0.049, -0.047, -0.045, -0.042, -0.037,
    -0.003, -0.013, -0.01, -0.003, -0.007, -0.008 };

  int32_T c2_i10;
  static real_T c2_dv6[84] = { 0.005, 0.017, 0.014, 0.01, -0.005, 0.009, 0.019,
    0.005, 0.0, -0.005, -0.011, 0.008, 0.007, 0.016, 0.014, 0.014, 0.013, 0.009,
    0.012, 0.005, 0.0, 0.004, 0.009, 0.007, 0.013, 0.013, 0.011, 0.012, 0.011,
    0.009, 0.008, 0.005, -0.002, 0.005, 0.003, 0.005, 0.018, 0.015, 0.015, 0.014,
    0.014, 0.014, 0.014, 0.015, 0.013, 0.011, 0.006, 0.001, 0.015, 0.014, 0.013,
    0.013, 0.012, 0.011, 0.011, 0.01, 0.008, 0.008, 0.007, 0.003, 0.021, 0.011,
    0.01, 0.011, 0.01, 0.009, 0.008, 0.01, 0.006, 0.005, 0.0, 0.001, 0.023, 0.01,
    0.011, 0.011, 0.011, 0.01, 0.008, 0.01, 0.006, 0.014, 0.02, 0.0 };

  int32_T c2_i11;
  static real_T c2_dv7[84] = { 0.001, -0.027, -0.017, -0.013, -0.012, -0.016,
    0.001, 0.017, 0.011, 0.017, 0.008, 0.016, 0.002, -0.014, -0.016, -0.016,
    -0.014, -0.019, -0.021, 0.002, 0.012, 0.015, 0.015, 0.011, -0.006, -0.008,
    -0.006, -0.006, -0.005, -0.008, -0.005, 0.007, 0.004, 0.007, 0.006, 0.006,
    -0.011, -0.011, -0.01, -0.009, -0.008, -0.006, 0.0, 0.004, 0.007, 0.1, 0.004,
    0.1, -0.015, -0.015, -0.014, -0.012, -0.011, -0.008, -0.002, 0.002, 0.006,
    0.012, 0.011, 0.011, -0.024, -0.01, -0.004, -0.002, -0.001, 0.003, 0.014,
    0.006, -0.001, 0.004, 0.004, 0.006, -0.022, 0.002, -0.003, -0.005, -0.003,
    -0.001, -0.009, -0.009, -0.001, 0.003, -0.002, 0.001 };

  int32_T c2_i12;
  static real_T c2_dv8[84] = { -0.018, -0.052, -0.052, -0.052, -0.054, -0.049,
    -0.059, -0.051, -0.03, -0.037, -0.026, -0.013, -0.028, -0.051, -0.043,
    -0.046, -0.045, -0.049, -0.057, -0.052, -0.03, -0.033, -0.03, -0.008, -0.037,
    -0.041, -0.038, -0.04, -0.04, -0.038, -0.037, -0.03, -0.027, -0.024, -0.019,
    -0.013, -0.048, -0.045, -0.045, -0.045, -0.044, -0.045, -0.047, -0.048,
    -0.049, -0.045, -0.033, -0.016, -0.043, -0.044, -0.041, -0.041, -0.04,
    -0.038, -0.034, -0.035, -0.035, -0.029, -0.022, -0.009, -0.052, -0.034,
    -0.036, -0.036, -0.035, -0.028, -0.024, -0.023, -0.02, -0.016, -0.01, -0.014,
    -0.062, -0.034, -0.027, -0.028, -0.027, -0.027, -0.023, -0.023, -0.019,
    -0.009, -0.025, -0.01 };

  int32_T c2_i13;
  static real_T c2_dv9[36] = { 1060.0, 635.0, 60.0, -1020.0, -2700.0, -3600.0,
    670.0, 425.0, 25.0, -710.0, -1900.0, -1400.0, 880.0, 690.0, 345.0, -300.0,
    -1300.0, -595.0, 1140.0, 1010.0, 755.0, 350.0, -247.0, -342.0, 1500.0,
    1330.0, 1130.0, 910.0, 600.0, -200.0, 1860.0, 1700.0, 1525.0, 1360.0, 1100.0,
    700.0 };

  int32_T c2_i14;
  static real_T c2_dv10[36] = { 12680.0, 12680.0, 12610.0, 12640.0, 12390.0,
    11680.0, 9150.0, 9150.0, 9312.0, 9839.0, 10176.0, 9848.0, 6200.0, 6313.0,
    6610.0, 7090.0, 7750.0, 8050.0, 3950.0, 4040.0, 4290.0, 4660.0, 5320.0,
    6100.0, 2450.0, 2470.0, 2600.0, 2840.0, 3250.0, 3800.0, 1400.0, 1400.0,
    1560.0, 1660.0, 1930.0, 2310.0 };

  int32_T c2_i15;
  static real_T c2_dv11[36] = { 20000.0, 21420.0, 22700.0, 24240.0, 26070.0,
    28886.0, 15000.0, 15700.0, 16860.0, 18910.0, 21075.0, 23319.0, 10800.0,
    11225.0, 12250.0, 13760.0, 15975.0, 18300.0, 7000.0, 7323.0, 8154.0, 9285.0,
    11115.0, 13484.0, 4000.0, 4435.0, 5000.0, 5700.0, 6860.0, 8642.0, 2500.0,
    2600.0, 2835.0, 3215.0, 3950.0, 5057.0 };

  int32_T c2_i16;
  static real_T c2_dv12[12] = { -0.267, -0.11, 0.308, 1.34, 2.08, 2.91, 2.76,
    2.05, 1.5, 1.49, 1.83, 1.21 };

  int32_T c2_i17;
  static real_T c2_dv13[12] = { 0.882, 0.852, 0.876, 0.958, 0.962, 0.974, 0.819,
    0.483, 0.59, 1.21, -0.493, -1.04 };

  int32_T c2_i18;
  static real_T c2_dv14[12] = { -0.108, -0.108, -0.188, 0.11, 0.258, 0.226,
    -0.344, 0.362, 0.611, 0.529, 0.298, -2.27 };

  int32_T c2_i19;
  static real_T c2_dv15[12] = { -8.8, -25.8, -28.9, -31.4, -31.2, -30.7, -27.7,
    -28.2, -29.0, -29.8, -38.3, -35.3 };

  int32_T c2_i20;
  static real_T c2_dv16[12] = { -0.126, -0.126, 0.063, 0.113, 0.208, 0.23, 0.319,
    0.437, 0.68, 0.1, 0.447, -0.33 };

  int32_T c2_i21;
  static real_T c2_dv17[12] = { -0.36, -0.359, -0.443, -0.42, -0.383, -0.375,
    -0.329, -0.294, -0.23, -0.21, -0.12, -0.1 };

  int32_T c2_i22;
  static real_T c2_dv18[12] = { -7.21, -0.54, -5.23, -5.26, -6.11, -6.64, -5.69,
    -6.0, -6.2, -6.4, -6.6, -6.0 };

  int32_T c2_i23;
  static real_T c2_dv19[12] = { -0.38, -0.363, -0.378, -0.386, -0.37, -0.453,
    -0.55, -0.582, -0.595, -0.637, -1.02, -0.804 };

  int32_T c2_i24;
  static real_T c2_dv20[12] = { 0.061, 0.052, 0.052, -0.012, -0.013, -0.024,
    0.05, 0.15, 0.13, 0.158, 0.24, 0.15 };

  int32_T c2_i25;
  int32_T c2_i26;
  int32_T c2_i27;
  int32_T c2_i28;
  static real_T c2_dv21[6] = { 0.0, 0.2, 0.4, 0.6, 0.8, 1.0 };

  real_T c2_c_A;
  real_T c2_c_B;
  real_T c2_e_x;
  real_T c2_g_y;
  real_T c2_f_x;
  real_T c2_h_y;
  real_T c2_d_A;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_i_y;
  real_T c2_e_A;
  real_T c2_i_x;
  real_T c2_j_x;
  real_T c2_j_y;
  real_T c2_f_A;
  real_T c2_d_B;
  real_T c2_k_x;
  real_T c2_k_y;
  real_T c2_l_x;
  real_T c2_l_y;
  real_T c2_g_A;
  real_T c2_m_x;
  real_T c2_n_x;
  real_T c2_m_y;
  real_T c2_h_A;
  real_T c2_o_x;
  real_T c2_p_x;
  real_T c2_n_y;
  real_T c2_i_A;
  real_T c2_q_x;
  real_T c2_r_x;
  real_T c2_o_y;
  int32_T c2_i29;
  static real_T c2_dv22[3] = { 0.0, 0.77, 1.0 };

  int32_T c2_i30;
  int32_T c2_i31;
  real_T c2_dv23[3];
  real_T c2_b_matrix_interp[3];
  real_T c2_j_A;
  real_T c2_e_B;
  real_T c2_s_x;
  real_T c2_p_y;
  real_T c2_t_x;
  real_T c2_q_y;
  real_T c2_r_y;
  real_T c2_k_A;
  real_T c2_u_x;
  real_T c2_v_x;
  real_T c2_s_y;
  real_T c2_l_A;
  real_T c2_w_x;
  real_T c2_x_x;
  real_T c2_t_y;
  real_T c2_f_B;
  real_T c2_u_y;
  real_T c2_v_y;
  real_T c2_w_y;
  real_T c2_m_A;
  real_T c2_y_x;
  real_T c2_ab_x;
  real_T c2_x_y;
  real_T c2_n_A;
  real_T c2_g_B;
  real_T c2_bb_x;
  real_T c2_y_y;
  real_T c2_cb_x;
  real_T c2_ab_y;
  real_T c2_bb_y;
  real_T c2_h_B;
  real_T c2_cb_y;
  real_T c2_db_y;
  real_T c2_eb_y;
  real_T c2_d1;
  real_T c2_o_A;
  real_T c2_i_B;
  real_T c2_db_x;
  real_T c2_fb_y;
  real_T c2_eb_x;
  real_T c2_gb_y;
  real_T c2_hb_y;
  real_T c2_j_B;
  real_T c2_ib_y;
  real_T c2_jb_y;
  real_T c2_kb_y;
  real_T c2_d2;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  c2_hoistedGlobal = *chartInstance->c2_delta_e;
  c2_b_hoistedGlobal = *chartInstance->c2_delta_r;
  c2_c_hoistedGlobal = *chartInstance->c2_delta_a;
  c2_d_hoistedGlobal = *chartInstance->c2_delta_T;
  c2_b_delta_e = c2_hoistedGlobal;
  c2_b_delta_r = c2_b_hoistedGlobal;
  c2_b_delta_a = c2_c_hoistedGlobal;
  c2_b_delta_T = c2_d_hoistedGlobal;
  for (c2_i1 = 0; c2_i1 < 12; c2_i1++) {
    c2_b_state[c2_i1] = (*chartInstance->c2_state)[c2_i1];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 101U, 101U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_you, 0U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_v, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_w, 2U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_p, 3U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_q, 4U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_r, 5U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_fai, 6U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_theta, 7U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_psi, 8U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_ze, 9U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_H, 10U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_V, 11U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_R, 12U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c_, 13U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_S, 14U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b, 15U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_r2d, 16U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Vv, 17U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_alpha, 18U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_beta, 19U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_r2dalpha, 20U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_r2dbeta, 21U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Ix, 22U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Iy, 23U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Iz, 24U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_Ixz, 25U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Isum, 26U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_Alpha, 27U, c2_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_Elevator, 28U, c2_k_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_Cz, 29U, c2_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_Cx, 30U, c2_j_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_Cm, 31U, c2_j_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_Cl, 32U, c2_i_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_Cn, 33U, c2_i_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_Cl_delta_a, 34U, c2_i_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_Cl_delta_r, 35U, c2_i_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_Cn_delta_a, 36U, c2_i_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_Cn_delta_r, 37U, c2_i_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_throtte_000, 38U, c2_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_throtte_077, 39U, c2_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_throtte_100, 40U, c2_h_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_Cxq, 41U, c2_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_Cyr, 42U, c2_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_Cyp, 43U, c2_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_Czq, 44U, c2_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_Clr, 45U, c2_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_Clp, 46U, c2_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_Cmq, 47U, c2_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_Cnr, 48U, c2_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_Cnp, 49U, c2_g_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_delta_beta, 50U, c2_f_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_beta, 51U, c2_f_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_height, 52U, c2_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_given_mah, 53U, c2_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_rou0, 54U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_h1, 55U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_h2, 56U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_h_p, 57U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_WW, 58U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_T, 59U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_rou, 60U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_a, 61U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Mah, 62U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Cx_q, 63U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Cy_r, 64U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Cy_p, 65U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Cz_q, 66U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Cl_r, 67U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Cl_p, 68U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Cm_q, 69U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Cn_r, 70U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Cn_p, 71U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Interp_throtte_000, 72U,
    c2_sf_marshallOut, c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Interp_throtte_077, 73U,
    c2_sf_marshallOut, c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Interp_throtte_100, 74U,
    c2_sf_marshallOut, c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_matrix_interp, 75U,
    c2_c_sf_marshallOut, c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_throtte_value, 76U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_X_thrust, 77U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Cx, 78U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Cy, 79U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Cz, 80U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Cl_delta_a, 81U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Cl_delta_r, 82U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Cl, 83U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Cm, 84U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Cn_delta_a, 85U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Cn_delta_r, 86U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Cn, 87U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 88U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 89U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_delta_e, 90U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_delta_r, 91U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_delta_a, 92U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_delta_T, 93U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_b_state, 94U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_X, 95U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_Y, 96U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_Z, 97U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_L, 98U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_M, 99U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_N, 100U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 3);
  c2_you = c2_b_state[0];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 4);
  c2_v = c2_b_state[1];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 5);
  c2_w = c2_b_state[2];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 6);
  c2_p = c2_b_state[3];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 7);
  c2_q = c2_b_state[4];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 8);
  c2_r = c2_b_state[5];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 9);
  c2_fai = c2_b_state[6];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 10);
  c2_theta = c2_b_state[7];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
  c2_psi = c2_b_state[8];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 12);
  c2_ze = c2_b_state[11];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 14);
  c2_H = 1000.0 - c2_ze;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 15);
  c2_d0 = (c2_mpower(chartInstance, c2_you) + c2_mpower(chartInstance, c2_v)) +
    c2_mpower(chartInstance, c2_w);
  c2_b_sqrt(chartInstance, &c2_d0);
  c2_V = c2_d0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 18);
  c2_R = 6.356766E+6;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 21);
  c2_c_ = 3.4503;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 21);
  c2_S = 27.8709;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 21);
  c2_b = 9.144;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 21);
  c2_r2d = 57.29577951;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 23);
  c2_d0 = (c2_mpower(chartInstance, c2_you) + c2_mpower(chartInstance, c2_v)) +
    c2_mpower(chartInstance, c2_w);
  c2_b_sqrt(chartInstance, &c2_d0);
  c2_Vv = c2_d0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 25);
  c2_A = c2_w;
  c2_B = c2_you;
  c2_x = c2_A;
  c2_y = c2_B;
  c2_b_x = c2_x;
  c2_b_y = c2_y;
  c2_c_y = c2_b_x / c2_b_y;
  c2_d0 = c2_c_y;
  c2_b_atan(chartInstance, &c2_d0);
  c2_alpha = c2_d0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 26);
  c2_b_A = c2_v;
  c2_b_B = c2_Vv;
  c2_c_x = c2_b_A;
  c2_d_y = c2_b_B;
  c2_d_x = c2_c_x;
  c2_e_y = c2_d_y;
  c2_f_y = c2_d_x / c2_e_y;
  c2_d0 = c2_f_y;
  c2_b_asin(chartInstance, &c2_d0);
  c2_beta = c2_d0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 28);
  c2_r2dalpha = c2_r2d * c2_alpha;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 29);
  c2_r2dbeta = c2_r2d * c2_beta;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 32);
  c2_Ix = 12874.8446;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 32);
  c2_Iy = 75673.6077;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 32);
  c2_Iz = 85552.0953;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 32);
  c2_Ixz = 1331.413;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 32);
  c2_Isum = c2_Ix * c2_Iz - c2_mpower(chartInstance, 1331.413);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 36);
  for (c2_i2 = 0; c2_i2 < 12; c2_i2++) {
    c2_given_Alpha[c2_i2] = -10.0 + 5.0 * (real_T)c2_i2;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 37);
  for (c2_i3 = 0; c2_i3 < 5; c2_i3++) {
    c2_given_Elevator[c2_i3] = -24.0 + 12.0 * (real_T)c2_i3;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 38);
  for (c2_i4 = 0; c2_i4 < 12; c2_i4++) {
    c2_given_Cz[c2_i4] = c2_dv0[c2_i4];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 39);
  for (c2_i5 = 0; c2_i5 < 60; c2_i5++) {
    c2_given_Cx[c2_i5] = c2_dv1[c2_i5];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 51);
  for (c2_i6 = 0; c2_i6 < 60; c2_i6++) {
    c2_given_Cm[c2_i6] = c2_dv2[c2_i6];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 63);
  for (c2_i7 = 0; c2_i7 < 84; c2_i7++) {
    c2_given_Cl[c2_i7] = c2_dv3[c2_i7];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 75);
  for (c2_i8 = 0; c2_i8 < 84; c2_i8++) {
    c2_given_Cn[c2_i8] = c2_dv4[c2_i8];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 87);
  for (c2_i9 = 0; c2_i9 < 84; c2_i9++) {
    c2_given_Cl_delta_a[c2_i9] = c2_dv5[c2_i9];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 99);
  for (c2_i10 = 0; c2_i10 < 84; c2_i10++) {
    c2_given_Cl_delta_r[c2_i10] = c2_dv6[c2_i10];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 111);
  for (c2_i11 = 0; c2_i11 < 84; c2_i11++) {
    c2_given_Cn_delta_a[c2_i11] = c2_dv7[c2_i11];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 123);
  for (c2_i12 = 0; c2_i12 < 84; c2_i12++) {
    c2_given_Cn_delta_r[c2_i12] = c2_dv8[c2_i12];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 135U);
  for (c2_i13 = 0; c2_i13 < 36; c2_i13++) {
    c2_given_throtte_000[c2_i13] = c2_dv9[c2_i13];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 141U);
  for (c2_i14 = 0; c2_i14 < 36; c2_i14++) {
    c2_given_throtte_077[c2_i14] = c2_dv10[c2_i14];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 147U);
  for (c2_i15 = 0; c2_i15 < 36; c2_i15++) {
    c2_given_throtte_100[c2_i15] = c2_dv11[c2_i15];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 154U);
  for (c2_i16 = 0; c2_i16 < 12; c2_i16++) {
    c2_given_Cxq[c2_i16] = c2_dv12[c2_i16];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 155U);
  for (c2_i17 = 0; c2_i17 < 12; c2_i17++) {
    c2_given_Cyr[c2_i17] = c2_dv13[c2_i17];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 156U);
  for (c2_i18 = 0; c2_i18 < 12; c2_i18++) {
    c2_given_Cyp[c2_i18] = c2_dv14[c2_i18];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 157U);
  for (c2_i19 = 0; c2_i19 < 12; c2_i19++) {
    c2_given_Czq[c2_i19] = c2_dv15[c2_i19];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 158U);
  for (c2_i20 = 0; c2_i20 < 12; c2_i20++) {
    c2_given_Clr[c2_i20] = c2_dv16[c2_i20];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 159U);
  for (c2_i21 = 0; c2_i21 < 12; c2_i21++) {
    c2_given_Clp[c2_i21] = c2_dv17[c2_i21];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 160U);
  for (c2_i22 = 0; c2_i22 < 12; c2_i22++) {
    c2_given_Cmq[c2_i22] = c2_dv18[c2_i22];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 161U);
  for (c2_i23 = 0; c2_i23 < 12; c2_i23++) {
    c2_given_Cnr[c2_i23] = c2_dv19[c2_i23];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 162U);
  for (c2_i24 = 0; c2_i24 < 12; c2_i24++) {
    c2_given_Cnp[c2_i24] = c2_dv20[c2_i24];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 164U);
  for (c2_i25 = 0; c2_i25 < 7; c2_i25++) {
    c2_given_delta_beta[c2_i25] = -30.0 + 10.0 * (real_T)c2_i25;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 165U);
  for (c2_i26 = 0; c2_i26 < 7; c2_i26++) {
    c2_given_beta[c2_i26] = 5.0 * (real_T)c2_i26;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 168U);
  for (c2_i27 = 0; c2_i27 < 6; c2_i27++) {
    c2_given_height[c2_i27] = 10000.0 * (real_T)c2_i27;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 169U);
  for (c2_i28 = 0; c2_i28 < 6; c2_i28++) {
    c2_given_mah[c2_i28] = c2_dv21[c2_i28];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 173U);
  c2_rou0 = 1.225;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 176U);
  c2_h1 = 11019.1;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 177U);
  c2_h2 = 20063.1;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 181U);
  c2_c_A = c2_R * c2_H;
  c2_c_B = (c2_R + c2_H) * 1000.0;
  c2_e_x = c2_c_A;
  c2_g_y = c2_c_B;
  c2_f_x = c2_e_x;
  c2_h_y = c2_g_y;
  c2_h_p = c2_f_x / c2_h_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 182U);
  guard1 = false;
  if (CV_EML_COND(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 0, c2_H, 0.0, -1, 5U, c2_H
        >= 0.0))) {
    if (CV_EML_COND(0, 1, 1, CV_RELATIONAL_EVAL(4U, 0U, 1, c2_H, c2_h1, -1, 3U,
          c2_H <= c2_h1))) {
      CV_EML_MCDC(0, 1, 0, true);
      CV_EML_IF(0, 1, 0, true);
      _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 183U);
      c2_d_A = c2_h_p;
      c2_g_x = c2_d_A;
      c2_h_x = c2_g_x;
      c2_i_y = c2_h_x / 44.3308;
      c2_WW = 1.0 - c2_i_y;
      _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 184U);
      c2_T = 288.15 * c2_WW;
      _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 185U);
      c2_rou = c2_rou0 * c2_b_mpower(chartInstance, c2_WW);
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1 == true) {
    CV_EML_MCDC(0, 1, 0, false);
    CV_EML_IF(0, 1, 0, false);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 186U);
    guard2 = false;
    if (CV_EML_COND(0, 1, 2, CV_RELATIONAL_EVAL(4U, 0U, 2, c2_H, c2_h1, -1, 5U,
          c2_H >= c2_h1))) {
      if (CV_EML_COND(0, 1, 3, CV_RELATIONAL_EVAL(4U, 0U, 3, c2_H, c2_h2, -1, 3U,
            c2_H <= c2_h2))) {
        CV_EML_MCDC(0, 1, 1, true);
        CV_EML_IF(0, 1, 1, true);
        _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 187U);
        c2_e_A = 14.9647 - c2_h_p;
        c2_i_x = c2_e_A;
        c2_j_x = c2_i_x;
        c2_j_y = c2_j_x / 6.3416;
        c2_d0 = c2_j_y;
        c2_b_exp(chartInstance, &c2_d0);
        c2_WW = c2_d0;
        _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 188U);
        c2_T = 216.65;
        _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 189U);
        c2_rou = 1.5898 * c2_c_mpower(chartInstance, 10.0) * c2_rou0 * c2_WW;
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }

    if (guard2 == true) {
      CV_EML_MCDC(0, 1, 1, false);
      CV_EML_IF(0, 1, 1, false);
      _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 191U);
      c2_WW = 0.0;
      _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 192U);
      c2_T = 0.0 * c2_WW;
      _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 193U);
      c2_rou = 0.0;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 197U);
  c2_a = 20.0468 * c2_d_mpower(chartInstance, c2_T);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 198U);
  c2_f_A = c2_V;
  c2_d_B = c2_a;
  c2_k_x = c2_f_A;
  c2_k_y = c2_d_B;
  c2_l_x = c2_k_x;
  c2_l_y = c2_k_y;
  c2_Mah = c2_l_x / c2_l_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 201U);
  c2_Cx_q = c2_interp1(chartInstance, c2_r2dalpha);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 202U);
  c2_Cy_r = c2_b_interp1(chartInstance, c2_r2dalpha);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 203U);
  c2_Cy_p = c2_c_interp1(chartInstance, c2_r2dalpha);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 204U);
  c2_Cz_q = c2_d_interp1(chartInstance, c2_r2dalpha);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 205U);
  c2_Cl_r = c2_e_interp1(chartInstance, c2_r2dalpha);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 206U);
  c2_Cl_p = c2_f_interp1(chartInstance, c2_r2dalpha);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 207U);
  c2_Cm_q = c2_g_interp1(chartInstance, c2_r2dalpha);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 208U);
  c2_Cn_r = c2_h_interp1(chartInstance, c2_r2dalpha);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 209U);
  c2_Cn_p = c2_i_interp1(chartInstance, c2_r2dalpha);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 211U);
  c2_g_A = c2_H;
  c2_m_x = c2_g_A;
  c2_n_x = c2_m_x;
  c2_m_y = c2_n_x / 0.3048;
  c2_Interp_throtte_000 = c2_interp2(chartInstance, c2_m_y, c2_Mah) * 4.44822072;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 212U);
  c2_h_A = c2_H;
  c2_o_x = c2_h_A;
  c2_p_x = c2_o_x;
  c2_n_y = c2_p_x / 0.3048;
  c2_Interp_throtte_077 = c2_b_interp2(chartInstance, c2_n_y, c2_Mah) *
    4.44822072;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 213U);
  c2_i_A = c2_H;
  c2_q_x = c2_i_A;
  c2_r_x = c2_q_x;
  c2_o_y = c2_r_x / 0.3048;
  c2_Interp_throtte_100 = c2_c_interp2(chartInstance, c2_o_y, c2_Mah) *
    4.44822072;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 214U);
  c2_matrix_interp[0] = c2_Interp_throtte_000;
  c2_matrix_interp[1] = c2_Interp_throtte_077;
  c2_matrix_interp[2] = c2_Interp_throtte_100;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 215U);
  for (c2_i29 = 0; c2_i29 < 3; c2_i29++) {
    c2_throtte_value[c2_i29] = c2_dv22[c2_i29];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 216U);
  for (c2_i30 = 0; c2_i30 < 3; c2_i30++) {
    c2_dv23[c2_i30] = c2_dv22[c2_i30];
  }

  for (c2_i31 = 0; c2_i31 < 3; c2_i31++) {
    c2_b_matrix_interp[c2_i31] = c2_matrix_interp[c2_i31];
  }

  c2_X_thrust = c2_j_interp1(chartInstance, c2_dv23, c2_b_matrix_interp,
    c2_b_delta_T);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 218U);
  c2_j_A = c2_Cx_q * c2_q * c2_c_;
  c2_e_B = 2.0 * c2_V;
  c2_s_x = c2_j_A;
  c2_p_y = c2_e_B;
  c2_t_x = c2_s_x;
  c2_q_y = c2_p_y;
  c2_r_y = c2_t_x / c2_q_y;
  c2_Cx = c2_d_interp2(chartInstance, c2_b_delta_e, c2_r2dalpha) + c2_r_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 219U);
  c2_k_A = c2_b_delta_a;
  c2_u_x = c2_k_A;
  c2_v_x = c2_u_x;
  c2_s_y = c2_v_x / 20.0;
  c2_l_A = c2_b_delta_r;
  c2_w_x = c2_l_A;
  c2_x_x = c2_w_x;
  c2_t_y = c2_x_x / 30.0;
  c2_f_B = 2.0 * c2_V;
  c2_u_y = c2_f_B;
  c2_v_y = c2_u_y;
  c2_w_y = 9.144 / c2_v_y;
  c2_Cy = ((-0.02 * c2_r2dbeta + 0.021 * c2_s_y) + 0.086 * c2_t_y) + c2_w_y *
    (c2_Cy_p * c2_p + c2_Cy_r * c2_r);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 220U);
  c2_m_A = c2_b_delta_e;
  c2_y_x = c2_m_A;
  c2_ab_x = c2_y_x;
  c2_x_y = c2_ab_x / 25.0;
  c2_n_A = c2_Cz_q * c2_q * c2_c_;
  c2_g_B = 2.0 * c2_V;
  c2_bb_x = c2_n_A;
  c2_y_y = c2_g_B;
  c2_cb_x = c2_bb_x;
  c2_ab_y = c2_y_y;
  c2_bb_y = c2_cb_x / c2_ab_y;
  c2_Cz = (c2_k_interp1(chartInstance, c2_r2dalpha) * (1.0 - c2_mpower
            (chartInstance, c2_beta)) - 0.19 * c2_x_y) + c2_bb_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 222U);
  c2_Cl_delta_a = c2_e_interp2(chartInstance, c2_r2dbeta, c2_r2dalpha);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 223U);
  c2_Cl_delta_r = c2_f_interp2(chartInstance, c2_r2dbeta, c2_r2dalpha);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 224U);
  c2_h_B = 2.0 * c2_V;
  c2_cb_y = c2_h_B;
  c2_db_y = c2_cb_y;
  c2_eb_y = 9.144 / c2_db_y;
  c2_d1 = c2_r2dbeta;
  c2_b_sign(chartInstance, &c2_d1);
  c2_Cl = ((c2_d1 * c2_g_interp2(chartInstance, c2_abs(chartInstance, c2_r2dbeta),
             c2_r2dalpha) + c2_Cl_delta_a * c2_b_delta_a) + c2_Cl_delta_r *
           c2_b_delta_r) + c2_eb_y * (c2_Cl_p * c2_p + c2_Cl_r * c2_r);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 227U);
  c2_o_A = c2_Cm_q * c2_q * c2_c_;
  c2_i_B = 2.0 * c2_V;
  c2_db_x = c2_o_A;
  c2_fb_y = c2_i_B;
  c2_eb_x = c2_db_x;
  c2_gb_y = c2_fb_y;
  c2_hb_y = c2_eb_x / c2_gb_y;
  c2_Cm = c2_h_interp2(chartInstance, c2_b_delta_e, c2_r2dalpha) + c2_hb_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 229U);
  c2_Cn_delta_a = c2_i_interp2(chartInstance, c2_r2dbeta, c2_r2dalpha);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 230U);
  c2_Cn_delta_r = c2_j_interp2(chartInstance, c2_r2dbeta, c2_r2dalpha);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 231U);
  c2_j_B = 2.0 * c2_V;
  c2_ib_y = c2_j_B;
  c2_jb_y = c2_ib_y;
  c2_kb_y = 9.144 / c2_jb_y;
  c2_d2 = c2_r2dbeta;
  c2_b_sign(chartInstance, &c2_d2);
  c2_Cn = ((c2_d2 * c2_k_interp2(chartInstance, c2_abs(chartInstance, c2_r2dbeta),
             c2_r2dalpha) + c2_Cn_delta_a * c2_b_delta_a) + c2_Cn_delta_r *
           c2_b_delta_r) + c2_kb_y * (c2_Cn_r * c2_r + c2_Cn_p * c2_p);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 236U);
  c2_b_X = c2_X_thrust + 0.5 * c2_rou * c2_mpower(chartInstance, c2_V) * c2_S *
    c2_Cx;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 237U);
  c2_b_Y = 0.5 * c2_rou * c2_mpower(chartInstance, c2_V) * c2_S * c2_Cy;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 238U);
  c2_b_Z = 0.5 * c2_rou * c2_mpower(chartInstance, c2_V) * c2_S * c2_Cz;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 239U);
  c2_b_L = 0.5 * c2_rou * c2_mpower(chartInstance, c2_V) * c2_S * c2_b * c2_Cl;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 240U);
  c2_b_M = 0.5 * c2_rou * c2_mpower(chartInstance, c2_V) * c2_S * c2_c_ * c2_Cm;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 241U);
  c2_b_N = 0.5 * c2_rou * c2_mpower(chartInstance, c2_V) * c2_S * c2_b * c2_Cn;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -241);
  _SFD_SYMBOL_SCOPE_POP();
  *chartInstance->c2_X = c2_b_X;
  *chartInstance->c2_Y = c2_b_Y;
  *chartInstance->c2_Z = c2_b_Z;
  *chartInstance->c2_L = c2_b_L;
  *chartInstance->c2_M = c2_b_M;
  *chartInstance->c2_N = c2_b_N;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_Quan(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber)
{
  (void)c2_machineNumber;
  (void)c2_chartNumber;
  (void)c2_instanceNumber;
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuanInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static real_T c2_emlrt_marshallIn(SFc2_QuanInstanceStruct *chartInstance, const
  mxArray *c2_b_N, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_N), &c2_thisId);
  sf_mex_destroy(&c2_b_N);
  return c2_y;
}

static real_T c2_b_emlrt_marshallIn(SFc2_QuanInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d3;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d3, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d3;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_N;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuanInstanceStruct *)chartInstanceVoid;
  c2_b_N = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_N), &c2_thisId);
  sf_mex_destroy(&c2_b_N);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i32;
  const mxArray *c2_y = NULL;
  real_T c2_u[12];
  SFc2_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuanInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i32 = 0; c2_i32 < 12; c2_i32++) {
    c2_u[c2_i32] = (*(real_T (*)[12])c2_inData)[c2_i32];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 12), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i33;
  const mxArray *c2_y = NULL;
  real_T c2_u[3];
  SFc2_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuanInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i33 = 0; c2_i33 < 3; c2_i33++) {
    c2_u[c2_i33] = (*(real_T (*)[3])c2_inData)[c2_i33];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 3), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_c_emlrt_marshallIn(SFc2_QuanInstanceStruct *chartInstance, const
  mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[3])
{
  real_T c2_dv24[3];
  int32_T c2_i34;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv24, 1, 0, 0U, 1, 0U, 2, 1, 3);
  for (c2_i34 = 0; c2_i34 < 3; c2_i34++) {
    c2_y[c2_i34] = c2_dv24[c2_i34];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_matrix_interp;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[3];
  int32_T c2_i35;
  SFc2_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuanInstanceStruct *)chartInstanceVoid;
  c2_matrix_interp = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_matrix_interp), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_matrix_interp);
  for (c2_i35 = 0; c2_i35 < 3; c2_i35++) {
    (*(real_T (*)[3])c2_outData)[c2_i35] = c2_y[c2_i35];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i36;
  const mxArray *c2_y = NULL;
  real_T c2_u[6];
  SFc2_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuanInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i36 = 0; c2_i36 < 6; c2_i36++) {
    c2_u[c2_i36] = (*(real_T (*)[6])c2_inData)[c2_i36];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 6), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i37;
  const mxArray *c2_y = NULL;
  real_T c2_u[6];
  SFc2_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuanInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i37 = 0; c2_i37 < 6; c2_i37++) {
    c2_u[c2_i37] = (*(real_T (*)[6])c2_inData)[c2_i37];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 6), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i38;
  const mxArray *c2_y = NULL;
  real_T c2_u[7];
  SFc2_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuanInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i38 = 0; c2_i38 < 7; c2_i38++) {
    c2_u[c2_i38] = (*(real_T (*)[7])c2_inData)[c2_i38];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 7), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i39;
  const mxArray *c2_y = NULL;
  real_T c2_u[12];
  SFc2_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuanInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i39 = 0; c2_i39 < 12; c2_i39++) {
    c2_u[c2_i39] = (*(real_T (*)[12])c2_inData)[c2_i39];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 12), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i40;
  int32_T c2_i41;
  const mxArray *c2_y = NULL;
  int32_T c2_i42;
  real_T c2_u[36];
  SFc2_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuanInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i40 = 0;
  for (c2_i41 = 0; c2_i41 < 6; c2_i41++) {
    for (c2_i42 = 0; c2_i42 < 6; c2_i42++) {
      c2_u[c2_i42 + c2_i40] = (*(real_T (*)[36])c2_inData)[c2_i42 + c2_i40];
    }

    c2_i40 += 6;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 6, 6), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i43;
  int32_T c2_i44;
  const mxArray *c2_y = NULL;
  int32_T c2_i45;
  real_T c2_u[84];
  SFc2_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuanInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i43 = 0;
  for (c2_i44 = 0; c2_i44 < 7; c2_i44++) {
    for (c2_i45 = 0; c2_i45 < 12; c2_i45++) {
      c2_u[c2_i45 + c2_i43] = (*(real_T (*)[84])c2_inData)[c2_i45 + c2_i43];
    }

    c2_i43 += 12;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 12, 7), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i46;
  int32_T c2_i47;
  const mxArray *c2_y = NULL;
  int32_T c2_i48;
  real_T c2_u[60];
  SFc2_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuanInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i46 = 0;
  for (c2_i47 = 0; c2_i47 < 5; c2_i47++) {
    for (c2_i48 = 0; c2_i48 < 12; c2_i48++) {
      c2_u[c2_i48 + c2_i46] = (*(real_T (*)[60])c2_inData)[c2_i48 + c2_i46];
    }

    c2_i46 += 12;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 12, 5), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_k_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i49;
  const mxArray *c2_y = NULL;
  real_T c2_u[5];
  SFc2_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuanInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i49 = 0; c2_i49 < 5; c2_i49++) {
    c2_u[c2_i49] = (*(real_T (*)[5])c2_inData)[c2_i49];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 5), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

const mxArray *sf_c2_Quan_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  sf_mex_assign(&c2_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c2_nameCaptureInfo;
}

static real_T c2_mpower(SFc2_QuanInstanceStruct *chartInstance, real_T c2_a)
{
  real_T c2_c;
  real_T c2_b_a;
  real_T c2_c_a;
  real_T c2_x;
  real_T c2_d_a;
  c2_b_a = c2_a;
  c2_c_a = c2_b_a;
  c2_x = c2_c_a;
  c2_d_a = c2_x;
  c2_c = c2_d_a * c2_d_a;
  if (c2_fltpower_domain_error(chartInstance, c2_c_a, 2.0)) {
    c2_error(chartInstance);
  }

  return c2_c;
}

static void c2_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_dimagree(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static boolean_T c2_fltpower_domain_error(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_a, real_T c2_b)
{
  boolean_T c2_p;
  real_T c2_x;
  boolean_T c2_b_b;
  boolean_T c2_b0;
  real_T c2_b_x;
  real_T c2_c_x;
  boolean_T c2_b1;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  (void)chartInstance;
  c2_p = false;
  if (c2_a < 0.0) {
    guard1 = false;
    if (c2_p) {
      guard1 = true;
    } else {
      c2_x = c2_b;
      c2_b_b = muDoubleScalarIsNaN(c2_x);
      guard2 = false;
      if (c2_b_b) {
        guard2 = true;
      } else {
        c2_b_x = c2_b;
        c2_c_x = c2_b_x;
        c2_c_x = muDoubleScalarFloor(c2_c_x);
        if (c2_c_x == c2_b) {
          guard2 = true;
        } else {
          c2_b1 = false;
        }
      }

      if (guard2 == true) {
        c2_b1 = true;
      }

      if (!c2_b1) {
        guard1 = true;
      } else {
        c2_b0 = false;
      }
    }

    if (guard1 == true) {
      c2_b0 = true;
    }

    c2_p = c2_b0;
  }

  return c2_p;
}

static void c2_error(SFc2_QuanInstanceStruct *chartInstance)
{
  const mxArray *c2_y = NULL;
  static char_T c2_u[31] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'p', 'o', 'w', 'e', 'r', '_', 'd', 'o', 'm', 'a', 'i',
    'n', 'E', 'r', 'r', 'o', 'r' };

  (void)chartInstance;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 31), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c2_y));
}

static real_T c2_sqrt(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x)
{
  real_T c2_b_x;
  c2_b_x = c2_x;
  c2_b_sqrt(chartInstance, &c2_b_x);
  return c2_b_x;
}

static void c2_b_error(SFc2_QuanInstanceStruct *chartInstance)
{
  const mxArray *c2_y = NULL;
  static char_T c2_u[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  const mxArray *c2_b_y = NULL;
  static char_T c2_b_u[4] = { 's', 'q', 'r', 't' };

  (void)chartInstance;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c2_y, 14, c2_b_y));
}

static real_T c2_atan(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x)
{
  real_T c2_b_x;
  c2_b_x = c2_x;
  c2_b_atan(chartInstance, &c2_b_x);
  return c2_b_x;
}

static real_T c2_asin(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x)
{
  real_T c2_b_x;
  c2_b_x = c2_x;
  c2_b_asin(chartInstance, &c2_b_x);
  return c2_b_x;
}

static void c2_c_error(SFc2_QuanInstanceStruct *chartInstance)
{
  const mxArray *c2_y = NULL;
  static char_T c2_u[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  const mxArray *c2_b_y = NULL;
  static char_T c2_b_u[4] = { 'a', 's', 'i', 'n' };

  (void)chartInstance;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c2_y, 14, c2_b_y));
}

static real_T c2_b_mpower(SFc2_QuanInstanceStruct *chartInstance, real_T c2_a)
{
  real_T c2_c;
  real_T c2_b_a;
  real_T c2_c_a;
  real_T c2_x;
  real_T c2_d_a;
  real_T c2_ar;
  c2_b_a = c2_a;
  c2_c_a = c2_b_a;
  c2_x = c2_c_a;
  c2_d_a = c2_x;
  c2_ar = c2_d_a;
  c2_c = muDoubleScalarPower(c2_ar, 4.2559);
  if (c2_fltpower_domain_error(chartInstance, c2_c_a, 4.2559)) {
    c2_error(chartInstance);
  }

  return c2_c;
}

static real_T c2_exp(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x)
{
  real_T c2_b_x;
  c2_b_x = c2_x;
  c2_b_exp(chartInstance, &c2_b_x);
  return c2_b_x;
}

static real_T c2_c_mpower(SFc2_QuanInstanceStruct *chartInstance, real_T c2_a)
{
  real_T c2_c;
  real_T c2_b_a;
  real_T c2_c_a;
  real_T c2_x;
  real_T c2_d_a;
  real_T c2_y;
  c2_b_a = c2_a;
  c2_c_a = c2_b_a;
  c2_x = c2_c_a;
  c2_d_a = c2_x;
  c2_y = c2_d_a;
  c2_c = 1.0 / c2_y;
  if (c2_fltpower_domain_error(chartInstance, c2_c_a, -1.0)) {
    c2_error(chartInstance);
  }

  return c2_c;
}

static real_T c2_d_mpower(SFc2_QuanInstanceStruct *chartInstance, real_T c2_a)
{
  real_T c2_c;
  real_T c2_b_a;
  real_T c2_c_a;
  real_T c2_x;
  real_T c2_d_a;
  c2_b_a = c2_a;
  c2_c_a = c2_b_a;
  c2_x = c2_c_a;
  c2_d_a = c2_x;
  c2_c = c2_d_a;
  c2_b_sqrt(chartInstance, &c2_c);
  if (c2_fltpower_domain_error(chartInstance, c2_c_a, 0.5)) {
    c2_error(chartInstance);
  }

  return c2_c;
}

static void c2_b_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_StringToMethodID(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_d_error(SFc2_QuanInstanceStruct *chartInstance)
{
  const mxArray *c2_y = NULL;
  static char_T c2_u[21] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'i', 'n', 't',
    'e', 'r', 'p', '1', ':', 'N', 'a', 'N', 'i', 'n', 'X' };

  (void)chartInstance;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 21), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c2_y));
}

static void c2_e_error(SFc2_QuanInstanceStruct *chartInstance)
{
  const mxArray *c2_y = NULL;
  static char_T c2_u[35] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', 'e', 'r', 'p', '1', '_', 'n', 'o', 'n',
    'M', 'o', 'n', 'o', 't', 'o', 'n', 'i', 'c', 'X' };

  (void)chartInstance;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 35), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c2_y));
}

static real_T c2_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_3)
{
  real_T c2_Vq;
  real_T c2_xi;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_x;
  boolean_T c2_b;
  real_T c2_b_xi;
  c2_sSYDZjSrzjRAmypAou1DUJH c2_r0;
  static c2_sSYDZjSrzjRAmypAou1DUJH c2_r1 = { { -10.0, -5.0, 0.0, 5.0, 10.0,
      15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0 }, { 0.0010905663337032931,
      0.0010905663337032936, -0.0026288316685164647, 0.00217676034036256,
      -0.0030222096929337773, 0.0013520784313725506, 0.00097389596744357969,
      0.0005123376988531254, 1.675323714391483E-5, -0.0020993506474287827,
      -0.0020993506474287849, -0.011138495005549401, 0.0052199999999999989,
      0.02157849500554939, -0.017853980022197548, 0.014797425083240834,
      -0.030535720310765829, -0.010254543840177559, 0.00435389567147614,
      0.012038961154273024, 0.012290259711431745, -0.019200000000000002,
      0.059828316685164681, 0.030235841657417661, 0.16422831668516466,
      0.18285089160192372, 0.16756811690714027, 0.088876640769515319,
      -0.11507467998520168, -0.14457792082870879, -0.062613636699962991,
      0.059032467628560861, 0.024483766185719607, -0.267, -0.11, 0.308, 1.34,
      2.08, 2.91, 2.76, 2.05, 1.5, 1.49, 1.83 } };

  int32_T exitg1;
  c2_xi = c2_varargin_3;
  c2_k = 1;
  do {
    exitg1 = 0;
    if (c2_k < 13) {
      c2_c_k = c2_k - 1;
      c2_x = -10.0 + 5.0 * (real_T)c2_c_k;
      c2_b = muDoubleScalarIsNaN(c2_x);
      if (c2_b) {
        c2_d_error(chartInstance);
        exitg1 = 1;
      } else {
        c2_k++;
      }
    } else {
      for (c2_b_k = 2; c2_b_k < 13; c2_b_k++) {
        c2_c_k = c2_b_k - 2;
        if (-10.0 + 5.0 * (real_T)(c2_c_k + 1) <= -10.0 + 5.0 * (real_T)c2_c_k)
        {
          c2_e_error(chartInstance);
        }
      }

      c2_b_xi = c2_xi;
      c2_r0 = c2_r1;
      c2_Vq = c2_ppval(chartInstance, &c2_r0, c2_b_xi);
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return c2_Vq;
}

static real_T c2_ppval(SFc2_QuanInstanceStruct *chartInstance,
  c2_sSYDZjSrzjRAmypAou1DUJH *c2_pp, real_T c2_x)
{
  real_T c2_v;
  real_T c2_b_x;
  real_T c2_c_x;
  boolean_T c2_b;
  int32_T c2_i50;
  int32_T c2_ip;
  real_T c2_b_pp[12];
  real_T c2_xloc;
  int32_T c2_ic;
  int32_T c2_b_ic;
  c2_b_x = c2_x;
  c2_c_x = c2_b_x;
  c2_b = muDoubleScalarIsNaN(c2_c_x);
  if (c2_b) {
    c2_v = c2_b_x;
  } else {
    for (c2_i50 = 0; c2_i50 < 12; c2_i50++) {
      c2_b_pp[c2_i50] = c2_pp->breaks[c2_i50];
    }

    c2_ip = c2_bsearch(chartInstance, c2_b_pp, c2_b_x) - 1;
    c2_xloc = c2_b_x - c2_pp->breaks[c2_ip];
    c2_v = c2_pp->coefs[c2_ip];
    for (c2_ic = 2; c2_ic < 5; c2_ic++) {
      c2_b_ic = c2_ic - 1;
      c2_v = c2_xloc * c2_v + c2_pp->coefs[c2_ip + c2_b_ic * 11];
    }
  }

  return c2_v;
}

static int32_T c2_bsearch(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x[12],
  real_T c2_xi)
{
  int32_T c2_low_i;
  int32_T c2_low_ip1;
  int32_T c2_high_i;
  int32_T c2_b_low_i;
  int32_T c2_b_high_i;
  int32_T c2_mid_i;
  (void)chartInstance;
  c2_low_i = 1;
  c2_low_ip1 = 1;
  c2_high_i = 12;
  while (c2_high_i > c2_low_ip1 + 1) {
    c2_b_low_i = c2_low_i;
    c2_b_high_i = c2_high_i;
    c2_mid_i = (c2_b_low_i + c2_b_high_i) >> 1;
    if (c2_xi >= c2_x[c2_mid_i - 1]) {
      c2_low_i = c2_mid_i;
      c2_low_ip1 = c2_low_i;
    } else {
      c2_high_i = c2_mid_i;
    }
  }

  return c2_low_i;
}

static real_T c2_b_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_3)
{
  real_T c2_Vq;
  real_T c2_xi;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_x;
  boolean_T c2_b;
  real_T c2_b_xi;
  c2_sSYDZjSrzjRAmypAou1DUJH c2_r2;
  static c2_sSYDZjSrzjRAmypAou1DUJH c2_r3 = { { -10.0, -5.0, 0.0, 5.0, 10.0,
      15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0 }, { 8.3545426891930565E-5,
      8.35454268919305E-5, -0.00038572713445965352, 0.00037136311094668472,
      -0.00041172530932708531, -0.00012446187363834328, 0.00079757280388046228,
      0.0019261706581164945, -0.0079422554363464438, 0.0071548510872692815,
      0.0071548510872693, -0.00017318140337895752, 0.0010800000000000013,
      0.0023331814033789584, -0.0034527256135158451, 0.0021177210506844249,
      -0.0040581585892218571, -0.0059250866937970194, 0.0060385053644099219,
      0.034931065236157341, -0.08420276630903932, 0.02312,
      -0.0072227286554034818, -0.0026886356722982648, 0.014377271344596536,
      0.0087795502939121087, 0.00210452747975501, -0.00759766021293214,
      -0.057513886628026457, -0.056946793274961979, 0.14790105972787435,
      -0.098457445636535426, -0.40387127718173249, 0.882, 0.852, 0.876, 0.958,
      0.962, 0.974, 0.819, 0.483, 0.59, 1.21, -0.493 } };

  int32_T exitg1;
  c2_xi = c2_varargin_3;
  c2_k = 1;
  do {
    exitg1 = 0;
    if (c2_k < 13) {
      c2_c_k = c2_k - 1;
      c2_x = -10.0 + 5.0 * (real_T)c2_c_k;
      c2_b = muDoubleScalarIsNaN(c2_x);
      if (c2_b) {
        c2_d_error(chartInstance);
        exitg1 = 1;
      } else {
        c2_k++;
      }
    } else {
      for (c2_b_k = 2; c2_b_k < 13; c2_b_k++) {
        c2_c_k = c2_b_k - 2;
        if (-10.0 + 5.0 * (real_T)(c2_c_k + 1) <= -10.0 + 5.0 * (real_T)c2_c_k)
        {
          c2_e_error(chartInstance);
        }
      }

      c2_b_xi = c2_xi;
      c2_r2 = c2_r3;
      c2_Vq = c2_ppval(chartInstance, &c2_r2, c2_b_xi);
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return c2_Vq;
}

static real_T c2_c_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_3)
{
  real_T c2_Vq;
  real_T c2_xi;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_x;
  boolean_T c2_b;
  real_T c2_b_xi;
  c2_sSYDZjSrzjRAmypAou1DUJH c2_r4;
  static c2_sSYDZjSrzjRAmypAou1DUJH c2_r5 = { { -10.0, -5.0, 0.0, 5.0, 10.0,
      15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0 }, { 0.0010413089324618737,
      0.0010413089324618732, -0.0015425446623093682, 0.00090486971677559915,
      -0.0023169342047930281, 0.0054988671023965142, -0.0051665342047930284,
      0.0013032697167755986, 0.00096145533769063343, -0.0036930910675381347,
      -0.0036930910675381152, -0.017219633986928103, -0.001599999999999999,
      0.014019633986928107, -0.0091185359477124178, 0.0044545098039215653,
      -0.03029950326797385, 0.052183503267973858, -0.025314509803921564,
      -0.0057654640522875871, 0.00865636601307192, -0.046740000000000018,
      0.060065446623093675, -0.034032723311546838, 0.028065446623093675,
      0.052570936819172118, 0.029250806100217869, -0.099974161220043573,
      0.0094458387799564291, 0.14379080610021786, -0.011609063180827888,
      0.0028454466230937417, -0.18757272331154709, -0.108, -0.108, -0.188, 0.11,
      0.258, 0.226, -0.344, 0.362, 0.611, 0.529, 0.298 } };

  int32_T exitg1;
  c2_xi = c2_varargin_3;
  c2_k = 1;
  do {
    exitg1 = 0;
    if (c2_k < 13) {
      c2_c_k = c2_k - 1;
      c2_x = -10.0 + 5.0 * (real_T)c2_c_k;
      c2_b = muDoubleScalarIsNaN(c2_x);
      if (c2_b) {
        c2_d_error(chartInstance);
        exitg1 = 1;
      } else {
        c2_k++;
      }
    } else {
      for (c2_b_k = 2; c2_b_k < 13; c2_b_k++) {
        c2_c_k = c2_b_k - 2;
        if (-10.0 + 5.0 * (real_T)(c2_c_k + 1) <= -10.0 + 5.0 * (real_T)c2_c_k)
        {
          c2_e_error(chartInstance);
        }
      }

      c2_b_xi = c2_xi;
      c2_r4 = c2_r5;
      c2_Vq = c2_ppval(chartInstance, &c2_r4, c2_b_xi);
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return c2_Vq;
}

static real_T c2_d_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_3)
{
  real_T c2_Vq;
  real_T c2_xi;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_x;
  boolean_T c2_b;
  real_T c2_b_xi;
  c2_sSYDZjSrzjRAmypAou1DUJH c2_r6;
  static c2_sSYDZjSrzjRAmypAou1DUJH c2_r7 = { { -10.0, -5.0, 0.0, 5.0, 10.0,
      15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0 }, { -0.023856791219632467,
      -0.023856791219632537, 0.012883956098162544, -0.010879033173017633,
      0.011432176593908001, -0.017249673202614373, 0.0095665162165495116,
      0.0045836083364163257, -0.025500949562214818, 0.035820189912442943,
      0.035820189912442957, 0.63585186829448714, 0.2780000000000003,
      -0.079851868294487732, 0.11340747317795043, -0.049778024417314076,
      0.12170462449130592, -0.13704047354790971, 0.0064572697003329529,
      0.075211394746577884, -0.30730284868664426, 0.22999999999999987,
      -5.9828395609816241, -1.4135802195091876, -0.42283956098162495,
      -0.25506153656431146, 0.063085707238870359, 0.42271870760882974,
      0.34603946232581084, -0.30687655691207305, 0.1014667653224809,
      -1.0589905043778516, -1.4455047478110734, -8.8, -25.8, -28.9, -31.4, -31.2,
      -30.7, -27.7, -28.2, -29.0, -29.8, -38.3 } };

  int32_T exitg1;
  c2_xi = c2_varargin_3;
  c2_k = 1;
  do {
    exitg1 = 0;
    if (c2_k < 13) {
      c2_c_k = c2_k - 1;
      c2_x = -10.0 + 5.0 * (real_T)c2_c_k;
      c2_b = muDoubleScalarIsNaN(c2_x);
      if (c2_b) {
        c2_d_error(chartInstance);
        exitg1 = 1;
      } else {
        c2_k++;
      }
    } else {
      for (c2_b_k = 2; c2_b_k < 13; c2_b_k++) {
        c2_c_k = c2_b_k - 2;
        if (-10.0 + 5.0 * (real_T)(c2_c_k + 1) <= -10.0 + 5.0 * (real_T)c2_c_k)
        {
          c2_e_error(chartInstance);
        }
      }

      c2_b_xi = c2_xi;
      c2_r6 = c2_r7;
      c2_Vq = c2_ppval(chartInstance, &c2_r6, c2_b_xi);
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return c2_Vq;
}

static real_T c2_e_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_3)
{
  real_T c2_Vq;
  real_T c2_xi;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_x;
  boolean_T c2_b;
  real_T c2_b_xi;
  c2_sSYDZjSrzjRAmypAou1DUJH c2_r8;
  static c2_sSYDZjSrzjRAmypAou1DUJH c2_r9 = { { -10.0, -5.0, 0.0, 5.0, 10.0,
      15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0 }, { -0.00065824636001150935,
      -0.00065824636001151032, 0.000667231800057549, -0.00053868084021868709,
      0.00054349156081719889, -0.000515285403050109, 0.001213650051383237,
      -0.0035713148024828375, 0.0054876091585481165, -0.0043791218317096219,
      -0.0043791218317096219, 0.01365369540017264, 0.0037800000000000047,
      -0.0060936954001726464, 0.003914781600690591, -0.0041654310025897139,
      0.00398694240966827, -0.0037423386360833652, 0.014462412134665186,
      -0.039107309902577395, 0.04320682747564434, -0.022480000000000014,
      -0.051812318000575466, 0.035356159000287733, 0.02378768199942451,
      0.012893113002014216, 0.011639865992518601, 0.010747423027911377,
      0.011970441895835897, 0.06557080938874503, -0.05765367945081596,
      -0.037156091585481173, 0.066478045792740609, -0.126, -0.126, 0.063, 0.113,
      0.208, 0.23, 0.319, 0.437, 0.68, 0.1, 0.447 } };

  int32_T exitg1;
  c2_xi = c2_varargin_3;
  c2_k = 1;
  do {
    exitg1 = 0;
    if (c2_k < 13) {
      c2_c_k = c2_k - 1;
      c2_x = -10.0 + 5.0 * (real_T)c2_c_k;
      c2_b = muDoubleScalarIsNaN(c2_x);
      if (c2_b) {
        c2_d_error(chartInstance);
        exitg1 = 1;
      } else {
        c2_k++;
      }
    } else {
      for (c2_b_k = 2; c2_b_k < 13; c2_b_k++) {
        c2_c_k = c2_b_k - 2;
        if (-10.0 + 5.0 * (real_T)(c2_c_k + 1) <= -10.0 + 5.0 * (real_T)c2_c_k)
        {
          c2_e_error(chartInstance);
        }
      }

      c2_b_xi = c2_xi;
      c2_r8 = c2_r9;
      c2_Vq = c2_ppval(chartInstance, &c2_r8, c2_b_xi);
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return c2_Vq;
}

static real_T c2_f_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_3)
{
  real_T c2_Vq;
  real_T c2_xi;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_x;
  boolean_T c2_b;
  real_T c2_b_xi;
  c2_sSYDZjSrzjRAmypAou1DUJH c2_r10;
  static c2_sSYDZjSrzjRAmypAou1DUJH c2_r11 = { { -10.0, -5.0, 0.0, 5.0, 10.0,
      15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0 }, { 0.00035873572573683575,
      0.0003587357257368357, -0.00025767862868417829, -7.2021211000123E-5,
      0.00020176347268467112, -0.00019903267973856177, 0.00020236724626957668,
      -0.0002904363053397456, 0.00037537797508940661, -0.00029907559501788115,
      -0.00029907559501788169, -0.0070810358860525363, -0.0016999999999999995,
      0.0036810358860525356, -0.00018414354421013941, -0.0012644617092119849,
      0.0017619903810580817, -0.0012234998150203453, 0.001812008879023304,
      -0.00254453570107288, 0.0030861339252682184, -0.0013999999999999985,
      0.026636786286841789, -0.017268393143420897, -0.0073632137131582169,
      0.010121247996053767, 0.0028782217289431483, 0.0053658650881736325,
      0.0080583179183623156, 0.011000863238377115, 0.0073382291281292372,
      0.010046220249105935, 0.018476889875447034, -0.36, -0.359, -0.443, -0.42,
      -0.383, -0.375, -0.329, -0.294, -0.23, -0.21, -0.12 } };

  int32_T exitg1;
  c2_xi = c2_varargin_3;
  c2_k = 1;
  do {
    exitg1 = 0;
    if (c2_k < 13) {
      c2_c_k = c2_k - 1;
      c2_x = -10.0 + 5.0 * (real_T)c2_c_k;
      c2_b = muDoubleScalarIsNaN(c2_x);
      if (c2_b) {
        c2_d_error(chartInstance);
        exitg1 = 1;
      } else {
        c2_k++;
      }
    } else {
      for (c2_b_k = 2; c2_b_k < 13; c2_b_k++) {
        c2_c_k = c2_b_k - 2;
        if (-10.0 + 5.0 * (real_T)(c2_c_k + 1) <= -10.0 + 5.0 * (real_T)c2_c_k)
        {
          c2_e_error(chartInstance);
        }
      }

      c2_b_xi = c2_xi;
      c2_r10 = c2_r11;
      c2_Vq = c2_ppval(chartInstance, &c2_r10, c2_b_xi);
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return c2_Vq;
}

static real_T c2_g_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_3)
{
  real_T c2_Vq;
  real_T c2_xi;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_x;
  boolean_T c2_b;
  real_T c2_b_xi;
  c2_sSYDZjSrzjRAmypAou1DUJH c2_r12;
  static c2_sSYDZjSrzjRAmypAou1DUJH c2_r13 = { { -10.0, -5.0, 0.0, 5.0, 10.0,
      15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0 }, { 0.029639331442430229,
      0.029639331442430212, -0.020036657212151119, 0.0066672974061742175,
      0.0024874675874542612, -0.0073371677559912777, 0.0049412034365108664,
      -0.0014676459900522035, 4.9380523697950543E-5, 0.0012701238952604059,
      0.0012701238952604068, -0.67178997163645349, -0.22719999999999987,
      0.21738997163645338, -0.083159886545813355, 0.01684957454679991,
      0.054161588358613816, -0.055895927981255332, 0.018222123566407667,
      -0.0037925662843753829, -0.0030518584289061165, 0.01599999999999998,
      3.951966572121512, -0.542983286060756, -0.59203342787848889,
      0.07911699757471119, -0.25243456242035595, 0.10262125210671272,
      0.093949553993505089, -0.094419468080733274, -0.022271681670571884,
      -0.056493805236979423, 0.0082469026184898639, -7.21, -0.54, -5.23, -5.26,
      -6.11, -6.64, -5.69, -6.0, -6.2, -6.4, -6.6 } };

  int32_T exitg1;
  c2_xi = c2_varargin_3;
  c2_k = 1;
  do {
    exitg1 = 0;
    if (c2_k < 13) {
      c2_c_k = c2_k - 1;
      c2_x = -10.0 + 5.0 * (real_T)c2_c_k;
      c2_b = muDoubleScalarIsNaN(c2_x);
      if (c2_b) {
        c2_d_error(chartInstance);
        exitg1 = 1;
      } else {
        c2_k++;
      }
    } else {
      for (c2_b_k = 2; c2_b_k < 13; c2_b_k++) {
        c2_c_k = c2_b_k - 2;
        if (-10.0 + 5.0 * (real_T)(c2_c_k + 1) <= -10.0 + 5.0 * (real_T)c2_c_k)
        {
          c2_e_error(chartInstance);
        }
      }

      c2_b_xi = c2_xi;
      c2_r12 = c2_r13;
      c2_Vq = c2_ppval(chartInstance, &c2_r12, c2_b_xi);
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return c2_Vq;
}

static real_T c2_h_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_3)
{
  real_T c2_Vq;
  real_T c2_xi;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_x;
  boolean_T c2_b;
  real_T c2_b_xi;
  c2_sSYDZjSrzjRAmypAou1DUJH c2_r14;
  static c2_sSYDZjSrzjRAmypAou1DUJH c2_r15 = { { -10.0, -5.0, 0.0, 5.0, 10.0,
      15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0 }, { 4.1386344391005886E-5,
      4.1386344391005907E-5, 0.0001050682780449706, -0.00032565945657088842,
      0.00021356954823858269, 0.00015138126361655832, -0.00018709460270481412,
      0.0002289971472026968, -0.0011128939861059725, 0.0017265787972211941,
      0.0017265787972211954, -0.0012607951658650891, -0.00064000000000000038,
      -1.9204834134911412E-5, 0.001556819336539648, -0.0033280725120236792,
      -0.00012452928844494042, 0.0021461896658034325, -0.00066022937476877919,
      0.0027747278332716724, -0.013918681958317917, 0.011979999999999998,
      0.0086693172195503, -0.000834658609775148, -0.0041306827804497093,
      0.0035573897315739722, -0.0052988761458461773, -0.022561885148189263,
      -0.012453583261396794, -0.0050237818062235269, 0.0055487104862909425,
      -0.050171060138940272, -0.059864469930529879, -0.38, -0.363, -0.378,
      -0.386, -0.37, -0.453, -0.55, -0.582, -0.595, -0.637, -1.02 } };

  int32_T exitg1;
  c2_xi = c2_varargin_3;
  c2_k = 1;
  do {
    exitg1 = 0;
    if (c2_k < 13) {
      c2_c_k = c2_k - 1;
      c2_x = -10.0 + 5.0 * (real_T)c2_c_k;
      c2_b = muDoubleScalarIsNaN(c2_x);
      if (c2_b) {
        c2_d_error(chartInstance);
        exitg1 = 1;
      } else {
        c2_k++;
      }
    } else {
      for (c2_b_k = 2; c2_b_k < 13; c2_b_k++) {
        c2_c_k = c2_b_k - 2;
        if (-10.0 + 5.0 * (real_T)(c2_c_k + 1) <= -10.0 + 5.0 * (real_T)c2_c_k)
        {
          c2_e_error(chartInstance);
        }
      }

      c2_b_xi = c2_xi;
      c2_r14 = c2_r15;
      c2_Vq = c2_ppval(chartInstance, &c2_r14, c2_b_xi);
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return c2_Vq;
}

static real_T c2_i_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_3)
{
  real_T c2_Vq;
  real_T c2_xi;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_x;
  boolean_T c2_b;
  real_T c2_b_xi;
  c2_sSYDZjSrzjRAmypAou1DUJH c2_r16;
  static c2_sSYDZjSrzjRAmypAou1DUJH c2_r17 = { { -10.0, -5.0, 0.0, 5.0, 10.0,
      15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0 }, { -0.00019297120072347607,
      -0.00019297120072347596, 0.00038085600361737989, -0.00031445281374604353,
      0.00029295525136679415, -9.7368191721133049E-5, -0.00037548248448226242,
      0.00043129812965018278, -5.7100341184691646E-6, -0.000360457993176306,
      -0.00036045799317630629, 0.0030745680108521405, 0.00017999999999999982,
      -0.00271456801085214, 0.0029982720434085586, -0.0017185201627820943,
      0.0026758086077198181, 0.001215285731902823, -0.0044169515353311125,
      0.00205252040942163, 0.0019668698976445915, -0.0034399999999999995,
      -0.0123485600361738, 0.0039242800180869, -0.008748560036173799,
      -0.0073300398733917044, -0.00093128047025938345, 0.0038551617544292376,
      0.023310633452542442, 0.007302304435400993, -0.00451985119414642,
      0.01557710034118469, 0.0082114498294076563, 0.061, 0.052, 0.052, -0.012,
      -0.013, -0.024, 0.05, 0.15, 0.13, 0.158, 0.24 } };

  int32_T exitg1;
  c2_xi = c2_varargin_3;
  c2_k = 1;
  do {
    exitg1 = 0;
    if (c2_k < 13) {
      c2_c_k = c2_k - 1;
      c2_x = -10.0 + 5.0 * (real_T)c2_c_k;
      c2_b = muDoubleScalarIsNaN(c2_x);
      if (c2_b) {
        c2_d_error(chartInstance);
        exitg1 = 1;
      } else {
        c2_k++;
      }
    } else {
      for (c2_b_k = 2; c2_b_k < 13; c2_b_k++) {
        c2_c_k = c2_b_k - 2;
        if (-10.0 + 5.0 * (real_T)(c2_c_k + 1) <= -10.0 + 5.0 * (real_T)c2_c_k)
        {
          c2_e_error(chartInstance);
        }
      }

      c2_b_xi = c2_xi;
      c2_r16 = c2_r17;
      c2_Vq = c2_ppval(chartInstance, &c2_r16, c2_b_xi);
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return c2_Vq;
}

static real_T c2_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_Xq;
  real_T c2_Yq;
  real_T c2_b_Xq;
  real_T c2_b_Yq;
  real_T c2_c_Xq;
  real_T c2_c_Yq;
  c2_Xq = c2_varargin_4;
  c2_Yq = c2_varargin_5;
  c2_b_Xq = c2_Xq;
  c2_b_Yq = c2_Yq;
  c2_c_Xq = c2_b_Xq;
  c2_c_Yq = c2_b_Yq;
  return c2_TensorInterp23(chartInstance, c2_c_Yq, c2_c_Xq);
}

static void c2_b_StringToMethodID(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_interp2_validate(SFc2_QuanInstanceStruct *chartInstance, uint8_T
  c2_METHOD)
{
  (void)chartInstance;
  (void)c2_METHOD;
}

static void c2_c_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static boolean_T c2_isplaid(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_1, real_T c2_varargin_2)
{
  (void)chartInstance;
  (void)c2_varargin_1;
  (void)c2_varargin_2;
  return true;
}

static void c2_d_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static real_T c2_TensorInterp23(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_wi;
  real_T c2_a;
  real_T c2_uxi;
  real_T c2_uxi_data[1];
  int32_T c2_uxi_sizes[2];
  int32_T c2_indx_data[1];
  int32_T c2_indx_sizes;
  int32_T c2_ipos;
  real_T c2_uyi;
  int32_T c2_b_ipos;
  real_T c2_ix;
  real_T c2_b_ix;
  real_T c2_b_a;
  real_T c2_uyi_data[1];
  int32_T c2_uyi_sizes[2];
  int32_T c2_c_ipos;
  int32_T c2_d_ipos;
  real_T c2_iy;
  real_T c2_b_iy;
  real_T c2_outsize[2];
  real_T c2_nxxi;
  int32_T c2_i51;
  real_T c2_varargin_3[36];
  c2_s5aHNKucFmPykI7NAl3JfH c2_pp;
  static real_T c2_b_varargin_3[36] = { 1060.0, 635.0, 60.0, -1020.0, -2700.0,
    -3600.0, 670.0, 425.0, 25.0, -710.0, -1900.0, -1400.0, 880.0, 690.0, 345.0,
    -300.0, -1300.0, -595.0, 1140.0, 1010.0, 755.0, 350.0, -247.0, -342.0,
    1500.0, 1330.0, 1130.0, 910.0, 600.0, -200.0, 1860.0, 1700.0, 1525.0, 1360.0,
    1100.0, 700.0 };

  real_T c2_sv[2];
  int32_T c2_i52;
  int32_T c2_varargout_1_sizes[2];
  int32_T c2_varargout_1;
  int32_T c2_b_varargout_1;
  int32_T c2_loop_ub;
  int32_T c2_i53;
  real_T c2_b_nxxi;
  real_T c2_varargout_1_data[6];
  int32_T c2_i54;
  int32_T c2_j;
  real_T c2_b_j;
  c2_s5aHNKucFmPykI7NAl3JfH c2_b_pp;
  int32_T c2_b_uyi;
  int32_T c2_c_uyi[1];
  int32_T c2_d_uyi;
  real_T c2_vkj[6];
  int32_T c2_b_loop_ub;
  int32_T c2_i;
  int32_T c2_i55;
  real_T c2_b_i;
  c2_sca5AIK8nC7yf5Qh9WghmhF c2_ppk_data;
  c2_sca5AIK8nC7yf5Qh9WghmhF_size c2_ppk_elems_sizes;
  int32_T c2_i56;
  int32_T c2_uwi_sizes[2];
  int32_T c2_uwi;
  int32_T c2_b_uwi;
  int32_T c2_c_loop_ub;
  int32_T c2_i57;
  real_T c2_c_nxxi;
  real_T c2_uwi_data[1];
  int32_T c2_i58;
  int32_T c2_c_j;
  c2_sca5AIK8nC7yf5Qh9WghmhF_size c2_b_ppk_elems_sizes;
  c2_sca5AIK8nC7yf5Qh9WghmhF c2_b_ppk_data;
  int32_T c2_e_uyi[1];
  real_T c2_vkj_data[1];
  int32_T c2_vkj_sizes;
  real_T c2_d4;
  int32_T c2_i59;
  int32_T c2_c_i;
  if (c2_isplaid(chartInstance, c2_varargin_4, c2_varargin_5)) {
    c2_uxi = c2_varargin_4;
    c2_uyi = c2_varargin_5;
    c2_wi = c2_TensorGriddedInterp(chartInstance, c2_uxi, c2_uyi);
  } else {
    c2_a = c2_varargin_4;
    c2_unique_vector(chartInstance, c2_a, c2_uxi_data, c2_uxi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_ipos);
    c2_b_ipos = c2_ipos;
    c2_ix = (real_T)c2_b_ipos;
    c2_b_ix = c2_ix;
    c2_b_a = c2_varargin_5;
    c2_unique_vector(chartInstance, c2_b_a, c2_uyi_data, c2_uyi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_c_ipos);
    c2_d_ipos = c2_c_ipos;
    c2_iy = (real_T)c2_d_ipos;
    c2_b_iy = c2_iy;
    c2_outsize[0] = (real_T)c2_uxi_sizes[1];
    c2_outsize[1] = (real_T)c2_uyi_sizes[1];
    c2_nxxi = (real_T)c2_uyi_sizes[1];
    for (c2_i51 = 0; c2_i51 < 36; c2_i51++) {
      c2_varargin_3[c2_i51] = c2_b_varargin_3[c2_i51];
    }

    c2_splinepp(chartInstance, c2_varargin_3, &c2_pp);
    c2_sv[0] = (real_T)c2_uyi_sizes[1];
    c2_sv[1] = 6.0;
    for (c2_i52 = 0; c2_i52 < 2; c2_i52++) {
      c2_sv[c2_i52] = _SFD_NON_NEGATIVE_CHECK("", c2_sv[c2_i52]);
    }

    c2_varargout_1_sizes[0] = (int32_T)c2_sv[0];
    c2_varargout_1_sizes[1] = 6;
    c2_varargout_1 = c2_varargout_1_sizes[0];
    c2_b_varargout_1 = c2_varargout_1_sizes[1];
    c2_loop_ub = (int32_T)c2_sv[0] * (int32_T)c2_sv[1] - 1;
    for (c2_i53 = 0; c2_i53 <= c2_loop_ub; c2_i53++) {
      c2_varargout_1_data[c2_i53] = 0.0;
    }

    c2_b_nxxi = c2_nxxi;
    c2_i54 = (int32_T)c2_b_nxxi;
    _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_b_nxxi, mxDOUBLE_CLASS, c2_i54);
    for (c2_j = 0; c2_j < c2_i54; c2_j++) {
      c2_b_j = 1.0 + (real_T)c2_j;
      c2_b_pp = c2_pp;
      c2_c_uyi[0] = c2_uyi_sizes[1];
      c2_b_ppval(chartInstance, &c2_b_pp, c2_uyi_data[(int32_T)c2_b_j - 1],
                 c2_vkj);
      for (c2_i = 0; c2_i < 6; c2_i++) {
        c2_b_i = 1.0 + (real_T)c2_i;
        c2_varargout_1_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
          c2_vkj[(int32_T)c2_b_i - 1];
      }
    }

    c2_uyi_sizes[0] = 1;
    c2_uyi_sizes[1] = c2_uxi_sizes[1];
    c2_b_uyi = c2_uyi_sizes[0];
    c2_d_uyi = c2_uyi_sizes[1];
    c2_b_loop_ub = c2_uxi_sizes[0] * c2_uxi_sizes[1] - 1;
    for (c2_i55 = 0; c2_i55 <= c2_b_loop_ub; c2_i55++) {
      c2_uyi_data[c2_i55] = c2_uxi_data[c2_i55];
    }

    c2_nxxi = (real_T)c2_uyi_sizes[1];
    c2_spline(chartInstance, c2_varargout_1_data, c2_varargout_1_sizes,
              &c2_ppk_data, &c2_ppk_elems_sizes);
    for (c2_i56 = 0; c2_i56 < 2; c2_i56++) {
      c2_outsize[c2_i56] = _SFD_NON_NEGATIVE_CHECK("", c2_outsize[c2_i56]);
    }

    c2_uwi_sizes[0] = (int32_T)c2_outsize[0];
    c2_uwi_sizes[1] = (int32_T)c2_outsize[1];
    c2_uwi = c2_uwi_sizes[0];
    c2_b_uwi = c2_uwi_sizes[1];
    c2_c_loop_ub = (int32_T)c2_outsize[0] * (int32_T)c2_outsize[1] - 1;
    for (c2_i57 = 0; c2_i57 <= c2_c_loop_ub; c2_i57++) {
      c2_uwi_data[c2_i57] = 0.0;
    }

    c2_c_nxxi = c2_nxxi;
    c2_i58 = (int32_T)c2_c_nxxi;
    _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_c_nxxi, mxDOUBLE_CLASS, c2_i58);
    for (c2_c_j = 0; c2_c_j < c2_i58; c2_c_j++) {
      c2_b_j = 1.0 + (real_T)c2_c_j;
      c2_b_ppk_elems_sizes = c2_ppk_elems_sizes;
      c2_b_ppk_data = c2_ppk_data;
      c2_e_uyi[0] = c2_uyi_sizes[1];
      c2_d_ppval(chartInstance, &c2_b_ppk_data, c2_b_ppk_elems_sizes,
                 c2_uyi_data[(int32_T)c2_b_j - 1], c2_vkj_data, &c2_vkj_sizes);
      c2_d4 = (real_T)c2_vkj_sizes;
      c2_i59 = (int32_T)c2_d4;
      _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_d4, mxDOUBLE_CLASS, c2_i59);
      for (c2_c_i = 0; c2_c_i < c2_i59; c2_c_i++) {
        c2_b_i = 1.0 + (real_T)c2_c_i;
        c2_uwi_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
          c2_vkj_data[(int32_T)c2_b_i - 1];
      }
    }

    c2_wi = c2_uwi_data[((int32_T)c2_b_ix + c2_uwi_sizes[0] * ((int32_T)c2_b_iy
      - 1)) - 1];
  }

  return c2_wi;
}

static real_T c2_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_xxi;
  int32_T c2_i60;
  real_T c2_varargin_3[36];
  c2_s5aHNKucFmPykI7NAl3JfH c2_pp;
  static real_T c2_b_varargin_3[36] = { 1060.0, 635.0, 60.0, -1020.0, -2700.0,
    -3600.0, 670.0, 425.0, 25.0, -710.0, -1900.0, -1400.0, 880.0, 690.0, 345.0,
    -300.0, -1300.0, -595.0, 1140.0, 1010.0, 755.0, 350.0, -247.0, -342.0,
    1500.0, 1330.0, 1130.0, 910.0, 600.0, -200.0, 1860.0, 1700.0, 1525.0, 1360.0,
    1100.0, 700.0 };

  int32_T c2_i61;
  real_T c2_vkj[6];
  real_T c2_varargout_1[6];
  int32_T c2_i;
  real_T c2_b_i;
  c2_syzabYTcbAiThYrkWkxvAFC c2_b_pp;
  real_T c2_b_vkj;
  c2_xxi = c2_varargin_5;
  for (c2_i60 = 0; c2_i60 < 36; c2_i60++) {
    c2_varargin_3[c2_i60] = c2_b_varargin_3[c2_i60];
  }

  c2_splinepp(chartInstance, c2_varargin_3, &c2_pp);
  for (c2_i61 = 0; c2_i61 < 6; c2_i61++) {
    c2_varargout_1[c2_i61] = 0.0;
  }

  c2_b_ppval(chartInstance, &c2_pp, c2_xxi, c2_vkj);
  for (c2_i = 0; c2_i < 6; c2_i++) {
    c2_b_i = 1.0 + (real_T)c2_i;
    c2_varargout_1[(int32_T)(1.0 + (c2_b_i - 1.0)) - 1] = c2_vkj[(int32_T)c2_b_i
      - 1];
  }

  c2_xxi = c2_varargin_4;
  c2_b_splinepp(chartInstance, c2_varargout_1, &c2_b_pp);
  c2_b_vkj = c2_c_ppval(chartInstance, &c2_b_pp, c2_xxi);
  return c2_b_vkj;
}

static void c2_splinepp(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[36],
  c2_s5aHNKucFmPykI7NAl3JfH *c2_pp)
{
  int32_T c2_i62;
  real_T c2_b_y[36];
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_dx[5];
  int32_T c2_dpg;
  real_T c2_d1;
  int32_T c2_pg;
  real_T c2_d2;
  int32_T c2_pgp1;
  int32_T c2_j;
  int32_T c2_pgm1;
  int32_T c2_b_j;
  int32_T c2_c_j;
  int32_T c2_d_j;
  real_T c2_A;
  real_T c2_dvdf[30];
  real_T c2_b_A;
  int32_T c2_e_j;
  real_T c2_x;
  real_T c2_B;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_c_y;
  real_T c2_s[36];
  real_T c2_d_y;
  real_T c2_md[6];
  real_T c2_d_x;
  real_T c2_c_A;
  real_T c2_e_y;
  int32_T c2_d_k;
  real_T c2_e_x;
  real_T c2_f_y;
  real_T c2_f_x;
  real_T c2_g_y;
  real_T c2_d_A;
  real_T c2_b_B;
  real_T c2_g_x;
  real_T c2_h_y;
  real_T c2_h_x;
  real_T c2_i_y;
  real_T c2_r;
  int32_T c2_f_j;
  int32_T c2_e_k;
  real_T c2_c_B;
  real_T c2_j_y;
  real_T c2_e_A;
  real_T c2_k_y;
  real_T c2_d_B;
  real_T c2_i_x;
  real_T c2_l_y;
  int32_T c2_g_j;
  real_T c2_j_x;
  real_T c2_m_y;
  int32_T c2_h_j;
  int32_T c2_f_k;
  int32_T c2_i_j;
  real_T c2_f_A;
  real_T c2_e_B;
  int32_T c2_j_j;
  real_T c2_k_x;
  real_T c2_n_y;
  real_T c2_l_x;
  int32_T c2_i63;
  real_T c2_o_y;
  real_T c2_g_A;
  int32_T c2_k_j;
  real_T c2_p_y;
  real_T c2_f_B;
  int32_T c2_i64;
  real_T c2_dv25[6];
  real_T c2_m_x;
  real_T c2_q_y;
  real_T c2_n_x;
  real_T c2_h_A;
  real_T c2_r_y[36];
  real_T c2_s_y;
  real_T c2_g_B;
  real_T c2_t_y;
  real_T c2_o_x;
  real_T c2_u_y;
  real_T c2_p_x;
  real_T c2_v_y;
  real_T c2_w_y;
  for (c2_i62 = 0; c2_i62 < 36; c2_i62++) {
    c2_b_y[c2_i62] = c2_y[c2_i62];
  }

  c2_chckxy(chartInstance, c2_b_y);
  for (c2_k = 1; c2_k < 6; c2_k++) {
    c2_c_k = c2_k - 1;
    c2_dx[c2_c_k] = 10000.0 * (real_T)(c2_c_k + 1) - 10000.0 * (real_T)c2_c_k;
    c2_dpg = c2_c_k * 6;
    c2_pg = c2_dpg;
    c2_pgp1 = (c2_c_k + 1) * 6;
    for (c2_b_j = 1; c2_b_j < 7; c2_b_j++) {
      c2_c_j = c2_b_j - 1;
      c2_b_A = c2_y[c2_pgp1 + c2_c_j] - c2_y[c2_pg + c2_c_j];
      c2_B = c2_dx[c2_c_k];
      c2_c_x = c2_b_A;
      c2_d_y = c2_B;
      c2_d_x = c2_c_x;
      c2_e_y = c2_d_y;
      c2_f_y = c2_d_x / c2_e_y;
      c2_dvdf[c2_dpg + c2_c_j] = c2_f_y;
    }
  }

  for (c2_b_k = 2; c2_b_k < 6; c2_b_k++) {
    c2_c_k = c2_b_k - 2;
    c2_pg = (c2_c_k + 1) * 6;
    c2_pgm1 = c2_c_k * 6;
    c2_d1 = c2_dx[c2_c_k + 1];
    c2_d2 = c2_dx[c2_c_k];
    for (c2_d_j = 1; c2_d_j < 7; c2_d_j++) {
      c2_c_j = c2_d_j - 1;
      c2_s[c2_pg + c2_c_j] = 3.0 * (c2_d1 * c2_dvdf[c2_pgm1 + c2_c_j] + c2_d2 *
        c2_dvdf[c2_pg + c2_c_j]);
    }
  }

  c2_d1 = c2_dx[0];
  c2_d2 = c2_dx[1];
  for (c2_j = 1; c2_j < 7; c2_j++) {
    c2_c_j = c2_j - 1;
    c2_A = (c2_d1 + 40000.0) * c2_d2 * c2_dvdf[c2_c_j] + c2_d1 * c2_d1 *
      c2_dvdf[c2_c_j + 6];
    c2_x = c2_A;
    c2_b_x = c2_x;
    c2_c_y = c2_b_x / 20000.0;
    c2_s[c2_c_j] = c2_c_y;
  }

  c2_d1 = c2_dx[4];
  c2_d2 = c2_dx[3];
  for (c2_e_j = 1; c2_e_j < 7; c2_e_j++) {
    c2_c_j = c2_e_j + 17;
    c2_c_A = (c2_d1 + 40000.0) * c2_d2 * c2_dvdf[c2_c_j + 6] + c2_d1 * c2_d1 *
      c2_dvdf[c2_c_j];
    c2_e_x = c2_c_A;
    c2_f_x = c2_e_x;
    c2_g_y = c2_f_x / 20000.0;
    c2_s[c2_c_j + 12] = c2_g_y;
  }

  c2_md[0] = c2_dx[1];
  c2_md[5] = c2_dx[3];
  for (c2_d_k = 2; c2_d_k < 6; c2_d_k++) {
    c2_c_k = c2_d_k - 1;
    c2_md[c2_c_k] = 2.0 * (c2_dx[c2_c_k] + c2_dx[c2_c_k - 1]);
  }

  c2_d_A = c2_dx[1];
  c2_b_B = c2_md[0];
  c2_g_x = c2_d_A;
  c2_h_y = c2_b_B;
  c2_h_x = c2_g_x;
  c2_i_y = c2_h_y;
  c2_r = c2_h_x / c2_i_y;
  c2_md[1] -= c2_r * 20000.0;
  for (c2_f_j = 1; c2_f_j < 7; c2_f_j++) {
    c2_c_j = c2_f_j + 5;
    c2_s[c2_c_j] -= c2_r * c2_s[c2_c_j - 6];
  }

  for (c2_e_k = 3; c2_e_k < 6; c2_e_k++) {
    c2_c_k = c2_e_k - 1;
    c2_e_A = c2_dx[c2_c_k];
    c2_d_B = c2_md[c2_c_k - 1];
    c2_i_x = c2_e_A;
    c2_l_y = c2_d_B;
    c2_j_x = c2_i_x;
    c2_m_y = c2_l_y;
    c2_r = c2_j_x / c2_m_y;
    c2_md[c2_c_k] -= c2_r * c2_dx[c2_c_k - 2];
    c2_pg = c2_c_k * 6;
    c2_pgm1 = (c2_c_k - 1) * 6;
    for (c2_i_j = 1; c2_i_j < 7; c2_i_j++) {
      c2_c_j = c2_i_j - 1;
      c2_s[c2_pg + c2_c_j] -= c2_r * c2_s[c2_pgm1 + c2_c_j];
    }
  }

  c2_c_B = c2_md[4];
  c2_j_y = c2_c_B;
  c2_k_y = c2_j_y;
  c2_r = 20000.0 / c2_k_y;
  c2_md[5] -= c2_r * c2_dx[3];
  for (c2_g_j = 1; c2_g_j < 7; c2_g_j++) {
    c2_c_j = c2_g_j + 29;
    c2_s[c2_c_j] -= c2_r * c2_s[c2_c_j - 6];
  }

  for (c2_h_j = 1; c2_h_j < 7; c2_h_j++) {
    c2_c_j = c2_h_j + 29;
    c2_f_A = c2_s[c2_c_j];
    c2_e_B = c2_md[5];
    c2_k_x = c2_f_A;
    c2_n_y = c2_e_B;
    c2_l_x = c2_k_x;
    c2_o_y = c2_n_y;
    c2_p_y = c2_l_x / c2_o_y;
    c2_s[c2_c_j] = c2_p_y;
  }

  for (c2_f_k = 5; c2_f_k > 1; c2_f_k--) {
    c2_c_k = c2_f_k - 1;
    c2_pg = c2_c_k * 6;
    c2_pgp1 = (c2_c_k + 1) * 6;
    c2_d1 = c2_dx[c2_c_k - 1];
    for (c2_k_j = 1; c2_k_j < 7; c2_k_j++) {
      c2_c_j = c2_k_j - 1;
      c2_h_A = c2_s[c2_pg + c2_c_j] - c2_d1 * c2_s[c2_pgp1 + c2_c_j];
      c2_g_B = c2_md[c2_c_k];
      c2_o_x = c2_h_A;
      c2_u_y = c2_g_B;
      c2_p_x = c2_o_x;
      c2_v_y = c2_u_y;
      c2_w_y = c2_p_x / c2_v_y;
      c2_s[c2_pg + c2_c_j] = c2_w_y;
    }
  }

  for (c2_j_j = 1; c2_j_j < 7; c2_j_j++) {
    c2_c_j = c2_j_j - 1;
    c2_g_A = c2_s[c2_c_j] - 20000.0 * c2_s[c2_c_j + 6];
    c2_f_B = c2_md[0];
    c2_m_x = c2_g_A;
    c2_q_y = c2_f_B;
    c2_n_x = c2_m_x;
    c2_s_y = c2_q_y;
    c2_t_y = c2_n_x / c2_s_y;
    c2_s[c2_c_j] = c2_t_y;
  }

  for (c2_i63 = 0; c2_i63 < 6; c2_i63++) {
    c2_dv25[c2_i63] = 10000.0 * (real_T)c2_i63;
  }

  for (c2_i64 = 0; c2_i64 < 36; c2_i64++) {
    c2_r_y[c2_i64] = c2_y[c2_i64];
  }

  c2_pwchcore(chartInstance, c2_dv25, c2_r_y, c2_s, c2_dx, c2_dvdf, c2_pp);
}

static void c2_chckxy(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[36])
{
  boolean_T c2_p;
  int32_T c2_k;
  real_T c2_b_k;
  real_T c2_x;
  const mxArray *c2_b_y = NULL;
  boolean_T c2_b;
  static char_T c2_u[28] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'U', 'n', 's', 'u', 'p', 'p', 'o', 'r', 't', 'e', 'd',
    'N', 'a', 'N' };

  boolean_T exitg1;
  (void)chartInstance;
  c2_p = false;
  c2_k = 0;
  exitg1 = false;
  while ((exitg1 == false) && (c2_k < 36)) {
    c2_b_k = 1.0 + (real_T)c2_k;
    c2_x = c2_y[(int32_T)c2_b_k - 1];
    c2_b = muDoubleScalarIsNaN(c2_x);
    if (c2_b) {
      c2_p = true;
      exitg1 = true;
    } else {
      c2_k++;
    }
  }

  if (!c2_p) {
  } else {
    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 28),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c2_b_y));
  }
}

static void c2_e_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_pwchcore(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x[6],
  real_T c2_y[36], real_T c2_s[36], real_T c2_dx[5], real_T c2_divdif[30],
  c2_s5aHNKucFmPykI7NAl3JfH *c2_pp)
{
  int32_T c2_i65;
  int32_T c2_j;
  int32_T c2_b_j;
  real_T c2_dxj;
  int32_T c2_joffset;
  int32_T c2_i;
  int32_T c2_b_i;
  real_T c2_divdifij;
  real_T c2_A;
  real_T c2_B;
  real_T c2_b_x;
  real_T c2_b_y;
  real_T c2_c_x;
  real_T c2_c_y;
  real_T c2_dzzdx;
  real_T c2_b_A;
  real_T c2_b_B;
  real_T c2_d_x;
  real_T c2_d_y;
  real_T c2_e_x;
  real_T c2_e_y;
  real_T c2_dzdxdx;
  real_T c2_c_A;
  real_T c2_c_B;
  real_T c2_f_x;
  real_T c2_f_y;
  real_T c2_g_x;
  real_T c2_g_y;
  real_T c2_h_y;
  (void)chartInstance;
  for (c2_i65 = 0; c2_i65 < 6; c2_i65++) {
    c2_pp->breaks[c2_i65] = c2_x[c2_i65];
  }

  for (c2_j = 1; c2_j < 6; c2_j++) {
    c2_b_j = c2_j - 1;
    c2_dxj = c2_dx[c2_b_j];
    c2_joffset = c2_b_j * 6;
    for (c2_i = 1; c2_i < 7; c2_i++) {
      c2_b_i = c2_i - 1;
      c2_divdifij = c2_divdif[c2_joffset + c2_b_i];
      c2_A = c2_divdifij - c2_s[c2_joffset + c2_b_i];
      c2_B = c2_dxj;
      c2_b_x = c2_A;
      c2_b_y = c2_B;
      c2_c_x = c2_b_x;
      c2_c_y = c2_b_y;
      c2_dzzdx = c2_c_x / c2_c_y;
      c2_b_A = c2_s[(c2_joffset + c2_b_i) + 6] - c2_divdifij;
      c2_b_B = c2_dxj;
      c2_d_x = c2_b_A;
      c2_d_y = c2_b_B;
      c2_e_x = c2_d_x;
      c2_e_y = c2_d_y;
      c2_dzdxdx = c2_e_x / c2_e_y;
      c2_c_A = c2_dzdxdx - c2_dzzdx;
      c2_c_B = c2_dxj;
      c2_f_x = c2_c_A;
      c2_f_y = c2_c_B;
      c2_g_x = c2_f_x;
      c2_g_y = c2_f_y;
      c2_h_y = c2_g_x / c2_g_y;
      c2_pp->coefs[c2_joffset + c2_b_i] = c2_h_y;
      c2_pp->coefs[(c2_joffset + c2_b_i) + 30] = 2.0 * c2_dzzdx - c2_dzdxdx;
      c2_pp->coefs[(c2_joffset + c2_b_i) + 60] = c2_s[c2_joffset + c2_b_i];
      c2_pp->coefs[(c2_joffset + c2_b_i) + 90] = c2_y[c2_joffset + c2_b_i];
    }
  }
}

static void c2_f_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_b_ppval(SFc2_QuanInstanceStruct *chartInstance,
  c2_s5aHNKucFmPykI7NAl3JfH *c2_pp, real_T c2_x, real_T c2_v[6])
{
  real_T c2_b_x;
  boolean_T c2_b;
  int32_T c2_i66;
  int32_T c2_j;
  int32_T c2_ip;
  real_T c2_b_pp[6];
  int32_T c2_b_j;
  int32_T c2_icp;
  real_T c2_xloc;
  int32_T c2_c_j;
  int32_T c2_ic;
  int32_T c2_b_ic;
  int32_T c2_ic0;
  int32_T c2_d_j;
  c2_b_x = c2_x;
  c2_b = muDoubleScalarIsNaN(c2_b_x);
  if (c2_b) {
    for (c2_j = 1; c2_j < 7; c2_j++) {
      c2_b_j = c2_j - 1;
      c2_v[c2_b_j] = c2_x;
    }
  } else {
    for (c2_i66 = 0; c2_i66 < 6; c2_i66++) {
      c2_b_pp[c2_i66] = c2_pp->breaks[c2_i66];
    }

    c2_ip = c2_b_bsearch(chartInstance, c2_b_pp, c2_x) - 1;
    c2_icp = c2_ip * 6;
    c2_xloc = c2_x - c2_pp->breaks[c2_ip];
    for (c2_c_j = 1; c2_c_j < 7; c2_c_j++) {
      c2_b_j = c2_c_j - 1;
      c2_v[c2_b_j] = c2_pp->coefs[c2_icp + c2_b_j];
    }

    for (c2_ic = 2; c2_ic < 5; c2_ic++) {
      c2_b_ic = c2_ic - 1;
      c2_ic0 = (c2_icp + c2_b_ic * 30) - 1;
      for (c2_d_j = 1; c2_d_j < 7; c2_d_j++) {
        c2_b_j = c2_d_j - 1;
        c2_v[c2_b_j] = c2_xloc * c2_v[c2_b_j] + c2_pp->coefs[(c2_ic0 + c2_b_j) +
          1];
      }
    }
  }
}

static int32_T c2_b_bsearch(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x
  [6], real_T c2_xi)
{
  int32_T c2_low_i;
  int32_T c2_low_ip1;
  int32_T c2_high_i;
  int32_T c2_b_low_i;
  int32_T c2_b_high_i;
  int32_T c2_mid_i;
  (void)chartInstance;
  c2_low_i = 1;
  c2_low_ip1 = 1;
  c2_high_i = 6;
  while (c2_high_i > c2_low_ip1 + 1) {
    c2_b_low_i = c2_low_i;
    c2_b_high_i = c2_high_i;
    c2_mid_i = (c2_b_low_i + c2_b_high_i) >> 1;
    if (c2_xi >= c2_x[c2_mid_i - 1]) {
      c2_low_i = c2_mid_i;
      c2_low_ip1 = c2_low_i;
    } else {
      c2_high_i = c2_mid_i;
    }
  }

  return c2_low_i;
}

static void c2_b_splinepp(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[6],
  c2_syzabYTcbAiThYrkWkxvAFC *c2_pp)
{
  int32_T c2_i67;
  real_T c2_b_y[6];
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_dx[5];
  static real_T c2_x[6] = { 0.0, 0.2, 0.4, 0.6, 0.8, 1.0 };

  int32_T c2_dpg;
  real_T c2_d1;
  int32_T c2_pg;
  real_T c2_d2;
  int32_T c2_pgp1;
  real_T c2_A;
  real_T c2_dvdf[5];
  int32_T c2_pgm1;
  real_T c2_b_A;
  real_T c2_b_x;
  real_T c2_B;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_c_y;
  real_T c2_s[6];
  real_T c2_d_y;
  real_T c2_e_x;
  real_T c2_e_y;
  real_T c2_f_y;
  real_T c2_c_A;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_g_y;
  real_T c2_md[6];
  int32_T c2_d_k;
  real_T c2_d_A;
  real_T c2_b_B;
  real_T c2_h_x;
  real_T c2_h_y;
  real_T c2_i_x;
  real_T c2_i_y;
  real_T c2_r;
  int32_T c2_e_k;
  real_T c2_c_B;
  real_T c2_j_y;
  real_T c2_e_A;
  real_T c2_k_y;
  real_T c2_d_B;
  real_T c2_j_x;
  real_T c2_l_y;
  real_T c2_k_x;
  real_T c2_f_A;
  real_T c2_m_y;
  real_T c2_e_B;
  real_T c2_l_x;
  real_T c2_n_y;
  real_T c2_m_x;
  real_T c2_o_y;
  real_T c2_p_y;
  int32_T c2_f_k;
  real_T c2_g_A;
  real_T c2_f_B;
  real_T c2_n_x;
  real_T c2_q_y;
  real_T c2_o_x;
  real_T c2_h_A;
  real_T c2_r_y;
  real_T c2_g_B;
  real_T c2_s_y;
  real_T c2_p_x;
  real_T c2_t_y;
  int32_T c2_i68;
  real_T c2_q_x;
  real_T c2_u_y;
  real_T c2_v_y;
  int32_T c2_i69;
  real_T c2_r_x[6];
  real_T c2_w_y[6];
  for (c2_i67 = 0; c2_i67 < 6; c2_i67++) {
    c2_b_y[c2_i67] = c2_y[c2_i67];
  }

  c2_b_chckxy(chartInstance, c2_b_y);
  for (c2_k = 1; c2_k < 6; c2_k++) {
    c2_c_k = c2_k - 1;
    c2_dx[c2_c_k] = c2_x[c2_c_k + 1] - c2_x[c2_c_k];
    c2_dpg = c2_c_k;
    c2_pg = c2_dpg;
    c2_pgp1 = c2_c_k + 1;
    c2_b_A = c2_y[c2_pgp1] - c2_y[c2_pg];
    c2_B = c2_dx[c2_c_k];
    c2_d_x = c2_b_A;
    c2_d_y = c2_B;
    c2_e_x = c2_d_x;
    c2_e_y = c2_d_y;
    c2_f_y = c2_e_x / c2_e_y;
    c2_dvdf[c2_dpg] = c2_f_y;
  }

  for (c2_b_k = 2; c2_b_k < 6; c2_b_k++) {
    c2_c_k = c2_b_k - 2;
    c2_pg = c2_c_k + 1;
    c2_pgm1 = c2_c_k;
    c2_d1 = c2_dx[c2_c_k + 1];
    c2_d2 = c2_dx[c2_c_k];
    c2_s[c2_pg] = 3.0 * (c2_d1 * c2_dvdf[c2_pgm1] + c2_d2 * c2_dvdf[c2_pg]);
  }

  c2_d1 = c2_dx[0];
  c2_d2 = c2_dx[1];
  c2_A = (c2_d1 + 0.8) * c2_d2 * c2_dvdf[0] + c2_d1 * c2_d1 * c2_dvdf[1];
  c2_b_x = c2_A;
  c2_c_x = c2_b_x;
  c2_c_y = c2_c_x / 0.4;
  c2_s[0] = c2_c_y;
  c2_d1 = c2_dx[4];
  c2_d2 = c2_dx[3];
  c2_c_A = (c2_d1 + 0.8) * c2_d2 * c2_dvdf[4] + c2_d1 * c2_d1 * c2_dvdf[3];
  c2_f_x = c2_c_A;
  c2_g_x = c2_f_x;
  c2_g_y = c2_g_x / 0.4;
  c2_s[5] = c2_g_y;
  c2_md[0] = c2_dx[1];
  c2_md[5] = c2_dx[3];
  for (c2_d_k = 2; c2_d_k < 6; c2_d_k++) {
    c2_c_k = c2_d_k - 1;
    c2_md[c2_c_k] = 2.0 * (c2_dx[c2_c_k] + c2_dx[c2_c_k - 1]);
  }

  c2_d_A = c2_dx[1];
  c2_b_B = c2_md[0];
  c2_h_x = c2_d_A;
  c2_h_y = c2_b_B;
  c2_i_x = c2_h_x;
  c2_i_y = c2_h_y;
  c2_r = c2_i_x / c2_i_y;
  c2_md[1] -= c2_r * 0.4;
  c2_s[1] -= c2_r * c2_s[0];
  for (c2_e_k = 3; c2_e_k < 6; c2_e_k++) {
    c2_c_k = c2_e_k - 1;
    c2_e_A = c2_dx[c2_c_k];
    c2_d_B = c2_md[c2_c_k - 1];
    c2_j_x = c2_e_A;
    c2_l_y = c2_d_B;
    c2_k_x = c2_j_x;
    c2_m_y = c2_l_y;
    c2_r = c2_k_x / c2_m_y;
    c2_md[c2_c_k] -= c2_r * c2_dx[c2_c_k - 2];
    c2_pg = c2_c_k;
    c2_pgm1 = c2_c_k;
    c2_s[c2_pg] -= c2_r * c2_s[c2_pgm1 - 1];
  }

  c2_c_B = c2_md[4];
  c2_j_y = c2_c_B;
  c2_k_y = c2_j_y;
  c2_r = 0.4 / c2_k_y;
  c2_md[5] -= c2_r * c2_dx[3];
  c2_s[5] -= c2_r * c2_s[4];
  c2_f_A = c2_s[5];
  c2_e_B = c2_md[5];
  c2_l_x = c2_f_A;
  c2_n_y = c2_e_B;
  c2_m_x = c2_l_x;
  c2_o_y = c2_n_y;
  c2_p_y = c2_m_x / c2_o_y;
  c2_s[5] = c2_p_y;
  for (c2_f_k = 5; c2_f_k > 1; c2_f_k--) {
    c2_c_k = c2_f_k - 1;
    c2_pg = c2_c_k;
    c2_pgp1 = c2_c_k + 1;
    c2_d1 = c2_dx[c2_c_k - 1];
    c2_h_A = c2_s[c2_pg] - c2_d1 * c2_s[c2_pgp1];
    c2_g_B = c2_md[c2_c_k];
    c2_p_x = c2_h_A;
    c2_t_y = c2_g_B;
    c2_q_x = c2_p_x;
    c2_u_y = c2_t_y;
    c2_v_y = c2_q_x / c2_u_y;
    c2_s[c2_pg] = c2_v_y;
  }

  c2_g_A = c2_s[0] - 0.4 * c2_s[1];
  c2_f_B = c2_md[0];
  c2_n_x = c2_g_A;
  c2_q_y = c2_f_B;
  c2_o_x = c2_n_x;
  c2_r_y = c2_q_y;
  c2_s_y = c2_o_x / c2_r_y;
  c2_s[0] = c2_s_y;
  for (c2_i68 = 0; c2_i68 < 6; c2_i68++) {
    c2_r_x[c2_i68] = c2_x[c2_i68];
  }

  for (c2_i69 = 0; c2_i69 < 6; c2_i69++) {
    c2_w_y[c2_i69] = c2_y[c2_i69];
  }

  c2_b_pwchcore(chartInstance, c2_r_x, c2_w_y, c2_s, c2_dx, c2_dvdf, c2_pp);
}

static void c2_b_chckxy(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[6])
{
  boolean_T c2_p;
  int32_T c2_k;
  real_T c2_b_k;
  real_T c2_x;
  const mxArray *c2_b_y = NULL;
  boolean_T c2_b;
  static char_T c2_u[28] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'U', 'n', 's', 'u', 'p', 'p', 'o', 'r', 't', 'e', 'd',
    'N', 'a', 'N' };

  boolean_T exitg1;
  (void)chartInstance;
  c2_p = false;
  c2_k = 0;
  exitg1 = false;
  while ((exitg1 == false) && (c2_k < 6)) {
    c2_b_k = 1.0 + (real_T)c2_k;
    c2_x = c2_y[(int32_T)c2_b_k - 1];
    c2_b = muDoubleScalarIsNaN(c2_x);
    if (c2_b) {
      c2_p = true;
      exitg1 = true;
    } else {
      c2_k++;
    }
  }

  if (!c2_p) {
  } else {
    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 28),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c2_b_y));
  }
}

static void c2_g_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_b_pwchcore(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x[6],
  real_T c2_y[6], real_T c2_s[6], real_T c2_dx[5], real_T c2_divdif[5],
  c2_syzabYTcbAiThYrkWkxvAFC *c2_pp)
{
  int32_T c2_i70;
  int32_T c2_j;
  int32_T c2_b_j;
  real_T c2_dxj;
  int32_T c2_joffset;
  real_T c2_divdifij;
  real_T c2_A;
  real_T c2_B;
  real_T c2_b_x;
  real_T c2_b_y;
  real_T c2_c_x;
  real_T c2_c_y;
  real_T c2_dzzdx;
  real_T c2_b_A;
  real_T c2_b_B;
  real_T c2_d_x;
  real_T c2_d_y;
  real_T c2_e_x;
  real_T c2_e_y;
  real_T c2_dzdxdx;
  real_T c2_c_A;
  real_T c2_c_B;
  real_T c2_f_x;
  real_T c2_f_y;
  real_T c2_g_x;
  real_T c2_g_y;
  real_T c2_h_y;
  (void)chartInstance;
  for (c2_i70 = 0; c2_i70 < 6; c2_i70++) {
    c2_pp->breaks[c2_i70] = c2_x[c2_i70];
  }

  for (c2_j = 1; c2_j < 6; c2_j++) {
    c2_b_j = c2_j - 1;
    c2_dxj = c2_dx[c2_b_j];
    c2_joffset = c2_b_j;
    c2_divdifij = c2_divdif[c2_joffset];
    c2_A = c2_divdifij - c2_s[c2_joffset];
    c2_B = c2_dxj;
    c2_b_x = c2_A;
    c2_b_y = c2_B;
    c2_c_x = c2_b_x;
    c2_c_y = c2_b_y;
    c2_dzzdx = c2_c_x / c2_c_y;
    c2_b_A = c2_s[c2_joffset + 1] - c2_divdifij;
    c2_b_B = c2_dxj;
    c2_d_x = c2_b_A;
    c2_d_y = c2_b_B;
    c2_e_x = c2_d_x;
    c2_e_y = c2_d_y;
    c2_dzdxdx = c2_e_x / c2_e_y;
    c2_c_A = c2_dzdxdx - c2_dzzdx;
    c2_c_B = c2_dxj;
    c2_f_x = c2_c_A;
    c2_f_y = c2_c_B;
    c2_g_x = c2_f_x;
    c2_g_y = c2_f_y;
    c2_h_y = c2_g_x / c2_g_y;
    c2_pp->coefs[c2_joffset] = c2_h_y;
    c2_pp->coefs[c2_joffset + 5] = 2.0 * c2_dzzdx - c2_dzdxdx;
    c2_pp->coefs[c2_joffset + 10] = c2_s[c2_joffset];
    c2_pp->coefs[c2_joffset + 15] = c2_y[c2_joffset];
  }
}

static void c2_h_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static real_T c2_c_ppval(SFc2_QuanInstanceStruct *chartInstance,
  c2_syzabYTcbAiThYrkWkxvAFC *c2_pp, real_T c2_x)
{
  real_T c2_v;
  real_T c2_b_x;
  real_T c2_c_x;
  boolean_T c2_b;
  int32_T c2_i71;
  int32_T c2_ip;
  real_T c2_b_pp[6];
  real_T c2_xloc;
  int32_T c2_ic;
  int32_T c2_b_ic;
  c2_b_x = c2_x;
  c2_c_x = c2_b_x;
  c2_b = muDoubleScalarIsNaN(c2_c_x);
  if (c2_b) {
    c2_v = c2_b_x;
  } else {
    for (c2_i71 = 0; c2_i71 < 6; c2_i71++) {
      c2_b_pp[c2_i71] = c2_pp->breaks[c2_i71];
    }

    c2_ip = c2_b_bsearch(chartInstance, c2_b_pp, c2_b_x) - 1;
    c2_xloc = c2_b_x - c2_pp->breaks[c2_ip];
    c2_v = c2_pp->coefs[c2_ip];
    for (c2_ic = 2; c2_ic < 5; c2_ic++) {
      c2_b_ic = c2_ic - 1;
      c2_v = c2_xloc * c2_v + c2_pp->coefs[c2_ip + c2_b_ic * 5];
    }
  }

  return c2_v;
}

static void c2_unique_vector(SFc2_QuanInstanceStruct *chartInstance, real_T c2_a,
  real_T c2_b_data[], int32_T c2_b_sizes[2], int32_T c2_ndx_data[], int32_T
  *c2_ndx_sizes, int32_T *c2_pos)
{
  int32_T c2_idx;
  int32_T c2_i72;
  int32_T c2_b_b_sizes[2];
  int32_T c2_b;
  int32_T c2_b_b;
  int32_T c2_loop_ub;
  int32_T c2_i73;
  real_T c2_b_b_data[1];
  int32_T c2_nMInf;
  int32_T c2_nFinite;
  int32_T c2_nInf;
  int32_T c2_nNaN;
  int32_T c2_b_nMInf;
  int32_T c2_b_nFinite;
  int32_T c2_b_nInf;
  int32_T c2_b_nNaN;
  int32_T c2_nb;
  int32_T c2_b_a;
  int32_T c2_c_nMInf;
  int32_T c2_c_b;
  int32_T c2_d_b;
  int32_T c2_khi;
  int32_T c2_e_b;
  int32_T c2_c_a;
  boolean_T c2_overflow;
  int32_T c2_k;
  int32_T c2_j;
  real_T c2_x;
  int32_T c2_d_a;
  int32_T c2_k0;
  int32_T c2_e_a;
  int32_T c2_f_b;
  int32_T c2_f_a;
  int32_T c2_g_a;
  int32_T c2_c;
  int32_T c2_c_nNaN;
  int32_T c2_g_b;
  int32_T c2_c_nInf;
  real_T c2_h_a;
  int32_T c2_h_b;
  int32_T c2_i_b;
  real_T c2_j_b;
  int32_T c2_i_a;
  boolean_T c2_b_overflow;
  int32_T c2_k_b;
  real_T c2_A;
  boolean_T c2_c_overflow;
  real_T c2_b_x;
  real_T c2_c_x;
  int32_T c2_b_k0;
  int32_T c2_b_j;
  real_T c2_y;
  int32_T c2_j_a;
  int32_T c2_c_j;
  real_T c2_d_x;
  int32_T c2_i74;
  real_T c2_absxk;
  int32_T c2_k_a;
  int32_T c2_d_j;
  real_T c2_e_x;
  int32_T c2_l_b;
  const mxArray *c2_b_y = NULL;
  int32_T c2_l_a;
  real_T c2_f_x;
  int32_T c2_m_a;
  static char_T c2_u[30] = { 'C', 'o', 'd', 'e', 'r', ':', 'b', 'u', 'i', 'l',
    't', 'i', 'n', 's', ':', 'A', 's', 's', 'e', 'r', 't', 'i', 'o', 'n', 'F',
    'a', 'i', 'l', 'e', 'd' };

  boolean_T c2_b2;
  boolean_T c2_m_b;
  int32_T c2_n_b;
  boolean_T c2_b3;
  int32_T c2_n_a;
  boolean_T c2_b4;
  boolean_T c2_d_overflow;
  boolean_T c2_b5;
  int32_T c2_o_b;
  real_T c2_g_x;
  boolean_T c2_b6;
  int32_T c2_b_c;
  boolean_T c2_p_b;
  boolean_T c2_b7;
  int32_T c2_e_j;
  int32_T c2_i75;
  boolean_T c2_q_b;
  int32_T c2_i76;
  real_T c2_r;
  int32_T c2_r_b;
  int32_T c2_exponent;
  int32_T c2_s_b[1];
  int32_T c2_b_exponent;
  int32_T c2_c_b_sizes[2];
  real_T c2_h_x;
  int32_T c2_c_exponent;
  boolean_T c2_t_b;
  boolean_T c2_b8;
  int32_T c2_b_loop_ub;
  int32_T c2_i77;
  real_T c2_i_x;
  boolean_T c2_p;
  boolean_T c2_u_b;
  real_T c2_c_b_data[1];
  int32_T c2_c_loop_ub;
  int32_T c2_i78;
  int32_T c2_b_nb;
  int32_T c2_v_b;
  int32_T c2_w_b;
  boolean_T c2_e_overflow;
  int32_T c2_b_k;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  int32_T exitg1;
  c2_idx = c2_sortIdx(chartInstance, c2_a);
  for (c2_i72 = 0; c2_i72 < 2; c2_i72++) {
    c2_b_sizes[c2_i72] = 1;
  }

  c2_b_data[0] = c2_a;
  c2_b_b_sizes[0] = 1;
  c2_b_b_sizes[1] = c2_b_sizes[1];
  c2_b = c2_b_b_sizes[0];
  c2_b_b = c2_b_b_sizes[1];
  c2_loop_ub = c2_b_sizes[0] * c2_b_sizes[1] - 1;
  for (c2_i73 = 0; c2_i73 <= c2_loop_ub; c2_i73++) {
    c2_b_b_data[c2_i73] = c2_b_data[c2_i73];
  }

  c2_count_nonfinites(chartInstance, c2_b_b_data, c2_b_b_sizes, 1, &c2_nMInf,
                      &c2_nFinite, &c2_nInf, &c2_nNaN);
  c2_b_nMInf = c2_nMInf;
  c2_b_nFinite = c2_nFinite;
  c2_b_nInf = c2_nInf;
  c2_b_nNaN = c2_nNaN;
  c2_nb = 0;
  if (c2_b_nMInf > 0) {
    c2_nb = 1;
    c2_c_nMInf = c2_b_nMInf;
    c2_d_b = c2_c_nMInf;
    c2_e_b = c2_d_b;
    c2_overflow = ((!(1 > c2_e_b)) && (c2_e_b > 2147483646));
    if (c2_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_overflow);
    }

    for (c2_j = 1; c2_j <= c2_c_nMInf; c2_j++) {
      *c2_pos = 1;
    }
  }

  c2_b_a = c2_b_nMInf;
  c2_c_b = c2_b_nFinite;
  c2_khi = c2_b_a + c2_c_b;
  c2_c_a = c2_b_nMInf + 1;
  c2_k = c2_c_a;
  while (c2_k <= c2_khi) {
    c2_x = c2_b_data[c2_k - 1];
    c2_k0 = c2_k;
    do {
      exitg1 = 0;
      c2_g_a = c2_k + 1;
      c2_k = c2_g_a;
      if (c2_k > c2_khi) {
        exitg1 = 1;
      } else {
        c2_h_a = c2_b_data[c2_k - 1];
        c2_j_b = c2_x;
        c2_A = c2_j_b;
        c2_b_x = c2_A;
        c2_c_x = c2_b_x;
        c2_y = c2_c_x / 2.0;
        c2_d_x = c2_y;
        c2_absxk = c2_abs(chartInstance, c2_d_x);
        c2_e_x = c2_absxk;
        c2_f_x = c2_e_x;
        c2_m_b = muDoubleScalarIsInf(c2_f_x);
        c2_b4 = !c2_m_b;
        c2_g_x = c2_e_x;
        c2_p_b = muDoubleScalarIsNaN(c2_g_x);
        c2_b7 = !c2_p_b;
        c2_q_b = (c2_b4 && c2_b7);
        if (c2_q_b) {
          if (c2_absxk <= 2.2250738585072014E-308) {
            c2_r = 4.94065645841247E-324;
          } else {
            frexp(c2_absxk, &c2_exponent);
            c2_b_exponent = c2_exponent;
            c2_c_exponent = c2_b_exponent - 53;
            c2_c_exponent;
            c2_r = ldexp(1.0, c2_c_exponent);
          }
        } else {
          c2_r = rtNaN;
        }

        guard1 = false;
        guard2 = false;
        guard3 = false;
        if (c2_abs(chartInstance, c2_j_b - c2_h_a) < c2_r) {
          guard2 = true;
        } else {
          c2_h_x = c2_h_a;
          c2_t_b = muDoubleScalarIsInf(c2_h_x);
          if (c2_t_b) {
            c2_i_x = c2_j_b;
            c2_u_b = muDoubleScalarIsInf(c2_i_x);
            if (c2_u_b) {
              if (c2_h_a > 0.0 == c2_j_b > 0.0) {
                guard2 = true;
              } else {
                guard1 = true;
              }
            } else {
              guard3 = true;
            }
          } else {
            guard3 = true;
          }
        }

        if (guard3 == true) {
          guard1 = true;
        }

        if (guard2 == true) {
          c2_b8 = true;
        }

        if (guard1 == true) {
          c2_b8 = false;
        }

        c2_p = c2_b8;
        if (!c2_p) {
          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);

    c2_i_a = c2_nb + 1;
    c2_nb = c2_i_a;
    c2_b_data[c2_nb - 1] = c2_x;
    c2_b_k0 = c2_k0;
    c2_j_a = c2_k - 1;
    c2_i74 = c2_j_a;
    c2_k_a = c2_b_k0;
    c2_l_b = c2_i74;
    c2_m_a = c2_k_a;
    c2_n_b = c2_l_b;
    c2_d_overflow = ((!(c2_m_a > c2_n_b)) && (c2_n_b > 2147483646));
    if (c2_d_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_d_overflow);
    }

    for (c2_e_j = c2_b_k0; c2_e_j <= c2_i74; c2_e_j++) {
      *c2_pos = c2_nb;
    }
  }

  if (c2_b_nInf > 0) {
    c2_d_a = c2_nb + 1;
    c2_nb = c2_d_a;
    c2_f_a = c2_khi + 1;
    c2_c = c2_f_a - 1;
    c2_b_data[c2_nb - 1] = c2_b_data[c2_c];
    c2_c_nInf = c2_b_nInf;
    c2_i_b = c2_c_nInf;
    c2_k_b = c2_i_b;
    c2_c_overflow = ((!(1 > c2_k_b)) && (c2_k_b > 2147483646));
    if (c2_c_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_c_overflow);
    }

    for (c2_c_j = 1; c2_c_j <= c2_c_nInf; c2_c_j++) {
      *c2_pos = c2_nb;
    }
  }

  c2_e_a = c2_khi;
  c2_f_b = c2_b_nInf;
  c2_k = c2_e_a + c2_f_b;
  c2_c_nNaN = c2_b_nNaN;
  c2_g_b = c2_c_nNaN;
  c2_h_b = c2_g_b;
  c2_b_overflow = ((!(1 > c2_h_b)) && (c2_h_b > 2147483646));
  if (c2_b_overflow) {
    c2_check_forloop_overflow_error(chartInstance, c2_b_overflow);
  }

  for (c2_b_j = 1; c2_b_j <= c2_c_nNaN; c2_b_j++) {
    c2_d_j = c2_b_j;
    c2_l_a = c2_nb + 1;
    c2_nb = c2_l_a;
    c2_n_a = c2_k;
    c2_o_b = c2_d_j;
    c2_b_c = (c2_n_a + c2_o_b) - 1;
    c2_b_data[c2_nb - 1] = c2_b_data[c2_b_c];
    *c2_pos = c2_nb;
  }

  if (c2_nb <= 1) {
  } else {
    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 30),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c2_b_y));
  }

  c2_b2 = (1 > c2_nb);
  c2_b3 = c2_b2;
  c2_b5 = false;
  c2_b6 = (c2_b3 || c2_b5);
  if (c2_b6) {
    c2_i75 = 1;
    c2_i76 = 0;
  } else {
    c2_i75 = 1;
    c2_i76 = c2_nb;
  }

  c2_r_b = c2_b_sizes[1];
  c2_s_b[0] = c2_r_b;
  c2_c_b_sizes[0] = 1;
  c2_c_b_sizes[1] = (c2_i76 - c2_i75) + 1;
  c2_b_loop_ub = c2_i76 - c2_i75;
  for (c2_i77 = 0; c2_i77 <= c2_b_loop_ub; c2_i77++) {
    c2_c_b_data[c2_c_b_sizes[0] * c2_i77] = c2_b_data[(c2_i75 + c2_i77) - 1];
  }

  c2_b_sizes[0] = 1;
  c2_b_sizes[1] = c2_c_b_sizes[1];
  c2_c_loop_ub = c2_c_b_sizes[1] - 1;
  for (c2_i78 = 0; c2_i78 <= c2_c_loop_ub; c2_i78++) {
    c2_b_data[c2_b_sizes[0] * c2_i78] = c2_c_b_data[c2_c_b_sizes[0] * c2_i78];
  }

  *c2_ndx_sizes = (int32_T)_SFD_NON_NEGATIVE_CHECK("", (real_T)c2_nb);
  c2_b_nb = c2_nb;
  c2_v_b = c2_b_nb;
  c2_w_b = c2_v_b;
  c2_e_overflow = ((!(1 > c2_w_b)) && (c2_w_b > 2147483646));
  if (c2_e_overflow) {
    c2_check_forloop_overflow_error(chartInstance, c2_e_overflow);
  }

  for (c2_b_k = 1; c2_b_k <= c2_b_nb; c2_b_k++) {
    c2_k = c2_b_k - 1;
    c2_ndx_data[c2_k] = c2_idx;
  }
}

static int32_T c2_sortIdx(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x)
{
  (void)chartInstance;
  (void)c2_x;
  return 1;
}

static void c2_i_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_count_nonfinites(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_b_data[], int32_T c2_b_sizes[2], int32_T c2_nb, int32_T *c2_nMInf, int32_T *
  c2_nFinite, int32_T *c2_nPInf, int32_T *c2_nNaN)
{
  int32_T c2_k;
  real_T c2_x;
  boolean_T c2_b;
  int32_T c2_a;
  int32_T c2_b_a;
  real_T c2_b_x;
  int32_T c2_c_a;
  boolean_T c2_b_b;
  int32_T c2_c_b;
  int32_T c2_d_a;
  real_T c2_c_x;
  boolean_T c2_d_b;
  int32_T c2_e_a;
  int32_T c2_e_b;
  int32_T c2_c;
  int32_T c2_f_a;
  int32_T c2_g_a;
  int32_T c2_f_b;
  int32_T c2_h_a;
  int32_T c2_g_b;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  (void)chartInstance;
  (void)c2_b_sizes;
  c2_k = 0;
  exitg3 = false;
  while ((exitg3 == false) && (c2_k + 1 <= c2_nb)) {
    c2_x = c2_b_data[c2_k];
    c2_b = muDoubleScalarIsInf(c2_x);
    if (c2_b) {
      if (c2_b_data[c2_k] < 0.0) {
        c2_b_a = c2_k + 1;
        c2_k = c2_b_a;
      } else {
        exitg3 = true;
      }
    } else {
      exitg3 = true;
    }
  }

  c2_a = c2_k;
  *c2_nMInf = c2_a;
  c2_k = c2_nb;
  exitg2 = false;
  while ((exitg2 == false) && (c2_k >= 1)) {
    c2_b_x = c2_b_data[c2_k - 1];
    c2_b_b = muDoubleScalarIsNaN(c2_b_x);
    if (c2_b_b) {
      c2_d_a = c2_k - 1;
      c2_k = c2_d_a;
    } else {
      exitg2 = true;
    }
  }

  c2_c_a = c2_nb;
  c2_c_b = c2_k;
  *c2_nNaN = c2_c_a - c2_c_b;
  exitg1 = false;
  while ((exitg1 == false) && (c2_k >= 1)) {
    c2_c_x = c2_b_data[c2_k - 1];
    c2_d_b = muDoubleScalarIsInf(c2_c_x);
    if (c2_d_b) {
      if (c2_b_data[c2_k - 1] > 0.0) {
        c2_f_a = c2_k - 1;
        c2_k = c2_f_a;
      } else {
        exitg1 = true;
      }
    } else {
      exitg1 = true;
    }
  }

  c2_e_a = c2_nb;
  c2_e_b = c2_k;
  c2_c = c2_e_a - c2_e_b;
  c2_g_a = c2_c;
  c2_f_b = *c2_nNaN;
  *c2_nPInf = c2_g_a - c2_f_b;
  c2_h_a = c2_k;
  c2_g_b = *c2_nMInf;
  *c2_nFinite = c2_h_a - c2_g_b;
}

static void c2_check_forloop_overflow_error(SFc2_QuanInstanceStruct
  *chartInstance, boolean_T c2_overflow)
{
  const mxArray *c2_y = NULL;
  static char_T c2_u[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  const mxArray *c2_b_y = NULL;
  static char_T c2_b_u[5] = { 'i', 'n', 't', '3', '2' };

  (void)chartInstance;
  if (!c2_overflow) {
  } else {
    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 34),
                  false);
    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 5),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 2U, 14, c2_y, 14, c2_b_y));
  }
}

static real_T c2_abs(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x)
{
  real_T c2_b_x;
  real_T c2_c_x;
  (void)chartInstance;
  c2_b_x = c2_x;
  c2_c_x = c2_b_x;
  return muDoubleScalarAbs(c2_c_x);
}

static void c2_output_size(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_1_data[], int32_T c2_varargin_1_sizes[2], real_T
  c2_varargin_2_data[], int32_T c2_varargin_2_sizes[2], real_T c2_sz[2])
{
  (void)chartInstance;
  (void)c2_varargin_1_data;
  (void)c2_varargin_2_data;
  c2_sz[0] = (real_T)c2_varargin_1_sizes[1];
  c2_sz[1] = (real_T)c2_varargin_2_sizes[1];
}

static boolean_T c2_iscolumn(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_x_data[], int32_T c2_x_sizes[2])
{
  boolean_T c2_b9;
  (void)chartInstance;
  (void)c2_x_data;
  c2_b9 = ((real_T)c2_x_sizes[1] == 1.0);
  return c2_b9;
}

static void c2_spline(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y_data[],
                      int32_T c2_y_sizes[2], c2_sca5AIK8nC7yf5Qh9WghmhF
                      *c2_output_data, c2_sca5AIK8nC7yf5Qh9WghmhF_size
                      *c2_output_elems_sizes)
{
  int32_T c2_b_y_sizes[2];
  int32_T c2_y;
  int32_T c2_b_y;
  int32_T c2_loop_ub;
  int32_T c2_i79;
  real_T c2_b_y_data[6];
  int32_T c2_npages;
  int32_T c2_x_sizes[2];
  int32_T c2_x;
  int32_T c2_b_x;
  int32_T c2_b_loop_ub;
  int32_T c2_i80;
  int32_T c2_pglen;
  real_T c2_x_data[12];
  boolean_T c2_has_endslopes;
  int32_T c2_i81;
  int32_T c2_i82;
  int32_T c2_i83;
  real_T c2_c_y[2];
  int32_T c2_i84;
  real_T c2_szdvdf[2];
  int32_T c2_i85;
  int32_T c2_i86;
  int32_T c2_i87;
  int32_T c2_i88;
  int32_T c2_yoffset;
  real_T c2_szs[2];
  int32_T c2_i89;
  int32_T c2_dvdf_sizes[2];
  int32_T c2_i90;
  int32_T c2_s_sizes[2];
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_dx[5];
  static real_T c2_c_x[6] = { 0.0, 0.2, 0.4, 0.6, 0.8, 1.0 };

  int32_T c2_dpg;
  int32_T c2_pg;
  real_T c2_d31;
  int32_T c2_pgp1;
  real_T c2_dnnm2;
  int32_T c2_pgm1;
  int32_T c2_b_pglen;
  real_T c2_d1;
  int32_T c2_c_pglen;
  int32_T c2_b;
  real_T c2_d2;
  int32_T c2_b_b;
  int32_T c2_c_b;
  int32_T c2_d_pglen;
  int32_T c2_d_b;
  int32_T c2_e_pglen;
  boolean_T c2_overflow;
  int32_T c2_e_b;
  boolean_T c2_b_overflow;
  int32_T c2_f_b;
  int32_T c2_g_b;
  int32_T c2_h_b;
  boolean_T c2_c_overflow;
  boolean_T c2_d_overflow;
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_c_j;
  int32_T c2_d_j;
  int32_T c2_e_j;
  int32_T c2_pgend2;
  real_T c2_A;
  int32_T c2_pglast;
  real_T c2_s_data[6];
  real_T c2_B;
  int32_T c2_f_pglen;
  real_T c2_d_x;
  real_T c2_b_A;
  real_T c2_dvdf_data[5];
  int32_T c2_i_b;
  real_T c2_d_y;
  int32_T c2_pgm2;
  real_T c2_e_x;
  int32_T c2_j_b;
  real_T c2_f_x;
  real_T c2_g_x;
  boolean_T c2_e_overflow;
  real_T c2_e_y;
  real_T c2_f_y;
  real_T c2_g_y;
  int32_T c2_g_pglen;
  int32_T c2_k_b;
  int32_T c2_f_j;
  int32_T c2_l_b;
  boolean_T c2_f_overflow;
  real_T c2_md[6];
  int32_T c2_g_j;
  int32_T c2_d_k;
  real_T c2_c_A;
  real_T c2_d_A;
  real_T c2_h_x;
  real_T c2_b_B;
  real_T c2_i_x;
  real_T c2_j_x;
  real_T c2_h_y;
  real_T c2_i_y;
  real_T c2_k_x;
  real_T c2_j_y;
  real_T c2_r;
  int32_T c2_h_pglen;
  int32_T c2_m_b;
  int32_T c2_n_b;
  boolean_T c2_g_overflow;
  int32_T c2_h_j;
  int32_T c2_e_k;
  real_T c2_e_A;
  real_T c2_c_B;
  real_T c2_f_A;
  real_T c2_l_x;
  real_T c2_d_B;
  real_T c2_k_y;
  real_T c2_m_x;
  real_T c2_n_x;
  real_T c2_l_y;
  real_T c2_m_y;
  real_T c2_o_x;
  real_T c2_n_y;
  int32_T c2_i_pglen;
  int32_T c2_o_b;
  int32_T c2_j_pglen;
  int32_T c2_p_b;
  int32_T c2_q_b;
  boolean_T c2_h_overflow;
  int32_T c2_r_b;
  boolean_T c2_i_overflow;
  int32_T c2_i_j;
  int32_T c2_j_j;
  int32_T c2_k_pglen;
  int32_T c2_s_b;
  int32_T c2_t_b;
  boolean_T c2_j_overflow;
  int32_T c2_k_j;
  int32_T c2_f_k;
  real_T c2_g_A;
  real_T c2_e_B;
  int32_T c2_l_pglen;
  real_T c2_p_x;
  int32_T c2_u_b;
  real_T c2_o_y;
  int32_T c2_v_b;
  real_T c2_q_x;
  boolean_T c2_k_overflow;
  real_T c2_p_y;
  int32_T c2_m_pglen;
  real_T c2_q_y;
  int32_T c2_w_b;
  int32_T c2_l_j;
  int32_T c2_x_b;
  boolean_T c2_l_overflow;
  int32_T c2_i91;
  real_T c2_h_A;
  int32_T c2_m_j;
  real_T c2_f_B;
  int32_T c2_c_y_sizes[2];
  real_T c2_r_x[6];
  real_T c2_s_x;
  real_T c2_r_y;
  int32_T c2_s_y;
  real_T c2_t_x;
  real_T c2_i_A;
  int32_T c2_t_y;
  real_T c2_u_y;
  real_T c2_g_B;
  int32_T c2_c_loop_ub;
  real_T c2_v_y;
  real_T c2_u_x;
  int32_T c2_i92;
  real_T c2_w_y;
  real_T c2_v_x;
  real_T c2_x_y;
  real_T c2_c_y_data[6];
  real_T c2_y_y;
  c2_b_y_sizes[0] = c2_y_sizes[0];
  c2_b_y_sizes[1] = 6;
  c2_y = c2_b_y_sizes[0];
  c2_b_y = c2_b_y_sizes[1];
  c2_loop_ub = c2_y_sizes[0] * c2_y_sizes[1] - 1;
  for (c2_i79 = 0; c2_i79 <= c2_loop_ub; c2_i79++) {
    c2_b_y_data[c2_i79] = c2_y_data[c2_i79];
  }

  c2_c_chckxy(chartInstance, c2_b_y_data, c2_b_y_sizes);
  c2_npages = c2_y_sizes[1];
  c2_x_sizes[0] = c2_y_sizes[0];
  c2_x_sizes[1] = 6;
  c2_x = c2_x_sizes[0];
  c2_b_x = c2_x_sizes[1];
  c2_b_loop_ub = c2_y_sizes[0] * c2_y_sizes[1] - 1;
  for (c2_i80 = 0; c2_i80 <= c2_b_loop_ub; c2_i80++) {
    c2_x_data[c2_i80] = c2_y_data[c2_i80];
  }

  c2_pglen = c2_x_sizes[0];
  c2_has_endslopes = (c2_npages == 8);
  if (c2_has_endslopes) {
    for (c2_i82 = 0; c2_i82 < 2; c2_i82++) {
      c2_c_y[c2_i82] = (real_T)c2_y_sizes[c2_i82];
    }

    for (c2_i84 = 0; c2_i84 < 2; c2_i84++) {
      c2_szdvdf[c2_i84] = c2_c_y[c2_i84];
    }

    c2_szdvdf[1] = (real_T)c2_y_sizes[1] - 3.0;
    for (c2_i86 = 0; c2_i86 < 2; c2_i86++) {
      c2_c_y[c2_i86] = (real_T)c2_y_sizes[c2_i86];
    }

    for (c2_i88 = 0; c2_i88 < 2; c2_i88++) {
      c2_szs[c2_i88] = c2_c_y[c2_i88];
    }

    c2_szs[1] = (real_T)c2_y_sizes[1] - 2.0;
    c2_yoffset = c2_pglen;
  } else {
    for (c2_i81 = 0; c2_i81 < 2; c2_i81++) {
      c2_c_y[c2_i81] = (real_T)c2_y_sizes[c2_i81];
    }

    for (c2_i83 = 0; c2_i83 < 2; c2_i83++) {
      c2_szdvdf[c2_i83] = c2_c_y[c2_i83];
    }

    c2_szdvdf[1] = (real_T)c2_y_sizes[1] - 1.0;
    for (c2_i85 = 0; c2_i85 < 2; c2_i85++) {
      c2_c_y[c2_i85] = (real_T)c2_y_sizes[c2_i85];
    }

    for (c2_i87 = 0; c2_i87 < 2; c2_i87++) {
      c2_szs[c2_i87] = c2_c_y[c2_i87];
    }

    c2_yoffset = 0;
  }

  for (c2_i89 = 0; c2_i89 < 2; c2_i89++) {
    c2_szdvdf[c2_i89] = _SFD_NON_NEGATIVE_CHECK("", c2_szdvdf[c2_i89]);
  }

  c2_dvdf_sizes[0] = (int32_T)c2_szdvdf[0];
  c2_dvdf_sizes[1] = (int32_T)c2_szdvdf[1];
  for (c2_i90 = 0; c2_i90 < 2; c2_i90++) {
    c2_szs[c2_i90] = _SFD_NON_NEGATIVE_CHECK("", c2_szs[c2_i90]);
  }

  c2_s_sizes[0] = (int32_T)c2_szs[0];
  c2_s_sizes[1] = (int32_T)c2_szs[1];
  for (c2_k = 1; c2_k < 6; c2_k++) {
    c2_c_k = c2_k - 1;
    c2_dx[c2_c_k] = c2_c_x[c2_c_k + 1] - c2_c_x[c2_c_k];
    c2_dpg = c2_c_k * c2_pglen;
    c2_pg = c2_yoffset + c2_dpg;
    c2_pgp1 = c2_yoffset + (c2_c_k + 1) * c2_pglen;
    c2_b_pglen = c2_pglen;
    c2_b = c2_b_pglen;
    c2_c_b = c2_b;
    c2_overflow = ((!(1 > c2_c_b)) && (c2_c_b > 2147483646));
    if (c2_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_overflow);
    }

    for (c2_j = 1; c2_j <= c2_b_pglen; c2_j++) {
      c2_e_j = c2_j - 1;
      c2_A = c2_y_data[c2_pgp1 + c2_e_j] - c2_y_data[c2_pg + c2_e_j];
      c2_B = c2_dx[c2_c_k];
      c2_d_x = c2_A;
      c2_d_y = c2_B;
      c2_f_x = c2_d_x;
      c2_e_y = c2_d_y;
      c2_g_y = c2_f_x / c2_e_y;
      c2_dvdf_data[c2_dpg + c2_e_j] = c2_g_y;
    }
  }

  for (c2_b_k = 2; c2_b_k < 6; c2_b_k++) {
    c2_c_k = c2_b_k - 2;
    c2_pg = (c2_c_k + 1) * c2_pglen;
    c2_pgm1 = c2_c_k * c2_pglen;
    c2_d1 = c2_dx[c2_c_k + 1];
    c2_d2 = c2_dx[c2_c_k];
    c2_e_pglen = c2_pglen;
    c2_f_b = c2_e_pglen;
    c2_h_b = c2_f_b;
    c2_d_overflow = ((!(1 > c2_h_b)) && (c2_h_b > 2147483646));
    if (c2_d_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_d_overflow);
    }

    for (c2_d_j = 1; c2_d_j <= c2_e_pglen; c2_d_j++) {
      c2_e_j = c2_d_j - 1;
      c2_s_data[c2_pg + c2_e_j] = 3.0 * (c2_d1 * c2_dvdf_data[c2_pgm1 + c2_e_j]
        + c2_d2 * c2_dvdf_data[c2_pg + c2_e_j]);
    }
  }

  if (c2_has_endslopes) {
    c2_d31 = 0.0;
    c2_dnnm2 = 0.0;
    c2_c_pglen = c2_pglen;
    c2_b_b = c2_c_pglen;
    c2_d_b = c2_b_b;
    c2_b_overflow = ((!(1 > c2_d_b)) && (c2_d_b > 2147483646));
    if (c2_b_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_b_overflow);
    }

    for (c2_b_j = 1; c2_b_j <= c2_c_pglen; c2_b_j++) {
      c2_e_j = c2_b_j - 1;
      c2_s_data[c2_e_j] = c2_dx[1] * c2_y_data[c2_e_j];
    }

    c2_pgend2 = 7 * c2_pglen;
    c2_pglast = 5 * c2_pglen;
    c2_f_pglen = c2_pglen;
    c2_i_b = c2_f_pglen;
    c2_j_b = c2_i_b;
    c2_e_overflow = ((!(1 > c2_j_b)) && (c2_j_b > 2147483646));
    if (c2_e_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_e_overflow);
    }

    for (c2_f_j = 1; c2_f_j <= c2_f_pglen; c2_f_j++) {
      c2_e_j = c2_f_j - 1;
      c2_s_data[c2_pglast + c2_e_j] = c2_dx[3] * c2_y_data[c2_pgend2 + c2_e_j];
    }
  } else {
    c2_d31 = 0.4;
    c2_dnnm2 = 0.4;
    c2_d1 = c2_dx[0];
    c2_d2 = c2_dx[1];
    c2_d_pglen = c2_pglen;
    c2_e_b = c2_d_pglen;
    c2_g_b = c2_e_b;
    c2_c_overflow = ((!(1 > c2_g_b)) && (c2_g_b > 2147483646));
    if (c2_c_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_c_overflow);
    }

    for (c2_c_j = 1; c2_c_j <= c2_d_pglen; c2_c_j++) {
      c2_e_j = c2_c_j - 1;
      c2_b_A = (c2_d1 + 0.8) * c2_d2 * c2_dvdf_data[c2_e_j] + c2_d1 * c2_d1 *
        c2_dvdf_data[c2_pglen + c2_e_j];
      c2_e_x = c2_b_A;
      c2_g_x = c2_e_x;
      c2_f_y = c2_g_x / 0.4;
      c2_s_data[c2_e_j] = c2_f_y;
    }

    c2_pg = 5 * c2_pglen;
    c2_pgm1 = c2_pglen << 2;
    c2_pgm2 = 3 * c2_pglen;
    c2_d1 = c2_dx[4];
    c2_d2 = c2_dx[3];
    c2_g_pglen = c2_pglen;
    c2_k_b = c2_g_pglen;
    c2_l_b = c2_k_b;
    c2_f_overflow = ((!(1 > c2_l_b)) && (c2_l_b > 2147483646));
    if (c2_f_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_f_overflow);
    }

    for (c2_g_j = 1; c2_g_j <= c2_g_pglen; c2_g_j++) {
      c2_e_j = c2_g_j - 1;
      c2_c_A = (c2_d1 + 0.8) * c2_d2 * c2_dvdf_data[c2_pgm1 + c2_e_j] + c2_d1 *
        c2_d1 * c2_dvdf_data[c2_pgm2 + c2_e_j];
      c2_h_x = c2_c_A;
      c2_i_x = c2_h_x;
      c2_h_y = c2_i_x / 0.4;
      c2_s_data[c2_pg + c2_e_j] = c2_h_y;
    }
  }

  c2_md[0] = c2_dx[1];
  c2_md[5] = c2_dx[3];
  for (c2_d_k = 2; c2_d_k < 6; c2_d_k++) {
    c2_c_k = c2_d_k - 1;
    c2_md[c2_c_k] = 2.0 * (c2_dx[c2_c_k] + c2_dx[c2_c_k - 1]);
  }

  c2_d_A = c2_dx[1];
  c2_b_B = c2_md[0];
  c2_j_x = c2_d_A;
  c2_i_y = c2_b_B;
  c2_k_x = c2_j_x;
  c2_j_y = c2_i_y;
  c2_r = c2_k_x / c2_j_y;
  c2_md[1] -= c2_r * c2_d31;
  c2_h_pglen = c2_pglen;
  c2_m_b = c2_h_pglen;
  c2_n_b = c2_m_b;
  c2_g_overflow = ((!(1 > c2_n_b)) && (c2_n_b > 2147483646));
  if (c2_g_overflow) {
    c2_check_forloop_overflow_error(chartInstance, c2_g_overflow);
  }

  for (c2_h_j = 1; c2_h_j <= c2_h_pglen; c2_h_j++) {
    c2_e_j = c2_h_j - 1;
    c2_s_data[c2_pglen + c2_e_j] -= c2_r * c2_s_data[c2_e_j];
  }

  for (c2_e_k = 3; c2_e_k < 6; c2_e_k++) {
    c2_c_k = c2_e_k - 1;
    c2_f_A = c2_dx[c2_c_k];
    c2_d_B = c2_md[c2_c_k - 1];
    c2_m_x = c2_f_A;
    c2_l_y = c2_d_B;
    c2_o_x = c2_m_x;
    c2_n_y = c2_l_y;
    c2_r = c2_o_x / c2_n_y;
    c2_md[c2_c_k] -= c2_r * c2_dx[c2_c_k - 2];
    c2_pg = c2_c_k * c2_pglen;
    c2_pgm1 = (c2_c_k - 1) * c2_pglen;
    c2_j_pglen = c2_pglen;
    c2_q_b = c2_j_pglen;
    c2_r_b = c2_q_b;
    c2_i_overflow = ((!(1 > c2_r_b)) && (c2_r_b > 2147483646));
    if (c2_i_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_i_overflow);
    }

    for (c2_j_j = 1; c2_j_j <= c2_j_pglen; c2_j_j++) {
      c2_e_j = c2_j_j - 1;
      c2_s_data[c2_pg + c2_e_j] -= c2_r * c2_s_data[c2_pgm1 + c2_e_j];
    }
  }

  c2_e_A = c2_dnnm2;
  c2_c_B = c2_md[4];
  c2_l_x = c2_e_A;
  c2_k_y = c2_c_B;
  c2_n_x = c2_l_x;
  c2_m_y = c2_k_y;
  c2_r = c2_n_x / c2_m_y;
  c2_md[5] -= c2_r * c2_dx[3];
  c2_pg = 5 * c2_pglen;
  c2_pgm1 = c2_pglen << 2;
  c2_i_pglen = c2_pglen;
  c2_o_b = c2_i_pglen;
  c2_p_b = c2_o_b;
  c2_h_overflow = ((!(1 > c2_p_b)) && (c2_p_b > 2147483646));
  if (c2_h_overflow) {
    c2_check_forloop_overflow_error(chartInstance, c2_h_overflow);
  }

  for (c2_i_j = 1; c2_i_j <= c2_i_pglen; c2_i_j++) {
    c2_e_j = c2_i_j - 1;
    c2_s_data[c2_pg + c2_e_j] -= c2_r * c2_s_data[c2_pgm1 + c2_e_j];
  }

  c2_k_pglen = c2_pglen;
  c2_s_b = c2_k_pglen;
  c2_t_b = c2_s_b;
  c2_j_overflow = ((!(1 > c2_t_b)) && (c2_t_b > 2147483646));
  if (c2_j_overflow) {
    c2_check_forloop_overflow_error(chartInstance, c2_j_overflow);
  }

  for (c2_k_j = 1; c2_k_j <= c2_k_pglen; c2_k_j++) {
    c2_e_j = c2_k_j - 1;
    c2_g_A = c2_s_data[c2_pg + c2_e_j];
    c2_e_B = c2_md[5];
    c2_p_x = c2_g_A;
    c2_o_y = c2_e_B;
    c2_q_x = c2_p_x;
    c2_p_y = c2_o_y;
    c2_q_y = c2_q_x / c2_p_y;
    c2_s_data[c2_pg + c2_e_j] = c2_q_y;
  }

  for (c2_f_k = 5; c2_f_k > 1; c2_f_k--) {
    c2_c_k = c2_f_k - 1;
    c2_pg = c2_c_k * c2_pglen;
    c2_pgp1 = (c2_c_k + 1) * c2_pglen;
    c2_d1 = c2_dx[c2_c_k - 1];
    c2_m_pglen = c2_pglen;
    c2_w_b = c2_m_pglen;
    c2_x_b = c2_w_b;
    c2_l_overflow = ((!(1 > c2_x_b)) && (c2_x_b > 2147483646));
    if (c2_l_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_l_overflow);
    }

    for (c2_m_j = 1; c2_m_j <= c2_m_pglen; c2_m_j++) {
      c2_e_j = c2_m_j - 1;
      c2_i_A = c2_s_data[c2_pg + c2_e_j] - c2_d1 * c2_s_data[c2_pgp1 + c2_e_j];
      c2_g_B = c2_md[c2_c_k];
      c2_u_x = c2_i_A;
      c2_w_y = c2_g_B;
      c2_v_x = c2_u_x;
      c2_x_y = c2_w_y;
      c2_y_y = c2_v_x / c2_x_y;
      c2_s_data[c2_pg + c2_e_j] = c2_y_y;
    }
  }

  c2_l_pglen = c2_pglen;
  c2_u_b = c2_l_pglen;
  c2_v_b = c2_u_b;
  c2_k_overflow = ((!(1 > c2_v_b)) && (c2_v_b > 2147483646));
  if (c2_k_overflow) {
    c2_check_forloop_overflow_error(chartInstance, c2_k_overflow);
  }

  for (c2_l_j = 1; c2_l_j <= c2_l_pglen; c2_l_j++) {
    c2_e_j = c2_l_j - 1;
    c2_h_A = c2_s_data[c2_e_j] - c2_d31 * c2_s_data[c2_pglen + c2_e_j];
    c2_f_B = c2_md[0];
    c2_s_x = c2_h_A;
    c2_r_y = c2_f_B;
    c2_t_x = c2_s_x;
    c2_u_y = c2_r_y;
    c2_v_y = c2_t_x / c2_u_y;
    c2_s_data[c2_e_j] = c2_v_y;
  }

  for (c2_i91 = 0; c2_i91 < 6; c2_i91++) {
    c2_r_x[c2_i91] = c2_c_x[c2_i91];
  }

  c2_c_y_sizes[0] = c2_y_sizes[0];
  c2_c_y_sizes[1] = 6;
  c2_s_y = c2_c_y_sizes[0];
  c2_t_y = c2_c_y_sizes[1];
  c2_c_loop_ub = c2_y_sizes[0] * c2_y_sizes[1] - 1;
  for (c2_i92 = 0; c2_i92 <= c2_c_loop_ub; c2_i92++) {
    c2_c_y_data[c2_i92] = c2_y_data[c2_i92];
  }

  c2_c_pwchcore(chartInstance, c2_r_x, c2_c_y_data, c2_c_y_sizes, c2_yoffset,
                c2_s_data, c2_s_sizes, c2_dx, c2_dvdf_data, c2_dvdf_sizes,
                c2_output_data, c2_output_elems_sizes);
}

static void c2_c_chckxy(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_y_data[], int32_T c2_y_sizes[2])
{
  int32_T c2_ny;
  int32_T c2_b_y_sizes[2];
  int32_T c2_y;
  int32_T c2_b_y;
  int32_T c2_loop_ub;
  int32_T c2_i93;
  real_T c2_b_y_data[6];
  const mxArray *c2_c_y = NULL;
  static char_T c2_u[28] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'U', 'n', 's', 'u', 'p', 'p', 'o', 'r', 't', 'e', 'd',
    'N', 'a', 'N' };

  const mxArray *c2_d_y = NULL;
  static char_T c2_b_u[36] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'c', 'h', 'c',
    'k', 'x', 'y', ':', 'N', 'u', 'm', 'S', 'i', 't', 'e', 's', 'M', 'i', 's',
    'm', 'a', 't', 'c', 'h', 'V', 'a', 'l', 'u', 'e', 's' };

  int32_T c2_c_u;
  const mxArray *c2_e_y = NULL;
  int32_T c2_d_u;
  const mxArray *c2_f_y = NULL;
  c2_ny = c2_y_sizes[1];
  c2_b_y_sizes[0] = c2_y_sizes[0];
  c2_b_y_sizes[1] = 6;
  c2_y = c2_b_y_sizes[0];
  c2_b_y = c2_b_y_sizes[1];
  c2_loop_ub = c2_y_sizes[0] * c2_y_sizes[1] - 1;
  for (c2_i93 = 0; c2_i93 <= c2_loop_ub; c2_i93++) {
    c2_b_y_data[c2_i93] = c2_y_data[c2_i93];
  }

  if (!c2_anyIsNaN(chartInstance, c2_b_y_data, c2_b_y_sizes)) {
  } else {
    c2_c_y = NULL;
    sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 28),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c2_c_y));
  }

  if (c2_ny != 6) {
    if (c2_ny == 8) {
    } else {
      c2_d_y = NULL;
      sf_mex_assign(&c2_d_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 36),
                    false);
      c2_c_u = 6;
      c2_e_y = NULL;
      sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_c_u, 6, 0U, 0U, 0U, 0),
                    false);
      c2_d_u = c2_ny;
      c2_f_y = NULL;
      sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_d_u, 6, 0U, 0U, 0U, 0),
                    false);
      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                        sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
        1U, 3U, 14, c2_d_y, 14, c2_e_y, 14, c2_f_y));
    }
  }
}

static boolean_T c2_anyIsNaN(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_y_data[], int32_T c2_y_sizes[2])
{
  boolean_T c2_p;
  real_T c2_d5;
  int32_T c2_i94;
  int32_T c2_k;
  real_T c2_b_k;
  real_T c2_x;
  boolean_T c2_b;
  boolean_T exitg1;
  (void)chartInstance;
  c2_p = false;
  c2_d5 = (real_T)(c2_y_sizes[0] * c2_y_sizes[1]);
  c2_i94 = (int32_T)c2_d5;
  _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_d5, mxDOUBLE_CLASS, c2_i94);
  c2_k = 0;
  exitg1 = false;
  while ((exitg1 == false) && (c2_k <= c2_i94 - 1)) {
    c2_b_k = 1.0 + (real_T)c2_k;
    c2_x = c2_y_data[(int32_T)c2_b_k - 1];
    c2_b = muDoubleScalarIsNaN(c2_x);
    if (c2_b) {
      c2_p = true;
      exitg1 = true;
    } else {
      c2_k++;
    }
  }

  return c2_p;
}

static void c2_j_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_k_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_c_pwchcore(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x[6],
  real_T c2_y_data[], int32_T c2_y_sizes[2], int32_T c2_yoffset, real_T
  c2_s_data[], int32_T c2_s_sizes[2], real_T c2_dx[5], real_T c2_divdif_data[],
  int32_T c2_divdif_sizes[2], c2_sca5AIK8nC7yf5Qh9WghmhF *c2_pp_data,
  c2_sca5AIK8nC7yf5Qh9WghmhF_size *c2_pp_elems_sizes)
{
  int32_T c2_i95;
  int32_T c2_x_sizes[2];
  int32_T c2_b_x;
  int32_T c2_c_x;
  int32_T c2_loop_ub;
  int32_T c2_i96;
  int32_T c2_nyrows;
  real_T c2_x_data[12];
  int32_T c2_cpage;
  int32_T c2_i97;
  int32_T c2_i98;
  real_T c2_s[2];
  int32_T c2_szc[3];
  int32_T c2_i99;
  int32_T c2_j;
  int32_T c2_b_j;
  real_T c2_dxj;
  int32_T c2_joffset;
  int32_T c2_b_nyrows;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_i;
  int32_T c2_b_i;
  real_T c2_divdifij;
  real_T c2_A;
  real_T c2_B;
  real_T c2_d_x;
  real_T c2_y;
  real_T c2_e_x;
  real_T c2_b_y;
  real_T c2_dzzdx;
  real_T c2_b_A;
  real_T c2_b_B;
  real_T c2_f_x;
  real_T c2_c_y;
  real_T c2_g_x;
  real_T c2_d_y;
  real_T c2_dzdxdx;
  real_T c2_c_A;
  real_T c2_c_B;
  real_T c2_h_x;
  real_T c2_e_y;
  real_T c2_i_x;
  real_T c2_f_y;
  real_T c2_g_y;
  int32_T c2_c_b;
  int32_T c2_h_y;
  int32_T c2_d_b;
  int32_T c2_i_y;
  (void)c2_y_sizes;
  (void)c2_divdif_sizes;
  for (c2_i95 = 0; c2_i95 < 6; c2_i95++) {
    c2_pp_data->breaks[c2_i95] = c2_x[c2_i95];
  }

  c2_x_sizes[0] = c2_s_sizes[0];
  c2_x_sizes[1] = 6;
  c2_b_x = c2_x_sizes[0];
  c2_c_x = c2_x_sizes[1];
  c2_loop_ub = c2_s_sizes[0] * c2_s_sizes[1] - 1;
  for (c2_i96 = 0; c2_i96 <= c2_loop_ub; c2_i96++) {
    c2_x_data[c2_i96] = c2_s_data[c2_i96];
  }

  c2_nyrows = c2_x_sizes[0];
  c2_cpage = c2_nyrows * 5;
  for (c2_i97 = 0; c2_i97 < 2; c2_i97++) {
    c2_s[c2_i97] = (real_T)c2_s_sizes[c2_i97];
  }

  for (c2_i98 = 0; c2_i98 < 2; c2_i98++) {
    c2_szc[c2_i98] = (int32_T)c2_s[c2_i98];
  }

  c2_szc[2] = 4;
  c2_szc[1] = 5;
  for (c2_i99 = 0; c2_i99 < 3; c2_i99++) {
    c2_szc[c2_i99] = (int32_T)_SFD_NON_NEGATIVE_CHECK("", (real_T)c2_szc[c2_i99]);
  }

  c2_pp_elems_sizes->coefs[0] = c2_szc[0];
  c2_pp_elems_sizes->coefs[1] = 5;
  c2_pp_elems_sizes->coefs[2] = 4;
  for (c2_j = 1; c2_j < 6; c2_j++) {
    c2_b_j = c2_j - 1;
    c2_dxj = c2_dx[c2_b_j];
    c2_joffset = c2_b_j * c2_nyrows;
    c2_b_nyrows = c2_nyrows;
    c2_b = c2_b_nyrows;
    c2_b_b = c2_b;
    c2_overflow = ((!(1 > c2_b_b)) && (c2_b_b > 2147483646));
    if (c2_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_overflow);
    }

    for (c2_i = 1; c2_i <= c2_b_nyrows; c2_i++) {
      c2_b_i = c2_i - 1;
      c2_divdifij = c2_divdif_data[c2_joffset + c2_b_i];
      c2_A = c2_divdifij - c2_s_data[c2_joffset + c2_b_i];
      c2_B = c2_dxj;
      c2_d_x = c2_A;
      c2_y = c2_B;
      c2_e_x = c2_d_x;
      c2_b_y = c2_y;
      c2_dzzdx = c2_e_x / c2_b_y;
      c2_b_A = c2_s_data[(c2_joffset + c2_nyrows) + c2_b_i] - c2_divdifij;
      c2_b_B = c2_dxj;
      c2_f_x = c2_b_A;
      c2_c_y = c2_b_B;
      c2_g_x = c2_f_x;
      c2_d_y = c2_c_y;
      c2_dzdxdx = c2_g_x / c2_d_y;
      c2_c_A = c2_dzdxdx - c2_dzzdx;
      c2_c_B = c2_dxj;
      c2_h_x = c2_c_A;
      c2_e_y = c2_c_B;
      c2_i_x = c2_h_x;
      c2_f_y = c2_e_y;
      c2_g_y = c2_i_x / c2_f_y;
      c2_pp_data->coefs[c2_joffset + c2_b_i] = c2_g_y;
      c2_pp_data->coefs[(c2_cpage + c2_joffset) + c2_b_i] = 2.0 * c2_dzzdx -
        c2_dzdxdx;
      c2_c_b = c2_cpage;
      c2_h_y = (c2_c_b << 1) - 1;
      c2_pp_data->coefs[((c2_h_y + c2_joffset) + c2_b_i) + 1] =
        c2_s_data[c2_joffset + c2_b_i];
      c2_d_b = c2_cpage;
      c2_i_y = 3 * c2_d_b - 1;
      c2_pp_data->coefs[((c2_i_y + c2_joffset) + c2_b_i) + 1] = c2_y_data
        [(c2_yoffset + c2_joffset) + c2_b_i];
    }
  }
}

static void c2_d_ppval(SFc2_QuanInstanceStruct *chartInstance,
  c2_sca5AIK8nC7yf5Qh9WghmhF *c2_pp_data, c2_sca5AIK8nC7yf5Qh9WghmhF_size
  c2_pp_elems_sizes, real_T c2_x, real_T c2_v_data[], int32_T *c2_v_sizes)
{
  int32_T c2_x_sizes[3];
  int32_T c2_b_x;
  int32_T c2_c_x;
  int32_T c2_d_x;
  int32_T c2_loop_ub;
  int32_T c2_i100;
  int32_T c2_elementsPerPage;
  real_T c2_x_data[20];
  int32_T c2_coefStride;
  real_T c2_szv[2];
  int32_T c2_i101;
  int32_T c2_iv0;
  real_T c2_e_x;
  real_T c2_f_x;
  int32_T c2_b_coefStride;
  boolean_T c2_b;
  real_T c2_g_x;
  boolean_T c2_b_b;
  int32_T c2_i102;
  int32_T c2_b_elementsPerPage;
  int32_T c2_c_b;
  int32_T c2_i103;
  real_T c2_v;
  int32_T c2_d_b;
  int32_T c2_ip;
  real_T c2_pp[6];
  boolean_T c2_overflow;
  int32_T c2_icp;
  int32_T c2_b_ip;
  real_T c2_b_pp[6];
  real_T c2_xloc;
  real_T c2_b_xloc;
  int32_T c2_c_elementsPerPage;
  int32_T c2_j;
  int32_T c2_e_b;
  int32_T c2_ic;
  int32_T c2_f_b;
  boolean_T c2_b_overflow;
  int32_T c2_b_j;
  int32_T c2_b_ic;
  int32_T c2_c_j;
  int32_T c2_c_ic;
  int32_T c2_d_ic;
  int32_T c2_ic0;
  int32_T c2_d_elementsPerPage;
  int32_T c2_g_b;
  int32_T c2_h_b;
  boolean_T c2_c_overflow;
  int32_T c2_d_j;
  c2_x_sizes[0] = c2_pp_elems_sizes.coefs[0];
  c2_x_sizes[1] = 5;
  c2_x_sizes[2] = 4;
  c2_b_x = c2_x_sizes[0];
  c2_c_x = c2_x_sizes[1];
  c2_d_x = c2_x_sizes[2];
  c2_loop_ub = c2_pp_elems_sizes.coefs[0] * c2_pp_elems_sizes.coefs[1] *
    c2_pp_elems_sizes.coefs[2] - 1;
  for (c2_i100 = 0; c2_i100 <= c2_loop_ub; c2_i100++) {
    c2_x_data[c2_i100] = c2_pp_data->coefs[c2_i100];
  }

  c2_elementsPerPage = c2_x_sizes[0];
  c2_coefStride = c2_elementsPerPage * 5;
  c2_szv[0] = (real_T)c2_pp_elems_sizes.coefs[0];
  c2_szv[1] = 1.0;
  for (c2_i101 = 0; c2_i101 < 2; c2_i101++) {
    c2_szv[c2_i101] = _SFD_NON_NEGATIVE_CHECK("", c2_szv[c2_i101]);
  }

  *c2_v_sizes = (int32_T)c2_szv[0];
  if (c2_elementsPerPage == 1) {
    c2_e_x = c2_x;
    c2_b_coefStride = c2_coefStride;
    c2_g_x = c2_e_x;
    c2_b_b = muDoubleScalarIsNaN(c2_g_x);
    if (c2_b_b) {
      c2_v = c2_e_x;
    } else {
      for (c2_i103 = 0; c2_i103 < 6; c2_i103++) {
        c2_b_pp[c2_i103] = c2_pp_data->breaks[c2_i103];
      }

      c2_b_ip = c2_b_bsearch(chartInstance, c2_b_pp, c2_e_x) - 1;
      c2_b_xloc = c2_e_x - c2_pp_data->breaks[c2_b_ip];
      c2_v = c2_pp_data->coefs[c2_b_ip];
      for (c2_ic = 2; c2_ic < 5; c2_ic++) {
        c2_b_ic = c2_ic - 1;
        c2_v = c2_b_xloc * c2_v + c2_pp_data->coefs[c2_b_ip + c2_b_ic *
          c2_b_coefStride];
      }
    }

    c2_v_data[0] = c2_v;
  } else {
    c2_iv0 = -1;
    c2_f_x = c2_x;
    c2_b = muDoubleScalarIsNaN(c2_f_x);
    if (c2_b) {
      c2_b_elementsPerPage = c2_elementsPerPage;
      c2_c_b = c2_b_elementsPerPage;
      c2_d_b = c2_c_b;
      c2_overflow = ((!(1 > c2_d_b)) && (c2_d_b > 2147483646));
      if (c2_overflow) {
        c2_check_forloop_overflow_error(chartInstance, c2_overflow);
      }

      for (c2_j = 1; c2_j <= c2_b_elementsPerPage; c2_j++) {
        c2_b_j = c2_j;
        c2_v_data[c2_iv0 + c2_b_j] = c2_x;
      }
    } else {
      for (c2_i102 = 0; c2_i102 < 6; c2_i102++) {
        c2_pp[c2_i102] = c2_pp_data->breaks[c2_i102];
      }

      c2_ip = c2_b_bsearch(chartInstance, c2_pp, c2_x) - 1;
      c2_icp = c2_ip * c2_elementsPerPage;
      c2_xloc = c2_x - c2_pp_data->breaks[c2_ip];
      c2_c_elementsPerPage = c2_elementsPerPage;
      c2_e_b = c2_c_elementsPerPage;
      c2_f_b = c2_e_b;
      c2_b_overflow = ((!(1 > c2_f_b)) && (c2_f_b > 2147483646));
      if (c2_b_overflow) {
        c2_check_forloop_overflow_error(chartInstance, c2_b_overflow);
      }

      for (c2_c_j = 1; c2_c_j <= c2_c_elementsPerPage; c2_c_j++) {
        c2_b_j = c2_c_j;
        c2_v_data[c2_iv0 + c2_b_j] = c2_pp_data->coefs[(c2_icp + c2_b_j) - 1];
      }

      for (c2_c_ic = 2; c2_c_ic < 5; c2_c_ic++) {
        c2_d_ic = c2_c_ic - 1;
        c2_ic0 = (c2_icp + c2_d_ic * c2_coefStride) - 1;
        c2_d_elementsPerPage = c2_elementsPerPage;
        c2_g_b = c2_d_elementsPerPage;
        c2_h_b = c2_g_b;
        c2_c_overflow = ((!(1 > c2_h_b)) && (c2_h_b > 2147483646));
        if (c2_c_overflow) {
          c2_check_forloop_overflow_error(chartInstance, c2_c_overflow);
        }

        for (c2_d_j = 1; c2_d_j <= c2_d_elementsPerPage; c2_d_j++) {
          c2_b_j = c2_d_j;
          c2_v_data[c2_iv0 + c2_b_j] = c2_xloc * c2_v_data[c2_iv0 + c2_b_j] +
            c2_pp_data->coefs[c2_ic0 + c2_b_j];
        }
      }
    }
  }
}

static void c2_l_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static real_T c2_b_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_Xq;
  real_T c2_Yq;
  real_T c2_b_Xq;
  real_T c2_b_Yq;
  real_T c2_c_Xq;
  real_T c2_c_Yq;
  c2_Xq = c2_varargin_4;
  c2_Yq = c2_varargin_5;
  c2_b_Xq = c2_Xq;
  c2_b_Yq = c2_Yq;
  c2_c_Xq = c2_b_Xq;
  c2_c_Yq = c2_b_Yq;
  return c2_b_TensorInterp23(chartInstance, c2_c_Yq, c2_c_Xq);
}

static real_T c2_b_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_3[36], real_T c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_xxi;
  int32_T c2_i104;
  real_T c2_b_varargin_3[36];
  c2_s5aHNKucFmPykI7NAl3JfH c2_pp;
  int32_T c2_i105;
  real_T c2_vkj[6];
  real_T c2_varargout_1[6];
  int32_T c2_i;
  real_T c2_b_i;
  c2_syzabYTcbAiThYrkWkxvAFC c2_b_pp;
  real_T c2_b_vkj;
  c2_xxi = c2_varargin_5;
  for (c2_i104 = 0; c2_i104 < 36; c2_i104++) {
    c2_b_varargin_3[c2_i104] = c2_varargin_3[c2_i104];
  }

  c2_splinepp(chartInstance, c2_b_varargin_3, &c2_pp);
  for (c2_i105 = 0; c2_i105 < 6; c2_i105++) {
    c2_varargout_1[c2_i105] = 0.0;
  }

  c2_b_ppval(chartInstance, &c2_pp, c2_xxi, c2_vkj);
  for (c2_i = 0; c2_i < 6; c2_i++) {
    c2_b_i = 1.0 + (real_T)c2_i;
    c2_varargout_1[(int32_T)(1.0 + (c2_b_i - 1.0)) - 1] = c2_vkj[(int32_T)c2_b_i
      - 1];
  }

  c2_xxi = c2_varargin_4;
  c2_b_splinepp(chartInstance, c2_varargout_1, &c2_b_pp);
  c2_b_vkj = c2_c_ppval(chartInstance, &c2_b_pp, c2_xxi);
  return c2_b_vkj;
}

static real_T c2_b_TensorInterp23(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_wi;
  real_T c2_a;
  real_T c2_uxi;
  real_T c2_uxi_data[1];
  int32_T c2_uxi_sizes[2];
  int32_T c2_indx_data[1];
  int32_T c2_indx_sizes;
  int32_T c2_ipos;
  real_T c2_uyi;
  int32_T c2_b_ipos;
  int32_T c2_i106;
  real_T c2_ix;
  real_T c2_b_ix;
  real_T c2_b_a;
  real_T c2_varargin_3[36];
  static real_T c2_b_varargin_3[36] = { 12680.0, 12680.0, 12610.0, 12640.0,
    12390.0, 11680.0, 9150.0, 9150.0, 9312.0, 9839.0, 10176.0, 9848.0, 6200.0,
    6313.0, 6610.0, 7090.0, 7750.0, 8050.0, 3950.0, 4040.0, 4290.0, 4660.0,
    5320.0, 6100.0, 2450.0, 2470.0, 2600.0, 2840.0, 3250.0, 3800.0, 1400.0,
    1400.0, 1560.0, 1660.0, 1930.0, 2310.0 };

  real_T c2_uyi_data[1];
  int32_T c2_uyi_sizes[2];
  int32_T c2_c_ipos;
  int32_T c2_d_ipos;
  real_T c2_iy;
  real_T c2_b_iy;
  real_T c2_outsize[2];
  real_T c2_nxxi;
  int32_T c2_i107;
  real_T c2_c_varargin_3[36];
  c2_s5aHNKucFmPykI7NAl3JfH c2_pp;
  real_T c2_sv[2];
  int32_T c2_i108;
  int32_T c2_varargout_1_sizes[2];
  int32_T c2_varargout_1;
  int32_T c2_b_varargout_1;
  int32_T c2_loop_ub;
  int32_T c2_i109;
  real_T c2_b_nxxi;
  real_T c2_varargout_1_data[6];
  int32_T c2_i110;
  int32_T c2_j;
  real_T c2_b_j;
  c2_s5aHNKucFmPykI7NAl3JfH c2_b_pp;
  int32_T c2_b_uyi;
  int32_T c2_c_uyi[1];
  int32_T c2_d_uyi;
  real_T c2_vkj[6];
  int32_T c2_b_loop_ub;
  int32_T c2_i;
  int32_T c2_i111;
  real_T c2_b_i;
  c2_sca5AIK8nC7yf5Qh9WghmhF c2_ppk_data;
  c2_sca5AIK8nC7yf5Qh9WghmhF_size c2_ppk_elems_sizes;
  int32_T c2_i112;
  int32_T c2_uwi_sizes[2];
  int32_T c2_uwi;
  int32_T c2_b_uwi;
  int32_T c2_c_loop_ub;
  int32_T c2_i113;
  real_T c2_c_nxxi;
  real_T c2_uwi_data[1];
  int32_T c2_i114;
  int32_T c2_c_j;
  c2_sca5AIK8nC7yf5Qh9WghmhF_size c2_b_ppk_elems_sizes;
  c2_sca5AIK8nC7yf5Qh9WghmhF c2_b_ppk_data;
  int32_T c2_e_uyi[1];
  real_T c2_vkj_data[1];
  int32_T c2_vkj_sizes;
  real_T c2_d6;
  int32_T c2_i115;
  int32_T c2_c_i;
  if (c2_isplaid(chartInstance, c2_varargin_4, c2_varargin_5)) {
    c2_uxi = c2_varargin_4;
    c2_uyi = c2_varargin_5;
    for (c2_i106 = 0; c2_i106 < 36; c2_i106++) {
      c2_varargin_3[c2_i106] = c2_b_varargin_3[c2_i106];
    }

    c2_wi = c2_b_TensorGriddedInterp(chartInstance, c2_varargin_3, c2_uxi,
      c2_uyi);
  } else {
    c2_a = c2_varargin_4;
    c2_unique_vector(chartInstance, c2_a, c2_uxi_data, c2_uxi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_ipos);
    c2_b_ipos = c2_ipos;
    c2_ix = (real_T)c2_b_ipos;
    c2_b_ix = c2_ix;
    c2_b_a = c2_varargin_5;
    c2_unique_vector(chartInstance, c2_b_a, c2_uyi_data, c2_uyi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_c_ipos);
    c2_d_ipos = c2_c_ipos;
    c2_iy = (real_T)c2_d_ipos;
    c2_b_iy = c2_iy;
    c2_outsize[0] = (real_T)c2_uxi_sizes[1];
    c2_outsize[1] = (real_T)c2_uyi_sizes[1];
    c2_nxxi = (real_T)c2_uyi_sizes[1];
    for (c2_i107 = 0; c2_i107 < 36; c2_i107++) {
      c2_c_varargin_3[c2_i107] = c2_b_varargin_3[c2_i107];
    }

    c2_splinepp(chartInstance, c2_c_varargin_3, &c2_pp);
    c2_sv[0] = (real_T)c2_uyi_sizes[1];
    c2_sv[1] = 6.0;
    for (c2_i108 = 0; c2_i108 < 2; c2_i108++) {
      c2_sv[c2_i108] = _SFD_NON_NEGATIVE_CHECK("", c2_sv[c2_i108]);
    }

    c2_varargout_1_sizes[0] = (int32_T)c2_sv[0];
    c2_varargout_1_sizes[1] = 6;
    c2_varargout_1 = c2_varargout_1_sizes[0];
    c2_b_varargout_1 = c2_varargout_1_sizes[1];
    c2_loop_ub = (int32_T)c2_sv[0] * (int32_T)c2_sv[1] - 1;
    for (c2_i109 = 0; c2_i109 <= c2_loop_ub; c2_i109++) {
      c2_varargout_1_data[c2_i109] = 0.0;
    }

    c2_b_nxxi = c2_nxxi;
    c2_i110 = (int32_T)c2_b_nxxi;
    _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_b_nxxi, mxDOUBLE_CLASS, c2_i110);
    for (c2_j = 0; c2_j < c2_i110; c2_j++) {
      c2_b_j = 1.0 + (real_T)c2_j;
      c2_b_pp = c2_pp;
      c2_c_uyi[0] = c2_uyi_sizes[1];
      c2_b_ppval(chartInstance, &c2_b_pp, c2_uyi_data[(int32_T)c2_b_j - 1],
                 c2_vkj);
      for (c2_i = 0; c2_i < 6; c2_i++) {
        c2_b_i = 1.0 + (real_T)c2_i;
        c2_varargout_1_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
          c2_vkj[(int32_T)c2_b_i - 1];
      }
    }

    c2_uyi_sizes[0] = 1;
    c2_uyi_sizes[1] = c2_uxi_sizes[1];
    c2_b_uyi = c2_uyi_sizes[0];
    c2_d_uyi = c2_uyi_sizes[1];
    c2_b_loop_ub = c2_uxi_sizes[0] * c2_uxi_sizes[1] - 1;
    for (c2_i111 = 0; c2_i111 <= c2_b_loop_ub; c2_i111++) {
      c2_uyi_data[c2_i111] = c2_uxi_data[c2_i111];
    }

    c2_nxxi = (real_T)c2_uyi_sizes[1];
    c2_spline(chartInstance, c2_varargout_1_data, c2_varargout_1_sizes,
              &c2_ppk_data, &c2_ppk_elems_sizes);
    for (c2_i112 = 0; c2_i112 < 2; c2_i112++) {
      c2_outsize[c2_i112] = _SFD_NON_NEGATIVE_CHECK("", c2_outsize[c2_i112]);
    }

    c2_uwi_sizes[0] = (int32_T)c2_outsize[0];
    c2_uwi_sizes[1] = (int32_T)c2_outsize[1];
    c2_uwi = c2_uwi_sizes[0];
    c2_b_uwi = c2_uwi_sizes[1];
    c2_c_loop_ub = (int32_T)c2_outsize[0] * (int32_T)c2_outsize[1] - 1;
    for (c2_i113 = 0; c2_i113 <= c2_c_loop_ub; c2_i113++) {
      c2_uwi_data[c2_i113] = 0.0;
    }

    c2_c_nxxi = c2_nxxi;
    c2_i114 = (int32_T)c2_c_nxxi;
    _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_c_nxxi, mxDOUBLE_CLASS, c2_i114);
    for (c2_c_j = 0; c2_c_j < c2_i114; c2_c_j++) {
      c2_b_j = 1.0 + (real_T)c2_c_j;
      c2_b_ppk_elems_sizes = c2_ppk_elems_sizes;
      c2_b_ppk_data = c2_ppk_data;
      c2_e_uyi[0] = c2_uyi_sizes[1];
      c2_d_ppval(chartInstance, &c2_b_ppk_data, c2_b_ppk_elems_sizes,
                 c2_uyi_data[(int32_T)c2_b_j - 1], c2_vkj_data, &c2_vkj_sizes);
      c2_d6 = (real_T)c2_vkj_sizes;
      c2_i115 = (int32_T)c2_d6;
      _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_d6, mxDOUBLE_CLASS, c2_i115);
      for (c2_c_i = 0; c2_c_i < c2_i115; c2_c_i++) {
        c2_b_i = 1.0 + (real_T)c2_c_i;
        c2_uwi_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
          c2_vkj_data[(int32_T)c2_b_i - 1];
      }
    }

    c2_wi = c2_uwi_data[((int32_T)c2_b_ix + c2_uwi_sizes[0] * ((int32_T)c2_b_iy
      - 1)) - 1];
  }

  return c2_wi;
}

static real_T c2_c_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_Xq;
  real_T c2_Yq;
  real_T c2_b_Xq;
  real_T c2_b_Yq;
  real_T c2_c_Xq;
  real_T c2_c_Yq;
  c2_Xq = c2_varargin_4;
  c2_Yq = c2_varargin_5;
  c2_b_Xq = c2_Xq;
  c2_b_Yq = c2_Yq;
  c2_c_Xq = c2_b_Xq;
  c2_c_Yq = c2_b_Yq;
  return c2_c_TensorInterp23(chartInstance, c2_c_Yq, c2_c_Xq);
}

static real_T c2_c_TensorInterp23(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_wi;
  real_T c2_a;
  real_T c2_uxi;
  real_T c2_uxi_data[1];
  int32_T c2_uxi_sizes[2];
  int32_T c2_indx_data[1];
  int32_T c2_indx_sizes;
  int32_T c2_ipos;
  real_T c2_uyi;
  int32_T c2_b_ipos;
  int32_T c2_i116;
  real_T c2_ix;
  real_T c2_b_ix;
  real_T c2_b_a;
  real_T c2_varargin_3[36];
  static real_T c2_b_varargin_3[36] = { 20000.0, 21420.0, 22700.0, 24240.0,
    26070.0, 28886.0, 15000.0, 15700.0, 16860.0, 18910.0, 21075.0, 23319.0,
    10800.0, 11225.0, 12250.0, 13760.0, 15975.0, 18300.0, 7000.0, 7323.0, 8154.0,
    9285.0, 11115.0, 13484.0, 4000.0, 4435.0, 5000.0, 5700.0, 6860.0, 8642.0,
    2500.0, 2600.0, 2835.0, 3215.0, 3950.0, 5057.0 };

  real_T c2_uyi_data[1];
  int32_T c2_uyi_sizes[2];
  int32_T c2_c_ipos;
  int32_T c2_d_ipos;
  real_T c2_iy;
  real_T c2_b_iy;
  real_T c2_outsize[2];
  real_T c2_nxxi;
  int32_T c2_i117;
  real_T c2_c_varargin_3[36];
  c2_s5aHNKucFmPykI7NAl3JfH c2_pp;
  real_T c2_sv[2];
  int32_T c2_i118;
  int32_T c2_varargout_1_sizes[2];
  int32_T c2_varargout_1;
  int32_T c2_b_varargout_1;
  int32_T c2_loop_ub;
  int32_T c2_i119;
  real_T c2_b_nxxi;
  real_T c2_varargout_1_data[6];
  int32_T c2_i120;
  int32_T c2_j;
  real_T c2_b_j;
  c2_s5aHNKucFmPykI7NAl3JfH c2_b_pp;
  int32_T c2_b_uyi;
  int32_T c2_c_uyi[1];
  int32_T c2_d_uyi;
  real_T c2_vkj[6];
  int32_T c2_b_loop_ub;
  int32_T c2_i;
  int32_T c2_i121;
  real_T c2_b_i;
  c2_sca5AIK8nC7yf5Qh9WghmhF c2_ppk_data;
  c2_sca5AIK8nC7yf5Qh9WghmhF_size c2_ppk_elems_sizes;
  int32_T c2_i122;
  int32_T c2_uwi_sizes[2];
  int32_T c2_uwi;
  int32_T c2_b_uwi;
  int32_T c2_c_loop_ub;
  int32_T c2_i123;
  real_T c2_c_nxxi;
  real_T c2_uwi_data[1];
  int32_T c2_i124;
  int32_T c2_c_j;
  c2_sca5AIK8nC7yf5Qh9WghmhF_size c2_b_ppk_elems_sizes;
  c2_sca5AIK8nC7yf5Qh9WghmhF c2_b_ppk_data;
  int32_T c2_e_uyi[1];
  real_T c2_vkj_data[1];
  int32_T c2_vkj_sizes;
  real_T c2_d7;
  int32_T c2_i125;
  int32_T c2_c_i;
  if (c2_isplaid(chartInstance, c2_varargin_4, c2_varargin_5)) {
    c2_uxi = c2_varargin_4;
    c2_uyi = c2_varargin_5;
    for (c2_i116 = 0; c2_i116 < 36; c2_i116++) {
      c2_varargin_3[c2_i116] = c2_b_varargin_3[c2_i116];
    }

    c2_wi = c2_b_TensorGriddedInterp(chartInstance, c2_varargin_3, c2_uxi,
      c2_uyi);
  } else {
    c2_a = c2_varargin_4;
    c2_unique_vector(chartInstance, c2_a, c2_uxi_data, c2_uxi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_ipos);
    c2_b_ipos = c2_ipos;
    c2_ix = (real_T)c2_b_ipos;
    c2_b_ix = c2_ix;
    c2_b_a = c2_varargin_5;
    c2_unique_vector(chartInstance, c2_b_a, c2_uyi_data, c2_uyi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_c_ipos);
    c2_d_ipos = c2_c_ipos;
    c2_iy = (real_T)c2_d_ipos;
    c2_b_iy = c2_iy;
    c2_outsize[0] = (real_T)c2_uxi_sizes[1];
    c2_outsize[1] = (real_T)c2_uyi_sizes[1];
    c2_nxxi = (real_T)c2_uyi_sizes[1];
    for (c2_i117 = 0; c2_i117 < 36; c2_i117++) {
      c2_c_varargin_3[c2_i117] = c2_b_varargin_3[c2_i117];
    }

    c2_splinepp(chartInstance, c2_c_varargin_3, &c2_pp);
    c2_sv[0] = (real_T)c2_uyi_sizes[1];
    c2_sv[1] = 6.0;
    for (c2_i118 = 0; c2_i118 < 2; c2_i118++) {
      c2_sv[c2_i118] = _SFD_NON_NEGATIVE_CHECK("", c2_sv[c2_i118]);
    }

    c2_varargout_1_sizes[0] = (int32_T)c2_sv[0];
    c2_varargout_1_sizes[1] = 6;
    c2_varargout_1 = c2_varargout_1_sizes[0];
    c2_b_varargout_1 = c2_varargout_1_sizes[1];
    c2_loop_ub = (int32_T)c2_sv[0] * (int32_T)c2_sv[1] - 1;
    for (c2_i119 = 0; c2_i119 <= c2_loop_ub; c2_i119++) {
      c2_varargout_1_data[c2_i119] = 0.0;
    }

    c2_b_nxxi = c2_nxxi;
    c2_i120 = (int32_T)c2_b_nxxi;
    _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_b_nxxi, mxDOUBLE_CLASS, c2_i120);
    for (c2_j = 0; c2_j < c2_i120; c2_j++) {
      c2_b_j = 1.0 + (real_T)c2_j;
      c2_b_pp = c2_pp;
      c2_c_uyi[0] = c2_uyi_sizes[1];
      c2_b_ppval(chartInstance, &c2_b_pp, c2_uyi_data[(int32_T)c2_b_j - 1],
                 c2_vkj);
      for (c2_i = 0; c2_i < 6; c2_i++) {
        c2_b_i = 1.0 + (real_T)c2_i;
        c2_varargout_1_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
          c2_vkj[(int32_T)c2_b_i - 1];
      }
    }

    c2_uyi_sizes[0] = 1;
    c2_uyi_sizes[1] = c2_uxi_sizes[1];
    c2_b_uyi = c2_uyi_sizes[0];
    c2_d_uyi = c2_uyi_sizes[1];
    c2_b_loop_ub = c2_uxi_sizes[0] * c2_uxi_sizes[1] - 1;
    for (c2_i121 = 0; c2_i121 <= c2_b_loop_ub; c2_i121++) {
      c2_uyi_data[c2_i121] = c2_uxi_data[c2_i121];
    }

    c2_nxxi = (real_T)c2_uyi_sizes[1];
    c2_spline(chartInstance, c2_varargout_1_data, c2_varargout_1_sizes,
              &c2_ppk_data, &c2_ppk_elems_sizes);
    for (c2_i122 = 0; c2_i122 < 2; c2_i122++) {
      c2_outsize[c2_i122] = _SFD_NON_NEGATIVE_CHECK("", c2_outsize[c2_i122]);
    }

    c2_uwi_sizes[0] = (int32_T)c2_outsize[0];
    c2_uwi_sizes[1] = (int32_T)c2_outsize[1];
    c2_uwi = c2_uwi_sizes[0];
    c2_b_uwi = c2_uwi_sizes[1];
    c2_c_loop_ub = (int32_T)c2_outsize[0] * (int32_T)c2_outsize[1] - 1;
    for (c2_i123 = 0; c2_i123 <= c2_c_loop_ub; c2_i123++) {
      c2_uwi_data[c2_i123] = 0.0;
    }

    c2_c_nxxi = c2_nxxi;
    c2_i124 = (int32_T)c2_c_nxxi;
    _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_c_nxxi, mxDOUBLE_CLASS, c2_i124);
    for (c2_c_j = 0; c2_c_j < c2_i124; c2_c_j++) {
      c2_b_j = 1.0 + (real_T)c2_c_j;
      c2_b_ppk_elems_sizes = c2_ppk_elems_sizes;
      c2_b_ppk_data = c2_ppk_data;
      c2_e_uyi[0] = c2_uyi_sizes[1];
      c2_d_ppval(chartInstance, &c2_b_ppk_data, c2_b_ppk_elems_sizes,
                 c2_uyi_data[(int32_T)c2_b_j - 1], c2_vkj_data, &c2_vkj_sizes);
      c2_d7 = (real_T)c2_vkj_sizes;
      c2_i125 = (int32_T)c2_d7;
      _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_d7, mxDOUBLE_CLASS, c2_i125);
      for (c2_c_i = 0; c2_c_i < c2_i125; c2_c_i++) {
        c2_b_i = 1.0 + (real_T)c2_c_i;
        c2_uwi_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
          c2_vkj_data[(int32_T)c2_b_i - 1];
      }
    }

    c2_wi = c2_uwi_data[((int32_T)c2_b_ix + c2_uwi_sizes[0] * ((int32_T)c2_b_iy
      - 1)) - 1];
  }

  return c2_wi;
}

static real_T c2_j_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_1[3], real_T c2_varargin_2[3], real_T c2_varargin_3)
{
  real_T c2_Vq;
  int32_T c2_i126;
  real_T c2_xi;
  real_T c2_y[3];
  int32_T c2_i127;
  int32_T c2_k;
  real_T c2_x[3];
  int32_T c2_b_k;
  real_T c2_xtmp;
  real_T c2_b_x;
  int32_T c2_c_k;
  boolean_T c2_b;
  real_T c2_b_xtmp;
  real_T c2_b_xi;
  real_T c2_minx;
  real_T c2_maxx;
  real_T c2_c_xi;
  real_T c2_b_minx;
  real_T c2_b_maxx;
  real_T c2_xik;
  real_T c2_c_x;
  boolean_T c2_b_b;
  real_T c2_d_xi;
  int32_T c2_low_i;
  int32_T c2_low_ip1;
  int32_T c2_high_i;
  int32_T c2_n;
  int32_T c2_b_low_i;
  real_T c2_xn;
  int32_T c2_b_high_i;
  real_T c2_xnp1;
  int32_T c2_mid_i;
  real_T c2_A;
  real_T c2_B;
  real_T c2_d_x;
  real_T c2_b_y;
  real_T c2_e_x;
  real_T c2_c_y;
  real_T c2_r;
  real_T c2_onemr;
  real_T c2_y1;
  real_T c2_y2;
  int32_T exitg1;
  for (c2_i126 = 0; c2_i126 < 3; c2_i126++) {
    c2_y[c2_i126] = c2_varargin_2[c2_i126];
  }

  c2_xi = c2_varargin_3;
  for (c2_i127 = 0; c2_i127 < 3; c2_i127++) {
    c2_x[c2_i127] = c2_varargin_1[c2_i127];
  }

  c2_k = 1;
  do {
    exitg1 = 0;
    if (c2_k < 4) {
      c2_b_k = c2_k - 1;
      c2_b_x = c2_x[c2_b_k];
      c2_b = muDoubleScalarIsNaN(c2_b_x);
      if (c2_b) {
        c2_d_error(chartInstance);
        exitg1 = 1;
      } else {
        c2_k++;
      }
    } else {
      if (c2_x[1] < c2_x[0]) {
        c2_xtmp = c2_x[0];
        c2_x[0] = c2_x[2];
        c2_x[2] = c2_xtmp;
        c2_b_xtmp = c2_y[0];
        c2_y[0] = c2_y[2];
        c2_y[2] = c2_b_xtmp;
      }

      for (c2_c_k = 2; c2_c_k < 4; c2_c_k++) {
        c2_b_k = c2_c_k - 2;
        if (c2_x[c2_b_k + 1] <= c2_x[c2_b_k]) {
          c2_e_error(chartInstance);
        }
      }

      c2_b_xi = c2_xi;
      c2_Vq = rtNaN;
      c2_minx = c2_x[0];
      c2_maxx = c2_x[2];
      c2_c_xi = c2_b_xi;
      c2_b_minx = c2_minx;
      c2_b_maxx = c2_maxx;
      c2_xik = c2_c_xi;
      c2_c_x = c2_xik;
      c2_b_b = muDoubleScalarIsNaN(c2_c_x);
      if (c2_b_b) {
        c2_Vq = rtNaN;
      } else if (c2_xik > c2_b_maxx) {
      } else if (c2_xik < c2_b_minx) {
      } else {
        c2_d_xi = c2_xik;
        c2_low_i = 1;
        c2_low_ip1 = 1;
        c2_high_i = 3;
        while (c2_high_i > c2_low_ip1 + 1) {
          c2_b_low_i = c2_low_i;
          c2_b_high_i = c2_high_i;
          c2_mid_i = (c2_b_low_i + c2_b_high_i) >> 1;
          if (c2_d_xi >= c2_x[c2_mid_i - 1]) {
            c2_low_i = c2_mid_i;
            c2_low_ip1 = c2_low_i;
          } else {
            c2_high_i = c2_mid_i;
          }
        }

        c2_n = c2_low_i;
        c2_xn = c2_x[c2_n - 1];
        c2_xnp1 = c2_x[c2_n];
        c2_A = c2_xik - c2_xn;
        c2_B = c2_xnp1 - c2_xn;
        c2_d_x = c2_A;
        c2_b_y = c2_B;
        c2_e_x = c2_d_x;
        c2_c_y = c2_b_y;
        c2_r = c2_e_x / c2_c_y;
        c2_onemr = 1.0 - c2_r;
        if (c2_r == 0.0) {
          c2_y1 = c2_y[c2_n - 1];
          c2_Vq = c2_y1;
        } else if (c2_r == 1.0) {
          c2_y2 = c2_y[c2_n];
          c2_Vq = c2_y2;
        } else {
          c2_y1 = c2_y[c2_n - 1];
          c2_y2 = c2_y[c2_n];
          if (c2_y1 == c2_y2) {
            c2_Vq = c2_y1;
          } else {
            c2_Vq = c2_onemr * c2_y1 + c2_r * c2_y2;
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return c2_Vq;
}

static void c2_m_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_c_StringToMethodID(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static real_T c2_d_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_Vq;
  real_T c2_Xq;
  real_T c2_Yq;
  real_T c2_b_Xq;
  real_T c2_b_Yq;
  real_T c2_c_Xq;
  real_T c2_c_Yq;
  real_T c2_b_varargin_4;
  real_T c2_b_varargin_5;
  real_T c2_a;
  real_T c2_uxi;
  real_T c2_uxi_data[1];
  int32_T c2_uxi_sizes[2];
  int32_T c2_indx_data[1];
  int32_T c2_indx_sizes;
  int32_T c2_ipos;
  real_T c2_uyi;
  int32_T c2_b_ipos;
  real_T c2_ix;
  real_T c2_b_ix;
  real_T c2_b_a;
  real_T c2_uyi_data[1];
  int32_T c2_uyi_sizes[2];
  int32_T c2_c_ipos;
  int32_T c2_d_ipos;
  real_T c2_iy;
  real_T c2_b_iy;
  real_T c2_uwi_data[1];
  int32_T c2_uwi_sizes[2];
  c2_Xq = c2_varargin_4;
  c2_Yq = c2_varargin_5;
  c2_b_Xq = c2_Xq;
  c2_b_Yq = c2_Yq;
  c2_c_Xq = c2_b_Xq;
  c2_c_Yq = c2_b_Yq;
  c2_b_varargin_4 = c2_c_Yq;
  c2_b_varargin_5 = c2_c_Xq;
  if (c2_isplaid(chartInstance, c2_b_varargin_4, c2_b_varargin_5)) {
    c2_uxi = c2_b_varargin_4;
    c2_uyi = c2_b_varargin_5;
    c2_Vq = c2_c_TensorGriddedInterp(chartInstance, c2_uxi, c2_uyi);
  } else {
    c2_a = c2_b_varargin_4;
    c2_unique_vector(chartInstance, c2_a, c2_uxi_data, c2_uxi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_ipos);
    c2_b_ipos = c2_ipos;
    c2_ix = (real_T)c2_b_ipos;
    c2_b_ix = c2_ix;
    c2_b_a = c2_b_varargin_5;
    c2_unique_vector(chartInstance, c2_b_a, c2_uyi_data, c2_uyi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_c_ipos);
    c2_d_ipos = c2_c_ipos;
    c2_iy = (real_T)c2_d_ipos;
    c2_b_iy = c2_iy;
    c2_d_TensorGriddedInterp(chartInstance, c2_uxi_data, c2_uxi_sizes,
      c2_uyi_data, c2_uyi_sizes, c2_uwi_data, c2_uwi_sizes);
    c2_Vq = c2_uwi_data[((int32_T)c2_b_ix + c2_uwi_sizes[0] * ((int32_T)c2_b_iy
      - 1)) - 1];
  }

  return c2_Vq;
}

static void c2_b_interp2_validate(SFc2_QuanInstanceStruct *chartInstance,
  uint8_T c2_METHOD)
{
  (void)chartInstance;
  (void)c2_METHOD;
}

static void c2_n_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_o_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static real_T c2_c_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_xxi;
  int32_T c2_i128;
  real_T c2_varargin_3[60];
  c2_samQD8b77pHdwkX3iY74akB c2_ppk;
  static real_T c2_b_varargin_3[60] = { -0.099, -0.081, -0.081, -0.063, -0.025,
    0.044, 0.097, 0.113, 0.145, 0.167, 0.174, 0.166, -0.048, -0.038, -0.04,
    -0.021, 0.016, 0.083, 0.127, 0.137, 0.162, 0.177, 0.179, 0.167, -0.022,
    -0.02, -0.021, -0.004, 0.032, 0.094, 0.128, 0.13, 0.154, 0.161, 0.155, 0.138,
    -0.04, -0.038, -0.039, -0.025, 0.006, 0.062, 0.087, 0.085, 0.1, 0.11, 0.104,
    0.091, -0.083, -0.073, -0.076, -0.072, -0.046, 0.012, 0.024, 0.025, 0.043,
    0.053, 0.047, 0.04 };

  int32_T c2_i129;
  real_T c2_vkj[12];
  real_T c2_varargout_1[12];
  int32_T c2_i;
  real_T c2_b_i;
  c2_sSYDZjSrzjRAmypAou1DUJH c2_b_ppk;
  real_T c2_b_vkj;
  c2_xxi = c2_varargin_5;
  for (c2_i128 = 0; c2_i128 < 60; c2_i128++) {
    c2_varargin_3[c2_i128] = c2_b_varargin_3[c2_i128];
  }

  c2_b_spline(chartInstance, c2_varargin_3, &c2_ppk);
  for (c2_i129 = 0; c2_i129 < 12; c2_i129++) {
    c2_varargout_1[c2_i129] = 0.0;
  }

  c2_e_ppval(chartInstance, &c2_ppk, c2_xxi, c2_vkj);
  for (c2_i = 0; c2_i < 12; c2_i++) {
    c2_b_i = 1.0 + (real_T)c2_i;
    c2_varargout_1[(int32_T)(1.0 + (c2_b_i - 1.0)) - 1] = c2_vkj[(int32_T)c2_b_i
      - 1];
  }

  c2_xxi = c2_varargin_4;
  c2_c_spline(chartInstance, c2_varargout_1, &c2_b_ppk);
  c2_b_vkj = c2_ppval(chartInstance, &c2_b_ppk, c2_xxi);
  return c2_b_vkj;
}

static void c2_b_spline(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[60],
  c2_samQD8b77pHdwkX3iY74akB *c2_output)
{
  int32_T c2_i130;
  real_T c2_b_y[60];
  for (c2_i130 = 0; c2_i130 < 60; c2_i130++) {
    c2_b_y[c2_i130] = c2_y[c2_i130];
  }

  c2_c_splinepp(chartInstance, c2_b_y, c2_output);
}

static void c2_c_splinepp(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[60],
  c2_samQD8b77pHdwkX3iY74akB *c2_pp)
{
  int32_T c2_i131;
  real_T c2_b_y[60];
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_dx[4];
  int32_T c2_dpg;
  real_T c2_d1;
  int32_T c2_pg;
  real_T c2_d2;
  int32_T c2_pgp1;
  int32_T c2_j;
  int32_T c2_pgm1;
  int32_T c2_b_j;
  int32_T c2_c_j;
  int32_T c2_d_j;
  real_T c2_A;
  real_T c2_dvdf[48];
  real_T c2_b_A;
  int32_T c2_e_j;
  real_T c2_x;
  real_T c2_B;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_c_y;
  real_T c2_s[60];
  real_T c2_d_y;
  real_T c2_md[5];
  real_T c2_d_x;
  real_T c2_c_A;
  real_T c2_e_y;
  int32_T c2_d_k;
  real_T c2_e_x;
  real_T c2_f_y;
  real_T c2_f_x;
  real_T c2_g_y;
  real_T c2_d_A;
  real_T c2_b_B;
  real_T c2_g_x;
  real_T c2_h_y;
  real_T c2_h_x;
  real_T c2_i_y;
  real_T c2_r;
  int32_T c2_f_j;
  int32_T c2_e_k;
  real_T c2_c_B;
  real_T c2_j_y;
  real_T c2_e_A;
  real_T c2_k_y;
  real_T c2_d_B;
  real_T c2_i_x;
  real_T c2_l_y;
  int32_T c2_g_j;
  real_T c2_j_x;
  real_T c2_m_y;
  int32_T c2_h_j;
  int32_T c2_f_k;
  int32_T c2_i_j;
  real_T c2_f_A;
  real_T c2_e_B;
  int32_T c2_j_j;
  real_T c2_k_x;
  real_T c2_n_y;
  real_T c2_l_x;
  int32_T c2_i132;
  real_T c2_o_y;
  real_T c2_g_A;
  int32_T c2_k_j;
  real_T c2_p_y;
  real_T c2_f_B;
  int32_T c2_i133;
  real_T c2_dv26[5];
  real_T c2_m_x;
  real_T c2_q_y;
  real_T c2_n_x;
  real_T c2_h_A;
  real_T c2_r_y[60];
  real_T c2_s_y;
  real_T c2_g_B;
  real_T c2_t_y;
  real_T c2_o_x;
  real_T c2_u_y;
  real_T c2_p_x;
  real_T c2_v_y;
  real_T c2_w_y;
  for (c2_i131 = 0; c2_i131 < 60; c2_i131++) {
    c2_b_y[c2_i131] = c2_y[c2_i131];
  }

  c2_d_chckxy(chartInstance, c2_b_y);
  for (c2_k = 1; c2_k < 5; c2_k++) {
    c2_c_k = c2_k - 1;
    c2_dx[c2_c_k] = (-24.0 + 12.0 * (real_T)(c2_c_k + 1)) - (-24.0 + 12.0 *
      (real_T)c2_c_k);
    c2_dpg = c2_c_k * 12;
    c2_pg = c2_dpg;
    c2_pgp1 = (c2_c_k + 1) * 12;
    for (c2_b_j = 1; c2_b_j < 13; c2_b_j++) {
      c2_c_j = c2_b_j - 1;
      c2_b_A = c2_y[c2_pgp1 + c2_c_j] - c2_y[c2_pg + c2_c_j];
      c2_B = c2_dx[c2_c_k];
      c2_c_x = c2_b_A;
      c2_d_y = c2_B;
      c2_d_x = c2_c_x;
      c2_e_y = c2_d_y;
      c2_f_y = c2_d_x / c2_e_y;
      c2_dvdf[c2_dpg + c2_c_j] = c2_f_y;
    }
  }

  for (c2_b_k = 2; c2_b_k < 5; c2_b_k++) {
    c2_c_k = c2_b_k - 2;
    c2_pg = (c2_c_k + 1) * 12;
    c2_pgm1 = c2_c_k * 12;
    c2_d1 = c2_dx[c2_c_k + 1];
    c2_d2 = c2_dx[c2_c_k];
    for (c2_d_j = 1; c2_d_j < 13; c2_d_j++) {
      c2_c_j = c2_d_j - 1;
      c2_s[c2_pg + c2_c_j] = 3.0 * (c2_d1 * c2_dvdf[c2_pgm1 + c2_c_j] + c2_d2 *
        c2_dvdf[c2_pg + c2_c_j]);
    }
  }

  c2_d1 = c2_dx[0];
  c2_d2 = c2_dx[1];
  for (c2_j = 1; c2_j < 13; c2_j++) {
    c2_c_j = c2_j - 1;
    c2_A = (c2_d1 + 48.0) * c2_d2 * c2_dvdf[c2_c_j] + c2_d1 * c2_d1 *
      c2_dvdf[c2_c_j + 12];
    c2_x = c2_A;
    c2_b_x = c2_x;
    c2_c_y = c2_b_x / 24.0;
    c2_s[c2_c_j] = c2_c_y;
  }

  c2_d1 = c2_dx[3];
  c2_d2 = c2_dx[2];
  for (c2_e_j = 1; c2_e_j < 13; c2_e_j++) {
    c2_c_j = c2_e_j + 23;
    c2_c_A = (c2_d1 + 48.0) * c2_d2 * c2_dvdf[c2_c_j + 12] + c2_d1 * c2_d1 *
      c2_dvdf[c2_c_j];
    c2_e_x = c2_c_A;
    c2_f_x = c2_e_x;
    c2_g_y = c2_f_x / 24.0;
    c2_s[c2_c_j + 24] = c2_g_y;
  }

  c2_md[0] = c2_dx[1];
  c2_md[4] = c2_dx[2];
  for (c2_d_k = 2; c2_d_k < 5; c2_d_k++) {
    c2_c_k = c2_d_k - 1;
    c2_md[c2_c_k] = 2.0 * (c2_dx[c2_c_k] + c2_dx[c2_c_k - 1]);
  }

  c2_d_A = c2_dx[1];
  c2_b_B = c2_md[0];
  c2_g_x = c2_d_A;
  c2_h_y = c2_b_B;
  c2_h_x = c2_g_x;
  c2_i_y = c2_h_y;
  c2_r = c2_h_x / c2_i_y;
  c2_md[1] -= c2_r * 24.0;
  for (c2_f_j = 1; c2_f_j < 13; c2_f_j++) {
    c2_c_j = c2_f_j + 11;
    c2_s[c2_c_j] -= c2_r * c2_s[c2_c_j - 12];
  }

  for (c2_e_k = 3; c2_e_k < 5; c2_e_k++) {
    c2_c_k = c2_e_k - 1;
    c2_e_A = c2_dx[c2_c_k];
    c2_d_B = c2_md[c2_c_k - 1];
    c2_i_x = c2_e_A;
    c2_l_y = c2_d_B;
    c2_j_x = c2_i_x;
    c2_m_y = c2_l_y;
    c2_r = c2_j_x / c2_m_y;
    c2_md[c2_c_k] -= c2_r * c2_dx[c2_c_k - 2];
    c2_pg = c2_c_k * 12;
    c2_pgm1 = (c2_c_k - 1) * 12;
    for (c2_i_j = 1; c2_i_j < 13; c2_i_j++) {
      c2_c_j = c2_i_j - 1;
      c2_s[c2_pg + c2_c_j] -= c2_r * c2_s[c2_pgm1 + c2_c_j];
    }
  }

  c2_c_B = c2_md[3];
  c2_j_y = c2_c_B;
  c2_k_y = c2_j_y;
  c2_r = 24.0 / c2_k_y;
  c2_md[4] -= c2_r * c2_dx[2];
  for (c2_g_j = 1; c2_g_j < 13; c2_g_j++) {
    c2_c_j = c2_g_j + 47;
    c2_s[c2_c_j] -= c2_r * c2_s[c2_c_j - 12];
  }

  for (c2_h_j = 1; c2_h_j < 13; c2_h_j++) {
    c2_c_j = c2_h_j + 47;
    c2_f_A = c2_s[c2_c_j];
    c2_e_B = c2_md[4];
    c2_k_x = c2_f_A;
    c2_n_y = c2_e_B;
    c2_l_x = c2_k_x;
    c2_o_y = c2_n_y;
    c2_p_y = c2_l_x / c2_o_y;
    c2_s[c2_c_j] = c2_p_y;
  }

  for (c2_f_k = 4; c2_f_k > 1; c2_f_k--) {
    c2_c_k = c2_f_k - 1;
    c2_pg = c2_c_k * 12;
    c2_pgp1 = (c2_c_k + 1) * 12;
    c2_d1 = c2_dx[c2_c_k - 1];
    for (c2_k_j = 1; c2_k_j < 13; c2_k_j++) {
      c2_c_j = c2_k_j - 1;
      c2_h_A = c2_s[c2_pg + c2_c_j] - c2_d1 * c2_s[c2_pgp1 + c2_c_j];
      c2_g_B = c2_md[c2_c_k];
      c2_o_x = c2_h_A;
      c2_u_y = c2_g_B;
      c2_p_x = c2_o_x;
      c2_v_y = c2_u_y;
      c2_w_y = c2_p_x / c2_v_y;
      c2_s[c2_pg + c2_c_j] = c2_w_y;
    }
  }

  for (c2_j_j = 1; c2_j_j < 13; c2_j_j++) {
    c2_c_j = c2_j_j - 1;
    c2_g_A = c2_s[c2_c_j] - 24.0 * c2_s[c2_c_j + 12];
    c2_f_B = c2_md[0];
    c2_m_x = c2_g_A;
    c2_q_y = c2_f_B;
    c2_n_x = c2_m_x;
    c2_s_y = c2_q_y;
    c2_t_y = c2_n_x / c2_s_y;
    c2_s[c2_c_j] = c2_t_y;
  }

  for (c2_i132 = 0; c2_i132 < 5; c2_i132++) {
    c2_dv26[c2_i132] = -24.0 + 12.0 * (real_T)c2_i132;
  }

  for (c2_i133 = 0; c2_i133 < 60; c2_i133++) {
    c2_r_y[c2_i133] = c2_y[c2_i133];
  }

  c2_d_pwchcore(chartInstance, c2_dv26, c2_r_y, c2_s, c2_dx, c2_dvdf, c2_pp);
}

static void c2_d_chckxy(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[60])
{
  boolean_T c2_p;
  int32_T c2_k;
  real_T c2_b_k;
  real_T c2_x;
  const mxArray *c2_b_y = NULL;
  boolean_T c2_b;
  static char_T c2_u[28] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'U', 'n', 's', 'u', 'p', 'p', 'o', 'r', 't', 'e', 'd',
    'N', 'a', 'N' };

  boolean_T exitg1;
  (void)chartInstance;
  c2_p = false;
  c2_k = 0;
  exitg1 = false;
  while ((exitg1 == false) && (c2_k < 60)) {
    c2_b_k = 1.0 + (real_T)c2_k;
    c2_x = c2_y[(int32_T)c2_b_k - 1];
    c2_b = muDoubleScalarIsNaN(c2_x);
    if (c2_b) {
      c2_p = true;
      exitg1 = true;
    } else {
      c2_k++;
    }
  }

  if (!c2_p) {
  } else {
    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 28),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c2_b_y));
  }
}

static void c2_p_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_d_pwchcore(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x[5],
  real_T c2_y[60], real_T c2_s[60], real_T c2_dx[4], real_T c2_divdif[48],
  c2_samQD8b77pHdwkX3iY74akB *c2_pp)
{
  int32_T c2_i134;
  int32_T c2_j;
  int32_T c2_b_j;
  real_T c2_dxj;
  int32_T c2_joffset;
  int32_T c2_i;
  int32_T c2_b_i;
  real_T c2_divdifij;
  real_T c2_A;
  real_T c2_B;
  real_T c2_b_x;
  real_T c2_b_y;
  real_T c2_c_x;
  real_T c2_c_y;
  real_T c2_dzzdx;
  real_T c2_b_A;
  real_T c2_b_B;
  real_T c2_d_x;
  real_T c2_d_y;
  real_T c2_e_x;
  real_T c2_e_y;
  real_T c2_dzdxdx;
  real_T c2_c_A;
  real_T c2_c_B;
  real_T c2_f_x;
  real_T c2_f_y;
  real_T c2_g_x;
  real_T c2_g_y;
  real_T c2_h_y;
  (void)chartInstance;
  for (c2_i134 = 0; c2_i134 < 5; c2_i134++) {
    c2_pp->breaks[c2_i134] = c2_x[c2_i134];
  }

  for (c2_j = 1; c2_j < 5; c2_j++) {
    c2_b_j = c2_j - 1;
    c2_dxj = c2_dx[c2_b_j];
    c2_joffset = c2_b_j * 12;
    for (c2_i = 1; c2_i < 13; c2_i++) {
      c2_b_i = c2_i - 1;
      c2_divdifij = c2_divdif[c2_joffset + c2_b_i];
      c2_A = c2_divdifij - c2_s[c2_joffset + c2_b_i];
      c2_B = c2_dxj;
      c2_b_x = c2_A;
      c2_b_y = c2_B;
      c2_c_x = c2_b_x;
      c2_c_y = c2_b_y;
      c2_dzzdx = c2_c_x / c2_c_y;
      c2_b_A = c2_s[(c2_joffset + c2_b_i) + 12] - c2_divdifij;
      c2_b_B = c2_dxj;
      c2_d_x = c2_b_A;
      c2_d_y = c2_b_B;
      c2_e_x = c2_d_x;
      c2_e_y = c2_d_y;
      c2_dzdxdx = c2_e_x / c2_e_y;
      c2_c_A = c2_dzdxdx - c2_dzzdx;
      c2_c_B = c2_dxj;
      c2_f_x = c2_c_A;
      c2_f_y = c2_c_B;
      c2_g_x = c2_f_x;
      c2_g_y = c2_f_y;
      c2_h_y = c2_g_x / c2_g_y;
      c2_pp->coefs[c2_joffset + c2_b_i] = c2_h_y;
      c2_pp->coefs[(c2_joffset + c2_b_i) + 48] = 2.0 * c2_dzzdx - c2_dzdxdx;
      c2_pp->coefs[(c2_joffset + c2_b_i) + 96] = c2_s[c2_joffset + c2_b_i];
      c2_pp->coefs[(c2_joffset + c2_b_i) + 144] = c2_y[c2_joffset + c2_b_i];
    }
  }
}

static void c2_q_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_e_ppval(SFc2_QuanInstanceStruct *chartInstance,
  c2_samQD8b77pHdwkX3iY74akB *c2_pp, real_T c2_x, real_T c2_v[12])
{
  real_T c2_b_x;
  boolean_T c2_b;
  int32_T c2_i135;
  int32_T c2_j;
  real_T c2_xi;
  real_T c2_c_x[5];
  int32_T c2_b_j;
  int32_T c2_low_i;
  int32_T c2_low_ip1;
  int32_T c2_high_i;
  int32_T c2_ip;
  int32_T c2_b_low_i;
  int32_T c2_icp;
  int32_T c2_b_high_i;
  real_T c2_xloc;
  int32_T c2_mid_i;
  int32_T c2_c_j;
  int32_T c2_ic;
  int32_T c2_b_ic;
  int32_T c2_ic0;
  int32_T c2_d_j;
  (void)chartInstance;
  c2_b_x = c2_x;
  c2_b = muDoubleScalarIsNaN(c2_b_x);
  if (c2_b) {
    for (c2_j = 1; c2_j < 13; c2_j++) {
      c2_b_j = c2_j - 1;
      c2_v[c2_b_j] = c2_x;
    }
  } else {
    for (c2_i135 = 0; c2_i135 < 5; c2_i135++) {
      c2_c_x[c2_i135] = c2_pp->breaks[c2_i135];
    }

    c2_xi = c2_x;
    c2_low_i = 1;
    c2_low_ip1 = 1;
    c2_high_i = 5;
    while (c2_high_i > c2_low_ip1 + 1) {
      c2_b_low_i = c2_low_i;
      c2_b_high_i = c2_high_i;
      c2_mid_i = (c2_b_low_i + c2_b_high_i) >> 1;
      if (c2_xi >= c2_c_x[c2_mid_i - 1]) {
        c2_low_i = c2_mid_i;
        c2_low_ip1 = c2_low_i;
      } else {
        c2_high_i = c2_mid_i;
      }
    }

    c2_ip = c2_low_i - 1;
    c2_icp = c2_ip * 12;
    c2_xloc = c2_x - c2_pp->breaks[c2_ip];
    for (c2_c_j = 1; c2_c_j < 13; c2_c_j++) {
      c2_b_j = c2_c_j - 1;
      c2_v[c2_b_j] = c2_pp->coefs[c2_icp + c2_b_j];
    }

    for (c2_ic = 2; c2_ic < 5; c2_ic++) {
      c2_b_ic = c2_ic - 1;
      c2_ic0 = (c2_icp + c2_b_ic * 48) - 1;
      for (c2_d_j = 1; c2_d_j < 13; c2_d_j++) {
        c2_b_j = c2_d_j - 1;
        c2_v[c2_b_j] = c2_xloc * c2_v[c2_b_j] + c2_pp->coefs[(c2_ic0 + c2_b_j) +
          1];
      }
    }
  }
}

static void c2_c_spline(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[12],
  c2_sSYDZjSrzjRAmypAou1DUJH *c2_output)
{
  int32_T c2_i136;
  real_T c2_b_y[12];
  for (c2_i136 = 0; c2_i136 < 12; c2_i136++) {
    c2_b_y[c2_i136] = c2_y[c2_i136];
  }

  c2_d_splinepp(chartInstance, c2_b_y, c2_output);
}

static void c2_d_splinepp(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[12],
  c2_sSYDZjSrzjRAmypAou1DUJH *c2_pp)
{
  int32_T c2_i137;
  real_T c2_b_y[12];
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_dx[11];
  int32_T c2_dpg;
  real_T c2_d1;
  int32_T c2_pg;
  real_T c2_d2;
  int32_T c2_pgp1;
  real_T c2_A;
  real_T c2_dvdf[11];
  int32_T c2_pgm1;
  real_T c2_b_A;
  real_T c2_x;
  real_T c2_B;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_c_y;
  real_T c2_s[12];
  real_T c2_d_y;
  real_T c2_d_x;
  real_T c2_e_y;
  real_T c2_f_y;
  real_T c2_c_A;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_g_y;
  real_T c2_md[12];
  int32_T c2_d_k;
  real_T c2_d_A;
  real_T c2_b_B;
  real_T c2_g_x;
  real_T c2_h_y;
  real_T c2_h_x;
  real_T c2_i_y;
  real_T c2_r;
  int32_T c2_e_k;
  real_T c2_c_B;
  real_T c2_j_y;
  real_T c2_e_A;
  real_T c2_k_y;
  real_T c2_d_B;
  real_T c2_i_x;
  real_T c2_l_y;
  real_T c2_j_x;
  real_T c2_f_A;
  real_T c2_m_y;
  real_T c2_e_B;
  real_T c2_k_x;
  real_T c2_n_y;
  real_T c2_l_x;
  real_T c2_o_y;
  real_T c2_p_y;
  int32_T c2_f_k;
  real_T c2_g_A;
  real_T c2_f_B;
  real_T c2_m_x;
  real_T c2_q_y;
  real_T c2_n_x;
  real_T c2_h_A;
  real_T c2_r_y;
  real_T c2_g_B;
  real_T c2_s_y;
  real_T c2_o_x;
  real_T c2_t_y;
  int32_T c2_i138;
  real_T c2_p_x;
  real_T c2_u_y;
  real_T c2_v_y;
  int32_T c2_i139;
  real_T c2_dv27[12];
  real_T c2_w_y[12];
  for (c2_i137 = 0; c2_i137 < 12; c2_i137++) {
    c2_b_y[c2_i137] = c2_y[c2_i137];
  }

  c2_e_chckxy(chartInstance, c2_b_y);
  for (c2_k = 1; c2_k < 12; c2_k++) {
    c2_c_k = c2_k - 1;
    c2_dx[c2_c_k] = (-10.0 + 5.0 * (real_T)(c2_c_k + 1)) - (-10.0 + 5.0 *
      (real_T)c2_c_k);
    c2_dpg = c2_c_k;
    c2_pg = c2_dpg;
    c2_pgp1 = c2_c_k + 1;
    c2_b_A = c2_y[c2_pgp1] - c2_y[c2_pg];
    c2_B = c2_dx[c2_c_k];
    c2_c_x = c2_b_A;
    c2_d_y = c2_B;
    c2_d_x = c2_c_x;
    c2_e_y = c2_d_y;
    c2_f_y = c2_d_x / c2_e_y;
    c2_dvdf[c2_dpg] = c2_f_y;
  }

  for (c2_b_k = 2; c2_b_k < 12; c2_b_k++) {
    c2_c_k = c2_b_k - 2;
    c2_pg = c2_c_k + 1;
    c2_pgm1 = c2_c_k;
    c2_d1 = c2_dx[c2_c_k + 1];
    c2_d2 = c2_dx[c2_c_k];
    c2_s[c2_pg] = 3.0 * (c2_d1 * c2_dvdf[c2_pgm1] + c2_d2 * c2_dvdf[c2_pg]);
  }

  c2_d1 = c2_dx[0];
  c2_d2 = c2_dx[1];
  c2_A = (c2_d1 + 20.0) * c2_d2 * c2_dvdf[0] + c2_d1 * c2_d1 * c2_dvdf[1];
  c2_x = c2_A;
  c2_b_x = c2_x;
  c2_c_y = c2_b_x / 10.0;
  c2_s[0] = c2_c_y;
  c2_d1 = c2_dx[10];
  c2_d2 = c2_dx[9];
  c2_c_A = (c2_d1 + 20.0) * c2_d2 * c2_dvdf[10] + c2_d1 * c2_d1 * c2_dvdf[9];
  c2_e_x = c2_c_A;
  c2_f_x = c2_e_x;
  c2_g_y = c2_f_x / 10.0;
  c2_s[11] = c2_g_y;
  c2_md[0] = c2_dx[1];
  c2_md[11] = c2_dx[9];
  for (c2_d_k = 2; c2_d_k < 12; c2_d_k++) {
    c2_c_k = c2_d_k - 1;
    c2_md[c2_c_k] = 2.0 * (c2_dx[c2_c_k] + c2_dx[c2_c_k - 1]);
  }

  c2_d_A = c2_dx[1];
  c2_b_B = c2_md[0];
  c2_g_x = c2_d_A;
  c2_h_y = c2_b_B;
  c2_h_x = c2_g_x;
  c2_i_y = c2_h_y;
  c2_r = c2_h_x / c2_i_y;
  c2_md[1] -= c2_r * 10.0;
  c2_s[1] -= c2_r * c2_s[0];
  for (c2_e_k = 3; c2_e_k < 12; c2_e_k++) {
    c2_c_k = c2_e_k - 1;
    c2_e_A = c2_dx[c2_c_k];
    c2_d_B = c2_md[c2_c_k - 1];
    c2_i_x = c2_e_A;
    c2_l_y = c2_d_B;
    c2_j_x = c2_i_x;
    c2_m_y = c2_l_y;
    c2_r = c2_j_x / c2_m_y;
    c2_md[c2_c_k] -= c2_r * c2_dx[c2_c_k - 2];
    c2_pg = c2_c_k;
    c2_pgm1 = c2_c_k;
    c2_s[c2_pg] -= c2_r * c2_s[c2_pgm1 - 1];
  }

  c2_c_B = c2_md[10];
  c2_j_y = c2_c_B;
  c2_k_y = c2_j_y;
  c2_r = 10.0 / c2_k_y;
  c2_md[11] -= c2_r * c2_dx[9];
  c2_s[11] -= c2_r * c2_s[10];
  c2_f_A = c2_s[11];
  c2_e_B = c2_md[11];
  c2_k_x = c2_f_A;
  c2_n_y = c2_e_B;
  c2_l_x = c2_k_x;
  c2_o_y = c2_n_y;
  c2_p_y = c2_l_x / c2_o_y;
  c2_s[11] = c2_p_y;
  for (c2_f_k = 11; c2_f_k > 1; c2_f_k--) {
    c2_c_k = c2_f_k - 1;
    c2_pg = c2_c_k;
    c2_pgp1 = c2_c_k + 1;
    c2_d1 = c2_dx[c2_c_k - 1];
    c2_h_A = c2_s[c2_pg] - c2_d1 * c2_s[c2_pgp1];
    c2_g_B = c2_md[c2_c_k];
    c2_o_x = c2_h_A;
    c2_t_y = c2_g_B;
    c2_p_x = c2_o_x;
    c2_u_y = c2_t_y;
    c2_v_y = c2_p_x / c2_u_y;
    c2_s[c2_pg] = c2_v_y;
  }

  c2_g_A = c2_s[0] - 10.0 * c2_s[1];
  c2_f_B = c2_md[0];
  c2_m_x = c2_g_A;
  c2_q_y = c2_f_B;
  c2_n_x = c2_m_x;
  c2_r_y = c2_q_y;
  c2_s_y = c2_n_x / c2_r_y;
  c2_s[0] = c2_s_y;
  for (c2_i138 = 0; c2_i138 < 12; c2_i138++) {
    c2_dv27[c2_i138] = -10.0 + 5.0 * (real_T)c2_i138;
  }

  for (c2_i139 = 0; c2_i139 < 12; c2_i139++) {
    c2_w_y[c2_i139] = c2_y[c2_i139];
  }

  c2_e_pwchcore(chartInstance, c2_dv27, c2_w_y, c2_s, c2_dx, c2_dvdf, c2_pp);
}

static void c2_e_chckxy(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[12])
{
  boolean_T c2_p;
  int32_T c2_k;
  real_T c2_b_k;
  real_T c2_x;
  const mxArray *c2_b_y = NULL;
  boolean_T c2_b;
  static char_T c2_u[28] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'U', 'n', 's', 'u', 'p', 'p', 'o', 'r', 't', 'e', 'd',
    'N', 'a', 'N' };

  boolean_T exitg1;
  (void)chartInstance;
  c2_p = false;
  c2_k = 0;
  exitg1 = false;
  while ((exitg1 == false) && (c2_k < 12)) {
    c2_b_k = 1.0 + (real_T)c2_k;
    c2_x = c2_y[(int32_T)c2_b_k - 1];
    c2_b = muDoubleScalarIsNaN(c2_x);
    if (c2_b) {
      c2_p = true;
      exitg1 = true;
    } else {
      c2_k++;
    }
  }

  if (!c2_p) {
  } else {
    c2_b_y = NULL;
    sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 28),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c2_b_y));
  }
}

static void c2_r_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_e_pwchcore(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x[12],
  real_T c2_y[12], real_T c2_s[12], real_T c2_dx[11], real_T c2_divdif[11],
  c2_sSYDZjSrzjRAmypAou1DUJH *c2_pp)
{
  int32_T c2_i140;
  int32_T c2_j;
  int32_T c2_b_j;
  real_T c2_dxj;
  int32_T c2_joffset;
  real_T c2_divdifij;
  real_T c2_A;
  real_T c2_B;
  real_T c2_b_x;
  real_T c2_b_y;
  real_T c2_c_x;
  real_T c2_c_y;
  real_T c2_dzzdx;
  real_T c2_b_A;
  real_T c2_b_B;
  real_T c2_d_x;
  real_T c2_d_y;
  real_T c2_e_x;
  real_T c2_e_y;
  real_T c2_dzdxdx;
  real_T c2_c_A;
  real_T c2_c_B;
  real_T c2_f_x;
  real_T c2_f_y;
  real_T c2_g_x;
  real_T c2_g_y;
  real_T c2_h_y;
  (void)chartInstance;
  for (c2_i140 = 0; c2_i140 < 12; c2_i140++) {
    c2_pp->breaks[c2_i140] = c2_x[c2_i140];
  }

  for (c2_j = 1; c2_j < 12; c2_j++) {
    c2_b_j = c2_j - 1;
    c2_dxj = c2_dx[c2_b_j];
    c2_joffset = c2_b_j;
    c2_divdifij = c2_divdif[c2_joffset];
    c2_A = c2_divdifij - c2_s[c2_joffset];
    c2_B = c2_dxj;
    c2_b_x = c2_A;
    c2_b_y = c2_B;
    c2_c_x = c2_b_x;
    c2_c_y = c2_b_y;
    c2_dzzdx = c2_c_x / c2_c_y;
    c2_b_A = c2_s[c2_joffset + 1] - c2_divdifij;
    c2_b_B = c2_dxj;
    c2_d_x = c2_b_A;
    c2_d_y = c2_b_B;
    c2_e_x = c2_d_x;
    c2_e_y = c2_d_y;
    c2_dzdxdx = c2_e_x / c2_e_y;
    c2_c_A = c2_dzdxdx - c2_dzzdx;
    c2_c_B = c2_dxj;
    c2_f_x = c2_c_A;
    c2_f_y = c2_c_B;
    c2_g_x = c2_f_x;
    c2_g_y = c2_f_y;
    c2_h_y = c2_g_x / c2_g_y;
    c2_pp->coefs[c2_joffset] = c2_h_y;
    c2_pp->coefs[c2_joffset + 11] = 2.0 * c2_dzzdx - c2_dzdxdx;
    c2_pp->coefs[c2_joffset + 22] = c2_s[c2_joffset];
    c2_pp->coefs[c2_joffset + 33] = c2_y[c2_joffset];
  }
}

static void c2_s_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_d_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4_data[], int32_T c2_varargin_4_sizes[2], real_T
  c2_varargin_5_data[], int32_T c2_varargin_5_sizes[2], real_T c2_yi_data[],
  int32_T c2_yi_sizes[2])
{
  real_T c2_outsize[2];
  int32_T c2_xxi_sizes[2];
  int32_T c2_xxi;
  int32_T c2_b_xxi;
  int32_T c2_loop_ub;
  int32_T c2_i141;
  real_T c2_nxxi;
  real_T c2_xxi_data[1];
  int32_T c2_i142;
  real_T c2_varargin_3[60];
  c2_samQD8b77pHdwkX3iY74akB c2_ppk;
  static real_T c2_b_varargin_3[60] = { -0.099, -0.081, -0.081, -0.063, -0.025,
    0.044, 0.097, 0.113, 0.145, 0.167, 0.174, 0.166, -0.048, -0.038, -0.04,
    -0.021, 0.016, 0.083, 0.127, 0.137, 0.162, 0.177, 0.179, 0.167, -0.022,
    -0.02, -0.021, -0.004, 0.032, 0.094, 0.128, 0.13, 0.154, 0.161, 0.155, 0.138,
    -0.04, -0.038, -0.039, -0.025, 0.006, 0.062, 0.087, 0.085, 0.1, 0.11, 0.104,
    0.091, -0.083, -0.073, -0.076, -0.072, -0.046, 0.012, 0.024, 0.025, 0.043,
    0.053, 0.047, 0.04 };

  real_T c2_sv[2];
  int32_T c2_i143;
  int32_T c2_varargout_1_sizes[2];
  int32_T c2_varargout_1;
  int32_T c2_b_varargout_1;
  int32_T c2_b_loop_ub;
  int32_T c2_i144;
  real_T c2_b_nxxi;
  real_T c2_varargout_1_data[12];
  int32_T c2_i145;
  int32_T c2_j;
  real_T c2_b_j;
  c2_samQD8b77pHdwkX3iY74akB c2_b_ppk;
  int32_T c2_c_xxi;
  int32_T c2_d_xxi[1];
  int32_T c2_e_xxi;
  real_T c2_vkj[12];
  int32_T c2_c_loop_ub;
  int32_T c2_i;
  int32_T c2_i146;
  real_T c2_b_i;
  c2_sL3Gi7vZqpbfvDjMPafkGnG c2_ppk_data;
  c2_sL3Gi7vZqpbfvDjMPafkGnG_size c2_ppk_elems_sizes;
  int32_T c2_i147;
  int32_T c2_yi;
  int32_T c2_b_yi;
  int32_T c2_d_loop_ub;
  int32_T c2_i148;
  real_T c2_c_nxxi;
  int32_T c2_i149;
  int32_T c2_c_j;
  c2_sL3Gi7vZqpbfvDjMPafkGnG_size c2_b_ppk_elems_sizes;
  c2_sL3Gi7vZqpbfvDjMPafkGnG c2_b_ppk_data;
  int32_T c2_f_xxi[1];
  real_T c2_vkj_data[1];
  int32_T c2_vkj_sizes;
  real_T c2_d8;
  int32_T c2_i150;
  int32_T c2_c_i;
  c2_outsize[0] = (real_T)c2_varargin_4_sizes[1];
  c2_outsize[1] = (real_T)c2_varargin_5_sizes[1];
  c2_xxi_sizes[0] = 1;
  c2_xxi_sizes[1] = c2_varargin_5_sizes[1];
  c2_xxi = c2_xxi_sizes[0];
  c2_b_xxi = c2_xxi_sizes[1];
  c2_loop_ub = c2_varargin_5_sizes[0] * c2_varargin_5_sizes[1] - 1;
  for (c2_i141 = 0; c2_i141 <= c2_loop_ub; c2_i141++) {
    c2_xxi_data[c2_i141] = c2_varargin_5_data[c2_i141];
  }

  c2_nxxi = (real_T)c2_xxi_sizes[1];
  for (c2_i142 = 0; c2_i142 < 60; c2_i142++) {
    c2_varargin_3[c2_i142] = c2_b_varargin_3[c2_i142];
  }

  c2_b_spline(chartInstance, c2_varargin_3, &c2_ppk);
  c2_sv[0] = (real_T)c2_varargin_5_sizes[1];
  c2_sv[1] = 12.0;
  for (c2_i143 = 0; c2_i143 < 2; c2_i143++) {
    c2_sv[c2_i143] = _SFD_NON_NEGATIVE_CHECK("", c2_sv[c2_i143]);
  }

  c2_varargout_1_sizes[0] = (int32_T)c2_sv[0];
  c2_varargout_1_sizes[1] = 12;
  c2_varargout_1 = c2_varargout_1_sizes[0];
  c2_b_varargout_1 = c2_varargout_1_sizes[1];
  c2_b_loop_ub = (int32_T)c2_sv[0] * (int32_T)c2_sv[1] - 1;
  for (c2_i144 = 0; c2_i144 <= c2_b_loop_ub; c2_i144++) {
    c2_varargout_1_data[c2_i144] = 0.0;
  }

  c2_b_nxxi = c2_nxxi;
  c2_i145 = (int32_T)c2_b_nxxi;
  _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_b_nxxi, mxDOUBLE_CLASS, c2_i145);
  for (c2_j = 0; c2_j < c2_i145; c2_j++) {
    c2_b_j = 1.0 + (real_T)c2_j;
    c2_b_ppk = c2_ppk;
    c2_d_xxi[0] = c2_xxi_sizes[1];
    c2_e_ppval(chartInstance, &c2_b_ppk, c2_xxi_data[(int32_T)c2_b_j - 1],
               c2_vkj);
    for (c2_i = 0; c2_i < 12; c2_i++) {
      c2_b_i = 1.0 + (real_T)c2_i;
      c2_varargout_1_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
        c2_vkj[(int32_T)c2_b_i - 1];
    }
  }

  c2_xxi_sizes[0] = 1;
  c2_xxi_sizes[1] = c2_varargin_4_sizes[1];
  c2_c_xxi = c2_xxi_sizes[0];
  c2_e_xxi = c2_xxi_sizes[1];
  c2_c_loop_ub = c2_varargin_4_sizes[0] * c2_varargin_4_sizes[1] - 1;
  for (c2_i146 = 0; c2_i146 <= c2_c_loop_ub; c2_i146++) {
    c2_xxi_data[c2_i146] = c2_varargin_4_data[c2_i146];
  }

  c2_nxxi = (real_T)c2_xxi_sizes[1];
  c2_d_spline(chartInstance, c2_varargout_1_data, c2_varargout_1_sizes,
              &c2_ppk_data, &c2_ppk_elems_sizes);
  for (c2_i147 = 0; c2_i147 < 2; c2_i147++) {
    c2_outsize[c2_i147] = _SFD_NON_NEGATIVE_CHECK("", c2_outsize[c2_i147]);
  }

  c2_yi_sizes[0] = (int32_T)c2_outsize[0];
  c2_yi_sizes[1] = (int32_T)c2_outsize[1];
  c2_yi = c2_yi_sizes[0];
  c2_b_yi = c2_yi_sizes[1];
  c2_d_loop_ub = (int32_T)c2_outsize[0] * (int32_T)c2_outsize[1] - 1;
  for (c2_i148 = 0; c2_i148 <= c2_d_loop_ub; c2_i148++) {
    c2_yi_data[c2_i148] = 0.0;
  }

  c2_c_nxxi = c2_nxxi;
  c2_i149 = (int32_T)c2_c_nxxi;
  _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_c_nxxi, mxDOUBLE_CLASS, c2_i149);
  for (c2_c_j = 0; c2_c_j < c2_i149; c2_c_j++) {
    c2_b_j = 1.0 + (real_T)c2_c_j;
    c2_b_ppk_elems_sizes = c2_ppk_elems_sizes;
    c2_b_ppk_data = c2_ppk_data;
    c2_f_xxi[0] = c2_xxi_sizes[1];
    c2_f_ppval(chartInstance, &c2_b_ppk_data, c2_b_ppk_elems_sizes, c2_xxi_data
               [(int32_T)c2_b_j - 1], c2_vkj_data, &c2_vkj_sizes);
    c2_d8 = (real_T)c2_vkj_sizes;
    c2_i150 = (int32_T)c2_d8;
    _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_d8, mxDOUBLE_CLASS, c2_i150);
    for (c2_c_i = 0; c2_c_i < c2_i150; c2_c_i++) {
      c2_b_i = 1.0 + (real_T)c2_c_i;
      c2_yi_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
        c2_vkj_data[(int32_T)c2_b_i - 1];
    }
  }
}

static void c2_d_spline(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_y_data[], int32_T c2_y_sizes[2], c2_sL3Gi7vZqpbfvDjMPafkGnG *c2_output_data,
  c2_sL3Gi7vZqpbfvDjMPafkGnG_size *c2_output_elems_sizes)
{
  int32_T c2_b_y_sizes[2];
  int32_T c2_y;
  int32_T c2_b_y;
  int32_T c2_loop_ub;
  int32_T c2_i151;
  real_T c2_b_y_data[12];
  int32_T c2_npages;
  int32_T c2_x_sizes[2];
  int32_T c2_x;
  int32_T c2_b_x;
  int32_T c2_b_loop_ub;
  int32_T c2_i152;
  int32_T c2_pglen;
  real_T c2_x_data[12];
  boolean_T c2_has_endslopes;
  int32_T c2_i153;
  int32_T c2_i154;
  int32_T c2_i155;
  real_T c2_c_y[2];
  int32_T c2_i156;
  real_T c2_szdvdf[2];
  int32_T c2_i157;
  int32_T c2_i158;
  int32_T c2_i159;
  int32_T c2_i160;
  int32_T c2_yoffset;
  real_T c2_szs[2];
  int32_T c2_i161;
  int32_T c2_dvdf_sizes[2];
  int32_T c2_i162;
  int32_T c2_s_sizes[2];
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_dx[11];
  int32_T c2_dpg;
  int32_T c2_pg;
  real_T c2_d31;
  int32_T c2_pgp1;
  real_T c2_dnnm2;
  int32_T c2_pgm1;
  int32_T c2_b_pglen;
  real_T c2_d1;
  int32_T c2_c_pglen;
  int32_T c2_b;
  real_T c2_d2;
  int32_T c2_b_b;
  int32_T c2_c_b;
  int32_T c2_d_pglen;
  int32_T c2_d_b;
  int32_T c2_e_pglen;
  boolean_T c2_overflow;
  int32_T c2_e_b;
  boolean_T c2_b_overflow;
  int32_T c2_f_b;
  int32_T c2_g_b;
  int32_T c2_h_b;
  boolean_T c2_c_overflow;
  boolean_T c2_d_overflow;
  int32_T c2_j;
  int32_T c2_b_j;
  int32_T c2_c_j;
  int32_T c2_d_j;
  int32_T c2_e_j;
  int32_T c2_pgend2;
  real_T c2_A;
  int32_T c2_pglast;
  real_T c2_s_data[12];
  real_T c2_B;
  int32_T c2_f_pglen;
  real_T c2_c_x;
  real_T c2_b_A;
  real_T c2_dvdf_data[11];
  int32_T c2_i_b;
  real_T c2_d_y;
  int32_T c2_pgm2;
  real_T c2_d_x;
  int32_T c2_j_b;
  real_T c2_e_x;
  real_T c2_f_x;
  boolean_T c2_e_overflow;
  real_T c2_e_y;
  real_T c2_f_y;
  real_T c2_g_y;
  int32_T c2_g_pglen;
  int32_T c2_k_b;
  int32_T c2_f_j;
  int32_T c2_l_b;
  boolean_T c2_f_overflow;
  real_T c2_md[12];
  int32_T c2_g_j;
  int32_T c2_d_k;
  real_T c2_c_A;
  real_T c2_d_A;
  real_T c2_g_x;
  real_T c2_b_B;
  real_T c2_h_x;
  real_T c2_i_x;
  real_T c2_h_y;
  real_T c2_i_y;
  real_T c2_j_x;
  real_T c2_j_y;
  real_T c2_r;
  int32_T c2_h_pglen;
  int32_T c2_m_b;
  int32_T c2_n_b;
  boolean_T c2_g_overflow;
  int32_T c2_h_j;
  int32_T c2_e_k;
  real_T c2_e_A;
  real_T c2_c_B;
  real_T c2_f_A;
  real_T c2_k_x;
  real_T c2_d_B;
  real_T c2_k_y;
  real_T c2_l_x;
  real_T c2_m_x;
  real_T c2_l_y;
  real_T c2_m_y;
  real_T c2_n_x;
  real_T c2_n_y;
  int32_T c2_i_pglen;
  int32_T c2_o_b;
  int32_T c2_j_pglen;
  int32_T c2_p_b;
  int32_T c2_q_b;
  boolean_T c2_h_overflow;
  int32_T c2_r_b;
  boolean_T c2_i_overflow;
  int32_T c2_i_j;
  int32_T c2_j_j;
  int32_T c2_k_pglen;
  int32_T c2_s_b;
  int32_T c2_t_b;
  boolean_T c2_j_overflow;
  int32_T c2_k_j;
  int32_T c2_f_k;
  real_T c2_g_A;
  real_T c2_e_B;
  int32_T c2_l_pglen;
  real_T c2_o_x;
  int32_T c2_u_b;
  real_T c2_o_y;
  int32_T c2_v_b;
  real_T c2_p_x;
  boolean_T c2_k_overflow;
  real_T c2_p_y;
  int32_T c2_m_pglen;
  real_T c2_q_y;
  int32_T c2_w_b;
  int32_T c2_l_j;
  int32_T c2_x_b;
  boolean_T c2_l_overflow;
  int32_T c2_i163;
  real_T c2_h_A;
  int32_T c2_m_j;
  real_T c2_f_B;
  int32_T c2_c_y_sizes[2];
  real_T c2_dv28[12];
  real_T c2_q_x;
  real_T c2_r_y;
  int32_T c2_s_y;
  real_T c2_r_x;
  real_T c2_i_A;
  int32_T c2_t_y;
  real_T c2_u_y;
  real_T c2_g_B;
  int32_T c2_c_loop_ub;
  real_T c2_v_y;
  real_T c2_s_x;
  int32_T c2_i164;
  real_T c2_w_y;
  real_T c2_t_x;
  real_T c2_x_y;
  real_T c2_c_y_data[12];
  real_T c2_y_y;
  c2_b_y_sizes[0] = c2_y_sizes[0];
  c2_b_y_sizes[1] = 12;
  c2_y = c2_b_y_sizes[0];
  c2_b_y = c2_b_y_sizes[1];
  c2_loop_ub = c2_y_sizes[0] * c2_y_sizes[1] - 1;
  for (c2_i151 = 0; c2_i151 <= c2_loop_ub; c2_i151++) {
    c2_b_y_data[c2_i151] = c2_y_data[c2_i151];
  }

  c2_f_chckxy(chartInstance, c2_b_y_data, c2_b_y_sizes);
  c2_npages = c2_y_sizes[1];
  c2_x_sizes[0] = c2_y_sizes[0];
  c2_x_sizes[1] = 12;
  c2_x = c2_x_sizes[0];
  c2_b_x = c2_x_sizes[1];
  c2_b_loop_ub = c2_y_sizes[0] * c2_y_sizes[1] - 1;
  for (c2_i152 = 0; c2_i152 <= c2_b_loop_ub; c2_i152++) {
    c2_x_data[c2_i152] = c2_y_data[c2_i152];
  }

  c2_pglen = c2_x_sizes[0];
  c2_has_endslopes = (c2_npages == 14);
  if (c2_has_endslopes) {
    for (c2_i154 = 0; c2_i154 < 2; c2_i154++) {
      c2_c_y[c2_i154] = (real_T)c2_y_sizes[c2_i154];
    }

    for (c2_i156 = 0; c2_i156 < 2; c2_i156++) {
      c2_szdvdf[c2_i156] = c2_c_y[c2_i156];
    }

    c2_szdvdf[1] = (real_T)c2_y_sizes[1] - 3.0;
    for (c2_i158 = 0; c2_i158 < 2; c2_i158++) {
      c2_c_y[c2_i158] = (real_T)c2_y_sizes[c2_i158];
    }

    for (c2_i160 = 0; c2_i160 < 2; c2_i160++) {
      c2_szs[c2_i160] = c2_c_y[c2_i160];
    }

    c2_szs[1] = (real_T)c2_y_sizes[1] - 2.0;
    c2_yoffset = c2_pglen;
  } else {
    for (c2_i153 = 0; c2_i153 < 2; c2_i153++) {
      c2_c_y[c2_i153] = (real_T)c2_y_sizes[c2_i153];
    }

    for (c2_i155 = 0; c2_i155 < 2; c2_i155++) {
      c2_szdvdf[c2_i155] = c2_c_y[c2_i155];
    }

    c2_szdvdf[1] = (real_T)c2_y_sizes[1] - 1.0;
    for (c2_i157 = 0; c2_i157 < 2; c2_i157++) {
      c2_c_y[c2_i157] = (real_T)c2_y_sizes[c2_i157];
    }

    for (c2_i159 = 0; c2_i159 < 2; c2_i159++) {
      c2_szs[c2_i159] = c2_c_y[c2_i159];
    }

    c2_yoffset = 0;
  }

  for (c2_i161 = 0; c2_i161 < 2; c2_i161++) {
    c2_szdvdf[c2_i161] = _SFD_NON_NEGATIVE_CHECK("", c2_szdvdf[c2_i161]);
  }

  c2_dvdf_sizes[0] = (int32_T)c2_szdvdf[0];
  c2_dvdf_sizes[1] = (int32_T)c2_szdvdf[1];
  for (c2_i162 = 0; c2_i162 < 2; c2_i162++) {
    c2_szs[c2_i162] = _SFD_NON_NEGATIVE_CHECK("", c2_szs[c2_i162]);
  }

  c2_s_sizes[0] = (int32_T)c2_szs[0];
  c2_s_sizes[1] = (int32_T)c2_szs[1];
  for (c2_k = 1; c2_k < 12; c2_k++) {
    c2_c_k = c2_k - 1;
    c2_dx[c2_c_k] = (-10.0 + 5.0 * (real_T)(c2_c_k + 1)) - (-10.0 + 5.0 *
      (real_T)c2_c_k);
    c2_dpg = c2_c_k * c2_pglen;
    c2_pg = c2_yoffset + c2_dpg;
    c2_pgp1 = c2_yoffset + (c2_c_k + 1) * c2_pglen;
    c2_b_pglen = c2_pglen;
    c2_b = c2_b_pglen;
    c2_c_b = c2_b;
    c2_overflow = ((!(1 > c2_c_b)) && (c2_c_b > 2147483646));
    if (c2_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_overflow);
    }

    for (c2_j = 1; c2_j <= c2_b_pglen; c2_j++) {
      c2_e_j = c2_j - 1;
      c2_A = c2_y_data[c2_pgp1 + c2_e_j] - c2_y_data[c2_pg + c2_e_j];
      c2_B = c2_dx[c2_c_k];
      c2_c_x = c2_A;
      c2_d_y = c2_B;
      c2_e_x = c2_c_x;
      c2_e_y = c2_d_y;
      c2_g_y = c2_e_x / c2_e_y;
      c2_dvdf_data[c2_dpg + c2_e_j] = c2_g_y;
    }
  }

  for (c2_b_k = 2; c2_b_k < 12; c2_b_k++) {
    c2_c_k = c2_b_k - 2;
    c2_pg = (c2_c_k + 1) * c2_pglen;
    c2_pgm1 = c2_c_k * c2_pglen;
    c2_d1 = c2_dx[c2_c_k + 1];
    c2_d2 = c2_dx[c2_c_k];
    c2_e_pglen = c2_pglen;
    c2_f_b = c2_e_pglen;
    c2_h_b = c2_f_b;
    c2_d_overflow = ((!(1 > c2_h_b)) && (c2_h_b > 2147483646));
    if (c2_d_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_d_overflow);
    }

    for (c2_d_j = 1; c2_d_j <= c2_e_pglen; c2_d_j++) {
      c2_e_j = c2_d_j - 1;
      c2_s_data[c2_pg + c2_e_j] = 3.0 * (c2_d1 * c2_dvdf_data[c2_pgm1 + c2_e_j]
        + c2_d2 * c2_dvdf_data[c2_pg + c2_e_j]);
    }
  }

  if (c2_has_endslopes) {
    c2_d31 = 0.0;
    c2_dnnm2 = 0.0;
    c2_c_pglen = c2_pglen;
    c2_b_b = c2_c_pglen;
    c2_d_b = c2_b_b;
    c2_b_overflow = ((!(1 > c2_d_b)) && (c2_d_b > 2147483646));
    if (c2_b_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_b_overflow);
    }

    for (c2_b_j = 1; c2_b_j <= c2_c_pglen; c2_b_j++) {
      c2_e_j = c2_b_j - 1;
      c2_s_data[c2_e_j] = c2_dx[1] * c2_y_data[c2_e_j];
    }

    c2_pgend2 = 13 * c2_pglen;
    c2_pglast = 11 * c2_pglen;
    c2_f_pglen = c2_pglen;
    c2_i_b = c2_f_pglen;
    c2_j_b = c2_i_b;
    c2_e_overflow = ((!(1 > c2_j_b)) && (c2_j_b > 2147483646));
    if (c2_e_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_e_overflow);
    }

    for (c2_f_j = 1; c2_f_j <= c2_f_pglen; c2_f_j++) {
      c2_e_j = c2_f_j - 1;
      c2_s_data[c2_pglast + c2_e_j] = c2_dx[9] * c2_y_data[c2_pgend2 + c2_e_j];
    }
  } else {
    c2_d31 = 10.0;
    c2_dnnm2 = 10.0;
    c2_d1 = c2_dx[0];
    c2_d2 = c2_dx[1];
    c2_d_pglen = c2_pglen;
    c2_e_b = c2_d_pglen;
    c2_g_b = c2_e_b;
    c2_c_overflow = ((!(1 > c2_g_b)) && (c2_g_b > 2147483646));
    if (c2_c_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_c_overflow);
    }

    for (c2_c_j = 1; c2_c_j <= c2_d_pglen; c2_c_j++) {
      c2_e_j = c2_c_j - 1;
      c2_b_A = (c2_d1 + 20.0) * c2_d2 * c2_dvdf_data[c2_e_j] + c2_d1 * c2_d1 *
        c2_dvdf_data[c2_pglen + c2_e_j];
      c2_d_x = c2_b_A;
      c2_f_x = c2_d_x;
      c2_f_y = c2_f_x / 10.0;
      c2_s_data[c2_e_j] = c2_f_y;
    }

    c2_pg = 11 * c2_pglen;
    c2_pgm1 = 10 * c2_pglen;
    c2_pgm2 = 9 * c2_pglen;
    c2_d1 = c2_dx[10];
    c2_d2 = c2_dx[9];
    c2_g_pglen = c2_pglen;
    c2_k_b = c2_g_pglen;
    c2_l_b = c2_k_b;
    c2_f_overflow = ((!(1 > c2_l_b)) && (c2_l_b > 2147483646));
    if (c2_f_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_f_overflow);
    }

    for (c2_g_j = 1; c2_g_j <= c2_g_pglen; c2_g_j++) {
      c2_e_j = c2_g_j - 1;
      c2_c_A = (c2_d1 + 20.0) * c2_d2 * c2_dvdf_data[c2_pgm1 + c2_e_j] + c2_d1 *
        c2_d1 * c2_dvdf_data[c2_pgm2 + c2_e_j];
      c2_g_x = c2_c_A;
      c2_h_x = c2_g_x;
      c2_h_y = c2_h_x / 10.0;
      c2_s_data[c2_pg + c2_e_j] = c2_h_y;
    }
  }

  c2_md[0] = c2_dx[1];
  c2_md[11] = c2_dx[9];
  for (c2_d_k = 2; c2_d_k < 12; c2_d_k++) {
    c2_c_k = c2_d_k - 1;
    c2_md[c2_c_k] = 2.0 * (c2_dx[c2_c_k] + c2_dx[c2_c_k - 1]);
  }

  c2_d_A = c2_dx[1];
  c2_b_B = c2_md[0];
  c2_i_x = c2_d_A;
  c2_i_y = c2_b_B;
  c2_j_x = c2_i_x;
  c2_j_y = c2_i_y;
  c2_r = c2_j_x / c2_j_y;
  c2_md[1] -= c2_r * c2_d31;
  c2_h_pglen = c2_pglen;
  c2_m_b = c2_h_pglen;
  c2_n_b = c2_m_b;
  c2_g_overflow = ((!(1 > c2_n_b)) && (c2_n_b > 2147483646));
  if (c2_g_overflow) {
    c2_check_forloop_overflow_error(chartInstance, c2_g_overflow);
  }

  for (c2_h_j = 1; c2_h_j <= c2_h_pglen; c2_h_j++) {
    c2_e_j = c2_h_j - 1;
    c2_s_data[c2_pglen + c2_e_j] -= c2_r * c2_s_data[c2_e_j];
  }

  for (c2_e_k = 3; c2_e_k < 12; c2_e_k++) {
    c2_c_k = c2_e_k - 1;
    c2_f_A = c2_dx[c2_c_k];
    c2_d_B = c2_md[c2_c_k - 1];
    c2_l_x = c2_f_A;
    c2_l_y = c2_d_B;
    c2_n_x = c2_l_x;
    c2_n_y = c2_l_y;
    c2_r = c2_n_x / c2_n_y;
    c2_md[c2_c_k] -= c2_r * c2_dx[c2_c_k - 2];
    c2_pg = c2_c_k * c2_pglen;
    c2_pgm1 = (c2_c_k - 1) * c2_pglen;
    c2_j_pglen = c2_pglen;
    c2_q_b = c2_j_pglen;
    c2_r_b = c2_q_b;
    c2_i_overflow = ((!(1 > c2_r_b)) && (c2_r_b > 2147483646));
    if (c2_i_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_i_overflow);
    }

    for (c2_j_j = 1; c2_j_j <= c2_j_pglen; c2_j_j++) {
      c2_e_j = c2_j_j - 1;
      c2_s_data[c2_pg + c2_e_j] -= c2_r * c2_s_data[c2_pgm1 + c2_e_j];
    }
  }

  c2_e_A = c2_dnnm2;
  c2_c_B = c2_md[10];
  c2_k_x = c2_e_A;
  c2_k_y = c2_c_B;
  c2_m_x = c2_k_x;
  c2_m_y = c2_k_y;
  c2_r = c2_m_x / c2_m_y;
  c2_md[11] -= c2_r * c2_dx[9];
  c2_pg = 11 * c2_pglen;
  c2_pgm1 = 10 * c2_pglen;
  c2_i_pglen = c2_pglen;
  c2_o_b = c2_i_pglen;
  c2_p_b = c2_o_b;
  c2_h_overflow = ((!(1 > c2_p_b)) && (c2_p_b > 2147483646));
  if (c2_h_overflow) {
    c2_check_forloop_overflow_error(chartInstance, c2_h_overflow);
  }

  for (c2_i_j = 1; c2_i_j <= c2_i_pglen; c2_i_j++) {
    c2_e_j = c2_i_j - 1;
    c2_s_data[c2_pg + c2_e_j] -= c2_r * c2_s_data[c2_pgm1 + c2_e_j];
  }

  c2_k_pglen = c2_pglen;
  c2_s_b = c2_k_pglen;
  c2_t_b = c2_s_b;
  c2_j_overflow = ((!(1 > c2_t_b)) && (c2_t_b > 2147483646));
  if (c2_j_overflow) {
    c2_check_forloop_overflow_error(chartInstance, c2_j_overflow);
  }

  for (c2_k_j = 1; c2_k_j <= c2_k_pglen; c2_k_j++) {
    c2_e_j = c2_k_j - 1;
    c2_g_A = c2_s_data[c2_pg + c2_e_j];
    c2_e_B = c2_md[11];
    c2_o_x = c2_g_A;
    c2_o_y = c2_e_B;
    c2_p_x = c2_o_x;
    c2_p_y = c2_o_y;
    c2_q_y = c2_p_x / c2_p_y;
    c2_s_data[c2_pg + c2_e_j] = c2_q_y;
  }

  for (c2_f_k = 11; c2_f_k > 1; c2_f_k--) {
    c2_c_k = c2_f_k - 1;
    c2_pg = c2_c_k * c2_pglen;
    c2_pgp1 = (c2_c_k + 1) * c2_pglen;
    c2_d1 = c2_dx[c2_c_k - 1];
    c2_m_pglen = c2_pglen;
    c2_w_b = c2_m_pglen;
    c2_x_b = c2_w_b;
    c2_l_overflow = ((!(1 > c2_x_b)) && (c2_x_b > 2147483646));
    if (c2_l_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_l_overflow);
    }

    for (c2_m_j = 1; c2_m_j <= c2_m_pglen; c2_m_j++) {
      c2_e_j = c2_m_j - 1;
      c2_i_A = c2_s_data[c2_pg + c2_e_j] - c2_d1 * c2_s_data[c2_pgp1 + c2_e_j];
      c2_g_B = c2_md[c2_c_k];
      c2_s_x = c2_i_A;
      c2_w_y = c2_g_B;
      c2_t_x = c2_s_x;
      c2_x_y = c2_w_y;
      c2_y_y = c2_t_x / c2_x_y;
      c2_s_data[c2_pg + c2_e_j] = c2_y_y;
    }
  }

  c2_l_pglen = c2_pglen;
  c2_u_b = c2_l_pglen;
  c2_v_b = c2_u_b;
  c2_k_overflow = ((!(1 > c2_v_b)) && (c2_v_b > 2147483646));
  if (c2_k_overflow) {
    c2_check_forloop_overflow_error(chartInstance, c2_k_overflow);
  }

  for (c2_l_j = 1; c2_l_j <= c2_l_pglen; c2_l_j++) {
    c2_e_j = c2_l_j - 1;
    c2_h_A = c2_s_data[c2_e_j] - c2_d31 * c2_s_data[c2_pglen + c2_e_j];
    c2_f_B = c2_md[0];
    c2_q_x = c2_h_A;
    c2_r_y = c2_f_B;
    c2_r_x = c2_q_x;
    c2_u_y = c2_r_y;
    c2_v_y = c2_r_x / c2_u_y;
    c2_s_data[c2_e_j] = c2_v_y;
  }

  for (c2_i163 = 0; c2_i163 < 12; c2_i163++) {
    c2_dv28[c2_i163] = -10.0 + 5.0 * (real_T)c2_i163;
  }

  c2_c_y_sizes[0] = c2_y_sizes[0];
  c2_c_y_sizes[1] = 12;
  c2_s_y = c2_c_y_sizes[0];
  c2_t_y = c2_c_y_sizes[1];
  c2_c_loop_ub = c2_y_sizes[0] * c2_y_sizes[1] - 1;
  for (c2_i164 = 0; c2_i164 <= c2_c_loop_ub; c2_i164++) {
    c2_c_y_data[c2_i164] = c2_y_data[c2_i164];
  }

  c2_f_pwchcore(chartInstance, c2_dv28, c2_c_y_data, c2_c_y_sizes, c2_yoffset,
                c2_s_data, c2_s_sizes, c2_dx, c2_dvdf_data, c2_dvdf_sizes,
                c2_output_data, c2_output_elems_sizes);
}

static void c2_f_chckxy(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_y_data[], int32_T c2_y_sizes[2])
{
  int32_T c2_ny;
  int32_T c2_b_y_sizes[2];
  int32_T c2_y;
  int32_T c2_b_y;
  int32_T c2_loop_ub;
  int32_T c2_i165;
  real_T c2_b_y_data[12];
  const mxArray *c2_c_y = NULL;
  static char_T c2_u[28] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'U', 'n', 's', 'u', 'p', 'p', 'o', 'r', 't', 'e', 'd',
    'N', 'a', 'N' };

  const mxArray *c2_d_y = NULL;
  static char_T c2_b_u[36] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'c', 'h', 'c',
    'k', 'x', 'y', ':', 'N', 'u', 'm', 'S', 'i', 't', 'e', 's', 'M', 'i', 's',
    'm', 'a', 't', 'c', 'h', 'V', 'a', 'l', 'u', 'e', 's' };

  int32_T c2_c_u;
  const mxArray *c2_e_y = NULL;
  int32_T c2_d_u;
  const mxArray *c2_f_y = NULL;
  c2_ny = c2_y_sizes[1];
  c2_b_y_sizes[0] = c2_y_sizes[0];
  c2_b_y_sizes[1] = 12;
  c2_y = c2_b_y_sizes[0];
  c2_b_y = c2_b_y_sizes[1];
  c2_loop_ub = c2_y_sizes[0] * c2_y_sizes[1] - 1;
  for (c2_i165 = 0; c2_i165 <= c2_loop_ub; c2_i165++) {
    c2_b_y_data[c2_i165] = c2_y_data[c2_i165];
  }

  if (!c2_anyIsNaN(chartInstance, c2_b_y_data, c2_b_y_sizes)) {
  } else {
    c2_c_y = NULL;
    sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 28),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c2_c_y));
  }

  if (c2_ny != 12) {
    if (c2_ny == 14) {
    } else {
      c2_d_y = NULL;
      sf_mex_assign(&c2_d_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 36),
                    false);
      c2_c_u = 12;
      c2_e_y = NULL;
      sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_c_u, 6, 0U, 0U, 0U, 0),
                    false);
      c2_d_u = c2_ny;
      c2_f_y = NULL;
      sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_d_u, 6, 0U, 0U, 0U, 0),
                    false);
      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                        sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
        1U, 3U, 14, c2_d_y, 14, c2_e_y, 14, c2_f_y));
    }
  }
}

static void c2_t_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_u_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_f_pwchcore(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x[12],
  real_T c2_y_data[], int32_T c2_y_sizes[2], int32_T c2_yoffset, real_T
  c2_s_data[], int32_T c2_s_sizes[2], real_T c2_dx[11], real_T c2_divdif_data[],
  int32_T c2_divdif_sizes[2], c2_sL3Gi7vZqpbfvDjMPafkGnG *c2_pp_data,
  c2_sL3Gi7vZqpbfvDjMPafkGnG_size *c2_pp_elems_sizes)
{
  int32_T c2_i166;
  int32_T c2_x_sizes[2];
  int32_T c2_b_x;
  int32_T c2_c_x;
  int32_T c2_loop_ub;
  int32_T c2_i167;
  int32_T c2_nyrows;
  real_T c2_x_data[12];
  int32_T c2_cpage;
  int32_T c2_i168;
  int32_T c2_i169;
  real_T c2_s[2];
  int32_T c2_szc[3];
  int32_T c2_i170;
  int32_T c2_j;
  int32_T c2_b_j;
  real_T c2_dxj;
  int32_T c2_joffset;
  int32_T c2_b_nyrows;
  int32_T c2_b;
  int32_T c2_b_b;
  boolean_T c2_overflow;
  int32_T c2_i;
  int32_T c2_b_i;
  real_T c2_divdifij;
  real_T c2_A;
  real_T c2_B;
  real_T c2_d_x;
  real_T c2_y;
  real_T c2_e_x;
  real_T c2_b_y;
  real_T c2_dzzdx;
  real_T c2_b_A;
  real_T c2_b_B;
  real_T c2_f_x;
  real_T c2_c_y;
  real_T c2_g_x;
  real_T c2_d_y;
  real_T c2_dzdxdx;
  real_T c2_c_A;
  real_T c2_c_B;
  real_T c2_h_x;
  real_T c2_e_y;
  real_T c2_i_x;
  real_T c2_f_y;
  real_T c2_g_y;
  int32_T c2_c_b;
  int32_T c2_h_y;
  int32_T c2_d_b;
  int32_T c2_i_y;
  (void)c2_y_sizes;
  (void)c2_divdif_sizes;
  for (c2_i166 = 0; c2_i166 < 12; c2_i166++) {
    c2_pp_data->breaks[c2_i166] = c2_x[c2_i166];
  }

  c2_x_sizes[0] = c2_s_sizes[0];
  c2_x_sizes[1] = 12;
  c2_b_x = c2_x_sizes[0];
  c2_c_x = c2_x_sizes[1];
  c2_loop_ub = c2_s_sizes[0] * c2_s_sizes[1] - 1;
  for (c2_i167 = 0; c2_i167 <= c2_loop_ub; c2_i167++) {
    c2_x_data[c2_i167] = c2_s_data[c2_i167];
  }

  c2_nyrows = c2_x_sizes[0];
  c2_cpage = c2_nyrows * 11;
  for (c2_i168 = 0; c2_i168 < 2; c2_i168++) {
    c2_s[c2_i168] = (real_T)c2_s_sizes[c2_i168];
  }

  for (c2_i169 = 0; c2_i169 < 2; c2_i169++) {
    c2_szc[c2_i169] = (int32_T)c2_s[c2_i169];
  }

  c2_szc[2] = 4;
  c2_szc[1] = 11;
  for (c2_i170 = 0; c2_i170 < 3; c2_i170++) {
    c2_szc[c2_i170] = (int32_T)_SFD_NON_NEGATIVE_CHECK("", (real_T)
      c2_szc[c2_i170]);
  }

  c2_pp_elems_sizes->coefs[0] = c2_szc[0];
  c2_pp_elems_sizes->coefs[1] = 11;
  c2_pp_elems_sizes->coefs[2] = 4;
  for (c2_j = 1; c2_j < 12; c2_j++) {
    c2_b_j = c2_j - 1;
    c2_dxj = c2_dx[c2_b_j];
    c2_joffset = c2_b_j * c2_nyrows;
    c2_b_nyrows = c2_nyrows;
    c2_b = c2_b_nyrows;
    c2_b_b = c2_b;
    c2_overflow = ((!(1 > c2_b_b)) && (c2_b_b > 2147483646));
    if (c2_overflow) {
      c2_check_forloop_overflow_error(chartInstance, c2_overflow);
    }

    for (c2_i = 1; c2_i <= c2_b_nyrows; c2_i++) {
      c2_b_i = c2_i - 1;
      c2_divdifij = c2_divdif_data[c2_joffset + c2_b_i];
      c2_A = c2_divdifij - c2_s_data[c2_joffset + c2_b_i];
      c2_B = c2_dxj;
      c2_d_x = c2_A;
      c2_y = c2_B;
      c2_e_x = c2_d_x;
      c2_b_y = c2_y;
      c2_dzzdx = c2_e_x / c2_b_y;
      c2_b_A = c2_s_data[(c2_joffset + c2_nyrows) + c2_b_i] - c2_divdifij;
      c2_b_B = c2_dxj;
      c2_f_x = c2_b_A;
      c2_c_y = c2_b_B;
      c2_g_x = c2_f_x;
      c2_d_y = c2_c_y;
      c2_dzdxdx = c2_g_x / c2_d_y;
      c2_c_A = c2_dzdxdx - c2_dzzdx;
      c2_c_B = c2_dxj;
      c2_h_x = c2_c_A;
      c2_e_y = c2_c_B;
      c2_i_x = c2_h_x;
      c2_f_y = c2_e_y;
      c2_g_y = c2_i_x / c2_f_y;
      c2_pp_data->coefs[c2_joffset + c2_b_i] = c2_g_y;
      c2_pp_data->coefs[(c2_cpage + c2_joffset) + c2_b_i] = 2.0 * c2_dzzdx -
        c2_dzdxdx;
      c2_c_b = c2_cpage;
      c2_h_y = (c2_c_b << 1) - 1;
      c2_pp_data->coefs[((c2_h_y + c2_joffset) + c2_b_i) + 1] =
        c2_s_data[c2_joffset + c2_b_i];
      c2_d_b = c2_cpage;
      c2_i_y = 3 * c2_d_b - 1;
      c2_pp_data->coefs[((c2_i_y + c2_joffset) + c2_b_i) + 1] = c2_y_data
        [(c2_yoffset + c2_joffset) + c2_b_i];
    }
  }
}

static void c2_f_ppval(SFc2_QuanInstanceStruct *chartInstance,
  c2_sL3Gi7vZqpbfvDjMPafkGnG *c2_pp_data, c2_sL3Gi7vZqpbfvDjMPafkGnG_size
  c2_pp_elems_sizes, real_T c2_x, real_T c2_v_data[], int32_T *c2_v_sizes)
{
  int32_T c2_x_sizes[3];
  int32_T c2_b_x;
  int32_T c2_c_x;
  int32_T c2_d_x;
  int32_T c2_loop_ub;
  int32_T c2_i171;
  int32_T c2_elementsPerPage;
  real_T c2_x_data[44];
  int32_T c2_coefStride;
  real_T c2_szv[2];
  int32_T c2_i172;
  int32_T c2_iv0;
  real_T c2_e_x;
  real_T c2_f_x;
  int32_T c2_b_coefStride;
  boolean_T c2_b;
  real_T c2_g_x;
  boolean_T c2_b_b;
  int32_T c2_i173;
  int32_T c2_b_elementsPerPage;
  int32_T c2_c_b;
  int32_T c2_i174;
  real_T c2_v;
  int32_T c2_d_b;
  int32_T c2_ip;
  real_T c2_pp[12];
  boolean_T c2_overflow;
  int32_T c2_icp;
  int32_T c2_b_ip;
  real_T c2_b_pp[12];
  real_T c2_xloc;
  real_T c2_b_xloc;
  int32_T c2_c_elementsPerPage;
  int32_T c2_j;
  int32_T c2_e_b;
  int32_T c2_ic;
  int32_T c2_f_b;
  boolean_T c2_b_overflow;
  int32_T c2_b_j;
  int32_T c2_b_ic;
  int32_T c2_c_j;
  int32_T c2_c_ic;
  int32_T c2_d_ic;
  int32_T c2_ic0;
  int32_T c2_d_elementsPerPage;
  int32_T c2_g_b;
  int32_T c2_h_b;
  boolean_T c2_c_overflow;
  int32_T c2_d_j;
  c2_x_sizes[0] = c2_pp_elems_sizes.coefs[0];
  c2_x_sizes[1] = 11;
  c2_x_sizes[2] = 4;
  c2_b_x = c2_x_sizes[0];
  c2_c_x = c2_x_sizes[1];
  c2_d_x = c2_x_sizes[2];
  c2_loop_ub = c2_pp_elems_sizes.coefs[0] * c2_pp_elems_sizes.coefs[1] *
    c2_pp_elems_sizes.coefs[2] - 1;
  for (c2_i171 = 0; c2_i171 <= c2_loop_ub; c2_i171++) {
    c2_x_data[c2_i171] = c2_pp_data->coefs[c2_i171];
  }

  c2_elementsPerPage = c2_x_sizes[0];
  c2_coefStride = c2_elementsPerPage * 11;
  c2_szv[0] = (real_T)c2_pp_elems_sizes.coefs[0];
  c2_szv[1] = 1.0;
  for (c2_i172 = 0; c2_i172 < 2; c2_i172++) {
    c2_szv[c2_i172] = _SFD_NON_NEGATIVE_CHECK("", c2_szv[c2_i172]);
  }

  *c2_v_sizes = (int32_T)c2_szv[0];
  if (c2_elementsPerPage == 1) {
    c2_e_x = c2_x;
    c2_b_coefStride = c2_coefStride;
    c2_g_x = c2_e_x;
    c2_b_b = muDoubleScalarIsNaN(c2_g_x);
    if (c2_b_b) {
      c2_v = c2_e_x;
    } else {
      for (c2_i174 = 0; c2_i174 < 12; c2_i174++) {
        c2_b_pp[c2_i174] = c2_pp_data->breaks[c2_i174];
      }

      c2_b_ip = c2_bsearch(chartInstance, c2_b_pp, c2_e_x) - 1;
      c2_b_xloc = c2_e_x - c2_pp_data->breaks[c2_b_ip];
      c2_v = c2_pp_data->coefs[c2_b_ip];
      for (c2_ic = 2; c2_ic < 5; c2_ic++) {
        c2_b_ic = c2_ic - 1;
        c2_v = c2_b_xloc * c2_v + c2_pp_data->coefs[c2_b_ip + c2_b_ic *
          c2_b_coefStride];
      }
    }

    c2_v_data[0] = c2_v;
  } else {
    c2_iv0 = -1;
    c2_f_x = c2_x;
    c2_b = muDoubleScalarIsNaN(c2_f_x);
    if (c2_b) {
      c2_b_elementsPerPage = c2_elementsPerPage;
      c2_c_b = c2_b_elementsPerPage;
      c2_d_b = c2_c_b;
      c2_overflow = ((!(1 > c2_d_b)) && (c2_d_b > 2147483646));
      if (c2_overflow) {
        c2_check_forloop_overflow_error(chartInstance, c2_overflow);
      }

      for (c2_j = 1; c2_j <= c2_b_elementsPerPage; c2_j++) {
        c2_b_j = c2_j;
        c2_v_data[c2_iv0 + c2_b_j] = c2_x;
      }
    } else {
      for (c2_i173 = 0; c2_i173 < 12; c2_i173++) {
        c2_pp[c2_i173] = c2_pp_data->breaks[c2_i173];
      }

      c2_ip = c2_bsearch(chartInstance, c2_pp, c2_x) - 1;
      c2_icp = c2_ip * c2_elementsPerPage;
      c2_xloc = c2_x - c2_pp_data->breaks[c2_ip];
      c2_c_elementsPerPage = c2_elementsPerPage;
      c2_e_b = c2_c_elementsPerPage;
      c2_f_b = c2_e_b;
      c2_b_overflow = ((!(1 > c2_f_b)) && (c2_f_b > 2147483646));
      if (c2_b_overflow) {
        c2_check_forloop_overflow_error(chartInstance, c2_b_overflow);
      }

      for (c2_c_j = 1; c2_c_j <= c2_c_elementsPerPage; c2_c_j++) {
        c2_b_j = c2_c_j;
        c2_v_data[c2_iv0 + c2_b_j] = c2_pp_data->coefs[(c2_icp + c2_b_j) - 1];
      }

      for (c2_c_ic = 2; c2_c_ic < 5; c2_c_ic++) {
        c2_d_ic = c2_c_ic - 1;
        c2_ic0 = (c2_icp + c2_d_ic * c2_coefStride) - 1;
        c2_d_elementsPerPage = c2_elementsPerPage;
        c2_g_b = c2_d_elementsPerPage;
        c2_h_b = c2_g_b;
        c2_c_overflow = ((!(1 > c2_h_b)) && (c2_h_b > 2147483646));
        if (c2_c_overflow) {
          c2_check_forloop_overflow_error(chartInstance, c2_c_overflow);
        }

        for (c2_d_j = 1; c2_d_j <= c2_d_elementsPerPage; c2_d_j++) {
          c2_b_j = c2_d_j;
          c2_v_data[c2_iv0 + c2_b_j] = c2_xloc * c2_v_data[c2_iv0 + c2_b_j] +
            c2_pp_data->coefs[c2_ic0 + c2_b_j];
        }
      }
    }
  }
}

static void c2_v_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static real_T c2_k_interp1(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_3)
{
  real_T c2_Vq;
  real_T c2_xi;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_x;
  boolean_T c2_b;
  real_T c2_b_xi;
  c2_sSYDZjSrzjRAmypAou1DUJH c2_r18;
  static c2_sSYDZjSrzjRAmypAou1DUJH c2_r19 = { { -10.0, -5.0, 0.0, 5.0, 10.0,
      15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0 }, { -0.00026588389854893758,
      -0.00026588389854893644, 2.5419492744687133E-5, -2.7794072429810354E-5,
      2.1756796974553572E-5, 6.8766884531592514E-5, -0.00010482433510091953,
      0.00015853045587207738, -5.729748838738946E-5, 0.00012665949767747811,
      0.00012665949767747846, 0.0077482584782340661, 0.0037599999999999934,
      -0.00022825847823406409, 0.00015303391293624292, -0.00026387717351090408,
      6.2474781107396725E-5, 0.00109397804908129, -0.00047838697743249488,
      0.0018995698606486713, 0.0010401075348378305, 0.0029400000000000051,
      -0.13789419492744689, -0.080352902536276555, -0.06269419492744685,
      -0.063070317753935956, -0.0636245340568093, -0.064631546018826827,
      -0.058849281867883421, -0.055771326509639486, -0.048665412093558631,
      -0.033967025116126127, -0.014066487441936964, 0.77, 0.241, -0.1, -0.416,
      -0.731, -1.053, -1.366, -1.646, -1.917, -2.12, -2.248 } };

  int32_T exitg1;
  c2_xi = c2_varargin_3;
  c2_k = 1;
  do {
    exitg1 = 0;
    if (c2_k < 13) {
      c2_c_k = c2_k - 1;
      c2_x = -10.0 + 5.0 * (real_T)c2_c_k;
      c2_b = muDoubleScalarIsNaN(c2_x);
      if (c2_b) {
        c2_d_error(chartInstance);
        exitg1 = 1;
      } else {
        c2_k++;
      }
    } else {
      for (c2_b_k = 2; c2_b_k < 13; c2_b_k++) {
        c2_c_k = c2_b_k - 2;
        if (-10.0 + 5.0 * (real_T)(c2_c_k + 1) <= -10.0 + 5.0 * (real_T)c2_c_k)
        {
          c2_e_error(chartInstance);
        }
      }

      c2_b_xi = c2_xi;
      c2_r18 = c2_r19;
      c2_Vq = c2_ppval(chartInstance, &c2_r18, c2_b_xi);
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return c2_Vq;
}

static real_T c2_e_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_Xq;
  real_T c2_Yq;
  c2_Xq = c2_varargin_4;
  c2_Yq = c2_varargin_5;
  return c2_interp2_dispatch(chartInstance, c2_Xq, c2_Yq);
}

static void c2_c_interp2_validate(SFc2_QuanInstanceStruct *chartInstance,
  uint8_T c2_METHOD)
{
  (void)chartInstance;
  (void)c2_METHOD;
}

static void c2_w_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_x_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static boolean_T c2_b_anyIsNaN(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_y[84])
{
  boolean_T c2_p;
  int32_T c2_k;
  real_T c2_b_k;
  real_T c2_x;
  boolean_T c2_b;
  boolean_T exitg1;
  (void)chartInstance;
  c2_p = false;
  c2_k = 0;
  exitg1 = false;
  while ((exitg1 == false) && (c2_k < 84)) {
    c2_b_k = 1.0 + (real_T)c2_k;
    c2_x = c2_y[(int32_T)c2_b_k - 1];
    c2_b = muDoubleScalarIsNaN(c2_x);
    if (c2_b) {
      c2_p = true;
      exitg1 = true;
    } else {
      c2_k++;
    }
  }

  return c2_p;
}

static real_T c2_interp2_dispatch(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_Xq, real_T c2_Yq)
{
  real_T c2_Vq;
  real_T c2_b_Xq;
  real_T c2_b_Yq;
  real_T c2_varargin_4;
  real_T c2_varargin_5;
  real_T c2_a;
  real_T c2_uxi;
  real_T c2_uxi_data[1];
  int32_T c2_uxi_sizes[2];
  int32_T c2_indx_data[1];
  int32_T c2_indx_sizes;
  int32_T c2_ipos;
  real_T c2_uyi;
  int32_T c2_b_ipos;
  real_T c2_ix;
  real_T c2_b_ix;
  real_T c2_b_a;
  real_T c2_uyi_data[1];
  int32_T c2_uyi_sizes[2];
  int32_T c2_c_ipos;
  int32_T c2_d_ipos;
  real_T c2_iy;
  real_T c2_b_iy;
  real_T c2_uwi_data[1];
  int32_T c2_uwi_sizes[2];
  c2_b_Xq = c2_Xq;
  c2_b_Yq = c2_Yq;
  c2_varargin_4 = c2_b_Yq;
  c2_varargin_5 = c2_b_Xq;
  if (c2_isplaid(chartInstance, c2_varargin_4, c2_varargin_5)) {
    c2_uxi = c2_varargin_4;
    c2_uyi = c2_varargin_5;
    c2_Vq = c2_e_TensorGriddedInterp(chartInstance, c2_uxi, c2_uyi);
  } else {
    c2_a = c2_varargin_4;
    c2_unique_vector(chartInstance, c2_a, c2_uxi_data, c2_uxi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_ipos);
    c2_b_ipos = c2_ipos;
    c2_ix = (real_T)c2_b_ipos;
    c2_b_ix = c2_ix;
    c2_b_a = c2_varargin_5;
    c2_unique_vector(chartInstance, c2_b_a, c2_uyi_data, c2_uyi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_c_ipos);
    c2_d_ipos = c2_c_ipos;
    c2_iy = (real_T)c2_d_ipos;
    c2_b_iy = c2_iy;
    c2_f_TensorGriddedInterp(chartInstance, c2_uxi_data, c2_uxi_sizes,
      c2_uyi_data, c2_uyi_sizes, c2_uwi_data, c2_uwi_sizes);
    c2_Vq = c2_uwi_data[((int32_T)c2_b_ix + c2_uwi_sizes[0] * ((int32_T)c2_b_iy
      - 1)) - 1];
  }

  return c2_Vq;
}

static real_T c2_e_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_xxi;
  int32_T c2_i175;
  real_T c2_varargin_3[84];
  c2_sPMb9Nq525A1JP3SLksK2fC c2_ppk;
  static real_T c2_b_varargin_3[84] = { -0.041, -0.052, -0.053, -0.056, -0.05,
    -0.056, -0.082, -0.059, -0.042, -0.038, -0.027, -0.017, -0.041, -0.053,
    -0.053, -0.053, -0.05, -0.051, -0.066, -0.043, -0.038, -0.027, -0.023,
    -0.016, -0.042, -0.053, -0.052, -0.051, -0.049, -0.049, -0.043, -0.035,
    -0.026, -0.016, -0.018, -0.014, -0.04, -0.052, -0.051, -0.052, -0.048,
    -0.048, -0.042, -0.037, -0.031, -0.026, -0.017, -0.012, -0.043, -0.049,
    -0.048, -0.049, -0.043, -0.042, -0.042, -0.036, -0.025, -0.021, -0.016,
    -0.011, -0.044, -0.048, -0.048, -0.047, -0.042, -0.041, -0.02, -0.028,
    -0.013, -0.014, -0.011, -0.01, -0.043, -0.049, -0.047, -0.045, -0.042,
    -0.037, -0.003, -0.013, -0.01, -0.003, -0.007, -0.008 };

  int32_T c2_i176;
  real_T c2_vkj[12];
  real_T c2_varargout_1[12];
  int32_T c2_i;
  real_T c2_b_i;
  c2_sSYDZjSrzjRAmypAou1DUJH c2_b_ppk;
  real_T c2_b_vkj;
  c2_xxi = c2_varargin_5;
  for (c2_i175 = 0; c2_i175 < 84; c2_i175++) {
    c2_varargin_3[c2_i175] = c2_b_varargin_3[c2_i175];
  }

  c2_e_spline(chartInstance, c2_varargin_3, &c2_ppk);
  for (c2_i176 = 0; c2_i176 < 12; c2_i176++) {
    c2_varargout_1[c2_i176] = 0.0;
  }

  c2_g_ppval(chartInstance, &c2_ppk, c2_xxi, c2_vkj);
  for (c2_i = 0; c2_i < 12; c2_i++) {
    c2_b_i = 1.0 + (real_T)c2_i;
    c2_varargout_1[(int32_T)(1.0 + (c2_b_i - 1.0)) - 1] = c2_vkj[(int32_T)c2_b_i
      - 1];
  }

  c2_xxi = c2_varargin_4;
  c2_c_spline(chartInstance, c2_varargout_1, &c2_b_ppk);
  c2_b_vkj = c2_ppval(chartInstance, &c2_b_ppk, c2_xxi);
  return c2_b_vkj;
}

static void c2_e_spline(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[84],
  c2_sPMb9Nq525A1JP3SLksK2fC *c2_output)
{
  int32_T c2_i177;
  real_T c2_b_y[84];
  for (c2_i177 = 0; c2_i177 < 84; c2_i177++) {
    c2_b_y[c2_i177] = c2_y[c2_i177];
  }

  c2_e_splinepp(chartInstance, c2_b_y, c2_output);
}

static void c2_e_splinepp(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[84],
  c2_sPMb9Nq525A1JP3SLksK2fC *c2_pp)
{
  int32_T c2_i178;
  real_T c2_b_y[84];
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_dx[6];
  int32_T c2_dpg;
  real_T c2_d1;
  int32_T c2_pg;
  real_T c2_d2;
  int32_T c2_pgp1;
  int32_T c2_j;
  int32_T c2_pgm1;
  int32_T c2_b_j;
  int32_T c2_c_j;
  int32_T c2_d_j;
  real_T c2_A;
  real_T c2_dvdf[72];
  real_T c2_b_A;
  int32_T c2_e_j;
  real_T c2_x;
  real_T c2_B;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_c_y;
  real_T c2_s[84];
  real_T c2_d_y;
  real_T c2_md[7];
  real_T c2_d_x;
  real_T c2_c_A;
  real_T c2_e_y;
  int32_T c2_d_k;
  real_T c2_e_x;
  real_T c2_f_y;
  real_T c2_f_x;
  real_T c2_g_y;
  real_T c2_d_A;
  real_T c2_b_B;
  real_T c2_g_x;
  real_T c2_h_y;
  real_T c2_h_x;
  real_T c2_i_y;
  real_T c2_r;
  int32_T c2_f_j;
  int32_T c2_e_k;
  real_T c2_c_B;
  real_T c2_j_y;
  real_T c2_e_A;
  real_T c2_k_y;
  real_T c2_d_B;
  real_T c2_i_x;
  real_T c2_l_y;
  int32_T c2_g_j;
  real_T c2_j_x;
  real_T c2_m_y;
  int32_T c2_h_j;
  int32_T c2_f_k;
  int32_T c2_i_j;
  real_T c2_f_A;
  real_T c2_e_B;
  int32_T c2_j_j;
  real_T c2_k_x;
  real_T c2_n_y;
  real_T c2_l_x;
  int32_T c2_i179;
  real_T c2_o_y;
  real_T c2_g_A;
  int32_T c2_k_j;
  real_T c2_p_y;
  real_T c2_f_B;
  int32_T c2_i180;
  real_T c2_dv29[7];
  real_T c2_m_x;
  real_T c2_q_y;
  real_T c2_n_x;
  real_T c2_h_A;
  real_T c2_r_y[84];
  real_T c2_s_y;
  real_T c2_g_B;
  real_T c2_t_y;
  real_T c2_o_x;
  real_T c2_u_y;
  real_T c2_p_x;
  real_T c2_v_y;
  real_T c2_w_y;
  for (c2_i178 = 0; c2_i178 < 84; c2_i178++) {
    c2_b_y[c2_i178] = c2_y[c2_i178];
  }

  c2_g_chckxy(chartInstance, c2_b_y);
  for (c2_k = 1; c2_k < 7; c2_k++) {
    c2_c_k = c2_k - 1;
    c2_dx[c2_c_k] = (-30.0 + 10.0 * (real_T)(c2_c_k + 1)) - (-30.0 + 10.0 *
      (real_T)c2_c_k);
    c2_dpg = c2_c_k * 12;
    c2_pg = c2_dpg;
    c2_pgp1 = (c2_c_k + 1) * 12;
    for (c2_b_j = 1; c2_b_j < 13; c2_b_j++) {
      c2_c_j = c2_b_j - 1;
      c2_b_A = c2_y[c2_pgp1 + c2_c_j] - c2_y[c2_pg + c2_c_j];
      c2_B = c2_dx[c2_c_k];
      c2_c_x = c2_b_A;
      c2_d_y = c2_B;
      c2_d_x = c2_c_x;
      c2_e_y = c2_d_y;
      c2_f_y = c2_d_x / c2_e_y;
      c2_dvdf[c2_dpg + c2_c_j] = c2_f_y;
    }
  }

  for (c2_b_k = 2; c2_b_k < 7; c2_b_k++) {
    c2_c_k = c2_b_k - 2;
    c2_pg = (c2_c_k + 1) * 12;
    c2_pgm1 = c2_c_k * 12;
    c2_d1 = c2_dx[c2_c_k + 1];
    c2_d2 = c2_dx[c2_c_k];
    for (c2_d_j = 1; c2_d_j < 13; c2_d_j++) {
      c2_c_j = c2_d_j - 1;
      c2_s[c2_pg + c2_c_j] = 3.0 * (c2_d1 * c2_dvdf[c2_pgm1 + c2_c_j] + c2_d2 *
        c2_dvdf[c2_pg + c2_c_j]);
    }
  }

  c2_d1 = c2_dx[0];
  c2_d2 = c2_dx[1];
  for (c2_j = 1; c2_j < 13; c2_j++) {
    c2_c_j = c2_j - 1;
    c2_A = (c2_d1 + 40.0) * c2_d2 * c2_dvdf[c2_c_j] + c2_d1 * c2_d1 *
      c2_dvdf[c2_c_j + 12];
    c2_x = c2_A;
    c2_b_x = c2_x;
    c2_c_y = c2_b_x / 20.0;
    c2_s[c2_c_j] = c2_c_y;
  }

  c2_d1 = c2_dx[5];
  c2_d2 = c2_dx[4];
  for (c2_e_j = 1; c2_e_j < 13; c2_e_j++) {
    c2_c_j = c2_e_j + 47;
    c2_c_A = (c2_d1 + 40.0) * c2_d2 * c2_dvdf[c2_c_j + 12] + c2_d1 * c2_d1 *
      c2_dvdf[c2_c_j];
    c2_e_x = c2_c_A;
    c2_f_x = c2_e_x;
    c2_g_y = c2_f_x / 20.0;
    c2_s[c2_c_j + 24] = c2_g_y;
  }

  c2_md[0] = c2_dx[1];
  c2_md[6] = c2_dx[4];
  for (c2_d_k = 2; c2_d_k < 7; c2_d_k++) {
    c2_c_k = c2_d_k - 1;
    c2_md[c2_c_k] = 2.0 * (c2_dx[c2_c_k] + c2_dx[c2_c_k - 1]);
  }

  c2_d_A = c2_dx[1];
  c2_b_B = c2_md[0];
  c2_g_x = c2_d_A;
  c2_h_y = c2_b_B;
  c2_h_x = c2_g_x;
  c2_i_y = c2_h_y;
  c2_r = c2_h_x / c2_i_y;
  c2_md[1] -= c2_r * 20.0;
  for (c2_f_j = 1; c2_f_j < 13; c2_f_j++) {
    c2_c_j = c2_f_j + 11;
    c2_s[c2_c_j] -= c2_r * c2_s[c2_c_j - 12];
  }

  for (c2_e_k = 3; c2_e_k < 7; c2_e_k++) {
    c2_c_k = c2_e_k - 1;
    c2_e_A = c2_dx[c2_c_k];
    c2_d_B = c2_md[c2_c_k - 1];
    c2_i_x = c2_e_A;
    c2_l_y = c2_d_B;
    c2_j_x = c2_i_x;
    c2_m_y = c2_l_y;
    c2_r = c2_j_x / c2_m_y;
    c2_md[c2_c_k] -= c2_r * c2_dx[c2_c_k - 2];
    c2_pg = c2_c_k * 12;
    c2_pgm1 = (c2_c_k - 1) * 12;
    for (c2_i_j = 1; c2_i_j < 13; c2_i_j++) {
      c2_c_j = c2_i_j - 1;
      c2_s[c2_pg + c2_c_j] -= c2_r * c2_s[c2_pgm1 + c2_c_j];
    }
  }

  c2_c_B = c2_md[5];
  c2_j_y = c2_c_B;
  c2_k_y = c2_j_y;
  c2_r = 20.0 / c2_k_y;
  c2_md[6] -= c2_r * c2_dx[4];
  for (c2_g_j = 1; c2_g_j < 13; c2_g_j++) {
    c2_c_j = c2_g_j + 71;
    c2_s[c2_c_j] -= c2_r * c2_s[c2_c_j - 12];
  }

  for (c2_h_j = 1; c2_h_j < 13; c2_h_j++) {
    c2_c_j = c2_h_j + 71;
    c2_f_A = c2_s[c2_c_j];
    c2_e_B = c2_md[6];
    c2_k_x = c2_f_A;
    c2_n_y = c2_e_B;
    c2_l_x = c2_k_x;
    c2_o_y = c2_n_y;
    c2_p_y = c2_l_x / c2_o_y;
    c2_s[c2_c_j] = c2_p_y;
  }

  for (c2_f_k = 6; c2_f_k > 1; c2_f_k--) {
    c2_c_k = c2_f_k - 1;
    c2_pg = c2_c_k * 12;
    c2_pgp1 = (c2_c_k + 1) * 12;
    c2_d1 = c2_dx[c2_c_k - 1];
    for (c2_k_j = 1; c2_k_j < 13; c2_k_j++) {
      c2_c_j = c2_k_j - 1;
      c2_h_A = c2_s[c2_pg + c2_c_j] - c2_d1 * c2_s[c2_pgp1 + c2_c_j];
      c2_g_B = c2_md[c2_c_k];
      c2_o_x = c2_h_A;
      c2_u_y = c2_g_B;
      c2_p_x = c2_o_x;
      c2_v_y = c2_u_y;
      c2_w_y = c2_p_x / c2_v_y;
      c2_s[c2_pg + c2_c_j] = c2_w_y;
    }
  }

  for (c2_j_j = 1; c2_j_j < 13; c2_j_j++) {
    c2_c_j = c2_j_j - 1;
    c2_g_A = c2_s[c2_c_j] - 20.0 * c2_s[c2_c_j + 12];
    c2_f_B = c2_md[0];
    c2_m_x = c2_g_A;
    c2_q_y = c2_f_B;
    c2_n_x = c2_m_x;
    c2_s_y = c2_q_y;
    c2_t_y = c2_n_x / c2_s_y;
    c2_s[c2_c_j] = c2_t_y;
  }

  for (c2_i179 = 0; c2_i179 < 7; c2_i179++) {
    c2_dv29[c2_i179] = -30.0 + 10.0 * (real_T)c2_i179;
  }

  for (c2_i180 = 0; c2_i180 < 84; c2_i180++) {
    c2_r_y[c2_i180] = c2_y[c2_i180];
  }

  c2_g_pwchcore(chartInstance, c2_dv29, c2_r_y, c2_s, c2_dx, c2_dvdf, c2_pp);
}

static void c2_g_chckxy(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[84])
{
  int32_T c2_i181;
  real_T c2_b_y[84];
  const mxArray *c2_c_y = NULL;
  static char_T c2_u[28] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'U', 'n', 's', 'u', 'p', 'p', 'o', 'r', 't', 'e', 'd',
    'N', 'a', 'N' };

  for (c2_i181 = 0; c2_i181 < 84; c2_i181++) {
    c2_b_y[c2_i181] = c2_y[c2_i181];
  }

  if (!c2_b_anyIsNaN(chartInstance, c2_b_y)) {
  } else {
    c2_c_y = NULL;
    sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 28),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c2_c_y));
  }
}

static void c2_y_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_g_pwchcore(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x[7],
  real_T c2_y[84], real_T c2_s[84], real_T c2_dx[6], real_T c2_divdif[72],
  c2_sPMb9Nq525A1JP3SLksK2fC *c2_pp)
{
  int32_T c2_i182;
  int32_T c2_j;
  int32_T c2_b_j;
  real_T c2_dxj;
  int32_T c2_joffset;
  int32_T c2_i;
  int32_T c2_b_i;
  real_T c2_divdifij;
  real_T c2_A;
  real_T c2_B;
  real_T c2_b_x;
  real_T c2_b_y;
  real_T c2_c_x;
  real_T c2_c_y;
  real_T c2_dzzdx;
  real_T c2_b_A;
  real_T c2_b_B;
  real_T c2_d_x;
  real_T c2_d_y;
  real_T c2_e_x;
  real_T c2_e_y;
  real_T c2_dzdxdx;
  real_T c2_c_A;
  real_T c2_c_B;
  real_T c2_f_x;
  real_T c2_f_y;
  real_T c2_g_x;
  real_T c2_g_y;
  real_T c2_h_y;
  (void)chartInstance;
  for (c2_i182 = 0; c2_i182 < 7; c2_i182++) {
    c2_pp->breaks[c2_i182] = c2_x[c2_i182];
  }

  for (c2_j = 1; c2_j < 7; c2_j++) {
    c2_b_j = c2_j - 1;
    c2_dxj = c2_dx[c2_b_j];
    c2_joffset = c2_b_j * 12;
    for (c2_i = 1; c2_i < 13; c2_i++) {
      c2_b_i = c2_i - 1;
      c2_divdifij = c2_divdif[c2_joffset + c2_b_i];
      c2_A = c2_divdifij - c2_s[c2_joffset + c2_b_i];
      c2_B = c2_dxj;
      c2_b_x = c2_A;
      c2_b_y = c2_B;
      c2_c_x = c2_b_x;
      c2_c_y = c2_b_y;
      c2_dzzdx = c2_c_x / c2_c_y;
      c2_b_A = c2_s[(c2_joffset + c2_b_i) + 12] - c2_divdifij;
      c2_b_B = c2_dxj;
      c2_d_x = c2_b_A;
      c2_d_y = c2_b_B;
      c2_e_x = c2_d_x;
      c2_e_y = c2_d_y;
      c2_dzdxdx = c2_e_x / c2_e_y;
      c2_c_A = c2_dzdxdx - c2_dzzdx;
      c2_c_B = c2_dxj;
      c2_f_x = c2_c_A;
      c2_f_y = c2_c_B;
      c2_g_x = c2_f_x;
      c2_g_y = c2_f_y;
      c2_h_y = c2_g_x / c2_g_y;
      c2_pp->coefs[c2_joffset + c2_b_i] = c2_h_y;
      c2_pp->coefs[(c2_joffset + c2_b_i) + 72] = 2.0 * c2_dzzdx - c2_dzdxdx;
      c2_pp->coefs[(c2_joffset + c2_b_i) + 144] = c2_s[c2_joffset + c2_b_i];
      c2_pp->coefs[(c2_joffset + c2_b_i) + 216] = c2_y[c2_joffset + c2_b_i];
    }
  }
}

static void c2_ab_scalarEg(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_g_ppval(SFc2_QuanInstanceStruct *chartInstance,
  c2_sPMb9Nq525A1JP3SLksK2fC *c2_pp, real_T c2_x, real_T c2_v[12])
{
  real_T c2_b_x;
  boolean_T c2_b;
  int32_T c2_i183;
  int32_T c2_j;
  real_T c2_xi;
  real_T c2_c_x[7];
  int32_T c2_b_j;
  int32_T c2_low_i;
  int32_T c2_low_ip1;
  int32_T c2_high_i;
  int32_T c2_ip;
  int32_T c2_b_low_i;
  int32_T c2_icp;
  int32_T c2_b_high_i;
  real_T c2_xloc;
  int32_T c2_mid_i;
  int32_T c2_c_j;
  int32_T c2_ic;
  int32_T c2_b_ic;
  int32_T c2_ic0;
  int32_T c2_d_j;
  (void)chartInstance;
  c2_b_x = c2_x;
  c2_b = muDoubleScalarIsNaN(c2_b_x);
  if (c2_b) {
    for (c2_j = 1; c2_j < 13; c2_j++) {
      c2_b_j = c2_j - 1;
      c2_v[c2_b_j] = c2_x;
    }
  } else {
    for (c2_i183 = 0; c2_i183 < 7; c2_i183++) {
      c2_c_x[c2_i183] = c2_pp->breaks[c2_i183];
    }

    c2_xi = c2_x;
    c2_low_i = 1;
    c2_low_ip1 = 1;
    c2_high_i = 7;
    while (c2_high_i > c2_low_ip1 + 1) {
      c2_b_low_i = c2_low_i;
      c2_b_high_i = c2_high_i;
      c2_mid_i = (c2_b_low_i + c2_b_high_i) >> 1;
      if (c2_xi >= c2_c_x[c2_mid_i - 1]) {
        c2_low_i = c2_mid_i;
        c2_low_ip1 = c2_low_i;
      } else {
        c2_high_i = c2_mid_i;
      }
    }

    c2_ip = c2_low_i - 1;
    c2_icp = c2_ip * 12;
    c2_xloc = c2_x - c2_pp->breaks[c2_ip];
    for (c2_c_j = 1; c2_c_j < 13; c2_c_j++) {
      c2_b_j = c2_c_j - 1;
      c2_v[c2_b_j] = c2_pp->coefs[c2_icp + c2_b_j];
    }

    for (c2_ic = 2; c2_ic < 5; c2_ic++) {
      c2_b_ic = c2_ic - 1;
      c2_ic0 = (c2_icp + c2_b_ic * 72) - 1;
      for (c2_d_j = 1; c2_d_j < 13; c2_d_j++) {
        c2_b_j = c2_d_j - 1;
        c2_v[c2_b_j] = c2_xloc * c2_v[c2_b_j] + c2_pp->coefs[(c2_ic0 + c2_b_j) +
          1];
      }
    }
  }
}

static void c2_f_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4_data[], int32_T c2_varargin_4_sizes[2], real_T
  c2_varargin_5_data[], int32_T c2_varargin_5_sizes[2], real_T c2_yi_data[],
  int32_T c2_yi_sizes[2])
{
  int32_T c2_b_varargin_4_sizes[2];
  int32_T c2_varargin_4;
  int32_T c2_b_varargin_4;
  int32_T c2_loop_ub;
  int32_T c2_i184;
  int32_T c2_b_varargin_5_sizes[2];
  real_T c2_b_varargin_4_data[1];
  int32_T c2_varargin_5;
  int32_T c2_b_varargin_5;
  int32_T c2_b_loop_ub;
  int32_T c2_i185;
  real_T c2_b_varargin_5_data[1];
  real_T c2_outsize[2];
  int32_T c2_xxi_sizes[2];
  int32_T c2_xxi;
  int32_T c2_b_xxi;
  int32_T c2_c_loop_ub;
  int32_T c2_i186;
  real_T c2_nxxi;
  real_T c2_xxi_data[1];
  int32_T c2_i187;
  real_T c2_varargin_3[84];
  c2_sPMb9Nq525A1JP3SLksK2fC c2_ppk;
  static real_T c2_b_varargin_3[84] = { -0.041, -0.052, -0.053, -0.056, -0.05,
    -0.056, -0.082, -0.059, -0.042, -0.038, -0.027, -0.017, -0.041, -0.053,
    -0.053, -0.053, -0.05, -0.051, -0.066, -0.043, -0.038, -0.027, -0.023,
    -0.016, -0.042, -0.053, -0.052, -0.051, -0.049, -0.049, -0.043, -0.035,
    -0.026, -0.016, -0.018, -0.014, -0.04, -0.052, -0.051, -0.052, -0.048,
    -0.048, -0.042, -0.037, -0.031, -0.026, -0.017, -0.012, -0.043, -0.049,
    -0.048, -0.049, -0.043, -0.042, -0.042, -0.036, -0.025, -0.021, -0.016,
    -0.011, -0.044, -0.048, -0.048, -0.047, -0.042, -0.041, -0.02, -0.028,
    -0.013, -0.014, -0.011, -0.01, -0.043, -0.049, -0.047, -0.045, -0.042,
    -0.037, -0.003, -0.013, -0.01, -0.003, -0.007, -0.008 };

  int32_T c2_c_varargin_5_sizes[2];
  int32_T c2_c_varargin_5;
  int32_T c2_d_varargin_5;
  int32_T c2_d_loop_ub;
  int32_T c2_i188;
  real_T c2_c_varargin_5_data[1];
  real_T c2_sv[2];
  int32_T c2_i189;
  int32_T c2_varargout_1_sizes[2];
  int32_T c2_varargout_1;
  int32_T c2_b_varargout_1;
  int32_T c2_e_loop_ub;
  int32_T c2_i190;
  real_T c2_b_nxxi;
  real_T c2_varargout_1_data[12];
  int32_T c2_i191;
  int32_T c2_j;
  real_T c2_b_j;
  c2_sPMb9Nq525A1JP3SLksK2fC c2_b_ppk;
  int32_T c2_c_xxi;
  int32_T c2_d_xxi[1];
  int32_T c2_e_xxi;
  real_T c2_vkj[12];
  int32_T c2_f_loop_ub;
  int32_T c2_i;
  int32_T c2_i192;
  real_T c2_b_i;
  c2_sL3Gi7vZqpbfvDjMPafkGnG c2_ppk_data;
  c2_sL3Gi7vZqpbfvDjMPafkGnG_size c2_ppk_elems_sizes;
  int32_T c2_i193;
  int32_T c2_yi;
  int32_T c2_b_yi;
  int32_T c2_g_loop_ub;
  int32_T c2_i194;
  real_T c2_c_nxxi;
  int32_T c2_i195;
  int32_T c2_c_j;
  c2_sL3Gi7vZqpbfvDjMPafkGnG_size c2_b_ppk_elems_sizes;
  c2_sL3Gi7vZqpbfvDjMPafkGnG c2_b_ppk_data;
  int32_T c2_f_xxi[1];
  real_T c2_vkj_data[1];
  int32_T c2_vkj_sizes;
  real_T c2_d9;
  int32_T c2_i196;
  int32_T c2_c_i;
  c2_b_varargin_4_sizes[0] = 1;
  c2_b_varargin_4_sizes[1] = c2_varargin_4_sizes[1];
  c2_varargin_4 = c2_b_varargin_4_sizes[0];
  c2_b_varargin_4 = c2_b_varargin_4_sizes[1];
  c2_loop_ub = c2_varargin_4_sizes[0] * c2_varargin_4_sizes[1] - 1;
  for (c2_i184 = 0; c2_i184 <= c2_loop_ub; c2_i184++) {
    c2_b_varargin_4_data[c2_i184] = c2_varargin_4_data[c2_i184];
  }

  c2_b_varargin_5_sizes[0] = 1;
  c2_b_varargin_5_sizes[1] = c2_varargin_5_sizes[1];
  c2_varargin_5 = c2_b_varargin_5_sizes[0];
  c2_b_varargin_5 = c2_b_varargin_5_sizes[1];
  c2_b_loop_ub = c2_varargin_5_sizes[0] * c2_varargin_5_sizes[1] - 1;
  for (c2_i185 = 0; c2_i185 <= c2_b_loop_ub; c2_i185++) {
    c2_b_varargin_5_data[c2_i185] = c2_varargin_5_data[c2_i185];
  }

  c2_output_size(chartInstance, c2_b_varargin_4_data, c2_b_varargin_4_sizes,
                 c2_b_varargin_5_data, c2_b_varargin_5_sizes, c2_outsize);
  c2_xxi_sizes[0] = 1;
  c2_xxi_sizes[1] = c2_varargin_5_sizes[1];
  c2_xxi = c2_xxi_sizes[0];
  c2_b_xxi = c2_xxi_sizes[1];
  c2_c_loop_ub = c2_varargin_5_sizes[0] * c2_varargin_5_sizes[1] - 1;
  for (c2_i186 = 0; c2_i186 <= c2_c_loop_ub; c2_i186++) {
    c2_xxi_data[c2_i186] = c2_varargin_5_data[c2_i186];
  }

  c2_nxxi = (real_T)c2_xxi_sizes[1];
  for (c2_i187 = 0; c2_i187 < 84; c2_i187++) {
    c2_varargin_3[c2_i187] = c2_b_varargin_3[c2_i187];
  }

  c2_e_spline(chartInstance, c2_varargin_3, &c2_ppk);
  c2_c_varargin_5_sizes[0] = 1;
  c2_c_varargin_5_sizes[1] = c2_varargin_5_sizes[1];
  c2_c_varargin_5 = c2_c_varargin_5_sizes[0];
  c2_d_varargin_5 = c2_c_varargin_5_sizes[1];
  c2_d_loop_ub = c2_varargin_5_sizes[0] * c2_varargin_5_sizes[1] - 1;
  for (c2_i188 = 0; c2_i188 <= c2_d_loop_ub; c2_i188++) {
    c2_c_varargin_5_data[c2_i188] = c2_varargin_5_data[c2_i188];
  }

  c2_intermediate_size(chartInstance, c2_c_varargin_5_data,
                       c2_c_varargin_5_sizes, c2_sv);
  for (c2_i189 = 0; c2_i189 < 2; c2_i189++) {
    c2_sv[c2_i189] = _SFD_NON_NEGATIVE_CHECK("", c2_sv[c2_i189]);
  }

  c2_varargout_1_sizes[0] = (int32_T)c2_sv[0];
  c2_varargout_1_sizes[1] = 12;
  c2_varargout_1 = c2_varargout_1_sizes[0];
  c2_b_varargout_1 = c2_varargout_1_sizes[1];
  c2_e_loop_ub = (int32_T)c2_sv[0] * (int32_T)c2_sv[1] - 1;
  for (c2_i190 = 0; c2_i190 <= c2_e_loop_ub; c2_i190++) {
    c2_varargout_1_data[c2_i190] = 0.0;
  }

  c2_b_nxxi = c2_nxxi;
  c2_i191 = (int32_T)c2_b_nxxi;
  _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_b_nxxi, mxDOUBLE_CLASS, c2_i191);
  for (c2_j = 0; c2_j < c2_i191; c2_j++) {
    c2_b_j = 1.0 + (real_T)c2_j;
    c2_b_ppk = c2_ppk;
    c2_d_xxi[0] = c2_xxi_sizes[1];
    c2_g_ppval(chartInstance, &c2_b_ppk, c2_xxi_data[(int32_T)c2_b_j - 1],
               c2_vkj);
    for (c2_i = 0; c2_i < 12; c2_i++) {
      c2_b_i = 1.0 + (real_T)c2_i;
      c2_varargout_1_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
        c2_vkj[(int32_T)c2_b_i - 1];
    }
  }

  c2_xxi_sizes[0] = 1;
  c2_xxi_sizes[1] = c2_varargin_4_sizes[1];
  c2_c_xxi = c2_xxi_sizes[0];
  c2_e_xxi = c2_xxi_sizes[1];
  c2_f_loop_ub = c2_varargin_4_sizes[0] * c2_varargin_4_sizes[1] - 1;
  for (c2_i192 = 0; c2_i192 <= c2_f_loop_ub; c2_i192++) {
    c2_xxi_data[c2_i192] = c2_varargin_4_data[c2_i192];
  }

  c2_nxxi = (real_T)c2_xxi_sizes[1];
  c2_d_spline(chartInstance, c2_varargout_1_data, c2_varargout_1_sizes,
              &c2_ppk_data, &c2_ppk_elems_sizes);
  for (c2_i193 = 0; c2_i193 < 2; c2_i193++) {
    c2_outsize[c2_i193] = _SFD_NON_NEGATIVE_CHECK("", c2_outsize[c2_i193]);
  }

  c2_yi_sizes[0] = (int32_T)c2_outsize[0];
  c2_yi_sizes[1] = (int32_T)c2_outsize[1];
  c2_yi = c2_yi_sizes[0];
  c2_b_yi = c2_yi_sizes[1];
  c2_g_loop_ub = (int32_T)c2_outsize[0] * (int32_T)c2_outsize[1] - 1;
  for (c2_i194 = 0; c2_i194 <= c2_g_loop_ub; c2_i194++) {
    c2_yi_data[c2_i194] = 0.0;
  }

  c2_c_nxxi = c2_nxxi;
  c2_i195 = (int32_T)c2_c_nxxi;
  _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_c_nxxi, mxDOUBLE_CLASS, c2_i195);
  for (c2_c_j = 0; c2_c_j < c2_i195; c2_c_j++) {
    c2_b_j = 1.0 + (real_T)c2_c_j;
    c2_b_ppk_elems_sizes = c2_ppk_elems_sizes;
    c2_b_ppk_data = c2_ppk_data;
    c2_f_xxi[0] = c2_xxi_sizes[1];
    c2_f_ppval(chartInstance, &c2_b_ppk_data, c2_b_ppk_elems_sizes, c2_xxi_data
               [(int32_T)c2_b_j - 1], c2_vkj_data, &c2_vkj_sizes);
    c2_d9 = (real_T)c2_vkj_sizes;
    c2_i196 = (int32_T)c2_d9;
    _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_d9, mxDOUBLE_CLASS, c2_i196);
    for (c2_c_i = 0; c2_c_i < c2_i196; c2_c_i++) {
      c2_b_i = 1.0 + (real_T)c2_c_i;
      c2_yi_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
        c2_vkj_data[(int32_T)c2_b_i - 1];
    }
  }
}

static void c2_intermediate_size(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_b_data[], int32_T c2_b_sizes[2], real_T c2_sz[2])
{
  (void)chartInstance;
  (void)c2_b_data;
  c2_sz[0] = (real_T)c2_b_sizes[1];
  c2_sz[1] = 12.0;
}

static real_T c2_f_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_Xq;
  real_T c2_Yq;
  c2_Xq = c2_varargin_4;
  c2_Yq = c2_varargin_5;
  return c2_b_interp2_dispatch(chartInstance, c2_Xq, c2_Yq);
}

static real_T c2_g_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_3[84], real_T c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_xxi;
  int32_T c2_i197;
  real_T c2_b_varargin_3[84];
  c2_sPMb9Nq525A1JP3SLksK2fC c2_ppk;
  int32_T c2_i198;
  real_T c2_vkj[12];
  real_T c2_varargout_1[12];
  int32_T c2_i;
  real_T c2_b_i;
  c2_sSYDZjSrzjRAmypAou1DUJH c2_b_ppk;
  real_T c2_b_vkj;
  c2_xxi = c2_varargin_5;
  for (c2_i197 = 0; c2_i197 < 84; c2_i197++) {
    c2_b_varargin_3[c2_i197] = c2_varargin_3[c2_i197];
  }

  c2_e_spline(chartInstance, c2_b_varargin_3, &c2_ppk);
  for (c2_i198 = 0; c2_i198 < 12; c2_i198++) {
    c2_varargout_1[c2_i198] = 0.0;
  }

  c2_g_ppval(chartInstance, &c2_ppk, c2_xxi, c2_vkj);
  for (c2_i = 0; c2_i < 12; c2_i++) {
    c2_b_i = 1.0 + (real_T)c2_i;
    c2_varargout_1[(int32_T)(1.0 + (c2_b_i - 1.0)) - 1] = c2_vkj[(int32_T)c2_b_i
      - 1];
  }

  c2_xxi = c2_varargin_4;
  c2_c_spline(chartInstance, c2_varargout_1, &c2_b_ppk);
  c2_b_vkj = c2_ppval(chartInstance, &c2_b_ppk, c2_xxi);
  return c2_b_vkj;
}

static real_T c2_b_interp2_dispatch(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_Xq, real_T c2_Yq)
{
  real_T c2_Vq;
  real_T c2_b_Xq;
  real_T c2_b_Yq;
  real_T c2_varargin_4;
  real_T c2_varargin_5;
  real_T c2_a;
  real_T c2_uxi;
  real_T c2_uxi_data[1];
  int32_T c2_uxi_sizes[2];
  int32_T c2_indx_data[1];
  int32_T c2_indx_sizes;
  int32_T c2_ipos;
  real_T c2_uyi;
  int32_T c2_b_ipos;
  int32_T c2_i199;
  real_T c2_ix;
  real_T c2_b_ix;
  real_T c2_b_a;
  real_T c2_dv30[84];
  static real_T c2_dv31[84] = { 0.005, 0.017, 0.014, 0.01, -0.005, 0.009, 0.019,
    0.005, 0.0, -0.005, -0.011, 0.008, 0.007, 0.016, 0.014, 0.014, 0.013, 0.009,
    0.012, 0.005, 0.0, 0.004, 0.009, 0.007, 0.013, 0.013, 0.011, 0.012, 0.011,
    0.009, 0.008, 0.005, -0.002, 0.005, 0.003, 0.005, 0.018, 0.015, 0.015, 0.014,
    0.014, 0.014, 0.014, 0.015, 0.013, 0.011, 0.006, 0.001, 0.015, 0.014, 0.013,
    0.013, 0.012, 0.011, 0.011, 0.01, 0.008, 0.008, 0.007, 0.003, 0.021, 0.011,
    0.01, 0.011, 0.01, 0.009, 0.008, 0.01, 0.006, 0.005, 0.0, 0.001, 0.023, 0.01,
    0.011, 0.011, 0.011, 0.01, 0.008, 0.01, 0.006, 0.014, 0.02, 0.0 };

  real_T c2_uyi_data[1];
  int32_T c2_uyi_sizes[2];
  int32_T c2_c_ipos;
  int32_T c2_d_ipos;
  real_T c2_iy;
  real_T c2_b_iy;
  real_T c2_uwi_data[1];
  int32_T c2_uwi_sizes[2];
  c2_b_Xq = c2_Xq;
  c2_b_Yq = c2_Yq;
  c2_varargin_4 = c2_b_Yq;
  c2_varargin_5 = c2_b_Xq;
  if (c2_isplaid(chartInstance, c2_varargin_4, c2_varargin_5)) {
    c2_uxi = c2_varargin_4;
    c2_uyi = c2_varargin_5;
    for (c2_i199 = 0; c2_i199 < 84; c2_i199++) {
      c2_dv30[c2_i199] = c2_dv31[c2_i199];
    }

    c2_Vq = c2_g_TensorGriddedInterp(chartInstance, c2_dv30, c2_uxi, c2_uyi);
  } else {
    c2_a = c2_varargin_4;
    c2_unique_vector(chartInstance, c2_a, c2_uxi_data, c2_uxi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_ipos);
    c2_b_ipos = c2_ipos;
    c2_ix = (real_T)c2_b_ipos;
    c2_b_ix = c2_ix;
    c2_b_a = c2_varargin_5;
    c2_unique_vector(chartInstance, c2_b_a, c2_uyi_data, c2_uyi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_c_ipos);
    c2_d_ipos = c2_c_ipos;
    c2_iy = (real_T)c2_d_ipos;
    c2_b_iy = c2_iy;
    c2_h_TensorGriddedInterp(chartInstance, c2_uxi_data, c2_uxi_sizes,
      c2_uyi_data, c2_uyi_sizes, c2_uwi_data, c2_uwi_sizes);
    c2_Vq = c2_uwi_data[((int32_T)c2_b_ix + c2_uwi_sizes[0] * ((int32_T)c2_b_iy
      - 1)) - 1];
  }

  return c2_Vq;
}

static void c2_h_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4_data[], int32_T c2_varargin_4_sizes[2], real_T
  c2_varargin_5_data[], int32_T c2_varargin_5_sizes[2], real_T c2_yi_data[],
  int32_T c2_yi_sizes[2])
{
  int32_T c2_b_varargin_4_sizes[2];
  int32_T c2_varargin_4;
  int32_T c2_b_varargin_4;
  int32_T c2_loop_ub;
  int32_T c2_i200;
  int32_T c2_b_varargin_5_sizes[2];
  real_T c2_b_varargin_4_data[1];
  int32_T c2_varargin_5;
  int32_T c2_b_varargin_5;
  int32_T c2_b_loop_ub;
  int32_T c2_i201;
  real_T c2_b_varargin_5_data[1];
  real_T c2_outsize[2];
  int32_T c2_xxi_sizes[2];
  int32_T c2_xxi;
  int32_T c2_b_xxi;
  int32_T c2_c_loop_ub;
  int32_T c2_i202;
  real_T c2_nxxi;
  real_T c2_xxi_data[1];
  int32_T c2_i203;
  real_T c2_varargin_3[84];
  c2_sPMb9Nq525A1JP3SLksK2fC c2_ppk;
  static real_T c2_b_varargin_3[84] = { 0.005, 0.017, 0.014, 0.01, -0.005, 0.009,
    0.019, 0.005, 0.0, -0.005, -0.011, 0.008, 0.007, 0.016, 0.014, 0.014, 0.013,
    0.009, 0.012, 0.005, 0.0, 0.004, 0.009, 0.007, 0.013, 0.013, 0.011, 0.012,
    0.011, 0.009, 0.008, 0.005, -0.002, 0.005, 0.003, 0.005, 0.018, 0.015, 0.015,
    0.014, 0.014, 0.014, 0.014, 0.015, 0.013, 0.011, 0.006, 0.001, 0.015, 0.014,
    0.013, 0.013, 0.012, 0.011, 0.011, 0.01, 0.008, 0.008, 0.007, 0.003, 0.021,
    0.011, 0.01, 0.011, 0.01, 0.009, 0.008, 0.01, 0.006, 0.005, 0.0, 0.001,
    0.023, 0.01, 0.011, 0.011, 0.011, 0.01, 0.008, 0.01, 0.006, 0.014, 0.02, 0.0
  };

  int32_T c2_c_varargin_5_sizes[2];
  int32_T c2_c_varargin_5;
  int32_T c2_d_varargin_5;
  int32_T c2_d_loop_ub;
  int32_T c2_i204;
  real_T c2_c_varargin_5_data[1];
  real_T c2_sv[2];
  int32_T c2_i205;
  int32_T c2_varargout_1_sizes[2];
  int32_T c2_varargout_1;
  int32_T c2_b_varargout_1;
  int32_T c2_e_loop_ub;
  int32_T c2_i206;
  real_T c2_b_nxxi;
  real_T c2_varargout_1_data[12];
  int32_T c2_i207;
  int32_T c2_j;
  real_T c2_b_j;
  c2_sPMb9Nq525A1JP3SLksK2fC c2_b_ppk;
  int32_T c2_c_xxi;
  int32_T c2_d_xxi[1];
  int32_T c2_e_xxi;
  real_T c2_vkj[12];
  int32_T c2_f_loop_ub;
  int32_T c2_i;
  int32_T c2_i208;
  real_T c2_b_i;
  c2_sL3Gi7vZqpbfvDjMPafkGnG c2_ppk_data;
  c2_sL3Gi7vZqpbfvDjMPafkGnG_size c2_ppk_elems_sizes;
  int32_T c2_i209;
  int32_T c2_yi;
  int32_T c2_b_yi;
  int32_T c2_g_loop_ub;
  int32_T c2_i210;
  real_T c2_c_nxxi;
  int32_T c2_i211;
  int32_T c2_c_j;
  c2_sL3Gi7vZqpbfvDjMPafkGnG_size c2_b_ppk_elems_sizes;
  c2_sL3Gi7vZqpbfvDjMPafkGnG c2_b_ppk_data;
  int32_T c2_f_xxi[1];
  real_T c2_vkj_data[1];
  int32_T c2_vkj_sizes;
  real_T c2_d10;
  int32_T c2_i212;
  int32_T c2_c_i;
  c2_b_varargin_4_sizes[0] = 1;
  c2_b_varargin_4_sizes[1] = c2_varargin_4_sizes[1];
  c2_varargin_4 = c2_b_varargin_4_sizes[0];
  c2_b_varargin_4 = c2_b_varargin_4_sizes[1];
  c2_loop_ub = c2_varargin_4_sizes[0] * c2_varargin_4_sizes[1] - 1;
  for (c2_i200 = 0; c2_i200 <= c2_loop_ub; c2_i200++) {
    c2_b_varargin_4_data[c2_i200] = c2_varargin_4_data[c2_i200];
  }

  c2_b_varargin_5_sizes[0] = 1;
  c2_b_varargin_5_sizes[1] = c2_varargin_5_sizes[1];
  c2_varargin_5 = c2_b_varargin_5_sizes[0];
  c2_b_varargin_5 = c2_b_varargin_5_sizes[1];
  c2_b_loop_ub = c2_varargin_5_sizes[0] * c2_varargin_5_sizes[1] - 1;
  for (c2_i201 = 0; c2_i201 <= c2_b_loop_ub; c2_i201++) {
    c2_b_varargin_5_data[c2_i201] = c2_varargin_5_data[c2_i201];
  }

  c2_output_size(chartInstance, c2_b_varargin_4_data, c2_b_varargin_4_sizes,
                 c2_b_varargin_5_data, c2_b_varargin_5_sizes, c2_outsize);
  c2_xxi_sizes[0] = 1;
  c2_xxi_sizes[1] = c2_varargin_5_sizes[1];
  c2_xxi = c2_xxi_sizes[0];
  c2_b_xxi = c2_xxi_sizes[1];
  c2_c_loop_ub = c2_varargin_5_sizes[0] * c2_varargin_5_sizes[1] - 1;
  for (c2_i202 = 0; c2_i202 <= c2_c_loop_ub; c2_i202++) {
    c2_xxi_data[c2_i202] = c2_varargin_5_data[c2_i202];
  }

  c2_nxxi = (real_T)c2_xxi_sizes[1];
  for (c2_i203 = 0; c2_i203 < 84; c2_i203++) {
    c2_varargin_3[c2_i203] = c2_b_varargin_3[c2_i203];
  }

  c2_e_spline(chartInstance, c2_varargin_3, &c2_ppk);
  c2_c_varargin_5_sizes[0] = 1;
  c2_c_varargin_5_sizes[1] = c2_varargin_5_sizes[1];
  c2_c_varargin_5 = c2_c_varargin_5_sizes[0];
  c2_d_varargin_5 = c2_c_varargin_5_sizes[1];
  c2_d_loop_ub = c2_varargin_5_sizes[0] * c2_varargin_5_sizes[1] - 1;
  for (c2_i204 = 0; c2_i204 <= c2_d_loop_ub; c2_i204++) {
    c2_c_varargin_5_data[c2_i204] = c2_varargin_5_data[c2_i204];
  }

  c2_intermediate_size(chartInstance, c2_c_varargin_5_data,
                       c2_c_varargin_5_sizes, c2_sv);
  for (c2_i205 = 0; c2_i205 < 2; c2_i205++) {
    c2_sv[c2_i205] = _SFD_NON_NEGATIVE_CHECK("", c2_sv[c2_i205]);
  }

  c2_varargout_1_sizes[0] = (int32_T)c2_sv[0];
  c2_varargout_1_sizes[1] = 12;
  c2_varargout_1 = c2_varargout_1_sizes[0];
  c2_b_varargout_1 = c2_varargout_1_sizes[1];
  c2_e_loop_ub = (int32_T)c2_sv[0] * (int32_T)c2_sv[1] - 1;
  for (c2_i206 = 0; c2_i206 <= c2_e_loop_ub; c2_i206++) {
    c2_varargout_1_data[c2_i206] = 0.0;
  }

  c2_b_nxxi = c2_nxxi;
  c2_i207 = (int32_T)c2_b_nxxi;
  _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_b_nxxi, mxDOUBLE_CLASS, c2_i207);
  for (c2_j = 0; c2_j < c2_i207; c2_j++) {
    c2_b_j = 1.0 + (real_T)c2_j;
    c2_b_ppk = c2_ppk;
    c2_d_xxi[0] = c2_xxi_sizes[1];
    c2_g_ppval(chartInstance, &c2_b_ppk, c2_xxi_data[(int32_T)c2_b_j - 1],
               c2_vkj);
    for (c2_i = 0; c2_i < 12; c2_i++) {
      c2_b_i = 1.0 + (real_T)c2_i;
      c2_varargout_1_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
        c2_vkj[(int32_T)c2_b_i - 1];
    }
  }

  c2_xxi_sizes[0] = 1;
  c2_xxi_sizes[1] = c2_varargin_4_sizes[1];
  c2_c_xxi = c2_xxi_sizes[0];
  c2_e_xxi = c2_xxi_sizes[1];
  c2_f_loop_ub = c2_varargin_4_sizes[0] * c2_varargin_4_sizes[1] - 1;
  for (c2_i208 = 0; c2_i208 <= c2_f_loop_ub; c2_i208++) {
    c2_xxi_data[c2_i208] = c2_varargin_4_data[c2_i208];
  }

  c2_nxxi = (real_T)c2_xxi_sizes[1];
  c2_d_spline(chartInstance, c2_varargout_1_data, c2_varargout_1_sizes,
              &c2_ppk_data, &c2_ppk_elems_sizes);
  for (c2_i209 = 0; c2_i209 < 2; c2_i209++) {
    c2_outsize[c2_i209] = _SFD_NON_NEGATIVE_CHECK("", c2_outsize[c2_i209]);
  }

  c2_yi_sizes[0] = (int32_T)c2_outsize[0];
  c2_yi_sizes[1] = (int32_T)c2_outsize[1];
  c2_yi = c2_yi_sizes[0];
  c2_b_yi = c2_yi_sizes[1];
  c2_g_loop_ub = (int32_T)c2_outsize[0] * (int32_T)c2_outsize[1] - 1;
  for (c2_i210 = 0; c2_i210 <= c2_g_loop_ub; c2_i210++) {
    c2_yi_data[c2_i210] = 0.0;
  }

  c2_c_nxxi = c2_nxxi;
  c2_i211 = (int32_T)c2_c_nxxi;
  _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_c_nxxi, mxDOUBLE_CLASS, c2_i211);
  for (c2_c_j = 0; c2_c_j < c2_i211; c2_c_j++) {
    c2_b_j = 1.0 + (real_T)c2_c_j;
    c2_b_ppk_elems_sizes = c2_ppk_elems_sizes;
    c2_b_ppk_data = c2_ppk_data;
    c2_f_xxi[0] = c2_xxi_sizes[1];
    c2_f_ppval(chartInstance, &c2_b_ppk_data, c2_b_ppk_elems_sizes, c2_xxi_data
               [(int32_T)c2_b_j - 1], c2_vkj_data, &c2_vkj_sizes);
    c2_d10 = (real_T)c2_vkj_sizes;
    c2_i212 = (int32_T)c2_d10;
    _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_d10, mxDOUBLE_CLASS, c2_i212);
    for (c2_c_i = 0; c2_c_i < c2_i212; c2_c_i++) {
      c2_b_i = 1.0 + (real_T)c2_c_i;
      c2_yi_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
        c2_vkj_data[(int32_T)c2_b_i - 1];
    }
  }
}

static real_T c2_sign(SFc2_QuanInstanceStruct *chartInstance, real_T c2_x)
{
  real_T c2_b_x;
  c2_b_x = c2_x;
  c2_b_sign(chartInstance, &c2_b_x);
  return c2_b_x;
}

static real_T c2_g_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_Xq;
  real_T c2_Yq;
  c2_Xq = c2_varargin_4;
  c2_Yq = c2_varargin_5;
  return c2_c_interp2_dispatch(chartInstance, c2_Xq, c2_Yq);
}

static real_T c2_c_interp2_dispatch(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_Xq, real_T c2_Yq)
{
  real_T c2_Vq;
  real_T c2_b_Xq;
  real_T c2_b_Yq;
  real_T c2_varargin_4;
  real_T c2_varargin_5;
  real_T c2_a;
  real_T c2_uxi;
  real_T c2_uxi_data[1];
  int32_T c2_uxi_sizes[2];
  int32_T c2_indx_data[1];
  int32_T c2_indx_sizes;
  int32_T c2_ipos;
  real_T c2_uyi;
  int32_T c2_b_ipos;
  real_T c2_ix;
  real_T c2_b_ix;
  real_T c2_b_a;
  real_T c2_uyi_data[1];
  int32_T c2_uyi_sizes[2];
  int32_T c2_c_ipos;
  int32_T c2_d_ipos;
  real_T c2_iy;
  real_T c2_b_iy;
  real_T c2_uwi_data[1];
  int32_T c2_uwi_sizes[2];
  c2_b_Xq = c2_Xq;
  c2_b_Yq = c2_Yq;
  c2_varargin_4 = c2_b_Yq;
  c2_varargin_5 = c2_b_Xq;
  if (c2_isplaid(chartInstance, c2_varargin_4, c2_varargin_5)) {
    c2_uxi = c2_varargin_4;
    c2_uyi = c2_varargin_5;
    c2_Vq = c2_i_TensorGriddedInterp(chartInstance, c2_uxi, c2_uyi);
  } else {
    c2_a = c2_varargin_4;
    c2_unique_vector(chartInstance, c2_a, c2_uxi_data, c2_uxi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_ipos);
    c2_b_ipos = c2_ipos;
    c2_ix = (real_T)c2_b_ipos;
    c2_b_ix = c2_ix;
    c2_b_a = c2_varargin_5;
    c2_unique_vector(chartInstance, c2_b_a, c2_uyi_data, c2_uyi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_c_ipos);
    c2_d_ipos = c2_c_ipos;
    c2_iy = (real_T)c2_d_ipos;
    c2_b_iy = c2_iy;
    c2_j_TensorGriddedInterp(chartInstance, c2_uxi_data, c2_uxi_sizes,
      c2_uyi_data, c2_uyi_sizes, c2_uwi_data, c2_uwi_sizes);
    c2_Vq = c2_uwi_data[((int32_T)c2_b_ix + c2_uwi_sizes[0] * ((int32_T)c2_b_iy
      - 1)) - 1];
  }

  return c2_Vq;
}

static real_T c2_i_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_xxi;
  int32_T c2_i213;
  real_T c2_varargin_3[84];
  c2_sPMb9Nq525A1JP3SLksK2fC c2_ppk;
  static real_T c2_b_varargin_3[84] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.001, -0.004, -0.008, -0.012, -0.016, -0.019, -0.02,
    -0.02, -0.015, -0.008, -0.013, -0.015, -0.003, -0.009, -0.017, -0.024, -0.03,
    -0.034, -0.04, -0.037, -0.016, -0.002, -0.1, -0.19, -0.001, -0.01, -0.02,
    -0.03, -0.039, -0.044, -0.05, -0.049, -0.023, -0.006, -0.014, -0.027, 0.0,
    -0.01, -0.022, -0.034, -0.047, -0.046, -0.059, -0.061, -0.033, -0.036,
    -0.035, -0.035, 0.07, -0.01, -0.023, -0.034, -0.049, -0.046, -0.068, -0.071,
    -0.06, -0.058, -0.062, -0.059, 0.009, -0.011, -0.023, -0.037, -0.05, -0.047,
    -0.074, -0.079, -0.091, -0.076, -0.077, -0.076 };

  int32_T c2_i214;
  real_T c2_vkj[12];
  real_T c2_varargout_1[12];
  int32_T c2_i;
  real_T c2_b_i;
  c2_sSYDZjSrzjRAmypAou1DUJH c2_b_ppk;
  real_T c2_b_vkj;
  c2_xxi = c2_varargin_5;
  for (c2_i213 = 0; c2_i213 < 84; c2_i213++) {
    c2_varargin_3[c2_i213] = c2_b_varargin_3[c2_i213];
  }

  c2_f_spline(chartInstance, c2_varargin_3, &c2_ppk);
  for (c2_i214 = 0; c2_i214 < 12; c2_i214++) {
    c2_varargout_1[c2_i214] = 0.0;
  }

  c2_g_ppval(chartInstance, &c2_ppk, c2_xxi, c2_vkj);
  for (c2_i = 0; c2_i < 12; c2_i++) {
    c2_b_i = 1.0 + (real_T)c2_i;
    c2_varargout_1[(int32_T)(1.0 + (c2_b_i - 1.0)) - 1] = c2_vkj[(int32_T)c2_b_i
      - 1];
  }

  c2_xxi = c2_varargin_4;
  c2_c_spline(chartInstance, c2_varargout_1, &c2_b_ppk);
  c2_b_vkj = c2_ppval(chartInstance, &c2_b_ppk, c2_xxi);
  return c2_b_vkj;
}

static void c2_f_spline(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[84],
  c2_sPMb9Nq525A1JP3SLksK2fC *c2_output)
{
  int32_T c2_i215;
  real_T c2_b_y[84];
  for (c2_i215 = 0; c2_i215 < 84; c2_i215++) {
    c2_b_y[c2_i215] = c2_y[c2_i215];
  }

  c2_f_splinepp(chartInstance, c2_b_y, c2_output);
}

static void c2_f_splinepp(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[84],
  c2_sPMb9Nq525A1JP3SLksK2fC *c2_pp)
{
  int32_T c2_i216;
  real_T c2_b_y[84];
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_dx[6];
  int32_T c2_dpg;
  real_T c2_d1;
  int32_T c2_pg;
  real_T c2_d2;
  int32_T c2_pgp1;
  int32_T c2_j;
  int32_T c2_pgm1;
  int32_T c2_b_j;
  int32_T c2_c_j;
  int32_T c2_d_j;
  real_T c2_A;
  real_T c2_dvdf[72];
  real_T c2_b_A;
  int32_T c2_e_j;
  real_T c2_x;
  real_T c2_B;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_c_y;
  real_T c2_s[84];
  real_T c2_d_y;
  real_T c2_md[7];
  real_T c2_d_x;
  real_T c2_c_A;
  real_T c2_e_y;
  int32_T c2_d_k;
  real_T c2_e_x;
  real_T c2_f_y;
  real_T c2_f_x;
  real_T c2_g_y;
  real_T c2_d_A;
  real_T c2_b_B;
  real_T c2_g_x;
  real_T c2_h_y;
  real_T c2_h_x;
  real_T c2_i_y;
  real_T c2_r;
  int32_T c2_f_j;
  int32_T c2_e_k;
  real_T c2_c_B;
  real_T c2_j_y;
  real_T c2_e_A;
  real_T c2_k_y;
  real_T c2_d_B;
  real_T c2_i_x;
  real_T c2_l_y;
  int32_T c2_g_j;
  real_T c2_j_x;
  real_T c2_m_y;
  int32_T c2_h_j;
  int32_T c2_f_k;
  int32_T c2_i_j;
  real_T c2_f_A;
  real_T c2_e_B;
  int32_T c2_j_j;
  real_T c2_k_x;
  real_T c2_n_y;
  real_T c2_l_x;
  int32_T c2_i217;
  real_T c2_o_y;
  real_T c2_g_A;
  int32_T c2_k_j;
  real_T c2_p_y;
  real_T c2_f_B;
  int32_T c2_i218;
  real_T c2_dv32[7];
  real_T c2_m_x;
  real_T c2_q_y;
  real_T c2_n_x;
  real_T c2_h_A;
  real_T c2_r_y[84];
  real_T c2_s_y;
  real_T c2_g_B;
  real_T c2_t_y;
  real_T c2_o_x;
  real_T c2_u_y;
  real_T c2_p_x;
  real_T c2_v_y;
  real_T c2_w_y;
  for (c2_i216 = 0; c2_i216 < 84; c2_i216++) {
    c2_b_y[c2_i216] = c2_y[c2_i216];
  }

  c2_h_chckxy(chartInstance, c2_b_y);
  for (c2_k = 1; c2_k < 7; c2_k++) {
    c2_c_k = c2_k - 1;
    c2_dx[c2_c_k] = 5.0 * (real_T)(c2_c_k + 1) - 5.0 * (real_T)c2_c_k;
    c2_dpg = c2_c_k * 12;
    c2_pg = c2_dpg;
    c2_pgp1 = (c2_c_k + 1) * 12;
    for (c2_b_j = 1; c2_b_j < 13; c2_b_j++) {
      c2_c_j = c2_b_j - 1;
      c2_b_A = c2_y[c2_pgp1 + c2_c_j] - c2_y[c2_pg + c2_c_j];
      c2_B = c2_dx[c2_c_k];
      c2_c_x = c2_b_A;
      c2_d_y = c2_B;
      c2_d_x = c2_c_x;
      c2_e_y = c2_d_y;
      c2_f_y = c2_d_x / c2_e_y;
      c2_dvdf[c2_dpg + c2_c_j] = c2_f_y;
    }
  }

  for (c2_b_k = 2; c2_b_k < 7; c2_b_k++) {
    c2_c_k = c2_b_k - 2;
    c2_pg = (c2_c_k + 1) * 12;
    c2_pgm1 = c2_c_k * 12;
    c2_d1 = c2_dx[c2_c_k + 1];
    c2_d2 = c2_dx[c2_c_k];
    for (c2_d_j = 1; c2_d_j < 13; c2_d_j++) {
      c2_c_j = c2_d_j - 1;
      c2_s[c2_pg + c2_c_j] = 3.0 * (c2_d1 * c2_dvdf[c2_pgm1 + c2_c_j] + c2_d2 *
        c2_dvdf[c2_pg + c2_c_j]);
    }
  }

  c2_d1 = c2_dx[0];
  c2_d2 = c2_dx[1];
  for (c2_j = 1; c2_j < 13; c2_j++) {
    c2_c_j = c2_j - 1;
    c2_A = (c2_d1 + 20.0) * c2_d2 * c2_dvdf[c2_c_j] + c2_d1 * c2_d1 *
      c2_dvdf[c2_c_j + 12];
    c2_x = c2_A;
    c2_b_x = c2_x;
    c2_c_y = c2_b_x / 10.0;
    c2_s[c2_c_j] = c2_c_y;
  }

  c2_d1 = c2_dx[5];
  c2_d2 = c2_dx[4];
  for (c2_e_j = 1; c2_e_j < 13; c2_e_j++) {
    c2_c_j = c2_e_j + 47;
    c2_c_A = (c2_d1 + 20.0) * c2_d2 * c2_dvdf[c2_c_j + 12] + c2_d1 * c2_d1 *
      c2_dvdf[c2_c_j];
    c2_e_x = c2_c_A;
    c2_f_x = c2_e_x;
    c2_g_y = c2_f_x / 10.0;
    c2_s[c2_c_j + 24] = c2_g_y;
  }

  c2_md[0] = c2_dx[1];
  c2_md[6] = c2_dx[4];
  for (c2_d_k = 2; c2_d_k < 7; c2_d_k++) {
    c2_c_k = c2_d_k - 1;
    c2_md[c2_c_k] = 2.0 * (c2_dx[c2_c_k] + c2_dx[c2_c_k - 1]);
  }

  c2_d_A = c2_dx[1];
  c2_b_B = c2_md[0];
  c2_g_x = c2_d_A;
  c2_h_y = c2_b_B;
  c2_h_x = c2_g_x;
  c2_i_y = c2_h_y;
  c2_r = c2_h_x / c2_i_y;
  c2_md[1] -= c2_r * 10.0;
  for (c2_f_j = 1; c2_f_j < 13; c2_f_j++) {
    c2_c_j = c2_f_j + 11;
    c2_s[c2_c_j] -= c2_r * c2_s[c2_c_j - 12];
  }

  for (c2_e_k = 3; c2_e_k < 7; c2_e_k++) {
    c2_c_k = c2_e_k - 1;
    c2_e_A = c2_dx[c2_c_k];
    c2_d_B = c2_md[c2_c_k - 1];
    c2_i_x = c2_e_A;
    c2_l_y = c2_d_B;
    c2_j_x = c2_i_x;
    c2_m_y = c2_l_y;
    c2_r = c2_j_x / c2_m_y;
    c2_md[c2_c_k] -= c2_r * c2_dx[c2_c_k - 2];
    c2_pg = c2_c_k * 12;
    c2_pgm1 = (c2_c_k - 1) * 12;
    for (c2_i_j = 1; c2_i_j < 13; c2_i_j++) {
      c2_c_j = c2_i_j - 1;
      c2_s[c2_pg + c2_c_j] -= c2_r * c2_s[c2_pgm1 + c2_c_j];
    }
  }

  c2_c_B = c2_md[5];
  c2_j_y = c2_c_B;
  c2_k_y = c2_j_y;
  c2_r = 10.0 / c2_k_y;
  c2_md[6] -= c2_r * c2_dx[4];
  for (c2_g_j = 1; c2_g_j < 13; c2_g_j++) {
    c2_c_j = c2_g_j + 71;
    c2_s[c2_c_j] -= c2_r * c2_s[c2_c_j - 12];
  }

  for (c2_h_j = 1; c2_h_j < 13; c2_h_j++) {
    c2_c_j = c2_h_j + 71;
    c2_f_A = c2_s[c2_c_j];
    c2_e_B = c2_md[6];
    c2_k_x = c2_f_A;
    c2_n_y = c2_e_B;
    c2_l_x = c2_k_x;
    c2_o_y = c2_n_y;
    c2_p_y = c2_l_x / c2_o_y;
    c2_s[c2_c_j] = c2_p_y;
  }

  for (c2_f_k = 6; c2_f_k > 1; c2_f_k--) {
    c2_c_k = c2_f_k - 1;
    c2_pg = c2_c_k * 12;
    c2_pgp1 = (c2_c_k + 1) * 12;
    c2_d1 = c2_dx[c2_c_k - 1];
    for (c2_k_j = 1; c2_k_j < 13; c2_k_j++) {
      c2_c_j = c2_k_j - 1;
      c2_h_A = c2_s[c2_pg + c2_c_j] - c2_d1 * c2_s[c2_pgp1 + c2_c_j];
      c2_g_B = c2_md[c2_c_k];
      c2_o_x = c2_h_A;
      c2_u_y = c2_g_B;
      c2_p_x = c2_o_x;
      c2_v_y = c2_u_y;
      c2_w_y = c2_p_x / c2_v_y;
      c2_s[c2_pg + c2_c_j] = c2_w_y;
    }
  }

  for (c2_j_j = 1; c2_j_j < 13; c2_j_j++) {
    c2_c_j = c2_j_j - 1;
    c2_g_A = c2_s[c2_c_j] - 10.0 * c2_s[c2_c_j + 12];
    c2_f_B = c2_md[0];
    c2_m_x = c2_g_A;
    c2_q_y = c2_f_B;
    c2_n_x = c2_m_x;
    c2_s_y = c2_q_y;
    c2_t_y = c2_n_x / c2_s_y;
    c2_s[c2_c_j] = c2_t_y;
  }

  for (c2_i217 = 0; c2_i217 < 7; c2_i217++) {
    c2_dv32[c2_i217] = 5.0 * (real_T)c2_i217;
  }

  for (c2_i218 = 0; c2_i218 < 84; c2_i218++) {
    c2_r_y[c2_i218] = c2_y[c2_i218];
  }

  c2_g_pwchcore(chartInstance, c2_dv32, c2_r_y, c2_s, c2_dx, c2_dvdf, c2_pp);
}

static void c2_h_chckxy(SFc2_QuanInstanceStruct *chartInstance, real_T c2_y[84])
{
  int32_T c2_i219;
  real_T c2_b_y[84];
  const mxArray *c2_c_y = NULL;
  static char_T c2_u[28] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'U', 'n', 's', 'u', 'p', 'p', 'o', 'r', 't', 'e', 'd',
    'N', 'a', 'N' };

  for (c2_i219 = 0; c2_i219 < 84; c2_i219++) {
    c2_b_y[c2_i219] = c2_y[c2_i219];
  }

  if (!c2_b_anyIsNaN(chartInstance, c2_b_y)) {
  } else {
    c2_c_y = NULL;
    sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 28),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c2_c_y));
  }
}

static void c2_j_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4_data[], int32_T c2_varargin_4_sizes[2], real_T
  c2_varargin_5_data[], int32_T c2_varargin_5_sizes[2], real_T c2_yi_data[],
  int32_T c2_yi_sizes[2])
{
  int32_T c2_b_varargin_4_sizes[2];
  int32_T c2_varargin_4;
  int32_T c2_b_varargin_4;
  int32_T c2_loop_ub;
  int32_T c2_i220;
  int32_T c2_b_varargin_5_sizes[2];
  real_T c2_b_varargin_4_data[1];
  int32_T c2_varargin_5;
  int32_T c2_b_varargin_5;
  int32_T c2_b_loop_ub;
  int32_T c2_i221;
  real_T c2_b_varargin_5_data[1];
  real_T c2_outsize[2];
  int32_T c2_xxi_sizes[2];
  int32_T c2_xxi;
  int32_T c2_b_xxi;
  int32_T c2_c_loop_ub;
  int32_T c2_i222;
  real_T c2_nxxi;
  real_T c2_xxi_data[1];
  int32_T c2_i223;
  real_T c2_varargin_3[84];
  c2_sPMb9Nq525A1JP3SLksK2fC c2_ppk;
  static real_T c2_b_varargin_3[84] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.001, -0.004, -0.008, -0.012, -0.016, -0.019, -0.02,
    -0.02, -0.015, -0.008, -0.013, -0.015, -0.003, -0.009, -0.017, -0.024, -0.03,
    -0.034, -0.04, -0.037, -0.016, -0.002, -0.1, -0.19, -0.001, -0.01, -0.02,
    -0.03, -0.039, -0.044, -0.05, -0.049, -0.023, -0.006, -0.014, -0.027, 0.0,
    -0.01, -0.022, -0.034, -0.047, -0.046, -0.059, -0.061, -0.033, -0.036,
    -0.035, -0.035, 0.07, -0.01, -0.023, -0.034, -0.049, -0.046, -0.068, -0.071,
    -0.06, -0.058, -0.062, -0.059, 0.009, -0.011, -0.023, -0.037, -0.05, -0.047,
    -0.074, -0.079, -0.091, -0.076, -0.077, -0.076 };

  int32_T c2_c_varargin_5_sizes[2];
  int32_T c2_c_varargin_5;
  int32_T c2_d_varargin_5;
  int32_T c2_d_loop_ub;
  int32_T c2_i224;
  real_T c2_c_varargin_5_data[1];
  real_T c2_sv[2];
  int32_T c2_i225;
  int32_T c2_varargout_1_sizes[2];
  int32_T c2_varargout_1;
  int32_T c2_b_varargout_1;
  int32_T c2_e_loop_ub;
  int32_T c2_i226;
  real_T c2_b_nxxi;
  real_T c2_varargout_1_data[12];
  int32_T c2_i227;
  int32_T c2_j;
  real_T c2_b_j;
  c2_sPMb9Nq525A1JP3SLksK2fC c2_b_ppk;
  int32_T c2_c_xxi;
  int32_T c2_d_xxi[1];
  int32_T c2_e_xxi;
  real_T c2_vkj[12];
  int32_T c2_f_loop_ub;
  int32_T c2_i;
  int32_T c2_i228;
  real_T c2_b_i;
  c2_sL3Gi7vZqpbfvDjMPafkGnG c2_ppk_data;
  c2_sL3Gi7vZqpbfvDjMPafkGnG_size c2_ppk_elems_sizes;
  int32_T c2_i229;
  int32_T c2_yi;
  int32_T c2_b_yi;
  int32_T c2_g_loop_ub;
  int32_T c2_i230;
  real_T c2_c_nxxi;
  int32_T c2_i231;
  int32_T c2_c_j;
  c2_sL3Gi7vZqpbfvDjMPafkGnG_size c2_b_ppk_elems_sizes;
  c2_sL3Gi7vZqpbfvDjMPafkGnG c2_b_ppk_data;
  int32_T c2_f_xxi[1];
  real_T c2_vkj_data[1];
  int32_T c2_vkj_sizes;
  real_T c2_d11;
  int32_T c2_i232;
  int32_T c2_c_i;
  c2_b_varargin_4_sizes[0] = 1;
  c2_b_varargin_4_sizes[1] = c2_varargin_4_sizes[1];
  c2_varargin_4 = c2_b_varargin_4_sizes[0];
  c2_b_varargin_4 = c2_b_varargin_4_sizes[1];
  c2_loop_ub = c2_varargin_4_sizes[0] * c2_varargin_4_sizes[1] - 1;
  for (c2_i220 = 0; c2_i220 <= c2_loop_ub; c2_i220++) {
    c2_b_varargin_4_data[c2_i220] = c2_varargin_4_data[c2_i220];
  }

  c2_b_varargin_5_sizes[0] = 1;
  c2_b_varargin_5_sizes[1] = c2_varargin_5_sizes[1];
  c2_varargin_5 = c2_b_varargin_5_sizes[0];
  c2_b_varargin_5 = c2_b_varargin_5_sizes[1];
  c2_b_loop_ub = c2_varargin_5_sizes[0] * c2_varargin_5_sizes[1] - 1;
  for (c2_i221 = 0; c2_i221 <= c2_b_loop_ub; c2_i221++) {
    c2_b_varargin_5_data[c2_i221] = c2_varargin_5_data[c2_i221];
  }

  c2_output_size(chartInstance, c2_b_varargin_4_data, c2_b_varargin_4_sizes,
                 c2_b_varargin_5_data, c2_b_varargin_5_sizes, c2_outsize);
  c2_xxi_sizes[0] = 1;
  c2_xxi_sizes[1] = c2_varargin_5_sizes[1];
  c2_xxi = c2_xxi_sizes[0];
  c2_b_xxi = c2_xxi_sizes[1];
  c2_c_loop_ub = c2_varargin_5_sizes[0] * c2_varargin_5_sizes[1] - 1;
  for (c2_i222 = 0; c2_i222 <= c2_c_loop_ub; c2_i222++) {
    c2_xxi_data[c2_i222] = c2_varargin_5_data[c2_i222];
  }

  c2_nxxi = (real_T)c2_xxi_sizes[1];
  for (c2_i223 = 0; c2_i223 < 84; c2_i223++) {
    c2_varargin_3[c2_i223] = c2_b_varargin_3[c2_i223];
  }

  c2_f_spline(chartInstance, c2_varargin_3, &c2_ppk);
  c2_c_varargin_5_sizes[0] = 1;
  c2_c_varargin_5_sizes[1] = c2_varargin_5_sizes[1];
  c2_c_varargin_5 = c2_c_varargin_5_sizes[0];
  c2_d_varargin_5 = c2_c_varargin_5_sizes[1];
  c2_d_loop_ub = c2_varargin_5_sizes[0] * c2_varargin_5_sizes[1] - 1;
  for (c2_i224 = 0; c2_i224 <= c2_d_loop_ub; c2_i224++) {
    c2_c_varargin_5_data[c2_i224] = c2_varargin_5_data[c2_i224];
  }

  c2_intermediate_size(chartInstance, c2_c_varargin_5_data,
                       c2_c_varargin_5_sizes, c2_sv);
  for (c2_i225 = 0; c2_i225 < 2; c2_i225++) {
    c2_sv[c2_i225] = _SFD_NON_NEGATIVE_CHECK("", c2_sv[c2_i225]);
  }

  c2_varargout_1_sizes[0] = (int32_T)c2_sv[0];
  c2_varargout_1_sizes[1] = 12;
  c2_varargout_1 = c2_varargout_1_sizes[0];
  c2_b_varargout_1 = c2_varargout_1_sizes[1];
  c2_e_loop_ub = (int32_T)c2_sv[0] * (int32_T)c2_sv[1] - 1;
  for (c2_i226 = 0; c2_i226 <= c2_e_loop_ub; c2_i226++) {
    c2_varargout_1_data[c2_i226] = 0.0;
  }

  c2_b_nxxi = c2_nxxi;
  c2_i227 = (int32_T)c2_b_nxxi;
  _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_b_nxxi, mxDOUBLE_CLASS, c2_i227);
  for (c2_j = 0; c2_j < c2_i227; c2_j++) {
    c2_b_j = 1.0 + (real_T)c2_j;
    c2_b_ppk = c2_ppk;
    c2_d_xxi[0] = c2_xxi_sizes[1];
    c2_g_ppval(chartInstance, &c2_b_ppk, c2_xxi_data[(int32_T)c2_b_j - 1],
               c2_vkj);
    for (c2_i = 0; c2_i < 12; c2_i++) {
      c2_b_i = 1.0 + (real_T)c2_i;
      c2_varargout_1_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
        c2_vkj[(int32_T)c2_b_i - 1];
    }
  }

  c2_xxi_sizes[0] = 1;
  c2_xxi_sizes[1] = c2_varargin_4_sizes[1];
  c2_c_xxi = c2_xxi_sizes[0];
  c2_e_xxi = c2_xxi_sizes[1];
  c2_f_loop_ub = c2_varargin_4_sizes[0] * c2_varargin_4_sizes[1] - 1;
  for (c2_i228 = 0; c2_i228 <= c2_f_loop_ub; c2_i228++) {
    c2_xxi_data[c2_i228] = c2_varargin_4_data[c2_i228];
  }

  c2_nxxi = (real_T)c2_xxi_sizes[1];
  c2_d_spline(chartInstance, c2_varargout_1_data, c2_varargout_1_sizes,
              &c2_ppk_data, &c2_ppk_elems_sizes);
  for (c2_i229 = 0; c2_i229 < 2; c2_i229++) {
    c2_outsize[c2_i229] = _SFD_NON_NEGATIVE_CHECK("", c2_outsize[c2_i229]);
  }

  c2_yi_sizes[0] = (int32_T)c2_outsize[0];
  c2_yi_sizes[1] = (int32_T)c2_outsize[1];
  c2_yi = c2_yi_sizes[0];
  c2_b_yi = c2_yi_sizes[1];
  c2_g_loop_ub = (int32_T)c2_outsize[0] * (int32_T)c2_outsize[1] - 1;
  for (c2_i230 = 0; c2_i230 <= c2_g_loop_ub; c2_i230++) {
    c2_yi_data[c2_i230] = 0.0;
  }

  c2_c_nxxi = c2_nxxi;
  c2_i231 = (int32_T)c2_c_nxxi;
  _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_c_nxxi, mxDOUBLE_CLASS, c2_i231);
  for (c2_c_j = 0; c2_c_j < c2_i231; c2_c_j++) {
    c2_b_j = 1.0 + (real_T)c2_c_j;
    c2_b_ppk_elems_sizes = c2_ppk_elems_sizes;
    c2_b_ppk_data = c2_ppk_data;
    c2_f_xxi[0] = c2_xxi_sizes[1];
    c2_f_ppval(chartInstance, &c2_b_ppk_data, c2_b_ppk_elems_sizes, c2_xxi_data
               [(int32_T)c2_b_j - 1], c2_vkj_data, &c2_vkj_sizes);
    c2_d11 = (real_T)c2_vkj_sizes;
    c2_i232 = (int32_T)c2_d11;
    _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_d11, mxDOUBLE_CLASS, c2_i232);
    for (c2_c_i = 0; c2_c_i < c2_i232; c2_c_i++) {
      c2_b_i = 1.0 + (real_T)c2_c_i;
      c2_yi_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
        c2_vkj_data[(int32_T)c2_b_i - 1];
    }
  }
}

static real_T c2_h_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_Vq;
  real_T c2_Xq;
  real_T c2_Yq;
  real_T c2_b_Xq;
  real_T c2_b_Yq;
  real_T c2_c_Xq;
  real_T c2_c_Yq;
  real_T c2_b_varargin_4;
  real_T c2_b_varargin_5;
  real_T c2_a;
  real_T c2_uxi;
  real_T c2_uxi_data[1];
  int32_T c2_uxi_sizes[2];
  int32_T c2_indx_data[1];
  int32_T c2_indx_sizes;
  int32_T c2_ipos;
  real_T c2_uyi;
  int32_T c2_b_ipos;
  int32_T c2_i233;
  real_T c2_ix;
  real_T c2_b_ix;
  real_T c2_b_a;
  real_T c2_dv33[60];
  static real_T c2_dv34[60] = { 0.205, 0.168, 0.186, 0.196, 0.213, 0.251, 0.245,
    0.238, 0.252, 0.231, 0.198, 0.192, 0.081, 0.077, 0.107, 0.11, 0.11, 0.141,
    0.127, 0.119, 0.133, 0.108, 0.081, 0.093, -0.046, -0.02, -0.009, -0.005,
    -0.006, 0.01, 0.006, -0.001, 0.014, 0.0, -0.013, 0.032, -0.174, -0.145,
    -0.121, -0.127, -0.129, -0.102, -0.097, -0.113, -0.087, -0.084, -0.069,
    -0.006, -0.259, -0.202, -0.184, -0.193, -0.199, -0.15, -0.16, -0.167, -0.104,
    -0.076, -0.041, -0.005 };

  real_T c2_uyi_data[1];
  int32_T c2_uyi_sizes[2];
  int32_T c2_c_ipos;
  int32_T c2_d_ipos;
  real_T c2_iy;
  real_T c2_b_iy;
  real_T c2_uwi_data[1];
  int32_T c2_uwi_sizes[2];
  c2_Xq = c2_varargin_4;
  c2_Yq = c2_varargin_5;
  c2_b_Xq = c2_Xq;
  c2_b_Yq = c2_Yq;
  c2_c_Xq = c2_b_Xq;
  c2_c_Yq = c2_b_Yq;
  c2_b_varargin_4 = c2_c_Yq;
  c2_b_varargin_5 = c2_c_Xq;
  if (c2_isplaid(chartInstance, c2_b_varargin_4, c2_b_varargin_5)) {
    c2_uxi = c2_b_varargin_4;
    c2_uyi = c2_b_varargin_5;
    for (c2_i233 = 0; c2_i233 < 60; c2_i233++) {
      c2_dv33[c2_i233] = c2_dv34[c2_i233];
    }

    c2_Vq = c2_k_TensorGriddedInterp(chartInstance, c2_dv33, c2_uxi, c2_uyi);
  } else {
    c2_a = c2_b_varargin_4;
    c2_unique_vector(chartInstance, c2_a, c2_uxi_data, c2_uxi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_ipos);
    c2_b_ipos = c2_ipos;
    c2_ix = (real_T)c2_b_ipos;
    c2_b_ix = c2_ix;
    c2_b_a = c2_b_varargin_5;
    c2_unique_vector(chartInstance, c2_b_a, c2_uyi_data, c2_uyi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_c_ipos);
    c2_d_ipos = c2_c_ipos;
    c2_iy = (real_T)c2_d_ipos;
    c2_b_iy = c2_iy;
    c2_l_TensorGriddedInterp(chartInstance, c2_uxi_data, c2_uxi_sizes,
      c2_uyi_data, c2_uyi_sizes, c2_uwi_data, c2_uwi_sizes);
    c2_Vq = c2_uwi_data[((int32_T)c2_b_ix + c2_uwi_sizes[0] * ((int32_T)c2_b_iy
      - 1)) - 1];
  }

  return c2_Vq;
}

static real_T c2_k_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_3[60], real_T c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_xxi;
  int32_T c2_i234;
  real_T c2_b_varargin_3[60];
  c2_samQD8b77pHdwkX3iY74akB c2_ppk;
  int32_T c2_i235;
  real_T c2_vkj[12];
  real_T c2_varargout_1[12];
  int32_T c2_i;
  real_T c2_b_i;
  c2_sSYDZjSrzjRAmypAou1DUJH c2_pp;
  real_T c2_b_vkj;
  c2_xxi = c2_varargin_5;
  for (c2_i234 = 0; c2_i234 < 60; c2_i234++) {
    c2_b_varargin_3[c2_i234] = c2_varargin_3[c2_i234];
  }

  c2_b_spline(chartInstance, c2_b_varargin_3, &c2_ppk);
  for (c2_i235 = 0; c2_i235 < 12; c2_i235++) {
    c2_varargout_1[c2_i235] = 0.0;
  }

  c2_e_ppval(chartInstance, &c2_ppk, c2_xxi, c2_vkj);
  for (c2_i = 0; c2_i < 12; c2_i++) {
    c2_b_i = 1.0 + (real_T)c2_i;
    c2_varargout_1[(int32_T)(1.0 + (c2_b_i - 1.0)) - 1] = c2_vkj[(int32_T)c2_b_i
      - 1];
  }

  c2_xxi = c2_varargin_4;
  c2_d_splinepp(chartInstance, c2_varargout_1, &c2_pp);
  c2_b_vkj = c2_ppval(chartInstance, &c2_pp, c2_xxi);
  return c2_b_vkj;
}

static void c2_l_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4_data[], int32_T c2_varargin_4_sizes[2], real_T
  c2_varargin_5_data[], int32_T c2_varargin_5_sizes[2], real_T c2_yi_data[],
  int32_T c2_yi_sizes[2])
{
  real_T c2_outsize[2];
  int32_T c2_xxi_sizes[2];
  int32_T c2_xxi;
  int32_T c2_b_xxi;
  int32_T c2_loop_ub;
  int32_T c2_i236;
  real_T c2_nxxi;
  real_T c2_xxi_data[1];
  int32_T c2_i237;
  real_T c2_varargin_3[60];
  c2_samQD8b77pHdwkX3iY74akB c2_ppk;
  static real_T c2_b_varargin_3[60] = { 0.205, 0.168, 0.186, 0.196, 0.213, 0.251,
    0.245, 0.238, 0.252, 0.231, 0.198, 0.192, 0.081, 0.077, 0.107, 0.11, 0.11,
    0.141, 0.127, 0.119, 0.133, 0.108, 0.081, 0.093, -0.046, -0.02, -0.009,
    -0.005, -0.006, 0.01, 0.006, -0.001, 0.014, 0.0, -0.013, 0.032, -0.174,
    -0.145, -0.121, -0.127, -0.129, -0.102, -0.097, -0.113, -0.087, -0.084,
    -0.069, -0.006, -0.259, -0.202, -0.184, -0.193, -0.199, -0.15, -0.16, -0.167,
    -0.104, -0.076, -0.041, -0.005 };

  real_T c2_sv[2];
  int32_T c2_i238;
  int32_T c2_varargout_1_sizes[2];
  int32_T c2_varargout_1;
  int32_T c2_b_varargout_1;
  int32_T c2_b_loop_ub;
  int32_T c2_i239;
  real_T c2_b_nxxi;
  real_T c2_varargout_1_data[12];
  int32_T c2_i240;
  int32_T c2_j;
  real_T c2_b_j;
  c2_samQD8b77pHdwkX3iY74akB c2_b_ppk;
  int32_T c2_c_xxi;
  int32_T c2_d_xxi[1];
  int32_T c2_e_xxi;
  real_T c2_vkj[12];
  int32_T c2_c_loop_ub;
  int32_T c2_i;
  int32_T c2_i241;
  real_T c2_b_i;
  c2_sL3Gi7vZqpbfvDjMPafkGnG c2_ppk_data;
  c2_sL3Gi7vZqpbfvDjMPafkGnG_size c2_ppk_elems_sizes;
  int32_T c2_i242;
  int32_T c2_yi;
  int32_T c2_b_yi;
  int32_T c2_d_loop_ub;
  int32_T c2_i243;
  real_T c2_c_nxxi;
  int32_T c2_i244;
  int32_T c2_c_j;
  c2_sL3Gi7vZqpbfvDjMPafkGnG_size c2_b_ppk_elems_sizes;
  c2_sL3Gi7vZqpbfvDjMPafkGnG c2_b_ppk_data;
  int32_T c2_f_xxi[1];
  real_T c2_vkj_data[1];
  int32_T c2_vkj_sizes;
  real_T c2_d12;
  int32_T c2_i245;
  int32_T c2_c_i;
  c2_outsize[0] = (real_T)c2_varargin_4_sizes[1];
  c2_outsize[1] = (real_T)c2_varargin_5_sizes[1];
  c2_xxi_sizes[0] = 1;
  c2_xxi_sizes[1] = c2_varargin_5_sizes[1];
  c2_xxi = c2_xxi_sizes[0];
  c2_b_xxi = c2_xxi_sizes[1];
  c2_loop_ub = c2_varargin_5_sizes[0] * c2_varargin_5_sizes[1] - 1;
  for (c2_i236 = 0; c2_i236 <= c2_loop_ub; c2_i236++) {
    c2_xxi_data[c2_i236] = c2_varargin_5_data[c2_i236];
  }

  c2_nxxi = (real_T)c2_xxi_sizes[1];
  for (c2_i237 = 0; c2_i237 < 60; c2_i237++) {
    c2_varargin_3[c2_i237] = c2_b_varargin_3[c2_i237];
  }

  c2_b_spline(chartInstance, c2_varargin_3, &c2_ppk);
  c2_sv[0] = (real_T)c2_varargin_5_sizes[1];
  c2_sv[1] = 12.0;
  for (c2_i238 = 0; c2_i238 < 2; c2_i238++) {
    c2_sv[c2_i238] = _SFD_NON_NEGATIVE_CHECK("", c2_sv[c2_i238]);
  }

  c2_varargout_1_sizes[0] = (int32_T)c2_sv[0];
  c2_varargout_1_sizes[1] = 12;
  c2_varargout_1 = c2_varargout_1_sizes[0];
  c2_b_varargout_1 = c2_varargout_1_sizes[1];
  c2_b_loop_ub = (int32_T)c2_sv[0] * (int32_T)c2_sv[1] - 1;
  for (c2_i239 = 0; c2_i239 <= c2_b_loop_ub; c2_i239++) {
    c2_varargout_1_data[c2_i239] = 0.0;
  }

  c2_b_nxxi = c2_nxxi;
  c2_i240 = (int32_T)c2_b_nxxi;
  _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_b_nxxi, mxDOUBLE_CLASS, c2_i240);
  for (c2_j = 0; c2_j < c2_i240; c2_j++) {
    c2_b_j = 1.0 + (real_T)c2_j;
    c2_b_ppk = c2_ppk;
    c2_d_xxi[0] = c2_xxi_sizes[1];
    c2_e_ppval(chartInstance, &c2_b_ppk, c2_xxi_data[(int32_T)c2_b_j - 1],
               c2_vkj);
    for (c2_i = 0; c2_i < 12; c2_i++) {
      c2_b_i = 1.0 + (real_T)c2_i;
      c2_varargout_1_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
        c2_vkj[(int32_T)c2_b_i - 1];
    }
  }

  c2_xxi_sizes[0] = 1;
  c2_xxi_sizes[1] = c2_varargin_4_sizes[1];
  c2_c_xxi = c2_xxi_sizes[0];
  c2_e_xxi = c2_xxi_sizes[1];
  c2_c_loop_ub = c2_varargin_4_sizes[0] * c2_varargin_4_sizes[1] - 1;
  for (c2_i241 = 0; c2_i241 <= c2_c_loop_ub; c2_i241++) {
    c2_xxi_data[c2_i241] = c2_varargin_4_data[c2_i241];
  }

  c2_nxxi = (real_T)c2_xxi_sizes[1];
  c2_d_spline(chartInstance, c2_varargout_1_data, c2_varargout_1_sizes,
              &c2_ppk_data, &c2_ppk_elems_sizes);
  for (c2_i242 = 0; c2_i242 < 2; c2_i242++) {
    c2_outsize[c2_i242] = _SFD_NON_NEGATIVE_CHECK("", c2_outsize[c2_i242]);
  }

  c2_yi_sizes[0] = (int32_T)c2_outsize[0];
  c2_yi_sizes[1] = (int32_T)c2_outsize[1];
  c2_yi = c2_yi_sizes[0];
  c2_b_yi = c2_yi_sizes[1];
  c2_d_loop_ub = (int32_T)c2_outsize[0] * (int32_T)c2_outsize[1] - 1;
  for (c2_i243 = 0; c2_i243 <= c2_d_loop_ub; c2_i243++) {
    c2_yi_data[c2_i243] = 0.0;
  }

  c2_c_nxxi = c2_nxxi;
  c2_i244 = (int32_T)c2_c_nxxi;
  _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_c_nxxi, mxDOUBLE_CLASS, c2_i244);
  for (c2_c_j = 0; c2_c_j < c2_i244; c2_c_j++) {
    c2_b_j = 1.0 + (real_T)c2_c_j;
    c2_b_ppk_elems_sizes = c2_ppk_elems_sizes;
    c2_b_ppk_data = c2_ppk_data;
    c2_f_xxi[0] = c2_xxi_sizes[1];
    c2_f_ppval(chartInstance, &c2_b_ppk_data, c2_b_ppk_elems_sizes, c2_xxi_data
               [(int32_T)c2_b_j - 1], c2_vkj_data, &c2_vkj_sizes);
    c2_d12 = (real_T)c2_vkj_sizes;
    c2_i245 = (int32_T)c2_d12;
    _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_d12, mxDOUBLE_CLASS, c2_i245);
    for (c2_c_i = 0; c2_c_i < c2_i245; c2_c_i++) {
      c2_b_i = 1.0 + (real_T)c2_c_i;
      c2_yi_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
        c2_vkj_data[(int32_T)c2_b_i - 1];
    }
  }
}

static real_T c2_i_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_Xq;
  real_T c2_Yq;
  c2_Xq = c2_varargin_4;
  c2_Yq = c2_varargin_5;
  return c2_d_interp2_dispatch(chartInstance, c2_Xq, c2_Yq);
}

static real_T c2_d_interp2_dispatch(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_Xq, real_T c2_Yq)
{
  real_T c2_Vq;
  real_T c2_b_Xq;
  real_T c2_b_Yq;
  real_T c2_varargin_4;
  real_T c2_varargin_5;
  real_T c2_a;
  real_T c2_uxi;
  real_T c2_uxi_data[1];
  int32_T c2_uxi_sizes[2];
  int32_T c2_indx_data[1];
  int32_T c2_indx_sizes;
  int32_T c2_ipos;
  real_T c2_uyi;
  int32_T c2_b_ipos;
  int32_T c2_i246;
  real_T c2_ix;
  real_T c2_b_ix;
  real_T c2_b_a;
  real_T c2_dv35[84];
  static real_T c2_dv36[84] = { 0.001, -0.027, -0.017, -0.013, -0.012, -0.016,
    0.001, 0.017, 0.011, 0.017, 0.008, 0.016, 0.002, -0.014, -0.016, -0.016,
    -0.014, -0.019, -0.021, 0.002, 0.012, 0.015, 0.015, 0.011, -0.006, -0.008,
    -0.006, -0.006, -0.005, -0.008, -0.005, 0.007, 0.004, 0.007, 0.006, 0.006,
    -0.011, -0.011, -0.01, -0.009, -0.008, -0.006, 0.0, 0.004, 0.007, 0.1, 0.004,
    0.1, -0.015, -0.015, -0.014, -0.012, -0.011, -0.008, -0.002, 0.002, 0.006,
    0.012, 0.011, 0.011, -0.024, -0.01, -0.004, -0.002, -0.001, 0.003, 0.014,
    0.006, -0.001, 0.004, 0.004, 0.006, -0.022, 0.002, -0.003, -0.005, -0.003,
    -0.001, -0.009, -0.009, -0.001, 0.003, -0.002, 0.001 };

  real_T c2_uyi_data[1];
  int32_T c2_uyi_sizes[2];
  int32_T c2_c_ipos;
  int32_T c2_d_ipos;
  real_T c2_iy;
  real_T c2_b_iy;
  real_T c2_uwi_data[1];
  int32_T c2_uwi_sizes[2];
  c2_b_Xq = c2_Xq;
  c2_b_Yq = c2_Yq;
  c2_varargin_4 = c2_b_Yq;
  c2_varargin_5 = c2_b_Xq;
  if (c2_isplaid(chartInstance, c2_varargin_4, c2_varargin_5)) {
    c2_uxi = c2_varargin_4;
    c2_uyi = c2_varargin_5;
    for (c2_i246 = 0; c2_i246 < 84; c2_i246++) {
      c2_dv35[c2_i246] = c2_dv36[c2_i246];
    }

    c2_Vq = c2_g_TensorGriddedInterp(chartInstance, c2_dv35, c2_uxi, c2_uyi);
  } else {
    c2_a = c2_varargin_4;
    c2_unique_vector(chartInstance, c2_a, c2_uxi_data, c2_uxi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_ipos);
    c2_b_ipos = c2_ipos;
    c2_ix = (real_T)c2_b_ipos;
    c2_b_ix = c2_ix;
    c2_b_a = c2_varargin_5;
    c2_unique_vector(chartInstance, c2_b_a, c2_uyi_data, c2_uyi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_c_ipos);
    c2_d_ipos = c2_c_ipos;
    c2_iy = (real_T)c2_d_ipos;
    c2_b_iy = c2_iy;
    c2_m_TensorGriddedInterp(chartInstance, c2_uxi_data, c2_uxi_sizes,
      c2_uyi_data, c2_uyi_sizes, c2_uwi_data, c2_uwi_sizes);
    c2_Vq = c2_uwi_data[((int32_T)c2_b_ix + c2_uwi_sizes[0] * ((int32_T)c2_b_iy
      - 1)) - 1];
  }

  return c2_Vq;
}

static void c2_m_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4_data[], int32_T c2_varargin_4_sizes[2], real_T
  c2_varargin_5_data[], int32_T c2_varargin_5_sizes[2], real_T c2_yi_data[],
  int32_T c2_yi_sizes[2])
{
  int32_T c2_b_varargin_4_sizes[2];
  int32_T c2_varargin_4;
  int32_T c2_b_varargin_4;
  int32_T c2_loop_ub;
  int32_T c2_i247;
  int32_T c2_b_varargin_5_sizes[2];
  real_T c2_b_varargin_4_data[1];
  int32_T c2_varargin_5;
  int32_T c2_b_varargin_5;
  int32_T c2_b_loop_ub;
  int32_T c2_i248;
  real_T c2_b_varargin_5_data[1];
  real_T c2_outsize[2];
  int32_T c2_xxi_sizes[2];
  int32_T c2_xxi;
  int32_T c2_b_xxi;
  int32_T c2_c_loop_ub;
  int32_T c2_i249;
  real_T c2_nxxi;
  real_T c2_xxi_data[1];
  int32_T c2_i250;
  real_T c2_varargin_3[84];
  c2_sPMb9Nq525A1JP3SLksK2fC c2_ppk;
  static real_T c2_b_varargin_3[84] = { 0.001, -0.027, -0.017, -0.013, -0.012,
    -0.016, 0.001, 0.017, 0.011, 0.017, 0.008, 0.016, 0.002, -0.014, -0.016,
    -0.016, -0.014, -0.019, -0.021, 0.002, 0.012, 0.015, 0.015, 0.011, -0.006,
    -0.008, -0.006, -0.006, -0.005, -0.008, -0.005, 0.007, 0.004, 0.007, 0.006,
    0.006, -0.011, -0.011, -0.01, -0.009, -0.008, -0.006, 0.0, 0.004, 0.007, 0.1,
    0.004, 0.1, -0.015, -0.015, -0.014, -0.012, -0.011, -0.008, -0.002, 0.002,
    0.006, 0.012, 0.011, 0.011, -0.024, -0.01, -0.004, -0.002, -0.001, 0.003,
    0.014, 0.006, -0.001, 0.004, 0.004, 0.006, -0.022, 0.002, -0.003, -0.005,
    -0.003, -0.001, -0.009, -0.009, -0.001, 0.003, -0.002, 0.001 };

  int32_T c2_c_varargin_5_sizes[2];
  int32_T c2_c_varargin_5;
  int32_T c2_d_varargin_5;
  int32_T c2_d_loop_ub;
  int32_T c2_i251;
  real_T c2_c_varargin_5_data[1];
  real_T c2_sv[2];
  int32_T c2_i252;
  int32_T c2_varargout_1_sizes[2];
  int32_T c2_varargout_1;
  int32_T c2_b_varargout_1;
  int32_T c2_e_loop_ub;
  int32_T c2_i253;
  real_T c2_b_nxxi;
  real_T c2_varargout_1_data[12];
  int32_T c2_i254;
  int32_T c2_j;
  real_T c2_b_j;
  c2_sPMb9Nq525A1JP3SLksK2fC c2_b_ppk;
  int32_T c2_c_xxi;
  int32_T c2_d_xxi[1];
  int32_T c2_e_xxi;
  real_T c2_vkj[12];
  int32_T c2_f_loop_ub;
  int32_T c2_i;
  int32_T c2_i255;
  real_T c2_b_i;
  c2_sL3Gi7vZqpbfvDjMPafkGnG c2_ppk_data;
  c2_sL3Gi7vZqpbfvDjMPafkGnG_size c2_ppk_elems_sizes;
  int32_T c2_i256;
  int32_T c2_yi;
  int32_T c2_b_yi;
  int32_T c2_g_loop_ub;
  int32_T c2_i257;
  real_T c2_c_nxxi;
  int32_T c2_i258;
  int32_T c2_c_j;
  c2_sL3Gi7vZqpbfvDjMPafkGnG_size c2_b_ppk_elems_sizes;
  c2_sL3Gi7vZqpbfvDjMPafkGnG c2_b_ppk_data;
  int32_T c2_f_xxi[1];
  real_T c2_vkj_data[1];
  int32_T c2_vkj_sizes;
  real_T c2_d13;
  int32_T c2_i259;
  int32_T c2_c_i;
  c2_b_varargin_4_sizes[0] = 1;
  c2_b_varargin_4_sizes[1] = c2_varargin_4_sizes[1];
  c2_varargin_4 = c2_b_varargin_4_sizes[0];
  c2_b_varargin_4 = c2_b_varargin_4_sizes[1];
  c2_loop_ub = c2_varargin_4_sizes[0] * c2_varargin_4_sizes[1] - 1;
  for (c2_i247 = 0; c2_i247 <= c2_loop_ub; c2_i247++) {
    c2_b_varargin_4_data[c2_i247] = c2_varargin_4_data[c2_i247];
  }

  c2_b_varargin_5_sizes[0] = 1;
  c2_b_varargin_5_sizes[1] = c2_varargin_5_sizes[1];
  c2_varargin_5 = c2_b_varargin_5_sizes[0];
  c2_b_varargin_5 = c2_b_varargin_5_sizes[1];
  c2_b_loop_ub = c2_varargin_5_sizes[0] * c2_varargin_5_sizes[1] - 1;
  for (c2_i248 = 0; c2_i248 <= c2_b_loop_ub; c2_i248++) {
    c2_b_varargin_5_data[c2_i248] = c2_varargin_5_data[c2_i248];
  }

  c2_output_size(chartInstance, c2_b_varargin_4_data, c2_b_varargin_4_sizes,
                 c2_b_varargin_5_data, c2_b_varargin_5_sizes, c2_outsize);
  c2_xxi_sizes[0] = 1;
  c2_xxi_sizes[1] = c2_varargin_5_sizes[1];
  c2_xxi = c2_xxi_sizes[0];
  c2_b_xxi = c2_xxi_sizes[1];
  c2_c_loop_ub = c2_varargin_5_sizes[0] * c2_varargin_5_sizes[1] - 1;
  for (c2_i249 = 0; c2_i249 <= c2_c_loop_ub; c2_i249++) {
    c2_xxi_data[c2_i249] = c2_varargin_5_data[c2_i249];
  }

  c2_nxxi = (real_T)c2_xxi_sizes[1];
  for (c2_i250 = 0; c2_i250 < 84; c2_i250++) {
    c2_varargin_3[c2_i250] = c2_b_varargin_3[c2_i250];
  }

  c2_e_spline(chartInstance, c2_varargin_3, &c2_ppk);
  c2_c_varargin_5_sizes[0] = 1;
  c2_c_varargin_5_sizes[1] = c2_varargin_5_sizes[1];
  c2_c_varargin_5 = c2_c_varargin_5_sizes[0];
  c2_d_varargin_5 = c2_c_varargin_5_sizes[1];
  c2_d_loop_ub = c2_varargin_5_sizes[0] * c2_varargin_5_sizes[1] - 1;
  for (c2_i251 = 0; c2_i251 <= c2_d_loop_ub; c2_i251++) {
    c2_c_varargin_5_data[c2_i251] = c2_varargin_5_data[c2_i251];
  }

  c2_intermediate_size(chartInstance, c2_c_varargin_5_data,
                       c2_c_varargin_5_sizes, c2_sv);
  for (c2_i252 = 0; c2_i252 < 2; c2_i252++) {
    c2_sv[c2_i252] = _SFD_NON_NEGATIVE_CHECK("", c2_sv[c2_i252]);
  }

  c2_varargout_1_sizes[0] = (int32_T)c2_sv[0];
  c2_varargout_1_sizes[1] = 12;
  c2_varargout_1 = c2_varargout_1_sizes[0];
  c2_b_varargout_1 = c2_varargout_1_sizes[1];
  c2_e_loop_ub = (int32_T)c2_sv[0] * (int32_T)c2_sv[1] - 1;
  for (c2_i253 = 0; c2_i253 <= c2_e_loop_ub; c2_i253++) {
    c2_varargout_1_data[c2_i253] = 0.0;
  }

  c2_b_nxxi = c2_nxxi;
  c2_i254 = (int32_T)c2_b_nxxi;
  _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_b_nxxi, mxDOUBLE_CLASS, c2_i254);
  for (c2_j = 0; c2_j < c2_i254; c2_j++) {
    c2_b_j = 1.0 + (real_T)c2_j;
    c2_b_ppk = c2_ppk;
    c2_d_xxi[0] = c2_xxi_sizes[1];
    c2_g_ppval(chartInstance, &c2_b_ppk, c2_xxi_data[(int32_T)c2_b_j - 1],
               c2_vkj);
    for (c2_i = 0; c2_i < 12; c2_i++) {
      c2_b_i = 1.0 + (real_T)c2_i;
      c2_varargout_1_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
        c2_vkj[(int32_T)c2_b_i - 1];
    }
  }

  c2_xxi_sizes[0] = 1;
  c2_xxi_sizes[1] = c2_varargin_4_sizes[1];
  c2_c_xxi = c2_xxi_sizes[0];
  c2_e_xxi = c2_xxi_sizes[1];
  c2_f_loop_ub = c2_varargin_4_sizes[0] * c2_varargin_4_sizes[1] - 1;
  for (c2_i255 = 0; c2_i255 <= c2_f_loop_ub; c2_i255++) {
    c2_xxi_data[c2_i255] = c2_varargin_4_data[c2_i255];
  }

  c2_nxxi = (real_T)c2_xxi_sizes[1];
  c2_d_spline(chartInstance, c2_varargout_1_data, c2_varargout_1_sizes,
              &c2_ppk_data, &c2_ppk_elems_sizes);
  for (c2_i256 = 0; c2_i256 < 2; c2_i256++) {
    c2_outsize[c2_i256] = _SFD_NON_NEGATIVE_CHECK("", c2_outsize[c2_i256]);
  }

  c2_yi_sizes[0] = (int32_T)c2_outsize[0];
  c2_yi_sizes[1] = (int32_T)c2_outsize[1];
  c2_yi = c2_yi_sizes[0];
  c2_b_yi = c2_yi_sizes[1];
  c2_g_loop_ub = (int32_T)c2_outsize[0] * (int32_T)c2_outsize[1] - 1;
  for (c2_i257 = 0; c2_i257 <= c2_g_loop_ub; c2_i257++) {
    c2_yi_data[c2_i257] = 0.0;
  }

  c2_c_nxxi = c2_nxxi;
  c2_i258 = (int32_T)c2_c_nxxi;
  _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_c_nxxi, mxDOUBLE_CLASS, c2_i258);
  for (c2_c_j = 0; c2_c_j < c2_i258; c2_c_j++) {
    c2_b_j = 1.0 + (real_T)c2_c_j;
    c2_b_ppk_elems_sizes = c2_ppk_elems_sizes;
    c2_b_ppk_data = c2_ppk_data;
    c2_f_xxi[0] = c2_xxi_sizes[1];
    c2_f_ppval(chartInstance, &c2_b_ppk_data, c2_b_ppk_elems_sizes, c2_xxi_data
               [(int32_T)c2_b_j - 1], c2_vkj_data, &c2_vkj_sizes);
    c2_d13 = (real_T)c2_vkj_sizes;
    c2_i259 = (int32_T)c2_d13;
    _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_d13, mxDOUBLE_CLASS, c2_i259);
    for (c2_c_i = 0; c2_c_i < c2_i259; c2_c_i++) {
      c2_b_i = 1.0 + (real_T)c2_c_i;
      c2_yi_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
        c2_vkj_data[(int32_T)c2_b_i - 1];
    }
  }
}

static real_T c2_j_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_Xq;
  real_T c2_Yq;
  c2_Xq = c2_varargin_4;
  c2_Yq = c2_varargin_5;
  return c2_e_interp2_dispatch(chartInstance, c2_Xq, c2_Yq);
}

static real_T c2_e_interp2_dispatch(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_Xq, real_T c2_Yq)
{
  real_T c2_Vq;
  real_T c2_b_Xq;
  real_T c2_b_Yq;
  real_T c2_varargin_4;
  real_T c2_varargin_5;
  real_T c2_a;
  real_T c2_uxi;
  real_T c2_uxi_data[1];
  int32_T c2_uxi_sizes[2];
  int32_T c2_indx_data[1];
  int32_T c2_indx_sizes;
  int32_T c2_ipos;
  real_T c2_uyi;
  int32_T c2_b_ipos;
  int32_T c2_i260;
  real_T c2_ix;
  real_T c2_b_ix;
  real_T c2_b_a;
  real_T c2_dv37[84];
  static real_T c2_dv38[84] = { -0.018, -0.052, -0.052, -0.052, -0.054, -0.049,
    -0.059, -0.051, -0.03, -0.037, -0.026, -0.013, -0.028, -0.051, -0.043,
    -0.046, -0.045, -0.049, -0.057, -0.052, -0.03, -0.033, -0.03, -0.008, -0.037,
    -0.041, -0.038, -0.04, -0.04, -0.038, -0.037, -0.03, -0.027, -0.024, -0.019,
    -0.013, -0.048, -0.045, -0.045, -0.045, -0.044, -0.045, -0.047, -0.048,
    -0.049, -0.045, -0.033, -0.016, -0.043, -0.044, -0.041, -0.041, -0.04,
    -0.038, -0.034, -0.035, -0.035, -0.029, -0.022, -0.009, -0.052, -0.034,
    -0.036, -0.036, -0.035, -0.028, -0.024, -0.023, -0.02, -0.016, -0.01, -0.014,
    -0.062, -0.034, -0.027, -0.028, -0.027, -0.027, -0.023, -0.023, -0.019,
    -0.009, -0.025, -0.01 };

  real_T c2_uyi_data[1];
  int32_T c2_uyi_sizes[2];
  int32_T c2_c_ipos;
  int32_T c2_d_ipos;
  real_T c2_iy;
  real_T c2_b_iy;
  real_T c2_uwi_data[1];
  int32_T c2_uwi_sizes[2];
  c2_b_Xq = c2_Xq;
  c2_b_Yq = c2_Yq;
  c2_varargin_4 = c2_b_Yq;
  c2_varargin_5 = c2_b_Xq;
  if (c2_isplaid(chartInstance, c2_varargin_4, c2_varargin_5)) {
    c2_uxi = c2_varargin_4;
    c2_uyi = c2_varargin_5;
    for (c2_i260 = 0; c2_i260 < 84; c2_i260++) {
      c2_dv37[c2_i260] = c2_dv38[c2_i260];
    }

    c2_Vq = c2_g_TensorGriddedInterp(chartInstance, c2_dv37, c2_uxi, c2_uyi);
  } else {
    c2_a = c2_varargin_4;
    c2_unique_vector(chartInstance, c2_a, c2_uxi_data, c2_uxi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_ipos);
    c2_b_ipos = c2_ipos;
    c2_ix = (real_T)c2_b_ipos;
    c2_b_ix = c2_ix;
    c2_b_a = c2_varargin_5;
    c2_unique_vector(chartInstance, c2_b_a, c2_uyi_data, c2_uyi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_c_ipos);
    c2_d_ipos = c2_c_ipos;
    c2_iy = (real_T)c2_d_ipos;
    c2_b_iy = c2_iy;
    c2_n_TensorGriddedInterp(chartInstance, c2_uxi_data, c2_uxi_sizes,
      c2_uyi_data, c2_uyi_sizes, c2_uwi_data, c2_uwi_sizes);
    c2_Vq = c2_uwi_data[((int32_T)c2_b_ix + c2_uwi_sizes[0] * ((int32_T)c2_b_iy
      - 1)) - 1];
  }

  return c2_Vq;
}

static void c2_n_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4_data[], int32_T c2_varargin_4_sizes[2], real_T
  c2_varargin_5_data[], int32_T c2_varargin_5_sizes[2], real_T c2_yi_data[],
  int32_T c2_yi_sizes[2])
{
  int32_T c2_b_varargin_4_sizes[2];
  int32_T c2_varargin_4;
  int32_T c2_b_varargin_4;
  int32_T c2_loop_ub;
  int32_T c2_i261;
  int32_T c2_b_varargin_5_sizes[2];
  real_T c2_b_varargin_4_data[1];
  int32_T c2_varargin_5;
  int32_T c2_b_varargin_5;
  int32_T c2_b_loop_ub;
  int32_T c2_i262;
  real_T c2_b_varargin_5_data[1];
  real_T c2_outsize[2];
  int32_T c2_xxi_sizes[2];
  int32_T c2_xxi;
  int32_T c2_b_xxi;
  int32_T c2_c_loop_ub;
  int32_T c2_i263;
  real_T c2_nxxi;
  real_T c2_xxi_data[1];
  int32_T c2_i264;
  real_T c2_varargin_3[84];
  c2_sPMb9Nq525A1JP3SLksK2fC c2_ppk;
  static real_T c2_b_varargin_3[84] = { -0.018, -0.052, -0.052, -0.052, -0.054,
    -0.049, -0.059, -0.051, -0.03, -0.037, -0.026, -0.013, -0.028, -0.051,
    -0.043, -0.046, -0.045, -0.049, -0.057, -0.052, -0.03, -0.033, -0.03, -0.008,
    -0.037, -0.041, -0.038, -0.04, -0.04, -0.038, -0.037, -0.03, -0.027, -0.024,
    -0.019, -0.013, -0.048, -0.045, -0.045, -0.045, -0.044, -0.045, -0.047,
    -0.048, -0.049, -0.045, -0.033, -0.016, -0.043, -0.044, -0.041, -0.041,
    -0.04, -0.038, -0.034, -0.035, -0.035, -0.029, -0.022, -0.009, -0.052,
    -0.034, -0.036, -0.036, -0.035, -0.028, -0.024, -0.023, -0.02, -0.016, -0.01,
    -0.014, -0.062, -0.034, -0.027, -0.028, -0.027, -0.027, -0.023, -0.023,
    -0.019, -0.009, -0.025, -0.01 };

  int32_T c2_c_varargin_5_sizes[2];
  int32_T c2_c_varargin_5;
  int32_T c2_d_varargin_5;
  int32_T c2_d_loop_ub;
  int32_T c2_i265;
  real_T c2_c_varargin_5_data[1];
  real_T c2_sv[2];
  int32_T c2_i266;
  int32_T c2_varargout_1_sizes[2];
  int32_T c2_varargout_1;
  int32_T c2_b_varargout_1;
  int32_T c2_e_loop_ub;
  int32_T c2_i267;
  real_T c2_b_nxxi;
  real_T c2_varargout_1_data[12];
  int32_T c2_i268;
  int32_T c2_j;
  real_T c2_b_j;
  c2_sPMb9Nq525A1JP3SLksK2fC c2_b_ppk;
  int32_T c2_c_xxi;
  int32_T c2_d_xxi[1];
  int32_T c2_e_xxi;
  real_T c2_vkj[12];
  int32_T c2_f_loop_ub;
  int32_T c2_i;
  int32_T c2_i269;
  real_T c2_b_i;
  c2_sL3Gi7vZqpbfvDjMPafkGnG c2_ppk_data;
  c2_sL3Gi7vZqpbfvDjMPafkGnG_size c2_ppk_elems_sizes;
  int32_T c2_i270;
  int32_T c2_yi;
  int32_T c2_b_yi;
  int32_T c2_g_loop_ub;
  int32_T c2_i271;
  real_T c2_c_nxxi;
  int32_T c2_i272;
  int32_T c2_c_j;
  c2_sL3Gi7vZqpbfvDjMPafkGnG_size c2_b_ppk_elems_sizes;
  c2_sL3Gi7vZqpbfvDjMPafkGnG c2_b_ppk_data;
  int32_T c2_f_xxi[1];
  real_T c2_vkj_data[1];
  int32_T c2_vkj_sizes;
  real_T c2_d14;
  int32_T c2_i273;
  int32_T c2_c_i;
  c2_b_varargin_4_sizes[0] = 1;
  c2_b_varargin_4_sizes[1] = c2_varargin_4_sizes[1];
  c2_varargin_4 = c2_b_varargin_4_sizes[0];
  c2_b_varargin_4 = c2_b_varargin_4_sizes[1];
  c2_loop_ub = c2_varargin_4_sizes[0] * c2_varargin_4_sizes[1] - 1;
  for (c2_i261 = 0; c2_i261 <= c2_loop_ub; c2_i261++) {
    c2_b_varargin_4_data[c2_i261] = c2_varargin_4_data[c2_i261];
  }

  c2_b_varargin_5_sizes[0] = 1;
  c2_b_varargin_5_sizes[1] = c2_varargin_5_sizes[1];
  c2_varargin_5 = c2_b_varargin_5_sizes[0];
  c2_b_varargin_5 = c2_b_varargin_5_sizes[1];
  c2_b_loop_ub = c2_varargin_5_sizes[0] * c2_varargin_5_sizes[1] - 1;
  for (c2_i262 = 0; c2_i262 <= c2_b_loop_ub; c2_i262++) {
    c2_b_varargin_5_data[c2_i262] = c2_varargin_5_data[c2_i262];
  }

  c2_output_size(chartInstance, c2_b_varargin_4_data, c2_b_varargin_4_sizes,
                 c2_b_varargin_5_data, c2_b_varargin_5_sizes, c2_outsize);
  c2_xxi_sizes[0] = 1;
  c2_xxi_sizes[1] = c2_varargin_5_sizes[1];
  c2_xxi = c2_xxi_sizes[0];
  c2_b_xxi = c2_xxi_sizes[1];
  c2_c_loop_ub = c2_varargin_5_sizes[0] * c2_varargin_5_sizes[1] - 1;
  for (c2_i263 = 0; c2_i263 <= c2_c_loop_ub; c2_i263++) {
    c2_xxi_data[c2_i263] = c2_varargin_5_data[c2_i263];
  }

  c2_nxxi = (real_T)c2_xxi_sizes[1];
  for (c2_i264 = 0; c2_i264 < 84; c2_i264++) {
    c2_varargin_3[c2_i264] = c2_b_varargin_3[c2_i264];
  }

  c2_e_spline(chartInstance, c2_varargin_3, &c2_ppk);
  c2_c_varargin_5_sizes[0] = 1;
  c2_c_varargin_5_sizes[1] = c2_varargin_5_sizes[1];
  c2_c_varargin_5 = c2_c_varargin_5_sizes[0];
  c2_d_varargin_5 = c2_c_varargin_5_sizes[1];
  c2_d_loop_ub = c2_varargin_5_sizes[0] * c2_varargin_5_sizes[1] - 1;
  for (c2_i265 = 0; c2_i265 <= c2_d_loop_ub; c2_i265++) {
    c2_c_varargin_5_data[c2_i265] = c2_varargin_5_data[c2_i265];
  }

  c2_intermediate_size(chartInstance, c2_c_varargin_5_data,
                       c2_c_varargin_5_sizes, c2_sv);
  for (c2_i266 = 0; c2_i266 < 2; c2_i266++) {
    c2_sv[c2_i266] = _SFD_NON_NEGATIVE_CHECK("", c2_sv[c2_i266]);
  }

  c2_varargout_1_sizes[0] = (int32_T)c2_sv[0];
  c2_varargout_1_sizes[1] = 12;
  c2_varargout_1 = c2_varargout_1_sizes[0];
  c2_b_varargout_1 = c2_varargout_1_sizes[1];
  c2_e_loop_ub = (int32_T)c2_sv[0] * (int32_T)c2_sv[1] - 1;
  for (c2_i267 = 0; c2_i267 <= c2_e_loop_ub; c2_i267++) {
    c2_varargout_1_data[c2_i267] = 0.0;
  }

  c2_b_nxxi = c2_nxxi;
  c2_i268 = (int32_T)c2_b_nxxi;
  _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_b_nxxi, mxDOUBLE_CLASS, c2_i268);
  for (c2_j = 0; c2_j < c2_i268; c2_j++) {
    c2_b_j = 1.0 + (real_T)c2_j;
    c2_b_ppk = c2_ppk;
    c2_d_xxi[0] = c2_xxi_sizes[1];
    c2_g_ppval(chartInstance, &c2_b_ppk, c2_xxi_data[(int32_T)c2_b_j - 1],
               c2_vkj);
    for (c2_i = 0; c2_i < 12; c2_i++) {
      c2_b_i = 1.0 + (real_T)c2_i;
      c2_varargout_1_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
        c2_vkj[(int32_T)c2_b_i - 1];
    }
  }

  c2_xxi_sizes[0] = 1;
  c2_xxi_sizes[1] = c2_varargin_4_sizes[1];
  c2_c_xxi = c2_xxi_sizes[0];
  c2_e_xxi = c2_xxi_sizes[1];
  c2_f_loop_ub = c2_varargin_4_sizes[0] * c2_varargin_4_sizes[1] - 1;
  for (c2_i269 = 0; c2_i269 <= c2_f_loop_ub; c2_i269++) {
    c2_xxi_data[c2_i269] = c2_varargin_4_data[c2_i269];
  }

  c2_nxxi = (real_T)c2_xxi_sizes[1];
  c2_d_spline(chartInstance, c2_varargout_1_data, c2_varargout_1_sizes,
              &c2_ppk_data, &c2_ppk_elems_sizes);
  for (c2_i270 = 0; c2_i270 < 2; c2_i270++) {
    c2_outsize[c2_i270] = _SFD_NON_NEGATIVE_CHECK("", c2_outsize[c2_i270]);
  }

  c2_yi_sizes[0] = (int32_T)c2_outsize[0];
  c2_yi_sizes[1] = (int32_T)c2_outsize[1];
  c2_yi = c2_yi_sizes[0];
  c2_b_yi = c2_yi_sizes[1];
  c2_g_loop_ub = (int32_T)c2_outsize[0] * (int32_T)c2_outsize[1] - 1;
  for (c2_i271 = 0; c2_i271 <= c2_g_loop_ub; c2_i271++) {
    c2_yi_data[c2_i271] = 0.0;
  }

  c2_c_nxxi = c2_nxxi;
  c2_i272 = (int32_T)c2_c_nxxi;
  _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_c_nxxi, mxDOUBLE_CLASS, c2_i272);
  for (c2_c_j = 0; c2_c_j < c2_i272; c2_c_j++) {
    c2_b_j = 1.0 + (real_T)c2_c_j;
    c2_b_ppk_elems_sizes = c2_ppk_elems_sizes;
    c2_b_ppk_data = c2_ppk_data;
    c2_f_xxi[0] = c2_xxi_sizes[1];
    c2_f_ppval(chartInstance, &c2_b_ppk_data, c2_b_ppk_elems_sizes, c2_xxi_data
               [(int32_T)c2_b_j - 1], c2_vkj_data, &c2_vkj_sizes);
    c2_d14 = (real_T)c2_vkj_sizes;
    c2_i273 = (int32_T)c2_d14;
    _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_d14, mxDOUBLE_CLASS, c2_i273);
    for (c2_c_i = 0; c2_c_i < c2_i273; c2_c_i++) {
      c2_b_i = 1.0 + (real_T)c2_c_i;
      c2_yi_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
        c2_vkj_data[(int32_T)c2_b_i - 1];
    }
  }
}

static real_T c2_k_interp2(SFc2_QuanInstanceStruct *chartInstance, real_T
  c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_Xq;
  real_T c2_Yq;
  c2_Xq = c2_varargin_4;
  c2_Yq = c2_varargin_5;
  return c2_f_interp2_dispatch(chartInstance, c2_Xq, c2_Yq);
}

static real_T c2_o_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_3[84], real_T c2_varargin_4, real_T c2_varargin_5)
{
  real_T c2_xxi;
  int32_T c2_i274;
  real_T c2_b_varargin_3[84];
  c2_sPMb9Nq525A1JP3SLksK2fC c2_ppk;
  int32_T c2_i275;
  real_T c2_vkj[12];
  real_T c2_varargout_1[12];
  int32_T c2_i;
  real_T c2_b_i;
  c2_sSYDZjSrzjRAmypAou1DUJH c2_b_ppk;
  real_T c2_b_vkj;
  c2_xxi = c2_varargin_5;
  for (c2_i274 = 0; c2_i274 < 84; c2_i274++) {
    c2_b_varargin_3[c2_i274] = c2_varargin_3[c2_i274];
  }

  c2_f_spline(chartInstance, c2_b_varargin_3, &c2_ppk);
  for (c2_i275 = 0; c2_i275 < 12; c2_i275++) {
    c2_varargout_1[c2_i275] = 0.0;
  }

  c2_g_ppval(chartInstance, &c2_ppk, c2_xxi, c2_vkj);
  for (c2_i = 0; c2_i < 12; c2_i++) {
    c2_b_i = 1.0 + (real_T)c2_i;
    c2_varargout_1[(int32_T)(1.0 + (c2_b_i - 1.0)) - 1] = c2_vkj[(int32_T)c2_b_i
      - 1];
  }

  c2_xxi = c2_varargin_4;
  c2_c_spline(chartInstance, c2_varargout_1, &c2_b_ppk);
  c2_b_vkj = c2_ppval(chartInstance, &c2_b_ppk, c2_xxi);
  return c2_b_vkj;
}

static real_T c2_f_interp2_dispatch(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_Xq, real_T c2_Yq)
{
  real_T c2_Vq;
  real_T c2_b_Xq;
  real_T c2_b_Yq;
  real_T c2_varargin_4;
  real_T c2_varargin_5;
  real_T c2_a;
  real_T c2_uxi;
  real_T c2_uxi_data[1];
  int32_T c2_uxi_sizes[2];
  int32_T c2_indx_data[1];
  int32_T c2_indx_sizes;
  int32_T c2_ipos;
  real_T c2_uyi;
  int32_T c2_b_ipos;
  int32_T c2_i276;
  real_T c2_ix;
  real_T c2_b_ix;
  real_T c2_b_a;
  real_T c2_dv39[84];
  static real_T c2_dv40[84] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.018, 0.019, 0.018, 0.019, 0.019, 0.018, 0.013, 0.007, 0.004,
    -0.014, -0.017, -0.033, 0.038, 0.042, 0.042, 0.042, 0.043, 0.039, 0.03,
    0.017, 0.004, -0.035, -0.047, -0.057, 0.056, 0.057, 0.059, 0.058, 0.058,
    0.053, 0.032, 0.012, 0.002, -0.046, -0.071, -0.073, 0.064, 0.077, 0.076,
    0.074, 0.073, 0.057, 0.029, 0.007, 0.012, -0.034, -0.065, -0.041, 0.074,
    0.086, 0.093, 0.089, 0.08, 0.062, 0.049, 0.022, 0.028, -0.012, -0.002,
    -0.013, 0.079, 0.09, 0.106, 0.106, 0.096, 0.08, 0.068, 0.03, 0.064, 0.015,
    0.011, -0.001 };

  real_T c2_uyi_data[1];
  int32_T c2_uyi_sizes[2];
  int32_T c2_c_ipos;
  int32_T c2_d_ipos;
  real_T c2_iy;
  real_T c2_b_iy;
  real_T c2_uwi_data[1];
  int32_T c2_uwi_sizes[2];
  c2_b_Xq = c2_Xq;
  c2_b_Yq = c2_Yq;
  c2_varargin_4 = c2_b_Yq;
  c2_varargin_5 = c2_b_Xq;
  if (c2_isplaid(chartInstance, c2_varargin_4, c2_varargin_5)) {
    c2_uxi = c2_varargin_4;
    c2_uyi = c2_varargin_5;
    for (c2_i276 = 0; c2_i276 < 84; c2_i276++) {
      c2_dv39[c2_i276] = c2_dv40[c2_i276];
    }

    c2_Vq = c2_o_TensorGriddedInterp(chartInstance, c2_dv39, c2_uxi, c2_uyi);
  } else {
    c2_a = c2_varargin_4;
    c2_unique_vector(chartInstance, c2_a, c2_uxi_data, c2_uxi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_ipos);
    c2_b_ipos = c2_ipos;
    c2_ix = (real_T)c2_b_ipos;
    c2_b_ix = c2_ix;
    c2_b_a = c2_varargin_5;
    c2_unique_vector(chartInstance, c2_b_a, c2_uyi_data, c2_uyi_sizes,
                     c2_indx_data, &c2_indx_sizes, &c2_c_ipos);
    c2_d_ipos = c2_c_ipos;
    c2_iy = (real_T)c2_d_ipos;
    c2_b_iy = c2_iy;
    c2_p_TensorGriddedInterp(chartInstance, c2_uxi_data, c2_uxi_sizes,
      c2_uyi_data, c2_uyi_sizes, c2_uwi_data, c2_uwi_sizes);
    c2_Vq = c2_uwi_data[((int32_T)c2_b_ix + c2_uwi_sizes[0] * ((int32_T)c2_b_iy
      - 1)) - 1];
  }

  return c2_Vq;
}

static void c2_p_TensorGriddedInterp(SFc2_QuanInstanceStruct *chartInstance,
  real_T c2_varargin_4_data[], int32_T c2_varargin_4_sizes[2], real_T
  c2_varargin_5_data[], int32_T c2_varargin_5_sizes[2], real_T c2_yi_data[],
  int32_T c2_yi_sizes[2])
{
  int32_T c2_b_varargin_4_sizes[2];
  int32_T c2_varargin_4;
  int32_T c2_b_varargin_4;
  int32_T c2_loop_ub;
  int32_T c2_i277;
  int32_T c2_b_varargin_5_sizes[2];
  real_T c2_b_varargin_4_data[1];
  int32_T c2_varargin_5;
  int32_T c2_b_varargin_5;
  int32_T c2_b_loop_ub;
  int32_T c2_i278;
  real_T c2_b_varargin_5_data[1];
  real_T c2_outsize[2];
  int32_T c2_xxi_sizes[2];
  int32_T c2_xxi;
  int32_T c2_b_xxi;
  int32_T c2_c_loop_ub;
  int32_T c2_i279;
  real_T c2_nxxi;
  real_T c2_xxi_data[1];
  int32_T c2_i280;
  real_T c2_varargin_3[84];
  c2_sPMb9Nq525A1JP3SLksK2fC c2_ppk;
  static real_T c2_b_varargin_3[84] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.018, 0.019, 0.018, 0.019, 0.019, 0.018, 0.013, 0.007,
    0.004, -0.014, -0.017, -0.033, 0.038, 0.042, 0.042, 0.042, 0.043, 0.039,
    0.03, 0.017, 0.004, -0.035, -0.047, -0.057, 0.056, 0.057, 0.059, 0.058,
    0.058, 0.053, 0.032, 0.012, 0.002, -0.046, -0.071, -0.073, 0.064, 0.077,
    0.076, 0.074, 0.073, 0.057, 0.029, 0.007, 0.012, -0.034, -0.065, -0.041,
    0.074, 0.086, 0.093, 0.089, 0.08, 0.062, 0.049, 0.022, 0.028, -0.012, -0.002,
    -0.013, 0.079, 0.09, 0.106, 0.106, 0.096, 0.08, 0.068, 0.03, 0.064, 0.015,
    0.011, -0.001 };

  int32_T c2_c_varargin_5_sizes[2];
  int32_T c2_c_varargin_5;
  int32_T c2_d_varargin_5;
  int32_T c2_d_loop_ub;
  int32_T c2_i281;
  real_T c2_c_varargin_5_data[1];
  real_T c2_sv[2];
  int32_T c2_i282;
  int32_T c2_varargout_1_sizes[2];
  int32_T c2_varargout_1;
  int32_T c2_b_varargout_1;
  int32_T c2_e_loop_ub;
  int32_T c2_i283;
  real_T c2_b_nxxi;
  real_T c2_varargout_1_data[12];
  int32_T c2_i284;
  int32_T c2_j;
  real_T c2_b_j;
  c2_sPMb9Nq525A1JP3SLksK2fC c2_b_ppk;
  int32_T c2_c_xxi;
  int32_T c2_d_xxi[1];
  int32_T c2_e_xxi;
  real_T c2_vkj[12];
  int32_T c2_f_loop_ub;
  int32_T c2_i;
  int32_T c2_i285;
  real_T c2_b_i;
  c2_sL3Gi7vZqpbfvDjMPafkGnG c2_ppk_data;
  c2_sL3Gi7vZqpbfvDjMPafkGnG_size c2_ppk_elems_sizes;
  int32_T c2_i286;
  int32_T c2_yi;
  int32_T c2_b_yi;
  int32_T c2_g_loop_ub;
  int32_T c2_i287;
  real_T c2_c_nxxi;
  int32_T c2_i288;
  int32_T c2_c_j;
  c2_sL3Gi7vZqpbfvDjMPafkGnG_size c2_b_ppk_elems_sizes;
  c2_sL3Gi7vZqpbfvDjMPafkGnG c2_b_ppk_data;
  int32_T c2_f_xxi[1];
  real_T c2_vkj_data[1];
  int32_T c2_vkj_sizes;
  real_T c2_d15;
  int32_T c2_i289;
  int32_T c2_c_i;
  c2_b_varargin_4_sizes[0] = 1;
  c2_b_varargin_4_sizes[1] = c2_varargin_4_sizes[1];
  c2_varargin_4 = c2_b_varargin_4_sizes[0];
  c2_b_varargin_4 = c2_b_varargin_4_sizes[1];
  c2_loop_ub = c2_varargin_4_sizes[0] * c2_varargin_4_sizes[1] - 1;
  for (c2_i277 = 0; c2_i277 <= c2_loop_ub; c2_i277++) {
    c2_b_varargin_4_data[c2_i277] = c2_varargin_4_data[c2_i277];
  }

  c2_b_varargin_5_sizes[0] = 1;
  c2_b_varargin_5_sizes[1] = c2_varargin_5_sizes[1];
  c2_varargin_5 = c2_b_varargin_5_sizes[0];
  c2_b_varargin_5 = c2_b_varargin_5_sizes[1];
  c2_b_loop_ub = c2_varargin_5_sizes[0] * c2_varargin_5_sizes[1] - 1;
  for (c2_i278 = 0; c2_i278 <= c2_b_loop_ub; c2_i278++) {
    c2_b_varargin_5_data[c2_i278] = c2_varargin_5_data[c2_i278];
  }

  c2_output_size(chartInstance, c2_b_varargin_4_data, c2_b_varargin_4_sizes,
                 c2_b_varargin_5_data, c2_b_varargin_5_sizes, c2_outsize);
  c2_xxi_sizes[0] = 1;
  c2_xxi_sizes[1] = c2_varargin_5_sizes[1];
  c2_xxi = c2_xxi_sizes[0];
  c2_b_xxi = c2_xxi_sizes[1];
  c2_c_loop_ub = c2_varargin_5_sizes[0] * c2_varargin_5_sizes[1] - 1;
  for (c2_i279 = 0; c2_i279 <= c2_c_loop_ub; c2_i279++) {
    c2_xxi_data[c2_i279] = c2_varargin_5_data[c2_i279];
  }

  c2_nxxi = (real_T)c2_xxi_sizes[1];
  for (c2_i280 = 0; c2_i280 < 84; c2_i280++) {
    c2_varargin_3[c2_i280] = c2_b_varargin_3[c2_i280];
  }

  c2_f_spline(chartInstance, c2_varargin_3, &c2_ppk);
  c2_c_varargin_5_sizes[0] = 1;
  c2_c_varargin_5_sizes[1] = c2_varargin_5_sizes[1];
  c2_c_varargin_5 = c2_c_varargin_5_sizes[0];
  c2_d_varargin_5 = c2_c_varargin_5_sizes[1];
  c2_d_loop_ub = c2_varargin_5_sizes[0] * c2_varargin_5_sizes[1] - 1;
  for (c2_i281 = 0; c2_i281 <= c2_d_loop_ub; c2_i281++) {
    c2_c_varargin_5_data[c2_i281] = c2_varargin_5_data[c2_i281];
  }

  c2_intermediate_size(chartInstance, c2_c_varargin_5_data,
                       c2_c_varargin_5_sizes, c2_sv);
  for (c2_i282 = 0; c2_i282 < 2; c2_i282++) {
    c2_sv[c2_i282] = _SFD_NON_NEGATIVE_CHECK("", c2_sv[c2_i282]);
  }

  c2_varargout_1_sizes[0] = (int32_T)c2_sv[0];
  c2_varargout_1_sizes[1] = 12;
  c2_varargout_1 = c2_varargout_1_sizes[0];
  c2_b_varargout_1 = c2_varargout_1_sizes[1];
  c2_e_loop_ub = (int32_T)c2_sv[0] * (int32_T)c2_sv[1] - 1;
  for (c2_i283 = 0; c2_i283 <= c2_e_loop_ub; c2_i283++) {
    c2_varargout_1_data[c2_i283] = 0.0;
  }

  c2_b_nxxi = c2_nxxi;
  c2_i284 = (int32_T)c2_b_nxxi;
  _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_b_nxxi, mxDOUBLE_CLASS, c2_i284);
  for (c2_j = 0; c2_j < c2_i284; c2_j++) {
    c2_b_j = 1.0 + (real_T)c2_j;
    c2_b_ppk = c2_ppk;
    c2_d_xxi[0] = c2_xxi_sizes[1];
    c2_g_ppval(chartInstance, &c2_b_ppk, c2_xxi_data[(int32_T)c2_b_j - 1],
               c2_vkj);
    for (c2_i = 0; c2_i < 12; c2_i++) {
      c2_b_i = 1.0 + (real_T)c2_i;
      c2_varargout_1_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
        c2_vkj[(int32_T)c2_b_i - 1];
    }
  }

  c2_xxi_sizes[0] = 1;
  c2_xxi_sizes[1] = c2_varargin_4_sizes[1];
  c2_c_xxi = c2_xxi_sizes[0];
  c2_e_xxi = c2_xxi_sizes[1];
  c2_f_loop_ub = c2_varargin_4_sizes[0] * c2_varargin_4_sizes[1] - 1;
  for (c2_i285 = 0; c2_i285 <= c2_f_loop_ub; c2_i285++) {
    c2_xxi_data[c2_i285] = c2_varargin_4_data[c2_i285];
  }

  c2_nxxi = (real_T)c2_xxi_sizes[1];
  c2_d_spline(chartInstance, c2_varargout_1_data, c2_varargout_1_sizes,
              &c2_ppk_data, &c2_ppk_elems_sizes);
  for (c2_i286 = 0; c2_i286 < 2; c2_i286++) {
    c2_outsize[c2_i286] = _SFD_NON_NEGATIVE_CHECK("", c2_outsize[c2_i286]);
  }

  c2_yi_sizes[0] = (int32_T)c2_outsize[0];
  c2_yi_sizes[1] = (int32_T)c2_outsize[1];
  c2_yi = c2_yi_sizes[0];
  c2_b_yi = c2_yi_sizes[1];
  c2_g_loop_ub = (int32_T)c2_outsize[0] * (int32_T)c2_outsize[1] - 1;
  for (c2_i287 = 0; c2_i287 <= c2_g_loop_ub; c2_i287++) {
    c2_yi_data[c2_i287] = 0.0;
  }

  c2_c_nxxi = c2_nxxi;
  c2_i288 = (int32_T)c2_c_nxxi;
  _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_c_nxxi, mxDOUBLE_CLASS, c2_i288);
  for (c2_c_j = 0; c2_c_j < c2_i288; c2_c_j++) {
    c2_b_j = 1.0 + (real_T)c2_c_j;
    c2_b_ppk_elems_sizes = c2_ppk_elems_sizes;
    c2_b_ppk_data = c2_ppk_data;
    c2_f_xxi[0] = c2_xxi_sizes[1];
    c2_f_ppval(chartInstance, &c2_b_ppk_data, c2_b_ppk_elems_sizes, c2_xxi_data
               [(int32_T)c2_b_j - 1], c2_vkj_data, &c2_vkj_sizes);
    c2_d15 = (real_T)c2_vkj_sizes;
    c2_i289 = (int32_T)c2_d15;
    _SFD_FOR_LOOP_VECTOR_CHECK(1.0, 1.0, c2_d15, mxDOUBLE_CLASS, c2_i289);
    for (c2_c_i = 0; c2_c_i < c2_i289; c2_c_i++) {
      c2_b_i = 1.0 + (real_T)c2_c_i;
      c2_yi_data[(int32_T)(c2_b_j + (c2_b_i - 1.0) * c2_nxxi) - 1] =
        c2_vkj_data[(int32_T)c2_b_i - 1];
    }
  }
}

static const mxArray *c2_l_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuanInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static int32_T c2_d_emlrt_marshallIn(SFc2_QuanInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i290;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i290, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i290;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuanInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_e_emlrt_marshallIn(SFc2_QuanInstanceStruct *chartInstance,
  const mxArray *c2_b_is_active_c2_Quan, const char_T *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_is_active_c2_Quan),
    &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_Quan);
  return c2_y;
}

static uint8_T c2_f_emlrt_marshallIn(SFc2_QuanInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sqrt(SFc2_QuanInstanceStruct *chartInstance, real_T *c2_x)
{
  real_T c2_b_x;
  boolean_T c2_b10;
  boolean_T c2_p;
  c2_b_x = *c2_x;
  c2_b10 = (c2_b_x < 0.0);
  c2_p = c2_b10;
  if (c2_p) {
    c2_b_error(chartInstance);
  }

  *c2_x = muDoubleScalarSqrt(*c2_x);
}

static void c2_b_atan(SFc2_QuanInstanceStruct *chartInstance, real_T *c2_x)
{
  (void)chartInstance;
  *c2_x = muDoubleScalarAtan(*c2_x);
}

static void c2_b_asin(SFc2_QuanInstanceStruct *chartInstance, real_T *c2_x)
{
  real_T c2_b_x;
  boolean_T c2_b11;
  boolean_T c2_b12;
  boolean_T c2_b13;
  boolean_T c2_p;
  c2_b_x = *c2_x;
  c2_b11 = (c2_b_x < -1.0);
  c2_b12 = (c2_b_x > 1.0);
  c2_b13 = (c2_b11 || c2_b12);
  c2_p = c2_b13;
  if (c2_p) {
    c2_c_error(chartInstance);
  }

  *c2_x = muDoubleScalarAsin(*c2_x);
}

static void c2_b_exp(SFc2_QuanInstanceStruct *chartInstance, real_T *c2_x)
{
  (void)chartInstance;
  *c2_x = muDoubleScalarExp(*c2_x);
}

static void c2_b_sign(SFc2_QuanInstanceStruct *chartInstance, real_T *c2_x)
{
  (void)chartInstance;
  *c2_x = muDoubleScalarSign(*c2_x);
}

static void init_dsm_address_info(SFc2_QuanInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc2_QuanInstanceStruct *chartInstance)
{
  chartInstance->c2_delta_e = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c2_X = (real_T *)ssGetOutputPortSignal_wrapper(chartInstance->S,
    1);
  chartInstance->c2_delta_r = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c2_delta_a = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c2_delta_T = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c2_state = (real_T (*)[12])ssGetInputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c2_Y = (real_T *)ssGetOutputPortSignal_wrapper(chartInstance->S,
    2);
  chartInstance->c2_Z = (real_T *)ssGetOutputPortSignal_wrapper(chartInstance->S,
    3);
  chartInstance->c2_L = (real_T *)ssGetOutputPortSignal_wrapper(chartInstance->S,
    4);
  chartInstance->c2_M = (real_T *)ssGetOutputPortSignal_wrapper(chartInstance->S,
    5);
  chartInstance->c2_N = (real_T *)ssGetOutputPortSignal_wrapper(chartInstance->S,
    6);
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

void sf_c2_Quan_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1323034781U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1010257619U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2363669419U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2785590418U);
}

mxArray* sf_c2_Quan_get_post_codegen_info(void);
mxArray *sf_c2_Quan_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("LG2HkWcvu4c5eK9OtPiSXC");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,5,3,dataFields);

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
      mxArray *mxSize = mxCreateDoubleMatrix(1,1,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(12);
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
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,6,3,dataFields);

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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c2_Quan_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c2_Quan_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c2_Quan_jit_fallback_info(void)
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

mxArray *sf_c2_Quan_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c2_Quan_get_post_codegen_info(void)
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

static const mxArray *sf_get_sim_state_info_c2_Quan(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x7'type','srcId','name','auxInfo'{{M[1],M[12],T\"L\",},{M[1],M[13],T\"M\",},{M[1],M[14],T\"N\",},{M[1],M[5],T\"X\",},{M[1],M[10],T\"Y\",},{M[1],M[11],T\"Z\",},{M[8],M[0],T\"is_active_c2_Quan\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 7, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_Quan_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_QuanInstanceStruct *chartInstance = (SFc2_QuanInstanceStruct *)
      sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _QuanMachineNumber_,
           2,
           1,
           1,
           0,
           11,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"delta_e");
          _SFD_SET_DATA_PROPS(1,1,1,0,"delta_r");
          _SFD_SET_DATA_PROPS(2,1,1,0,"delta_a");
          _SFD_SET_DATA_PROPS(3,1,1,0,"delta_T");
          _SFD_SET_DATA_PROPS(4,1,1,0,"state");
          _SFD_SET_DATA_PROPS(5,2,0,1,"X");
          _SFD_SET_DATA_PROPS(6,2,0,1,"Y");
          _SFD_SET_DATA_PROPS(7,2,0,1,"Z");
          _SFD_SET_DATA_PROPS(8,2,0,1,"L");
          _SFD_SET_DATA_PROPS(9,2,0,1,"M");
          _SFD_SET_DATA_PROPS(10,2,0,1,"N");
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
        _SFD_CV_INIT_EML(0,1,1,0,2,0,0,0,0,0,4,2);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,9782);
        _SFD_CV_INIT_EML_IF(0,1,0,7168,7184,7255,7412);
        _SFD_CV_INIT_EML_IF(0,1,1,7255,7276,7365,7412);

        {
          static int condStart[] = { 7171, 7179 };

          static int condEnd[] = { 7175, 7184 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,7171,7184,2,0,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 7262, 7271 };

          static int condEnd[] = { 7267, 7276 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,1,7262,7276,2,2,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,7171,7175,-1,5);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,1,7179,7184,-1,3);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,2,7262,7267,-1,5);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,3,7271,7276,-1,3);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 12U;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
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
    SFc2_QuanInstanceStruct *chartInstance = (SFc2_QuanInstanceStruct *)
      sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c2_delta_e);
        _SFD_SET_DATA_VALUE_PTR(5U, chartInstance->c2_X);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c2_delta_r);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c2_delta_a);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c2_delta_T);
        _SFD_SET_DATA_VALUE_PTR(4U, *chartInstance->c2_state);
        _SFD_SET_DATA_VALUE_PTR(6U, chartInstance->c2_Y);
        _SFD_SET_DATA_VALUE_PTR(7U, chartInstance->c2_Z);
        _SFD_SET_DATA_VALUE_PTR(8U, chartInstance->c2_L);
        _SFD_SET_DATA_VALUE_PTR(9U, chartInstance->c2_M);
        _SFD_SET_DATA_VALUE_PTR(10U, chartInstance->c2_N);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "sNFYkoeXQ61UwTrw2VoCmt";
}

static void sf_opaque_initialize_c2_Quan(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_QuanInstanceStruct*) chartInstanceVar)->S,0);
  initialize_params_c2_Quan((SFc2_QuanInstanceStruct*) chartInstanceVar);
  initialize_c2_Quan((SFc2_QuanInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_Quan(void *chartInstanceVar)
{
  enable_c2_Quan((SFc2_QuanInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_Quan(void *chartInstanceVar)
{
  disable_c2_Quan((SFc2_QuanInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_Quan(void *chartInstanceVar)
{
  sf_gateway_c2_Quan((SFc2_QuanInstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c2_Quan(SimStruct* S)
{
  return get_sim_state_c2_Quan((SFc2_QuanInstanceStruct *)
    sf_get_chart_instance_ptr(S));     /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c2_Quan(SimStruct* S, const mxArray *st)
{
  set_sim_state_c2_Quan((SFc2_QuanInstanceStruct*)sf_get_chart_instance_ptr(S),
                        st);
}

static void sf_opaque_terminate_c2_Quan(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_QuanInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_Quan_optimization_info();
    }

    finalize_c2_Quan((SFc2_QuanInstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_Quan((SFc2_QuanInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_Quan(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_Quan((SFc2_QuanInstanceStruct*)
      sf_get_chart_instance_ptr(S));
  }
}

static void mdlSetWorkWidths_c2_Quan(SimStruct *S)
{
  /* Set overwritable ports for inplace optimization */
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_Quan_optimization_info(sim_mode_is_rtw_gen(S),
      sim_mode_is_modelref_sim(S), sim_mode_is_external(S));
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,2,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_set_chart_accesses_machine_info(S, sf_get_instance_specialization(),
      infoStruct, 2);
    sf_update_buildInfo(S, sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,5);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,6);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=6; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 5; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3367836718U));
  ssSetChecksum1(S,(1026208938U));
  ssSetChecksum2(S,(2775147113U));
  ssSetChecksum3(S,(3336330398U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSetStateSemanticsClassicAndSynchronous(S, true);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_Quan(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_Quan(SimStruct *S)
{
  SFc2_QuanInstanceStruct *chartInstance;
  chartInstance = (SFc2_QuanInstanceStruct *)utMalloc(sizeof
    (SFc2_QuanInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc2_QuanInstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c2_Quan;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c2_Quan;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c2_Quan;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_Quan;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_Quan;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c2_Quan;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c2_Quan;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c2_Quan;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_Quan;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_Quan;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c2_Quan;
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
  mdl_start_c2_Quan(chartInstance);
}

void c2_Quan_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_Quan(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_Quan(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_Quan(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_Quan_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
