#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6069280071856715832) {
   out_6069280071856715832[0] = delta_x[0] + nom_x[0];
   out_6069280071856715832[1] = delta_x[1] + nom_x[1];
   out_6069280071856715832[2] = delta_x[2] + nom_x[2];
   out_6069280071856715832[3] = delta_x[3] + nom_x[3];
   out_6069280071856715832[4] = delta_x[4] + nom_x[4];
   out_6069280071856715832[5] = delta_x[5] + nom_x[5];
   out_6069280071856715832[6] = delta_x[6] + nom_x[6];
   out_6069280071856715832[7] = delta_x[7] + nom_x[7];
   out_6069280071856715832[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2104573396595581804) {
   out_2104573396595581804[0] = -nom_x[0] + true_x[0];
   out_2104573396595581804[1] = -nom_x[1] + true_x[1];
   out_2104573396595581804[2] = -nom_x[2] + true_x[2];
   out_2104573396595581804[3] = -nom_x[3] + true_x[3];
   out_2104573396595581804[4] = -nom_x[4] + true_x[4];
   out_2104573396595581804[5] = -nom_x[5] + true_x[5];
   out_2104573396595581804[6] = -nom_x[6] + true_x[6];
   out_2104573396595581804[7] = -nom_x[7] + true_x[7];
   out_2104573396595581804[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5923235516676841394) {
   out_5923235516676841394[0] = 1.0;
   out_5923235516676841394[1] = 0.0;
   out_5923235516676841394[2] = 0.0;
   out_5923235516676841394[3] = 0.0;
   out_5923235516676841394[4] = 0.0;
   out_5923235516676841394[5] = 0.0;
   out_5923235516676841394[6] = 0.0;
   out_5923235516676841394[7] = 0.0;
   out_5923235516676841394[8] = 0.0;
   out_5923235516676841394[9] = 0.0;
   out_5923235516676841394[10] = 1.0;
   out_5923235516676841394[11] = 0.0;
   out_5923235516676841394[12] = 0.0;
   out_5923235516676841394[13] = 0.0;
   out_5923235516676841394[14] = 0.0;
   out_5923235516676841394[15] = 0.0;
   out_5923235516676841394[16] = 0.0;
   out_5923235516676841394[17] = 0.0;
   out_5923235516676841394[18] = 0.0;
   out_5923235516676841394[19] = 0.0;
   out_5923235516676841394[20] = 1.0;
   out_5923235516676841394[21] = 0.0;
   out_5923235516676841394[22] = 0.0;
   out_5923235516676841394[23] = 0.0;
   out_5923235516676841394[24] = 0.0;
   out_5923235516676841394[25] = 0.0;
   out_5923235516676841394[26] = 0.0;
   out_5923235516676841394[27] = 0.0;
   out_5923235516676841394[28] = 0.0;
   out_5923235516676841394[29] = 0.0;
   out_5923235516676841394[30] = 1.0;
   out_5923235516676841394[31] = 0.0;
   out_5923235516676841394[32] = 0.0;
   out_5923235516676841394[33] = 0.0;
   out_5923235516676841394[34] = 0.0;
   out_5923235516676841394[35] = 0.0;
   out_5923235516676841394[36] = 0.0;
   out_5923235516676841394[37] = 0.0;
   out_5923235516676841394[38] = 0.0;
   out_5923235516676841394[39] = 0.0;
   out_5923235516676841394[40] = 1.0;
   out_5923235516676841394[41] = 0.0;
   out_5923235516676841394[42] = 0.0;
   out_5923235516676841394[43] = 0.0;
   out_5923235516676841394[44] = 0.0;
   out_5923235516676841394[45] = 0.0;
   out_5923235516676841394[46] = 0.0;
   out_5923235516676841394[47] = 0.0;
   out_5923235516676841394[48] = 0.0;
   out_5923235516676841394[49] = 0.0;
   out_5923235516676841394[50] = 1.0;
   out_5923235516676841394[51] = 0.0;
   out_5923235516676841394[52] = 0.0;
   out_5923235516676841394[53] = 0.0;
   out_5923235516676841394[54] = 0.0;
   out_5923235516676841394[55] = 0.0;
   out_5923235516676841394[56] = 0.0;
   out_5923235516676841394[57] = 0.0;
   out_5923235516676841394[58] = 0.0;
   out_5923235516676841394[59] = 0.0;
   out_5923235516676841394[60] = 1.0;
   out_5923235516676841394[61] = 0.0;
   out_5923235516676841394[62] = 0.0;
   out_5923235516676841394[63] = 0.0;
   out_5923235516676841394[64] = 0.0;
   out_5923235516676841394[65] = 0.0;
   out_5923235516676841394[66] = 0.0;
   out_5923235516676841394[67] = 0.0;
   out_5923235516676841394[68] = 0.0;
   out_5923235516676841394[69] = 0.0;
   out_5923235516676841394[70] = 1.0;
   out_5923235516676841394[71] = 0.0;
   out_5923235516676841394[72] = 0.0;
   out_5923235516676841394[73] = 0.0;
   out_5923235516676841394[74] = 0.0;
   out_5923235516676841394[75] = 0.0;
   out_5923235516676841394[76] = 0.0;
   out_5923235516676841394[77] = 0.0;
   out_5923235516676841394[78] = 0.0;
   out_5923235516676841394[79] = 0.0;
   out_5923235516676841394[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5110482243731547769) {
   out_5110482243731547769[0] = state[0];
   out_5110482243731547769[1] = state[1];
   out_5110482243731547769[2] = state[2];
   out_5110482243731547769[3] = state[3];
   out_5110482243731547769[4] = state[4];
   out_5110482243731547769[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5110482243731547769[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5110482243731547769[7] = state[7];
   out_5110482243731547769[8] = state[8];
}
void F_fun(double *state, double dt, double *out_383262304490403471) {
   out_383262304490403471[0] = 1;
   out_383262304490403471[1] = 0;
   out_383262304490403471[2] = 0;
   out_383262304490403471[3] = 0;
   out_383262304490403471[4] = 0;
   out_383262304490403471[5] = 0;
   out_383262304490403471[6] = 0;
   out_383262304490403471[7] = 0;
   out_383262304490403471[8] = 0;
   out_383262304490403471[9] = 0;
   out_383262304490403471[10] = 1;
   out_383262304490403471[11] = 0;
   out_383262304490403471[12] = 0;
   out_383262304490403471[13] = 0;
   out_383262304490403471[14] = 0;
   out_383262304490403471[15] = 0;
   out_383262304490403471[16] = 0;
   out_383262304490403471[17] = 0;
   out_383262304490403471[18] = 0;
   out_383262304490403471[19] = 0;
   out_383262304490403471[20] = 1;
   out_383262304490403471[21] = 0;
   out_383262304490403471[22] = 0;
   out_383262304490403471[23] = 0;
   out_383262304490403471[24] = 0;
   out_383262304490403471[25] = 0;
   out_383262304490403471[26] = 0;
   out_383262304490403471[27] = 0;
   out_383262304490403471[28] = 0;
   out_383262304490403471[29] = 0;
   out_383262304490403471[30] = 1;
   out_383262304490403471[31] = 0;
   out_383262304490403471[32] = 0;
   out_383262304490403471[33] = 0;
   out_383262304490403471[34] = 0;
   out_383262304490403471[35] = 0;
   out_383262304490403471[36] = 0;
   out_383262304490403471[37] = 0;
   out_383262304490403471[38] = 0;
   out_383262304490403471[39] = 0;
   out_383262304490403471[40] = 1;
   out_383262304490403471[41] = 0;
   out_383262304490403471[42] = 0;
   out_383262304490403471[43] = 0;
   out_383262304490403471[44] = 0;
   out_383262304490403471[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_383262304490403471[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_383262304490403471[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_383262304490403471[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_383262304490403471[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_383262304490403471[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_383262304490403471[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_383262304490403471[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_383262304490403471[53] = -9.8100000000000005*dt;
   out_383262304490403471[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_383262304490403471[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_383262304490403471[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_383262304490403471[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_383262304490403471[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_383262304490403471[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_383262304490403471[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_383262304490403471[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_383262304490403471[62] = 0;
   out_383262304490403471[63] = 0;
   out_383262304490403471[64] = 0;
   out_383262304490403471[65] = 0;
   out_383262304490403471[66] = 0;
   out_383262304490403471[67] = 0;
   out_383262304490403471[68] = 0;
   out_383262304490403471[69] = 0;
   out_383262304490403471[70] = 1;
   out_383262304490403471[71] = 0;
   out_383262304490403471[72] = 0;
   out_383262304490403471[73] = 0;
   out_383262304490403471[74] = 0;
   out_383262304490403471[75] = 0;
   out_383262304490403471[76] = 0;
   out_383262304490403471[77] = 0;
   out_383262304490403471[78] = 0;
   out_383262304490403471[79] = 0;
   out_383262304490403471[80] = 1;
}
void h_25(double *state, double *unused, double *out_4852584950656477855) {
   out_4852584950656477855[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2732061361125631472) {
   out_2732061361125631472[0] = 0;
   out_2732061361125631472[1] = 0;
   out_2732061361125631472[2] = 0;
   out_2732061361125631472[3] = 0;
   out_2732061361125631472[4] = 0;
   out_2732061361125631472[5] = 0;
   out_2732061361125631472[6] = 1;
   out_2732061361125631472[7] = 0;
   out_2732061361125631472[8] = 0;
}
void h_24(double *state, double *unused, double *out_9173501940634478645) {
   out_9173501940634478645[0] = state[4];
   out_9173501940634478645[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1044244925256527281) {
   out_1044244925256527281[0] = 0;
   out_1044244925256527281[1] = 0;
   out_1044244925256527281[2] = 0;
   out_1044244925256527281[3] = 0;
   out_1044244925256527281[4] = 1;
   out_1044244925256527281[5] = 0;
   out_1044244925256527281[6] = 0;
   out_1044244925256527281[7] = 0;
   out_1044244925256527281[8] = 0;
   out_1044244925256527281[9] = 0;
   out_1044244925256527281[10] = 0;
   out_1044244925256527281[11] = 0;
   out_1044244925256527281[12] = 0;
   out_1044244925256527281[13] = 0;
   out_1044244925256527281[14] = 1;
   out_1044244925256527281[15] = 0;
   out_1044244925256527281[16] = 0;
   out_1044244925256527281[17] = 0;
}
void h_30(double *state, double *unused, double *out_4695398282305348710) {
   out_4695398282305348710[0] = state[4];
}
void H_30(double *state, double *unused, double *out_213728402618382845) {
   out_213728402618382845[0] = 0;
   out_213728402618382845[1] = 0;
   out_213728402618382845[2] = 0;
   out_213728402618382845[3] = 0;
   out_213728402618382845[4] = 1;
   out_213728402618382845[5] = 0;
   out_213728402618382845[6] = 0;
   out_213728402618382845[7] = 0;
   out_213728402618382845[8] = 0;
}
void h_26(double *state, double *unused, double *out_974740422792210646) {
   out_974740422792210646[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6473564679999687696) {
   out_6473564679999687696[0] = 0;
   out_6473564679999687696[1] = 0;
   out_6473564679999687696[2] = 0;
   out_6473564679999687696[3] = 0;
   out_6473564679999687696[4] = 0;
   out_6473564679999687696[5] = 0;
   out_6473564679999687696[6] = 0;
   out_6473564679999687696[7] = 1;
   out_6473564679999687696[8] = 0;
}
void h_27(double *state, double *unused, double *out_2582558483772222958) {
   out_2582558483772222958[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2388491714418807756) {
   out_2388491714418807756[0] = 0;
   out_2388491714418807756[1] = 0;
   out_2388491714418807756[2] = 0;
   out_2388491714418807756[3] = 1;
   out_2388491714418807756[4] = 0;
   out_2388491714418807756[5] = 0;
   out_2388491714418807756[6] = 0;
   out_2388491714418807756[7] = 0;
   out_2388491714418807756[8] = 0;
}
void h_29(double *state, double *unused, double *out_9093350363586977722) {
   out_9093350363586977722[0] = state[1];
}
void H_29(double *state, double *unused, double *out_296502941696009339) {
   out_296502941696009339[0] = 0;
   out_296502941696009339[1] = 1;
   out_296502941696009339[2] = 0;
   out_296502941696009339[3] = 0;
   out_296502941696009339[4] = 0;
   out_296502941696009339[5] = 0;
   out_296502941696009339[6] = 0;
   out_296502941696009339[7] = 0;
   out_296502941696009339[8] = 0;
}
void h_28(double *state, double *unused, double *out_2419114783888622687) {
   out_2419114783888622687[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4785896075373521235) {
   out_4785896075373521235[0] = 1;
   out_4785896075373521235[1] = 0;
   out_4785896075373521235[2] = 0;
   out_4785896075373521235[3] = 0;
   out_4785896075373521235[4] = 0;
   out_4785896075373521235[5] = 0;
   out_4785896075373521235[6] = 0;
   out_4785896075373521235[7] = 0;
   out_4785896075373521235[8] = 0;
}
void h_31(double *state, double *unused, double *out_179033505387603838) {
   out_179033505387603838[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7099772782233039172) {
   out_7099772782233039172[0] = 0;
   out_7099772782233039172[1] = 0;
   out_7099772782233039172[2] = 0;
   out_7099772782233039172[3] = 0;
   out_7099772782233039172[4] = 0;
   out_7099772782233039172[5] = 0;
   out_7099772782233039172[6] = 0;
   out_7099772782233039172[7] = 0;
   out_7099772782233039172[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_6069280071856715832) {
  err_fun(nom_x, delta_x, out_6069280071856715832);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2104573396595581804) {
  inv_err_fun(nom_x, true_x, out_2104573396595581804);
}
void car_H_mod_fun(double *state, double *out_5923235516676841394) {
  H_mod_fun(state, out_5923235516676841394);
}
void car_f_fun(double *state, double dt, double *out_5110482243731547769) {
  f_fun(state,  dt, out_5110482243731547769);
}
void car_F_fun(double *state, double dt, double *out_383262304490403471) {
  F_fun(state,  dt, out_383262304490403471);
}
void car_h_25(double *state, double *unused, double *out_4852584950656477855) {
  h_25(state, unused, out_4852584950656477855);
}
void car_H_25(double *state, double *unused, double *out_2732061361125631472) {
  H_25(state, unused, out_2732061361125631472);
}
void car_h_24(double *state, double *unused, double *out_9173501940634478645) {
  h_24(state, unused, out_9173501940634478645);
}
void car_H_24(double *state, double *unused, double *out_1044244925256527281) {
  H_24(state, unused, out_1044244925256527281);
}
void car_h_30(double *state, double *unused, double *out_4695398282305348710) {
  h_30(state, unused, out_4695398282305348710);
}
void car_H_30(double *state, double *unused, double *out_213728402618382845) {
  H_30(state, unused, out_213728402618382845);
}
void car_h_26(double *state, double *unused, double *out_974740422792210646) {
  h_26(state, unused, out_974740422792210646);
}
void car_H_26(double *state, double *unused, double *out_6473564679999687696) {
  H_26(state, unused, out_6473564679999687696);
}
void car_h_27(double *state, double *unused, double *out_2582558483772222958) {
  h_27(state, unused, out_2582558483772222958);
}
void car_H_27(double *state, double *unused, double *out_2388491714418807756) {
  H_27(state, unused, out_2388491714418807756);
}
void car_h_29(double *state, double *unused, double *out_9093350363586977722) {
  h_29(state, unused, out_9093350363586977722);
}
void car_H_29(double *state, double *unused, double *out_296502941696009339) {
  H_29(state, unused, out_296502941696009339);
}
void car_h_28(double *state, double *unused, double *out_2419114783888622687) {
  h_28(state, unused, out_2419114783888622687);
}
void car_H_28(double *state, double *unused, double *out_4785896075373521235) {
  H_28(state, unused, out_4785896075373521235);
}
void car_h_31(double *state, double *unused, double *out_179033505387603838) {
  h_31(state, unused, out_179033505387603838);
}
void car_H_31(double *state, double *unused, double *out_7099772782233039172) {
  H_31(state, unused, out_7099772782233039172);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
