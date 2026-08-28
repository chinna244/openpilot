#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_2023516669171357479);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_812571722217577142);
void pose_H_mod_fun(double *state, double *out_865899901402682167);
void pose_f_fun(double *state, double dt, double *out_8462551609575224310);
void pose_F_fun(double *state, double dt, double *out_58769962423284028);
void pose_h_4(double *state, double *unused, double *out_4089771592570596149);
void pose_H_4(double *state, double *unused, double *out_332047580480215937);
void pose_h_10(double *state, double *unused, double *out_5220405385265130455);
void pose_H_10(double *state, double *unused, double *out_4499148858588669044);
void pose_h_13(double *state, double *unused, double *out_5128205467874007671);
void pose_H_13(double *state, double *unused, double *out_3544321405812548738);
void pose_h_14(double *state, double *unused, double *out_7590889396526969494);
void pose_H_14(double *state, double *unused, double *out_4295288436819700466);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}