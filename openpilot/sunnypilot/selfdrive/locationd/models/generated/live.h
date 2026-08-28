#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_3818317328549354767);
void live_err_fun(double *nom_x, double *delta_x, double *out_1602837391426954502);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_9118853704503754112);
void live_H_mod_fun(double *state, double *out_6279270162200650564);
void live_f_fun(double *state, double dt, double *out_7050107027821943742);
void live_F_fun(double *state, double dt, double *out_1706884619704818999);
void live_h_4(double *state, double *unused, double *out_8288147402007187768);
void live_H_4(double *state, double *unused, double *out_471533054928683194);
void live_h_9(double *state, double *unused, double *out_8454971196814801449);
void live_H_9(double *state, double *unused, double *out_712722701558273839);
void live_h_10(double *state, double *unused, double *out_1589043339799016123);
void live_H_10(double *state, double *unused, double *out_3087627905751493657);
void live_h_12(double *state, double *unused, double *out_2923793128584207083);
void live_H_12(double *state, double *unused, double *out_5490989462960644989);
void live_h_35(double *state, double *unused, double *out_1193682003941975658);
void live_H_35(double *state, double *unused, double *out_3838195112301290570);
void live_h_32(double *state, double *unused, double *out_5182237245243880553);
void live_H_32(double *state, double *unused, double *out_951713689452836074);
void live_h_13(double *state, double *unused, double *out_7900194348497152437);
void live_H_13(double *state, double *unused, double *out_3850897084892108416);
void live_h_14(double *state, double *unused, double *out_8454971196814801449);
void live_H_14(double *state, double *unused, double *out_712722701558273839);
void live_h_33(double *state, double *unused, double *out_2662818595039193343);
void live_H_33(double *state, double *unused, double *out_6988752116940148174);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}