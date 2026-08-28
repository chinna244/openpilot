#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_6069280071856715832);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2104573396595581804);
void car_H_mod_fun(double *state, double *out_5923235516676841394);
void car_f_fun(double *state, double dt, double *out_5110482243731547769);
void car_F_fun(double *state, double dt, double *out_383262304490403471);
void car_h_25(double *state, double *unused, double *out_4852584950656477855);
void car_H_25(double *state, double *unused, double *out_2732061361125631472);
void car_h_24(double *state, double *unused, double *out_9173501940634478645);
void car_H_24(double *state, double *unused, double *out_1044244925256527281);
void car_h_30(double *state, double *unused, double *out_4695398282305348710);
void car_H_30(double *state, double *unused, double *out_213728402618382845);
void car_h_26(double *state, double *unused, double *out_974740422792210646);
void car_H_26(double *state, double *unused, double *out_6473564679999687696);
void car_h_27(double *state, double *unused, double *out_2582558483772222958);
void car_H_27(double *state, double *unused, double *out_2388491714418807756);
void car_h_29(double *state, double *unused, double *out_9093350363586977722);
void car_H_29(double *state, double *unused, double *out_296502941696009339);
void car_h_28(double *state, double *unused, double *out_2419114783888622687);
void car_H_28(double *state, double *unused, double *out_4785896075373521235);
void car_h_31(double *state, double *unused, double *out_179033505387603838);
void car_H_31(double *state, double *unused, double *out_7099772782233039172);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}