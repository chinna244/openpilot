#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2023516669171357479) {
   out_2023516669171357479[0] = delta_x[0] + nom_x[0];
   out_2023516669171357479[1] = delta_x[1] + nom_x[1];
   out_2023516669171357479[2] = delta_x[2] + nom_x[2];
   out_2023516669171357479[3] = delta_x[3] + nom_x[3];
   out_2023516669171357479[4] = delta_x[4] + nom_x[4];
   out_2023516669171357479[5] = delta_x[5] + nom_x[5];
   out_2023516669171357479[6] = delta_x[6] + nom_x[6];
   out_2023516669171357479[7] = delta_x[7] + nom_x[7];
   out_2023516669171357479[8] = delta_x[8] + nom_x[8];
   out_2023516669171357479[9] = delta_x[9] + nom_x[9];
   out_2023516669171357479[10] = delta_x[10] + nom_x[10];
   out_2023516669171357479[11] = delta_x[11] + nom_x[11];
   out_2023516669171357479[12] = delta_x[12] + nom_x[12];
   out_2023516669171357479[13] = delta_x[13] + nom_x[13];
   out_2023516669171357479[14] = delta_x[14] + nom_x[14];
   out_2023516669171357479[15] = delta_x[15] + nom_x[15];
   out_2023516669171357479[16] = delta_x[16] + nom_x[16];
   out_2023516669171357479[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_812571722217577142) {
   out_812571722217577142[0] = -nom_x[0] + true_x[0];
   out_812571722217577142[1] = -nom_x[1] + true_x[1];
   out_812571722217577142[2] = -nom_x[2] + true_x[2];
   out_812571722217577142[3] = -nom_x[3] + true_x[3];
   out_812571722217577142[4] = -nom_x[4] + true_x[4];
   out_812571722217577142[5] = -nom_x[5] + true_x[5];
   out_812571722217577142[6] = -nom_x[6] + true_x[6];
   out_812571722217577142[7] = -nom_x[7] + true_x[7];
   out_812571722217577142[8] = -nom_x[8] + true_x[8];
   out_812571722217577142[9] = -nom_x[9] + true_x[9];
   out_812571722217577142[10] = -nom_x[10] + true_x[10];
   out_812571722217577142[11] = -nom_x[11] + true_x[11];
   out_812571722217577142[12] = -nom_x[12] + true_x[12];
   out_812571722217577142[13] = -nom_x[13] + true_x[13];
   out_812571722217577142[14] = -nom_x[14] + true_x[14];
   out_812571722217577142[15] = -nom_x[15] + true_x[15];
   out_812571722217577142[16] = -nom_x[16] + true_x[16];
   out_812571722217577142[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_865899901402682167) {
   out_865899901402682167[0] = 1.0;
   out_865899901402682167[1] = 0.0;
   out_865899901402682167[2] = 0.0;
   out_865899901402682167[3] = 0.0;
   out_865899901402682167[4] = 0.0;
   out_865899901402682167[5] = 0.0;
   out_865899901402682167[6] = 0.0;
   out_865899901402682167[7] = 0.0;
   out_865899901402682167[8] = 0.0;
   out_865899901402682167[9] = 0.0;
   out_865899901402682167[10] = 0.0;
   out_865899901402682167[11] = 0.0;
   out_865899901402682167[12] = 0.0;
   out_865899901402682167[13] = 0.0;
   out_865899901402682167[14] = 0.0;
   out_865899901402682167[15] = 0.0;
   out_865899901402682167[16] = 0.0;
   out_865899901402682167[17] = 0.0;
   out_865899901402682167[18] = 0.0;
   out_865899901402682167[19] = 1.0;
   out_865899901402682167[20] = 0.0;
   out_865899901402682167[21] = 0.0;
   out_865899901402682167[22] = 0.0;
   out_865899901402682167[23] = 0.0;
   out_865899901402682167[24] = 0.0;
   out_865899901402682167[25] = 0.0;
   out_865899901402682167[26] = 0.0;
   out_865899901402682167[27] = 0.0;
   out_865899901402682167[28] = 0.0;
   out_865899901402682167[29] = 0.0;
   out_865899901402682167[30] = 0.0;
   out_865899901402682167[31] = 0.0;
   out_865899901402682167[32] = 0.0;
   out_865899901402682167[33] = 0.0;
   out_865899901402682167[34] = 0.0;
   out_865899901402682167[35] = 0.0;
   out_865899901402682167[36] = 0.0;
   out_865899901402682167[37] = 0.0;
   out_865899901402682167[38] = 1.0;
   out_865899901402682167[39] = 0.0;
   out_865899901402682167[40] = 0.0;
   out_865899901402682167[41] = 0.0;
   out_865899901402682167[42] = 0.0;
   out_865899901402682167[43] = 0.0;
   out_865899901402682167[44] = 0.0;
   out_865899901402682167[45] = 0.0;
   out_865899901402682167[46] = 0.0;
   out_865899901402682167[47] = 0.0;
   out_865899901402682167[48] = 0.0;
   out_865899901402682167[49] = 0.0;
   out_865899901402682167[50] = 0.0;
   out_865899901402682167[51] = 0.0;
   out_865899901402682167[52] = 0.0;
   out_865899901402682167[53] = 0.0;
   out_865899901402682167[54] = 0.0;
   out_865899901402682167[55] = 0.0;
   out_865899901402682167[56] = 0.0;
   out_865899901402682167[57] = 1.0;
   out_865899901402682167[58] = 0.0;
   out_865899901402682167[59] = 0.0;
   out_865899901402682167[60] = 0.0;
   out_865899901402682167[61] = 0.0;
   out_865899901402682167[62] = 0.0;
   out_865899901402682167[63] = 0.0;
   out_865899901402682167[64] = 0.0;
   out_865899901402682167[65] = 0.0;
   out_865899901402682167[66] = 0.0;
   out_865899901402682167[67] = 0.0;
   out_865899901402682167[68] = 0.0;
   out_865899901402682167[69] = 0.0;
   out_865899901402682167[70] = 0.0;
   out_865899901402682167[71] = 0.0;
   out_865899901402682167[72] = 0.0;
   out_865899901402682167[73] = 0.0;
   out_865899901402682167[74] = 0.0;
   out_865899901402682167[75] = 0.0;
   out_865899901402682167[76] = 1.0;
   out_865899901402682167[77] = 0.0;
   out_865899901402682167[78] = 0.0;
   out_865899901402682167[79] = 0.0;
   out_865899901402682167[80] = 0.0;
   out_865899901402682167[81] = 0.0;
   out_865899901402682167[82] = 0.0;
   out_865899901402682167[83] = 0.0;
   out_865899901402682167[84] = 0.0;
   out_865899901402682167[85] = 0.0;
   out_865899901402682167[86] = 0.0;
   out_865899901402682167[87] = 0.0;
   out_865899901402682167[88] = 0.0;
   out_865899901402682167[89] = 0.0;
   out_865899901402682167[90] = 0.0;
   out_865899901402682167[91] = 0.0;
   out_865899901402682167[92] = 0.0;
   out_865899901402682167[93] = 0.0;
   out_865899901402682167[94] = 0.0;
   out_865899901402682167[95] = 1.0;
   out_865899901402682167[96] = 0.0;
   out_865899901402682167[97] = 0.0;
   out_865899901402682167[98] = 0.0;
   out_865899901402682167[99] = 0.0;
   out_865899901402682167[100] = 0.0;
   out_865899901402682167[101] = 0.0;
   out_865899901402682167[102] = 0.0;
   out_865899901402682167[103] = 0.0;
   out_865899901402682167[104] = 0.0;
   out_865899901402682167[105] = 0.0;
   out_865899901402682167[106] = 0.0;
   out_865899901402682167[107] = 0.0;
   out_865899901402682167[108] = 0.0;
   out_865899901402682167[109] = 0.0;
   out_865899901402682167[110] = 0.0;
   out_865899901402682167[111] = 0.0;
   out_865899901402682167[112] = 0.0;
   out_865899901402682167[113] = 0.0;
   out_865899901402682167[114] = 1.0;
   out_865899901402682167[115] = 0.0;
   out_865899901402682167[116] = 0.0;
   out_865899901402682167[117] = 0.0;
   out_865899901402682167[118] = 0.0;
   out_865899901402682167[119] = 0.0;
   out_865899901402682167[120] = 0.0;
   out_865899901402682167[121] = 0.0;
   out_865899901402682167[122] = 0.0;
   out_865899901402682167[123] = 0.0;
   out_865899901402682167[124] = 0.0;
   out_865899901402682167[125] = 0.0;
   out_865899901402682167[126] = 0.0;
   out_865899901402682167[127] = 0.0;
   out_865899901402682167[128] = 0.0;
   out_865899901402682167[129] = 0.0;
   out_865899901402682167[130] = 0.0;
   out_865899901402682167[131] = 0.0;
   out_865899901402682167[132] = 0.0;
   out_865899901402682167[133] = 1.0;
   out_865899901402682167[134] = 0.0;
   out_865899901402682167[135] = 0.0;
   out_865899901402682167[136] = 0.0;
   out_865899901402682167[137] = 0.0;
   out_865899901402682167[138] = 0.0;
   out_865899901402682167[139] = 0.0;
   out_865899901402682167[140] = 0.0;
   out_865899901402682167[141] = 0.0;
   out_865899901402682167[142] = 0.0;
   out_865899901402682167[143] = 0.0;
   out_865899901402682167[144] = 0.0;
   out_865899901402682167[145] = 0.0;
   out_865899901402682167[146] = 0.0;
   out_865899901402682167[147] = 0.0;
   out_865899901402682167[148] = 0.0;
   out_865899901402682167[149] = 0.0;
   out_865899901402682167[150] = 0.0;
   out_865899901402682167[151] = 0.0;
   out_865899901402682167[152] = 1.0;
   out_865899901402682167[153] = 0.0;
   out_865899901402682167[154] = 0.0;
   out_865899901402682167[155] = 0.0;
   out_865899901402682167[156] = 0.0;
   out_865899901402682167[157] = 0.0;
   out_865899901402682167[158] = 0.0;
   out_865899901402682167[159] = 0.0;
   out_865899901402682167[160] = 0.0;
   out_865899901402682167[161] = 0.0;
   out_865899901402682167[162] = 0.0;
   out_865899901402682167[163] = 0.0;
   out_865899901402682167[164] = 0.0;
   out_865899901402682167[165] = 0.0;
   out_865899901402682167[166] = 0.0;
   out_865899901402682167[167] = 0.0;
   out_865899901402682167[168] = 0.0;
   out_865899901402682167[169] = 0.0;
   out_865899901402682167[170] = 0.0;
   out_865899901402682167[171] = 1.0;
   out_865899901402682167[172] = 0.0;
   out_865899901402682167[173] = 0.0;
   out_865899901402682167[174] = 0.0;
   out_865899901402682167[175] = 0.0;
   out_865899901402682167[176] = 0.0;
   out_865899901402682167[177] = 0.0;
   out_865899901402682167[178] = 0.0;
   out_865899901402682167[179] = 0.0;
   out_865899901402682167[180] = 0.0;
   out_865899901402682167[181] = 0.0;
   out_865899901402682167[182] = 0.0;
   out_865899901402682167[183] = 0.0;
   out_865899901402682167[184] = 0.0;
   out_865899901402682167[185] = 0.0;
   out_865899901402682167[186] = 0.0;
   out_865899901402682167[187] = 0.0;
   out_865899901402682167[188] = 0.0;
   out_865899901402682167[189] = 0.0;
   out_865899901402682167[190] = 1.0;
   out_865899901402682167[191] = 0.0;
   out_865899901402682167[192] = 0.0;
   out_865899901402682167[193] = 0.0;
   out_865899901402682167[194] = 0.0;
   out_865899901402682167[195] = 0.0;
   out_865899901402682167[196] = 0.0;
   out_865899901402682167[197] = 0.0;
   out_865899901402682167[198] = 0.0;
   out_865899901402682167[199] = 0.0;
   out_865899901402682167[200] = 0.0;
   out_865899901402682167[201] = 0.0;
   out_865899901402682167[202] = 0.0;
   out_865899901402682167[203] = 0.0;
   out_865899901402682167[204] = 0.0;
   out_865899901402682167[205] = 0.0;
   out_865899901402682167[206] = 0.0;
   out_865899901402682167[207] = 0.0;
   out_865899901402682167[208] = 0.0;
   out_865899901402682167[209] = 1.0;
   out_865899901402682167[210] = 0.0;
   out_865899901402682167[211] = 0.0;
   out_865899901402682167[212] = 0.0;
   out_865899901402682167[213] = 0.0;
   out_865899901402682167[214] = 0.0;
   out_865899901402682167[215] = 0.0;
   out_865899901402682167[216] = 0.0;
   out_865899901402682167[217] = 0.0;
   out_865899901402682167[218] = 0.0;
   out_865899901402682167[219] = 0.0;
   out_865899901402682167[220] = 0.0;
   out_865899901402682167[221] = 0.0;
   out_865899901402682167[222] = 0.0;
   out_865899901402682167[223] = 0.0;
   out_865899901402682167[224] = 0.0;
   out_865899901402682167[225] = 0.0;
   out_865899901402682167[226] = 0.0;
   out_865899901402682167[227] = 0.0;
   out_865899901402682167[228] = 1.0;
   out_865899901402682167[229] = 0.0;
   out_865899901402682167[230] = 0.0;
   out_865899901402682167[231] = 0.0;
   out_865899901402682167[232] = 0.0;
   out_865899901402682167[233] = 0.0;
   out_865899901402682167[234] = 0.0;
   out_865899901402682167[235] = 0.0;
   out_865899901402682167[236] = 0.0;
   out_865899901402682167[237] = 0.0;
   out_865899901402682167[238] = 0.0;
   out_865899901402682167[239] = 0.0;
   out_865899901402682167[240] = 0.0;
   out_865899901402682167[241] = 0.0;
   out_865899901402682167[242] = 0.0;
   out_865899901402682167[243] = 0.0;
   out_865899901402682167[244] = 0.0;
   out_865899901402682167[245] = 0.0;
   out_865899901402682167[246] = 0.0;
   out_865899901402682167[247] = 1.0;
   out_865899901402682167[248] = 0.0;
   out_865899901402682167[249] = 0.0;
   out_865899901402682167[250] = 0.0;
   out_865899901402682167[251] = 0.0;
   out_865899901402682167[252] = 0.0;
   out_865899901402682167[253] = 0.0;
   out_865899901402682167[254] = 0.0;
   out_865899901402682167[255] = 0.0;
   out_865899901402682167[256] = 0.0;
   out_865899901402682167[257] = 0.0;
   out_865899901402682167[258] = 0.0;
   out_865899901402682167[259] = 0.0;
   out_865899901402682167[260] = 0.0;
   out_865899901402682167[261] = 0.0;
   out_865899901402682167[262] = 0.0;
   out_865899901402682167[263] = 0.0;
   out_865899901402682167[264] = 0.0;
   out_865899901402682167[265] = 0.0;
   out_865899901402682167[266] = 1.0;
   out_865899901402682167[267] = 0.0;
   out_865899901402682167[268] = 0.0;
   out_865899901402682167[269] = 0.0;
   out_865899901402682167[270] = 0.0;
   out_865899901402682167[271] = 0.0;
   out_865899901402682167[272] = 0.0;
   out_865899901402682167[273] = 0.0;
   out_865899901402682167[274] = 0.0;
   out_865899901402682167[275] = 0.0;
   out_865899901402682167[276] = 0.0;
   out_865899901402682167[277] = 0.0;
   out_865899901402682167[278] = 0.0;
   out_865899901402682167[279] = 0.0;
   out_865899901402682167[280] = 0.0;
   out_865899901402682167[281] = 0.0;
   out_865899901402682167[282] = 0.0;
   out_865899901402682167[283] = 0.0;
   out_865899901402682167[284] = 0.0;
   out_865899901402682167[285] = 1.0;
   out_865899901402682167[286] = 0.0;
   out_865899901402682167[287] = 0.0;
   out_865899901402682167[288] = 0.0;
   out_865899901402682167[289] = 0.0;
   out_865899901402682167[290] = 0.0;
   out_865899901402682167[291] = 0.0;
   out_865899901402682167[292] = 0.0;
   out_865899901402682167[293] = 0.0;
   out_865899901402682167[294] = 0.0;
   out_865899901402682167[295] = 0.0;
   out_865899901402682167[296] = 0.0;
   out_865899901402682167[297] = 0.0;
   out_865899901402682167[298] = 0.0;
   out_865899901402682167[299] = 0.0;
   out_865899901402682167[300] = 0.0;
   out_865899901402682167[301] = 0.0;
   out_865899901402682167[302] = 0.0;
   out_865899901402682167[303] = 0.0;
   out_865899901402682167[304] = 1.0;
   out_865899901402682167[305] = 0.0;
   out_865899901402682167[306] = 0.0;
   out_865899901402682167[307] = 0.0;
   out_865899901402682167[308] = 0.0;
   out_865899901402682167[309] = 0.0;
   out_865899901402682167[310] = 0.0;
   out_865899901402682167[311] = 0.0;
   out_865899901402682167[312] = 0.0;
   out_865899901402682167[313] = 0.0;
   out_865899901402682167[314] = 0.0;
   out_865899901402682167[315] = 0.0;
   out_865899901402682167[316] = 0.0;
   out_865899901402682167[317] = 0.0;
   out_865899901402682167[318] = 0.0;
   out_865899901402682167[319] = 0.0;
   out_865899901402682167[320] = 0.0;
   out_865899901402682167[321] = 0.0;
   out_865899901402682167[322] = 0.0;
   out_865899901402682167[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_8462551609575224310) {
   out_8462551609575224310[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_8462551609575224310[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_8462551609575224310[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_8462551609575224310[3] = dt*state[12] + state[3];
   out_8462551609575224310[4] = dt*state[13] + state[4];
   out_8462551609575224310[5] = dt*state[14] + state[5];
   out_8462551609575224310[6] = state[6];
   out_8462551609575224310[7] = state[7];
   out_8462551609575224310[8] = state[8];
   out_8462551609575224310[9] = state[9];
   out_8462551609575224310[10] = state[10];
   out_8462551609575224310[11] = state[11];
   out_8462551609575224310[12] = state[12];
   out_8462551609575224310[13] = state[13];
   out_8462551609575224310[14] = state[14];
   out_8462551609575224310[15] = state[15];
   out_8462551609575224310[16] = state[16];
   out_8462551609575224310[17] = state[17];
}
void F_fun(double *state, double dt, double *out_58769962423284028) {
   out_58769962423284028[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_58769962423284028[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_58769962423284028[2] = 0;
   out_58769962423284028[3] = 0;
   out_58769962423284028[4] = 0;
   out_58769962423284028[5] = 0;
   out_58769962423284028[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_58769962423284028[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_58769962423284028[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_58769962423284028[9] = 0;
   out_58769962423284028[10] = 0;
   out_58769962423284028[11] = 0;
   out_58769962423284028[12] = 0;
   out_58769962423284028[13] = 0;
   out_58769962423284028[14] = 0;
   out_58769962423284028[15] = 0;
   out_58769962423284028[16] = 0;
   out_58769962423284028[17] = 0;
   out_58769962423284028[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_58769962423284028[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_58769962423284028[20] = 0;
   out_58769962423284028[21] = 0;
   out_58769962423284028[22] = 0;
   out_58769962423284028[23] = 0;
   out_58769962423284028[24] = 0;
   out_58769962423284028[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_58769962423284028[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_58769962423284028[27] = 0;
   out_58769962423284028[28] = 0;
   out_58769962423284028[29] = 0;
   out_58769962423284028[30] = 0;
   out_58769962423284028[31] = 0;
   out_58769962423284028[32] = 0;
   out_58769962423284028[33] = 0;
   out_58769962423284028[34] = 0;
   out_58769962423284028[35] = 0;
   out_58769962423284028[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_58769962423284028[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_58769962423284028[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_58769962423284028[39] = 0;
   out_58769962423284028[40] = 0;
   out_58769962423284028[41] = 0;
   out_58769962423284028[42] = 0;
   out_58769962423284028[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_58769962423284028[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_58769962423284028[45] = 0;
   out_58769962423284028[46] = 0;
   out_58769962423284028[47] = 0;
   out_58769962423284028[48] = 0;
   out_58769962423284028[49] = 0;
   out_58769962423284028[50] = 0;
   out_58769962423284028[51] = 0;
   out_58769962423284028[52] = 0;
   out_58769962423284028[53] = 0;
   out_58769962423284028[54] = 0;
   out_58769962423284028[55] = 0;
   out_58769962423284028[56] = 0;
   out_58769962423284028[57] = 1;
   out_58769962423284028[58] = 0;
   out_58769962423284028[59] = 0;
   out_58769962423284028[60] = 0;
   out_58769962423284028[61] = 0;
   out_58769962423284028[62] = 0;
   out_58769962423284028[63] = 0;
   out_58769962423284028[64] = 0;
   out_58769962423284028[65] = 0;
   out_58769962423284028[66] = dt;
   out_58769962423284028[67] = 0;
   out_58769962423284028[68] = 0;
   out_58769962423284028[69] = 0;
   out_58769962423284028[70] = 0;
   out_58769962423284028[71] = 0;
   out_58769962423284028[72] = 0;
   out_58769962423284028[73] = 0;
   out_58769962423284028[74] = 0;
   out_58769962423284028[75] = 0;
   out_58769962423284028[76] = 1;
   out_58769962423284028[77] = 0;
   out_58769962423284028[78] = 0;
   out_58769962423284028[79] = 0;
   out_58769962423284028[80] = 0;
   out_58769962423284028[81] = 0;
   out_58769962423284028[82] = 0;
   out_58769962423284028[83] = 0;
   out_58769962423284028[84] = 0;
   out_58769962423284028[85] = dt;
   out_58769962423284028[86] = 0;
   out_58769962423284028[87] = 0;
   out_58769962423284028[88] = 0;
   out_58769962423284028[89] = 0;
   out_58769962423284028[90] = 0;
   out_58769962423284028[91] = 0;
   out_58769962423284028[92] = 0;
   out_58769962423284028[93] = 0;
   out_58769962423284028[94] = 0;
   out_58769962423284028[95] = 1;
   out_58769962423284028[96] = 0;
   out_58769962423284028[97] = 0;
   out_58769962423284028[98] = 0;
   out_58769962423284028[99] = 0;
   out_58769962423284028[100] = 0;
   out_58769962423284028[101] = 0;
   out_58769962423284028[102] = 0;
   out_58769962423284028[103] = 0;
   out_58769962423284028[104] = dt;
   out_58769962423284028[105] = 0;
   out_58769962423284028[106] = 0;
   out_58769962423284028[107] = 0;
   out_58769962423284028[108] = 0;
   out_58769962423284028[109] = 0;
   out_58769962423284028[110] = 0;
   out_58769962423284028[111] = 0;
   out_58769962423284028[112] = 0;
   out_58769962423284028[113] = 0;
   out_58769962423284028[114] = 1;
   out_58769962423284028[115] = 0;
   out_58769962423284028[116] = 0;
   out_58769962423284028[117] = 0;
   out_58769962423284028[118] = 0;
   out_58769962423284028[119] = 0;
   out_58769962423284028[120] = 0;
   out_58769962423284028[121] = 0;
   out_58769962423284028[122] = 0;
   out_58769962423284028[123] = 0;
   out_58769962423284028[124] = 0;
   out_58769962423284028[125] = 0;
   out_58769962423284028[126] = 0;
   out_58769962423284028[127] = 0;
   out_58769962423284028[128] = 0;
   out_58769962423284028[129] = 0;
   out_58769962423284028[130] = 0;
   out_58769962423284028[131] = 0;
   out_58769962423284028[132] = 0;
   out_58769962423284028[133] = 1;
   out_58769962423284028[134] = 0;
   out_58769962423284028[135] = 0;
   out_58769962423284028[136] = 0;
   out_58769962423284028[137] = 0;
   out_58769962423284028[138] = 0;
   out_58769962423284028[139] = 0;
   out_58769962423284028[140] = 0;
   out_58769962423284028[141] = 0;
   out_58769962423284028[142] = 0;
   out_58769962423284028[143] = 0;
   out_58769962423284028[144] = 0;
   out_58769962423284028[145] = 0;
   out_58769962423284028[146] = 0;
   out_58769962423284028[147] = 0;
   out_58769962423284028[148] = 0;
   out_58769962423284028[149] = 0;
   out_58769962423284028[150] = 0;
   out_58769962423284028[151] = 0;
   out_58769962423284028[152] = 1;
   out_58769962423284028[153] = 0;
   out_58769962423284028[154] = 0;
   out_58769962423284028[155] = 0;
   out_58769962423284028[156] = 0;
   out_58769962423284028[157] = 0;
   out_58769962423284028[158] = 0;
   out_58769962423284028[159] = 0;
   out_58769962423284028[160] = 0;
   out_58769962423284028[161] = 0;
   out_58769962423284028[162] = 0;
   out_58769962423284028[163] = 0;
   out_58769962423284028[164] = 0;
   out_58769962423284028[165] = 0;
   out_58769962423284028[166] = 0;
   out_58769962423284028[167] = 0;
   out_58769962423284028[168] = 0;
   out_58769962423284028[169] = 0;
   out_58769962423284028[170] = 0;
   out_58769962423284028[171] = 1;
   out_58769962423284028[172] = 0;
   out_58769962423284028[173] = 0;
   out_58769962423284028[174] = 0;
   out_58769962423284028[175] = 0;
   out_58769962423284028[176] = 0;
   out_58769962423284028[177] = 0;
   out_58769962423284028[178] = 0;
   out_58769962423284028[179] = 0;
   out_58769962423284028[180] = 0;
   out_58769962423284028[181] = 0;
   out_58769962423284028[182] = 0;
   out_58769962423284028[183] = 0;
   out_58769962423284028[184] = 0;
   out_58769962423284028[185] = 0;
   out_58769962423284028[186] = 0;
   out_58769962423284028[187] = 0;
   out_58769962423284028[188] = 0;
   out_58769962423284028[189] = 0;
   out_58769962423284028[190] = 1;
   out_58769962423284028[191] = 0;
   out_58769962423284028[192] = 0;
   out_58769962423284028[193] = 0;
   out_58769962423284028[194] = 0;
   out_58769962423284028[195] = 0;
   out_58769962423284028[196] = 0;
   out_58769962423284028[197] = 0;
   out_58769962423284028[198] = 0;
   out_58769962423284028[199] = 0;
   out_58769962423284028[200] = 0;
   out_58769962423284028[201] = 0;
   out_58769962423284028[202] = 0;
   out_58769962423284028[203] = 0;
   out_58769962423284028[204] = 0;
   out_58769962423284028[205] = 0;
   out_58769962423284028[206] = 0;
   out_58769962423284028[207] = 0;
   out_58769962423284028[208] = 0;
   out_58769962423284028[209] = 1;
   out_58769962423284028[210] = 0;
   out_58769962423284028[211] = 0;
   out_58769962423284028[212] = 0;
   out_58769962423284028[213] = 0;
   out_58769962423284028[214] = 0;
   out_58769962423284028[215] = 0;
   out_58769962423284028[216] = 0;
   out_58769962423284028[217] = 0;
   out_58769962423284028[218] = 0;
   out_58769962423284028[219] = 0;
   out_58769962423284028[220] = 0;
   out_58769962423284028[221] = 0;
   out_58769962423284028[222] = 0;
   out_58769962423284028[223] = 0;
   out_58769962423284028[224] = 0;
   out_58769962423284028[225] = 0;
   out_58769962423284028[226] = 0;
   out_58769962423284028[227] = 0;
   out_58769962423284028[228] = 1;
   out_58769962423284028[229] = 0;
   out_58769962423284028[230] = 0;
   out_58769962423284028[231] = 0;
   out_58769962423284028[232] = 0;
   out_58769962423284028[233] = 0;
   out_58769962423284028[234] = 0;
   out_58769962423284028[235] = 0;
   out_58769962423284028[236] = 0;
   out_58769962423284028[237] = 0;
   out_58769962423284028[238] = 0;
   out_58769962423284028[239] = 0;
   out_58769962423284028[240] = 0;
   out_58769962423284028[241] = 0;
   out_58769962423284028[242] = 0;
   out_58769962423284028[243] = 0;
   out_58769962423284028[244] = 0;
   out_58769962423284028[245] = 0;
   out_58769962423284028[246] = 0;
   out_58769962423284028[247] = 1;
   out_58769962423284028[248] = 0;
   out_58769962423284028[249] = 0;
   out_58769962423284028[250] = 0;
   out_58769962423284028[251] = 0;
   out_58769962423284028[252] = 0;
   out_58769962423284028[253] = 0;
   out_58769962423284028[254] = 0;
   out_58769962423284028[255] = 0;
   out_58769962423284028[256] = 0;
   out_58769962423284028[257] = 0;
   out_58769962423284028[258] = 0;
   out_58769962423284028[259] = 0;
   out_58769962423284028[260] = 0;
   out_58769962423284028[261] = 0;
   out_58769962423284028[262] = 0;
   out_58769962423284028[263] = 0;
   out_58769962423284028[264] = 0;
   out_58769962423284028[265] = 0;
   out_58769962423284028[266] = 1;
   out_58769962423284028[267] = 0;
   out_58769962423284028[268] = 0;
   out_58769962423284028[269] = 0;
   out_58769962423284028[270] = 0;
   out_58769962423284028[271] = 0;
   out_58769962423284028[272] = 0;
   out_58769962423284028[273] = 0;
   out_58769962423284028[274] = 0;
   out_58769962423284028[275] = 0;
   out_58769962423284028[276] = 0;
   out_58769962423284028[277] = 0;
   out_58769962423284028[278] = 0;
   out_58769962423284028[279] = 0;
   out_58769962423284028[280] = 0;
   out_58769962423284028[281] = 0;
   out_58769962423284028[282] = 0;
   out_58769962423284028[283] = 0;
   out_58769962423284028[284] = 0;
   out_58769962423284028[285] = 1;
   out_58769962423284028[286] = 0;
   out_58769962423284028[287] = 0;
   out_58769962423284028[288] = 0;
   out_58769962423284028[289] = 0;
   out_58769962423284028[290] = 0;
   out_58769962423284028[291] = 0;
   out_58769962423284028[292] = 0;
   out_58769962423284028[293] = 0;
   out_58769962423284028[294] = 0;
   out_58769962423284028[295] = 0;
   out_58769962423284028[296] = 0;
   out_58769962423284028[297] = 0;
   out_58769962423284028[298] = 0;
   out_58769962423284028[299] = 0;
   out_58769962423284028[300] = 0;
   out_58769962423284028[301] = 0;
   out_58769962423284028[302] = 0;
   out_58769962423284028[303] = 0;
   out_58769962423284028[304] = 1;
   out_58769962423284028[305] = 0;
   out_58769962423284028[306] = 0;
   out_58769962423284028[307] = 0;
   out_58769962423284028[308] = 0;
   out_58769962423284028[309] = 0;
   out_58769962423284028[310] = 0;
   out_58769962423284028[311] = 0;
   out_58769962423284028[312] = 0;
   out_58769962423284028[313] = 0;
   out_58769962423284028[314] = 0;
   out_58769962423284028[315] = 0;
   out_58769962423284028[316] = 0;
   out_58769962423284028[317] = 0;
   out_58769962423284028[318] = 0;
   out_58769962423284028[319] = 0;
   out_58769962423284028[320] = 0;
   out_58769962423284028[321] = 0;
   out_58769962423284028[322] = 0;
   out_58769962423284028[323] = 1;
}
void h_4(double *state, double *unused, double *out_4089771592570596149) {
   out_4089771592570596149[0] = state[6] + state[9];
   out_4089771592570596149[1] = state[7] + state[10];
   out_4089771592570596149[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_332047580480215937) {
   out_332047580480215937[0] = 0;
   out_332047580480215937[1] = 0;
   out_332047580480215937[2] = 0;
   out_332047580480215937[3] = 0;
   out_332047580480215937[4] = 0;
   out_332047580480215937[5] = 0;
   out_332047580480215937[6] = 1;
   out_332047580480215937[7] = 0;
   out_332047580480215937[8] = 0;
   out_332047580480215937[9] = 1;
   out_332047580480215937[10] = 0;
   out_332047580480215937[11] = 0;
   out_332047580480215937[12] = 0;
   out_332047580480215937[13] = 0;
   out_332047580480215937[14] = 0;
   out_332047580480215937[15] = 0;
   out_332047580480215937[16] = 0;
   out_332047580480215937[17] = 0;
   out_332047580480215937[18] = 0;
   out_332047580480215937[19] = 0;
   out_332047580480215937[20] = 0;
   out_332047580480215937[21] = 0;
   out_332047580480215937[22] = 0;
   out_332047580480215937[23] = 0;
   out_332047580480215937[24] = 0;
   out_332047580480215937[25] = 1;
   out_332047580480215937[26] = 0;
   out_332047580480215937[27] = 0;
   out_332047580480215937[28] = 1;
   out_332047580480215937[29] = 0;
   out_332047580480215937[30] = 0;
   out_332047580480215937[31] = 0;
   out_332047580480215937[32] = 0;
   out_332047580480215937[33] = 0;
   out_332047580480215937[34] = 0;
   out_332047580480215937[35] = 0;
   out_332047580480215937[36] = 0;
   out_332047580480215937[37] = 0;
   out_332047580480215937[38] = 0;
   out_332047580480215937[39] = 0;
   out_332047580480215937[40] = 0;
   out_332047580480215937[41] = 0;
   out_332047580480215937[42] = 0;
   out_332047580480215937[43] = 0;
   out_332047580480215937[44] = 1;
   out_332047580480215937[45] = 0;
   out_332047580480215937[46] = 0;
   out_332047580480215937[47] = 1;
   out_332047580480215937[48] = 0;
   out_332047580480215937[49] = 0;
   out_332047580480215937[50] = 0;
   out_332047580480215937[51] = 0;
   out_332047580480215937[52] = 0;
   out_332047580480215937[53] = 0;
}
void h_10(double *state, double *unused, double *out_5220405385265130455) {
   out_5220405385265130455[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_5220405385265130455[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_5220405385265130455[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_4499148858588669044) {
   out_4499148858588669044[0] = 0;
   out_4499148858588669044[1] = 9.8100000000000005*cos(state[1]);
   out_4499148858588669044[2] = 0;
   out_4499148858588669044[3] = 0;
   out_4499148858588669044[4] = -state[8];
   out_4499148858588669044[5] = state[7];
   out_4499148858588669044[6] = 0;
   out_4499148858588669044[7] = state[5];
   out_4499148858588669044[8] = -state[4];
   out_4499148858588669044[9] = 0;
   out_4499148858588669044[10] = 0;
   out_4499148858588669044[11] = 0;
   out_4499148858588669044[12] = 1;
   out_4499148858588669044[13] = 0;
   out_4499148858588669044[14] = 0;
   out_4499148858588669044[15] = 1;
   out_4499148858588669044[16] = 0;
   out_4499148858588669044[17] = 0;
   out_4499148858588669044[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_4499148858588669044[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_4499148858588669044[20] = 0;
   out_4499148858588669044[21] = state[8];
   out_4499148858588669044[22] = 0;
   out_4499148858588669044[23] = -state[6];
   out_4499148858588669044[24] = -state[5];
   out_4499148858588669044[25] = 0;
   out_4499148858588669044[26] = state[3];
   out_4499148858588669044[27] = 0;
   out_4499148858588669044[28] = 0;
   out_4499148858588669044[29] = 0;
   out_4499148858588669044[30] = 0;
   out_4499148858588669044[31] = 1;
   out_4499148858588669044[32] = 0;
   out_4499148858588669044[33] = 0;
   out_4499148858588669044[34] = 1;
   out_4499148858588669044[35] = 0;
   out_4499148858588669044[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_4499148858588669044[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_4499148858588669044[38] = 0;
   out_4499148858588669044[39] = -state[7];
   out_4499148858588669044[40] = state[6];
   out_4499148858588669044[41] = 0;
   out_4499148858588669044[42] = state[4];
   out_4499148858588669044[43] = -state[3];
   out_4499148858588669044[44] = 0;
   out_4499148858588669044[45] = 0;
   out_4499148858588669044[46] = 0;
   out_4499148858588669044[47] = 0;
   out_4499148858588669044[48] = 0;
   out_4499148858588669044[49] = 0;
   out_4499148858588669044[50] = 1;
   out_4499148858588669044[51] = 0;
   out_4499148858588669044[52] = 0;
   out_4499148858588669044[53] = 1;
}
void h_13(double *state, double *unused, double *out_5128205467874007671) {
   out_5128205467874007671[0] = state[3];
   out_5128205467874007671[1] = state[4];
   out_5128205467874007671[2] = state[5];
}
void H_13(double *state, double *unused, double *out_3544321405812548738) {
   out_3544321405812548738[0] = 0;
   out_3544321405812548738[1] = 0;
   out_3544321405812548738[2] = 0;
   out_3544321405812548738[3] = 1;
   out_3544321405812548738[4] = 0;
   out_3544321405812548738[5] = 0;
   out_3544321405812548738[6] = 0;
   out_3544321405812548738[7] = 0;
   out_3544321405812548738[8] = 0;
   out_3544321405812548738[9] = 0;
   out_3544321405812548738[10] = 0;
   out_3544321405812548738[11] = 0;
   out_3544321405812548738[12] = 0;
   out_3544321405812548738[13] = 0;
   out_3544321405812548738[14] = 0;
   out_3544321405812548738[15] = 0;
   out_3544321405812548738[16] = 0;
   out_3544321405812548738[17] = 0;
   out_3544321405812548738[18] = 0;
   out_3544321405812548738[19] = 0;
   out_3544321405812548738[20] = 0;
   out_3544321405812548738[21] = 0;
   out_3544321405812548738[22] = 1;
   out_3544321405812548738[23] = 0;
   out_3544321405812548738[24] = 0;
   out_3544321405812548738[25] = 0;
   out_3544321405812548738[26] = 0;
   out_3544321405812548738[27] = 0;
   out_3544321405812548738[28] = 0;
   out_3544321405812548738[29] = 0;
   out_3544321405812548738[30] = 0;
   out_3544321405812548738[31] = 0;
   out_3544321405812548738[32] = 0;
   out_3544321405812548738[33] = 0;
   out_3544321405812548738[34] = 0;
   out_3544321405812548738[35] = 0;
   out_3544321405812548738[36] = 0;
   out_3544321405812548738[37] = 0;
   out_3544321405812548738[38] = 0;
   out_3544321405812548738[39] = 0;
   out_3544321405812548738[40] = 0;
   out_3544321405812548738[41] = 1;
   out_3544321405812548738[42] = 0;
   out_3544321405812548738[43] = 0;
   out_3544321405812548738[44] = 0;
   out_3544321405812548738[45] = 0;
   out_3544321405812548738[46] = 0;
   out_3544321405812548738[47] = 0;
   out_3544321405812548738[48] = 0;
   out_3544321405812548738[49] = 0;
   out_3544321405812548738[50] = 0;
   out_3544321405812548738[51] = 0;
   out_3544321405812548738[52] = 0;
   out_3544321405812548738[53] = 0;
}
void h_14(double *state, double *unused, double *out_7590889396526969494) {
   out_7590889396526969494[0] = state[6];
   out_7590889396526969494[1] = state[7];
   out_7590889396526969494[2] = state[8];
}
void H_14(double *state, double *unused, double *out_4295288436819700466) {
   out_4295288436819700466[0] = 0;
   out_4295288436819700466[1] = 0;
   out_4295288436819700466[2] = 0;
   out_4295288436819700466[3] = 0;
   out_4295288436819700466[4] = 0;
   out_4295288436819700466[5] = 0;
   out_4295288436819700466[6] = 1;
   out_4295288436819700466[7] = 0;
   out_4295288436819700466[8] = 0;
   out_4295288436819700466[9] = 0;
   out_4295288436819700466[10] = 0;
   out_4295288436819700466[11] = 0;
   out_4295288436819700466[12] = 0;
   out_4295288436819700466[13] = 0;
   out_4295288436819700466[14] = 0;
   out_4295288436819700466[15] = 0;
   out_4295288436819700466[16] = 0;
   out_4295288436819700466[17] = 0;
   out_4295288436819700466[18] = 0;
   out_4295288436819700466[19] = 0;
   out_4295288436819700466[20] = 0;
   out_4295288436819700466[21] = 0;
   out_4295288436819700466[22] = 0;
   out_4295288436819700466[23] = 0;
   out_4295288436819700466[24] = 0;
   out_4295288436819700466[25] = 1;
   out_4295288436819700466[26] = 0;
   out_4295288436819700466[27] = 0;
   out_4295288436819700466[28] = 0;
   out_4295288436819700466[29] = 0;
   out_4295288436819700466[30] = 0;
   out_4295288436819700466[31] = 0;
   out_4295288436819700466[32] = 0;
   out_4295288436819700466[33] = 0;
   out_4295288436819700466[34] = 0;
   out_4295288436819700466[35] = 0;
   out_4295288436819700466[36] = 0;
   out_4295288436819700466[37] = 0;
   out_4295288436819700466[38] = 0;
   out_4295288436819700466[39] = 0;
   out_4295288436819700466[40] = 0;
   out_4295288436819700466[41] = 0;
   out_4295288436819700466[42] = 0;
   out_4295288436819700466[43] = 0;
   out_4295288436819700466[44] = 1;
   out_4295288436819700466[45] = 0;
   out_4295288436819700466[46] = 0;
   out_4295288436819700466[47] = 0;
   out_4295288436819700466[48] = 0;
   out_4295288436819700466[49] = 0;
   out_4295288436819700466[50] = 0;
   out_4295288436819700466[51] = 0;
   out_4295288436819700466[52] = 0;
   out_4295288436819700466[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_2023516669171357479) {
  err_fun(nom_x, delta_x, out_2023516669171357479);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_812571722217577142) {
  inv_err_fun(nom_x, true_x, out_812571722217577142);
}
void pose_H_mod_fun(double *state, double *out_865899901402682167) {
  H_mod_fun(state, out_865899901402682167);
}
void pose_f_fun(double *state, double dt, double *out_8462551609575224310) {
  f_fun(state,  dt, out_8462551609575224310);
}
void pose_F_fun(double *state, double dt, double *out_58769962423284028) {
  F_fun(state,  dt, out_58769962423284028);
}
void pose_h_4(double *state, double *unused, double *out_4089771592570596149) {
  h_4(state, unused, out_4089771592570596149);
}
void pose_H_4(double *state, double *unused, double *out_332047580480215937) {
  H_4(state, unused, out_332047580480215937);
}
void pose_h_10(double *state, double *unused, double *out_5220405385265130455) {
  h_10(state, unused, out_5220405385265130455);
}
void pose_H_10(double *state, double *unused, double *out_4499148858588669044) {
  H_10(state, unused, out_4499148858588669044);
}
void pose_h_13(double *state, double *unused, double *out_5128205467874007671) {
  h_13(state, unused, out_5128205467874007671);
}
void pose_H_13(double *state, double *unused, double *out_3544321405812548738) {
  H_13(state, unused, out_3544321405812548738);
}
void pose_h_14(double *state, double *unused, double *out_7590889396526969494) {
  h_14(state, unused, out_7590889396526969494);
}
void pose_H_14(double *state, double *unused, double *out_4295288436819700466) {
  H_14(state, unused, out_4295288436819700466);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
