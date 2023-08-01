/*
 * kalman.h
 *
 *  Created on: Jul 30, 2023
 *      Author: joe_n
 */

#include "matrix_Operation.h"
#include "init_RC.h"
#include "hal_data.h"

#ifndef KALMAN_H_
#define KALMAN_H_

double * kalman_init(double ,double [2][2],double [2][1],double [1][2],double,double,double);
double * kalman_update(double[2][1],double[2][2],double[2][2],double[2][1],double[1][2],double,double, double);

double *p8,soc_obv[2],x0[2][1],xr[2][1],soc_cou_0,pk[2][2];
double temp_1[2][1],temp_4[2][2],temp_5,temp_6[2][2];
double yx,xsp[2][1],x1[2][1],x2[2][1],zeta[N][1],pkn[2][2];;

double P0[2][2] = {{0.000004,0},{0,0.0000025}};
double Qm[2][2] = {{2.7129e-5,0},{0,2.7129e-5}};
double Rm = 6.7243e-04;

#endif /* KALMAN_H_ */
