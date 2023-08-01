/*
 * init_RC.h
 *
 *  Created on: Jul 31, 2023
 *      Author: joe_n
 */

#ifndef INIT_RC_H_
#define INIT_RC_H_

#include "hal_data.h"
#include "matrix_Operation.h"
#include "inverse_Matrix.h"
#include "updateParam.h"
#include <math.h>

#define N  4
#define Ts 0.1
#define voltage_size 12
#define current_size 12
#define DataRowSize voltage_size-1



double res_1[N][N],res_4[N][1],yt[1][DataRowSize],phi[DataRowSize][N],*p1;
double * init_RC(double[voltage_size],double[current_size]);

#endif /* INIT_RC_H_ */
