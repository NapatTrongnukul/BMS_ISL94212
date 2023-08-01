/*
 * OnlineEstimation.h
 *
 *  Created on: Jul 31, 2023
 *      Author: joe_n
 */

#ifndef ONLINEESTIMATION_H_
#define ONLINEESTIMATION_H_

#include "matrix_Operation.h"
#include "inverse_Matrix.h"

#define N  4
double *p2;

double * OnlineEstimation(double,double,double,double,double[N][1],double);

#endif /* ONLINEESTIMATION_H_ */
