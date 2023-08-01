/*
 * matrix_Operation.h
 *
 *  Created on: Jul 30, 2023
 *      Author: joe_n
 */
#include "hal_data.h"

#ifndef MATRIX_OPERATION_H_
#define MATRIX_OPERATION_H_

#define voltage_size 12
#define N 4
#define DataRowSize voltage_size-1


void transpose(double [][N], int);
void transpose_yt(double [][DataRowSize]);
void transpose_1x4(double [][N], int);
void transpose_1x2(double [][2]);
void transpose_2x1(double [][1]);
void transpose_4x1(double [N][1], int);
void transpose_2x2(double [][2], int);

double transpos_4x1[N][1],transpos_1x4[1][N],transpos_2x2_m[2][2],transpos_2x1_m[2][1],transpos_1x2_m[1][2];
double transpos[N][DataRowSize],transpos_yt[DataRowSize][1];


#endif /* MATRIX_OPERATION_H_ */
