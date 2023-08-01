/*
 * inverse_Matrix.h
 *
 *  Created on: Jul 31, 2023
 *      Author: joe_n
 */

#ifndef INVERSE_MATRIX_H_
#define INVERSE_MATRIX_H_

#define N  4

double temp_inv[2*N][2*N],res_2[3][3],res_3[N][N],PN[N][N];

void inv(double [][N], int);
void inv_3x3(double [][3], int);

#endif /* INVERSE_MATRIX_H_ */
