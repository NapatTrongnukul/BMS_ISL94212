/*
 * matrix_Operation.c
 *
 *  Created on: Jul 30, 2023
 *      Author: joe_n
 */

#include "matrix_Operation.h"
#include <math.h>

void transpose(double num[][N], int n)
    {
    memset (transpos, 0, N*n*sizeof(double));
      for (int i = 0;i < n; i++)
        {
         for (int j = 0;j < N; j++)
           {
             transpos[j][i] = num[i][j];

            }
        }
    }

void transpose_2x2(double num[][2], int n)
    {
    memset (transpos_2x2_m, 0, 2*2*sizeof(double));
      for (int i = 0;i < n; i++)
        {
         for (int j = 0;j < 2; j++)
           {
             transpos_2x2_m[j][i] = num[i][j];

           }
         }
     }

void transpose_1x4(double num[][N], int n)
    {
    memset (transpos_4x1, 0, N*n*sizeof(double));
      for (int i = 0;i < n; i++)
        {
         for (int j = 0;j < N; j++)
           {
                 transpos_4x1[j][i] = num[i][j];

            }
        }
    }

void transpose_1x2(double num[][2])
    {
    memset (transpos_2x1_m, 0, 2*1*sizeof(double));
      for (int i = 0;i < 1; i++)
        {
         for (int j = 0;j < 2; j++)
           {
                 transpos_2x1_m[j][i] = num[i][j];

            }
        }
    }

void transpose_2x1(double num[][1])
    {
    memset (transpos_1x2_m, 0, 1*2*sizeof(double));
      for (int i = 0;i < 2; i++)
        {
         for (int j = 0;j < 1; j++)
           {
                 transpos_1x2_m[j][i] = num[i][j];

            }
        }
    }

void transpose_4x1(double num[N][1], int n)
    {
    memset (transpos_1x4, 0, n*N*sizeof(double));
      for (int i = 0;i < N; i++)
        {
         for (int j = 0;j < n; j++)
           {
                 transpos_1x4[j][i] = num[i][j];

            }
        }
    }
void transpose_yt(double num[][DataRowSize])
    {
    memset (transpos_yt, 0, DataRowSize*sizeof(double));
      for (int i = 0;i < 1; i++)
        {
         for (int j = 0;j < DataRowSize; j++)
           {
                 transpos_yt[j][i] = num[i][j];

            }
        }
    }
