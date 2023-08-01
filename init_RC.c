/*
 * init_RC.c
 *
 *  Created on: Jul 31, 2023
 *      Author: joe_n
 */


#include "init_RC.h"

double * init_RC(double V[voltage_size],double I[current_size])
{
    double zeta_temp[N][1], zeta_temp_2[N];
    memset (res_1, 0, N*N*sizeof(double));
    memset (res_4, 0, N*sizeof(double));
    memset (zeta_temp, 0, N*sizeof(double));
    memset (zeta_temp_2, 0, N*sizeof(double));

    for(int i = 0;i<DataRowSize;i++)
        {

                yt[0][i] = V[i+1];
        }

    for (int i = 0;i<DataRowSize;i++)
        {

        phi[i][0] = 1;
        phi[i][1] = -V[i];
        phi[i][2] = I[i+1];
        phi[i][3] = I[i];

        }

        transpose(phi,DataRowSize);
        transpose_yt(yt);

        for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < N; j++)
                    {
                        for (int k = 0; k < DataRowSize; k++)
                            {
                                 res_1[i][j] += transpos[i][k]*phi[k][j];
                            }
                    }
            }

         for (int i = 0; i < N; i++)
            {
                 for (int j = 0; j < 1; j++)
                     {
                         for (int k = 0; k < DataRowSize; k++)

                             res_4[i][j] += transpos[i][k]*transpos_yt[k][j];
                     }
            }

        inv(res_1,4);

        for (int i = 0; i < N; i++)
            {
                  for (int j = 0; j < 1; j++)
                      {
                          for (int k = 0; k < N; k++)

                              zeta_temp[i][j] += res_3[i][k]*res_4[k][j];

                      }
            }

        for(int i = 0;i<N;i++)
            {

                zeta_temp_2[i] = zeta_temp[i][0];

            }

        p1 = updateParameters(zeta_temp,V[DataRowSize],I[DataRowSize]);

        return p1;

}

