/*
 * OnlineEstimation.c
 *
 *  Created on: Jul 31, 2023
 *      Author: joe_n
 */

#include "hal_data.h"
#include "OnlineEstimation.h"
#include "matrix_Operation.h"
#include <math.h>


double * OnlineEstimation(double g1,double g2,double g3,double g4,double z[N][1],double g5)
{
    double phi_n[1][N];
    static double res_6[N][1],PN1[N][N],gamma_n[N][1],zeta_n[N][1];
    double res_7 = 0;
    double res_9 = 0;
    static double yt_n = 0;

    memset (res_6, 0, N*1*sizeof(double));
    memset (PN1, 0, N*N*sizeof(double));
    memset (gamma_n, 0, N*1*sizeof(double));
    memset (zeta_n, 0, N*1*sizeof(double));
    memset (phi_n, 0, N*sizeof(double));

    phi_n[0][0] = g1;
    phi_n[0][1] = -g2;
    phi_n[0][2] = g3;
    phi_n[0][3] = g4;    //81.6us

    transpose_1x4(phi_n,1);  //82.4us

    for (int i = 0; i < N; i++)
    {
            for (int j = 0; j < 1; j++)
                {
                    for (int k = 0; k < N; k++)
                            res_6[i][j] += PN[i][k]*transpos_4x1[k][j];
                }
    }//243us

    for (int i = 0; i < 1; i++)
    {
            for (int j = 0; j < 1; j++)
                {
                    for (int k = 0; k < N; k++)

                            res_7 += phi_n[i][k]*res_6[k][j];

                }
    }

    res_7 = 1/(res_7+1);

    transpose_4x1(res_6,1);  //245us

    for (int i = 0; i < N; i++)
    {
            for (int j = 0; j < N; j++)
                {
                    for (int k = 0; k < 1; k++)
                        {

                            PN1[i][j] += res_6[i][k]*transpos_1x4[k][j];

                        }

                    PN1[i][j] = res_7*PN1[i][j];
                    PN1[i][j] = PN[i][j]-PN1[i][j];

                }
    }

    yt_n = g5;  //550us

    for (int i = 0; i < N; i++)
    {
            for (int j = 0; j < 1; j++)
                {
                    for (int k = 0; k < N; k++)
                        {

                            gamma_n[i][j] += PN1[i][k]*transpos_4x1[k][j];

                        }

                }

    }//709us

    for (int i = 0; i < 1; i++)
    {
            for (int j = 0; j < 1; j++)
                {
                    for (int k = 0; k < N; k++)
                        {

                            res_9 += phi_n[i][k]*z[k][j];
                        }

                }

                res_9 = yt_n-res_9;

    }//725us

    for (int i = 0; i < N; i++)
    {
            for (int j = 0; j < 1; j++)
                {

                    zeta_n[i][j] = res_9*gamma_n[i][j];
                    zeta_n[i][j] = z[i][j]+zeta_n[i][j];

                }
    }//814us

    for (int i = 0; i < N; i++)
    {
            for (int j = 0; j < N; j++)
                {

                    PN[i][j] = PN1[i][j];

                }
    }//816us

        p2 = updateParameters(zeta_n,g2,g3);    //942us

        return p2;  //947us
}
