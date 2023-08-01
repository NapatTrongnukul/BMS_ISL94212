/*
 * kalman.c
 *
 *  Created on: Jul 30, 2023
 *      Author: joe_n
 */

#include "kalman.h"

double * kalman_init(double a1,double a2[2][2],double a3[2][1],double a4[1][2], double I_n,double V, double I)
    {
         double  temp[6];
         memset (temp, 0, 6*sizeof(double));

         a1 = a1/100;

         soc_cou_0 = p8[0]/100;
         soc_obv[0] = p8[0]/100;
         soc_obv[1] = p8[0]/100;
         x0[0][0] =  0;
         x0[1][0] =  p8[0]/100;

         transpose_2x2(a2,2);

         xr[0][0] = (a2[0][0]*x0[0][0]+a2[0][1]*x0[1][0])+(a3[0][0]*I);
         xr[1][0] = (a2[1][0]*x0[0][0]+a2[1][1]*x0[1][0])+(a3[1][0]*I);

         for(int ii = 0; ii<2;ii++)
              {
                  for(int jj = 0; jj<2;jj++)
                      {
                          temp_4[ii][jj] = P0[ii][jj]*transpos_2x2_m[ii][jj];
                          pk[ii][jj] = a2[ii][jj]*temp_4[ii][jj];
                      }
              }
          pk[0][0] = pk[0][0]+Qm[0][0];
          pk[1][1] = pk[1][1]+Qm[1][1];

          transpose_1x2(a4);

          temp_1[0][0] =  pk[0][0]*transpos_2x1_m[0][0]+pk[0][1]*transpos_2x1_m[1][0];
          temp_1[1][0] =  pk[1][0]*transpos_2x1_m[0][0]+pk[1][1]*transpos_2x1_m[1][0];

          temp_5 = 1/(a4[0][0]*temp_1[0][0]+a4[0][1]*temp_1[1][0]+Rm);

          temp_1[0][0] = temp_1[0][0]*temp_5;
          temp_1[1][0] = temp_1[1][0]*temp_5;

           //yx = (a4[0][0]*xr[0][0]+a4[0][1]*xr[1][0])+(a5[0]*I)+p8[2];

           yx = zeta[0][0]-zeta[1][0]*V+zeta[2][0]*I_n+zeta[3][0]*I;
           xsp[0][0] = xr[0][0]+(temp_1[0][0])*(V-yx);
           xsp[1][0] = xr[1][0]+(temp_1[1][0])*(V-yx);

           for(int ii = 0; ii<2;ii++)
              {
                 for(int jj = 0; jj<1;jj++)
                        {
                            for(int kk = 0; kk<2;kk++)
                                 {
                                    temp_6[ii][kk] += temp_1[ii][jj]*a4[jj][kk] ;
                                 }
                        }
               }
           for(int ii = 0; ii<2;ii++)
               {
                  for(int jj = 0; jj<2;jj++)
                        {
                             for(int kk = 0; kk<2;kk++)
                                 {
                                    pkn[ii][kk] += temp_6[ii][jj]*pk[jj][kk];

                                 }
                         }
               }
            for(int ii = 0; ii<2;ii++)
               {
                  for(int jj = 0; jj<2;jj++)
                         {
                             if (ii == jj)
                                 {
                                    pkn[ii][jj] = 1-pkn[ii][jj];
                                 }else{
                                    pkn[ii][jj] = 0;
                                 }
                          }
               }

            temp[0] = xsp[0][0];
            temp[1] = xsp[1][0];

            temp[2] = pk[0][0];
            temp[3] = pk[0][1];
            temp[4] = pk[1][0];
            temp[5] = pk[1][1];

            return temp;
    }

double * kalman_update(double xr_s[2][1],double pkn_s[2][2],double a2[2][2],double a3[2][1],double a4[1][2], double I_n,double V, double I)
    {
           double  temp[6];
            memset (temp, 0, 6*sizeof(double));

           x1[0][0] = (a2[0][0]*xr_s[0][0]+a2[0][1]*xr_s[1][0])+(a3[0][0]*I);
           x1[1][0] = (a2[1][0]*xr_s[0][0]+a2[1][1]*xr_s[1][0])+(a3[1][0]*I);

           transpose_2x2(a2,2);

           for(int ii = 0; ii<2;ii++)
               {
                  for(int jj = 0; jj<2;jj++)
                        {
                           temp_4[ii][jj] = pkn_s[ii][jj]*transpos_2x2_m[ii][jj];
                           pk[ii][jj] = a2[ii][jj]*temp_4[ii][jj];
                        }
               }

           pk[0][0] = pk[0][0]+Qm[0][0];
           pk[1][1] = pk[1][1]+Qm[1][1];

           if ((pk[0][0]>1.0e-05) |  (pk[0][0]<-1.0e-05))
                 {
                     pk[0][0] = 1e-10;
                 }else{
                      ;
                 }
           if ((pk[1][1]>1.0e-05) |  (pk[1][1]<-1.0e-05))
                 {
                      pk[1][1] = 1e-10;
                 }else{
                      ;
                 }

           transpose_1x2(a4);

           temp_1[0][0] =  pk[0][0]*transpos_2x1_m[0][0]+pk[0][1]*transpos_2x1_m[1][0];
           temp_1[1][0] =  pk[1][0]*transpos_2x1_m[0][0]+pk[1][1]*transpos_2x1_m[1][0];

           temp_5 = 1/(a4[0][0]*temp_1[0][0]+a4[0][1]*temp_1[1][0]+Rm);

           temp_1[0][0] = temp_1[0][0]*temp_5;
           temp_1[1][0] = temp_1[1][0]*temp_5;

           yx = zeta[0][0]-zeta[1][0]*V+zeta[2][0]*I_n+zeta[3][0]*I;


           x2[0][0] = 0;
           x2[1][0] = x1[1][0]+(temp_1[1][0])*(V-yx);

           temp[0] = x2[0][0];
           temp[1] = x2[1][0];

           temp[2] = pk[0][0];
           temp[3] = pk[0][1];
           temp[4] = pk[1][0];
           temp[5] = pk[1][1];

           return temp;
    }
