/*
 * inverse_Matrix.c
 *
 *  Created on: Jul 31, 2023
 *      Author: joe_n
 */

#include <math.h>
#include "inverse_Matrix.h"

void inv(double mat[][N], int n)
   {
    double ratio = 0;
    memset (temp_inv, 0, n*n*sizeof(double));
    memset (res_3, 0, N*N*sizeof(double));

       for(int i=0;i<n;i++)
                {
                     for(int j=0;j<n;j++)
                     {
                         temp_inv[i][j] = mat[i][j];
                     }
                }

       for(int i=0;i<n;i++)
                {
                     for(int j=0;j<n;j++)
                     {
                          if(i==j)
                          {
                              temp_inv[i][j+n] = 1;
                          }
                          else
                          {
                              temp_inv[i][j+n] = 0;
                          }
                     }
                }

       for(int i=0;i<n;i++)
                {

           if(temp_inv[i][i] == 0.0)
                        {

                             exit(0);
                        }
                  for(int j=0;j<n;j++)
                     {
                          if(i!=j)
                          {
                               ratio = temp_inv[j][i]/temp_inv[i][i];
                               for(int k=0;k<2*n;k++)
                               {
                                   temp_inv[j][k] = temp_inv[j][k] - ratio*temp_inv[i][k];
                               }
                          }
                     }
                }

       for(int i=0;i<n;i++)
                {
                     for(int j=n;j<2*n;j++)
                     {
                         temp_inv[i][j] = temp_inv[i][j]/temp_inv[i][i];
                     }
               }
       for(int i = 0;i<n;i++)
       {
           for(int j = n;j<2*n;j++)
           {
               res_3[i][j-n] = temp_inv[i][j];
           }
       }

       for(int i = 0;i<N;i++)
             {
                 for(int j = 0;j<N;j++)
                 {
                     PN[i][j] = res_3[i][j];
                 }
            }
        }

void inv_3x3(double mat[][3], int n)
   {
    double ratio = 0;
    //memset (temp_inv, 0, n*n*sizeof(double));
    //memset (res_2, 0, 3*3*sizeof(double));

       for(int i=0;i<n;i++)
                {
                     for(int j=0;j<n;j++)
                     {
                         temp_inv[i][j] = mat[i][j];
                     }
                }

       for(int i=0;i<n;i++)
                {
                     for(int j=0;j<n;j++)
                     {
                          if(i==j)
                          {
                              temp_inv[i][j+n] = 1;
                          }
                          else
                          {
                              temp_inv[i][j+n] = 0;
                          }
                     }
                }

       for(int i=0;i<n;i++)
                {

           if(temp_inv[i][i] == 0.0)
                        {

                             exit(0);
                        }
                  for(int j=0;j<n;j++)
                     {
                          if(i!=j)
                          {
                               ratio = temp_inv[j][i]/temp_inv[i][i];
                               for(int k=0;k<2*n;k++)
                               {
                                   temp_inv[j][k] = temp_inv[j][k] - ratio*temp_inv[i][k];
                               }
                          }
                     }
                }

       for(int i=0;i<n;i++)
                {
                     for(int j=n;j<2*n;j++)
                     {
                         temp_inv[i][j] = temp_inv[i][j]/temp_inv[i][i];
                     }
               }
       for(int i = 0;i<n;i++)
       {
           for(int j = n;j<2*n;j++)
           {
               res_2[i][j-n] = temp_inv[i][j];
           }
       }

   }
