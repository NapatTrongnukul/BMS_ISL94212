/*
 * SOC_lookUptable.c
 *
 *  Created on: Jul 31, 2023
 *      Author: joe_n
 */

#include "SOC_lookUptable.h"
#include "hal_data.h"

double * LookUpTable_SoC(double g1)
{
    double ocv;
    double  temp_soc[3];
    memset (temp_soc, 0, 3*sizeof(double));
    ocv = g1;

    if (ocv <= 3.299)
        {
            soc_t = 0;
            slope_predict = slope[0];
            y_intercept = y_c[0];

         }
            else if(ocv > 3.299 && ocv<= 3.466)
               {
                    soc_t = slope[0]*ocv+y_c[0];
                    slope_predict = slope[0];
                    y_intercept = y_c[0];
               }
            else if(ocv > 3.466 && ocv<= 3.549)
               {
                    soc_t = slope[1]*ocv+y_c[1];
                    slope_predict = slope[1];
                    y_intercept = y_c[1];
               }
            else if(ocv > 3.549 && ocv<= 3.578)
               {
                    soc_t = slope[2]*ocv+y_c[2];
                    slope_predict = slope[2];
                    y_intercept = y_c[2];
               }
            else if(ocv > 3.578 && ocv<= 3.602)
               {
                    soc_t = slope[3]*ocv+y_c[3];
                    slope_predict = slope[3];
                    y_intercept = y_c[3];
               }
            else if(ocv > 3.602 && ocv<= 3.654)
               {
                    soc_t = slope[4]*ocv+y_c[4];
                    slope_predict = slope[4];
                    y_intercept = y_c[4];
               }
            else if(ocv > 3.654 && ocv<= 3.743)
               {
                    soc_t = slope[5]*ocv+y_c[5];
                    slope_predict = slope[5];
                    y_intercept = y_c[5];
               }
            else if(ocv > 3.743 && ocv<= 3.834)
               {
                    soc_t = slope[6]*ocv+y_c[6];
                    slope_predict = slope[6];
                    y_intercept = y_c[6];
               }
            else if(ocv > 3.834 && ocv<= 3.933)
               {
                    soc_t = slope[7]*ocv+y_c[7];
                    slope_predict = slope[7];
                    y_intercept = y_c[7];
               }
            else if(ocv > 3.933 && ocv<= 4.017)
               {
                    soc_t = slope[8]*ocv+y_c[8];
                    slope_predict = slope[8];
                    y_intercept = y_c[8];
               }
            else if(ocv > 4.017 && ocv<= 4.067)
               {
                    soc_t = slope[9]*ocv+y_c[9];
                    slope_predict = slope[9];
                    y_intercept = y_c[9];
               }
            else if(ocv > 4.067 && ocv<= 4.095)
               {
                    soc_t = slope[10]*ocv+y_c[10];
                    slope_predict = slope[10];
                    y_intercept = y_c[10];
               }
            else if(ocv > 4.095 && ocv<= 4.113)
               {
                    soc_t = slope[11]*ocv+y_c[11];
                    slope_predict = slope[11];
                    y_intercept = y_c[11];
               }
            else if(ocv > 4.113 && ocv<= 4.121)
               {
                    soc_t = slope[12]*ocv+y_c[12];
                    slope_predict = slope[12];
                    y_intercept = y_c[12];
               }
            else if(ocv > 4.121)
               {
                    soc_t = 100;
                    slope_predict = slope[12];
                    y_intercept = y_c[12];
               }
            else
               {
                        ;
               }

            temp_soc[0] = soc_t;
            temp_soc[1] = slope_predict;
            temp_soc[2] = y_intercept;

    return temp_soc;
    }
