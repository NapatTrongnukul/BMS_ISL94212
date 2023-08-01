/*
 * balancing.c
 *
 *  Created on: Jul 30, 2023
 *      Author: joe_n
 */


#include "hal_data.h"
#include "balancing.h"
#include <math.h>

double time_balance_min(double time_in[12])
{
    time_min = 100000;

    for (int i = 0;i<12;i++)
        {
            if (time_in[i] >0)
                {
                    if (time_in[i]<time_min)
                        {
                            time_min = time_in[i];
                        }
                }else{
                   ;
                }
        }

    return time_min;
}


double * time_balance_recal(double time_in[12], double time_minimum)
{
    double time_rearr[12];
    memset (time_rearr, 0, 12*sizeof(double));

    for (int i = 0;i<12;i++)
        {
            time_rearr[i] =  time_in[i]-time_minimum;

                if (time_rearr[i]<=0)
                    {
                        time_rearr[i] = 0;
                    }
        }

    return time_rearr;
}

int * bal_pattern(double time_in[12])
{
    static int temp_bal[12];
    memset (temp_bal, 0, 12*sizeof(int));

    for(int i = 0;i<12;i++)
        {
            if (time_in[i]>0)
                {
                    temp_bal[i] = (int)pow(2,i);

                }else{

                    temp_bal[i] = 0;
                }
        }

    return temp_bal;

}
