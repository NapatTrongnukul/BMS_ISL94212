/*
 * updateParam.c
 *
 *  Created on: Jul 31, 2023
 *      Author: joe_n
 */

#include "updateParam.h"
#include "init_RC.h"
#include <math.h>

double * updateParameters(double m[N][1],double V, double I)
    {
       double k1, temp[8];

       k1 = ((Ts*(1-m[1][0]))/(2*(1+m[1][0])));

       temp[0] =  ((m[2][0]-m[3][0])/2)*((Ts+2*k1)/(2*k1));
       temp[1] =  (((m[2][0]+m[3][0])*(Ts+2*k1))/(2*Ts))-temp[0];
       temp[2] =  k1/temp[1];
       temp[3] =  V-(I*temp[0]);
       temp[4] =  m[0][0];
       temp[5] =  m[1][0];
       temp[6] =  m[2][0];
       temp[7] =  m[3][0];

       return temp;
    }

