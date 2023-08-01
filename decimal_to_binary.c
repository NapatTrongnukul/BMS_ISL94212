/*
 * decimal_to_binary.c
 *
 *  Created on: Jul 31, 2023
 *      Author: joe_n
 */

#include "decimal_to_binary.h"

int * decimal_to_binary_16bit(int param_in)
    {

      int  param_out_16bit[2], binary_out_16bit[16];
       memset(param_out_16bit, 0, 2*sizeof(int));
       memset(binary_out_16bit, 0, 16*sizeof(int));

       for(int i =0; param_in>0;i++)
           {
              binary_out_16bit[i] = param_in%2;
              param_in = param_in/2;
           }

       param_out_16bit[0] = (int)(binary_out_16bit[7]*pow(2,7)+binary_out_16bit[6]*pow(2,6)+binary_out_16bit[5]*pow(2,5)+binary_out_16bit[4]*pow(2,4)+binary_out_16bit[3]*pow(2,3)+binary_out_16bit[2]*pow(2,2)+binary_out_16bit[1]*pow(2,1)+binary_out_16bit[0]*pow(2,0));
       param_out_16bit[1] = (int)(binary_out_16bit[15]*pow(2,7)+binary_out_16bit[14]*pow(2,6)+binary_out_16bit[13]*pow(2,5)+binary_out_16bit[12]*pow(2,4)+binary_out_16bit[11]*pow(2,3)+binary_out_16bit[10]*pow(2,2)+binary_out_16bit[9]*pow(2,1)+binary_out_16bit[8]*pow(2,0));

       return  param_out_16bit;
    }
