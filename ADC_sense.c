/*
 * ADC_sense.c
 *
 *  Created on: Jul 31, 2023
 *      Author: joe_n
 */

#include "hal_data.h"
#include "ADC_sense.h"
#include <math.h>

double ADC_sense()
    {
            double  I_sense;

            R_ADC_Read (&g_adc_ctrl, ADC_CHANNEL_23, &adc_result_23);

            I_sense = ((adc_result_23*I_CH1_CONV)+I_CH1_OFFSET);

            return I_sense;
    }

int * volt_can_conv(int volt_in)
    {
        int  volt_can[2], binary[16];
        memset (volt_can, 0, 2*sizeof(int));
        memset (binary, 0, 16*sizeof(int));

        for(int i =0; volt_in>0;i++)
           {
               binary[i] = volt_in%2;
               volt_in = volt_in/2;
           }

        volt_can[0] = (int)(binary[7]*pow(2,7)+binary[6]*pow(2,6)+binary[5]*pow(2,5)+binary[4]*pow(2,4)+binary[3]*pow(2,3)+binary[2]*pow(2,2)+binary[1]*pow(2,1)+binary[0]*pow(2,0));
        volt_can[1] = (int)(binary[15]*pow(2,7)+binary[14]*pow(2,6)+binary[13]*pow(2,5)+binary[12]*pow(2,4)+binary[11]*pow(2,3)+binary[10]*pow(2,2)+binary[9]*pow(2,1)+binary[8]*pow(2,0));

        return volt_can;
    }
