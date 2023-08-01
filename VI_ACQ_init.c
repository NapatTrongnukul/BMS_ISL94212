/*
 * VI_ACQ_init.c
 *
 *  Created on: Jul 31, 2023
 *      Author: joe_n
 */

#include "VI_ACQ_init.h"
#include <math.h>

double * VI_ACQ_init()
    {

    double temp_6[number_of_cell+number_of_current];

    R_ADC_Read (&g_adc_ctrl, ADC_CHANNEL_23, &adc_result_23);
    g_bfe0.p_api->vAllGet(&g_bfe0_ctrl, &g_bfe0_data[0]);

    temp_6[0] = volt_cells[0][0]*V_CH_CONV;
    temp_6[1] = volt_cells[0][1]*V_CH_CONV;
    temp_6[2] = volt_cells[0][2]*V_CH_CONV;
    temp_6[3] = volt_cells[0][3]*V_CH_CONV;
    temp_6[4] = volt_cells[0][4]*V_CH_CONV;
    temp_6[5] = volt_cells[0][5]*V_CH_CONV;
    temp_6[6] = volt_cells[0][6]*V_CH_CONV;
    temp_6[7] = volt_cells[0][7]*V_CH_CONV;
    temp_6[8] = volt_cells[0][8]*V_CH_CONV;
    temp_6[9] = volt_cells[0][9]*V_CH_CONV;
    temp_6[10] = volt_cells[0][10]*V_CH_CONV;
    temp_6[11] = volt_cells[0][11]*V_CH_CONV;
    temp_6[12] = volt_cells[1][0]*V_CH_CONV;
    temp_6[13] = volt_cells[1][1]*V_CH_CONV;
    temp_6[14] = volt_cells[1][2]*V_CH_CONV;
    temp_6[15] = volt_cells[1][3]*V_CH_CONV;
    temp_6[16] = volt_cells[1][4]*V_CH_CONV;
    temp_6[17] = volt_cells[1][5]*V_CH_CONV;
    temp_6[18] = volt_cells[1][6]*V_CH_CONV;
    temp_6[19] = volt_cells[1][7]*V_CH_CONV;
    temp_6[20] = volt_cells[1][8]*V_CH_CONV;
    temp_6[21] = volt_cells[1][9]*V_CH_CONV;
    temp_6[22] = volt_cells[1][10]*V_CH_CONV;
    temp_6[23] = volt_cells[1][11]*V_CH_CONV;
    temp_6[24] = I_CH1;

    return temp_6;
    }
