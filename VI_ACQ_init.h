/*
 * VI_ACQ_init.h
 *
 *  Created on: Jul 31, 2023
 *      Author: joe_n
 */

#ifndef VI_ACQ_INIT_H_
#define VI_ACQ_INIT_H_

#include <r_bms.h>
#include <bfe/r_bfe_isl94212.h>

#define number_of_cell  24
#define number_of_current 1


st_bfe_meas_t   g_bfe0_data[BFE_CFG_STA_DEV] ;
st_bfe_faults_t g_bfe0_faults[BFE_CFG_STA_DEV];
uint16_t        g_bfe0_user_data[BFE_USR_REG_NUM * BFE_CFG_STA_DEV] ;
uint16_t adc_result_23,adc_result_22,adc_result_00,adc_result_01;

#define V_CH_CONV 0.0006103515625
#define V_CH1_OFFSET 0.0640603

double I_CH1;
extern uint16_t volt_cells[2][12];

double * VI_ACQ_init();


#endif /* VI_ACQ_INIT_H_ */
