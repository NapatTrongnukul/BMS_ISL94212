/*
 * hal_entry.h
 *
 *  Created on: Jul 30, 2023
 *      Author: joe_n
 */

#ifndef HAL_ENTRY_H_
#define HAL_ENTRY_H_

#include "common_utils.h"
#include "hal_data.h"
#include <stdio.h>
#include <stdlib.h>
#include <r_bms.h>
#include <bfe/r_bfe_isl94212.h>
#include <math.h>
#include "matrix_Operation.h"
#include "updateParam.h"
#include "inverse_Matrix.h"
#include "VI_ACQ_init.h"
#include "balancing.h"
#include "decimal_to_binary.h"


#define WAIT_TIME  (1000U)
#define CAN_MAILBOX_NUMBER_0            (0U)
#define CAN_MAILBOX_NUMBER_1            (1U)
#define CAN_MAILBOX_NUMBER_2            (2U)
#define CAN_MAILBOX_NUMBER_3            (3U)
#define CAN_MAILBOX_NUMBER_4            (4U)
#define CAN_MAILBOX_NUMBER_5            (5U)
#define CAN_MAILBOX_NUMBER_6            (6U)
#define CAN_MAILBOX_NUMBER_7            (7U)
#define CAN_FRAME_TRANSMIT_DATA_BYTES   (8U)
#define V_CON_const 0.000610426

static volatile bool b_can_tx = false;
static volatile bool b_can_rx = false;
static volatile bool b_can_err = false;

can_frame_t g_can_tx_frame;
can_frame_t g_can_rx_frame;

uint32_t  counter_balance_time  = 0;
uint32_t  counter_balance_loops = 0;

#define number_of_cell  24
#define number_of_current 1
#define voltage_size 12
#define current_size 12
#define voltage_size_n 89
#define current_size_n 89

double *p3,*p5,*p8,*p9,*p10,*p11;
double V_sensor[20], I_sensor[20],xr[4][4],zeta[4][4],pkn[4][4],soc_obv[4],I_sense;
double R0,Rp,Cp,QR,ocv_perdict;

e_bms_err_t  bms_err = BMS_SUCCESS;

bool g_timer_finished = false;

static volatile bool s_periodic_timer_event_flag = false;

uint8_t batt_volt_first[24];
uint8_t batt_volt_second[24];
uint8_t check_test;
uint8_t balance_time  = 0;


uint8_t CGC_Open,CGC_ClockStart,CAN_Open, CAN_Write,CAN_Write_2,timeout_1,timeout_2,CAN_success;
uint8_t count;

void R_BSP_WarmStart(bsp_warm_start_event_t event);

static void agt_init (void);

st_bfe_meas_t   g_bfe0_data[BFE_CFG_STA_DEV] ;
st_bfe_faults_t g_bfe0_faults[BFE_CFG_STA_DEV];
uint16_t        g_bfe0_user_data[BFE_USR_REG_NUM * BFE_CFG_STA_DEV] ;

st_bms_trans_flags_t   g_bms_trans_flags;

uint8_t open_error = 0;
uint8_t comTest_error = 0;
uint8_t userRegs_error = 0;

double v_cell[24],soc_cell[24], soc_diff[12],cou_diff[12],I_bal[12],time_bal[12];
double soc_max,soc_min;

double *p12;
int *p13,*p14;

int counter = 0;
int bal_time;


#endif /* HAL_ENTRY_H_ */
