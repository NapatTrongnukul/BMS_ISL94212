#include "hal_entry.h"

typedef enum {
        BMS_NORMAL_OPERATION,
        BMS_BALANCING_OPERATION,
        BMS_CAN_TX,
        BMS_SLEEPING_OPERATION

    }bms_state;

void hal_entry(void)
{
    fsp_err_t err = FSP_SUCCESS;

    R_IOPORT_Open (&g_ioport_ctrl, &g_bsp_pin_cfg);
    R_ADC_Open(&g_adc_ctrl, &g_adc_cfg);
    R_ADC_ScanCfg(&g_adc_ctrl, &g_adc_channel_cfg);
    R_ADC_Calibrate(&g_adc_ctrl, NULL);
    R_BSP_SoftwareDelay(50,BSP_DELAY_UNITS_MILLISECONDS);
    R_ADC_ScanStart(&g_adc_ctrl);

    R_CGC_Open (&g_cgc0_ctrl, &g_cgc0_cfg);
    R_CGC_ClockStart (&g_cgc0_ctrl, CGC_CLOCK_MAIN_OSC, NULL);

    do {
        err = R_CGC_ClockCheck(&g_cgc0_ctrl, CGC_CLOCK_MAIN_OSC);
            check_test = err;

        }while(FSP_ERR_NOT_STABILIZED == err);

    R_CAN_Open(&g_can0_ctrl, &g_can0_cfg);
    R_CAN_ModeTransition(&g_can0_ctrl, CAN_OPERATION_MODE_NORMAL, CAN_TEST_MODE_DISABLED);

    memset(&g_bfe0_data[0],   0, sizeof(g_bfe0_data));
    memset(&g_bfe0_faults[0], 0, sizeof(g_bfe0_faults));

    g_bfe0_user_data[0] = '1';
    g_bfe0_user_data[1] = '1';
    g_bfe0.p_api->open(&g_bfe0_ctrl, &g_bfe0_cfg);

    g_bfe0.p_api->commTest(&g_bfe0_ctrl);
    g_bfe0.p_api->userRegsAccess(&g_bfe0_ctrl, &g_bfe0_user_data[0], BFE_WRITE_REG);

    counter = 0;
    
    bms_state operation_mode = BMS_NORMAL_OPERATION;

    while (1)
    {
        switch(operation_mode)
        {
            case BMS_NORMAL_OPERATION:
            {
                while (g_timer_finished == false)
                {
                    uint8_t count_cell = 0;

                    while (1)
                    {
                        for (int i = 0;i<2;i++)
                        {
                            for (int j = 0;j<12;j++)
                            {
                                v_cell[count_cell] = volt_cells[i][j]*V_CON_const;
                            }
                        }

                        count_cell++;

                        if (count_cell == 24)
                            break;
                    }

                    for (int i = 0;i<24;i++)
                    {
                        p8 = (double*)(LookUpTable_SoC(v_cell[i]));
                        soc_cell[i] = p8[0];
                    }

                    soc_min = soc_max = soc_cell[0];

                    for (int i = 1;i<12;i++)
                    {
                        if (soc_cell[i]<soc_min)
                        {
                            soc_min = soc_cell[i];
                        }else{
                            ;
                        }

                        if (soc_cell[i]>soc_max)
                        {
                            soc_max = soc_cell[i];
                        }else{
                            ;
                        }

                        soc_diff[i] = soc_cell[i]-soc_min;
                        cou_diff[i] = soc_diff[i]*0.01*8586;
                        I_bal[i] = v_cell[i]*0.030303;
                        time_bal[i] = cou_diff[i]/I_bal[i];

                    }

                    if(counter<voltage_size)
                    {
                        V_sensor[counter] = p11[0];
                        I_sensor[counter] = p11[1];
                        p8 = (double*)(LookUpTable_SoC(p11[0]));
                        xr[0][0] = 0;
                        xr[1][0] = p8[0]/100;

                        if(counter ==voltage_size-1)
                        {
                            p5 = (double*)(init_RC(V_sensor,I_sensor));
                            zeta[0][0] = p5[4];
                            zeta[1][0] = p5[5];
                            zeta[2][0] = p5[6];
                            zeta[3][0] = p5[7];

                        }else{
                            ;
                        }
                    }else{

                        if (counter == voltage_size)
                        {
                            V_sensor[0] =  V_sensor[counter-1];
                            V_sensor[1] =  p11[0];
                            I_sensor[0] =  I_sensor[counter-1];
                            I_sensor[1] =  p11[1];
                        }else{

                            V_sensor[0] =  V_sensor[1];
                            V_sensor[1] =  p11[0];
                            I_sensor[0] =  I_sensor[1];
                            I_sensor[1] =  p11[1];
                        }

                        if (counter ==voltage_size+25)
                        {

                            p8 = (double*)(LookUpTable_SoC(V_sensor[0]));
                            xr[1][0] = p8[0]/100;
                            p3 = (double*)(OnlineEstimation(1,V_sensor[0],I_sensor[1],I_sensor[0],zeta,V_sensor[1]));
                            R0 = p3[0];
                            Rp  = p3[1];
                            Cp  = p3[2];

                            double A[2][2] = {{-1/(Rp*Cp),0},{0,1}};
                            double B[2][1] = {{1/Cp},{1/QR}};
                            double C[1][2] = {{1,p8[1]}};
                            double D[1] = {R0};

                            p9 = (double*)(kalman_init(p8[0],A,B,C,D,I_sensor[1],V_sensor[0],I_sensor[0]));
                            xr[0][0] = p9[0];
                            xr[1][0] = p9[1];
                            pkn[0][0] = p9[2];
                            pkn[0][1] = p9[3];
                            pkn[1][0] = p9[4];
                            pkn[1][1] = p9[5];

                            soc_obv[0] = p8[0]/100;
                            soc_obv[1] = p8[1]/100;

                        }else if(counter >voltage_size+25){

                            p3 = (double*)(OnlineEstimation(1,V_sensor[0],I_sensor[1],I_sensor[0],zeta,V_sensor[1]));
                            R0 = p3[0];
                            Rp  = p3[1];
                            Cp  = p3[2];

                            double A[2][2] = {{-1/(Rp*Cp),0},{0,1}};
                            double B[2][1] = {{1/Cp},{1/QR}};
                            double C[1][2] = {{1,p8[1]}};
                            double D[1] = {R0};

                            zeta[0][0] = p3[4];
                            zeta[1][0] = p3[5];
                            zeta[2][0] = p3[6];
                            zeta[3][0] = p3[7];

                            ocv_perdict = V_sensor[0]-I_sensor[0]*R0;
                            p8 = (double*)(LookUpTable_SoC(ocv_perdict));
                            p10 = (double*)(kalman_update(xr,pkn,A,B,C,D, I_sensor[1],V_sensor[0], I_sensor[0]));

                            xr[0][0] = p10[0];
                            xr[1][0] = p10[1];
                            pkn[0][0] = p10[2];
                            pkn[0][1] = p10[3];
                            pkn[1][0] = p10[4];
                            pkn[1][1] = p10[5];
                        }
                    }
                }

                g_timer_finished = false;
                break;
            }

            case BMS_BALANCING_OPERATION:  //Passive Balancing
            {
                   p13 = bal_pattern(time_bal);
                   time_bal_min = time_balance_min(time_bal);
                   bal_time = (int)(time_bal_min);
                   g_bfe0_ctrl.bal_pattern = (uint16_t)(p13[0] | p13[1] | p13[2] | p13[3] | p13[4] | p13[5] | p13[6] | p13[7] | p13[8] | p13[9] | p13[10] | p13[11]);
                   bms_err = g_bfe0.p_api->balanceControl(&g_bfe0_ctrl, BFE_CELL_BALANCE_ENABLE);
                   R_BSP_SoftwareDelay((uint32_t)(bal_time), BSP_DELAY_UNITS_SECONDS);
                   p12 = time_balance_recal(time_bal, time_bal_min);
                       for (int i=0; i<12;i++)
                       {
                         time_bal[i] = p12[i];
                       }

                   g_bfe0_ctrl.balance_cell_sel[0] = 0;
                   g_bfe0_ctrl.balance_cell_sel[1] = 0;

                break;
            }

            case BMS_CAN_TX:
            {

                for(int i =0; i<2;i++)
                       {
                        for(int j =0; j<12;j++)
                            {
                                p14 = (int*)(volt_can_conv(volt_cells[i][j]));
                                batt_volt_first[12*i+j] = (uint8_t)(p14[0]);
                                batt_volt_second[12*i+j] = (uint8_t)(p14[1]);
                            }
                       }

                   g_can_tx_frame.id = 0x600;
                   g_can_tx_frame.type = CAN_FRAME_TYPE_DATA;
                   g_can_tx_frame.data_length_code = CAN_FRAME_TRANSMIT_DATA_BYTES;
                   R_CAN_Open(&g_can0_ctrl, &g_can0_cfg);
                   R_CAN_ModeTransition(&g_can0_ctrl, CAN_OPERATION_MODE_NORMAL, CAN_TEST_MODE_DISABLED);

                   g_can_tx_frame.data[0] = batt_volt_first[0];
                   g_can_tx_frame.data[1] = batt_volt_second[0];
                   g_can_tx_frame.data[2] = batt_volt_first[1];
                   g_can_tx_frame.data[3] = batt_volt_second[1];
                   g_can_tx_frame.data[4] = batt_volt_first[2];
                   g_can_tx_frame.data[5] = batt_volt_second[2];
                   g_can_tx_frame.data[6] = batt_volt_first[3];
                   g_can_tx_frame.data[7] = batt_volt_second[3];

                   R_CAN_Write(&g_can0_ctrl, CAN_MAILBOX_NUMBER_3, &g_can_tx_frame);
                   R_BSP_SoftwareDelay(200,BSP_DELAY_UNITS_MICROSECONDS);

                   g_can_tx_frame.id = 0x601;
                   g_can_tx_frame.type = CAN_FRAME_TYPE_DATA;
                   g_can_tx_frame.data_length_code = CAN_FRAME_TRANSMIT_DATA_BYTES;

                   R_CAN_Open(&g_can0_ctrl, &g_can0_cfg);
                   R_CAN_ModeTransition(&g_can0_ctrl, CAN_OPERATION_MODE_NORMAL, CAN_TEST_MODE_DISABLED);

                   g_can_tx_frame.data[0] = batt_volt_first[4];
                   g_can_tx_frame.data[1] = batt_volt_second[4];
                   g_can_tx_frame.data[2] = batt_volt_first[5];
                   g_can_tx_frame.data[3] = batt_volt_second[5];
                   g_can_tx_frame.data[4] = batt_volt_first[6];
                   g_can_tx_frame.data[5] = batt_volt_second[6];
                   g_can_tx_frame.data[6] = batt_volt_first[7];
                   g_can_tx_frame.data[7] = batt_volt_second[7];

                   R_CAN_Write(&g_can0_ctrl, CAN_MAILBOX_NUMBER_0, &g_can_tx_frame);
                   R_BSP_SoftwareDelay(220,BSP_DELAY_UNITS_MICROSECONDS);

                   g_can_tx_frame.id = 0x602;
                   g_can_tx_frame.type = CAN_FRAME_TYPE_DATA;
                   g_can_tx_frame.data_length_code = CAN_FRAME_TRANSMIT_DATA_BYTES;
                   R_CAN_Open(&g_can0_ctrl, &g_can0_cfg);
                   R_CAN_ModeTransition(&g_can0_ctrl, CAN_OPERATION_MODE_NORMAL, CAN_TEST_MODE_DISABLED);

                   g_can_tx_frame.data[0] = batt_volt_first[8];
                   g_can_tx_frame.data[1] = batt_volt_second[8];
                   g_can_tx_frame.data[2] = batt_volt_first[9];
                   g_can_tx_frame.data[3] = batt_volt_second[9];
                   g_can_tx_frame.data[4] = batt_volt_first[10];
                   g_can_tx_frame.data[5] = batt_volt_second[10];
                   g_can_tx_frame.data[6] = batt_volt_first[11];
                   g_can_tx_frame.data[7] = batt_volt_second[11];

                   R_CAN_Write(&g_can0_ctrl, CAN_MAILBOX_NUMBER_1, &g_can_tx_frame);
                   R_BSP_SoftwareDelay(220,BSP_DELAY_UNITS_MICROSECONDS);

                   g_can_tx_frame.id = 0x603;
                   g_can_tx_frame.type = CAN_FRAME_TYPE_DATA;
                   g_can_tx_frame.data_length_code = CAN_FRAME_TRANSMIT_DATA_BYTES;
                   R_CAN_Open(&g_can0_ctrl, &g_can0_cfg);
                   R_CAN_ModeTransition(&g_can0_ctrl, CAN_OPERATION_MODE_NORMAL, CAN_TEST_MODE_DISABLED);

                   g_can_tx_frame.data[0] = batt_volt_first[12];
                   g_can_tx_frame.data[1] = batt_volt_second[12];
                   g_can_tx_frame.data[2] = batt_volt_first[13];
                   g_can_tx_frame.data[3] = batt_volt_second[13];
                   g_can_tx_frame.data[4] = batt_volt_first[14];
                   g_can_tx_frame.data[5] = batt_volt_second[14];
                   g_can_tx_frame.data[6] = batt_volt_first[15];
                   g_can_tx_frame.data[7] = batt_volt_second[15];

                   R_CAN_Write(&g_can0_ctrl, CAN_MAILBOX_NUMBER_2, &g_can_tx_frame);
                   R_BSP_SoftwareDelay(220,BSP_DELAY_UNITS_MICROSECONDS);

                   g_can_tx_frame.id = 0x604;
                   g_can_tx_frame.type = CAN_FRAME_TYPE_DATA;
                   g_can_tx_frame.data_length_code = CAN_FRAME_TRANSMIT_DATA_BYTES;
                   R_CAN_Open(&g_can0_ctrl, &g_can0_cfg);
                   R_CAN_ModeTransition(&g_can0_ctrl, CAN_OPERATION_MODE_NORMAL, CAN_TEST_MODE_DISABLED);

                   g_can_tx_frame.data[0] = batt_volt_first[16];
                   g_can_tx_frame.data[1] = batt_volt_second[16];
                   g_can_tx_frame.data[2] = batt_volt_first[17];
                   g_can_tx_frame.data[3] = batt_volt_second[17];
                   g_can_tx_frame.data[4] = batt_volt_first[18];
                   g_can_tx_frame.data[5] = batt_volt_second[18];
                   g_can_tx_frame.data[6] = batt_volt_first[19];
                   g_can_tx_frame.data[7] = batt_volt_second[19];

                   R_CAN_Write(&g_can0_ctrl, CAN_MAILBOX_NUMBER_4, &g_can_tx_frame);
                   R_BSP_SoftwareDelay(220,BSP_DELAY_UNITS_MICROSECONDS);

                   g_can_tx_frame.id = 0x605;
                   g_can_tx_frame.type = CAN_FRAME_TYPE_DATA;
                   g_can_tx_frame.data_length_code = CAN_FRAME_TRANSMIT_DATA_BYTES;
                   R_CAN_Open(&g_can0_ctrl, &g_can0_cfg);
                   R_CAN_ModeTransition(&g_can0_ctrl, CAN_OPERATION_MODE_NORMAL, CAN_TEST_MODE_DISABLED);

                   g_can_tx_frame.data[0] = batt_volt_first[20];
                   g_can_tx_frame.data[1] = batt_volt_second[20];
                   g_can_tx_frame.data[2] = batt_volt_first[21];
                   g_can_tx_frame.data[3] = batt_volt_second[21];
                   g_can_tx_frame.data[4] = batt_volt_first[22];
                   g_can_tx_frame.data[5] = batt_volt_second[22];
                   g_can_tx_frame.data[6] = batt_volt_first[23];
                   g_can_tx_frame.data[7] = batt_volt_second[23];

                   R_CAN_Write(&g_can0_ctrl, CAN_MAILBOX_NUMBER_5, &g_can_tx_frame);
                   R_BSP_SoftwareDelay(220,BSP_DELAY_UNITS_MICROSECONDS);

                   R_CAN_Close(&g_can0_ctrl);

                   break;
            }

            case BMS_SLEEPING_OPERATION:
            {
                g_bfe0.p_api->sleep(&g_bfe0_ctrl);

                break;
            }

            counter++;
        }

    }
}

void can_callback(can_callback_args_t *p_args)
{

    switch (p_args->event)
    {
        case CAN_EVENT_TX_COMPLETE:
        {

            b_can_tx = true;
            break;
        }

        case CAN_EVENT_RX_COMPLETE:
        {

            b_can_rx = true;
            memcpy(&g_can_rx_frame, p_args->p_frame, sizeof(can_frame_t));  //copy the received data to rx_frame
            break;
        }

        case CAN_EVENT_MAILBOX_MESSAGE_LOST:    //overwrite/overrun error event
        case CAN_EVENT_BUS_RECOVERY:            //Bus recovery error event
        case CAN_EVENT_ERR_BUS_OFF:             //error Bus Off event
        case CAN_EVENT_ERR_PASSIVE:             //error passive event
        case CAN_EVENT_ERR_WARNING:             //error warning event
        case CAN_EVENT_ERR_BUS_LOCK:            //error bus lock
        case CAN_EVENT_ERR_CHANNEL:             //error channel
        case CAN_EVENT_ERR_GLOBAL:              //error global
        case CAN_EVENT_TX_ABORTED:              //error transmit abort
        case CAN_EVENT_TX_FIFO_EMPTY:           //error transmit FIFO is empty
        {
            b_can_err = true;                   //set flag bit
            break;
        }
    }
}

void timer_callback(timer_callback_args_t *p_args)
{
    g_bfe0.p_api->vAllGet(&g_bfe0_ctrl, &g_bfe0_data[0]);
    I_sense = ADC_sense();
    g_timer_finished = true;
}

void periodic_timer_callback (timer_callback_args_t * p_args)
{
    if(TIMER_EVENT_CYCLE_END == p_args->event)
    {
        s_periodic_timer_event_flag = true;
    }
    else
    {
       ;
    }
}

void R_BSP_WarmStart(bsp_warm_start_event_t event)
{
    if (BSP_WARM_START_RESET == event)
    {
#if BSP_FEATURE_FLASH_LP_VERSION != 0

        /* Enable reading from data flash. */
        R_FACI_LP->DFLCTL = 1U;

#endif
    }

    if (BSP_WARM_START_POST_C == event)
    {

        /* Configure pins. */
        R_IOPORT_Open (&g_ioport_ctrl, g_ioport.p_cfg);
    }
}

#if BSP_TZ_SECURE_BUILD

BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ();

BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ()
{

}
#endif



