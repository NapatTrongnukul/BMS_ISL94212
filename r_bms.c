
#include <stdint.h>
#include <stdbool.h>
#include <string.h>


#include <r_bms.h>

 #if BFE_CFG_STA_DEV > 1U
st_bfe_meas_t   g_bfe0_data[BFE_CFG_STA_DEV]                        = {0};
st_bfe_faults_t g_bfe0_faults[BFE_CFG_STA_DEV]                      = {0};
uint16_t        g_bfe0_user_data[BFE_USR_REG_NUM * BFE_CFG_STA_DEV] = {0};
#else
st_bfe_meas_t   g_bfe0_data[ONE]                        = {0};
st_bfe_faults_t g_bfe0_faults[ONE]                      = {0};
uint16_t        g_bfe0_user_data[BFE_USR_REG_NUM * ONE] = {0};
#endif

/* Global variables for the USB */
usb_status_t     usb_event           = USB_STATUS_NONE;
usb_setup_t      usb_setup           = {0};
uint8_t          g_usb_module_number = 0x00;
usb_class_t      g_usb_class_type    = 0x00;
static bool      g_usb_attach        = false;
static bool      g_do_once           = false;

char g_print_data_buff[USB_LONG_PACKET_SIZE] = {'\0'}; // User interface display data buffer.

volatile e_bms_machine_states_t bms_state = BMS_INIT_STATE;
volatile e_bms_balance_states_t bal_state = BAL_MEASURE;

volatile st_bms_trans_flags_t   g_bms_trans_flags =
                                                    {
                                                         .goto_normal  = false,
                                                         .goto_sleep   = false,
                                                         .goto_balance = false,
                                                         .exit_balance = false,
                                                         .goto_fault   = false,
                                                         .clear_fault  = false,
                                                         .wakeup       = false,
                                                         .measure      = false
                                                    };

/* Private functions */
static fsp_err_t   check_for_write_complete(void);
static fsp_err_t   print_to_console (char *p_data);
static e_bms_err_t cell_balance (void);
static e_bms_err_t fault_management (e_bms_err_t bms_err);
static fsp_err_t   measured_data_process (st_bfe_meas_t * p_data);
static fsp_err_t   faults_data_process (st_bfe_faults_t * p_faults);


void r_bms_state_machine(void)
{
    static e_bms_err_t s_bfe_err = BMS_SUCCESS; // Error status
    static fsp_err_t   fsp_err   = FSP_SUCCESS; // Error status

    static uint32_t  s_counter_mem_check = 0; // Memory check interval counter

    /* State machine transition rules. */
    if(g_bms_trans_flags.goto_fault == true)
    {

        g_bms_trans_flags.goto_fault = false; // Clear transition flag

        g_do_once = true;

        bms_state = BMS_FAULT_STATE;
    }
    else if(g_bms_trans_flags.goto_normal == true)
    {

        g_bms_trans_flags.goto_normal = false; // Clear transition flag

        bms_state = BMS_NORMAL_STATE;
    }
    else if((bms_state == BMS_NORMAL_STATE) && (g_bms_trans_flags.goto_sleep == true))
    {

        g_bms_trans_flags.goto_sleep = false; // Clear transition flag

        bms_state = BMS_SLEEP_STATE;
    }
    else if((bms_state == BMS_NORMAL_STATE) && (g_bms_trans_flags.goto_balance == true))
    {

        g_bms_trans_flags.goto_balance = false; // Clear transition flag

        bal_state = BAL_MEASURE;
        bms_state = BMS_BALANCE_STATE;
    }
    else
    {
        ;
    }

    /* Process states. */
    switch(bms_state)
    {
        case BMS_INIT_STATE: // Initialize BMS on start-up
        {

            /* Clean data structures. */
            memset(&g_bfe0_data[0],   ZERO, sizeof(g_bfe0_data));
            memset(&g_bfe0_faults[0], ZERO, sizeof(g_bfe0_faults));

            /* User data */
            g_bfe0_user_data[0] = '1';
            g_bfe0_user_data[1] = '1';

            /* Initialize the Battery Front End. */
            s_bfe_err = g_bfe0.p_api->open(&g_bfe0_ctrl, &g_bfe0_cfg);

            if(s_bfe_err != BMS_SUCCESS) // Check for error return
            {
                g_bms_trans_flags.goto_fault = true; // Go to Fault state to process error.
                break;
            }
            else
            {
                ;
            }

            /* Test communication. */
            s_bfe_err = g_bfe0.p_api->commTest(&g_bfe0_ctrl);

            if(s_bfe_err != BMS_SUCCESS) // Check for error return
            {
                g_bms_trans_flags.goto_fault = true; // Go to Fault state to process error.
                break;
            }
            else
            {
                ;
            }

            /* Write user data. */
            s_bfe_err = g_bfe0.p_api->userRegsAccess(&g_bfe0_ctrl, &g_bfe0_user_data[0], BFE_WRITE_REG);

            if(s_bfe_err != BMS_SUCCESS) // Check for error return
            {
                g_bms_trans_flags.goto_fault = true; // Go to Fault state to process error.
                break;
            }
            else
            {
                ;
            }

            g_bms_trans_flags.goto_normal = true;

            break;
        }

        case BMS_NORMAL_STATE:
        {

            if(g_do_once == true) // Send to terminal once
            {
                /* Print banner info to console. */
                fsp_err = print_to_console((char *) (MAIN_MENU));

                if (FSP_SUCCESS != fsp_err) // Check for error return
                {
                   s_bfe_err = BMS_ERR_FSP;

                   g_bms_trans_flags.goto_fault = true; // Go to Fault state to process error.
                   break;
                }
                else
                {
                    ;
                }

                g_do_once = false;
            }
            else
            {
                ;
            }

            if(g_bms_trans_flags.measure == true) // Is measure command sent?
            {
                /* Acquire all voltages and temperatures. */
                s_bfe_err = g_bfe0.p_api->voltTempAllGet(&g_bfe0_ctrl, &g_bfe0_data[0]);

                if(s_bfe_err != BMS_SUCCESS) // Check for error return
                {
                    g_bms_trans_flags.goto_fault = true; // Go to Fault state to process error.
                    break;
                }
                else
                {
                    ;
                }

                fsp_err = print_to_console((char *) (MEAS_MSG_2));

                if (FSP_SUCCESS != fsp_err) // Check for error return
                {
                   s_bfe_err = BMS_ERR_FSP;

                   g_bms_trans_flags.goto_fault = true; // Go to Fault state to process error.
                   break;
                }
                else
                {
                    ;
                }

                fsp_err = measured_data_process(&g_bfe0_data[0]);

                if (FSP_SUCCESS != fsp_err) // Check for error return
                {
                   s_bfe_err = BMS_ERR_FSP;

                   g_bms_trans_flags.goto_fault = true; // Go to Fault state to process error.
                   break;
                }
                else
                {
                    ;
                }

                fsp_err = print_to_console((char *) (MEAS_MSG_3));

                if (FSP_SUCCESS != fsp_err) // Check for error return
                {
                   s_bfe_err = BMS_ERR_FSP;

                   g_bms_trans_flags.goto_fault = true; // Go to Fault state to process error.
                   break;
                }
                else
                {
                    ;
                }

                g_bms_trans_flags.measure = false;
            }
            else
            {
                ;
            }

            s_counter_mem_check++; // Increment watchdog timeout reset counter

            /* Memory check timeout. */
            if(s_counter_mem_check > BMS_MEMORY_CHECK)
            {
                s_counter_mem_check = 0; // Reset counter

                /* Check setup registers for corruption and reset watchdog timeout. */
                s_bfe_err = g_bfe0.p_api->memoryCheck(&g_bfe0_ctrl);

                if(s_bfe_err != BMS_SUCCESS) // Check for error return
                {
                    g_bms_trans_flags.goto_fault = true; // Go to Fault state to process error.
                    break;
                }
                else
                {
                    ;
                }

                /* Check for faults. */
                s_bfe_err = g_bfe0.p_api->faultsCheck(&g_bfe0_ctrl);

                if(s_bfe_err != BMS_SUCCESS) // Check for error return
                {
                    g_bms_trans_flags.goto_fault = true; // Go to Fault state to process error.
                    break;
                }
                else
                {
                    ;
                }
            }
            else
            {
                ;
            }

            break;
        }

        case BMS_SLEEP_STATE:
        {
            if(g_bfe0_ctrl.status.in_sleep_mode == false) // Check if already in sleep mode
            {
                /* Print banner info to console. */
                fsp_err = print_to_console((char *) (SLEEP_STATE_2));

                if (FSP_SUCCESS != fsp_err) // Check for error return
                {
                   s_bfe_err = BMS_ERR_FSP;

                   g_bms_trans_flags.goto_fault = true; // Go to Fault state to process error.
                   break;
                }
                else
                {
                    ;
                }

                /* Send sleep command to the BFE. */
                s_bfe_err = g_bfe0.p_api->sleep(&g_bfe0_ctrl);

                if(BMS_SUCCESS != s_bfe_err) // Check for error return
                {
                    g_bms_trans_flags.goto_fault = true; // Go to Fault state to process error.
                    break;
                }
                else
                {
                    ;
                }

                /* Print banner info to console. */
                fsp_err = print_to_console((char *) (SLEEP_STATE_1));

                if (FSP_SUCCESS != fsp_err) // Check for error return
                {
                   s_bfe_err = BMS_ERR_FSP;

                   g_bms_trans_flags.goto_fault = true; // Go to Fault state to process error.
                   break;
                }
                else
                {
                    ;
                }
            }
            else if(g_bms_trans_flags.wakeup == true) // Check for wake up command from terminal
            {
                /* Send a wake up command to the BFE. */
                s_bfe_err = g_bfe0.p_api->wakeUp(&g_bfe0_ctrl);

                if(s_bfe_err != BMS_SUCCESS) // Check for error return
                {
                    g_bms_trans_flags.goto_fault = true; // Go to Fault state to process error.
                    break;
                }
                else
                {
                    ;
                }

                g_bms_trans_flags.wakeup = false; // Clear wake up flag.
                g_bms_trans_flags.goto_normal = true;
            }
            else
            {
                /* Check for faults. */
                s_bfe_err = g_bfe0.p_api->faultsCheck(&g_bfe0_ctrl);

                if(s_bfe_err != BMS_SUCCESS) // Check for error return
                {
                    g_bms_trans_flags.goto_fault = true; // Go to Fault state to process error.
                    break;
                }
                else
                {
                    ;
                }
            }

            break;
        }

        case BMS_BALANCE_STATE:
        {
            if(g_do_once == true) // Send to terminal once
            {
                /* Print banner info to console. */
                fsp_err = print_to_console((char *) (BALANCING_STATE_1));

                if (FSP_SUCCESS != fsp_err) // Check for error return
                {
                   s_bfe_err = BMS_ERR_FSP;

                   g_bms_trans_flags.goto_fault = true; // Go to Fault state to process error.
                   break;
                }
                else
                {
                    ;
                }

                g_do_once = false;
            }
            else
            {
                ;
            }

            /* Begin cell balancing algorithm. */
            s_bfe_err = cell_balance();

            if(s_bfe_err != BMS_SUCCESS) // Check for error return
            {
                g_bms_trans_flags.goto_fault = true; // Go to Fault state to process error.
            }
            else
            {
                ;
            }

            break;
        }

        case BMS_FAULT_STATE:
        {
            TURN_RED_ON // Turn ON RED LED to indicate fatal error.

            if(g_do_once == true) // Send to terminal once
            {
                /* Print banner info to console. */
                fsp_err = print_to_console((char *) (FAULTS_MSG_1));

                if (FSP_SUCCESS != fsp_err) // Check for error return
                {
                   s_bfe_err = BMS_ERR_FSP;
                   g_bms_trans_flags.goto_fault = true; // Go to Fault state to process error.
                   break;
                }
                else
                {
                    ;
                }
            }
            else
            {
                ;
            }

            // Call fault management routine
            s_bfe_err = fault_management(s_bfe_err);

            break;
        }

        default:
        {
            break;
        }
    }
    return;
}
/**********************************************************************************************************************
 * End of function bms_state_machine
 *********************************************************************************************************************/

/*****************************************************************************************************************
 *  @brief      This function initializes the USB interface.
 *  @param      none
 *  @retval     FSP_SUCCESS     Successful initialization of interface.
 *  @retval     Inherit from    g_basic0.p_api routines.
 ****************************************************************************************************************/
fsp_err_t usb_interface_init(void)
{
    fsp_err_t fsp_err = FSP_SUCCESS;

    /* Open USB instance */
    fsp_err = g_basic0.p_api->open(&g_basic0_ctrl, &g_basic0_cfg);
    FSP_ERROR_RETURN(FSP_SUCCESS == fsp_err, fsp_err); // Check for errors.

    /* Get USB class type */
    fsp_err = g_basic0.p_api->classTypeGet(&g_basic0_ctrl, &g_usb_class_type);
    FSP_ERROR_RETURN(FSP_SUCCESS == fsp_err, fsp_err); // Check for errors.

    /* Get module number */
    fsp_err = g_basic0.p_api->moduleNumberGet(&g_basic0_ctrl, &g_usb_module_number);
    FSP_ERROR_RETURN(FSP_SUCCESS == fsp_err, fsp_err); // Check for errors.

    return fsp_err;
}
/**********************************************************************************************************************
 * End of function usb_interface_init
 *********************************************************************************************************************/

/*****************************************************************************************************************
 *  @brief      This function processes a USB event and checks for input data from terminal.
 *  @param      none
 *  @retval     FSP_SUCCESS     Successful USB event processing.
 *  @retval     Inherit from    g_basic0.p_api routines.
 ****************************************************************************************************************/
fsp_err_t usb_event_process(void)
{
    fsp_err_t fsp_err = FSP_SUCCESS;

    uint8_t g_buf[READ_BUF_SIZE]               = {0};

    static usb_pcdc_linecoding_t g_line_coding = {0}; // Hold settings for the virtual UART.

    g_basic0.p_api->eventGet(&g_basic0_ctrl, &usb_event);
    FSP_ERROR_RETURN(FSP_SUCCESS == fsp_err, fsp_err); // Check for errors.

    /* USB event received by eventGet */
    switch (usb_event)
    {
        case USB_STATUS_CONFIGURED:
        {
            fsp_err = g_basic0.p_api->read(&g_basic0_ctrl, g_buf, READ_BUF_SIZE, (uint8_t)g_usb_class_type);
            FSP_ERROR_RETURN(FSP_SUCCESS == fsp_err, fsp_err); // Check for errors.
            break;
        }

        case USB_STATUS_READ_COMPLETE:
        {
            if(g_usb_attach)
            {
                fsp_err = g_basic0.p_api->read(&g_basic0_ctrl, g_buf, 1, (uint8_t)g_usb_class_type);
                FSP_ERROR_RETURN(FSP_SUCCESS == fsp_err, fsp_err); // Check for errors.
            }
            else
            {
                ;
            }

            /* Switch case evaluation of user input */
            switch (g_buf[0])
            {
                case GET_VOLT_TEMP:
                {
                    g_bms_trans_flags.measure = true; // Set transition flag
                    break;
                }

                case BALANCE_CELLS:
                {
                    g_bms_trans_flags.goto_balance = true; // Set transition flag
                    g_do_once = true;
                    break;
                }

                case SLEEP:
                {
                    g_bms_trans_flags.goto_sleep = true; // Set transition flag
                    g_do_once = true;
                    break;
                }

                case MORE_INFO:
                {
                    fsp_err = print_to_console((char *) (INFO_BANNER_1));
                    FSP_ERROR_RETURN(FSP_SUCCESS == fsp_err, fsp_err); // Check for errors.
                    break;
                }

                case CARRIAGE_RETURN:
                {
                    if(bms_state == BMS_SLEEP_STATE)
                    {
                        g_bms_trans_flags.wakeup = true; // Set transition flag
                    }
                    else if(bms_state == BMS_BALANCE_STATE)
                    {
                        bal_state = BAL_INHIBIT;

                        g_bms_trans_flags.exit_balance = true; // Set transition flag
                    }
                    else if(bms_state == BMS_FAULT_STATE)
                    {
                        g_bms_trans_flags.clear_fault = true; // Set transition flag
                    }
                    else
                    {
                        g_do_once = true;
                    }

                    break;
                }

                default:
                {
                    break;
                }
            }

            break;
        }

        case USB_STATUS_REQUEST: // Receive Class Request
        {
            fsp_err = g_basic0.p_api->setupGet(&g_basic0_ctrl, &usb_setup);
            FSP_ERROR_RETURN(FSP_SUCCESS == fsp_err, fsp_err); // Check for errors.

            /* Check for the specific CDC class request IDs */
            if (USB_PCDC_SET_LINE_CODING == (usb_setup.request_type & USB_BREQUEST))
            {
                fsp_err = g_basic0.p_api->periControlDataGet(&g_basic0_ctrl,
                                                             (uint8_t *) &g_line_coding,
                                                             LINE_CODING_LENGTH );
                FSP_ERROR_RETURN(FSP_SUCCESS == fsp_err, fsp_err); // Check for errors.
            }
            else if (USB_PCDC_GET_LINE_CODING == (usb_setup.request_type & USB_BREQUEST))
            {
                fsp_err = g_basic0.p_api->periControlDataSet(&g_basic0_ctrl,
                                                             (uint8_t *) &g_line_coding,
                                                             LINE_CODING_LENGTH );
                FSP_ERROR_RETURN(FSP_SUCCESS == fsp_err, fsp_err); // Check for errors.
            }
            else if (USB_PCDC_SET_CONTROL_LINE_STATE == (usb_setup.request_type & USB_BREQUEST))
            {
                fsp_err = g_basic0.p_api->periControlStatusSet(&g_basic0_ctrl, USB_SETUP_STATUS_ACK);
                FSP_ERROR_RETURN(FSP_SUCCESS == fsp_err, fsp_err); // Check for errors.
            }
            else
            {
                ;
            }

            break;
        }
        case USB_STATUS_DETACH:
        {
            break;
        }

        case USB_STATUS_SUSPEND:
        {
            g_usb_attach = false;
            memset (g_buf, 0, sizeof(g_buf));
            break;
        }

        case USB_STATUS_RESUME:
        {
            g_usb_attach = true;
            break;
        }

        default:
        {
            break;
        }
    }

    return fsp_err;
}
/**********************************************************************************************************************
 * End of function usb_event_process
 *********************************************************************************************************************/

/*****************************************************************************************************************
 *  @brief      This function runs cell balancing algorithm
 *  @param      none
 *  @retval     BMS_SUCCESS
 *  @retval     BMS_ERR_CELL_BALANCE  Error in cell balancing algorithm
 *  @retval     Inherit from g_bfe0.p_api routines.
 ****************************************************************************************************************/
static e_bms_err_t cell_balance(void)
{
    static uint32_t  counter_balance_time  = 0;
    static uint32_t  counter_balance_loops = 0;

    fsp_err_t        fsp_err               = FSP_SUCCESS; // Error status
    e_bms_err_t      bms_err               = BMS_SUCCESS; // Error status
    uint16_t         tmp1                  = 0xFFFF;
    uint32_t         tmp2                  = 0;
    uint16_t         tmp3                  = 0;

    /* Cell balancing algorithm. */
    switch(bal_state)
    {
        case BAL_MEASURE:
        {
            counter_balance_loops++;

            /* Acquire all voltages. */
            bms_err = g_bfe0.p_api->vAllGet(&g_bfe0_ctrl, &g_bfe0_data[0]);
            BFE_ERROR_RETURN(BMS_SUCCESS == bms_err, BMS_ERR_CELL_BALANCE); // Check for errors.

            /* Check for faults. */
            bms_err = g_bfe0.p_api->faultsCheck(&g_bfe0_ctrl);
            BFE_ERROR_RETURN(BMS_SUCCESS == bms_err, BMS_ERR_CELL_BALANCE); // Check for errors.

            /* Find the cell with lowest voltage. */
            for(uint16_t i = 0; i < g_bfe0_ctrl.status.stacked_devs; i++ )
            {
                for(uint16_t j = 0; j < BFE_MAX_CELLS_PER_IC; j++ )
                {
                    tmp3 = (uint16_t) (1 << j);

                    if((g_bfe0_data[i].v_cells[j] < tmp1) && ((g_bfe0_ctrl.setup.cells_cfg[i] & tmp3) > 0))
                    {
                        tmp1 = g_bfe0_data[i].v_cells[j];
                    }
                    else
                    {
                        ;
                    }
                }
            }

            /* Finds cells that need balancing. Check for delta voltage fault threshold. */
            for(uint16_t i = 0; i < g_bfe0_ctrl.status.stacked_devs; i++ )
            {
                g_bfe0_ctrl.balance_cell_sel[i] = 0;

                for(uint16_t j = 0; j < BFE_MAX_CELLS_PER_IC; j++ )
                {
                    tmp3 = (uint16_t) (1 << j);

                    if(((g_bfe0_data[i].v_cells[j] - tmp1) > BMS_DELTA_V_MAX_F_TH)
                    && ((g_bfe0_ctrl.setup.cells_cfg[i] & tmp3) > 0))
                    {
                        return BMS_ERR_CELL_BALANCE;
                    }
                    else if(((g_bfe0_data[i].v_cells[j] - tmp1) > BMS_DELTA_V_MAX)
                         && ((g_bfe0_ctrl.setup.cells_cfg[i] & tmp3) > 0))
                    {
                        g_bfe0_ctrl.balance_cell_sel[i] |= tmp3;
                    }
                    else
                    {
                        ;
                    }
                }

                tmp2 |= g_bfe0_ctrl.balance_cell_sel[i];
            }

            /* Check if cell balancing is needed. */
            if((tmp2 == 0)
            || (g_bms_trans_flags.exit_balance == true)
            || (counter_balance_loops > BMS_CB_LOOPS_MAX))
            {
                /* Print banner info to console. */
                fsp_err = print_to_console((char *) (BALANCING_STATE_2));
                BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_FSP); // Check for errors.

                /* Trigger a measurement after entering Normal operating state. */
                g_bms_trans_flags.measure = true;

                g_bms_trans_flags.exit_balance = false;
                g_bms_trans_flags.goto_normal = true;
                counter_balance_loops = 0;

                bal_state = BAL_MEASURE;
            }
            else
            {
                bal_state = BAL_MASK1_APPLY;
            }

            break;
        }

        case BAL_MASK1_APPLY:
        {
            counter_balance_time = 0; // Reset counter

            /* Enable cell balancing for odd cells. */
            g_bfe0_ctrl.bal_pattern = BFE_REG_BAL_MASK_ODD_CELL;

            bms_err = g_bfe0.p_api->balanceControl(&g_bfe0_ctrl, BFE_CELL_BALANCE_ENABLE);
            BFE_ERROR_RETURN(BMS_SUCCESS == bms_err, BMS_ERR_CELL_BALANCE); // Check for errors.

            /* Print banner info to console. */
            fsp_err = print_to_console((char *) (BALANCING_STATE_3));
            BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_FSP); // Check for errors.

            bal_state = BAL_ON_TIMER1;

            break;
        }

        case BAL_ON_TIMER1:
        {
            counter_balance_time++; // Increment balancing activity counter

            /* Check if balancing activity interval has passed. */
            if(counter_balance_time > BMS_CB_ON_TIMER)
            {
                bal_state = BAL_MASK2_APPLY;
            }
            else
            {
                ;
            }

            break;
        }

        case BAL_MASK2_APPLY:
        {

            counter_balance_time = 0; // Reset counter

            /* Enable cell balancing on even cells. */
            g_bfe0_ctrl.bal_pattern = BFE_REG_BAL_MASK_EVEN_CELL;

            bms_err = g_bfe0.p_api->balanceControl(&g_bfe0_ctrl, BFE_CELL_BALANCE_ENABLE);
            BFE_ERROR_RETURN(BMS_SUCCESS == bms_err, BMS_ERR_CELL_BALANCE); // Check for errors.

            /* Print banner info to console. */
            fsp_err = print_to_console((char *) (BALANCING_STATE_3));
            BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_FSP); // Check for errors.

            bal_state = BAL_ON_TIMER2;
            break;
        }

        case BAL_ON_TIMER2:
        {
            counter_balance_time++; // Increment balancing activity counter

            /* Check if balancing activity interval has passed. */
            if(counter_balance_time > BMS_CB_ON_TIMER)
            {
                bal_state = BAL_INHIBIT;
            }
            else
            {
                ;
            }
            break;
        }

        case BAL_INHIBIT:
        {
            /* Inhibit cell balancing. */
            g_bfe0_ctrl.bal_pattern = BFE_REG_BAL_MASK_NO_CELL;

            bms_err = g_bfe0.p_api->balanceControl(&g_bfe0_ctrl, BFE_CELL_BALANCE_INHIBIT);
            BFE_ERROR_RETURN(BMS_SUCCESS == bms_err, BMS_ERR_CELL_BALANCE); // Check for errors.

            counter_balance_time = 0; // Reset counter

            bal_state = BAL_OFF_TIMER;
            break;
        }

        case BAL_OFF_TIMER:
        {
            counter_balance_time++; // Increment balancing activity counter

            /* Check if balancing activity interval has passed. */
            if(counter_balance_time > BMS_CB_OFF_TIMER)
            {
                bal_state = BAL_MEASURE;
            }
            else
            {
                ;
            }

            break;
        }

        default:
        {
            break;
        }
    }

    return bms_err;
}
/**********************************************************************************************************************
 * End of function cells_balance
 *********************************************************************************************************************/

/*****************************************************************************************************************
 *  @brief      This function manages faults.
 *  @param[in]  bms_err_in            Detected bms error.
 *  @retval     BMS_SUCCESS           The fault was successfully managed.
 *  @retval     BMS_ERR_COMM_ERR_CLR  Error in fault clearing.
 *  @retval     Inherit from g_bfe0.p_api routines.
 ****************************************************************************************************************/
static e_bms_err_t fault_management(e_bms_err_t bms_err_in)
{
    fsp_err_t        fsp_err               = FSP_SUCCESS; // Error status
    e_bms_err_t      bms_err               = BMS_SUCCESS; // Error status

    volatile bool    flt_clear             = false;

    uint16_t         dev_addr              = 0;
    uint32_t         reg_addr              = 0;
    uint16_t         reg_data              = 0;

    FSP_PARAMETER_NOT_USED(dev_addr); // Mute unused parameter
    FSP_PARAMETER_NOT_USED(reg_addr); // Mute unused parameter
    FSP_PARAMETER_NOT_USED(reg_data); // Mute unused parameter

    if(bms_err_in == BMS_FAULT) // Fault pin assertion.
    {
        if(g_do_once == true) // Send to terminal once
        {

            /* Read the fault. */
            bms_err = g_bfe0.p_api->faultsAllRead(&g_bfe0_ctrl, &g_bfe0_faults[0]);
            BFE_ERROR_RETURN(BMS_SUCCESS == bms_err, bms_err); // Check for errors.

            faults_data_process(&g_bfe0_faults[0]);
            g_do_once = false;
        }
        else
        {
            ;
        }

        /* Check each device from stack. */
        for(uint32_t i = 0; i < g_bfe0_ctrl.status.stacked_devs; i++)
        {
            /* Oscillator fault processing. */
            if(g_bfe0_faults[i].flt_oscillator == true)
            {
                if(g_bms_trans_flags.clear_fault == true)
                {
                    /* Wake up any sleeping device from stack. */
                    bms_err = g_bfe0.p_api->randomWakeUp(&g_bfe0_ctrl);
                    BFE_ERROR_RETURN(BMS_SUCCESS == bms_err, BMS_ERR_COMM_ERR_CLR); // Check for errors.

                    flt_clear = true;
                }
                else
                {
                    ;
                }
            }
            else
            {
                ;
            }

            /* Watchdog timeout fault processing. */
            if(g_bfe0_faults[i].flt_wdt_timout == true)
            {
                if(g_bms_trans_flags.clear_fault == true)
                {

                    /* Add custom code. */
                }
                else
                {
                    ;
                }
            }
            else
            {
                ;
            }

            /* Cell overvoltage processing. */
            if(g_bfe0_faults[i].flt_overvolt > ZERO)
            {
                if(g_bms_trans_flags.clear_fault == true)
                {
                    flt_clear = true;

                    /* Add custom code. */
                }
                else
                {
                    ;
                }
            }
            else
            {
                ;
            }

            /* Cell undervoltage processing. */
            if(g_bfe0_faults[i].flt_undervolt > ZERO)
            {
                if(g_bms_trans_flags.clear_fault == true)
                {
                    flt_clear = true;

                    /* Add custom code. */
                }
                else
                {
                    ;
                }
            }
            else
            {
                ;
            }

            /* Over-temperature processing. */
            if(g_bfe0_faults[i].flt_over_temp > ZERO)
            {
                if(g_bms_trans_flags.clear_fault == true)
                {
                    flt_clear = true;

                    /* Add custom code. */
                }
                else
                {
                    ;
                }
            }
            else
            {
                ;
            }

            /* Open wire fault processing on cells, battery or Vss connection. */
            if((g_bfe0_faults[i].flt_ow_vbat == true)
            || (g_bfe0_faults[i].flt_ow_vss == true)
            || (g_bfe0_faults[i].flt_open_wire > ZERO))
            {

                /* Add custom code. */
            }
            else
            {
                ;
            }

            /* Register checksum (parity) error processing. */
            if(g_bfe0_faults[i].flt_parity == true)
            {
                if(g_bms_trans_flags.clear_fault == true)
                {
                    /* Rewrite all setup registers. */
                    bms_err = g_bfe0.p_api->setup(&g_bfe0_ctrl);
                    BFE_ERROR_RETURN(BMS_SUCCESS == bms_err, BMS_ERR_COMM_ERR_CLR); // Check for errors.

                    /* Check memory again. */
                    bms_err = g_bfe0.p_api->memoryCheck(&g_bfe0_ctrl);
                    BFE_ERROR_RETURN(BMS_SUCCESS == bms_err, BMS_ERR_COMM_ERR_CLR); // Check for errors.

                    /* Check for faults. */
                    bms_err = g_bfe0.p_api->faultsCheck(&g_bfe0_ctrl);
                    BFE_ERROR_RETURN(BMS_SUCCESS == bms_err, BMS_ERR_COMM_ERR_CLR); // Check for errors.

                    flt_clear = true;
                }
                else
                {
                    ;
                }
            }
            else
            {
                ;
            }

            /* Reference voltage fault processing. */
            if(g_bfe0_faults[i].flt_v_ref == true)
            {

                /* Add custom code. */
            }
            else
            {
                ;
            }

            /* Regulator fault processing. */
            if(g_bfe0_faults[i].flt_v_reg == true)
            {

                /* Add custom code. */
            }
            else
            {
                ;
            }

            /* Multiplexer temperature error processing. */
            if(g_bfe0_faults[i].flt_temp_mux == true)
            {

                /* Add custom code. */
            }
            else
            {
                ;
            }
        }
    }
#if BFE_CFG_STA_DEV > 1U // Daisy chain operation

    /* Process communication failure, incorrect CRC or NAC response. */
    else if((bms_err_in == BMS_ERR_COMM)
         || (bms_err_in == BMS_ERR_NAK)
         || (bms_err_in == BMS_ERR_ACK)
         || (bms_err_in == BMS_ERR_CRC_INCORRECT)
         || (bms_err_in == BMS_ERR_DRDY_TIMEOUT))
    {
        if(g_do_once == true) // Send to terminal once
        {
            /* Print banner info to console. */
            fsp_err = print_to_console((char *) (FAULTS_MSG_2));
            if (FSP_SUCCESS != fsp_err)
            {
                APP_ERR_TRAP(fsp_err);
            }

            g_do_once = false;
        }
        else
        {
            ;
        }

        if(g_bms_trans_flags.clear_fault == true)
        {
            /* Reset flipped input condition of daisy chain port. Send FB FF FF FF data sequence. */
            dev_addr = BFE_DAISY_CHAIN_ADDRESS_ALL;
            reg_addr = BFE_INPUT_RESET;
            reg_data = BFE_MAX_14_BIT_DAT_VAL;

            bms_err = g_bfe0.p_api->singleRegAccess(&g_bfe0_ctrl, &dev_addr, &reg_addr, &reg_data, BFE_WRITE_REG);

            /* Check for NAK or Communication failure. After FB FF FF FF data sequence, NAK response is expected. */
            if((bms_err != BMS_ERR_NAK) && (bms_err != BMS_ERR_COMM))
            {
                return BMS_ERR_COMM_ERR_CLR;
            }
            else
            {
                dev_addr = (e_bfe_dev_addr_t) (g_bfe0_ctrl.status.stacked_devs); // Address top stack device.
                reg_addr = BFE_REG_ACK;
                reg_data = ZERO;

                /* Send ACK to top device to test vertical communication. */
                bms_err = g_bfe0.p_api->singleRegAccess(&g_bfe0_ctrl, &dev_addr, &reg_addr, &reg_data, BFE_READ_REG);

                /* Examine response. */
                if((bms_err != BMS_SUCCESS) || (reg_addr != (BFE_REG_ACK & BFE_REG_PAGE_ADDR_MASK)))
                {
                    /* Try to wake up any sleeping devices in the stack. */
                    bms_err = g_bfe0.p_api->randomWakeUp(&g_bfe0_ctrl);
                    BFE_ERROR_RETURN(BMS_SUCCESS == bms_err, BMS_ERR_COMM_ERR_CLR); // Check for errors.
                }
            }

            flt_clear = true;
        }
        else
        {
            ;
        }
    }
#endif /* BFE_CFG_DCH_OPERATION */

    /* Process error EEPROM checksum, calculated on BFE start-up. */
    else if(bms_err_in == BFE_ERR_EEPROM)
    {
        if(g_do_once == true) // Send to terminal once
        {
            /* Print banner info to console. */
            fsp_err = print_to_console((char *) (FAULTS_MSG_3));
            BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_FSP); // Check for errors.

            g_do_once = false;
        }
        else
        {
            ;
        }

        /* Reset the BFE to trigger shadow reload of registers that seem to be corrupted. */
        if(g_bms_trans_flags.clear_fault == true)
        {
            /* Reset and reinitialize the BFE. */
            bms_err = g_bfe0.p_api->reset(&g_bfe0_ctrl);
            BFE_ERROR_RETURN(BMS_SUCCESS == bms_err, BMS_ERR_COMM_ERR_CLR); // Check for errors.

            /* Write user data. */
            bms_err = g_bfe0.p_api->userRegsAccess(&g_bfe0_ctrl, &g_bfe0_user_data[0], BFE_WRITE_REG);
            BFE_ERROR_RETURN(BMS_SUCCESS == bms_err, BMS_ERR_COMM_ERR_CLR); // Check for errors.

            flt_clear = true;
        }
        else
        {
            ;
        }
    }
    else
    {
        if(g_do_once == true) // Send to terminal once
        {
            /* Print banner info to console. */
            fsp_err = print_to_console((char *) (FAULTS_MSG_4));
            BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_FSP); // Check for errors.

            g_do_once = false;
        }
        else
        {
            ;
        }

        /* Add custom code. */
    }

    /* Clear faults and exit fault state or halt if not possible. */
    if(g_bms_trans_flags.clear_fault == true)
    {
        if(flt_clear == true)
        {
            /* Clear all faults. */
            bms_err = g_bfe0.p_api->faultsAllClear(&g_bfe0_ctrl);
            BFE_ERROR_RETURN(BMS_SUCCESS == bms_err, BMS_ERR_COMM_ERR_CLR); // Check for errors.

            /* Clean data structures. */
            memset(&g_bfe0_faults[0], ZERO, sizeof(g_bfe0_faults));

            TURN_RED_OFF // Turn off the red LED

            g_do_once = true;

            g_bms_trans_flags.goto_normal = true; // Set transition flag.
        }
        else
        {
            /* Print banner info to console. */
            fsp_err = print_to_console((char *) (FAULTS_MSG_5));
            BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_COMM_ERR_CLR); // Check for errors.

            bms_err = bms_err_in;

            /* Halt */
            APP_ERR_TRAP(bms_err_in)
        }

        g_bms_trans_flags.clear_fault = false;
    }
    else
    {
        bms_err = bms_err_in;
    }

    return bms_err;
}
/**********************************************************************************************************************
 * End of function fault_management
 *********************************************************************************************************************/

/*****************************************************************************************************************
 *  @brief      Prints the message to console
 *  @param[in]  p_msg        Contains address of buffer to be printed.
 *  @retval     FSP_SUCCESS  The data is successfully sent to terminal.
 *  @retval     Inherit from g_basic0.p_api routines.
 ****************************************************************************************************************/
static fsp_err_t print_to_console(char * p_data)
{
    fsp_err_t err = FSP_SUCCESS;
    uint32_t  len = (uint32_t) strlen(p_data);

    /* Send data by USB. */
    err = g_basic0.p_api->write(&g_basic0_ctrl, (uint8_t*) p_data, len, (uint8_t) g_usb_class_type);
    FSP_ERROR_RETURN(FSP_SUCCESS == err, err); // Check for errors.

    /* Check if USB write has completed. */
    err = check_for_write_complete();
    FSP_ERROR_RETURN(FSP_SUCCESS == err, err); // Check for errors.

    return err;
}
/**********************************************************************************************************************
 * End of function print_to_console
 *********************************************************************************************************************/

/*****************************************************************************************************************
 *  @brief      Process measured data.
 *  @param[in]  p_data      Pointer to measured data structure.
 *  @retval     FSP_SUCCESS The input data was successfully sent to terminal.
 *  @retval     Inherit from print_to_console().
 ****************************************************************************************************************/
static fsp_err_t measured_data_process(st_bfe_meas_t * p_data)
{
    fsp_err_t err = FSP_SUCCESS;

    uint16_t buffer_index_count = 0x0000;
    float v_batt_f              = 0;
    float v_cell_f              = 0;
    float v_temp_f              = 0;

    uint32_t tmp1 = 0;

    /* Clear kit info buffer before updating data. */
    memset(g_print_data_buff, '\0', USB_LONG_PACKET_SIZE - 1U);

    /* Calculate current data filled length. */
    buffer_index_count = ((uint16_t) (strlen(g_print_data_buff)));

    /* Calculate the total battery pack voltage. */
    for(uint16_t i = 0; i < g_bfe0_ctrl.status.stacked_devs; i++ )
    {
        tmp1 += (p_data + i)->v_batt;
    }

    /* Convert ADC battery voltage reading to Volts. */
    v_batt_f = ADCVBATT_AS_V(tmp1);

    /* Append the data from current buffer_index_count. */
    sprintf( (char *) &g_print_data_buff[buffer_index_count],
             "%s%d.%02d \r\n",
             MEAS_MSG_1,
             FL_INT(v_batt_f),
             FL_FRAC(v_batt_f));

    /* Update index count. */
    buffer_index_count = ((uint16_t) (strlen(g_print_data_buff)));

    /* Append the data from current buffer_index_count. */
    sprintf( (char *) &g_print_data_buff[buffer_index_count], "%s", TABLE_HEADER_CVOLTS);

    /* Append data for every device in stack. */
    for(uint16_t i = 0; i < g_bfe0_ctrl.status.stacked_devs; i++ )
    {
        /* Update index count. */
        buffer_index_count = ((uint16_t) (strlen(g_print_data_buff)));

        /* appends the data from current buffer_index_count */
        sprintf((char *) &g_print_data_buff[buffer_index_count], "\r\n #%02d |", i + 1);

        /* Append data for every cell input of the current device. */
        for(uint16_t j = 0; j < BFE_MAX_CELLS_PER_IC; j++ )
        {
            /* Convert ADC cell voltage reading to Volts. */
            v_cell_f = ADCVCELL_AS_V((p_data + i)->v_cells[j]);

            /* Update index count. */
            buffer_index_count = ((uint16_t) (strlen(g_print_data_buff)));

            /* Append the data from current buffer_index_count. */
            sprintf((char *) &g_print_data_buff[buffer_index_count], " %d.%02d |", FL_INT(v_cell_f), FL_FRAC(v_cell_f));
        }

        /* Send data to terminal. */
        err = print_to_console(g_print_data_buff);
        FSP_ERROR_RETURN(FSP_SUCCESS == err, err); // Check for errors.

        /* Clear kit info buffer before updating data. */
        memset(g_print_data_buff, '\0', USB_LONG_PACKET_SIZE - 1U);

    }

    /* Update index count. */
    buffer_index_count = ((uint16_t) (strlen(g_print_data_buff)));

    /* Append the data from current buffer_index_count. */
    sprintf((char *) &g_print_data_buff[buffer_index_count], "\r\n %s", TABLE_HEADER_TEMPS);

    /* Append data for every device in stack. */
    for(uint16_t i = 0; i < g_bfe0_ctrl.status.stacked_devs; i++ )
    {
        /* Convert ADC temperature reading to deg. C. */
        v_temp_f = ADCVTEMP_AS_V((p_data + i)->v_ic_temp);

        /* Update index count. */
        buffer_index_count = ((uint16_t) (strlen(g_print_data_buff)));

        /* Append the data from current buffer_index_count. */
        sprintf((char *) &g_print_data_buff[buffer_index_count],
                "\r\n #%02d |              %d.%02d |",
                i + 1,
                FL_INT(v_temp_f), FL_FRAC(v_temp_f));

        /* Append data for every temperature input of the current device. */
        for(uint16_t j = 0; j < BFE_TEMP_INPUTS; j++ )
        {
            /* Convert ADC temperature reading to Volts. */
            v_temp_f = ADCVTEMP_AS_V(p_data->v_ext_temps[0]);

            /* Update index count. */
            buffer_index_count = ((uint16_t) (strlen(g_print_data_buff)));

            /* Append the data from current buffer_index_count. */
            sprintf((char *) &g_print_data_buff[buffer_index_count], " %d.%02d |", FL_INT(v_temp_f), FL_FRAC(v_temp_f));
        }

    	/* Send data to terminal. */
    	err = print_to_console(g_print_data_buff);
    	FSP_ERROR_RETURN(FSP_SUCCESS == err, err); // Check for errors.

    	/* Clear kit info buffer before updating data. */
    	memset(g_print_data_buff, '\0', USB_LONG_PACKET_SIZE - 1U);
    }

    return err;
}
/**********************************************************************************************************************
* End of function measured_data_process
**********************************************************************************************************************/

/*****************************************************************************************************************
 *  @brief      Process measured data
 *  @param[in]  p_faults    Pointer to faults data structure.
 *  @retval     FSP_SUCCESS The faults were successfully sent to terminal.
 *  @retval     @retval     Inherit from print_to_console().
 ****************************************************************************************************************/
static fsp_err_t faults_data_process(st_bfe_faults_t * p_faults)
{
    fsp_err_t err = FSP_SUCCESS;

    uint16_t  buffer_index_count = 0x0000;

    /* Clear kit info buffer before updating data. */
    memset(g_print_data_buff, '\0', USB_LONG_PACKET_SIZE - 1U);

    /* Check each device from stack. */
    for(uint16_t i = 0; i < g_bfe0_ctrl.status.stacked_devs; i++)
    {

        /* Update index count. */
        buffer_index_count = ((uint16_t)(strlen(g_print_data_buff)));

        /* Append the data from current buffer_index_count. */
        sprintf((char *)&g_print_data_buff[buffer_index_count], "\r\n\r\n Device: #%02d fault condition:", i + 1);

        /* Oscillator fault processing. */
        if((p_faults + i)->flt_oscillator == true)
        {

            /* Update index count. */
            buffer_index_count = ((uint16_t)(strlen(g_print_data_buff)));

            /* Append the data from current buffer_index_count. */
            sprintf((char *)&g_print_data_buff[buffer_index_count], "\r\n  Oscillator!");
        }
        else
        {
            ;
        }

        /* Watchdog timeout fault processing. */
        if((p_faults + i)->flt_wdt_timout == true)
        {

            /* Update index count. */
            buffer_index_count = ((uint16_t)(strlen(g_print_data_buff)));

            /* Append the data from current buffer_index_count. */
            sprintf((char *)&g_print_data_buff[buffer_index_count], "\r\n  Watchdog timeout!");
        }
        else
        {
            ;
        }

        /* Cell overvoltage processing. */
        if((p_faults + i)->flt_overvolt > ZERO)
        {
            /* Update index count. */
            buffer_index_count = ((uint16_t)(strlen(g_print_data_buff)));

            /* Append the data from current buffer_index_count. */
            sprintf((char *)&g_print_data_buff[buffer_index_count], "\r\n  Cell overvoltage!");
        }
        else
        {
            ;
        }

        /* Cell undervoltage processing. */
        if((p_faults + i)->flt_undervolt > ZERO)
        {
            /* Update index count. */
            buffer_index_count = ((uint16_t)(strlen(g_print_data_buff)));

            /* Append the data from current buffer_index_count. */
            sprintf((char *)&g_print_data_buff[buffer_index_count], "\r\n  Cell undervoltage!");
        }
        else
        {
            ;
        }

        /* Over-temperature processing. */
        if((p_faults + i)->flt_over_temp > ZERO)
        {
            /* Update index count. */
            buffer_index_count = ((uint16_t)(strlen(g_print_data_buff)));

            /* Append the data from current buffer_index_count. */
            sprintf((char *)&g_print_data_buff[buffer_index_count], "\r\n  Over temperature!");
        }
        else
        {
            ;
        }

        /* Open wire fault processing. Cell, battery or Vss connection. */
        if((p_faults + i)->flt_ow_vbat == true)
        {
            /* Update index count. */
            buffer_index_count = ((uint16_t)(strlen(g_print_data_buff)));

            /* Append the data from current buffer_index_count. */
            sprintf((char *)&g_print_data_buff[buffer_index_count], "\r\n  Open wire on vbat connection!");
        }
        else
        {
            ;
        }

        if((p_faults + i)->flt_ow_vss == true)
        {
            /* Update index count. */
            buffer_index_count = ((uint16_t)(strlen(g_print_data_buff)));

            /* Append the data from current buffer_index_count. */
            sprintf((char *)&g_print_data_buff[buffer_index_count], "\r\n  Open wire on vss connection!");
        }
        else
        {
            ;
        }

        if((p_faults + i)->flt_open_wire > ZERO)
        {
            /* Update index count. */
            buffer_index_count = ((uint16_t)(strlen(g_print_data_buff)));

            /* Append the data from current buffer_index_count. */
            sprintf((char *)&g_print_data_buff[buffer_index_count], "\r\n  Open wire on cells!");
        }
        else
        {
            ;
        }

        /* Register checksum (parity) error processing. */
        if((p_faults + i)->flt_parity == true)
        {
            /* Update index count. */
            buffer_index_count = ((uint16_t)(strlen(g_print_data_buff)));

            /* Append the data from current buffer_index_count. */
            sprintf((char *)&g_print_data_buff[buffer_index_count], "\r\n  Parity error!");
        }
        else
        {
            ;
        }

        /* Reference voltage fault processing. */
        if((p_faults + i)->flt_v_ref == true)
        {
            /* Update index count. */
            buffer_index_count = ((uint16_t)(strlen(g_print_data_buff)));

            /* Append the data from current buffer_index_count. */
            sprintf((char *)&g_print_data_buff[buffer_index_count], "\r\n  Reference voltage!");
        }
        else
        {
            ;
        }

        /* Regulator fault processing. */
        if((p_faults + i)->flt_v_reg == true)
        {

            /* Update index count. */
            buffer_index_count = ((uint16_t)(strlen(g_print_data_buff)));

            /* Append the data from current buffer_index_count. */
            sprintf((char *)&g_print_data_buff[buffer_index_count], "\r\n  Voltage regulator!");
        }
        else
        {
            ;
        }

        /* Multiplexer temperature error processing. */
        if((p_faults + i)->flt_temp_mux == true)
        {
            /* Update index count. */
            buffer_index_count = ((uint16_t)(strlen(g_print_data_buff)));

            /* Append the data from current buffer_index_count. */
            sprintf((char *)&g_print_data_buff[buffer_index_count], "\r\n  Temperature multiplexer error!");
        }
        else
        {
            ;
        }
    }

    /* Update index count. */
    buffer_index_count = ((uint16_t) (strlen(g_print_data_buff)));

    /* Append the data from current buffer_index_count. */
    sprintf((char *)&g_print_data_buff[buffer_index_count], "\r\n\r\n Press 'Enter' to clear all faults..");

    /* Print acquired data to console. */
    err = print_to_console(g_print_data_buff);
    FSP_ERROR_RETURN(FSP_SUCCESS == err, err); // Check for errors.

    return err;
}
/**********************************************************************************************************************
* End of function faults_data_process
**********************************************************************************************************************/

/*****************************************************************************************************************
 *  @brief      Check for write completion
 *  @param[in]  None
 *  @retval     FSP_SUCCESS     USB write has successfully completed.
 *  @retval     Inherit from g_basic0.p_api routines.
 ****************************************************************************************************************/
static fsp_err_t check_for_write_complete(void)
{
    fsp_err_t    err             = FSP_SUCCESS;
    usb_status_t usb_write_event = USB_STATUS_NONE;
    int32_t      timeout_count   = UINT16_MAX;

    do
    {
        err = g_basic0.p_api->eventGet(&g_basic0_ctrl, &usb_write_event);
        FSP_ERROR_RETURN(FSP_SUCCESS == err, err); // Check for errors.

        --timeout_count;

        if (0 > timeout_count)
        {
            timeout_count = 0;
            err = (fsp_err_t) USB_STATUS_NONE;
            break;
        }
    }
    while(USB_STATUS_WRITE_COMPLETE != usb_write_event);

    return err;
}
