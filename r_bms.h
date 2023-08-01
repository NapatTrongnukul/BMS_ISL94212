#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* BFE related headers */
#include "bal_data.h"
#include "common_utils.h"

#ifndef BMS_H_
#define BMS_H_

/* Memory check interval (# of cycles). */
#define BMS_MEMORY_CHECK            (10U)    // Every 10 cycles (approx. 1 second)

/* Max cell delta voltage threshold. Cells having higher voltage difference compared to the one with lowest voltage
 * will be balanced. */
#define BMS_DELTA_V_MAX             (0x00F0) // (0 = 0V; 8191 = 5V) Set to 20 mV

/* Max cell delta voltage fault threshold. If any cell has higher voltage difference compared to the one
 * with lowest voltage the balancing activity is stopped. */
#define BMS_DELTA_V_MAX_F_TH        (0x0333) // (0 = 0V; 8191 = 5V) Set to 500 mV.

/* Total cell balancing cycles  */
#define BMS_CB_LOOPS_MAX            (86U)    // Approximately 60 min total balancing time limit

/* Time interval for cell balancing activity. Two such balancing activities per cycle. */
#define BMS_CB_ON_TIMER             (200U)   // Approximately 20 s

/* Cell balancing off time interval per cycle. Needed for voltage relaxation before measurement. */
#define BMS_CB_OFF_TIMER            (10U)    // Approximately 1 s

/* Call state machine function period */
#define TIME_PERIOD_MS_PERIODIC     (100U)   // 100ms

#define LINE_CODING_LENGTH (0x07U)

/* Convert ADC data macros */
#define ADCVBATT_AS_V(a)            (((((float) a) * (39.8376962f)) / 8192.0f))
#define ADCVCELL_AS_V(a)            (((((float) a) * (5.0f)) / 8192.0f))
#define ADCVTEMP_AS_V(a)            (((((float) a) * (2.5f)) / 16384.0f))

/* Convert float numbers macros */
#define FL_INT(a)                   (((int) (a)))
#define FL_FRAC(a)                  (((int) ((a - (float) ((int) (a))) * 100.0f)))

/* USB communication buffers */
#define READ_BUF_SIZE               (8U)
#define USB_LONG_PACKET_SIZE        (512U)

/* User interface menu positions */
#define GET_VOLT_TEMP               ('1')
#define BALANCE_CELLS               ('2')
#define SLEEP                       ('3')
#define MORE_INFO                   ('4')
#define CARRIAGE_RETURN             ('\r')

/* Banners */
#define MAIN_MENU           "\r\n\r\n Welcome to ISL94212 sample project for BFE library v.1.1!\r\n" \
                            "\r\n Press 1 for voltage and temperature reading.\r\n" \
                            "\r\n Press 2 for starting cell balancing.\r\n" \
                            "\r\n Press 3 for entering sleep mode.\r\n" \
                            "\r\n Press 4 for more info.\r\n"

#define SLEEP_STATE_1       "\r\n\r\n The Battery Front End is in sleep mode!" \
                            "\r\n\r\n Press 'Enter' to exit!\r\n"

#define SLEEP_STATE_2       "\r\n\r\n 3. SLEEP MODE \r\n "

#define BALANCING_STATE_1   "\r\n\r\n 2. CELL BALANCING " \
                            "\r\n\r\nPress 'Enter' to terminate and exit!" \
                            "\r\n\r\nIn progress.."

#define BALANCING_STATE_2   "\r\n\r\n Cell balancing is completed!"
#define BALANCING_STATE_3   "*"

#define INFO_BANNER_1       "\r\n\r\n 4. MORE INFO \r\n" \
                            "\r\nVisit the following URLs to learn about the setup, " \
                            "the Battery Front End ICs and the RA family of MCUs, download tools " \
                            "and documentation, and get support:\r\n" \
                            "\r\n a) ISL94212EVKIT1Z resources: \t renesas.com/isl94212evkit1z" \
                            "\r\n b) EK-RA2A1 resources: \t renesas.com/ek-ra2a1" \
                            "\r\n c) BFE product information:\t renesas.com/isl94212" \
                            "\r\n d) Renesas support: \t\t renesas.com/support" \
                            "\r\n\r\n Press 'Enter' to return to menu...\r\n"

#define FAULTS_MSG_1        "\r\n\r\n FAULT DETECTED! \r\n "

#define FAULTS_MSG_2        "\r\n\r\n Communication failure! " \
                            "\r\n \r\n Press 'Enter' to clear all faults.."

#define FAULTS_MSG_3        "\r\n\r\n EEPROM error! " \
                            "\r\n\r\n Press 'Enter' to clear all faults.."

#define FAULTS_MSG_4        "\r\n\r\n Cannot process this error! "
#define FAULTS_MSG_5        "\r\n Cannot clear the faults! Please, restart! "

#define MEAS_MSG_1          "\r\n\r\n Battery pack voltage [V]: "
#define MEAS_MSG_2          "\r\n\r\n 1. VOLTAGE AND TEMPERATURE READINGS \r\n "
#define MEAS_MSG_3          "\r\n \r\n Press 'Enter' to continue.."

#define TABLE_HEADER_TEMPS  "\r\n     | Int temp [deg. C] |  External temp input [V]  |" \
                            "\r\n Dev |                   |  #1  |  #2  |  #3  |  #4  |"

#define TABLE_HEADER_CVOLTS "     |                                 Cell voltage [V]" \
                            "                                  |" \
                            "\r\n Dev |  #1  |  #2  |  #3  |  #4  |  #5  |  #6  |  #7" \
                            "  |  #8  |  #9  |  #10 |  #11 |  #12 |"


typedef enum
{
    BMS_INIT_STATE = 0,
    BMS_NORMAL_STATE,
    BMS_SLEEP_STATE,
    BMS_BALANCE_STATE,
    BMS_FAULT_STATE
} e_bms_machine_states_t;

typedef enum
{
    BAL_MEASURE = 0,
    BAL_MASK1_APPLY,
    BAL_ON_TIMER1,
    BAL_MASK2_APPLY,
    BAL_ON_TIMER2,
    BAL_INHIBIT,
    BAL_OFF_TIMER
} e_bms_balance_states_t;

typedef struct
{
    bool goto_normal;
    bool goto_sleep;
    bool goto_balance;
    bool exit_balance;
    bool goto_fault;
    bool clear_fault;
    bool wakeup;
    bool measure;
} st_bms_trans_flags_t;


void r_bms_state_machine(void);


fsp_err_t usb_interface_init (void);


fsp_err_t usb_event_process (void);

#endif /* BMS_H_ */
