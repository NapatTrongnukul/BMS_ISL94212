#include <stdint.h>
#include <stdbool.h>
#include "bal_data.h"

st_bfe_ctrl_t g_bfe0_ctrl =
{
    .bal_pattern        = ( BFE_REG_MASK_CELL_1
            | BFE_REG_MASK_CELL_2
            | BFE_REG_MASK_CELL_3
            | BFE_REG_MASK_CELL_4
            | BFE_REG_MASK_CELL_5
            | BFE_REG_MASK_CELL_6
            | BFE_REG_MASK_CELL_7
            | BFE_REG_MASK_CELL_8
            | BFE_REG_MASK_CELL_9
            | BFE_REG_MASK_CELL_10
            | BFE_REG_MASK_CELL_11
            | BFE_REG_MASK_CELL_12),

    .balance_cell_sel   =
    {
     ( BFE_REG_MASK_CELL_1
             | BFE_REG_MASK_CELL_2
                 | BFE_REG_MASK_CELL_3
                 | BFE_REG_MASK_CELL_4
                 | BFE_REG_MASK_CELL_5
                 | BFE_REG_MASK_CELL_6
                 | BFE_REG_MASK_CELL_7
                 | BFE_REG_MASK_CELL_8
                 | BFE_REG_MASK_CELL_9
                 | BFE_REG_MASK_CELL_10
                 | BFE_REG_MASK_CELL_11
                 | BFE_REG_MASK_CELL_12),
         0x0001,

    },

#if BFE_CFG_STA_DEV > 1U // Daisy chain operation
    .wd_time            = BFE_WDT_DSBL,
#else // Stand-alone operation
    .wd_time            = BFE_WDT_DSBL,
#endif // BFE_CFG_DCH_OPERATION

    .status             =
    {
         .initialized       = BFE_UNINITILIZED,
         .in_sleep_mode     = false,
         .balancing         = false,
         .stacked_devs      = 0,
         .identify_ctr      = 0,
         .serial_number[0]  = 0,
         .serial_number[1]  = 0,
    },

    .setup =
    {
         .overvoltage_limit  = BFE_CFG_OVP_LIMIT,
         .undervoltage_limit = BFE_CFG_UVP_LIMIT,
         .ext_temp_limit     = BFE_CFG_ETP_LIMIT,

         .i_wire_scan        = BFE_WIRE_I_SCAN_1M,

         .flt_tot_samples    = BFE_TOT_4_SMPL,
         .balance_mode       = BFE_BALANCE_MODE_MANUAL,

         /* Configure cell inputs for each device. */
         .cells_cfg          = {
                                    /* Device 1 */
                                    ( BFE_REG_MASK_CELL_1
                                    | BFE_REG_MASK_CELL_2
                                    | BFE_REG_MASK_CELL_3
                                    | BFE_REG_MASK_CELL_4
                                    | BFE_REG_MASK_CELL_5
                                    | BFE_REG_MASK_CELL_6
                                    | BFE_REG_MASK_CELL_7
                                    | BFE_REG_MASK_CELL_8
                                    | BFE_REG_MASK_CELL_9
                                    | BFE_REG_MASK_CELL_10
                                    | BFE_REG_MASK_CELL_11
                                    | BFE_REG_MASK_CELL_12),

                                    /* Device 2 */
                                    ( BFE_REG_MASK_CELL_1
                                    | BFE_REG_MASK_CELL_2
                                    | BFE_REG_MASK_CELL_3
                                    | BFE_REG_MASK_CELL_4
                                    | BFE_REG_MASK_CELL_5
                                    | BFE_REG_MASK_CELL_6
                                    | BFE_REG_MASK_CELL_7
                                    | BFE_REG_MASK_CELL_8
                                    | BFE_REG_MASK_CELL_9
                                    | BFE_REG_MASK_CELL_10
                                    | BFE_REG_MASK_CELL_11
                                    | BFE_REG_MASK_CELL_12),


                               },

             .temps_cfg          = {
                                    /* Device 1 */
                                    ( BFE_REG_MASK_TEMP_IC
                                    | BFE_REG_MASK_TEMP_EXT1
                                    | BFE_REG_MASK_TEMP_EXT2
                                    | BFE_REG_MASK_TEMP_EXT3
                                    | BFE_REG_MASK_TEMP_EXT4),

                                    /* Device 2 */
                                    ( BFE_REG_MASK_TEMP_IC
                                    | BFE_REG_MASK_TEMP_EXT1
                                    | BFE_REG_MASK_TEMP_EXT2
                                    | BFE_REG_MASK_TEMP_EXT3
                                    | BFE_REG_MASK_TEMP_EXT4),

                                },

         /* Configure temperatures to be fault monitored. */
         .temp_flt_mon       = {
                                    /* Device 1 */
                                    ( BFE_REG_MASK_TEMP_IC),

                                    /* Device 2 */
                                    ( BFE_REG_MASK_TEMP_IC),

                                    /* Device 3 */
                                  // ( BFE_REG_MASK_TEMP_IC),

                                    /* Add more if stack size is bigger */
                               }
    }
};

const st_bfe_cfg_t g_bfe0_cfg =
{
#if BFE_CFG_STA_DEV > 1U
    .mode               = BFE_CONFIG_DAISY_CHAIN,
    .d_rate_dch         = BFE_D_RT_62_5_KHZ
#else
    .mode               = BFE_CONFIG_STANDALONE
#endif // BFE_CFG_DCH_OPERATION
};

/* Instance structure to use BFE module. */
const bfe_instance_t g_bfe0 =
{ .p_ctrl = &g_bfe0_ctrl, .p_cfg = &g_bfe0_cfg, .p_api = &g_bfe_on_bfe };

/**********************************************************************************************************************
 Private (static) variables and functions
 *********************************************************************************************************************/

