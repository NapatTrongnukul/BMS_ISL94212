
/* Generic headers */
#include <stdint.h>
#include <stdbool.h>

/* BFE related headers */
#include <bfe/r_bfe_isl94212.h>
uint32_t cc = 0;
uint32_t dd = 0;
uint32_t ee = 0;
uint32_t ff = 0;
uint32_t gg = 0;
uint32_t identify_check = 0;
uint32_t datalength_check = 0;
uint32_t exp_resp_lenght_check = 0;
uint32_t BMS_ERR_DRDY_TIMEOUT_check = 0;
uint16_t volt_cells[2][12];
uint16_t volt_batt[2];
uint16_t bc;



float volt_conv = 0.00061035156;

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Local Typedef definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Exported global variables
 *********************************************************************************************************************/

/** BFE implementation of BFE interface. */
const bfe_api_t g_bfe_on_bfe =
{
    .open             = R_BFE_Open,
    .close            = R_BFE_Close,
    .reset            = R_BFE_Reset,
    .commTest         = R_BFE_CommTest,
    .sleep            = R_BFE_Sleep,
    .wakeUp           = R_BFE_WakeUp,
    .randomWakeUp     = R_BFE_RandomWakeUp,
    .voltTempAllGet   = R_BFE_VoltTempAllGet,
    .vAllGet          = R_BFE_VAllGet,
    .vBattGet         = R_BFE_VBattGet,
    .vMixGet          = R_BFE_VMixGet,
    .tempsGet         = R_BFE_TempsGet,
    .faultsAllRead    = R_BFE_FaultsAllRead,
    .faultsCheck      = R_BFE_FaultsCheck,
    .faultsAllClear   = R_BFE_FaultsAllClr,
    .balanceControl   = R_BFE_BalanceCtrl,
    .memoryCheck      = R_BFE_ConfRegCheck,
    .setup            = R_BFE_DeviceSetup,
    .wdControl        = R_BFE_WatchdogCtrl,
    .wdReset          = R_BFE_WatchdogReset,
    .scan             = R_BFE_Scan,
    .userRegsAccess   = R_BFE_UserRegsisterAccess,
    .singleRegAccess  = R_BFE_SingleRegisterAccess
};

/* Maximum delay between command and response at different daisy chain data rates [us].
 * Please, refer to ISL94212 Datasheet! */
const uint32_t g_bfe_max_resp_delay_us[13][4] = {{2640,  1320,  660,   330},   //2 devices in stack
                                                 {4010,  2010,  1010,  510},   //3 devices in stack
                                                 {5550,  2780,  1390,  700},   //4 devices in stack
                                                 {7570,  3790,  1900,  950},   //5 devices in stack
                                                 {9950,  4980,  2490,  1250},  //6 devices in stack
                                                 {12850, 6430,  3220,  1610},  //7 devices in stack
                                                 {16550, 8280,  4140,  2070},  //8 devices in stack
                                                 {20950, 10480, 5240,  2620},  //9 devices in stack
                                                 {26230, 13120, 6560,  3280},  //10 devices in stack
                                                 {32560, 16280, 8140,  4070},  //11 devices in stack
                                                 {41360, 20680, 10340, 5170},  //12 devices in stack
                                                 {50160, 25080, 12540, 6270},  //13 devices in stack
                                                 {62480, 31240, 15620, 7810}}; //14 devices in stack

/* Maximum delay between two adjacent response bytes at different daisy chain data rates [us].
 * Please, refer to ISL94212 Datasheet! */
const uint32_t g_bfe_max_bytes_delay_us[4] = {544, 272, 136, 68};

/* Minimum communications wait time [us] (Maximum time for daisy chain ports to clear).
 * Please, refer to ISL94212 Datasheet! */
const uint32_t g_bfe_min_wait_time_us[4] = {144, 72, 36, 18};

/* Maximum wait time for devices entering sleep mode.
 * Please, refer to ISL94212 Datasheet! */
const uint32_t g_bfe_sleep_time_us[4] = {4000, 2000, 1000, 500};

/* Maximum wait time for devices to exit sleep mode.
 * Please, refer to ISL94212 Datasheet! */
const uint32_t g_bfe_wakeup_time_us[4] = {100000, 100000, 100000, 100000};

/* Wait counter for wait operation monitoring */
static volatile uint32_t s_spi_wait_count = BFE_SPI_MAX_CNT;
static volatile uint32_t s_spi_timeout = BFE_SPI_TO_MAX_CNT;

/* Event flag for SPI */
static volatile spi_event_t s_spi_event_flag = (spi_event_t) RESET_VALUE; // SPI Transfer Event completion flag

/* Event flag for communication timeout 0 */
static volatile bool s_timeout_event_flag = false;  // Communication timeout flag

/* Temporary watchdog time holder. */
static volatile uint16_t s_wd_time_temp = 0;

/* SPI module buffers */
static uint8_t s_spi_tx_buff[BFE_SPI_TX_BUFF_LGTH] = {ZERO}; // SPI transmit buffer
static uint8_t s_spi_rx_buff[BFE_SPI_RX_BUFF_LGTH] = {ZERO}; // SPI receive buffer

static st_isl94212_spi_msg_t s_spi_msg =
{
    .r_w_data        = BFE_READ_REG,
    .command         = {.device_address = BFE_DAISY_CHAIN_IDENTIFY, .reg_address = ZERO, .data = ZERO},
    .response[0]     = {.device_address = BFE_DAISY_CHAIN_IDENTIFY, .reg_address = ZERO, .data = ZERO},
    .enc_cmnd_length = ZERO,
    .enc_resp_length = ZERO,
    .encoded_cmnd[0] = ZERO,
    .encoded_resp[0] = ZERO
};

static st_bfe_comm_timeouts_t s_comm_delays =
{
    .max_resp_delay   = ZERO,
    .max_bytes_delay  = ZERO,
    .min_wait_time_us = ZERO,
    .sleep_time_us    = ZERO,
    .wakeup_time_us   = ZERO
};

/**********************************************************************************************************************
 Private (static) variables and functions
 *********************************************************************************************************************/
static e_bms_err_t bfe_identify (st_bfe_ctrl_t * p_bfe_ctrl);
static e_bms_err_t bfe_reset (st_bfe_ctrl_t * p_bfe_ctrl);
static e_bms_err_t bfe_setup (st_bfe_ctrl_t * p_bfe_ctrl);
static e_bms_err_t bfe_faults_read(st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_faults_t * p_faults);
static e_bms_err_t bfe_watchdog (st_bfe_ctrl_t * p_bfe_ctrl);
static e_bms_err_t bfe_EEPROMcheck (st_bfe_ctrl_t * p_bfe_ctrl);
static e_bms_err_t bfe_spi_msg_send_resp_get (st_isl94212_spi_msg_t * p_spi_msg);
static e_bms_err_t bfe_spi_d_ch_resp_get (st_isl94212_spi_msg_t * p_spi_msg);
static e_bms_err_t bfe_spi_pending_resp_get (st_isl94212_spi_msg_t * p_spi_msg);
static e_bms_err_t bfe_command_encode (st_isl94212_spi_msg_t * p_spi_msg);
static e_bms_err_t bfe_responce_decode (st_isl94212_spi_msg_t * p_spi_msg);

/* Function Name: R_BFE_Open */
/******************************************************************************************************************//**
 * This function initializes a Battery Front End.
 *
 * This function performs the following tasks:
 * - Parameter checking and error conditions processing.
 * - Initializes a communication timeout timer.
 * - Initializes a communication interface to connect to the BFE (master) device.
 * - Resets the device (stack).
 * - Accomplishes identification procedure of daisy chain devices when initializing a stack.
 * - Reads serial numbers of all BFE devices.
 * - Checks the memory.
 * - Writes to non-default values to setup registers.
 *
 * The following input parameters need to be set:
 * - p_bfe_ctrl->wd_time
 * - p_bfe_ctrl->setup.i_wire_scan
 * - p_bfe_ctrl->setup.balance_mode
 * - p_bfe_ctrl->setup.temp_flt_mon
 * - p_bfe_ctrl->setup.flt_tot_samples
 * - p_bfe_ctrl->setup.overvoltage_limit
 * - p_bfe_ctrl->setup.undervoltage_limit
 * - p_bfe_ctrl->setup.ext_temp_limit
 * - p_bfe_ctrl->setup.cells_cfg
 *
 * The following output parameters are modified by this function:
 * - p_bfe_ctrl->status.ic_temp_limit
 * - p_bfe_ctrl->status.v_trim
 * - p_bfe_ctrl->status.identify_ctr
 * - p_bfe_ctrl->status.stacked_devs
 * - p_bfe_ctrl->status.serial_number
 *
 * @pre Please, perform an appropriate peripheral communication module setup, according to the BFE datasheet!
 * @warning This function does not check the communication module settings!
 * @param[in]       p_bfe_ctrl                  Pointer to battery front end control structure.
 * @param[in]       p_bfe_cfg                   Pointer to battery front end configuration structure.
 * @retval          BFE_SUCCESS                 Battery Front End is successfully initialized.
 * @retval          BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval          BMS_ERR_SPI_INIT            SPI initialization error.
 * @retval          BMS_ERR_COMM_TIMEOUT_INIT   Communication timeout timer initialization error.
 * @retval          BMS_ERR_...                 Inherit from bfe_reset().
 * @retval          BMS_ERR_...                 Inherit from bfe_identify().
 * @retval          BMS_ERR_...                 Inherit from bfe_EEPROMcheck().
 * @retval          BMS_ERR_...                 Inherit from bfe_setup().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_Open(st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_cfg_t const * const p_bfe_cfg)
{
    e_bms_err_t err     = BMS_SUCCESS; // Error status
    fsp_err_t fsp_err = FSP_SUCCESS; // Error status

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl, BMS_ERR_INVALID_POINTER);
    BFE_ERROR_RETURN(NULL != p_bfe_cfg,  BMS_ERR_INVALID_POINTER);

#endif // BFE_CFG_PARAM_CHECKING_EN

/** To work with ISL94212, configure the SPI peripheral module with the following settings:
* - Operating mode: Master
* - Clock phase: Data sampling on odd edge
* - Clock polarity: Low when idle
* - Bit order: MSB first
* - Slave select polarity: Active low
* - SPI Mode: SPI operation (asserts SSL automatically) for Full Duplex
* - MOSI Idle State: Idle value fixing disable
* - Bitrate: 2 Mbps (maximal)
*
* For more information, please, refer to ISL94212 Datasheet!
*/

    /* Clear status. */
    memset(&p_bfe_ctrl->status, ZERO, sizeof(st_bfe_status_t));

#if BFE_CFG_STA_DEV > 1U // Daisy chain operation


    /* Open (initialize) communication timeout timer. */
    fsp_err = g_timer_one_shot.p_api->open(&g_timer_one_shot_ctrl, &g_timer_one_shot_cfg);

    BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_COMM_TIMEOUT_INIT); // Check for errors.


    /* Get the source clock frequency (in Hz). */
    uint32_t timer_freq_hz = R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_PCLKB) >> g_timer_one_shot_cfg.source_div;

    /* Covert communication timeouts depending on communication speed.
     * A cast to uint64_t is used to prevent uint32 overflow. */
    s_comm_delays.max_resp_delay   = (uint32_t) (((uint64_t) timer_freq_hz *
                                                 g_bfe_max_resp_delay_us[BFE_CFG_STA_DEV - 2U][p_bfe_cfg->d_rate_dch])
                                                                                                       / BFE_US_PER_S);


    s_comm_delays.max_bytes_delay  = (uint32_t) (((uint64_t) timer_freq_hz *
                                     g_bfe_max_bytes_delay_us[p_bfe_cfg->d_rate_dch]) / BFE_US_PER_S);

    s_comm_delays.min_wait_time_us = g_bfe_min_wait_time_us[p_bfe_cfg->d_rate_dch];
    s_comm_delays.sleep_time_us    = g_bfe_sleep_time_us[p_bfe_cfg->d_rate_dch];
    s_comm_delays.wakeup_time_us   = g_bfe_wakeup_time_us[p_bfe_cfg->d_rate_dch];



#endif //BFE_CFG_DCH_OPERATION

    /* Open (initialize) SPI communication channel. */
    fsp_err = g_spi1.p_api->open(&g_spi1_ctrl, &g_spi1_cfg);

    BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_SPI_INIT); // Check for errors.

#if BFE_CFG_USE_ENBL_PIN // BFE enable pin is connected.


    R_IOPORT_PinWrite(&g_ioport_ctrl, BFE_ENBL_PIN, BSP_IO_LEVEL_HIGH); // Enable the BFE


#endif /* BFE_CFG_USE_ENBL_PIN */

    /* ISL94212 Power-up delay. */
    R_BSP_SoftwareDelay(BFE_START_UP_DELAY_MS, BSP_DELAY_UNITS_MILLISECONDS);

    /* Reset the device (stack). */
    err = bfe_reset(p_bfe_ctrl);

    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Identify daisy chain devices and/ or get serial number */
    err = bfe_identify(p_bfe_ctrl);


    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Check EEPROM data for corruption. */
    err = bfe_EEPROMcheck(p_bfe_ctrl);

    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    p_bfe_ctrl->status.initialized = BFE_INITILIZED; // Set the initialization complete flag.

    /* Device setup. Load non-default setup parameters. */
    err = bfe_setup(p_bfe_ctrl);

    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    return err;
}
/**********************************************************************************************************************
 * End of function R_BFE_Open
 *********************************************************************************************************************/

/* Function Name: R_BFE_Close */
/******************************************************************************************************************//**
 * This function deinitializes a Battery Front End.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Deinitializes the communication interface to connect to the BFE (master) device.
 *
 * The following output parameters are modified by this function:
 * - p_bfe_ctrl->status.serial_number
 * - p_bfe_ctrl->status.initialized
 *
 * @pre The BFE interface should have already been opened!
 * @param[in]       p_bfe_ctrl                  Pointer to Battery Front End control structure.
 * @retval          BFE_SUCCESS                 Battery Front End successfully deinitialized.
 * @retval          BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval          BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval          BMS_ERR_SPI_DEINIT          SPI deinitialization error.
 * @retval          BMS_ERR_COMM_TIMEOUT_INIT   One shot timer deinitialization error.
 *********************************************************************************************************************/
e_bms_err_t R_BFE_Close(st_bfe_ctrl_t * p_bfe_ctrl)
{
    e_bms_err_t err     = BMS_SUCCESS; // Error status
    fsp_err_t fsp_err = FSP_SUCCESS; // Error status

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl, BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);

#endif // BFE_CFG_PARAM_CHECKING_EN

#if BFE_CFG_USE_ENBL_PIN // BFE enable pin is connected.

    R_IOPORT_PinWrite(&g_ioport_ctrl, BFE_ENBL_PIN, BSP_IO_LEVEL_LOW); // Disable the BFE

#endif /* BFE_CFG_USE_ENBL_PIN */

    /* Close (deinitialize) SPI communication channel. */
    fsp_err = g_spi1.p_api->close(&g_spi1_ctrl);
    BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_SPI_DEINIT); // Check for errors.

    /* Close communication timeout timer. */
    fsp_err = g_timer_one_shot.p_api->close(&g_timer_one_shot_ctrl);

    BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_COMM_TIMEOUT_INIT); // Check for errors.

    /* Clear the serial number array. */
    memset(&p_bfe_ctrl->status.serial_number[0], ZERO, sizeof(p_bfe_ctrl->status.serial_number));

    /* Clear the initialization complete flag. */
    p_bfe_ctrl->status.initialized = BFE_UNINITILIZED;

    return err;
}
/**********************************************************************************************************************
 * End of function R_BFE_Close
 *********************************************************************************************************************/

/* Function Name: R_BFE_Reset */
/******************************************************************************************************************//**
 * This function resets, identifies and checks the Battery Front End.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Resets the device (stack).
 * - Accomplishes identification procedure of daisy chain devices when initializing a stack.
 * - Checks the memory.
 *
 * The following input parameters need to be set:
 * - p_bfe_ctrl->wd_time
 * - p_bfe_ctrl->setup.i_wire_scan
 * - p_bfe_ctrl->setup.balance_mode
 * - p_bfe_ctrl->setup.temp_flt_mon
 * - p_bfe_ctrl->setup.flt_tot_samples
 * - p_bfe_ctrl->setup.overvoltage_limit
 * - p_bfe_ctrl->setup.undervoltage_limit
 * - p_bfe_ctrl->setup.ext_temp_limit
 * - p_bfe_ctrl->setup.cells_cfg
 *
 * The following output parameters are modified by this function:
 * - p_bfe_ctrl->status.ic_temp_limit
 * - p_bfe_ctrl->status.v_trim
 * - p_bfe_ctrl->status.identify_ctr
 * - p_bfe_ctrl->status.stacked_devs
 * - p_bfe_ctrl->status.serial_number
 *
 * @pre The BFE interface should have already been opened!
 * @param[in]       p_bfe_ctrl                  Pointer to Battery Front End control structure.
 * @retval          BFE_SUCCESS                 Battery Front End is successfully reseted.
 * @retval          BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval          BMS_ERR_...                 Inherit from bfe_reset().
 * @retval          BMS_ERR_...                 Inherit from bfe_identify().
 * @retval          BMS_ERR_...                 Inherit from bfe_EEPROMcheck().
 * @retval          BMS_ERR_...                 Inherit from bfe_setup().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_Reset(st_bfe_ctrl_t * p_bfe_ctrl)
{
    e_bms_err_t err = BMS_SUCCESS; // Error status

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl, BMS_ERR_INVALID_POINTER);

#endif // BFE_CFG_PARAM_CHECKING_EN

    /* Clear the serial number. */
    memset(&p_bfe_ctrl->status.serial_number[0], ZERO, sizeof(p_bfe_ctrl->status.serial_number));

    p_bfe_ctrl->status.initialized = BFE_UNINITILIZED; // Clear the initialization complete flag.

#if BFE_CFG_USE_ENBL_PIN // Use EN pin on all devices.

    R_IOPORT_PinWrite(&g_ioport_ctrl, BFE_ENBL_PIN, BSP_IO_LEVEL_LOW); // Disable the BFE

    /* Hold enable pin down for a while. */
    R_BSP_SoftwareDelay(BFE_ENBL_PIN_HOLD_DELAY_MS, BSP_DELAY_UNITS_MILLISECONDS);

    R_IOPORT_PinWrite(&g_ioport_ctrl, BFE_ENBL_PIN, BSP_IO_LEVEL_HIGH); // Enable the BFE

    /* ISL94212 Enable delay. */
    R_BSP_SoftwareDelay(BFE_START_UP_DELAY_MS, BSP_DELAY_UNITS_MILLISECONDS);

    /* Identify daisy chain devices and/ or get serial number */
    err = bfe_identify(p_bfe_ctrl);
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

#endif /* BFE_CFG_USE_ENBL_PIN */

    /* Reset the device (stack). */
    err = bfe_reset(p_bfe_ctrl);
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Identify daisy chain devices and/ or get serial number */
    err = bfe_identify(p_bfe_ctrl);
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Check EEPROM data for corruption. */
    err = bfe_EEPROMcheck(p_bfe_ctrl);
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    p_bfe_ctrl->status.initialized = BFE_INITILIZED; // Set the initialization complete flag.

    /* Device setup. Load non-default setup parameters. */
    err = bfe_setup(p_bfe_ctrl);
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err);   // Check for errors.

    return err;
}
/**********************************************************************************************************************
 * End of function R_BFE_Reset
 *********************************************************************************************************************/

/* Function Name: R_BFE_CommTest */
/******************************************************************************************************************//**
 * This function tests the communication between the master device and microcontroller and the devices
 *  in the stack.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - When stand-alone operation reads User Register 1, stores the value, writes and reads User Register 1,
 * compares and restores value.
 * - When daisy-chain operation sends ACK command and waits for response form the top stack device.
 *
 * @pre The BFE interface should have already been opened!
 * @param[in]   p_bfe_ctrl                  Pointer to Battery Front End control structure.
 * @retval      BFE_SUCCESS                 The Battery Front End is reachable.
 * @retval      BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval      BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval      BMS_ERR_SPI_COMM_ERR        SPI communication error (stand-alone).
 * @retval      BMS_ERR_COMM_ERR            Communication error (daisy chain).
 * @retval      BMS_ERR_REG_VERIFY          Register write verification error.
 * @retval      BMS_ERR_...                 Inherit from bfe_spi_msg_send_resp_get().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_CommTest(st_bfe_ctrl_t * p_bfe_ctrl)
{
    e_bms_err_t err  = BMS_SUCCESS; // Error status
    uint16_t  tmp1 = 0;

    FSP_PARAMETER_NOT_USED(tmp1);

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl, BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);

#endif // BFE_CFG_PARAM_CHECKING_EN

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

    /* Send Acknowledge Command to the top stack device. */
    s_spi_msg.command.device_address = (e_bfe_dev_addr_t)p_bfe_ctrl->status.stacked_devs; // Address the top device.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_ACK;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Check for ACK. */
    BFE_ERROR_RETURN(s_spi_msg.response[0].reg_address == (BFE_REG_ACK & BFE_REG_PAGE_ADDR_MASK), BMS_ERR_COMM_ERR);

#else // Stand-alone operation

    /* Read and store the User Register 1 value. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_USER_REG1;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    tmp1 = s_spi_msg.response[0].data; // Store temporarily the result

    /* Write a test constant into the User Register 1. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
    s_spi_msg.r_w_data = BFE_WRITE_REG;
    s_spi_msg.command.reg_address = BFE_REG_USER_REG1;
    s_spi_msg.command.data = BFE_SPI_TEST_WRITE_MSG;

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Read and compare the test constant from the User Register 1. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_USER_REG1;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    BFE_ERROR_RETURN(s_spi_msg.response[0].data == BFE_SPI_TEST_WRITE_MSG, BMS_ERR_SPI_COMM);

    /* Restore the initial value of the User Register 1 value. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
    s_spi_msg.r_w_data = BFE_WRITE_REG;
    s_spi_msg.command.reg_address = BFE_REG_USER_REG1;
    s_spi_msg.command.data = tmp1;

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

#if BFE_CFG_REG_WRITE_VERIFY_EN // Verify register write
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_USER_REG1;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    BFE_ERROR_RETURN(s_spi_msg.response[0].data == tmp1, BMS_ERR_REG_VERIFY);
#endif // BFE_CFG_REG_WRITE_VERIFY_EN
#endif // BFE_CFG_DCH_OPERATION

    return err;
}
/**********************************************************************************************************************
 * End of function R_BFE_CommTest
 *********************************************************************************************************************/

/* Function Name: R_BFE_Sleep */
/******************************************************************************************************************//**
 * This function puts the Battery Front End in to Sleep Mode.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Sends a Sleep Command to the stand-alone device or all devices in the stack or sets the watchdog timeout to 1 s.
 * - Receive the ACK response from top device when Sleep Command is used.
 *
 * The following output parameters are modified by this function:
 * - p_bfe_ctrl->status.in_sleep_mode
 * - p_bfe_ctrl->status.balancing
 *
 * @pre The BFE interface should have already been opened!
 * @param[in]       p_bfe_ctrl                  Pointer to Battery Front End control structure.
 * @retval          BFE_SUCCESS                 The Sleep Commands are successfully sent.
 * @retval          BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval          BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval          BMS_ERR_SLEEP               Error entering sleep state.
 * @retval          BMS_ERR_...                 Inherit from bfe_watchdog().
 * @retval          BMS_ERR_...                 Inherit from bfe_spi_msg_send_resp_get().
 * @retval          BMS_ERR_...                 Inherit from bfe_spi_d_ch_resp_get().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_Sleep(st_bfe_ctrl_t * p_bfe_ctrl)
{
    e_bms_err_t err = BMS_SUCCESS; // Error status

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl, BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);

#endif // BFE_CFG_PARAM_CHECKING_EN

    FSP_PARAMETER_NOT_USED(s_wd_time_temp);

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

#if BFE_CFG_WDT_SLEEP_EN && (BFE_CFG_STA_DEV > 1U)  // Go to sleep using watchdog timeout

    /* Force daisy chain devices to go to sleep with watchdog time. */
    s_wd_time_temp = p_bfe_ctrl->wd_time;
    //p_bfe_ctrl->wd_time = BFE_WDT_5S;
    p_bfe_ctrl->wd_time = BFE_WDT_1S;

    /* Configure watchdog time. */
    err = bfe_watchdog(p_bfe_ctrl);
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

#else // Send Sleep Command

    /* Send Sleep Command. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_ADDRESS_ALL; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SLEEP;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

    /* Get the ACK from top device. */
    err = bfe_spi_d_ch_resp_get(&s_spi_msg); // Receive the response
    BFE_ERROR_RETURN((BMS_SUCCESS == err)
                  && (s_spi_msg.response[0].reg_address == (BFE_REG_ACK & BFE_REG_PAGE_ADDR_MASK)), BMS_ERR_SLEEP);

#endif //BFE_CFG_DCH_OPERATION
#endif /* BFE_CFG_WDT_SLEEP_EN */

    /*Wait for devices to enter sleep mode. */
    R_BSP_SoftwareDelay(s_comm_delays.sleep_time_us, BSP_DELAY_UNITS_MICROSECONDS);

    p_bfe_ctrl->status.in_sleep_mode = true; // Change status
    p_bfe_ctrl->status.balancing = false; // Update balancing flag

    return err;
}
/**********************************************************************************************************************
 * End of function R_BFE_Sleep
 *********************************************************************************************************************/

/* Function Name: R_BFE_WakeUp */
/******************************************************************************************************************//**
 * This function wakes up the Battery Front End.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Sends a Sleep Command to to verify master device is in sleep mode.
 * - Sends a Wake up Command to the stand-alone device or all devices in the stack.
 * - In daisy chain mode clears the Fault Status Register and reverts the watchdog timeout.
 *
 * The following output parameters are modified by this function:
 * - p_bfe_ctrl->status.in_sleep_mode
 *
 * @pre The BFE interface should have already been opened!
 * @param[in]       p_bfe_ctrl                  Pointer to Battery Front End control structure.
 * @retval          BFE_SUCCESS                 Battery Front End is successfully woken up.
 * @retval          BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval          BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval          BMS_ERR_WAKEUP              Device wake up error.
 * @retval          BMS_ERR_...                 Inherit from bfe_watchdog().
 * @retval          BMS_ERR_...                 Inherit from bfe_spi_msg_send_resp_get().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_WakeUp(st_bfe_ctrl_t * p_bfe_ctrl)
{
    e_bms_err_t err  = BMS_SUCCESS; // Error status
    uint16_t  tmp1 = 0;

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl, BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);

#endif // BFE_CFG_PARAM_CHECKING_EN

    FSP_PARAMETER_NOT_USED(tmp1);

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

    /* Send Sleep Command. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_ADDRESS_ALL; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SLEEP;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /*Wait for devices to enter sleep mode. */
    R_BSP_SoftwareDelay(s_comm_delays.sleep_time_us, BSP_DELAY_UNITS_MICROSECONDS);

    /* Send Wake up Command. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_ADDRESS_ALL; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_WAKEUP;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /*Wait for all devices to wake up. */
    R_BSP_SoftwareDelay(s_comm_delays.wakeup_time_us, BSP_DELAY_UNITS_MICROSECONDS);

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation
#if BFE_CFG_WDT_SLEEP_EN // Go to sleep using watchdog timeout

    /* Clear Watchdog timeout faults. */
    for(uint16_t i = p_bfe_ctrl->status.stacked_devs; i >= 1; i-- )
    {
        /* Clear fault. */
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)i; // Assign a device address.
        s_spi_msg.r_w_data = BFE_WRITE_REG;
        s_spi_msg.command.reg_address = BFE_REG_FAULT_STATUS;
        s_spi_msg.command.data = ZERO;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.
    }

    /* Restore watchdog time value. */
    p_bfe_ctrl->wd_time = s_wd_time_temp;

    /* Configure watchdog time. */
    err = bfe_watchdog(p_bfe_ctrl);
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

#else // No watchdog timeout is used for sleep mode

    /* Check for pending response. */
    err = bfe_spi_pending_resp_get(&s_spi_msg);
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for error.

    /* Check for ACK in the pending response. */
    BFE_ERROR_RETURN(s_spi_msg.response[0].reg_address == (BFE_REG_ACK & BFE_REG_PAGE_ADDR_MASK), BMS_ERR_WAKEUP);

#endif /* BFE_CFG_WDT_SLEEP_EN. */
#endif //BFE_CFG_DCH_OPERATION

    p_bfe_ctrl->status.in_sleep_mode = false; // Change status

    return err;
}
/**********************************************************************************************************************
 * End of function R_BFE_WakeUp
 *********************************************************************************************************************/

/* Function Name: R_BFE_RandomWakeUp */
/******************************************************************************************************************//**
 * This function wakes up a sleeping device on random position in the stack.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Sends a Sleep Command to to verify devices below the sleeping one are also in sleep mode.
 * - Sends a Wake up Command to all devices in the stack.
 * - Repeat sequence until all devises are awake or the loops has expired.
 *
 * The following output parameters are modified by this function:
 * - p_bfe_ctrl->status.in_sleep_mode
 *
 * @pre The BFE interface should have already been opened!
 * @param[in]       p_bfe_ctrl                  Pointer to Battery Front End control structure.
 * @retval          BFE_SUCCESS                 Battery Front End is successfully woken up.
 * @retval          BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval          BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval          BMS_ERR_WAKEUP              Device wake up error.
 * @retval          BMS_ERR_...                 Inherit from bfe_watchdog().
 * @retval          BMS_ERR_...                 Inherit from bfe_spi_msg_send_resp_get().
 * @retval          BMS_ERR_...                 Inherit from bfe_spi_d_ch_resp_get().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_RandomWakeUp(st_bfe_ctrl_t * p_bfe_ctrl)
{
    e_bms_err_t err  = BMS_SUCCESS; // Error status

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl, BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);

#endif // BFE_CFG_PARAM_CHECKING_EN

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

    /* Try to wake up sleeping devices. */
    for(uint16_t i = 1; i <= p_bfe_ctrl->status.stacked_devs; i++)
    {
        /* Send Sleep Command. */
        s_spi_msg.command.device_address = BFE_DAISY_CHAIN_ADDRESS_ALL; // Assign a device address.
        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_SLEEP;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        /* Get the ACK from top device. */
        err = bfe_spi_d_ch_resp_get(&s_spi_msg); // Receive the response
        BFE_ERROR_RETURN(BMS_ERR_SPI_MSG_BUF != err, err); // Ignore returned errors.

        /*Wait for devices to enter sleep mode. */
        R_BSP_SoftwareDelay(s_comm_delays.sleep_time_us, BSP_DELAY_UNITS_MICROSECONDS);

        /* Send Wake up Command. */
        s_spi_msg.command.device_address = BFE_DAISY_CHAIN_ADDRESS_ALL; // Assign a device address.
        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_WAKEUP;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        /*Wait for all devices to wake up. */
        R_BSP_SoftwareDelay(s_comm_delays.wakeup_time_us, BSP_DELAY_UNITS_MICROSECONDS);

        /* Check for pending response. */
        err = bfe_spi_pending_resp_get(&s_spi_msg);
        BFE_ERROR_RETURN(BMS_ERR_SPI_MSG_BUF != err, err); // Ignore returned errors.

        /* Check for ACK in the pending response. */
        if(s_spi_msg.response[0].reg_address == (BFE_REG_ACK & BFE_REG_PAGE_ADDR_MASK))
        {
            break; // Exit the loop when ACK is received.
        }
        else if(i == p_bfe_ctrl->status.stacked_devs)
        {
            return BMS_ERR_WAKEUP;
        }
    }
#else // Stand-alone operation

    /* Mute unused function. */
    if(false)
    {
        bfe_spi_pending_resp_get(&s_spi_msg);
    }
    else
    {
        ;
    }

#endif //BFE_CFG_DCH_OPERATION

    p_bfe_ctrl->status.in_sleep_mode = false; // Change status

    return err;
}
/**********************************************************************************************************************
 * End of function R_BFE_RandomWakeUp
 *********************************************************************************************************************/

/* Function Name: R_BFE_VoltTempAllGet */
/******************************************************************************************************************//**
 * This function performs battery, all cells, internal temperature and ExT1 voltage scan and read. (Stand-alone/
 *  Daisy chain)
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Sends a Scan All Command to the stand-alone device or all devices in the stack.
 * - Confirms the reception of the Scan All Command.
 * - Sends commands to read battery, cell voltages, internal and external temperatures and second reference voltage.
 *
 * The following output parameters are modified by this function:
 * - p_meas_data->v_sec_ref
 * - p_meas_data->v_ic_temp
 * - p_meas_data->v_ext_temps
 * - p_meas_data->v_batt
 * - p_meas_data->v_cells
 *
 * @pre The BFE interface should have already been opened!
 * @param[in]   p_bfe_ctrl                  Pointer to Battery Front End control structure.
 * @param[out]  p_meas_data                 Pointer to measured data array.
 * @retval      BFE_SUCCESS                 The temperatures and voltages successfully are read.
 * @retval      BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval      BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval      BMS_ERR_RESPONCE            Invalid response is received.
 * @retval      BMS_ERR_SCAN_CNTR           The Scan Command was not received.
 * @retval      BMS_ERR_...                 Inherit from bfe_spi_msg_send_resp_get().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_VoltTempAllGet(st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_meas_t * p_meas_data)
{
    e_bms_err_t err = BMS_SUCCESS; // Error status

    uint16_t  scan_count[BFE_CFG_STA_DEV] = {0};

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl,  BMS_ERR_INVALID_POINTER);
    BFE_ERROR_RETURN(NULL != p_meas_data, BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);

#endif // BFE_CFG_PARAM_CHECKING_EN

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

#if BFE_CFG_SCAN_CMND_VERIFY_EN // Read scan count.

    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SCAN_COUNT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    /* Send command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

        scan_count[i] = s_spi_msg.response[0].data;
    }

#endif // BFE_CFG_SCAN_CMND_VERIFY_EN

    /* Send Scan All Command. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_ADDRESS_ALL; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SCAN_ALL;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Scan processing time. */
    R_BSP_SoftwareDelay(BFE_SCAN_ALL_PROC_US, BSP_DELAY_UNITS_MICROSECONDS);

#if BFE_CFG_SCAN_CMND_VERIFY_EN // Verify Scan Command reception.
#if BFE_CFG_STA_DEV == 1U  // Stand-alone operation

    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SCAN_COUNT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    if(scan_count[0] < BFE_SCAN_CNTR_MAX)
    {
        scan_count[0]++;
    }
    else
    {
        scan_count[0] = 0;
    }

    BFE_ERROR_RETURN(scan_count[0] == s_spi_msg.response[0].data, BMS_ERR_SCAN_CNTR); // Check counter.

#endif //BFE_CFG_DCH_OPERATION
#endif // BFE_CFG_SCAN_CMND_VERIFY_EN

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

    /* Copy received data for every device. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {

        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        /* Send read all temperatures command. */
        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_ALL_TEMP_DATA;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type
        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.

        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        /* Check for valid response. */
        BFE_ERROR_RETURN(s_spi_msg.response[0].reg_address == (BFE_REG_SCAN_COUNT & BFE_REG_PAGE_ADDR_MASK),
                                                                                          BMS_ERR_RESPONCE);

        /* Compare counter for every device. */
        if(scan_count[i] < BFE_SCAN_CNTR_MAX)
        {
            scan_count[i]++;

        }
        else
        {
            scan_count[i] = 0;

        }

        BFE_ERROR_RETURN(scan_count[i] == s_spi_msg.response[0].data, BMS_ERR_SCAN_CNTR); // Check counter.

        /* Copy reference voltage to the input data structure. */
        (p_meas_data + i)->v_sec_ref = s_spi_msg.response[BFE_TALL_VREF_OFFST].data;

        /* Copy internal temperature to the input data structure. */
        (p_meas_data + i)->v_ic_temp = s_spi_msg.response[BFE_TALL_TIC_OFFST].data;

        /* Read external temperatures using register address offset. */
        for(uint16_t j = 1; j <= BFE_TEMP_INPUTS; j++ )
        {
            /* Copy external temperatures to the input data structure. */
            (p_meas_data + i)->v_ext_temps[BFE_TEMP_INPUTS - j] = s_spi_msg.response[BFE_TALL_VREF_OFFST + j].data;
        }

        /* Send Read All Registers Page 1 Command. */
        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_ALL_CELL_VOLT;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        /* Check for valid response. */
        BFE_ERROR_RETURN(s_spi_msg.response[0].reg_address == (BFE_REG_CELL_12_VOLT & BFE_REG_PAGE_ADDR_MASK),
                                                                                            BMS_ERR_RESPONCE);

        /* Copy battery voltage to the input data structure. */
        (p_meas_data + i)->v_batt = s_spi_msg.response[BFE_VALL_VBAT_OFFST].data;

        /* Read cell voltages using register address offset. */
        for(uint16_t j = 1; j <= BFE_VALL_VBAT_OFFST; j++ )
        {

            /* Copy cell voltages to the input data structure. */
            (p_meas_data + i)->v_cells[BFE_VALL_VBAT_OFFST - j] = s_spi_msg.response[j - 1].data;

        }
    }

#else // Stand-alone operation

    /* Send Read Battery Voltage Command. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_BAT_VOLT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Copy battery voltage to the input data structure. */
    p_meas_data->v_batt = s_spi_msg.response[0].data;

    /* Read cell voltages using register address offset. */
    for(uint32_t i = 0; i < BFE_MAX_CELLS_PER_IC; i++ )
    {
        s_spi_msg.command.reg_address = BFE_REG_CELL_1_VOLT + i;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        p_meas_data->v_cells[i] = s_spi_msg.response[0].data; // Copy cell voltages to the input data structure.
    }

    /* Send secondary reference voltage. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SEC_REF_VOLT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Copy IC temperature to the input data structure. */
    p_meas_data->v_sec_ref = s_spi_msg.response[0].data;

    /* Send Read Internal Temperature Command. */
    s_spi_msg.command.reg_address = BFE_REG_IC_TEMP;

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Copy IC temperature to the input data structure. */
    p_meas_data->v_ic_temp = s_spi_msg.response[0].data;

    /* Read external temperature inputs using register address offset. */
    for(uint32_t i = 0; i < BFE_TEMP_INPUTS; i++ )
    {
        s_spi_msg.command.reg_address = BFE_REG_EXT_TEMP_IN1_VOLT + i;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        p_meas_data->v_ext_temps[i] = s_spi_msg.response[0].data; // Copy temperature to the input data structure.
    }

#endif // BFE_CFG_DCH_OPERATION

    return err;
}
/**********************************************************************************************************************
 * End of function R_BFE_VoltTempAllGet
 *********************************************************************************************************************/

/* Function Name: R_BFE_VAllGet */
/******************************************************************************************************************//**
 * This function performs battery and all cells voltage scan and read.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Sends a Scan Voltages Command to the stand-alone device or all devices in the stack.
 * - Confirms the reception of the Scan Voltages Command
 * - Sends commands to read battery and cell voltages
 *
 * The following output parameters are modified by this function:
 * - p_meas_data->v_batt
 * - p_meas_data->v_cells
 *
 * @pre The BFE interface should have already been opened!
 * @param[in]   p_bfe_ctrl                  Pointer to Battery Front End control structure.
 * @param[out]  p_meas_data                 Pointer to measured data structure.
 * @retval      BFE_SUCCESS                 The voltages are successfully read.
 * @retval      BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval      BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval      BMS_ERR_RESPONCE            Invalid response is received.
 * @retval      BMS_ERR_SCAN_CNTR           The Scan Command was not received.
 * @retval      BMS_ERR_...                 Inherit from bfe_spi_msg_send_resp_get().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_VAllGet(st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_meas_t * p_meas_data)
{
    e_bms_err_t err                         = BMS_SUCCESS; // Error status
    uint16_t  scan_count[BFE_CFG_STA_DEV] = {0};

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl,  BMS_ERR_INVALID_POINTER);
    BFE_ERROR_RETURN(NULL != p_meas_data, BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);

#endif // BFE_CFG_PARAM_CHECKING_EN

    FSP_PARAMETER_NOT_USED(scan_count); // Mute unused parameter

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

#if BFE_CFG_SCAN_CMND_VERIFY_EN // Read scan count.

    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SCAN_COUNT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    /* Send command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

        scan_count[i] = s_spi_msg.response[0].data;
    }

#endif // BFE_CFG_SCAN_CMND_VERIFY_EN

    /* Send Scan All Voltages Command. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_ADDRESS_ALL; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SCAN_VOLTS;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Scan processing time. */
    R_BSP_SoftwareDelay(BFE_SCAN_VOLT_PROC_US, BSP_DELAY_UNITS_MICROSECONDS);

#if BFE_CFG_SCAN_CMND_VERIFY_EN // Verify Scan Command reception.

    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SCAN_COUNT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    /* Send command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

        /* Compare counter for every device. */
        if(scan_count[i] < BFE_SCAN_CNTR_MAX)
        {
            scan_count[i]++;
        }
        else
        {
            scan_count[i] = 0;
        }

        BFE_ERROR_RETURN(scan_count[i] == s_spi_msg.response[0].data, BMS_ERR_SCAN_CNTR); // Check counter.
    }

#endif // BFE_CFG_SCAN_CMND_VERIFY_EN

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

    /* Send Read All Register Page 1 Command. */
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_ALL_CELL_VOLT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    /* Copy received data for every device. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        /* Check for valid response. */
        BFE_ERROR_RETURN(s_spi_msg.response[0].reg_address == (BFE_REG_CELL_12_VOLT & BFE_REG_PAGE_ADDR_MASK),
                                                                                            BMS_ERR_RESPONCE);

        /* Copy battery voltage to the input data structure. */
        (p_meas_data + i)->v_batt = s_spi_msg.response[BFE_VALL_VBAT_OFFST].data;

        /* Read cell voltages using register address offset. */
        for(uint16_t j = 1; j <= BFE_VALL_VBAT_OFFST; j++ )
        {

            /* Copy cell voltages to the input data structure. */
            (p_meas_data + i)->v_cells[BFE_VALL_VBAT_OFFST - j] = s_spi_msg.response[j - 1].data;
            volt_cells[i][BFE_VALL_VBAT_OFFST - j] = s_spi_msg.response[j - 1].data;

        }
    }

#else // Stand-alone operation

    /* Send Read Battery Voltage Command. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_BAT_VOLT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Copy battery voltage to the input data structure. */
    p_meas_data->v_batt = s_spi_msg.response[0].data;

    /* Read cell voltages using register address offset. */
    for(uint32_t i = 0; i < BFE_MAX_CELLS_PER_IC; i++ )
    {
        s_spi_msg.command.reg_address = BFE_REG_CELL_1_VOLT + i;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        p_meas_data->v_cells[i] = s_spi_msg.response[0].data; // Copy cell voltages to the input data structure.
        volt_cells[0][i] = s_spi_msg.response[0].data;
    }

#endif //BFE_CFG_DCH_OPERATION

    return err;
}
/**********************************************************************************************************************
 * End of function R_BFE_VAllGet
 *********************************************************************************************************************/

/* Function Name: R_BFE_VBattGet */
/******************************************************************************************************************//**
 * This function performs battery voltage scan and read.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Sends a Scan Voltages Command to the stand-alone device or all devices in the stack.
 * - Confirms the reception of the Scan Voltages Command
 * - Sends commands to read battery voltage
 *
 * The following output parameters are modified by this function:
 * - p_meas_data->v_batt
 *
 * @pre The BFE interface should have already been opened!
 * @param[in]   p_bfe_ctrl                  Pointer to Battery Front End control structure.
 * @param[out]  p_meas_data                 Pointer to measured data array.
 * @retval      BFE_SUCCESS                 The voltages are successfully read.
 * @retval      BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval      BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval      BMS_ERR_SCAN_CNTR           The Scan Command was not received.
 * @retval      BMS_ERR_...                 Inherit from bfe_spi_msg_send_resp_get().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_VBattGet(st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_meas_t * p_meas_data)
{
    e_bms_err_t err                         = BMS_SUCCESS; // Error status
    uint16_t  scan_count[BFE_CFG_STA_DEV] = {0};

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl,  BMS_ERR_INVALID_POINTER);
    BFE_ERROR_RETURN(NULL != p_meas_data, BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);

#endif // BFE_CFG_PARAM_CHECKING_EN

    FSP_PARAMETER_NOT_USED(scan_count); // Mute unused parameter

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

#if BFE_CFG_SCAN_CMND_VERIFY_EN // Read scan count.

    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SCAN_COUNT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    /* Send Command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

        scan_count[i] = s_spi_msg.response[0].data;
    }

#endif // BFE_CFG_SCAN_CMND_VERIFY_EN

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

    /* Send Scan All Voltages Command. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_ADDRESS_ALL; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SCAN_VOLTS;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Scan processing time. */
    R_BSP_SoftwareDelay(BFE_SCAN_VOLT_PROC_US, BSP_DELAY_UNITS_MICROSECONDS);

#else // Stand-alone operation

    /* Send Measure Command. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_MEASURE;
    s_spi_msg.command.data = BFE_MEAS_VOLT_BATT; // Measure battery voltage

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Measure processing time. */
    R_BSP_SoftwareDelay(BFE_MEAS_V_BATT_PROC_US, BSP_DELAY_UNITS_MICROSECONDS);

#endif //BFE_CFG_DCH_OPERATION

#if BFE_CFG_SCAN_CMND_VERIFY_EN // Verify Scan Command reception.

    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SCAN_COUNT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    /* Send command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

        /* Compare counter for every device. */
        if(scan_count[i] < BFE_SCAN_CNTR_MAX)
        {
            scan_count[i]++;
        }
        else
        {
            scan_count[i] = 0;
        }

        BFE_ERROR_RETURN(scan_count[i] == s_spi_msg.response[0].data, BMS_ERR_SCAN_CNTR); // Check counter.
    }

#endif // BFE_CFG_SCAN_CMND_VERIFY_EN

    /* Send Read Battery Voltage Command. */
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_BAT_VOLT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

    /* Send command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

        (p_meas_data + i)->v_batt = s_spi_msg.response[0].data;
        volt_batt[i] = s_spi_msg.response[0].data;
    }

#else // Stand-alone operation

    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Copy the response to the input data structure. */
    p_meas_data->v_batt = s_spi_msg.response[0].data;
    volt_batt[0] = s_spi_msg.response[0].data;

#endif //BFE_CFG_DCH_OPERATION

    return err;
}
/**********************************************************************************************************************
 * End of function R_BFE_VBattGet
 *********************************************************************************************************************/

/* Function Name: R_BFE_VMixGet */
/******************************************************************************************************************//**
 * This function performs battery, all cells and ExT1 voltage and internal temperature scan and read. (Stand-alone/
 * Daisy chain) It can be used also in case that ExT1 input of the master device is monitoring current.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Sends a Scan Mix Command to the stand-alone device or all devices in the stack.
 * - Confirms the reception of the Scan Mix Command.
 * - Sends commands to Read Battery, Cell Voltages, Internal and Temperature and ExT1 Voltage.
 *
 * The following output parameters are modified by this function:
 * - p_meas_data->v_batt
 * - p_meas_data->v_cells
 * - p_meas_data->v_ic_temp
 * - p_meas_data->v_ext_temps
 *
 * @pre The BFE interface should have already been opened!
 * @param[in]   p_bfe_ctrl                  Pointer to Battery Front End control structure.
 * @param[out]  p_meas_data                 Pointer to measured data array.
 * @retval      BFE_SUCCESS                 The temperatures and voltages successfully are read.
 * @retval      BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval      BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval      BMS_ERR_RESPONCE            Invalid response is received.
 * @retval      BMS_ERR_SCAN_CNTR           The Scan Command was not received.
 * @retval      BMS_ERR_...                 Inherit from bfe_spi_msg_send_resp_get().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_VMixGet(st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_meas_t * p_meas_data)
{
    e_bms_err_t err                         = BMS_SUCCESS; // Error status
    uint16_t  scan_count[BFE_CFG_STA_DEV] = {0};

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl,  BMS_ERR_INVALID_POINTER);
    BFE_ERROR_RETURN(NULL != p_meas_data, BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);

#endif // BFE_CFG_PARAM_CHECKING_EN

    FSP_PARAMETER_NOT_USED(scan_count); // Mute unused parameter

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

#if BFE_CFG_SCAN_CMND_VERIFY_EN // Read scan count.

    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SCAN_COUNT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

    /* Send command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

        scan_count[i] = s_spi_msg.response[0].data;
    }

#else // Stand-alone operation
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    scan_count[0] = s_spi_msg.response[0].data; // Assign scan counter.
#endif //BFE_CFG_DCH_OPERATION
#endif // BFE_CFG_SCAN_CMND_VERIFY_EN

    /* Send Scan Voltages, Internal Temperature and ExT1 Command. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_ADDRESS_ALL; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SCAN_MIXED;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Scan processing time. */
    R_BSP_SoftwareDelay(BFE_SCAN_MIXED_PROC_US, BSP_DELAY_UNITS_MICROSECONDS);

#if BFE_CFG_SCAN_CMND_VERIFY_EN // Verify Scan Command reception.

    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SCAN_COUNT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    /* Send Command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

        /* Compare counter for every device. */
        if(scan_count[i] < BFE_SCAN_CNTR_MAX)
        {
            scan_count[i]++;
        }
        else
        {
            scan_count[i] = 0;
        }

        BFE_ERROR_RETURN(scan_count[i] == s_spi_msg.response[0].data, BMS_ERR_SCAN_CNTR); // Check counter.
    }

#endif // BFE_CFG_SCAN_CMND_VERIFY_EN

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

    /* Copy received data for every device. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        /* Send Read All Register Page 1 Command. */
        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_ALL_CELL_VOLT;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        /* Check for valid response. */
        BFE_ERROR_RETURN(s_spi_msg.response[0].reg_address == (BFE_REG_CELL_12_VOLT & BFE_REG_PAGE_ADDR_MASK),
                                                                                            BMS_ERR_RESPONCE);

        /* Copy battery voltage to the input data structure. */
        (p_meas_data + i)->v_batt = s_spi_msg.response[BFE_VALL_VBAT_OFFST].data;

        /* Read cell voltages using register address offset. */
        for(uint16_t j = 1; j <= BFE_VALL_VBAT_OFFST; j++ )
        {
            /* Copy cell voltages to the input data structure. */
            (p_meas_data + i)->v_cells[BFE_VALL_VBAT_OFFST - j] = s_spi_msg.response[j - 1].data;
        }

        /* Read Internal Temperature. */
        s_spi_msg.command.reg_address = BFE_REG_IC_TEMP;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

        /* Check for valid response. */
        BFE_ERROR_RETURN(s_spi_msg.response[0].reg_address == (BFE_REG_IC_TEMP & BFE_REG_PAGE_ADDR_MASK),
                                                                                       BMS_ERR_RESPONCE);
        (p_meas_data + i)->v_ic_temp = s_spi_msg.response[0].data; // Copy response data

        /* Read ExT1 Voltage. */
        s_spi_msg.command.reg_address = BFE_REG_EXT_TEMP_IN1_VOLT;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

        /* Check for valid response. */
        BFE_ERROR_RETURN(s_spi_msg.response[0].reg_address == (BFE_REG_EXT_TEMP_IN1_VOLT & BFE_REG_PAGE_ADDR_MASK),
                                                                                                 BMS_ERR_RESPONCE);

        (p_meas_data + i)->v_ext_temps[0] = s_spi_msg.response[0].data; // Copy response data
    }

#else // Stand-alone operation

    /* Send Read Battery Voltage Command. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_BAT_VOLT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Copy battery voltage to the input data structure. */
    p_meas_data->v_batt = s_spi_msg.response[0].data;

    /* Read cell voltages using register address offset. */
    for(uint32_t i = 0; i < BFE_MAX_CELLS_PER_IC; i++ )
    {
        s_spi_msg.command.reg_address = BFE_REG_CELL_1_VOLT + i;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        p_meas_data->v_cells[i] = s_spi_msg.response[0].data; // Copy cell voltages to the input data structure.
    }

    /* Read Internal Temperature. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_IC_TEMP;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Copy temperature to the input data structure. */
    p_meas_data->v_ic_temp = s_spi_msg.response[0].data;

    /* Read ExT1 Voltage. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_EXT_TEMP_IN1_VOLT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Copy ExT1 voltage to the input data structure. */
    p_meas_data->i_batt = s_spi_msg.response[0].data;

#endif //BFE_CFG_DCH_OPERATION

    return err;
}
/**********************************************************************************************************************
 * End of function R_BFE_VMixGet
 *********************************************************************************************************************/

/* Function Name: R_BFE_TempsGet */
/******************************************************************************************************************//**
 * This function performs internal temperature and ExTn voltages scan and read.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Sends a Scan Temperatures Command to the stand-alone device or all devices in the stack.
 * - Confirms the reception of the Scan Temperatures Command.
 * - Sends commands to read Internal Temperature and ExT1-ExT4 Voltages and Second Reference Voltage.
 *
 * The following output parameters are modified by this function:
 * - p_meas_data->v_sec_ref
 * - p_meas_data->v_ic_temp
 * - p_meas_data->v_ext_temps
 *
 * @pre The BFE interface should have already been opened!
 * @param[in]   p_bfe_ctrl                  Pointer to Battery Front End control structure.
 * @param[out]  p_meas_data                 Pointer to measured data array.
 * @retval      BFE_SUCCESS                 The temperatures are successfully read.
 * @retval      BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval      BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval      BMS_ERR_RESPONCE            Invalid response is received.
 * @retval      BMS_ERR_SCAN_CNTR           The Scan Command was not received.
 * @retval      BMS_ERR_...                 Inherit from bfe_spi_msg_send_resp_get().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_TempsGet(st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_meas_t * p_meas_data)
{
    e_bms_err_t err                         = BMS_SUCCESS; // Error status
    uint16_t  scan_count[BFE_CFG_STA_DEV] = {0};

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl,  BMS_ERR_INVALID_POINTER);
    BFE_ERROR_RETURN(NULL != p_meas_data, BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);

#endif // BFE_CFG_PARAM_CHECKING_EN

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

#if BFE_CFG_SCAN_CMND_VERIFY_EN // Read scan count.

    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SCAN_COUNT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    /* Send command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

        scan_count[i] = s_spi_msg.response[0].data;
    }

#endif // BFE_CFG_SCAN_CMND_VERIFY_EN

    /* Send Scan All Temperatures Command. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_ADDRESS_ALL; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SCAN_TEMPS;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Scan processing time. */
    R_BSP_SoftwareDelay(BFE_SCAN_TEMP_PROC_US, BSP_DELAY_UNITS_MICROSECONDS);

#if BFE_CFG_SCAN_CMND_VERIFY_EN // Verify Scan Command reception.
#if BFE_CFG_STA_DEV == 1U  // Stand-alone operation

    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SCAN_COUNT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    if(scan_count[0] < BFE_SCAN_CNTR_MAX)
    {
        scan_count[0]++;
    }
    else
    {
        scan_count[0] = 0;
    }

    BFE_ERROR_RETURN(scan_count[0] == s_spi_msg.response[0].data, BMS_ERR_SCAN_CNTR); // Check counter.

#endif //BFE_CFG_DCH_OPERATION
#endif // BFE_CFG_SCAN_CMND_VERIFY_EN

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

    /* Send Read All Temperatures Command. */
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_ALL_TEMP_DATA;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    /* Copy received data for every device. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        /* Check for valid response. */
        BFE_ERROR_RETURN(s_spi_msg.response[0].reg_address == (BFE_REG_SCAN_COUNT & BFE_REG_PAGE_ADDR_MASK),
                                                                                          BMS_ERR_RESPONCE);

        /* Compare counter for every device. */
        if(scan_count[i] < BFE_SCAN_CNTR_MAX)
        {
            scan_count[i]++;
        }
        else
        {
            scan_count[i] = 0;
        }

        BFE_ERROR_RETURN(scan_count[i] == s_spi_msg.response[0].data, BMS_ERR_SCAN_CNTR); // Check counter.

        /* Copy reference voltage to the input data structure. */
        (p_meas_data + i)->v_sec_ref = s_spi_msg.response[BFE_TALL_VREF_OFFST].data;

        /* Copy internal temperature to the input data structure. */
        (p_meas_data + i)->v_ic_temp = s_spi_msg.response[BFE_TALL_TIC_OFFST].data;

        /* Read external temperatures using register address offset. */
        for(uint16_t j = 1; j <= BFE_TEMP_INPUTS; j++ )
        {
            /* Copy external temperatures to the input data structure. */
            (p_meas_data + i)->v_ext_temps[BFE_TEMP_INPUTS - j] = s_spi_msg.response[BFE_TALL_VREF_OFFST + j].data;
        }
    }

#else // Stand-alone operation

    /* Read Secondary Reference Votage. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SEC_REF_VOLT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Copy IC temperature to the input data structure. */
    p_meas_data->v_sec_ref = s_spi_msg.response[0].data;

    /* Send Read Internal Temperature Command. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_IC_TEMP;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Copy IC temperature to the input data structure. */
    p_meas_data->v_ic_temp = s_spi_msg.response[0].data;

    /* Read external temperature inputs using register address offset. */
    for(uint32_t i = 0; i < BFE_TEMP_INPUTS; i++ )
    {
        s_spi_msg.command.reg_address = BFE_REG_EXT_TEMP_IN1_VOLT + i;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        p_meas_data->v_ext_temps[i] = s_spi_msg.response[0].data; // Copy temperature to the input data structure.
    }

#endif //BFE_CFG_DCH_OPERATION

    return err;
}
/**********************************************************************************************************************
 * End of function R_BFE_TempsGet
 *********************************************************************************************************************/

/* Function Name: R_BFE_FaultsAllRead */
/******************************************************************************************************************//**
 * This function reads the fault registers.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Calls fault read static function.
 *
 * The following output parameters are modified by this function:
 * - p_faults->flt_oscillator
 * - p_faults->flt_wdt_timout
 * - p_faults->flt_ow_vbat
 * - p_faults->flt_ow_vss
 * - p_faults->flt_parity
 * - p_faults->flt_v_ref
 * - p_faults->flt_v_reg
 * - p_faults->flt_temp_mux
 * - p_faults->flt_over_temp
 * - p_faults->flt_overvolt
 * - p_faults->flt_undervolt
 * - p_faults->flt_open_wire
 *
 * @pre The BFE interface should have already been opened!
 * @param[in]   p_bfe_ctrl                  Pointer to Battery Front End control structure.
 * @param[out]  p_faults                    Pointer to faults data structure.
 * @retval      BFE_SUCCESS                 The fault registers are successfully read.
 * @retval      BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval      BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval      BMS_ERR_...                 Inherit from bfe_faults_read().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_FaultsAllRead(st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_faults_t * p_faults)
{
#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl, BMS_ERR_INVALID_POINTER);
    BFE_ERROR_RETURN(NULL != p_faults,   BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);

#endif // BFE_CFG_PARAM_CHECKING_EN

    return bfe_faults_read(p_bfe_ctrl, p_faults);
}
/**********************************************************************************************************************
 * End of function R_BFE_FaultsAllRead
 *********************************************************************************************************************/

/* Function Name: R_BFE_FaultsCheck */
/******************************************************************************************************************//**
 * This function checks the Battery Front End for fault state.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Checks the fault pin for assertion and returns fault if asserted.
 * - Read all fault status registers in the stack and check for asserted bits.
 *
 * @pre The BFE interface should have already been opened!
 * @param[in]   p_bfe_ctrl                  Pointer to Battery Front End control structure.
 * @retval      BFE_SUCCESS                 The fault in is not asserted.
 * @retval      BMS_FAULT                   The fault in is asserted.
 * @retval      BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval      BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval      BMS_ERR_GPIO_READ           Error in GPIO port read.
 * @retval      BMS_ERR_...                 Inherit from bfe_faults_read().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_FaultsCheck(st_bfe_ctrl_t * p_bfe_ctrl)
{
    e_bms_err_t      err         = BMS_SUCCESS; // Error status
    fsp_err_t      fsp_err     = FSP_SUCCESS;       // Error status for fsp functions
    bsp_io_level_t flt_pin_val = BSP_IO_LEVEL_HIGH; // Read fault pin value

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl, BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);

#endif // BFE_CFG_PARAM_CHECKING_EN

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

    FSP_PARAMETER_NOT_USED(p_bfe_ctrl); // Mute unused parameter

    fsp_err = g_ioport.p_api->pinRead(&g_ioport_ctrl, BFE_FAULT_PIN, &flt_pin_val);
    BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_GPIO_READ); // Check for errors.

    if(flt_pin_val == BSP_IO_LEVEL_LOW)
    {
        /* Read error. */
        return BMS_FAULT;
    }
    else
    {
        ;
    }

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

    if(p_bfe_ctrl->status.in_sleep_mode == false)
    {
        /* Read Fault Status Register. */
        for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
        {
            s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

            /* Read Fault Status Register. */
            s_spi_msg.r_w_data = BFE_READ_REG;
            s_spi_msg.command.reg_address = BFE_REG_FAULT_STATUS;
            s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
            BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

            BFE_ERROR_RETURN(ZERO == s_spi_msg.response[0].data, BMS_FAULT); //Check for fault bits.
        }
    }
    else
    {
        ;
    }

#endif //BFE_CFG_DCH_OPERATION

    return err;
}
/**********************************************************************************************************************
 * End of function R_BFE_FaultsCheck
 *********************************************************************************************************************/

/* Function Name: R_BFE_FaultsAllClr */
/******************************************************************************************************************//**
 * This function clears all faults.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Sends a command to clear Fault Status, Over-temperature, Overvoltage, Undervoltage and Open Wire Registers
 * in stand-alone device or all devices in the stack.
 * - Resets the fault filter.
 * - Cleans the faults data structure.
 * - Checks if fault pin is deasserted.
 *
 * @pre The BFE interface should have already been opened!
 * @warning The fault data structure has to be cleaned manually!
 * @param[in]   p_bfe_ctrl                  Pointer to Battery Front End control structure.
 * @retval      BFE_SUCCESS                 All faults are successfully cleared.
 * @retval      BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval      BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval      BMS_ERR_REG_VERIFY          Register write verification error.
 * @retval      BMS_ERR_GPIO_READ           Error in GPIO port read.
 * @retval      BMS_ERR_FLT_DEASSERT        Fault was not cleared.
 * @retval      BMS_ERR_...                 Inherit from bfe_spi_msg_send_resp_get().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_FaultsAllClr(st_bfe_ctrl_t * p_bfe_ctrl)
{
    e_bms_err_t      err         = BMS_SUCCESS;       // Error status
    fsp_err_t      fsp_err     = FSP_SUCCESS;       // Error status for fsp functions
    bsp_io_level_t flt_pin_val = BSP_IO_LEVEL_HIGH; // Read fault pin value

    uint16_t       tmp1 = 0;

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl, BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);

#endif // BFE_CFG_PARAM_CHECKING_EN

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

    /* Read fault registers. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        /* Clear Over-temperature Fault Register. */
        s_spi_msg.r_w_data = BFE_WRITE_REG;
        s_spi_msg.command.reg_address = BFE_REG_OVER_TEMP_FAULT;
        s_spi_msg.command.data = ZERO;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        /* Clear Overvoltage Fault Register. */
        s_spi_msg.r_w_data = BFE_WRITE_REG;
        s_spi_msg.command.reg_address = BFE_REG_OVERVOLT_FAULT;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        /* Clear Undervoltage Fault Register. */
        s_spi_msg.r_w_data = BFE_WRITE_REG;
        s_spi_msg.command.reg_address = BFE_REG_UNDERVOLT_FAULT;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        /* Clear Open Wire Fault Register. */
        s_spi_msg.r_w_data = BFE_WRITE_REG;
        s_spi_msg.command.reg_address = BFE_REG_OPEN_WIRE_FAULT;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        /* Clear Fault Status Register. */
        s_spi_msg.r_w_data = BFE_WRITE_REG;
        s_spi_msg.command.reg_address = BFE_REG_FAULT_STATUS;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

#if BFE_CFG_REG_WRITE_VERIFY_EN // Verify register write
#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

        /* Read all faults. */
        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_ALL_FAULT_DATA;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        BFE_ERROR_RETURN((s_spi_msg.response[0].data == ZERO)
                      && (s_spi_msg.response[2].data == ZERO)
                      && (s_spi_msg.response[4].data == ZERO)
                      && (s_spi_msg.response[5].data == ZERO)
                      && (s_spi_msg.response[6].data == ZERO), BMS_ERR_REG_VERIFY);

#else // Stand-alone operation

        s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_OVER_TEMP_FAULT;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        BFE_ERROR_RETURN(s_spi_msg.response[0].data == ZERO, BMS_ERR_REG_VERIFY);

        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_OVERVOLT_FAULT;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        BFE_ERROR_RETURN(s_spi_msg.response[0].data == ZERO, BMS_ERR_REG_VERIFY);

        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_UNDERVOLT_FAULT;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        BFE_ERROR_RETURN(s_spi_msg.response[0].data == ZERO, BMS_ERR_REG_VERIFY);

        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_OPEN_WIRE_FAULT;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        BFE_ERROR_RETURN(s_spi_msg.response[0].data == ZERO, BMS_ERR_REG_VERIFY);

        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_FAULT_STATUS;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        BFE_ERROR_RETURN(s_spi_msg.response[0].data == ZERO, BMS_ERR_REG_VERIFY);

#endif //BFE_CFG_DCH_OPERATION
#endif // BFE_CFG_REG_WRITE_VERIFY_EN

        /* Reset fault filter. Change and then revert totalizer value. */
        tmp1 = 0;

        /* Casting is added to ensure data size after integer promotion. */
        if(p_bfe_ctrl->setup.flt_tot_samples != BFE_TOT_128_SMPL)
        {
            tmp1 |= (uint16_t) (BFE_TOT_128_SMPL << BFE_FSR_TOT_OFFSET);
        }
        else
        {
            tmp1 |= (uint16_t) (BFE_TOT_1_SMPL << BFE_FSR_TOT_OFFSET);
        }

        /* Casting is added to ensure data size after integer promotion. */
        tmp1 |= (uint16_t) ((BFE_ALL_TEMPS_BIT_MASK & p_bfe_ctrl->setup.temp_flt_mon[0]) << BFE_FSR_EOB_OFFSET);
        s_spi_msg.r_w_data = BFE_WRITE_REG;
        s_spi_msg.command.reg_address = BFE_REG_FAULT_SETUP;
        s_spi_msg.command.data = tmp1;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        tmp1 = 0;

        /* Casting is added to ensure data size after integer promotion. */
        tmp1 |= (uint16_t) (p_bfe_ctrl->setup.flt_tot_samples << BFE_FSR_TOT_OFFSET);
        tmp1 |= (uint16_t) ((BFE_ALL_TEMPS_BIT_MASK & p_bfe_ctrl->setup.temp_flt_mon[0]) << BFE_FSR_EOB_OFFSET);
        s_spi_msg.r_w_data = BFE_WRITE_REG;
        s_spi_msg.command.reg_address = BFE_REG_FAULT_SETUP;
        s_spi_msg.command.data = tmp1;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

#if BFE_CFG_REG_WRITE_VERIFY_EN // Verify register write
        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_FAULT_SETUP;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        BFE_ERROR_RETURN(s_spi_msg.response[0].data == tmp1, BMS_ERR_REG_VERIFY);
#endif // BFE_CFG_REG_WRITE_VERIFY_EN

        /* Send Calculate Register Checksum Command. */
        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_CALC_REG_CKSM; // Check checksum
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.
    }

    fsp_err = g_ioport.p_api->pinRead(&g_ioport_ctrl, BFE_FAULT_PIN, &flt_pin_val);
    BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_GPIO_READ); // Check for errors.

    BFE_ERROR_RETURN(BSP_IO_LEVEL_HIGH == flt_pin_val, BMS_ERR_FLT_DEASSERT);

    return err;
}
/**********************************************************************************************************************
 * End of function R_BFE_FaultsAllClr
 *********************************************************************************************************************/

/* Function Name: R_BFE_BalanceCtrl */
/******************************************************************************************************************//**
 * This function selects cells and enables or inhibits cell balancing.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Sends a command to select cells for balancing.
 * - Sends a command to enable or inhibit cell balancing to a stand-alone device or all devices in the stack.
 * - Verifies that the commands are received.
 *
 * The following input parameters need to be set:
 * - p_bfe_ctrl->balance_cell_sel
 * - p_bfe_ctrl->bal_pattern
 * - bal_ctrl
 *
 * The following output parameters are modified by this function:
 * - p_bfe_ctrl->status.balancing
 *
 * @pre The BFE interface should have already been opened!
 * @param[in]       p_bfe_ctrl                  Pointer to Battery Front End control structure.
 * @param[in]       bal_ctrl                    Select action - enable or inhibit.
 * @retval          BFE_SUCCESS                 The cell balancing is successfully enabled/ inhibited.
 * @retval          BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval          BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval          BMS_ERR_BALANCE             The balance cannot be enabled in sleep mode.
 * @retval          BMS_ERR_REG_VERIFY          Register write verification error.
 * @retval          BMS_ERR_...                 Inherit from bfe_spi_msg_send_resp_get().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_BalanceCtrl(st_bfe_ctrl_t * p_bfe_ctrl, e_bfe_ctrl_bal_t bal_ctrl)
{
    e_bms_err_t err       = BMS_SUCCESS; // Error status
    uint16_t  bal_cells = 0;

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl, BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);

#endif // BFE_CFG_PARAM_CHECKING_EN

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

    /* Send Balance Inhibit Command. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_ADDRESS_ALL; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_BAL_INHIBIT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

#if BFE_CFG_REG_WRITE_VERIFY_EN // Verify register write

    /* Send command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        /* Send Balance Enable Command. */
        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_BAL_SETUP;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        BFE_ERROR_RETURN(ZERO == (s_spi_msg.response[0].data & BFE_BSR_BEN_BIT_MASK), BMS_ERR_REG_VERIFY);
    }

#endif // BFE_CFG_REG_WRITE_VERIFY_EN

    p_bfe_ctrl->status.balancing = false; // Update balancing flag

    if(bal_ctrl == BFE_CELL_BALANCE_ENABLE)
    {
        /* Check for sleep mode. */
        BFE_ERROR_RETURN(false == p_bfe_ctrl->status.in_sleep_mode, BMS_ERR_BALANCE);

        /* Send command to each device from the stack. */
        for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
        {
            s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

            /* Cells for fault monitoring selection. */
            s_spi_msg.r_w_data = BFE_WRITE_REG;
            s_spi_msg.command.reg_address = BFE_REG_BAL_STATUS;
            bal_cells = (p_bfe_ctrl->balance_cell_sel[i] & p_bfe_ctrl->setup.cells_cfg[i]) & p_bfe_ctrl->bal_pattern;
            s_spi_msg.command.data = bal_cells;

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
            BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

#if BFE_CFG_REG_WRITE_VERIFY_EN // Verify register write
            s_spi_msg.r_w_data = BFE_READ_REG;
            s_spi_msg.command.reg_address = BFE_REG_BAL_STATUS;
            s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
            BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

            BFE_ERROR_RETURN(s_spi_msg.response[0].data == bal_cells, BMS_ERR_REG_VERIFY);
#endif // BFE_CFG_REG_WRITE_VERIFY_EN
        }

        /* Send Balance Enable Command. */
        s_spi_msg.command.device_address = BFE_DAISY_CHAIN_ADDRESS_ALL; // Assign a device address.
        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_BAL_ENABLE;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

#if BFE_CFG_REG_WRITE_VERIFY_EN // Verify register write

        /* Send command to each device from the stack. */
        for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
        {
            s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

            /* Send Balance Enable Command. */
            s_spi_msg.r_w_data = BFE_READ_REG;
            s_spi_msg.command.reg_address = BFE_REG_BAL_SETUP;
            s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
            BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

            BFE_ERROR_RETURN(ZERO < (s_spi_msg.response[0].data & BFE_BSR_BEN_BIT_MASK), BMS_ERR_REG_VERIFY);
        }

#endif // BFE_CFG_REG_WRITE_VERIFY_EN

        p_bfe_ctrl->status.balancing = true; // Update balancing flag
    }
    else
    {
        ;
    }

    /* Send Calculate and Check Register Checksum Command. */
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    /* Send command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        /* Send Calculate Register Checksum Command. */
        s_spi_msg.command.reg_address = BFE_CALC_REG_CKSM; // Check checksum

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.
    }

    return err;
}
/**********************************************************************************************************************
 * End of function R_BFE_BalanceCtrl
 *********************************************************************************************************************/

/* Function Name: R_BFE_ConfRegCheck */
/******************************************************************************************************************//**
 * This function checks the configuration registers (Page 2) for corruption.
 * The recalculated checksum is automatically compared to the checksum stored in BFE internal memory. If both are
 * not equal a fault response is initiated.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Send a command to recalculate Page 2 checksum.
 *
 * @pre The BFE interface should have already been opened!
 * @warning The 'Calc Register Checksum' command has to be send each time a Page 2 register is modified.
 * @param[in]   p_bfe_ctrl                  Pointer to Battery Front End control structure.
 * @retval      BFE_SUCCESS                 The compare command is successfully sent.
 * @retval      BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval      BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval      BMS_ERR_...                 Inherit from bfe_spi_msg_send_resp_get().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_ConfRegCheck(st_bfe_ctrl_t * p_bfe_ctrl)
{
    e_bms_err_t err = BMS_SUCCESS;    // Error status

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl, BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization.. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);

#endif // BFE_CFG_PARAM_CHECKING_EN

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

    /* Send Check Register Checksum Command. */
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    /* Send command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t) (i + 1); // Assign a device address.

        /* Send Check Register Checksum Command. */
        s_spi_msg.command.reg_address = BFE_CHECK_REG_CKSM;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

        /* Send Calculate Register Checksum Command. */
        s_spi_msg.command.reg_address = BFE_CALC_REG_CKSM; // Check checksum

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.
    }

    return err;
}
/**********************************************************************************************************************
 * End of function R_BFE_ConfRegCheck
 *********************************************************************************************************************/

/* Function Name: R_BFE_DeviceSetup */
/******************************************************************************************************************//**
 * This function writes and verifies all setup registers.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Calls device setup static function.
 *
 * The following input parameters need to be set:
 * - p_bfe_ctrl->wd_time
 * - p_bfe_ctrl->setup.i_wire_scan
 * - p_bfe_ctrl->setup.balance_mode
 * - p_bfe_ctrl->setup.temp_flt_mon
 * - p_bfe_ctrl->setup.flt_tot_samples
 * - p_bfe_ctrl->setup.overvoltage_limit
 * - p_bfe_ctrl->setup.undervoltage_limit
 * - p_bfe_ctrl->setup.ext_temp_limit
 * - p_bfe_ctrl->setup.cells_cfg
 *
 * The following output parameters are modified by this function:
 * - p_bfe_ctrl->status.ic_temp_limit
 * - p_bfe_ctrl->status.v_trim
 *
 * @pre The BFE interface should have already been opened!
 * @param[in]   p_bfe_ctrl                  Pointer to Battery Front End control structure.
 * @retval      BFE_SUCCESS                 The device is successfully configured.
 * @retval      BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval      BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval      BMS_ERR_...                 Inherit from bfe_setup().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_DeviceSetup(st_bfe_ctrl_t * p_bfe_ctrl)
{
#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl, BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization.. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);

#endif // BFE_CFG_PARAM_CHECKING_EN

    return bfe_setup(p_bfe_ctrl);
}
/**********************************************************************************************************************
 * End of function R_BFE_DeviceSetup
 *********************************************************************************************************************/

/* Function Name: R_BFE_WatchdogCtrl */
/******************************************************************************************************************//**
 * This function enables or disables the watchdog timer and selects time.
 * Set a watchdog time in p_bfe_ctrl->wd_time. When '0' the watchdog is disabled.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Calls a watchdog control static function.
 *
 * The following input parameters need to be set:
 * - p_bfe_ctrl->wd_time
 *
 * @pre The BFE interface should have already been opened!
 * @param[in]   p_bfe_ctrl                  Pointer to Battery Front End control structure.
 * @retval      BFE_SUCCESS                 The watchdog timer is successfully enabled/disabled.
 * @retval      BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval      BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval      BMS_ERR_WDT_TIME            Invalid watchdog time.
 * @retval      BMS_ERR_...                 Inherit from bfe_watchdog().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_WatchdogCtrl(st_bfe_ctrl_t * p_bfe_ctrl)
{
#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl, BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);

    /* Check watchdog time. */
    BFE_ERROR_RETURN(BFE_WDT_MAX_TIME >= p_bfe_ctrl->wd_time, BMS_ERR_WDT_TIME);

#endif // BFE_CFG_PARAM_CHECKING_EN

    return bfe_watchdog(p_bfe_ctrl);
}
/**********************************************************************************************************************
 * End of function R_BFE_WatchdogCtrl
 *********************************************************************************************************************/

/* Function Name: R_BFE_WatchdogReset */
/******************************************************************************************************************//**
 * This function resets the watchdog timer.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Sends ACK Command to all devices in the stack to reset watchdog counters.
 *
 * @pre The BFE interface should have already been opened!
 * @param[in]   p_bfe_ctrl                  Pointer to Battery Front End control structure.
 * @retval      BFE_SUCCESS                 The watchdog timer is successfully reseted.
 * @retval      BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval      BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval      BMS_ERR_WDT_RESET           Watchdog timeout was not reset.
 * @retval      BMS_ERR_...                 Inherit from bfe_spi_msg_send_resp_get().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_WatchdogReset(st_bfe_ctrl_t * p_bfe_ctrl)
{
    e_bms_err_t err = BMS_SUCCESS; // Error status

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl, BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);

#endif // BFE_CFG_PARAM_CHECKING_EN

    FSP_PARAMETER_NOT_USED(p_bfe_ctrl); // Mute unused parameter

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

    /* Send ACK Command to each device. */
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_ACK;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    /* Send command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

        BFE_ERROR_RETURN(s_spi_msg.response[0].reg_address == (BFE_REG_ACK & BFE_REG_PAGE_ADDR_MASK),
                                                                                  BMS_ERR_WDT_RESET);
    }

#endif //BFE_CFG_DCH_OPERATION

    return err;
}
/**********************************************************************************************************************
 * End of function R_BFE_WatchdogReset
 *********************************************************************************************************************/

/* Function Name: R_BFE_Scan */
/******************************************************************************************************************//**
 * This function sends a Scan Command depending on the cmnd_type input parameter.
 * Set a command type to Scan Voltages, Temperatures, Mixed or All.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Sends the selected Scan Command to the BFE.
 * - Confirms the reception of the Scan Command.
 *
 * The following output parameters are modified by this function:
 * - cmnd_type
 *
 * @pre The BFE is opened.
 * @param[in]   p_bfe_ctrl                  Pointer to Battery Front End control structure.
 * @param[in]   cmnd_type                   The Scan Command type.
 * @retval      BFE_SUCCESS                 The Scan All Command was successfully sent.
 * @retval      BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval      BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval      BMS_ERR_SCAN_TYPE           The Scan Command type input parameter is incorrect.
 * @retval      BMS_ERR_SCAN_CNTR           Scan Command was not received.
 * @retval      BMS_ERR_...                 Inherit from bfe_spi_msg_send_resp_get().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_Scan(st_bfe_ctrl_t * p_bfe_ctrl, e_bfe_scan_cmnd_types_t cmnd_type)
{
    e_bms_err_t err                         = BMS_SUCCESS; // Error status
    uint16_t  scan_count[BFE_CFG_STA_DEV] = {0};

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl, BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);

#endif // BFE_CFG_PARAM_CHECKING_EN

    FSP_PARAMETER_NOT_USED(p_bfe_ctrl); // Mute unused parameter
    FSP_PARAMETER_NOT_USED(scan_count); // Mute unused parameter

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

#if BFE_CFG_SCAN_CMND_VERIFY_EN // Read scan count.

    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SCAN_COUNT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    /* Send command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

        scan_count[i] = s_spi_msg.response[0].data;
    }

#endif // BFE_CFG_SCAN_CMND_VERIFY_EN

    /* Send Scan Command. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_ADDRESS_ALL; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    switch(cmnd_type)
    {
        case BFE_SCAN_VOLTAGES: // Scan all battery and cells voltages
        {
            s_spi_msg.command.reg_address = BFE_REG_SCAN_VOLTS;

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
            BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

            /* Scan processing time. */
            R_BSP_SoftwareDelay(BFE_SCAN_VOLT_PROC_US, BSP_DELAY_UNITS_MICROSECONDS);

            break;
        }

        case BFE_SCAN_TEMPS: // Scan all temperatures
        {
            s_spi_msg.command.reg_address = BFE_REG_SCAN_TEMPS;

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
            BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

            /* Scan processing time. */
            R_BSP_SoftwareDelay(BFE_SCAN_TEMP_PROC_US, BSP_DELAY_UNITS_MICROSECONDS);

            break;
        }
        case BFE_SCAN_MIXED: // Scan all voltages, IC temperature and ExT1
        {
            s_spi_msg.command.reg_address = BFE_REG_SCAN_MIXED;

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
            BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

            /* Scan processing time. */
            R_BSP_SoftwareDelay(BFE_SCAN_MIXED_PROC_US, BSP_DELAY_UNITS_MICROSECONDS);

            break;
        }
        case BFE_SCAN_WIRES: // Scan all wires
        {
            s_spi_msg.command.reg_address = BFE_REG_SCAN_WIRES;

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
            BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

            /* Scan processing time. */
            R_BSP_SoftwareDelay(BFE_SCAN_WIRES_PROCE_US, BSP_DELAY_UNITS_MICROSECONDS);

            break;
        }

        case BFE_SCAN_ALL: // Scan all temperatures, battery and cell voltages
        {
            s_spi_msg.command.reg_address = BFE_REG_SCAN_ALL;

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
            BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

            /* Scan processing time. */
            R_BSP_SoftwareDelay(BFE_SCAN_ALL_PROC_US, BSP_DELAY_UNITS_MICROSECONDS);

            break;
        }
        default:
        {
            BFE_ERROR_RETURN(false, BMS_ERR_SCAN_TYPE); // Return scan type error.

            break;
        }
    }

#if BFE_CFG_SCAN_CMND_VERIFY_EN // Verify Scan Command reception.

    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SCAN_COUNT;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    /* Send command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

        /* Compare counter for every device. */
        if(scan_count[i] < BFE_SCAN_CNTR_MAX)
        {
            scan_count[i]++;
        }
        else
        {
            scan_count[i] = 0;
        }

        BFE_ERROR_RETURN(scan_count[i] == s_spi_msg.response[0].data, BMS_ERR_SCAN_CNTR); // Check counter.
    }

#endif // BFE_CFG_SCAN_CMND_VERIFY_EN

    return err;
}
/**********************************************************************************************************************
 * End of function R_BFE_Scan
 *********************************************************************************************************************/

/* Function Name: R_BFE_UserRegsisterAccess */
/******************************************************************************************************************//**
 * This function provides access to read or write in User Registers in all devices.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Reads the content of the User Registers.
 * - Writes into User Registers in every device.
 * - Verifies the content.
 * - Recalculate Page 2 registers checksum.
 *
 * The following input parameters need to be set:
 * - p_data_array
 * - data_dir
 *
 * @pre The BFE interface should have already been opened!
 * @param[in]     p_bfe_ctrl                Pointer to Battery Front End control structure.
 * @param[in]     p_data_array              Pointer to data array.
 * @param[in]     p_bfe_ctrl                Direction of the data (Read/ Write).
 *
 * @retval      BFE_SUCCESS                 The User Registers are successfully read/ written.
 * @retval      BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval      BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval      BMS_ERR_INVALID_ARGUMENT    The function has invalid input argument.
 * @retval      BMS_ERR_REG_VERIFY          Register write verification error.
 * @retval      BMS_ERR_...                 Inherit from bfe_spi_msg_send_resp_get().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_UserRegsisterAccess(st_bfe_ctrl_t *  p_bfe_ctrl,
                                      uint16_t *       p_data_array,
                                      e_bfe_data_dir_t data_dir)
{
    e_bms_err_t err  = BMS_SUCCESS; // Error status
    uint16_t  tmp1 = 0;

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl,  BMS_ERR_INVALID_POINTER);
    BFE_ERROR_RETURN(NULL != p_data_array, BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);

#endif // BFE_CFG_PARAM_CHECKING_EN

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

    if(data_dir == BFE_READ_REG)
    {

        /* Send command to each device from the stack. */
        for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
        {
            s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

            /* Read first User Register value. */
            s_spi_msg.r_w_data = BFE_READ_REG;
            s_spi_msg.command.reg_address = BFE_REG_USER_REG1;
            s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
            BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

            p_data_array[i * BFE_USR_REG_NUM] = s_spi_msg.response[0].data;

            /* Read second User Register value. */
            s_spi_msg.r_w_data = BFE_READ_REG;
            s_spi_msg.command.reg_address = BFE_REG_USER_REG2;
            s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
            BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

            p_data_array[i * BFE_USR_REG_NUM + 1] = s_spi_msg.response[0].data;
        }
    }
    else if(data_dir == BFE_WRITE_REG)
    {
        /* Send command to each device from the stack. */
        for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
        {
            s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

            /* Write and verify first User Register value. */
            s_spi_msg.command.reg_address = BFE_REG_USER_REG1;

            s_spi_msg.r_w_data = BFE_WRITE_REG;
            tmp1 = p_data_array[i * BFE_USR_REG_NUM] & BFE_14BIT_FULL_REG;
            s_spi_msg.command.data = tmp1;

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
            BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

            s_spi_msg.r_w_data = BFE_READ_REG;
            s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
            BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

            BFE_ERROR_RETURN(s_spi_msg.response[0].data == tmp1, BMS_ERR_REG_VERIFY);

            /* Write and verify second User Register value. */
            s_spi_msg.command.reg_address = BFE_REG_USER_REG2;

            s_spi_msg.r_w_data = BFE_WRITE_REG;
            tmp1 = p_data_array[i * BFE_USR_REG_NUM + 1] & BFE_14BIT_FULL_REG;
            s_spi_msg.command.data = tmp1;

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
            BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

            s_spi_msg.r_w_data = BFE_READ_REG;
            s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
            BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

            BFE_ERROR_RETURN(s_spi_msg.response[0].data == tmp1, BMS_ERR_REG_VERIFY);

            /* Send Calculate Register Checksum Command. */
            s_spi_msg.command.reg_address = BFE_CALC_REG_CKSM; // Check checksum

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
            BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.
        }
    }
    else
    {
        err = BMS_ERR_INVALID_ARGUMENT;
    }

    return err;
}
/**********************************************************************************************************************
 * End of function R_BFE_UserRegsisterAccess
 *********************************************************************************************************************/

/* Function Name: R_BFE_SingleRegisterAccess */
/******************************************************************************************************************//**
 * This function provides selects a custom command to single register of a particular device from the stack.
 * It does not support multiple response commands.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Sends a command.
 * - Copies the response.
 *
 * The following input parameters need to be set:
 * - p_dev_addr
 * - p_reg_addr
 * - p_reg_data
 * - r_w_data
 *
 * The following output parameters are modified by this function:
 * - p_dev_addr
 * - p_reg_addr
 * - p_reg_data
 * - r_w_data
 *
 * @pre The BFE interface should have already been opened!
 * @param[in]     p_bfe_ctrl                Pointer to Battery Front End control structure.
 * @param[in]     p_dev_addr                Pointer to device address.
 * @param[in]     p_reg_addr                Pointer to register address.
 * @param[in]     p_reg_data                Pointer to data.
 * @param[in]     p_bfe_ctrl                Direction of the data (Read/ Write).
 *
 * @retval      BFE_SUCCESS                 The command is successfully sent.
 * @retval      BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval      BMS_ERR_INITIALIZATION      The BFE interface is not initialized.
 * @retval      BMS_ERR_...                 Inherit from bfe_spi_msg_send_resp_get().
 *********************************************************************************************************************/
e_bms_err_t R_BFE_SingleRegisterAccess(st_bfe_ctrl_t *            p_bfe_ctrl,
                                       uint16_t *              p_dev_addr,
                                       uint32_t *              p_reg_addr,
                                       uint16_t *              p_reg_data,
                                       e_bfe_data_dir_t          r_w_data)
{
    e_bms_err_t err = BMS_SUCCESS; // Error status

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.



    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_bfe_ctrl, BMS_ERR_INVALID_POINTER);

    /* Verify BFE initialization. */
    BFE_ERROR_RETURN(BFE_INITILIZED == p_bfe_ctrl->status.initialized, BMS_ERR_INITIALIZATION);



#endif // BFE_CFG_PARAM_CHECKING_EN

    FSP_PARAMETER_NOT_USED(p_bfe_ctrl); // Mute unused parameter

    /* Send command. */
    s_spi_msg.command.device_address = (e_bfe_dev_addr_t) (*p_dev_addr);
    s_spi_msg.r_w_data = r_w_data;
    s_spi_msg.command.reg_address = *p_reg_addr;
    s_spi_msg.command.data = *p_reg_data;

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.

    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Copy response. */
    *p_dev_addr = s_spi_msg.response[0].device_address;
    *p_reg_addr = s_spi_msg.response[0].reg_address;
    *p_reg_data = s_spi_msg.response[0].data;

    return err;
}
/**********************************************************************************************************************
 * End of function R_BFE_SingleRegisterAccess
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: bfe_identify
 * Description  : Runs identification procedure of the stack or/ and reads serial numbers.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Runs stack identification procedure (daisy chain operation).
 * - Reads the serial number of the stand-alone device or serial numbers of all devices in the stack.
 *
 * Arguments    : p_bfe_ctrl                Pointer to Battery Front End control structure.
 * Return Value : BFE_SUCCESS               The stack was successfully identified or/ and serial numbers were acquired.
 * Return Value : BMS_ERR_IDENT_ATTEMPTS    Three unsuccessful identification attempts were made.
 * Return Value : BMS_ERR_INVALID_STACK     Invalid stack size is detected.
 * Return Value : BMS_ERR_INVALID_CONF      Invalid configuration is detected.
 * Return Value : BFE_ERR_...               Inherit from bfe_spi_msg_send_resp_get().
 *********************************************************************************************************************/
static e_bms_err_t bfe_identify(st_bfe_ctrl_t * p_bfe_ctrl)
{
    e_bms_err_t err = BMS_SUCCESS; // Error status

    bool identified = false;       // Stack is identified

    FSP_PARAMETER_NOT_USED(identified);

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation


    p_bfe_ctrl->status.identify_ctr = 0;

    while(!identified)
    {

        if(p_bfe_ctrl->status.identify_ctr > 0)
        {

            /* Send Reset Command. */
            s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign master device address.
            s_spi_msg.r_w_data = BFE_READ_REG;
            s_spi_msg.command.reg_address = BFE_REG_RESET;
            s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.

            BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.
        }
        else
        {
             ;
        }

        /* Send Identify Command. */
        s_spi_msg.command.device_address = BFE_DAISY_CHAIN_IDENTIFY;
        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_IDENTIFY;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.

        BFE_ERROR_RETURN((BMS_SUCCESS == err) || (BMS_ERR_ACK == err), err); // Check for errors.
        dd = (BFE_REG_ACK & BFE_REG_PAGE_ADDR_MASK);
        /* Check for ACK. */
        if(s_spi_msg.response[0].reg_address == (BFE_REG_ACK & BFE_REG_PAGE_ADDR_MASK))
        {
            identified = true;
            identify_check = 1;
        }
        else
        {
            identify_check = 0;
        }

        /* Increment identification attempts counter. */
        p_bfe_ctrl->status.identify_ctr++;

        /* Check identification attempts. */
        BFE_ERROR_RETURN(BFE_CFG_STACK_IDENT_MAX > p_bfe_ctrl->status.identify_ctr, BMS_ERR_IDENT_ATTEMPTS);
    }

    for(uint16_t i = 2; i <= BFE_MAX_STACKED_DEV; i++ )
    {

        /* Send Identify particular stack device command. */
        s_spi_msg.command.device_address = BFE_DAISY_CHAIN_IDENTIFY;
        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_IDENTIFY;
        s_spi_msg.command.data = i & BFE_MAX_6_BIT_DATA_MASK;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        /* Check top device response. */
        if((s_spi_msg.response[0].data & BFE_IDENTIFY_COMS_SEL_MASK) == BFE_COMS_SEL_TOP_DEV)
        {
            /* Check detected stack size. */
            BFE_ERROR_RETURN(i == BFE_CFG_STA_DEV, BMS_ERR_INVALID_STACK);
            BFE_ERROR_RETURN(ONE < i, BMS_ERR_INVALID_CONF);
            BFE_ERROR_RETURN(BFE_MAX_STACKED_DEV >= i, BMS_ERR_INVALID_CONF);

            p_bfe_ctrl->status.stacked_devs = i;

            break;
        }
        else
        {
            /* Check top device not detected. */
            BFE_ERROR_RETURN(i < BFE_MAX_STACKED_DEV, BMS_ERR_INVALID_STACK);
        }
    }

    /* Send Identify particular stack device command. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_IDENTIFY;
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_IDENTIFY;
    s_spi_msg.command.data = BFE_IDENTIFY_EXIT;

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Check for ACK. */
    BFE_ERROR_RETURN(s_spi_msg.response[0].reg_address == (BFE_REG_ACK & BFE_REG_PAGE_ADDR_MASK),
                                                                         BMS_ERR_IDENT_ATTEMPTS);

    /* Acquire serial number 0. */
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SERIAL_NUMBER0;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    /* Send command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {

        s_spi_msg.command.device_address = (e_bfe_dev_addr_t) (i + 1); // Assign a device address.

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);   // Check for errors.

        p_bfe_ctrl->status.serial_number[i] = s_spi_msg.response[0].data;
    }

    /* Acquire serial number 1. */
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SERIAL_NUMBER1;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    /* Send command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t) (i + 1); // Assign a device address.

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

        /* Casting is used to ensure serial number length after promotion. */
        p_bfe_ctrl->status.serial_number[i] |= (uint32_t) (s_spi_msg.response[0].data << 16);
    }

#else

    p_bfe_ctrl->status.stacked_devs = ONE; // Assign single device

    /* Acquire serial number. */
    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SERIAL_NUMBER0;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    p_bfe_ctrl->status.serial_number[0] = s_spi_msg.response[0].data; // Assign serial number.

    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_SERIAL_NUMBER1;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Assign serial number. Casting is used to ensure serial number length after integer promotion. */
    p_bfe_ctrl->status.serial_number[0] |= (uint32_t) (s_spi_msg.response[0].data << 16);

#endif //BFE_CFG_DCH_OPERATION

    return err;
}
/**********************************************************************************************************************
 * End of function bfe_identify
 *********************************************************************************************************************/


/**********************************************************************************************************************
 * Function Name: bfe_reset
 * Description  : Sends a reset command to the Battery Front End.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Sends a Reset Command to the stand-alone device or all devices in the stack.
 *
 * Arguments    : p_bfe_ctrl                Pointer to Battery Front End control structure.
 * Return Value : BFE_SUCCESS               Battery Front End is successfully reseted.
 * Return Value : BFE_ERR_...               Inherit from bfe_spi_msg_send_resp_get().
 *********************************************************************************************************************/
static e_bms_err_t bfe_reset(st_bfe_ctrl_t * p_bfe_ctrl)
{
    e_bms_err_t err        = BMS_SUCCESS; // Error status

    FSP_PARAMETER_NOT_USED(p_bfe_ctrl); // Mute unused parameter

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

    /* Assign data into the spi message structure. */
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_RESET;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

    /* Send command to each device from the stack. */
    for(uint16_t i = p_bfe_ctrl->status.stacked_devs; i >= 1; i-- )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)i; // Assign a device address.

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.
    }

#else // Stand-alone operation

    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

#endif //BFE_CFG_DCH_OPERATION

    /* Wait for devices to process Reset Command. */
    R_BSP_SoftwareDelay(BFE_RST_PROC_US, BSP_DELAY_UNITS_MICROSECONDS);

    return err;
}
/**********************************************************************************************************************
 * End of function bfe_reset
 *********************************************************************************************************************/


/**********************************************************************************************************************
 * Function Name: bfe_setup
 * Description  : Writes into and verifies all setup registers.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Sends a write command to set setup registers of a stand-alone device or all stacked devices.
 * - Sends a write command to set Over-temperature, Overvoltage and Undervoltage Registers of a stand-alone device
 * or all stacked devices.
 * - Sends a write command to set Balance Status and Setup Registers of a stand-alone device or all stacked devices.
 * - Sends a write command to set Watchdog/balance Time Registers of a stand-alone device or all stacked devices.
 * - Sends a write command to configure Fault Setup Registers of a stand-alone device or all stacked devices.
 * - Verifies that the commands are received.
 * - Send a command to recalculate Page 2 checksum.
 *
 * Arguments    : p_bfe_ctrl                Pointer to Battery Front End control structure.
 * Return Value : BFE_SUCCESS               Battery Front End is successfully reseted.
 * Return Value : BMS_ERR_INITIALIZATION    The BFE interface is not initialized.
 * Return Value : BMS_ERR_REG_VERIFY        Register write verification error.
 * Return Value : BMS_ERR_BALANCE_MODE      Unsupported balance mode is selected.
 * Return Value : BFE_ERR_...               Inherit from bfe_spi_msg_send_resp_get().
 *********************************************************************************************************************/
e_bms_err_t bfe_setup(st_bfe_ctrl_t * p_bfe_ctrl)
{
    e_bms_err_t err = BMS_SUCCESS;    // Error status
    uint16_t  dev_setup = 0;
    uint16_t  bal_setup = 0;
    uint16_t  bal_stat  = 0;
    uint16_t  flt_setup  = 0;
    uint16_t  flt_cells  = 0;

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

    /* Set Device Setup Register. */
    if(p_bfe_ctrl->setup.i_wire_scan == BFE_WIRE_I_SCAN_150U)
    {
        dev_setup &= (uint16_t) (~BFE_DSR_ISCN_MASK);
    }
    else if(p_bfe_ctrl->setup.i_wire_scan == BFE_WIRE_I_SCAN_1M)
    {
        dev_setup |= BFE_DSR_ISCN_MASK;
    }

    /* Cell balancing setup. */
    switch(p_bfe_ctrl->setup.balance_mode)
    {
        case BFE_BALANCE_MODE_OFF: // Cell balancing is disabled
        {
            bal_setup |= BFE_BMD_BITS_OFF; // Configure BDM bits for no cell balancing

            break;
        }

        case BFE_BALANCE_MODE_MANUAL: // The microcontroller directly controls the state of each balancing FET.
        {
            /* Status pointer is 0, balance wait time is 0, balance is disabled. */
            bal_setup |= BFE_BMD_BITS_MANUAL; // Configure BDM bits for manual cell balancing

            bal_stat = BFE_REG_BAL_MASK_NO_CELL; // No balancing FETs are on

            break;
        }

        default:
        {
            BFE_ERROR_RETURN(false, BMS_ERR_BALANCE_MODE); // Return balancing mode error.

            break;
        }
    }

    /* Fault setup.
     * Casting is added to ensure data size after integer promotion. */
    flt_setup |= (uint16_t) (p_bfe_ctrl->setup.flt_tot_samples << BFE_FSR_TOT_OFFSET);
    flt_setup |= (uint16_t) ((BFE_ALL_TEMPS_BIT_MASK & p_bfe_ctrl->setup.temp_flt_mon[0]) << BFE_FSR_EOB_OFFSET);

    /* Send command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.
        s_spi_msg.r_w_data = BFE_WRITE_REG;

        /* Set Overvoltage Limit. */
        s_spi_msg.command.reg_address = BFE_REG_OVERVOLT_LIMIT;
        s_spi_msg.command.data = p_bfe_ctrl->setup.overvoltage_limit;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);   // Check for errors.

        /* Set Undervoltage Limit. */
        s_spi_msg.command.reg_address = BFE_REG_UNDERVOLT_LIMIT;
        s_spi_msg.command.data = p_bfe_ctrl->setup.undervoltage_limit;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);   // Check for errors.

        /* Set External Temperature Limit. */
        s_spi_msg.command.reg_address = BFE_REG_EXT_TEMP_LIMIT;
        s_spi_msg.command.data = p_bfe_ctrl->setup.ext_temp_limit;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);   // Check for errors.

        /* Set Balance Setup Register. */
        s_spi_msg.command.reg_address = BFE_REG_BAL_SETUP;
        s_spi_msg.command.data = bal_setup;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);   // Check for errors.

        /* Set Balance Status Register. */
        s_spi_msg.command.reg_address = BFE_REG_BAL_STATUS;
        s_spi_msg.command.data = bal_stat;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);   // Check for errors.

        /* Write Device Setup Register. */
        s_spi_msg.command.reg_address = BFE_REG_DEVICE_SETUP;
        s_spi_msg.command.data = dev_setup | BFE_WDT_PASSWORD;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);   // Check for errors.

        /* Set Watchdog Time. */
        s_spi_msg.command.reg_address = BFE_REG_WDG_BAL_TIME;
        s_spi_msg.command.data = p_bfe_ctrl->wd_time;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);   // Check for errors.

        /* Write Device Setup Register. */
        s_spi_msg.command.reg_address = BFE_REG_DEVICE_SETUP;
        s_spi_msg.command.data = dev_setup;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);   // Check for errors.

        /* Read Internal Temperature Limit. */
        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_INT_TEMP_LIMIT;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);   // Check for errors.

        p_bfe_ctrl->status.ic_temp_limit[i] = s_spi_msg.response[0].data;   // Assign temp limit

        /* Read Trim Voltage. */
        s_spi_msg.command.reg_address = BFE_REG_TRIM_VOLT;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);   // Check for errors.

        p_bfe_ctrl->status.v_trim[i] = s_spi_msg.response[0].data;   // Assign trim voltage

        /* Fault Setup. */
        s_spi_msg.r_w_data = BFE_WRITE_REG;
        s_spi_msg.command.reg_address = BFE_REG_FAULT_SETUP;
        s_spi_msg.command.data = flt_setup;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        /* Cells for fault monitoring selection. */
        flt_cells = BFE_ALL_CELLS_BIT_MASK & (uint16_t) (~p_bfe_ctrl->setup.cells_cfg[i]);
        s_spi_msg.command.reg_address = BFE_REG_CELL_SETUP;
        s_spi_msg.command.data = flt_cells;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

#if BFE_CFG_REG_WRITE_VERIFY_EN // Verify register write
#if BFE_CFG_STA_DEV > 1U        // Daisy chain operation

        /* Send read all register page 1 command. */
        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_ALL_SETUP_DATA;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);   // Check for errors.

        BFE_ERROR_RETURN(s_spi_msg.response[BFE_SALL_OVL_OFFST].data == p_bfe_ctrl->setup.overvoltage_limit,
                                                                                        BMS_ERR_REG_VERIFY);
        BFE_ERROR_RETURN(s_spi_msg.response[BFE_SALL_UVL_OFFST].data == p_bfe_ctrl->setup.undervoltage_limit,
                                                                                         BMS_ERR_REG_VERIFY);
        BFE_ERROR_RETURN(s_spi_msg.response[BFE_SALL_EXT_OFFST].data == p_bfe_ctrl->setup.ext_temp_limit,
                                                                                     BMS_ERR_REG_VERIFY);
        BFE_ERROR_RETURN(s_spi_msg.response[BFE_SALL_BAL_STP_OFFST].data == bal_setup, BMS_ERR_REG_VERIFY);
        BFE_ERROR_RETURN(s_spi_msg.response[BFE_SALL_BAL_STA_OFFST].data == bal_stat, BMS_ERR_REG_VERIFY);
        BFE_ERROR_RETURN(s_spi_msg.response[BFE_SALL_WDT_BAL_OFFST].data == p_bfe_ctrl->wd_time, BMS_ERR_REG_VERIFY);
        BFE_ERROR_RETURN((s_spi_msg.response[BFE_SALL_DEV_STP_OFFST].data & BFE_DSR_CHB_MASK) == dev_setup,
                                                                                       BMS_ERR_REG_VERIFY);

#else // Stand-alone operation

        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        s_spi_msg.command.reg_address = BFE_REG_OVERVOLT_LIMIT;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);   // Check for errors.

        BFE_ERROR_RETURN(s_spi_msg.response[0].data == p_bfe_ctrl->setup.overvoltage_limit, BMS_ERR_REG_VERIFY);

        s_spi_msg.command.reg_address = BFE_REG_UNDERVOLT_LIMIT;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);   // Check for errors.

        BFE_ERROR_RETURN(s_spi_msg.response[0].data == p_bfe_ctrl->setup.undervoltage_limit, BMS_ERR_REG_VERIFY);

        s_spi_msg.command.reg_address = BFE_REG_EXT_TEMP_LIMIT;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);   // Check for errors.

        BFE_ERROR_RETURN(s_spi_msg.response[0].data == p_bfe_ctrl->setup.ext_temp_limit, BMS_ERR_REG_VERIFY);

        s_spi_msg.command.reg_address = BFE_REG_BAL_SETUP;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);   // Check for errors.

        BFE_ERROR_RETURN(s_spi_msg.response[0].data == bal_setup, BMS_ERR_REG_VERIFY);

        s_spi_msg.command.reg_address = BFE_REG_BAL_STATUS;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);   // Check for errors.

        BFE_ERROR_RETURN(s_spi_msg.response[0].data == bal_stat, BMS_ERR_REG_VERIFY);

        s_spi_msg.command.reg_address = BFE_REG_WDG_BAL_TIME;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);   // Check for errors.

        BFE_ERROR_RETURN(s_spi_msg.response[0].data == p_bfe_ctrl->wd_time, BMS_ERR_REG_VERIFY);

        s_spi_msg.command.reg_address = BFE_REG_DEVICE_SETUP;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);   // Check for errors.

        BFE_ERROR_RETURN((s_spi_msg.response[0].data & BFE_DSR_CHB_MASK) == dev_setup, BMS_ERR_REG_VERIFY);

#endif //BFE_CFG_DCH_OPERATION

        s_spi_msg.command.reg_address = BFE_REG_FAULT_SETUP;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        BFE_ERROR_RETURN(s_spi_msg.response[0].data == flt_setup, BMS_ERR_REG_VERIFY);

        s_spi_msg.command.reg_address = BFE_REG_CELL_SETUP;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        BFE_ERROR_RETURN(s_spi_msg.response[0].data == flt_cells, BMS_ERR_REG_VERIFY);

#endif // BFE_CFG_REG_WRITE_VERIFY_EN
    }

    /* Send Calculate and Check Register Checksum Command. */
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    /* Send command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        /* Send Calculate Register Checksum Command. */
        s_spi_msg.command.reg_address = BFE_CALC_REG_CKSM; // Check checksum

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.
    }

    return err;
}
/**********************************************************************************************************************
 * End of function bfe_setup
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: bfe_faults_read
 * Description  : Reads the Fault Registers.
 *
 * This function performs the following tasks:
 * - If in daisy chain mode send read All Faults Command to all devices in the stack.
 * - If in stand-alone mode sends a Fault Status, Over-temperature, Overvoltage, Undervoltage and Open Wire Registers
 * read command.
 * - Set the fault flags in faults data structure.
 *
 * Arguments    : p_bfe_ctrl                Pointer to Battery Front End control structure.
 *                p_faults                  Faults data structure.
 * Return Value : BFE_SUCCESS               The fault registers are successfully read.
 * Return Value : BFE_ERR_...               Inherit from bfe_spi_msg_send_resp_get().
 *********************************************************************************************************************/
e_bms_err_t bfe_faults_read(st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_faults_t * p_faults)
{
    e_bms_err_t err = BMS_SUCCESS; // Error status
    uint16_t  tmp1;

    FSP_PARAMETER_NOT_USED(p_bfe_ctrl); // Mute unused parameter

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

    /* Read Fault Registers. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

        /* Read all faults. */
        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_ALL_FAULT_DATA;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        tmp1 = s_spi_msg.response[2].data; // Copy Fault Status Register

#else // Stand-alone operation

        /* Read Fault Register. */
        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_FAULT_STATUS;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        tmp1 = s_spi_msg.response[0].data; // Store temporarily the result

#endif //BFE_CFG_DCH_OPERATION

        /* Check oscillator fault bit. */
        if((tmp1 & BFE_FSTR_OSC_BIT_MASK) > 0)
        {
            (p_faults + i)->flt_oscillator = true;
        }
        else
        {
            ;
        }

        /* Check watchdog timeout fault bit. */
        if((tmp1 & BFE_FSTR_WDGF_BIT_MASK) > 0)
        {
            (p_faults + i)->flt_wdt_timout = true;
        }
        else
        {
            ;
        }

        /* Check open wire fault bit on VBAT connection. */
        if((tmp1 & BFE_FSTR_OVB_BIT_MASK) > 0)
        {
            (p_faults + i)->flt_ow_vbat = true;
        }
        else
        {
            ;
        }

        /* Check open wire fault bit on VSS connection. */
        if((tmp1 & BFE_FSTR_OVS_BIT_MASK) > 0)
        {
            (p_faults + i)->flt_ow_vss = true;
        }
        else
        {
            ;
        }

        /* Check register checksum (parity) error bit. */
        if((tmp1 & BFE_FSTR_PAR_BIT_MASK) > 0)
        {
            (p_faults + i)->flt_parity = true;
        }
        else
        {
            ;
        }

        /* Check voltage reference fault bit. */
        if((tmp1 & BFE_FSTR_REF_BIT_MASK) > 0)
        {
            (p_faults + i)->flt_v_ref = true;
        }
        else
        {
            ;
        }

        /* Check voltage regulator fault bit. */
        if((tmp1 & BFE_FSTR_REG_BIT_MASK) > 0)
        {
            (p_faults + i)->flt_v_reg = true;
        }
        else
        {
            ;
        }

        /* Check register checksum (parity) error bit. */
        if((tmp1 & BFE_FSTR_MUX_BIT_MASK) > 0)
        {
            (p_faults + i)->flt_temp_mux = true;
        }
        else
        {
            ;
        }

        /* Check over-temperature fault bit. */
        if((tmp1 & BFE_FSTR_OT_BIT_MASK) > 0)
        {

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

            (p_faults + i)->flt_over_temp |= s_spi_msg.response[0].data; // Copy over-temperature fault register

#else // Stand-alone operation

            /* Read Over-temperature Fault Register. */
            s_spi_msg.r_w_data = BFE_READ_REG;
            s_spi_msg.command.reg_address = BFE_REG_OVER_TEMP_FAULT;
            s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
            BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

            (p_faults + i)->flt_over_temp |= s_spi_msg.response[0].data;

#endif //BFE_CFG_DCH_OPERATION
        }
        else
        {
            ;
        }

        /* Check overvoltage fault bit. */
        if((tmp1 & BFE_FSTR_OV_BIT_MASK) > 0)
        {

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

            (p_faults + i)->flt_overvolt |= s_spi_msg.response[6].data; // Copy overvoltage fault register

#else // Stand-alone operation

            /* Read Overvoltage Fault Register. */
            s_spi_msg.r_w_data = BFE_READ_REG;
            s_spi_msg.command.reg_address = BFE_REG_OVERVOLT_FAULT;
            s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
            BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

            (p_faults + i)->flt_overvolt |= s_spi_msg.response[0].data;

#endif //BFE_CFG_DCH_OPERATION
        }
        else
        {
            ;
        }

        /* Check undervoltage fault bit. */
        if((tmp1 & BFE_FSTR_UV_BIT_MASK) > 0)
        {

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

            (p_faults + i)->flt_undervolt |= s_spi_msg.response[5].data; // Copy Undervoltage Fault Register

#else // Stand-alone operation

            /* Read Undervoltage Fault Register. */
            s_spi_msg.r_w_data = BFE_READ_REG;
            s_spi_msg.command.reg_address = BFE_REG_UNDERVOLT_FAULT;
            s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
            BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

            (p_faults + i)->flt_undervolt |= s_spi_msg.response[0].data;

#endif //BFE_CFG_DCH_OPERATION
        }
        else
        {
            ;
        }

        /* Check open wire fault bit. */
        if((tmp1 & BFE_FSTR_OW_BIT_MASK) > 0)
        {

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

            (p_faults + i)->flt_open_wire |= s_spi_msg.response[4].data; // Copy Open Wire Fault Register

#else // Stand-alone operation

            /* Read Open Wire Fault Register. */
            s_spi_msg.r_w_data = BFE_READ_REG;
            s_spi_msg.command.reg_address = BFE_REG_OPEN_WIRE_FAULT;
            s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

            err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
            BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

            (p_faults + i)->flt_open_wire |= s_spi_msg.response[0].data;

#endif //BFE_CFG_DCH_OPERATION
        }
        else
        {
            ;
        }
    }

    return err;
}
/**********************************************************************************************************************
 * End of function bfe_faults_read
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: bfe_watchdog
 * Description  : Enables or disables the watchdog timer and selects time. Set a watchdog time in p_bfe_ctrl->wd_time.
 * When '0' the watchdog is disabled.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Writes watchdog timer password to Device Setup Register if disabling.
 * - Writes watchdog time or 0 to Watchdog/Balance Time Register.
 * - Verifies register write.
 *
 * Arguments    : p_bfe_ctrl                Pointer to Battery Front End control structure.
 * Return Value : BMS_SUCCESS               The watchdog timer is successfully enabled/disabled.
 * Return Value : BMS_ERR_REG_VERIFY        Register write verification error.
 * Return Value : BMS_ERR_WDT_TIME          Invalid watchdog time.
 * Return Value : BFE_ERR_...               Inherit from bfe_spi_msg_send_resp_get().
 *********************************************************************************************************************/
static e_bms_err_t bfe_watchdog(st_bfe_ctrl_t * p_bfe_ctrl)
{
    e_bms_err_t err  = BMS_SUCCESS; // Error status
    uint16_t  tmp1 = 0;
    uint16_t  tmp2 = 0;

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Check watchdog time. */
    BFE_ERROR_RETURN(BFE_WDT_MAX_TIME >= p_bfe_ctrl->wd_time, BMS_ERR_WDT_TIME);

#endif // BFE_CFG_PARAM_CHECKING_EN

    FSP_PARAMETER_NOT_USED(tmp1);
    FSP_PARAMETER_NOT_USED(tmp2);

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

    /* Send command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        /* Read Device Setup Register. */
        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_DEVICE_SETUP;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        tmp2 = s_spi_msg.response[0].data;

        /* Enter password to modify watchdog time. */
        s_spi_msg.r_w_data = BFE_WRITE_REG;
        s_spi_msg.command.reg_address = BFE_REG_DEVICE_SETUP;
        s_spi_msg.command.data = tmp2 | BFE_WDT_PASSWORD;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        /* Read Watchdog/ Balance time. */
        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_WDG_BAL_TIME;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        /* Clear balance time bits. */
        tmp1 = (uint16_t) ((s_spi_msg.response[0].data & BFE_BALANCE_TIME_MASK) | p_bfe_ctrl->wd_time);

        /* Set watchdog time. */
        s_spi_msg.r_w_data = BFE_WRITE_REG;
        s_spi_msg.command.reg_address = BFE_REG_WDG_BAL_TIME;
        s_spi_msg.command.data = tmp1;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

#if BFE_CFG_REG_WRITE_VERIFY_EN // Verify register write

        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_WDG_BAL_TIME;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        BFE_ERROR_RETURN(tmp1 == s_spi_msg.response[0].data, BMS_ERR_REG_VERIFY);

#endif // BFE_CFG_REG_WRITE_VERIFY_EN

        /* Clear password field. */
        s_spi_msg.r_w_data = BFE_WRITE_REG;
        s_spi_msg.command.reg_address = BFE_REG_DEVICE_SETUP;
        s_spi_msg.command.data = tmp2;

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

#if BFE_CFG_REG_WRITE_VERIFY_EN // Verify register write

        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.reg_address = BFE_REG_DEVICE_SETUP;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        BFE_ERROR_RETURN(tmp2 == s_spi_msg.response[0].data, BMS_ERR_REG_VERIFY);

#endif // BFE_CFG_REG_WRITE_VERIFY_EN

        /* Send Calculate and Check Register Checksum Command. */
        s_spi_msg.r_w_data = BFE_READ_REG;
        s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

        /* Send Calculate Register Checksum Command. */
        s_spi_msg.command.reg_address = BFE_CALC_REG_CKSM; // Check checksum

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.
    }

#endif //BFE_CFG_DCH_OPERATION
    return err;
}
/**********************************************************************************************************************
 * End of function bfe_watchdog
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: bfe_EEPROMcheck
 * Description  : Compares shadow registers and factory written checksum to detect EEPROM data corruption (Stand-alone/
 * Daisy chain).
 *
 * This function performs the following tasks:
 * - Reads factory written checksum.
 * - Reads calculated checksum of shadow registers on power on or reset.
 * - Compares both checksums.
 *
 * Arguments    : p_bfe_ctrl                Pointer to Battery Front End control structure.
 * Return Value : BMS_SUCCESS               The checksums are equal and EEPROM data is not corrupted.
 * Return Value : BFE_ERR_EEPROM            The EEPROM data is corrupted.
 * Return Value : BMS_ERR_INVALID_POINTER   An input argument is invalid.
 * Return Value : BFE_ERR_...               Inherit from bfe_spi_msg_send_resp_get().
 *********************************************************************************************************************/
static e_bms_err_t bfe_EEPROMcheck(st_bfe_ctrl_t * p_bfe_ctrl)
{
    e_bms_err_t err                       = BMS_SUCCESS; // Error status
    uint16_t  tmp_data[BFE_CFG_STA_DEV] = {ZERO};      // Temporary data array

    FSP_PARAMETER_NOT_USED(p_bfe_ctrl); // Mute unused parameter

    /* Clean message structure. */
    memset(&s_spi_msg, ZERO, sizeof(st_isl94212_spi_msg_t));

    /* Clean data array. */
    memset(&tmp_data[0], ZERO, sizeof(tmp_data));

    /* Send Read Factory Checksum Command. */
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_EEPROM_MISR_DATA;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

    /* Send command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

        tmp_data[i] = s_spi_msg.response[0].data;
    }

#else // Stand-alone operation

    s_spi_msg.command.device_address = BFE_DAISY_CHAIN_DEVICE1; // Assign a device address.

    err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    tmp_data[0] = s_spi_msg.response[0].data;

#endif //BFE_CFG_DCH_OPERATION

    /* Send Read Calculated Shadow Register Checksum Command. */
    s_spi_msg.r_w_data = BFE_READ_REG;
    s_spi_msg.command.reg_address = BFE_REG_MISR_CALC_CKSM;
    s_spi_msg.command.data = ZERO; // Ensure command data is zero for this register type

    /* Send command to each device from the stack. */
    for(uint16_t i = 0; i < p_bfe_ctrl->status.stacked_devs; i++ )
    {
        s_spi_msg.command.device_address = (e_bfe_dev_addr_t)(i + 1); // Assign a device address.

        err = bfe_spi_msg_send_resp_get(&s_spi_msg); // Send spi message and get a response.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err);  // Check for errors.

        BFE_ERROR_RETURN(tmp_data[i] == s_spi_msg.response[0].data, BFE_ERR_EEPROM);    // Compare checksums.
    }

    return err;
}
/**********************************************************************************************************************
 * End of function bfe_EEPROMcheck
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: bfe_spi_msg_send_resp_get
 * Description  : Manages the sending and reception of a SPI data between the microcontroller and the Battery
 * Front End.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Checks for and receives pending data.
 * - Calls the command array encode function and copies the command to the transmit buffer.
 * - Calls SPI read/write function.
 * - Copies the response to the encoded response array and calls a decoding function.
 *
 * Arguments    : p_spi_msg                 Pointer to spi message structure.
 * Return Value : BFE_SUCCESS               The command was successfully encoded.
 * Return Value : BMS_ERR_INVALID_POINTER   An input argument is invalid.
 * Return Value : BMS_ERR_SPI_WRITE_READ    writeRead() has returned an error or transfer complete was no indicated
 * Return Value : BMS_ERR_RESP_LENGTH       The received response has unexpected length.
 * Return Value : BMS_ERR_ACK               Acknowledgment is not received.
 * Return Value : BMS_ERR_NAK               Unidentified command or CRC.
 * Return Value : BMS_ERR_GPIO_READ         Error reading the input.
 * Return Value : BMS_FAULT                 BMS fault is detected.
 * Return Value : BMS_ERR_COMM              Communication error.
 * Return Value : BFE_ERR_...               Inherit from bfe_spi_d_ch_resp_get().
 * Return Value : BFE_ERR_...               Inherit from bfe_encode_command().
 * Return Value : BFE_ERR_...               Inherit from bfe_decode_responce().
 *********************************************************************************************************************/
static e_bms_err_t bfe_spi_msg_send_resp_get(st_isl94212_spi_msg_t * p_spi_msg)
{
    e_bms_err_t      err           = BMS_SUCCESS;       // Error status for bsp functions
    fsp_err_t      fsp_err       = FSP_SUCCESS;       // Error status for fsp functions
    bsp_io_level_t drdy_pin_val  = BSP_IO_LEVEL_HIGH; // Read data ready pin value
    uint32_t       data_length   = ZERO;

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointers are valid. */
    BFE_ERROR_RETURN(NULL != p_spi_msg, BMS_ERR_INVALID_POINTER);


#endif // BFE_CFG_PARAM_CHECKING_EN

    FSP_PARAMETER_NOT_USED(drdy_pin_val);

    /* Clean buffers. */
    memset(&s_spi_tx_buff[0], ZERO, sizeof(s_spi_tx_buff));
    memset(&s_spi_rx_buff[0], ZERO, sizeof(s_spi_rx_buff));

    /* Clear response fields. */
    memset(&p_spi_msg->response[0], ZERO, sizeof(p_spi_msg->response));
    p_spi_msg->enc_resp_length = 0;

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation


    /* Check for pending data. Receive bytes and ignore content. */
    fsp_err = g_ioport.p_api->pinRead(&g_ioport_ctrl, BFE_DAT_RDY_PIN, &drdy_pin_val);

    BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_GPIO_READ); // Check for errors.


    if(drdy_pin_val == BSP_IO_LEVEL_LOW)
    {

        err = bfe_spi_d_ch_resp_get(p_spi_msg); // Receive the response

        BFE_ERROR_RETURN(BMS_ERR_SPI_MSG_BUF != err, err); // Ignore returned errors.
        BFE_ERROR_RETURN(p_spi_msg->response[0].reg_address != (BFE_REG_FAULT_STATUS & BFE_REG_PAGE_ADDR_MASK),
                                                                                                    BMS_FAULT);
        BFE_ERROR_RETURN(p_spi_msg->response[0].reg_address != (BFE_REG_COMMS_FAILURE & BFE_REG_PAGE_ADDR_MASK),
                                                                                                  BMS_ERR_COMM);

        /* Wait for daisy chain ports to clear. */
        R_BSP_SoftwareDelay(s_comm_delays.min_wait_time_us, BSP_DELAY_UNITS_MICROSECONDS);
    }
    else
    {
        ;
    }

#endif //BFE_CFG_DCH_OPERATION

    err = bfe_command_encode(p_spi_msg); // Assemble communication data to be send via SPI.

    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

    /* Determine response length. */
    p_spi_msg->exp_resp_length = (p_spi_msg->command.reg_address & BFE_REG_RESP_LGTH_MASK) >> 16;

    /* Assign number of transfers */
    data_length = p_spi_msg->enc_cmnd_length;
    datalength_check = data_length;

#else // Stand-alone operation

    /* Add dummy bytes for commands that return data. */
    if((p_spi_msg->command.reg_address & BFE_REG_RESP_STA_MASK) > ZERO)
    {
        p_spi_msg->enc_resp_length = BFE_SPI_RESP_DAT_LGTH_STA;

    }
    else
    {
        p_spi_msg->enc_resp_length = ZERO;
    }

    /* Assign number of transfers */
    data_length = p_spi_msg->enc_cmnd_length + p_spi_msg->enc_resp_length;

#endif //BFE_CFG_DCH_OPERATION

    /* Copy encoded command to output buffer. */
    for(uint32_t i = 0; i < p_spi_msg->enc_cmnd_length; i++ )
    {
        s_spi_tx_buff[i] = p_spi_msg->encoded_cmnd[i];
    }


    /* SPI transmit and receive data function. */

    fsp_err = g_spi1.p_api->writeRead(&g_spi1_ctrl, s_spi_tx_buff, s_spi_rx_buff, data_length, SPI_BIT_WIDTH_8_BITS);

    BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_SPI_WRITE_READ); // Check for errors.


    /* Wait until WriteRead() is complete. */
    while(SPI_EVENT_TRANSFER_COMPLETE != s_spi_event_flag)
    {

        /* Safety timeout operation, if SPI interrupt is masked. */

        s_spi_wait_count--;

        /* Check for error conditions. */
        BFE_ERROR_RETURN(BFE_SPI_MIN_CNT < s_spi_wait_count, BMS_ERR_SPI_WRITE_READ);
        BFE_ERROR_RETURN(SPI_EVENT_TRANSFER_ABORTED != s_spi_event_flag, BMS_ERR_SPI_WRITE_READ);
    }

    s_spi_wait_count = BFE_SPI_MAX_CNT; // Reset counter.
    s_spi_event_flag = (spi_event_t) RESET_VALUE; // Reseting spi_event flag.

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

    if(p_spi_msg->exp_resp_length > 0)
    {
        exp_resp_lenght_check = p_spi_msg->exp_resp_length;
        err = bfe_spi_d_ch_resp_get(p_spi_msg); // Receive the response

        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

        /* Check for communication error. */
        BFE_ERROR_RETURN(p_spi_msg->response[0].reg_address != (BFE_REG_COMMS_FAILURE & BFE_REG_PAGE_ADDR_MASK),
                                                                                                  BMS_ERR_COMM);

        /* Check for NAK. */
        BFE_ERROR_RETURN(p_spi_msg->response[0].reg_address != (BFE_REG_NAK & BFE_REG_PAGE_ADDR_MASK), BMS_ERR_NAK);


        /* Check for message length and fault status response. */
        if(p_spi_msg->enc_resp_length != p_spi_msg->exp_resp_length)
        {
            if((p_spi_msg->response[0].reg_address == (BFE_REG_FAULT_STATUS & BFE_REG_PAGE_ADDR_MASK))
            || (p_spi_msg->response[1].reg_address == (BFE_REG_FAULT_STATUS & BFE_REG_PAGE_ADDR_MASK)))
            {
                return BMS_FAULT;
            }
            else
            {
                return BMS_ERR_RESP_LENGTH;
            }
        }
        else
        {
            ;
        }

        /* Check for ACK when write command or other expecting ACK response. */
        if(((p_spi_msg->command.reg_address & BFE_REG_ACK_MASK) > ZERO) || (p_spi_msg->r_w_data ==  BFE_WRITE_REG))
        {
            BFE_ERROR_RETURN((p_spi_msg->response[0].reg_address == (BFE_REG_ACK & BFE_REG_PAGE_ADDR_MASK)), BMS_ERR_ACK);
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

    /* Wait for daisy chain ports to clear. */
    R_BSP_SoftwareDelay(s_comm_delays.min_wait_time_us, BSP_DELAY_UNITS_MICROSECONDS);

#else // Stand-alone operation

    /* Mute unused function. */
    if(false)
    {
        bfe_spi_d_ch_resp_get(p_spi_msg);
    }
    else
    {
        ;
    }

    if(p_spi_msg->enc_resp_length > 0)
    {
        /* Copy input buffer to command response */
        for(uint32_t i = 0; i < p_spi_msg->enc_resp_length; i++ )
        {
           p_spi_msg->encoded_resp[i] = s_spi_rx_buff[i + p_spi_msg->enc_cmnd_length]; // Ignore first 2 bytes
        }

        err = bfe_responce_decode(p_spi_msg); // Disassemble communication data received via SPI.
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.
    }
    else
    {
        ;
    }

#endif //BFE_CFG_DCH_OPERATION

    return err;
}
/**********************************************************************************************************************
 * End of function bfe_spi_msg_send_resp_get
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: bfe_spi_pending_resp_get
 * Description  : Checks for and receives a pending response data.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Checks data ready pin.
 * - Calls bfe_spi_d_ch_resp_get static function to receive the response.
 *
 * Arguments    : p_spi_msg                 Pointer to spi message structure.
 * Return Value : BFE_SUCCESS               The pending response was successfully processed.
 * Return Value : BMS_ERR_INVALID_POINTER   An input argument is invalid.
 * Return Value : BMS_ERR_SPI_WRITE_READ    writeRead() has returned an error or transfer complete was no indicated.
 * Return Value : BMS_ERR_GPIO_READ         Error reading the input.
 * Return Value : BMS_ERR_NO_PENDING_RESP   There is no pending response.
 * Return Value : BFE_ERR_...               Inherit from bfe_spi_d_ch_resp_get().
 *********************************************************************************************************************/
static e_bms_err_t bfe_spi_pending_resp_get(st_isl94212_spi_msg_t * p_spi_msg)
{
    e_bms_err_t      err           = BMS_SUCCESS;       // Error status for bsp functions
    fsp_err_t      fsp_err       = FSP_SUCCESS;       // Error status for fsp functions
    bsp_io_level_t drdy_pin_val  = BSP_IO_LEVEL_HIGH; // Read data ready pin value

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointers are valid. */
    BFE_ERROR_RETURN(NULL != p_spi_msg, BMS_ERR_INVALID_POINTER);

#endif // BFE_CFG_PARAM_CHECKING_EN

    FSP_PARAMETER_NOT_USED(fsp_err); // Mute unused parameter
    FSP_PARAMETER_NOT_USED(drdy_pin_val); // Mute unused parameter

    /* Clean buffers. */
    memset(&s_spi_tx_buff[0], ZERO, sizeof(s_spi_tx_buff));
    memset(&s_spi_rx_buff[0], ZERO, sizeof(s_spi_rx_buff));

    /* Clear response fields. */
    memset(&p_spi_msg->response[0], ZERO, sizeof(p_spi_msg->response));

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

    /* Check for pending data. */
    fsp_err = g_ioport.p_api->pinRead(&g_ioport_ctrl, BFE_DAT_RDY_PIN, &drdy_pin_val);
    BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_GPIO_READ); // Check for errors.

    BFE_ERROR_RETURN(BSP_IO_LEVEL_LOW == drdy_pin_val, BMS_ERR_NO_PENDING_RESP); // Check for pending response.

    err = bfe_spi_d_ch_resp_get(p_spi_msg); // Receive the response
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Wait for daisy chain ports to clear. */
    R_BSP_SoftwareDelay(s_comm_delays.min_wait_time_us, BSP_DELAY_UNITS_MICROSECONDS);

#endif //BFE_CFG_DCH_OPERATION

    return err;
}
/**********************************************************************************************************************
 * End of function bfe_spi_pending_resp_get
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: bfe_spi_d_ch_resp_get
 * Description  : Receives all response data bytes.
 *
 * This function performs the following tasks:
 * - Wait until the data ready pin is asserted.
 * - Receive all pending bytes.
 * - Decode the response.
 *
 * Arguments    : p_spi_msg                 Pointer to spi message structure.
 * Return Value : BFE_SUCCESS               The command was successfully encoded.
 * Return Value : BMS_ERR_INVALID_POINTER   An input argument is invalid.
 * Return Value : BMS_ERR_SPI_WRITE_READ    Read() has returned an error or transfer complete was no indicated
 * Return Value : BMS_ERR_COMM_TIMEOUT      Error in communication timeout asynchronous timer.
 * Return Value : BMS_ERR_DRDY_TIMEOUT      Data ready pin was not asserted in the expected time interval.
 * Return Value : BMS_ERR_GPIO_READ         Error reading the input.
 * Return Value : BFE_ERR_...               Inherit from bfe_decode_responce().
 *********************************************************************************************************************/
static e_bms_err_t bfe_spi_d_ch_resp_get(st_isl94212_spi_msg_t * p_spi_msg)
{
    e_bms_err_t    err           = BMS_SUCCESS;       // Error status for bsp functions
    fsp_err_t      fsp_err       = FSP_SUCCESS;       // Error status for fsp functions
    bsp_io_level_t drdy_pin_val  = BSP_IO_LEVEL_HIGH; // Read data ready pin value

    s_timeout_event_flag = false; // Clear communication timer event flag;

    /* Set response timeout and reset. */
    fsp_err = g_timer_one_shot.p_api->periodSet(&g_timer_one_shot_ctrl, s_comm_delays.max_resp_delay);
    BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_COMM_TIMEOUT); // Check for errors.

    /* Run response timeout timer. */
    fsp_err = g_timer_one_shot.p_api->start(&g_timer_one_shot_ctrl);
    BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_COMM_TIMEOUT); // Check for errors.

    /* Wait for a response (data ready assertion) after command was sent. */
    do
    {
        /* Safety timeout, if timer interrupt is masked. */

        s_spi_timeout--;

        fsp_err = g_ioport.p_api->pinRead(&g_ioport_ctrl, BFE_DAT_RDY_PIN, &drdy_pin_val);


        BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_GPIO_READ); // Check for errors.

        /* Check for response timeout. */
        BFE_ERROR_RETURN((false == s_timeout_event_flag) || (drdy_pin_val == BSP_IO_LEVEL_LOW),
                                                                          BMS_ERR_DRDY_TIMEOUT);

        /* Check for timeout error. */
        BFE_ERROR_RETURN(BFE_SPI_TO_MIN_CNT < s_spi_timeout, BMS_ERR_DRDY_TIMEOUT);
        if(err != BMS_ERR_DRDY_TIMEOUT){
            BMS_ERR_DRDY_TIMEOUT_check = 1;
        }else{
            BMS_ERR_DRDY_TIMEOUT_check = 2;
        }

    }
    while(drdy_pin_val == BSP_IO_LEVEL_HIGH);


    /* Stop response timeout timer. */
    fsp_err = g_timer_one_shot.p_api->stop(&g_timer_one_shot_ctrl);
    BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_COMM_TIMEOUT); // Check for errors.

    /* Set next response byte timeout and reset. */
    fsp_err = g_timer_one_shot.p_api->periodSet(&g_timer_one_shot_ctrl, s_comm_delays.max_bytes_delay);
    BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_COMM_TIMEOUT); // Check for errors.

    s_spi_timeout = BFE_SPI_TO_MAX_CNT; // Reset counter.

    /* Receive response bytes. */
    p_spi_msg->enc_resp_length = 0;

    do
    {

        //R_SPI_Read
        /* SPI receive data. */
        fsp_err = g_spi1.p_api->read(&g_spi1_ctrl, s_spi_rx_buff, BFE_SPI_ONE_BYTE_RESP_LGTH, SPI_BIT_WIDTH_8_BITS);
        if(FSP_SUCCESS == fsp_err){
                    gg = 1;
                }else{
                    gg = 2;
                }

        BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_SPI_WRITE_READ); // Check for errors.


        /* Wait until WriteRead() is complete. */
        while(SPI_EVENT_TRANSFER_COMPLETE != s_spi_event_flag)
        {
            /* Timer out operation, if SPI operation fails to complete. */
            s_spi_wait_count--;

            /* Check for error conditions. */
            BFE_ERROR_RETURN(BFE_SPI_MIN_CNT < s_spi_wait_count, BMS_ERR_SPI_WRITE_READ);
            BFE_ERROR_RETURN(SPI_EVENT_TRANSFER_ABORTED != s_spi_event_flag, BMS_ERR_SPI_WRITE_READ);
        }

        s_spi_wait_count = BFE_SPI_MAX_CNT; // Reset counter.
        s_spi_event_flag = (spi_event_t) RESET_VALUE; // Reseting spi_event flag.

        p_spi_msg->encoded_resp[p_spi_msg->enc_resp_length] = s_spi_rx_buff[ZERO]; // Copy the received byte

        p_spi_msg->enc_resp_length++;


        /* Check for message buffer overflow. */
        BFE_ERROR_RETURN(BFE_SPI_RESP_DAT_MAX >= p_spi_msg->enc_resp_length, BMS_ERR_SPI_MSG_BUF);

        /* Reset response timeout timer. */
        fsp_err = g_timer_one_shot.p_api->reset(&g_timer_one_shot_ctrl);
        BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_COMM_TIMEOUT); // Check for errors.

        /* Run response timeout timer. */
        fsp_err = g_timer_one_shot.p_api->start(&g_timer_one_shot_ctrl);
        BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_COMM_TIMEOUT); // Check for errors.

        /* Wait for data ready assertion between response bytes. */
        do
        {
            /* Safety timeout, if timer interrupt is masked. */
            s_spi_timeout--;

            fsp_err = g_ioport.p_api->pinRead(&g_ioport_ctrl, BFE_DAT_RDY_PIN, &drdy_pin_val);

            BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_GPIO_READ); // Check for errors.

            /* Check for timeout error. */
            BFE_ERROR_RETURN(BFE_SPI_TO_MIN_CNT < s_spi_timeout, BMS_ERR_COMM_TIMEOUT);

        }
        while((drdy_pin_val == BSP_IO_LEVEL_HIGH) && (s_timeout_event_flag == false));

        /* Stop response timeout timer. */
        fsp_err = g_timer_one_shot.p_api->stop(&g_timer_one_shot_ctrl);
        BFE_ERROR_RETURN(FSP_SUCCESS == fsp_err, BMS_ERR_COMM_TIMEOUT); // Check for errors.

        s_spi_timeout = BFE_SPI_TO_MAX_CNT; // Reset counter.

    }

    while(s_timeout_event_flag == false);

    err = bfe_responce_decode(p_spi_msg); // Disassemble communication data received via SPI.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    return err;
}
/**********************************************************************************************************************
 * End of function bfe_d_ch_p_resp_get
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: bfe_command_encode
 * Description  : Encodes the data to be sent via SPI from the message structure address and data fields to the
 * encoded command buffer.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Encodes the device address, register type, register address and data into the command array.
 *
 * Arguments    : p_spi_msg                         Pointer to spi message structure.
 * Return Value : BFE_SUCCESS                       The command was successfully encoded.
 * Return Value : BMS_ERR_INVALID_POINTER           An input argument is invalid.
 * Return Value : BMS_ERR_INVALID_DEV_ADDRESS       There is no such an address in the daisy chain
 * Return Value : BMS_ERR_INVALID_BROADCAST         Invalid address all.
 * Return Value : BMS_ERR_INVALID_REG_DATA_LENGTH   The command data length exceeds the maximal register bit space.
 * Return Value : BMS_ERR_REGISTER_NOT_WRITABLE     The accessed register is read-only.
 * Return Value : BFE_ERR_...                       Inherit from crc4_add().
 *********************************************************************************************************************/
static e_bms_err_t bfe_command_encode(st_isl94212_spi_msg_t * p_spi_msg)
{
    e_bms_err_t err = BMS_SUCCESS; // Error status

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify device address */
    BFE_ERROR_RETURN(((p_spi_msg->command.device_address == BFE_DAISY_CHAIN_ADDRESS_ALL)
                   || (p_spi_msg->command.device_address <= BFE_HIGHEST_DEV_ADDR)), BMS_ERR_INVALID_DEV_ADDRESS);

    /* Verify broadcast command */
    BFE_ERROR_RETURN(((p_spi_msg->command.device_address != BFE_DAISY_CHAIN_ADDRESS_ALL)
                  || ((p_spi_msg->command.reg_address & BFE_REG_BCAST_MASK) > ZERO)), BMS_ERR_INVALID_BROADCAST);


#endif // BFE_CFG_PARAM_CHECKING_EN

    /* Check if read or write command */
    if(p_spi_msg->r_w_data == BFE_READ_REG)
    {
#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

        /* Verify data length */
        BFE_ERROR_RETURN(p_spi_msg->command.data <= BFE_MAX_6_BIT_DAT_VAL, BMS_ERR_INVALID_REG_DATA_LENGTH);

#endif // BFE_CFG_PARAM_CHECKING_EN

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

        /* Casting is added to ensure data size after integer promotion. */
        p_spi_msg->encoded_cmnd[0] = (uint8_t) (p_spi_msg->command.device_address << 4)
                                   | (uint8_t) (p_spi_msg->r_w_data << 3)
                                   | (uint8_t) ((p_spi_msg->command.reg_address & 0x00000700) >> 8);
        p_spi_msg->encoded_cmnd[1] = (uint8_t) ((p_spi_msg->command.reg_address & 0x0000003F) << 2)
                                   | (uint8_t) ((p_spi_msg->command.data & 0x0030) >> 4);
        p_spi_msg->encoded_cmnd[2] = (uint8_t) ((p_spi_msg->command.data & 0x000F) << 4);

        p_spi_msg->enc_cmnd_length = BFE_READ_CMND_LGTH_B_DCH; // Assign command data length.

        /* Add CRC4 to the last nibble of the command array. */
        err = crc4_add(p_spi_msg->encoded_cmnd, p_spi_msg->enc_cmnd_length);
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.


#else // Stand-alone operation

        /* Casting is added to ensure data size after integer promotion. */
        p_spi_msg->encoded_cmnd[0] = (uint8_t) (p_spi_msg->r_w_data << 7)
                                   | (uint8_t) ((p_spi_msg->command.reg_address & 0x00000700) >> 4)
                                   | (uint8_t) ((p_spi_msg->command.reg_address & 0x0000003C) >> 2);
        p_spi_msg->encoded_cmnd[1] = (uint8_t) ((p_spi_msg->command.reg_address & 0x00000003) << 6)
                                   | (uint8_t) (p_spi_msg->command.data);

        p_spi_msg->enc_cmnd_length = BFE_READ_CMND_LGTH_B_STA; // Assign command data length.

#endif //BFE_CFG_DCH_OPERATION
    }
    else
    {

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

        /* Verify the register is writable. */
        BFE_ERROR_RETURN(((p_spi_msg->command.reg_address & BFE_REG_WRITE_MASK) > ZERO)
                                                       , BMS_ERR_REGISTER_NOT_WRITABLE);

#endif // BFE_CFG_PARAM_CHECKING_EN

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

        /* Casting is added to ensure data size after integer promotion. */
        p_spi_msg->encoded_cmnd[0] = (uint8_t) (p_spi_msg->command.device_address << 4)
                                   | (uint8_t) (p_spi_msg->r_w_data << 3)
                                   | (uint8_t) ((p_spi_msg->command.reg_address & 0x00000700) >> 8);
        p_spi_msg->encoded_cmnd[1] = (uint8_t) ((p_spi_msg->command.reg_address & 0x0000003F) << 2)
                                   | (uint8_t) ((p_spi_msg->command.data & 0x3000) >> 12);
        p_spi_msg->encoded_cmnd[2] = (uint8_t) ((p_spi_msg->command.data & 0x0FF0) >> 4);
        p_spi_msg->encoded_cmnd[3] = (uint8_t) ((p_spi_msg->command.data & 0x000F) << 4);

        p_spi_msg->enc_cmnd_length = BFE_WRITE_CMND_LGTH_B_DCH; // Assign command data length.

        /* Add CRC4 to the last nibble of the command array. */
        err = crc4_add(p_spi_msg->encoded_cmnd, p_spi_msg->enc_cmnd_length);
        BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

#else // Stand-alone operation

        /* Casting is added to ensure data size after integer promotion. */
        p_spi_msg->encoded_cmnd[0] = (uint8_t) (p_spi_msg->r_w_data << 7)
                                   | (uint8_t) ((p_spi_msg->command.reg_address & 0x00000700) >> 4)
                                   | (uint8_t) ((p_spi_msg->command.reg_address & 0x0000003C) >> 2);
        p_spi_msg->encoded_cmnd[1] = (uint8_t) ((p_spi_msg->command.reg_address & 0x00000003) << 6)
                                   | (uint8_t) ((p_spi_msg->command.data & 0x3F00) >> 8);
        p_spi_msg->encoded_cmnd[2] = (uint8_t) (p_spi_msg->command.data & 0x00FF);

        p_spi_msg->enc_cmnd_length = BFE_WRITE_CMND_LGTH_B_STA; // Assign command data length.

#endif //BFE_CFG_DCH_OPERATION
    }

    return err;
}
/**********************************************************************************************************************
 * End of function bfe_command_encode
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: bfe_responce_decode
 * Description  : Decodes the data received via SPI from the message structure encoded response buffer to the
 * response data fields.
 *
 * This function performs the following tasks:
 * - Checks parameters and processes errors.
 * - Decodes the data into the response data array.
 * - Detect and decode multiple data response.
 *
 * Arguments    : p_spi_msg                 Pointer to spi message structure.
 * Return Value : BFE_SUCCESS               The response was successfully decoded.
 * Return Value : BMS_ERR_INVALID_POINTER   An input argument is invalid.
 * Return Value : BFE_ERR_...               Inherit from crc4_check().
 *********************************************************************************************************************/
static e_bms_err_t bfe_responce_decode(st_isl94212_spi_msg_t * p_spi_msg)
{
    e_bms_err_t err = BMS_SUCCESS; // Error status

#if BFE_CFG_STA_DEV > 1U  // Daisy chain operation

    /* Check the received data CRC4. */
    err = crc4_check(p_spi_msg->encoded_resp, BFE_SPI_NORM_RESP_DAT_LGTH_DCH, ZERO);
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    for(uint32_t k = 0; k < p_spi_msg->enc_resp_length / BFE_SPI_RESP_TYPE1_LGTH_DCH; k++)
    {

        /* Calculate data offset. */
        uint32_t l = k * BFE_SPI_RESP_TYPE1_LGTH_DCH;

        /* Assign device address. */
        p_spi_msg->response[k].device_address = ((p_spi_msg->encoded_resp[l] & 0xF0) >> 4);


        /* Decode register address. */
        p_spi_msg->response[k].reg_address = (p_spi_msg->encoded_resp[l + 1] & 0xFC) >> 2;


        /* Casting is added to ensure data size after integer promotion. */
        p_spi_msg->response[k].reg_address = p_spi_msg->response[k].reg_address
                                           | (uint16_t) ((p_spi_msg->encoded_resp[l] & 0x07) << 8);

        /* Decode received data. */
        p_spi_msg->response[k].data = (p_spi_msg->encoded_resp[l + 3] & 0xF0) >> 4;

        /* Casting is added to ensure data size after integer promotion. */
        p_spi_msg->response[k].data = (uint16_t) (p_spi_msg->response[k].data
                                    | (p_spi_msg->encoded_resp[l + 2] << 4));
        p_spi_msg->response[k].data = (uint16_t) (p_spi_msg->response[k].data
                                    | (p_spi_msg->encoded_resp[l + 1] & 0x03) << 12);




        if(p_spi_msg->enc_resp_length >= BFE_SPI_ALL_RESP_LGTH_MIN)
        {
            for(uint32_t i = 1; i < p_spi_msg->enc_resp_length / BFE_SPI_RESP_TYPE2_LGTH_DCH; i++)
            {
                /* Calculate data offset. */
                uint32_t j = BFE_SPI_RESP_TYPE1_LGTH_DCH + ((i - 1) * BFE_SPI_RESP_TYPE2_LGTH_DCH);

                /* Check the received data CRC4. */
                err = crc4_check(p_spi_msg->encoded_resp, BFE_SPI_RESP_TYPE2_LGTH_DCH, j);
                BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

                /* Decode device address. */
                p_spi_msg->response[i].device_address = ((p_spi_msg->encoded_resp[l] & 0xF0) >> 4);

                /* Decode register address. */
                p_spi_msg->response[i].reg_address = (p_spi_msg->encoded_resp[j] & 0xFC) >> 2;

                /* Casting is added to ensure data size after integer promotion. */
                p_spi_msg->response[i].reg_address = p_spi_msg->response[i].reg_address
                                                   | (uint16_t) ((p_spi_msg->encoded_resp[l] & 0x07) << 8);

                /* Decode received data. */
                p_spi_msg->response[i].data = (p_spi_msg->encoded_resp[j + 2] & 0xF0) >> 4;

                /* Casting is added to ensure data size after integer promotion. */
                p_spi_msg->response[i].data = (uint16_t) (p_spi_msg->response[i].data
                                            | (p_spi_msg->encoded_resp[j + 1] << 4));
                p_spi_msg->response[i].data = (uint16_t) (p_spi_msg->response[i].data
                                            | ((p_spi_msg->encoded_resp[j] & 0x03) << 12));
            }

            break;  // Exit for loop after data reception
        }
        else
        {
            ;
        }
    }

#else // Stand-alone operation

    /* Casting is added to ensure data size after integer promotion. */
    p_spi_msg->response[0].data = (uint16_t) ((p_spi_msg->encoded_resp[0] & 0x3F) << 8);
    p_spi_msg->response[0].data = (uint16_t) (p_spi_msg->response[0].data | p_spi_msg->encoded_resp[1]);

#endif //BFE_CFG_DCH_OPERATION

    return err;
}
/**********************************************************************************************************************
 * End of function bfe_responce_decode
 *********************************************************************************************************************/

/* Function Name: spi_isr_callback */
/******************************************************************************************************************//**
 * SPI callback function.
 * @param[in] p_args
 * @retval None
 *********************************************************************************************************************/
void spi_isr_callback (spi_callback_args_t * p_args)
{
    /* Check for SPI transfer complete condition. */
    if (SPI_EVENT_TRANSFER_COMPLETE == p_args->event)
    {
        s_spi_event_flag = SPI_EVENT_TRANSFER_COMPLETE;
    }
    else
    {
        s_spi_event_flag = SPI_EVENT_TRANSFER_ABORTED;
    }
}
/**********************************************************************************************************************
 * End of function spi_isr_callback
 *********************************************************************************************************************/

/* Function Name: spi_callback */
/******************************************************************************************************************//**
 * SPI callback function.
 * @param[in] p_args
 * @retval None
 *********************************************************************************************************************/
void comm_timeout0_callback (timer_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);

    s_timeout_event_flag = true;
}
/**********************************************************************************************************************
 * End of function spi_callback
 *********************************************************************************************************************/
