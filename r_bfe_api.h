/**********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO
 * THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2021 Renesas Electronics Corporation. All rights reserved.
 *********************************************************************************************************************/
/******************************************************************************************************************//**
 * @file          r_bfe_api.h
 * @Version       1.0
 * @brief         Interface for communications with the BFE.
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 16.08.2021 1.00     First Release
 *         : 12.11.2021 1.01     Improved communications timing. Data structures are modified. Minor bugs are fixed.
 *********************************************************************************************************************/

#ifndef BFE_R_BFE_API_H_
#define BFE_R_BFE_API_H_

/*******************************************************************************************************************//**
 * @ingroup RENESAS_INTERFACES
 * @defgroup BFE Interface
 * @brief Interface  for accessing and configuring Battery Front End.
 *
 * @section BFE_API_SUMMARY Summary
 * The Battery Front End shared interface provides the ability to access the resources and functionalities of the
 * Battery Front End.
 *
 * BFE Interface description: @ref BFE
 *
 * @{
 **********************************************************************************************************************/

/**********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 *********************************************************************************************************************/
/* Generic headers */
#include <stdint.h>
#include <stdbool.h>

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Global Typedef definitions
 *********************************************************************************************************************/

/** Common error codes */
typedef enum
{
    BMS_SUCCESS = 0,

    BMS_ERR_ASSERTION                   = 1,    ///< Critical assertion has failed.
    BMS_ERR_INVALID_POINTER             = 2,    ///< Pointer points to invalid memory location.
    BMS_ERR_INVALID_ARGUMENT            = 3,    ///< Invalid input parameter.
    BMS_ERR_INVALID_CHANNEL             = 4,    ///< Selected channel does not exist.
    BMS_ERR_INVALID_MODE                = 5,    ///< Unsupported or incorrect mode.
    BMS_ERR_BUFFER_SIZE                 = 6,    ///< The buffer size is not big enough.
    BMS_ERR_INVALID_BROADCAST           = 7,    ///< Invalid broadcast command.
    BMS_ERR_INITIALIZATION              = 8,    ///< Not initialized.
    BMS_ERR_ADDR_MODE                   = 9,    ///< Incorrect device addressing mode.

    BMS_ERR_NA_STANDALONE               = 10,   ///< Not available in stand-alone operation.
    BMS_ERR_DAISY_CHAIN_COMM            = 11,   ///< Error in daisy chain communication.
    BMS_ERR_REG_VERIFY                  = 12,   ///< Register verification error.
    BMS_ERR_SCAN_TYPE                   = 13,   ///< Incorrect scan command.
    BMS_ERR_SCAN_CNTR                   = 14,   ///< Scan command was not received.
    BMS_ERR_ENBL_PIN                    = 15,   ///< Enable pin is not available.
    BMS_ERR_INVALID_CONF                = 16,   ///< Invalid BFE configuration.
    BMS_ERR_INVALID_CELLS               = 17,   ///< Invalid number of cells.
    BMS_ERR_INVALID_STACK               = 18,   ///< Invalid number of stacked devices.

    /* Start of Daisy chain communication specific. */
    BMS_ERR_COMM_ERR                    = 20,   ///< Communication error.
    BMS_ERR_PENDING_RESP                = 21,   ///< Pending response is not expected but available.
    BMS_ERR_NO_PENDING_RESP             = 22,   ///< Pending response is expected but not available.


    /* Start of SPI communication specific. */
    BMS_ERR_SPI_INIT                    = 30,   ///< Initialization error of SPI.
    BMS_ERR_SPI_DEINIT                  = 31,   ///< Deinitialization error of SPI.
    BMS_ERR_SPI_WRITE_READ              = 32,   ///< Write Read error.
    BMS_ERR_SPI_COMM                    = 33,   ///< SPI communication error.
    BMS_ERR_SPI_MSG_BUF                 = 34,   ///< SPI message buffer overflow.

    /* Start of timer specific. */
    BMS_ERR_WDT_TIME                    = 40,   ///< Invalid watchdog time.
    BMS_ERR_WDT_RESET                   = 41,   ///< Reset watchdog timer error.
    BMS_ERR_COMM_TIMEOUT_INIT           = 42,   ///< Communication timeout timer initialization error.
    BMS_ERR_COMM_TIMEOUT                = 43,   ///< Communication timeout timer error.
    BMS_ERR_TIMEOUT                     = 44,   ///< BMS timer error.

    /* Start of BFE control specific. */
    BMS_ERR_BALANCE                     = 50,   ///< Device balance error.
    BMS_ERR_BALANCE_MODE                = 51,   ///< Incorrect device balance mode.
    BMS_ERR_SLEEP                       = 52,   ///< Device sleep error.
    BMS_ERR_WAKEUP                      = 53,   ///< Device wake up error.

    /* Start of GPIO specific. */
    BMS_ERR_GPIO_READ                   = 60,   ///< GPIO port read error.

    /* Fault management specific. */
    BMS_ERR_FLT_DEASSERT                = 70,   ///< Fault pin deassert error.
    BMS_FAULT                           = 71,   ///< The fault pin is asserted.
    BMS_ERR_COMM                        = 72,   ///< Communication error.

    /* Daisy chain specific. */
    BMS_ERR_IDENT_ATTEMPTS              = 80,   ///< The identification attempts has expired.
    BMS_ERR_IDENT_FAILED                = 81,   ///< The identification has failed.
    BMS_ERR_DRDY_TIMEOUT                = 82,   ///< Data ready ping assertion timeout.
    BMS_ERR_NAK                         = 83,   ///< Not Acknowledge is received.
    BMS_ERR_ACK                         = 84,   ///< Acknowledge is not received.
    BMS_ERR_UNEXPECTED_RESP             = 85,   ///< Unexpected response to a command.
    BMS_ERR_RESPONCE                    = 86,   ///< Response error.
    BMS_ERR_RESP_LENGTH                 = 87,   ///< Unexpected response data length.
    BMS_ERR_CRC_INCORRECT               = 88,   ///< Incorrect CRC in the data array.
    BFE_ERR_EEPROM                      = 89,   ///< The EEPROM data is corrupted.

    /* Start of register specific */
    BMS_ERR_REGISTER_NOT_WRITABLE       = 91,   ///< The selected register is not writable.
    BMS_ERR_INVALID_REG_PAGE            = 92,   ///< Invalid register page is selected.
    BMS_ERR_INVALID_REG_ADDRESS         = 93,   ///< Invalid register address.
    BMS_ERR_INVALID_REG_DATA_LENGTH     = 94,   ///< Invalid input data length.
    BMS_ERR_INVALID_DEV_ADDRESS         = 95,   ///< Invalid device address.

    /* Start of APP specific. */
    BMS_ERR_COMM_ERR_CLR                = 100,  ///< Cannot resolve communication error.
    BMS_ERR_CELL_BALANCE                = 101,  ///< Cell balancing error.
    BMS_ERR_FSP                         = 102,  ///< Error return in Flexible Software Package

} e_bms_err_t;

/** Battery Front End measured data structure */
typedef struct
{
    uint16_t v_batt;                            ///< Battery voltage.
    uint16_t i_batt;                            ///< Battery current.
    uint16_t v_cells[12];                       ///< Cells voltage.
    uint16_t v_sec_ref;                         ///< Secondary reverence voltage.
    uint16_t v_ic_temp;                         ///< IC temperature.
    uint16_t v_ext_temps[4];                    ///< External temperatures.
} st_bfe_meas_t;

/** Battery Front End fault data structure */
typedef struct
{
    bool     flt_oscillator;                    ///< Oscillator fault
    bool     flt_wdt_timout;                    ///< Watchdog timeout fault
    bool     flt_ow_vbat;                       ///< Open wire fault on VBAT connection
    bool     flt_ow_vss;                        ///< Open wire fault on VSS connection
    bool     flt_parity;                        ///< Register checksum (parity) error
    bool     flt_v_ref;                         ///< Voltage reference fault
    bool     flt_v_reg;                         ///< Voltage regulator fault
    bool     flt_temp_mux;                      ///< Temperature multiplexer error
    uint16_t flt_over_temp;                     ///< Over-temperature fault
    uint16_t flt_overvolt;                      ///< Overvoltage fault
    uint16_t flt_undervolt;                     ///< Undervoltage fault
    uint16_t flt_open_wire;                     ///< Open wire fault
} st_bfe_faults_t;

/** Battery Front End cell bits in data registers. */
/* Please, refer to Register Map table from ISL94212 Datasheet! */
typedef enum
{
    BFE_REG_MASK_CELL_1             = 0x0001,   ///< 00000000 00000001 Cell 1
    BFE_REG_MASK_CELL_2             = 0x0002,   ///< 00000000 00000010 Cell 2
    BFE_REG_MASK_CELL_3             = 0x0004,   ///< 00000000 00000100 Cell 3
    BFE_REG_MASK_CELL_4             = 0x0008,   ///< 00000000 00001000 Cell 4
    BFE_REG_MASK_CELL_5             = 0x0010,   ///< 00000000 00010000 Cell 5
    BFE_REG_MASK_CELL_6             = 0x0020,   ///< 00000000 00100000 Cell 6
    BFE_REG_MASK_CELL_7             = 0x0040,   ///< 00000000 01000000 Cell 7
    BFE_REG_MASK_CELL_8             = 0x0080,   ///< 00000000 10000000 Cell 8
    BFE_REG_MASK_CELL_9             = 0x0100,   ///< 00000001 00000000 Cell 9
    BFE_REG_MASK_CELL_10            = 0x0200,   ///< 00000010 00000000 Cell 10
    BFE_REG_MASK_CELL_11            = 0x0400,   ///< 00000100 00000000 Cell 11
    BFE_REG_MASK_CELL_12            = 0x0800    ///< 00001000 00000000 Cell 12
} e_bfe_data_reg_mask_cell_t;

/** Battery Front End temperature bits in data registers. */
/* Please, refer to Register Map table from ISL94212 Datasheet! */
typedef enum
{
    BFE_REG_MASK_TEMP_IC            = 0x0001,   ///< 00000000 00000001 Internal temperature
    BFE_REG_MASK_TEMP_EXT1          = 0x0002,   ///< 00000000 00000010 ExT1
    BFE_REG_MASK_TEMP_EXT2          = 0x0004,   ///< 00000000 00000100 ExT2
    BFE_REG_MASK_TEMP_EXT3          = 0x0008,   ///< 00000000 00001000 ExT3
    BFE_REG_MASK_TEMP_EXT4          = 0x0010,   ///< 00000000 00010000 ExT4
} e_bfe_data_reg_mask_temp_t;

/** Battery Front End cell bits in data registers. */
/* Please, refer to Register Map table from ISL94212 Datasheet! */
typedef enum
{
    BFE_REG_CELL_1                  = 0x01,     ///< 00000001 Cell 1
    BFE_REG_CELL_2                  = 0x02,     ///< 00000010 Cell 2
    BFE_REG_CELL_3                  = 0x03,     ///< 00000011 Cell 3
    BFE_REG_CELL_4                  = 0x04,     ///< 00000100 Cell 4
    BFE_REG_CELL_5                  = 0x05,     ///< 00000101 Cell 5
    BFE_REG_CELL_6                  = 0x06,     ///< 00000110 Cell 6
    BFE_REG_CELL_7                  = 0x07,     ///< 00000111 Cell 7
    BFE_REG_CELL_8                  = 0x08,     ///< 00001000 Cell 8
    BFE_REG_CELL_9                  = 0x09,     ///< 00001001 Cell 9
    BFE_REG_CELL_10                 = 0x0A,     ///< 00001010 Cell 10
    BFE_REG_CELL_11                 = 0x0B,     ///< 00001011 Cell 11
    BFE_REG_CELL_12                 = 0x0C      ///< 00001100 Cell 12
} e_bfe_data_reg_cell_num_t;

/** Battery Front End cell balancing masks. */
typedef enum
{
    BFE_REG_BAL_MASK_NO_CELL        = 0x0000,   ///< No cells to be balanced
    BFE_REG_BAL_MASK_ODD_CELL       =
            ( BFE_REG_MASK_CELL_1
            | BFE_REG_MASK_CELL_3
            | BFE_REG_MASK_CELL_5
            | BFE_REG_MASK_CELL_7
            | BFE_REG_MASK_CELL_9
            | BFE_REG_MASK_CELL_11),            ///< All odd cells to be balanced
    BFE_REG_BAL_MASK_EVEN_CELL      =
            ( BFE_REG_MASK_CELL_2
            | BFE_REG_MASK_CELL_4
            | BFE_REG_MASK_CELL_6
            | BFE_REG_MASK_CELL_8
            | BFE_REG_MASK_CELL_10
            | BFE_REG_MASK_CELL_12),            ///< All even cells to be balanced
    BFE_REG_BAL_MASK_CUSTOM         = 0x0000    ///< Custom balancing pattern
} e_bfe_bal_cell_mask_t;

/** Battery front end initialization */
typedef enum
{
    BFE_UNINITILIZED,                           ///< BFE is initialized.
    BFE_INITILIZED                              ///< BFE is uninitialized.
} e_bfe_ctrl_init_t;

/** Stand-alone or Stacked (Daisy chain) operation. */
typedef enum
{
    BFE_CONFIG_STANDALONE,                      ///< BFE operates stand-alone.
    BFE_CONFIG_DAISY_CHAIN                      ///< BFE operates in a stack.
} e_bfe_cfg_stack_mode_t;

/** Cell balancing control options. */
typedef enum
{
    BFE_CELL_BALANCE_ENABLE,                    ///< Cell balancing is enabled.
    BFE_CELL_BALANCE_INHIBIT                    ///< Cell balancing is inhibited.
} e_bfe_ctrl_bal_t;

/** Wire scan current source/sink values. */
/* Please, refer to ISL94212 Datasheet! */
typedef enum
{
    BFE_WIRE_I_SCAN_150U,                       ///< Wire scan current 150 uA
    BFE_WIRE_I_SCAN_1M                          ///< Wire scan current 1 mA
} e_bfe_i_scan_t;

/** Balance modes. */
/* Please, refer to ISL94212 Datasheet! */
typedef enum
{
    BFE_BALANCE_MODE_OFF,                       ///< The BFE balance mode is off
    BFE_BALANCE_MODE_MANUAL,                    ///< The BFE balance is in manual balance mode
} e_bfe_bal_mode_t;

/** Scan command types */
typedef enum
{
    BFE_SCAN_VOLTAGES,                          ///< Scan all voltages command.
    BFE_SCAN_TEMPS,                             ///< Scan all temperatures command.
    BFE_SCAN_MIXED,                             ///< Mixed scan command.
    BFE_SCAN_WIRES,                             ///< Scan wires command.
    BFE_SCAN_ALL                                ///< Scan all command.
} e_bfe_scan_cmnd_types_t;

/** Read/write data */
typedef enum
{
    BFE_READ_REG  = 0x0000,                     ///< Read register
    BFE_WRITE_REG = 0x0001                      ///< Write register
} e_bfe_data_dir_t;

/** Battery Front End status variables */
typedef struct
{
    uint32_t          serial_number[BFE_CFG_STA_DEV]; ///< The (master) device serial number.

    e_bfe_ctrl_init_t initialized;                    ///< The BFE is initialized.
    bool              in_sleep_mode;                  ///< The device is in sleep mode.
    bool              balancing;                      ///< The device is balancing cells.

    uint16_t          ic_temp_limit[BFE_CFG_STA_DEV]; ///< Internal temperature limit (150 deg. C for ISL94212)
    uint16_t          v_trim[BFE_CFG_STA_DEV];        ///< Nominal cell voltage

    uint32_t          identify_ctr;                   ///< Stack identification counter
    uint16_t          stacked_devs;                   ///< Total number of detected stacked devices.
} st_bfe_status_t;

/** Battery Front End setup variables */
typedef struct
{
    /* Fault limits. */
    uint16_t           overvoltage_limit;             ///< Overvoltage limit.
    uint16_t           undervoltage_limit;            ///< Undervoltage limit.
    uint16_t           ext_temp_limit;                ///< External temperature limit.

    /* Setup registers. */
    e_bfe_i_scan_t     i_wire_scan;                   ///< Wire scan current.
    uint16_t           flt_tot_samples;               ///< Fault samples totalizer.
    e_bfe_bal_mode_t   balance_mode;                  ///< BFE balance mode.

    uint16_t           cells_cfg[BFE_CFG_STA_DEV];    ///< Cell inputs configuration.
    uint16_t           temps_cfg[BFE_CFG_STA_DEV];    ///< Temperature inputs configuration.
    uint16_t           temp_flt_mon[BFE_CFG_STA_DEV]; ///< Temperatures to be monitored for faults.
} st_bfe_setup_t;

/** Battery Front End control structure */
typedef struct
{
    uint16_t           bal_pattern;                   ///< Sell balancing pattern.
    uint16_t           balance_cell_sel[BFE_CFG_STA_DEV]; ///< Select cells to be balanced array

    uint16_t           wd_time;                       ///< Watchdog time (0 for disable; 0x3F for 63s; 0x7F for 128min).

    st_bfe_status_t    status;                        ///< BFE status structure
    st_bfe_setup_t     setup;                         ///< BFE setup structure
} st_bfe_ctrl_t;

/** Battery Front End configuration structure */
typedef struct
{
    e_bfe_cfg_stack_mode_t  mode;                     ///< Stand-alone or stacked operation.
    uint16_t                d_rate_dch;               ///< Daisy chain data rate.
} st_bfe_cfg_t;

/** Shared Interface definition for BFE */
typedef struct st_bfe_api
{
    /** Initialize Battery Front End. (Stand-alone/ Daisy chain)
     * @par Implemented as
     * - @ref R_BFE_Open()
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     * @param[in]  p_bfe_cfg        Pointer to battery front end configuration.
     */
    e_bms_err_t ( * open)(st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_cfg_t const * const p_bfe_cfg);

    /** Deinitialize Battery Front End (Stand-alone/ Daisy chain).
     * @par Implemented as
     * - @ref R_BFE_Close()
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     */
    e_bms_err_t ( * close)(st_bfe_ctrl_t * p_bfe_ctrl);

    /** Reset Battery Front End. (Stand-alone/ Daisy chain)
     * @par Implemented as
     * - @ref R_BFE_Reset()
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     */
    e_bms_err_t ( * reset)(st_bfe_ctrl_t * p_bfe_ctrl);

    /** Test the communication between the microcontroller, master device and the devices
     * in the stack. (Stand-alone/ Daisy chain)
     * @par Implemented as
     * - @ref R_BFE_CommTest()
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     */
    e_bms_err_t ( * commTest)(st_bfe_ctrl_t * p_bfe_ctrl);

    /** Put Battery Front End to sleep mode. (Stand-alone/ Daisy chain)
     * @par Implemented as
     * - @ref R_BFE_Sleep()
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     */
    e_bms_err_t ( * sleep)(st_bfe_ctrl_t * p_bfe_ctrl);

    /** Wake up a Battery Front End. (Stand-alone/ Daisy chain)
     * @par Implemented as
     * - @ref R_BFE_WakeUp()
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     */
    e_bms_err_t ( * wakeUp)(st_bfe_ctrl_t * p_bfe_ctrl);

    /** Wake up a device on a random position in the stack. (Daisy chain only)
     * @par Implemented as
     * - @ref R_BFE_RandomWakeUp()
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     */
    e_bms_err_t ( * randomWakeUp)(st_bfe_ctrl_t * p_bfe_ctrl);

    /** Perform wire scan, all battery, cells, internal temperature and ExTn inputs voltage
     * scan and read. (Stand-alone/ Daisy chain)
     * @par Implemented as
     * - @ref R_BFE_VoltsTempsAllGet()
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     * @param[out] p_meas_data      Pointer to measured data structure.
     */
    e_bms_err_t ( * voltTempAllGet)(st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_meas_t * p_meas_data);

    /** Perform battery and all cells voltage scan and read. (Stand-alone/ Daisy chain)
     * @par Implemented as
     * - @ref R_BFE_VAllGet()
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     * @param[out] p_meas_data      Pointer to measured data structure.
     */
    e_bms_err_t ( * vAllGet)(st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_meas_t * p_meas_data);

    /** Perform battery voltage scan and read. (Stand-alone/ Daisy chain)
     * @par Implemented as
     * - @ref R_BFE_VBattGet()
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     * @param[out] p_meas_data      Pointer to measured data structure.
     */
    e_bms_err_t ( * vBattGet)(st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_meas_t * p_meas_data);

    /** Perform battery, all cells, internal temperature and ExT1 voltage scan and read. (Stand-alone/ Daisy chain)
     * @par Implemented as
     * - @ref R_BFE_VMixGet()
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     * @param[out] p_meas_data      Pointer to measured data structure.
     */
    e_bms_err_t ( * vMixGet)(st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_meas_t * p_meas_data);

    /** Perform internal temperature and ExTn voltages scan and read. (Stand-alone/ Daisy chain)
     * @par Implemented as
     * - @ref R_BFE_TempsGet()
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     * @param[out] p_meas_data      Pointer to measured data structure.
     */
    e_bms_err_t ( * tempsGet)(st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_meas_t * p_meas_data);

    /** Read the fault registers (Stand-alone/ Daisy chain).
     * @par Implemented as
     * - @ref R_BFE_FaultsAllRead()
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     * @param[in]  p_bfe_cfg        Pointer to battery front end configuration.
     * @param[out] p_faults         Pointer to faults data structure.
     */
    e_bms_err_t ( * faultsAllRead)(st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_faults_t * p_faults);

    /** Check fault pin, reads fault registers and calls user callback. (Stand-alone/ Daisy chain)
     * @par Implemented as
     * - @ref R_BFE_FaultsCheck()
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     */
    e_bms_err_t ( * faultsCheck)(st_bfe_ctrl_t * p_bfe_ctrl);

    /** Clear all faults. (Stand-alone/ Daisy chain)
     * @par Implemented as
     * - @ref R_BFE_FaultsAllClr()
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     * @param[out] p_faults         Pointer to faults data structure.
     */
    e_bms_err_t ( * faultsAllClear)(st_bfe_ctrl_t * p_bfe_ctrl);

    /** Select cells and enable or inhibit cell balancing. (Stand-alone/ Daisy chain)
     * @par Implemented as
     * - @ref R_BFE_BalanceCtrl()
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     * @param[in]  bal_ctrl         Select action - enable or inhibit.
     */
    e_bms_err_t ( * balanceControl)(st_bfe_ctrl_t * p_bfe_ctrl, e_bfe_ctrl_bal_t bal_ctrl);

    /** Checks for memory corruption. (Stand-alone/ Daisy chain)
     * @par Implemented as
     * - @ref R_BFE_ConfRegCheck()
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     */
    e_bms_err_t ( * memoryCheck)(st_bfe_ctrl_t * p_bfe_ctrl);

    /** Configure Battery Front End. (Stand-alone/ Daisy chain)
     * @par Implemented as
     * - @ref R_BFE_DeviceSetup()
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     */
    e_bms_err_t ( * setup)(st_bfe_ctrl_t * p_bfe_ctrl);

    /** Enable or disable watchdog timer and select time. (Daisy chain only)
     * @par Implemented as
     * - @ref R_BFE_WatchdogCtrl()
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     */
    e_bms_err_t ( * wdControl)(st_bfe_ctrl_t * p_bfe_ctrl);

    /** Reset the watchdog timer. (Daisy chain only)
     * @par Implemented as
     * - @ref R_BFE_WatchdogReset()
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     */
    e_bms_err_t ( * wdReset)(st_bfe_ctrl_t * p_bfe_ctrl);

    /** Scan voltages, open wires, temperatures, mixed or all. (Stand-alone/ Daisy chain)
     * @par Implemented as
     * - @ref R_BFE_ScanAll()
     *
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     * @param[in]  cmnd_type        The scan command type.
     */
    e_bms_err_t ( * scan)(st_bfe_ctrl_t * p_bfe_ctrl, e_bfe_scan_cmnd_types_t cmnd_type);

    /** Provides access to the user registers. (Stand-alone/ Daisy chain)
     * @par Implemented as
     * - @ref R_BFE_UserRegsAccess()
     *
     * @param[in]      p_bfe_ctrl   Pointer to battery front end control structure.
     * @param[in/out]  p_data_array Pointer to user registers data array.
     * @param[in]      data_dir     Select data direction - Read or write.
     */
    e_bms_err_t ( * userRegsAccess)(st_bfe_ctrl_t *        p_bfe_ctrl,
                                  uint16_t *               p_data_array,
                                  e_bfe_data_dir_t         data_dir);

    /** Send a command to a single register of a device. (Stand-alone/ Daisy chain)
     * @par Implemented as
     * - @ref R_BFE_SingleRegisterRead()
     *
     * @param[in]  p_bfe_ctrl       Pointer to battery front end control structure.
     * @param[in/out] p_dev_addr    Pointer to device address
     * @param[in/out] p_reg_addr    Pointer to register address
     * @param[in/out] p_reg_data    Pointer to register data
     * @param[in]     r_w_data      Select data direction - Read or write.
     */
    e_bms_err_t ( * singleRegAccess)(st_bfe_ctrl_t *       p_bfe_ctrl,
                                   uint16_t *              p_dev_addr,
                                   uint32_t *              p_reg_addr,
                                   uint16_t *              p_reg_data,
                                   e_bfe_data_dir_t        r_w_data);
} bfe_api_t;

/** This structure comprises everything that is needed to use an instance of this interface. */
typedef struct st_bfe_instance
{
    st_bfe_ctrl_t const *   p_ctrl;     ///< Pointer to the control structure for this instance.
    st_bfe_cfg_t  const *   p_cfg;      ///< Pointer to the configuration structure for this instance.
    bfe_api_t     const *   p_api;      ///< Pointer to the API structure for this instance.
} bfe_instance_t;

/**********************************************************************************************************************
 External global variables
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Exported global functions
 *********************************************************************************************************************/

#endif /* BFE_R_BFE_API_H_ */

/*******************************************************************************************************************//**
 * @} (end defgroup IOPORT_API)
 **********************************************************************************************************************/
