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
 * @file          r_bfe_isl94212.h
 * @Version       1.0
 * @brief         Contains macros, data structures and declarations of functions used in r_bfe_isl94212.c
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 16.08.2021 1.00     First Release
 *         : 12.11.2021 1.01     Improved communications timing. Minor bugs are fixed.
 *********************************************************************************************************************/

/*******************************************************************************************************************//**
 * @addtogroup ISL94212
 * @{
 **********************************************************************************************************************/

#ifndef BFE_R_BFE_ISL94212_H_
#define BFE_R_BFE_ISL94212_H_

/**********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 *********************************************************************************************************************/
/* Generic headers */
#include <stdint.h>
#include <stdbool.h>

/* BFE related headers */
#include "r_bfe_common.h"
#include "r_bfe_api.h"
#include "r_bfe_crc4.h"

#include "hal_data.h"

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/

/* Commands byte lengths */
#define BFE_READ_CMND_LGTH_B_STA            (2U)         ///< Length of read command [Bytes] (stand-alone)
#define BFE_WRITE_CMND_LGTH_B_STA           (3U)         ///< Length of write command [Bytes] (stand-alone)
#define BFE_READ_CMND_LGTH_B_DCH            (3U)         ///< Length of read command [Bytes] (daisy chain)
#define BFE_WRITE_CMND_LGTH_B_DCH           (4U)         ///< Length of write command [Bytes] (daisy chain)
#define BFE_MAX_CMND_LGTH_B                 (4U)         ///< The maximal length of a command [Bytes]

/* SPI related definitions */
#define BFE_SPI_RX_BUFF_LGTH                (5U)         ///< Max length of the Rx SPI buffer [Bytes]
#define BFE_SPI_TX_BUFF_LGTH                (5U)         ///< Max length of the Tx SPI buffer [Bytes]
#define BFE_SPI_RESP_DAT_LGTH_STA           (2U)         ///< Device response data length [Bytes] (stand-alone)
#define BFE_SPI_ONE_BYTE_RESP_LGTH          (1U)         ///< Single response data unit length [Bytes] (daisy chain)
#define BFE_SPI_NORM_RESP_DAT_LGTH_DCH      (4U)         ///< Norm register read data length in Bytes (daisy chain)
#define BFE_SPI_RESP_TYPE1_LGTH_DCH         (4U)         ///< Data response type 1 length [Bytes] (daisy chain)
#define BFE_SPI_RESP_TYPE2_LGTH_DCH         (3U)         ///< Data response type 2 length [Bytes] (daisy chain)
#define BFE_SPI_ALL_RESP_LGTH_MIN           (22U)        ///< Minimum received Bytes after read all cmnd (daisy chain)
#define BFE_SPI_MAX_CNT                     (0xFFFFFF)   ///< Max wait count for SPI interrupt time-out operation
#define BFE_SPI_MIN_CNT                     (0xFF0000)   ///< Min wait count for SPI interrupt time-out operation
#define BFE_SPI_TO_MAX_CNT                  (0xFFFFFF)   ///< Max wait count for SPI receive time-out operation
#define BFE_SPI_TO_MIN_CNT                  (0xF00000)   ///< Min wait count for SPI receive time-out operation

/* Registers data filed size definitions */
#define BFE_MAX_6_BIT_DAT_VAL               (0x003F)     ///< Max value in a 6-bit data field
#define BFE_MAX_14_BIT_DAT_VAL              (0x3FFF)     ///< Max value in a 14-bit data field

/* Delays */
#define BFE_START_UP_DELAY_MS               (30U)        ///< Start-up delay (power on or enable)
#define BFE_ENBL_PIN_HOLD_DELAY_MS          (1U)         ///< Enable pin hold down delay (on reset)

/* Scan times. From start of scan to registers loaded to DATA READY going low. */
#define BFE_SCAN_VOLT_PROC_US               (847U)       ///< Scan voltages processing time
#define BFE_SCAN_TEMP_PROC_US               (2959U)      ///< Scan temperatures processing time
#define BFE_SCAN_MIXED_PROC_US              (913U)       ///< Scan mixed processing time
#define BFE_SCAN_WIRES_PROCE_US             (65000U)     ///< Scan wires processing time
#define BFE_SCAN_ALL_PROC_US                (69500U)     ///< Scan all processing time

/* Measure times. From start of Measurement to registers loaded to DATA READY going low. */
#define BFE_MEAS_CELL_PROC_US               (198U)       ///< Measure cell voltage processing time
#define BFE_MEAS_V_BATT_PROC_US             (143U)       ///< Measure battery voltage processing time
#define BFE_MEAS_IC_TEMP_PROC_US            (121U)       ///< Measure internal temperature processing time
#define BFE_MEAS_EXT_IN_PROC_US             (2772U)      ///< Measure external input processing time
#define BFE_MEAS_V_REF_PROC_US              (2772U)      ///< Measure secondary voltage reference time

/* Other definitions. Please, refer to ISL94212 Datasheet! */
#define BFE_MAX_CELLS_PER_IC                (12U)        ///< Max number of cells, connected in series to single BFE
#define BFE_MIN_CELLS_PER_IC                (6U)         ///< Min number of cells, connected in series to single BFE
#define BFE_MAX_STACKED_DEV                 (14U)        ///< Max number of stacked devices
#define BFE_TEMP_INPUTS                     (4U)         ///< Number of Measured external temperatures
#define BFE_SCAN_CNTR_MAX                   (15U)        ///< Maximal value of the scan counter

#define BFE_14BIT_REG_S_MAX                 (0x1FFF)     ///< Maximal positive value of 14bit signed register
#define BFE_14BIT_FULL_REG                  (0x3FFF)     ///< Highest value written in 14bit register

#define BFE_9BIT_REG_S_MAX                  (0xFF)       ///< Maximal positive value of 9bit signed register
#define BFE_9BIT_FULL_REG                   (0x1FF)      ///< Highest value written in 9bit register

#define BFE_COEFF_A_SHFT_VAL                (32U)        ///< Reference coefficient A shifting right value

#define BFE_WDT_PASSWORD                    (0x3A00)     ///< Watchdog timer password, needed for disabling
#define BFE_WDT_MAX_TIME                    (0x7F)       ///< Watchdog timer max time value

#define BFE_BALANCE_TIME_MASK               (0xFF80)     ///< Balance mask Watchdog/ Balance register
#define BFE_BSP_MAX                         (0x0C)       ///< Maximal value of balance status pointer
#define BFE_BSP_NUM                         (13U)        ///< Number of values for balance status pointer

#define BFE_COMS_SEL_TOP_DEV                (0x2000)     ///< Communication select top device bit configuration
#define BFE_IDENTIFY_EXIT                   (0x003F)     ///< Exit identification data

#define BFE_FLT_TOT_MAX                     (0x07)       ///< Maximal value of fault samples totalizer

#define BFE_READ_ALL_MAX_DATA_PER_DEV       (13U)        ///< Maximal data bytes per device in read all command

#define BFE_US_PER_S                        (1000000U)   ///< Microsecond to second conversion factor

#define BFE_USR_REG_NUM                     (2U)         ///< Number of user registers

#define BFE_RST_PROC_US                     (1000U)      ///< Reset processing time

/* Multiple response data offsets. */
#define BFE_VALL_VBAT_OFFST                 (12U)        ///< Battery voltage offset in all volts multiple response
#define BFE_TALL_VREF_OFFST                 (1U)         ///< Secondary reference voltage offset in all temps response
#define BFE_TALL_TIC_OFFST                  (6U)         ///< Internal temperature offset in all temps response
#define BFE_SALL_DEV_STP_OFFST              (4U)         ///< Device setup offset in all setup response
#define BFE_SALL_WDT_BAL_OFFST              (8U)         ///< Watchdog / balance time offset in all setup response
#define BFE_SALL_BAL_STA_OFFST              (9U)         ///< Balance status offset in all setup response
#define BFE_SALL_BAL_STP_OFFST              (10U)        ///< Balance setup offset in all setup response
#define BFE_SALL_EXT_OFFST                  (11U)        ///< External temperature limit offset in all setup response
#define BFE_SALL_UVL_OFFST                  (12U)        ///< Undervoltage limit offset in all setup response
#define BFE_SALL_OVL_OFFST                  (13U)        ///< Overvoltage limit offset in all setup response

/* Register bit masks.  Please, refer to ISL94212 Datasheet! */
#define BFE_DSR_EOB_MASK                    (0x0008)     ///< Device setup register EOB bit mask
#define BFE_DSR_SCAN_MASK                   (0x0010)     ///< Device setup register SCAN bit mask
#define BFE_DSR_ISCN_MASK                   (0x0020)     ///< Device setup register ISCN bit mask
#define BFE_DSR_BDDS_MASKT                  (0x0080)     ///< Device setup register BDDS bit mask
#define BFE_DSR_CHB_MASK                    (BFE_DSR_SCAN_MASK | BFE_DSR_ISCN_MASK | BFE_DSR_BDDS_MASKT)

#define BFE_BSR_BEN_BIT_MASK                (0x0200)     ///< Balance setup register BEN bit mask

#define BFE_FSTR_OSC_BIT_MASK               (0x0004)     ///< Fault status register OSC bit mask
#define BFE_FSTR_WDGF_BIT_MASK              (0x0008)     ///< Fault status register WDGF bit mask
#define BFE_FSTR_OT_BIT_MASK                (0x0010)     ///< Fault status register OT bit mask
#define BFE_FSTR_OV_BIT_MASK                (0x0020)     ///< Fault status register OV bit mask
#define BFE_FSTR_UV_BIT_MASK                (0x0040)     ///< Fault status register UV bit mask
#define BFE_FSTR_OW_BIT_MASK                (0x0080)     ///< Fault status register OW bit mask
#define BFE_FSTR_OVB_BIT_MASK               (0x0100)     ///< Fault status register OVB bit mask
#define BFE_FSTR_OVS_BIT_MASK               (0x0200)     ///< Fault status register OVS bit mask
#define BFE_FSTR_PAR_BIT_MASK               (0x0400)     ///< Fault status register PAR bit mask
#define BFE_FSTR_REF_BIT_MASK               (0x0800)     ///< Fault status register REF bit mask
#define BFE_FSTR_REG_BIT_MASK               (0x1000)     ///< Fault status register REG bit mask
#define BFE_FSTR_MUX_BIT_MASK               (0x2000)     ///< Fault status register MUX bit mask

#define BFE_FSR_TOT_BIT_MASK                (0x00E0)     ///< Fault setup register TOT bits mask

#define BFE_ALL_CELLS_BIT_MASK              (0x0FFF)     ///< All cells bits mask
#define BFE_ALL_TEMPS_BIT_MASK              (0x001F)     ///< All temperature bits mask

#define BFE_IDENTIFY_COMS_SEL_MASK          (0x3000)     ///< Communication select bits mask

#define BFE_MAX_6_BIT_DATA_MASK             (0x003F)     ///< Mask for 6-bit data field
#define BFE_MAX_14_BIT_DATA_MASK            (0x3FFF)     ///< Mask for 14-bit data field

#define BFE_REG_PAGE_ADDR_MASK              (0x00000FFF) ///< Register page and address mask
#define BFE_REG_WRITE_MASK                  (0x08000000) ///< Register is writable bit mask
#define BFE_REG_DCH_ONLY_MASK               (0x04000000) ///< Register daisy chain only bit mask
#define BFE_REG_BCAST_MASK                  (0x02000000) ///< Register broadcast bit mask
#define BFE_REG_ACK_MASK                    (0x01000000) ///< Register ACK response bit mask
#define BFE_REG_RESP_STA_MASK               (0x10000000) ///< Register expect response in stand-alone
#define BFE_REG_RESP_LGTH_MASK              (0x00FF0000) ///< Response length mask daisy chain

/* Register bit offsets.  Please, refer to ISL94212 Datasheet! */
#define BFE_FSR_EOB_OFFSET                  (8U)         ///< Fault setup register TST bits offset
#define BFE_FSR_TOT_OFFSET                  (5U)         ///< Fault setup register TOT bits offset

#define BFE_SPI_TEST_WRITE_MSG              (0x3A5A)     ///< Test message to be written to user register 1

#define BFE_SPI_RESP_REG_MAX                (14U)        ///< Max device response register length
#define BFE_SPI_RESP_DAT_MAX                (43U)        ///< Max device response data length
#define BFE_HIGHEST_DEV_ADDR                (BFE_CFG_STA_DEV) ///< Assign highest device address from stack

/**********************************************************************************************************************
 Global Typedef definitions
 *********************************************************************************************************************/

/** Daisy chain device addresses */
typedef enum
{
    BFE_DAISY_CHAIN_IDENTIFY        = 0x00,   ///< Identify devices
    BFE_DAISY_CHAIN_DEVICE1         = 0x01,   ///< The master device address
    BFE_DAISY_CHAIN_DEVICE2         = 0x02,   ///< Device 2 address
    BFE_DAISY_CHAIN_DEVICE3         = 0x03,   ///< Device 3 address
    BFE_DAISY_CHAIN_DEVICE4         = 0x04,   ///< Device 4 address
    BFE_DAISY_CHAIN_DEVICE5         = 0x05,   ///< Device 5 address
    BFE_DAISY_CHAIN_DEVICE6         = 0x06,   ///< Device 6 address
    BFE_DAISY_CHAIN_DEVICE7         = 0x07,   ///< Device 7 address
    BFE_DAISY_CHAIN_DEVICE8         = 0x08,   ///< Device 8 address
    BFE_DAISY_CHAIN_DEVICE9         = 0x09,   ///< Device 9 address
    BFE_DAISY_CHAIN_DEVICE10        = 0x0A,   ///< Device 10 address
    BFE_DAISY_CHAIN_DEVICE11        = 0x0B,   ///< Device 11 address
    BFE_DAISY_CHAIN_DEVICE12        = 0x0C,   ///< Device 12 address
    BFE_DAISY_CHAIN_DEVICE13        = 0x0D,   ///< Device 13 address
    BFE_DAISY_CHAIN_DEVICE14        = 0x0E,   ///< Device 14 address
    BFE_DAISY_CHAIN_ADDRESS_ALL     = 0x0F    ///< Address all devices
} e_bfe_dev_addr_t;

typedef enum
{
    BFE_BMD_BITS_OFF                = 0x00,   ///< The BFE balance mode is off
    BFE_BMD_BITS_MANUAL             = 0x01,   ///< The BFE balance is in manual balance mode
} e_bfe_bal_bmd_val_t;

/** Daisy chain data rates. */
/* Please, refer to ISL94212 Datasheet! */
typedef enum
{
    BFE_D_RT_62_5_KHZ               = 0,      ///< Data rate 62500 kHz
    BFE_D_RT_125_KHZ                = 1,      ///< Data rate 125000 kHz
    BFE_D_RT_250_KHZ                = 2,      ///< Data rate 250000 kHz
    BFE_D_RT_500_KHZ                = 3       ///< Data rate 500000 kHz
} e_bfe_dch_data_rate_t;

/** BFE communication delays. */
/* Please, refer to ISL94212 Datasheet! */
typedef struct
{
    /* Maximum time intervals between rising CS and assertion of data ready. */
    uint32_t    max_resp_delay;               ///< Maximum delay between command and response
    uint32_t    max_bytes_delay;              ///< Maximum delay between two adjacent response bytes
    uint32_t    min_wait_time_us;             ///< Minimum communications wait time
    uint32_t    sleep_time_us;                ///< Maximum wait time for devices entering sleep mode
    uint32_t    wakeup_time_us;               ///< Maximum wait time for devices to exit sleep mode
} st_bfe_comm_timeouts_t;

/** Watchdog timeouts. */
/* Please, refer to ISL94212 Datasheet! */
typedef enum
{
    BFE_WDT_DSBL                    = 0x00,   ///< WD disabled
    BFE_WDT_1S                      = 0x01,   ///< 1 s timeout
    BFE_WDT_2S                      = 0x02,   ///< 2 s timeout
    BFE_WDT_3S                      = 0x03,   ///< 3 s timeout
    BFE_WDT_4S                      = 0x04,   ///< 4 s timeout
    BFE_WDT_5S                      = 0x05,   ///< 5 s timeout
    BFE_WDT_6S                      = 0x06,   ///< 6 s timeout
    BFE_WDT_7S                      = 0x07,   ///< 7 s timeout
    BFE_WDT_8S                      = 0x08,   ///< 8 s timeout
    BFE_WDT_9S                      = 0x09,   ///< 9 s timeout
    BFE_WDT_10S                     = 0x0A,   ///< 10 s timeout
    /* Add needed values between 10 and 62 s with 1 s step. */
    BFE_WDT_62S                     = 0x3E,   ///< 62 s timeout
    BFE_WDT_63S                     = 0x3F,   ///< 63 s timeout
    BFE_WDT_2MIN                    = 0x40,   ///< 2 min timeout
    BFE_WDT_4MIN                    = 0x41,   ///< 4 min timeout
    BFE_WDT_6MIN                    = 0x42,   ///< 6 min timeout
    BFE_WDT_8MIN                    = 0x43,   ///< 8 min timeout
    BFE_WDT_10MIN                   = 0x44,   ///< 10 min timeout
    /* Add needed values between 10 and 126 min with 2 min step. */
    BFE_WDT_126MIN                  = 0x7E,   ///< 126 min timeout
    BFE_WDT_128MIN                  = 0x7F,   ///< 128 min timeout
} e_bfe_wd_timeouts_t;

/** Totalizer samples. */
/* Please, refer to ISL94212 Datasheet! */
typedef enum
{
    BFE_TOT_1_SMPL                  = 0x00,   ///< 1 totalizer sample
    BFE_TOT_2_SMPL                  = 0x01,   ///< 2 totalizer sample
    BFE_TOT_4_SMPL                  = 0x02,   ///< 3 totalizer sample
    BFE_TOT_8_SMPL                  = 0x03,   ///< 4 totalizer sample
    BFE_TOT_16_SMPL                 = 0x04,   ///< 5 totalizer sample
    BFE_TOT_32_SMPL                 = 0x05,   ///< 6 totalizer sample
    BFE_TOT_64_SMPL                 = 0x06,   ///< 7 totalizer sample
    BFE_TOT_128_SMPL                = 0x07,   ///< 8 totalizer sample
} e_bfe_tot_samples_t;

/** Battery Front End registers full addresses that that contains register page, address, response length etc.
  * Register full address decoding.
  *  b0  - b5 : register address
  *  b8  - b10: register page
  *  b16 - b23: response length in bytes in daisy chain mode
  *  b24      : register is '0' ACK is not expected, '1' ACK is expected
  *  b25      : register is '0' cannot be broadcasted, '1' can be broadcasted
  *  b26      : register is '0' universal, '1' daisy chain only
  *  b27      : register is '0' read only, '1' writable
  *  b28      : register '0' does not, '1' does return response in stand-alone operation. */
/* Please, refer to Register Map table from ISL94212 Datasheet! */
typedef enum
{
    /* Page 1 registers */
    BFE_REG_BAT_VOLT            = 0x10040100, ///< VBAT Voltage - Read only

    BFE_REG_CELL_1_VOLT         = 0x10040101, ///< Cell 1 Voltage - Read only
    BFE_REG_CELL_2_VOLT         = 0x10040102, ///< Cell 2 Voltage - Read only
    BFE_REG_CELL_3_VOLT         = 0x10040103, ///< Cell 3 Voltage - Read only
    BFE_REG_CELL_4_VOLT         = 0x10040104, ///< Cell 4 Voltage - Read only
    BFE_REG_CELL_5_VOLT         = 0x10040105, ///< Cell 5 Voltage - Read only
    BFE_REG_CELL_6_VOLT         = 0x10040106, ///< Cell 6 Voltage - Read only
    BFE_REG_CELL_7_VOLT         = 0x10040107, ///< Cell 7 Voltage - Read only
    BFE_REG_CELL_8_VOLT         = 0x10040108, ///< Cell 8 Voltage - Read only
    BFE_REG_CELL_9_VOLT         = 0x10040109, ///< Cell 9 Voltage - Read only
    BFE_REG_CELL_10_VOLT        = 0x1004010A, ///< Cell 10 Voltage - Read only
    BFE_REG_CELL_11_VOLT        = 0x1004010B, ///< Cell 11 Voltage - Read only
    BFE_REG_CELL_12_VOLT        = 0x1004010C, ///< Cell 12 Voltage - Read only

    BFE_REG_ALL_CELL_VOLT       = 0x0428010F, ///< All Cell Voltage - Read only, Daisy chain only

    BFE_REG_IC_TEMP             = 0x10040110, ///< Internal Temperature - Read only

    BFE_REG_EXT_TEMP_IN1_VOLT   = 0x10040111, ///< External Temperature Input 1 Voltage - Read only
    BFE_REG_EXT_TEMP_IN2_VOLT   = 0x10040112, ///< External Temperature Input 2 Voltage - Read only
    BFE_REG_EXT_TEMP_IN3_VOLT   = 0x10040113, ///< External Temperature Input 3 Voltage - Read only
    BFE_REG_EXT_TEMP_IN4_VOLT   = 0x10040114, ///< External Temperature Input 4 Voltage - Read only

    BFE_REG_SEC_REF_VOLT        = 0x10040115, ///< Secondary Reference Voltage - Read only

    BFE_REG_SCAN_COUNT          = 0x10040116, ///< Scan Count - Read only

    BFE_REG_ALL_TEMP_DATA       = 0x0416011F, ///< All Temperature Data - Read only, Daisy chain only

    /* Page 2 registers */
    BFE_REG_OVERVOLT_FAULT      = 0x18040200, ///< Overvoltage Fault - Read/Write
    BFE_REG_UNDERVOLT_FAULT     = 0x18040201, ///< Undervoltage Fault - Read/Write
    BFE_REG_OPEN_WIRE_FAULT     = 0x18040202, ///< Open Wire Fault - Read/Write
    BFE_REG_FAULT_SETUP         = 0x18040203, ///< Fault Setup - Read/Write
    BFE_REG_FAULT_STATUS        = 0x18040204, ///< Fault Status - Read/Write
    BFE_REG_CELL_SETUP          = 0x18040205, ///< Cell Setup - Read/Write
    BFE_REG_OVER_TEMP_FAULT     = 0x18040206, ///< Over Temperature Fault - Read/Write
    BFE_REG_ALL_FAULT_DATA      = 0x0416020F, ///< All Fault Data - Read only, Daisy chain only

    BFE_REG_OVERVOLT_LIMIT      = 0x18040210, ///< Overvoltage Limit - Read/Write
    BFE_REG_UNDERVOLT_LIMIT     = 0x18040211, ///< Undervoltage Limit - Read/Write
    BFE_REG_EXT_TEMP_LIMIT      = 0x18040212, ///< External Temperature Limit - Read/Write

    BFE_REG_BAL_SETUP           = 0x18040213, ///< Balance Setup - Read/Write
    BFE_REG_BAL_STATUS          = 0x18040214, ///< Balance Status - Read/Write
    BFE_REG_WDG_BAL_TIME        = 0x18040215, ///< Watchdog/Balance Time - Read/Write

    BFE_REG_USER_REG1           = 0x18040216, ///< User Register - Read/Write
    BFE_REG_USER_REG2           = 0x18040217, ///< User Register - Read/Write

    BFE_REG_COMMS_SETUP         = 0x10040218, ///< Communications Setup - Read only
    BFE_REG_DEVICE_SETUP        = 0x18040219, ///< Device Setup - Read/Write

    BFE_REG_INT_TEMP_LIMIT      = 0x1004021A, ///< Internal Temperature Limit - Read only

    BFE_REG_SERIAL_NUMBER0      = 0x1004021B, ///< Serial number 0 - Read only
    BFE_REG_SERIAL_NUMBER1      = 0x1004021C, ///< Serial number 1 - Read only

    BFE_REG_TRIM_VOLT           = 0x1004021D, ///< Trim Voltage - Read only

    BFE_REG_ALL_SETUP_DATA      = 0x042B021F, ///< All Setup Data - Read only, Daisy chain only

    BFE_REG_CELL1_BAL_VAL0      = 0x18040220, ///< Cell 1 Balance Value 0 - Read/Write
    BFE_REG_CELL1_BAL_VAL1      = 0x18040221, ///< Cell 1 Balance Value 1 - Read/Write
    BFE_REG_CELL2_BAL_VAL0      = 0x18040222, ///< Cell 2 Balance Value 0 - Read/Write
    BFE_REG_CELL2_BAL_VAL1      = 0x18040223, ///< Cell 2 Balance Value 1 - Read/Write
    BFE_REG_CELL3_BAL_VAL0      = 0x18040224, ///< Cell 3 Balance Value 0 - Read/Write
    BFE_REG_CELL3_BAL_VAL1      = 0x18040225, ///< Cell 3 Balance Value 1 - Read/Write
    BFE_REG_CELL4_BAL_VAL0      = 0x18040226, ///< Cell 4 Balance Value 0 - Read/Write
    BFE_REG_CELL4_BAL_VAL1      = 0x18040227, ///< Cell 4 Balance Value 1 - Read/Write
    BFE_REG_CELL5_BAL_VAL0      = 0x18040228, ///< Cell 5 Balance Value 0 - Read/Write
    BFE_REG_CELL5_BAL_VAL1      = 0x18040229, ///< Cell 5 Balance Value 1 - Read/Write
    BFE_REG_CELL6_BAL_VAL0      = 0x1804022A, ///< Cell 6 Balance Value 0 - Read/Write
    BFE_REG_CELL6_BAL_VAL1      = 0x1804022B, ///< Cell 6 Balance Value 1 - Read/Write
    BFE_REG_CELL7_BAL_VAL0      = 0x1804022C, ///< Cell 7 Balance Value 0 - Read/Write
    BFE_REG_CELL7_BAL_VAL1      = 0x1804022D, ///< Cell 7 Balance Value 1 - Read/Write
    BFE_REG_CELL8_BAL_VAL0      = 0x1804022E, ///< Cell 8 Balance Value 0 - Read/Write
    BFE_REG_CELL8_BAL_VAL1      = 0x1804022F, ///< Cell 8 Balance Value 1 - Read/Write
    BFE_REG_CELL9_BAL_VAL0      = 0x18040230, ///< Cell 9 Balance Value 0 - Read/Write
    BFE_REG_CELL9_BAL_VAL1      = 0x18040231, ///< Cell 9 Balance Value 1 - Read/Write
    BFE_REG_CELL10_BAL_VAL0     = 0x18040232, ///< Cell 10 Balance Value 0 - Read/Write
    BFE_REG_CELL10_BAL_VAL1     = 0x18040233, ///< Cell 10 Balance Value 1 - Read/Write
    BFE_REG_CELL11_BAL_VAL0     = 0x18040234, ///< Cell 11 Balance Value 0 - Read/Write
    BFE_REG_CELL11_BAL_VAL1     = 0x18040235, ///< Cell 11 Balance Value 1 - Read/Write
    BFE_REG_CELL12_BAL_VAL0     = 0x18040236, ///< Cell 12 Balance Value 0 - Read/Write
    BFE_REG_CELL12_BAL_VAL1     = 0x18040237, ///< Cell 12 Balance Value 1 - Read/Write

    BFE_REG_REF_COEF_C          = 0x10040238, ///< Reference Coefficient C - Read only
    BFE_REG_REF_COEF_B          = 0x10040239, ///< Reference Coefficient B - Read only
    BFE_REG_REF_COEF_A          = 0x1004023A, ///< Reference Coefficient A - Read only

    BFE_REG_CELL_BAL_ENABLE     = 0x1004023B, ///< Cell Balance Enabled - Read only

    /* Page 3 registers */
    BFE_REG_SCAN_VOLTS          = 0x02000301, ///< Scan Voltages - Command; no data; broadcast
    BFE_REG_SCAN_TEMPS          = 0x02000302, ///< Scan Temperatures - Command; no data; broadcast
    BFE_REG_SCAN_MIXED          = 0x02000303, ///< Scan Mixed - Command; no data; broadcast
    BFE_REG_SCAN_WIRES          = 0x02000304, ///< Scan Wires - Command; no data; broadcast
    BFE_REG_SCAN_ALL            = 0x02000305, ///< Scan All - Command; no data; broadcast
    BFE_REG_SCAN_CONT           = 0x00000306, ///< Scan Continuous - Commands containing no data
    BFE_REG_SCAN_INHIBIT        = 0x00000307, ///< Scan Inhibit - Commands containing no data

    BFE_REG_MEASURE             = 0x00000308, ///< Measure - Command containing 6-bit data

    BFE_REG_IDENTIFY            = 0x04040309, ///< Identify - Read only, Daisy chain only

    BFE_REG_SLEEP               = 0x0200030A, ///< Sleep - Command; no data; broadcast

    BFE_REG_NAK                 = 0x0404030B, ///< NAK - Read only, daisy chain only
    BFE_REG_ACK                 = 0x0404030C, ///< ACK - Read only, daisy chain only

    BFE_REG_COMMS_FAILURE       = 0x0404030E, ///< Communications Failure - Read only, Daisy chain only

    BFE_REG_WAKEUP              = 0x0200030F, ///< Wake up - Command; no data; broadcast

    BFE_BAL_ENABLE              = 0x03040310, ///< Balance Enable - Read only; no data; broadcast, ACK receive
    BFE_BAL_INHIBIT             = 0x03040311, ///< Balance Inhibit - Read only; no data; broadcast, ACK receive
    BFE_REG_RESET               = 0x00000312, ///< Reset - Command containing no data - Read only
    BFE_CALC_REG_CKSM           = 0x01040313, ///< Command containing no data - Read only, ACK receive
    BFE_CHECK_REG_CKSM          = 0x01040314, ///< Command containing no data - Read only, ACK receive

    /* Page 4 registers */
    BFE_REG_EEPROM_MISR_DATA    = 0x1004043F, ///< 14-bit MISR EEPROM checksum value - Read only

    /* Page 5 registers */
    BFE_REG_MISR_CALC_CKSM      = 0x10040500, ///< 14-bit shadow reg MISR checksum value - Read only

    /* Special commands */
    BFE_INPUT_RESET             = 0x0E040B3F, ///< Input reset sequence - Write only, broadcast
} e_bfe_reg_addr_t;

/** Battery Front End 'Measure' command target element addresses. */
/* Please, refer to Register Map table from ISL94212 Datasheet! */
typedef enum
{
    BFE_MEAS_VOLT_BATT              = 0x00,   ///< Measure battery voltage address
    BFE_MEAS_VOLT_CELL_1            = 0x01,   ///< Measure cell 1 voltage address
    BFE_MEAS_VOLT_CELL_2            = 0x02,   ///< Measure cell 2 voltage address
    BFE_MEAS_VOLT_CELL_3            = 0x03,   ///< Measure cell 3 voltage address
    BFE_MEAS_VOLT_CELL_4            = 0x04,   ///< Measure cell 4 voltage address
    BFE_MEAS_VOLT_CELL_5            = 0x05,   ///< Measure cell 5 voltage address
    BFE_MEAS_VOLT_CELL_6            = 0x06,   ///< Measure cell 6 voltage address
    BFE_MEAS_VOLT_CELL_7            = 0x07,   ///< Measure cell 7 voltage address
    BFE_MEAS_VOLT_CELL_8            = 0x08,   ///< Measure cell 8 voltage address
    BFE_MEAS_VOLT_CELL_9            = 0x09,   ///< Measure cell 9 voltage address
    BFE_MEAS_VOLT_CELL_10           = 0x0A,   ///< Measure cell 10 voltage address
    BFE_MEAS_VOLT_CELL_11           = 0x0B,   ///< Measure cell 11 voltage address
    BFE_MEAS_VOLT_CELL_12           = 0x0C,   ///< Measure cell 12 voltage address
    BFE_MEAS_TEMP_INT               = 0x10,   ///< Measure internal temperature address
    BFE_MEAS_TEMP_EX_T1             = 0x11,   ///< Measure external temperature input 1 address
    BFE_MEAS_TEMP_EX_T2             = 0x12,   ///< Measure external temperature input 2 address
    BFE_MEAS_TEMP_EX_T3             = 0x13,   ///< Measure external temperature input 3 address
    BFE_MEAS_TEMP_EX_T4             = 0x14,   ///< Measure external temperature input 4 address
    BFE_MEAS_REF_VOLT               = 0x15    ///< Measure reference voltage address
} e_st_bfe_meas_tar_addr_t;

/** Battery Front End configuration */
typedef struct
{
    e_bfe_dev_addr_t    device_address;       ///< Device # in daisy chain to send a command (Low 4-bits)
    uint32_t            reg_address;          ///< Target or response register address (b0-b5), page (b6-b8)
    uint16_t            data;                 ///< 14-bit or 6-bit command or 16-bit response field
} st_bfe_spi_data_t;

/** Battery Front End configuration */
typedef struct
{
    e_bfe_data_dir_t    r_w_data;             ///< Register read or write command
    st_bfe_spi_data_t   command;
    st_bfe_spi_data_t   response[BFE_SPI_RESP_REG_MAX];
    uint32_t            enc_cmnd_length;      ///< Encoded command length
    uint32_t            enc_resp_length;      ///< Actual encoded response length
    uint32_t            exp_resp_length;      ///< Expected encoded response length
    uint8_t             encoded_cmnd[BFE_MAX_CMND_LGTH_B];  ///< Bytes to send to the device
    uint8_t             encoded_resp[BFE_SPI_RESP_DAT_MAX]; ///< Bytes received from device
} st_isl94212_spi_msg_t;

/**********************************************************************************************************************
 External global variables
 *********************************************************************************************************************/
extern const bfe_api_t g_bfe_on_bfe;

/**********************************************************************************************************************
 Exported global functions
 *********************************************************************************************************************/

/***********************************************************************************************************************
 * Public APIs
 **********************************************************************************************************************/

/* Function Name: R_BFE_Open */
/******************************************************************************************************************//**
 * @brief This function initializes a Battery Front End. (Stand-alone/ Daisy chain)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_Open (st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_cfg_t const * const p_bfe_cfg);

/* Function Name: R_BFE_Close */
/******************************************************************************************************************//**
 * @brief This function deinitializes a Battery Front End. (Stand-alone/ Daisy chain)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_Close (st_bfe_ctrl_t * p_bfe_ctrl);

/* Function Name: R_BFE_Reset */
/******************************************************************************************************************//**
 * @brief This function resets a Battery Front End. (Stand-alone/ Daisy chain)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_Reset (st_bfe_ctrl_t * p_bfe_ctrl);

/* Function Name: R_BFE_CommTest */
/******************************************************************************************************************//**
 * @brief This function tests the communication between the microcontroller, master device and other devices
 * in the stack. (Stand-alone/ Daisy chain)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_CommTest (st_bfe_ctrl_t * p_bfe_ctrl);

/* Function Name: R_BFE_Sleep */
/******************************************************************************************************************//**
 * @brief This function puts a Battery Front End to Sleep Mode. (Stand-alone/ Daisy chain)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_Sleep (st_bfe_ctrl_t * p_bfe_ctrl);

/* Function Name: R_BFE_WakeUp */
/******************************************************************************************************************//**
 * @brief This function wakes up a Battery Front End from Sleep Mode. (Stand-alone/ Daisy chain)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_WakeUp (st_bfe_ctrl_t * p_bfe_ctrl);

/* Function Name: R_BFE_RandomWakeUp */
/******************************************************************************************************************//**
 * @brief This function wakes up a sleeping device on random position in the stack. (Daisy chain only)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_RandomWakeUp (st_bfe_ctrl_t * p_bfe_ctrl);

/* Function Name: R_BFE_VAllGet */
/******************************************************************************************************************//**
 * @brief This function performs wire scan, all battery, cells, internal temperature and ExTn inputs voltage
 * scan and read. (Stand-alone/ Daisy chain)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_VoltTempAllGet (st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_meas_t * p_meas_data);

/* Function Name: R_BFE_VAllGet */
/******************************************************************************************************************//**
 * @brief This function performs battery and all cells voltage scan and read. (Stand-alone/ Daisy chain)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_VAllGet (st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_meas_t * p_meas_data);

/* Function Name: R_BFE_VBattGet */
/******************************************************************************************************************//**
 * @brief This function performs battery voltage scan and read. (Stand-alone/ Daisy chain)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_VBattGet (st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_meas_t * p_meas_data);

/* Function Name: R_BFE_VMixGet */
/******************************************************************************************************************//**
 * @brief This function performs battery, all cells, internal temperature and ExT1 voltage scan and read.
 * (Stand-alone/ Daisy chain)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_VMixGet (st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_meas_t * p_meas_data);

/* Function Name: R_BFE_TempsGet */
/******************************************************************************************************************//**
 * @brief This function performs internal temperature and ExTn voltages scan and read. (Stand-alone/ Daisy chain)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_TempsGet (st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_meas_t * p_meas_data);

/* Function Name: R_BFE_FaultsAllRead */
/******************************************************************************************************************//**
 * @brief This function reads the fault registers. (Stand-alone/ Daisy chain)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_FaultsAllRead (st_bfe_ctrl_t * p_bfe_ctrl, st_bfe_faults_t * p_faults);

/* Function Name: R_BFE_FaultsCheck */
/******************************************************************************************************************//**
 * @brief This function checks fault pin, reads fault registers and calls user callback. (Stand-alone/ Daisy chain)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_FaultsCheck (st_bfe_ctrl_t * p_bfe_ctrl);

/* Function Name: R_BFE_FaultsAllClr */
/******************************************************************************************************************//**
 * @brief This function clears all faults. (Stand-alone/ Daisy chain)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_FaultsAllClr (st_bfe_ctrl_t * p_bfe_ctrl);

/* Function Name: R_BFE_BalanceCtrl */
/******************************************************************************************************************//**
 * @brief This function selects cells and enables or inhibits cell balancing. (Stand-alone/ Daisy chain)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_BalanceCtrl (st_bfe_ctrl_t * p_bfe_ctrl, e_bfe_ctrl_bal_t bal_ctrl);

/* Function Name: R_BFE_ChecksumChk */
/******************************************************************************************************************//**
 * @brief This function checks the configuration registers for corruption. (Stand-alone/ Daisy chain)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_ConfRegCheck (st_bfe_ctrl_t * p_bfe_ctrl);

/* Function Name: R_BFE_DeviceSetup */
/******************************************************************************************************************//**
 * @brief This function writes and verifies setup registers. (Stand-alone/ Daisy chain)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_DeviceSetup (st_bfe_ctrl_t * p_bfe_ctrl);

/* Function Name: R_BFE_WatchdogCtrl */
/******************************************************************************************************************//**
 * @brief This function enables or disables the watchdog timer and selects time. (Daisy chain only)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_WatchdogCtrl (st_bfe_ctrl_t * p_bfe_ctrl);

/* Function Name: R_BFE_WatchdogReset */
/******************************************************************************************************************//**
 * @brief This function resets the watchdog timer. (Daisy chain only)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_WatchdogReset (st_bfe_ctrl_t * p_bfe_ctrl);

/* Function Name: R_BFE_Scan */
/******************************************************************************************************************//**
 * @brief This function sends scan command. (Stand-alone/ Daisy chain)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_Scan (st_bfe_ctrl_t * p_bfe_ctrl, e_bfe_scan_cmnd_types_t cmnd_type);

/* Function Name: R_BFE_UserRegAccess */
/******************************************************************************************************************//**
 * @brief This function provides access to the user registers. (Stand-alone/ Daisy chain)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_UserRegsisterAccess (st_bfe_ctrl_t *        p_bfe_ctrl,
                                     uint16_t *               p_data_array,
                                     e_bfe_data_dir_t         data_dir);

/* Function Name: R_BFE_SingleRegisterAccess */
/******************************************************************************************************************//**
 * @brief This function sends a command to a single register of a device. (Stand-alone/ Daisy chain)
 *********************************************************************************************************************/
e_bms_err_t R_BFE_SingleRegisterAccess (st_bfe_ctrl_t *       p_bfe_ctrl,
                                      uint16_t *              p_dev_addr,
                                      uint32_t *              p_reg_addr,
                                      uint16_t *              p_reg_data,
                                      e_bfe_data_dir_t        r_w_data);

#endif /* BFE_R_BFE_ISL94212_H_ */

/*******************************************************************************************************************//**
 * @} (end defgroup ISL94212)
 **********************************************************************************************************************/
