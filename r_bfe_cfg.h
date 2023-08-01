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
 * @file          r_bfe_cfg.h
 * @Version       1.0
 * @brief         Contains configuration settings (macros) for the battery front end related functions.
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 16.08.2021 1.00     First Release
 *         : 12.11.2021 1.01     Configuration parameters are edited.
 *********************************************************************************************************************/

#ifndef R_BFE_CFG_H_
#define R_BFE_CFG_H_

/**********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 *********************************************************************************************************************/
/* Generic headers */
#include <stdint.h>
#include <stdbool.h>

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/

/*
 *   |             Pin Connections             |
 *   |-----------------------------------------|
 *   | Signal    | EK-RA2A1  | ISL94212EVKIT1Z |
 *   |-----------+-----------+-----------------|
 *   | SCK       | J1-RSPCKB | J2–1            |
 *   | MISO      | J1-MISOB  | J2-2            |
 *   | MOSI      | J1-MOSIB  | J2-3            |
 *   | CS        | J1-SSLB0  | J2-4            |
 *   | DRDY      | J1-RXD9   | J2-5            |
 *   | FAULT     | J1-SCK9   | J2-6            |
 *   | DGND      | J1-VSS    | J2-7            |
 *   | EN (opt.) | J1-CTS9   | EN              |
 *   |-----------------------------------------|
 */

/** Number of devices in a stack (14 max). Please, refer to ISL94212 Datasheet! */
#define BFE_CFG_STA_DEV             (2U)

/** BFE enable pin is '1' - connected / '0' - Not connected (Recommended). */
#define BFE_CFG_USE_ENBL_PIN        (0)

/** BFE go to sleep using watchdog timeout '1' - enabled/ '0' - disabled (Recommended). */
#define BFE_CFG_WDT_SLEEP_EN        (0)

/** '1' - Enable (Recommended)/ '0' - Disable input parameters checking of functions. */
#define BFE_CFG_PARAM_CHECKING_EN   (1)

/** '1' - Enable (Recommended)/ '0' - Disable register verification after write command. */
#define BFE_CFG_REG_WRITE_VERIFY_EN (1)

/** '1' - Enable (Recommended)/ '0' - Disable scan command verification. */
#define BFE_CFG_SCAN_CMND_VERIFY_EN (1)

/** Maximal attempts for stack identification. */
#define BFE_CFG_STACK_IDENT_MAX     (3U)

/** Data Ready pin. */
#define BFE_DAT_RDY_PIN             (BSP_IO_PORT_01_PIN_01)

/** Fault pin. */
#define BFE_FAULT_PIN               (BSP_IO_PORT_02_PIN_04)

/** Enable pin. */
#define BFE_ENBL_PIN                (BSP_IO_PORT_04_PIN_02)

/** Fault limits. */
#define BFE_CFG_OVP_LIMIT           (0x1CCC)    ///< Overvoltage limit (0 = 0V; 8191 = 5V) Set to 4.50 V
#define BFE_CFG_UVP_LIMIT           (0x0999)    ///< Overvoltage limit (0 = 0V; 8191 = 5V) Set to 2.00 V

#define BFE_CFG_ETP_LIMIT           (0x3FFF)    ///< External temperature limit (0 = 0V; 16383 = 2.5V) Set to 2.50 V

/**********************************************************************************************************************
 Global Typedef definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 External global variables
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Exported global functions
 *********************************************************************************************************************/

#endif /* R_BFE_CFG_H_ */
