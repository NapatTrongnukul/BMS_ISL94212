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
 * @file          r_crc4.h
 * @Version       1.0
 * @brief         Contains declarations of functions used in r_crc4.c.
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 16.08.2021 1.00     First Release
 *         : 12.11.2021 1.01     Errors in comments are fixed.
 *********************************************************************************************************************/

#ifndef BFE_R_BFE_CRC4_H_
#define BFE_R_BFE_CRC4_H_

/**********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 *********************************************************************************************************************/
/* Generic headers */
#include <stdint.h>
#include <stdbool.h>

/* bfe related headers */
#include "r_bfe_common.h"
#include "r_bfe_api.h"

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Global Typedef definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 External global variables
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Exported global functions
 *********************************************************************************************************************/

/* Function Name: crc4_check */
/******************************************************************************************************************//**
 * @brief This function calculates the CRC4 of the input data array and compares it with the last nibble.
 *********************************************************************************************************************/
e_bms_err_t crc4_check (uint8_t * p_data_array, uint32_t const length, uint32_t const offset);

/* Function Name: crc4_add */
/******************************************************************************************************************//**
 * @brief This function calculates the CRC4 of the input data array and adds it to the last nibble.
 *********************************************************************************************************************/
e_bms_err_t crc4_add (uint8_t * p_data_array, uint32_t const length);

#endif /* BFE_R_BFE_CRC4_H_ */
