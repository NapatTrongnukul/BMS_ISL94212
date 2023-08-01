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
 * @file          bal_data.h
 * @Version       1.0
 * @brief         Contains global structures exports, defined in bal_data.c
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 16.08.2021 1.00     First Release
 *         : 12.11.2021 1.01     Data structures are modified.
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 *********************************************************************************************************************/
/* Generic headers */
#include <stdint.h>
#include <stdbool.h>

/* BFE related headers */
#include "bfe/r_bfe_isl94212.h"
#include "bfe/r_bfe_api.h"
#include "bfe/r_bfe_cfg.h"
#include "bfe/r_bfe_common.h"

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/
#ifndef BAL_DATA_H_
#define BAL_DATA_H_

/**********************************************************************************************************************
 Global Typedef definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 External global variables
 *********************************************************************************************************************/

/** Access the BFE instance using these structures when calling API functions directly (::p_api is not used). */
extern          st_bfe_ctrl_t   g_bfe0_ctrl;
extern const    st_bfe_cfg_t    g_bfe0_cfg;

/** BFE on BFE Instance. */
extern const    bfe_instance_t  g_bfe0;

/**********************************************************************************************************************
 Exported global functions
 *********************************************************************************************************************/

#endif /* BAL_DATA_H_ */
