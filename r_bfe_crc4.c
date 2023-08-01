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
 * @file          r_crc4.c
 * @version       1.00
 * @brief         Contains functions and routines for CRC4 generation and check
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 16.08.2021 1.00     First Release
 *         : 12.11.2021 1.01     Errors in comments are fixed.
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 *********************************************************************************************************************/
/* Generic headers */
#include <stdint.h>
#include <stdbool.h>

/* bfe related headers */
#include <bfe/r_bfe_crc4.h>

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Local Typedef definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Exported global variables
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Private (static) variables and functions
 *********************************************************************************************************************/
static e_bms_err_t crc4_calculate (uint8_t *      p_data_array,
                                 uint32_t const length,
                                 uint32_t const offset,
                                 uint8_t *      p_crc4_res);

/* Function Name: crc4_check */
/******************************************************************************************************************//**
 * This function checks the cheksum in the input data array.
 *
 * This function performs the following tasks:
 * - Parameter checking and processes error conditions.
 * - Calculates the CRC4 of the input data array.
 * - Compares the obtained CRC4 with the last nibble of the input array.
 *
 * @warning     The data array could have any length but more than one byte!
 * @param[in]   p_data_array                A pointer to the data array to be checked.
 * @param[in]   length                      The length (in bytes) of p_data_array.
 * @param[in]   offset                      The data offset (in bytes) of p_data_array.
 * @retval      BFE_SUCCESS                 The CRC4 is correct.
 * @retval      BMS_ERR_CRC_INCORRECT       The CRC in the data array is incorrect.
 * @retval      BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval      BMS_ERR_INVALID_ARGUMENT    Input argument is invalid.
 * @retval      BMS_ERR_BUFFER_SIZE         The data array is not big enough.
 * @retval      BMS_ERR_...                 Inherit from crc4_calculate().
 *********************************************************************************************************************/
e_bms_err_t crc4_check(uint8_t * p_data_array, uint32_t const length, uint32_t const offset)
{
    e_bms_err_t err = BMS_SUCCESS;    // Error status

    uint8_t last_nibble = 0;
    uint8_t crc4 = 0;

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_data_array, BMS_ERR_INVALID_POINTER);

    /* Verify the array has non-zero length. */
    BFE_ERROR_RETURN(1 < length, BMS_ERR_INVALID_ARGUMENT);

#endif // BFE_CFG_PARAM_CHECKING_EN

    /* Extract the last nibble from the data array.
     * Casting is added to ensure data size after integer promotion. */
    last_nibble = (uint8_t)(p_data_array[offset + length - 1] & 0x0F);

    err = crc4_calculate(p_data_array, length, offset, &crc4); // Calculate checksum of input data array.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.
    BFE_ERROR_RETURN(last_nibble == crc4, BMS_ERR_CRC_INCORRECT); // Compare the checksum.

    return err;
}
/**********************************************************************************************************************
 * End of function crc4_check
 *********************************************************************************************************************/

/* Function Name: crc4_add */
/******************************************************************************************************************//**
 * This function adds CRC4 to the last nibble of the input data array.
 *
 * This function performs the following tasks:
 * - Parameter checking and processes error conditions.
 * - Calculates the CRC4 of the input data array.
 * - Add the obtained CRC4 to the last nibble of the input array.
 *
 * @warning     The data array could have any length but more than one byte!
 * @param[in]   length                      The length (in bytes) of p_data_array.
 * @param[out]  p_data_array                A pointer to the data array to which the checksum will be added.
 * @retval      BFE_SUCCESS                 The CRC4 is successfully added.
 * @retval      BMS_ERR_INVALID_POINTER     Input argument is invalid.
 * @retval      BMS_ERR_BUFFER_SIZE         The data array is not big enough.
 * @retval      BMS_ERR_...                 Inherit from crc4_calculate().
 *********************************************************************************************************************/
e_bms_err_t crc4_add(uint8_t * p_data_array, uint32_t const length)
{
    e_bms_err_t err = BMS_SUCCESS;    // Error status

    uint8_t crc4 = 0;

#if BFE_CFG_PARAM_CHECKING_EN // Perform parameter checking.

    /* Verify the pointer is valid. */
    BFE_ERROR_RETURN(NULL != p_data_array, BMS_ERR_INVALID_POINTER);

    /* Verify the array has non-zero length. */
    BFE_ERROR_RETURN(1 < length, BMS_ERR_BUFFER_SIZE);

#endif // BFE_CFG_PARAM_CHECKING_EN

    err = crc4_calculate(p_data_array, length, 0, &crc4); // Calculate checksum of input data array.
    BFE_ERROR_RETURN(BMS_SUCCESS == err, err); // Check for errors.

    /* Copy the CRC4 to the last nibble of the data array.
     * Casting is added to ensure data size after integer promotion. */
    p_data_array[length - 1] = (uint8_t)((p_data_array[length - 1] & 0xF0) | crc4);

    return err;
}
/**********************************************************************************************************************
 * End of function crc4_add
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Function Name: crc4_calculate
 * Description  : This function calculates CRC4 of the input data array using polynomial X^4 + X + 1. The
 * last nibble of the input array is not included into the calculation.
 * Arguments    : p_data_array      A pointer to the data array for CRC calculation;
 *                length            The length (in bytes) of p_data_array;
 *                offset            The data offset (in bytes) of p_data_array;
 *                p_crc4_res        The resultant from the CRC4 calculation.
 * Return Value : BFE_SUCCESS       The CRC4 is successfully calculated.
 *********************************************************************************************************************/
static e_bms_err_t crc4_calculate (uint8_t *      p_data_array,
                                   uint32_t const length,
                                   uint32_t const offset,
                                   uint8_t *      p_crc4_res)
{
    uint8_t  data_array_cpy[length];
    uint32_t k;
    bool     bit0 = false;
    bool     bit1 = false;
    bool     bit2 = false;
    bool     bit3 = false;

    bool     ff0;
    bool     ff1;
    bool     ff2;
    bool     ff3;

    bool     carry;

    /* Copy source array so that its content is not compromised. */
    for(uint32_t i = 0; i < length; i++ )
    {
        data_array_cpy[i] = p_data_array[offset + i];
    }

    /* A simple implementation of CRC4 (using polynomial X^4 + X + 1). */
    for(uint32_t i = 0; i < length; i++ )
    {
        /* The last nibble is ignored for CRC4 calculations. */
        if(i == (length - 1))
        {
            k = 4;
        }
        else
        {
            k = 8;
        }

        for(uint32_t j = 0; j < k; j++ )
        {
            /* Shift left one bit. Casting is added to ensure data size after integer promotion. */
            if((data_array_cpy[i] & 0x80) > 0)
            {
                carry = true;
            }
            else
            {
                carry = false;
            }

            data_array_cpy[i] = (uint8_t) (data_array_cpy[i] << 1);

            /* Calculate bits according to polynomial. */
            ff0 = carry ^ bit3;
            ff1 = bit0 ^ bit3;
            ff2 = bit1;
            ff3 = bit2;
            bit0 = ff0;
            bit1 = ff1;
            bit2 = ff2;
            bit3 = ff3;
        }
    }

    /* Combine bits to obtain CRC4 result. */
    *p_crc4_res = 0x00;

    if(bit0)
    {
        *p_crc4_res = (uint8_t) ((*p_crc4_res) + 0x01);  // Casting ensures data size after integer promotion.
    }
    else
    {
        ;
    }

    if(bit1)
    {
        *p_crc4_res = (uint8_t) ((*p_crc4_res) + 0x02);  // Casting ensures data size after integer promotion.
    }
    else
    {
        ;
    }

    if(bit2)
    {
        *p_crc4_res = (uint8_t) ((*p_crc4_res) + 0x04);  // Casting ensures data size after integer promotion.
    }
    else
    {
        ;
    }

    if(bit3)
    {
        *p_crc4_res = (uint8_t) ((*p_crc4_res) + 0x08);  // Casting ensures data size after integer promotion.
    }
    else
    {
        ;
    }

    return BMS_SUCCESS;
}
/**********************************************************************************************************************
 * End of function crc4_calculate
 *********************************************************************************************************************/
