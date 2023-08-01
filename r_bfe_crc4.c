#include <stdint.h>
#include <stdbool.h>


#include <bfe/r_bfe_crc4.h>

static e_bms_err_t crc4_calculate (uint8_t *      p_data_array,
                                 uint32_t const length,
                                 uint32_t const offset,
                                 uint8_t *      p_crc4_res);


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

    for(uint32_t i = 0; i < length; i++ )
    {
        data_array_cpy[i] = p_data_array[offset + i];
    }

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
