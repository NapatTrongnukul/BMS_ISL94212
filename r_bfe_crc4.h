
#ifndef BFE_R_BFE_CRC4_H_
#define BFE_R_BFE_CRC4_H_

#include <stdint.h>
#include <stdbool.h>

#include "r_bfe_common.h"
#include "r_bfe_api.h"

e_bms_err_t crc4_check (uint8_t * p_data_array, uint32_t const length, uint32_t const offset);

e_bms_err_t crc4_add (uint8_t * p_data_array, uint32_t const length);

#endif /* BFE_R_BFE_CRC4_H_ */
