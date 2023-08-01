#ifndef R_BFE_COMMON_H_
#define R_BFE_COMMON_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>


#include "r_bfe_cfg.h"
#include "hal_data.h"

#define ONE  (1U)
#define ZERO (0U)

#define BFE_ERROR_RETURN(a, err)                        \
    {                                                   \
        if ((a))                                        \
        {                                               \
            (void) 0;                  /* Do nothing */ \
        }                                               \
        else                                            \
        {                                               \
            return(err);                                 \
        }                                               \
    }

#define RESET_VALUE (0x00)

 Exported global functions
 *********************************************************************************************************************/

#endif /* R_BFE_COMMON_H_ */
