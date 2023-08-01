
#ifndef R_BFE_CFG_H_
#define R_BFE_CFG_H_

#include <stdint.h>
#include <stdbool.h>

#define BFE_CFG_STA_DEV             (2U)


#define BFE_CFG_USE_ENBL_PIN        (0)

#define BFE_CFG_WDT_SLEEP_EN        (0)

#define BFE_CFG_PARAM_CHECKING_EN   (1)

#define BFE_CFG_REG_WRITE_VERIFY_EN (1)

#define BFE_CFG_SCAN_CMND_VERIFY_EN (1)

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

#endif /* R_BFE_CFG_H_ */
