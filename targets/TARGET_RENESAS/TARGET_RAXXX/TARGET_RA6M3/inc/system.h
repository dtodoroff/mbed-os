#ifndef SYSTEM_RENESAS_ARM_H
#define SYSTEM_RENESAS_ARM_H
/* -------------------------------------------------------------------------+
 |                               DiNiTo Ltd.                                |
 + -------------------------------------------------------------------------*/

/* --------------------------- COPYRIGHT INFORMATION -----------------------+
 |                                                                          |
 + -------------------------------------------------------------------------*/

/* -------------------------------------------------------------------------+
 | Module        : <mandatory>
 | File name     : <mandatory>
 | Description   : <mandatory>
 | Reference(s)  : <mandatory>
 + -------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------+
 |   HEADER (INCLUDE) SECTION                                               |
 + -------------------------------------------------------------------------*/

 #include <stdint.h>

/* -------------------------------------------------------------------------+
 |   MACROS                                                                 |
 + -------------------------------------------------------------------------*/

/* none*/

/* -------------------------------------------------------------------------+
 |   TYPE DEFINITIONS                                                       |
 + -------------------------------------------------------------------------*/

/* none*/

/* -------------------------------------------------------------------------+
 |   EXTERNAL DATA (+ meaning)                                              |
 + -------------------------------------------------------------------------*/

extern uint32_t SystemCoreClock;       /** System Clock Frequency (Core Clock)  */

/* -------------------------------------------------------------------------+
 |   GLOBAL FUNCTION PROTOTYPES                                             |
 + -------------------------------------------------------------------------*/
                                
/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemCoreClock variable.
 */
void SystemInit(void);

/**
 * Update SystemCoreClock variable
 *
 * @param  none
 * @return none
 *
 * @brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from cpu registers.
 */
void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif
/* -------------------------------------------------------------------------+
 |   END                                                                    |
 + -------------------------------------------------------------------------*/

#endif /* SYSTEM_RENESAS_ARM_H */
