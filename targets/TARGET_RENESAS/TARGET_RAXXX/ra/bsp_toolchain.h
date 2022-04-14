#ifndef __TOOLCHAIN_H__
#define __TOOLCHAIN_H__
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

/* -------------------------------------------------------------------------+
 |   HEADER (INCLUDE) SECTION                                               |
 + -------------------------------------------------------------------------*/

#include "mbed_toolchain.h"

 #ifdef __cplusplus
extern "C" {
 #endif
/* -------------------------------------------------------------------------+
 |   MACROS                                                                 |
 + -------------------------------------------------------------------------*/

 #if defined(__ARMCC_VERSION)          /* AC6 compiler */

/* The AC6 linker requires uninitialized code to be placed in a section that starts with ".bss." Without this, load
 * memory (ROM) is reserved unnecessarily. */
  #define UNINIT_SECTION_PREFIX         ".bss"
  #define BSP_DONT_REMOVE
  #define __NAKED                      __attribute__((naked))
 #elif   defined(__GNUC__)             /* GCC compiler */
  #define UNINIT_SECTION_PREFIX
  #define BSP_DONT_REMOVE
  #define __NAKED                      __attribute__((naked))
 #elif defined(__ICCARM__)             /* IAR compiler */
  #define UNINIT_SECTION_PREFIX
  #define BSP_DONT_REMOVE              __root
  #define __NAKED                      __stackless
 #endif

#define BSP_SECTION_NOINIT         UNINIT_SECTION_PREFIX ".noinit"
#define BSP_SECTION_APPLICATION_VECTORS    ".application_vectors"
 #define BSP_SECTION_ROM_REGISTERS          ".rom_registers"
#define BSP_SECTION_ID_CODE                ".id_code"


#define BSP_PLACE_IN_SECTION(x)    __attribute__((section(x))) __attribute__((__used__))

/* -------------------------------------------------------------------------+
 |   TYPE DEFINITIONS                                                       |
 + -------------------------------------------------------------------------*/

/* none */

/* -------------------------------------------------------------------------+
 |   EXTERNAL DATA (+ meaning)                                              |
 + -------------------------------------------------------------------------*/

/* none */

/* -------------------------------------------------------------------------+
 |   GLOBAL FUNCTION PROTOTYPES                                             |
 + -------------------------------------------------------------------------*/
                                
/* none */

/* -------------------------------------------------------------------------+
 |   END                                                                    |
 + -------------------------------------------------------------------------*/
 #ifdef __cplusplus
}
 #endif

#endif /* __TOOLCHAIN_H__ */
