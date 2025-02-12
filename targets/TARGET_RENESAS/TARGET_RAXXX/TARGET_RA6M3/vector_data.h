#ifndef VECTOR_DATA_H
#define VECTOR_DATA_H
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

/* none */

/* -------------------------------------------------------------------------+
 |   MACROS                                                                 |
 + -------------------------------------------------------------------------*/

        /* Number of interrupts allocated */
#ifndef VECTOR_DATA_IRQ_COUNT
#define VECTOR_DATA_IRQ_COUNT    (4)
#endif

        /* Vector table allocations */
#define VECTOR_NUMBER_SCI9_RXI ((IRQn_Type) 0) /* SCI9 RXI (Received data full) */
#define SCI9_RXI_IRQn          ((IRQn_Type) 0) /* SCI9 RXI (Received data full) */
#define VECTOR_NUMBER_SCI9_TXI ((IRQn_Type) 1) /* SCI9 TXI (Transmit data empty) */
#define SCI9_TXI_IRQn          ((IRQn_Type) 1) /* SCI9 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI9_TEI ((IRQn_Type) 2) /* SCI9 TEI (Transmit end) */
#define SCI9_TEI_IRQn          ((IRQn_Type) 2) /* SCI9 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI9_ERI ((IRQn_Type) 3) /* SCI9 ERI (Receive error) */
#define SCI9_ERI_IRQn          ((IRQn_Type) 3) /* SCI9 ERI (Receive error) */

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
                                
        /* ISR prototypes */
void sci_uart_rxi_isr(void);
void sci_uart_txi_isr(void);
void sci_uart_tei_isr(void);
void sci_uart_eri_isr(void);


#ifdef __cplusplus
}
#endif
/* -------------------------------------------------------------------------+
 |   END                                                                    |
 + -------------------------------------------------------------------------*/

#endif /* VECTOR_DATA_H */