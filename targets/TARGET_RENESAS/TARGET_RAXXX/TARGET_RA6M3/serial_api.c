/* -------------------------------------------------------------------------+
 |                                                                          |
 + -------------------------------------------------------------------------*/

/* --------------------------- COPYRIGHT INFORMATION -----------------------+
 |                                                                          |
 |                                                                          |
 |                                                                          |
 |                                                                          |
 + -------------------------------------------------------------------------*/

/* -------------------------------------------------------------------------+
 | Module        : <mandatory>
 | File name     : <mandatory>
 | Description   : <mandatory>
 | Reference(s)  : <mandatory>
 + -------------------------------------------------------------------------*/

#define  __SERIAL_API_C__ 0x0100

/* -------------------------------------------------------------------------+
 |   HEADER (INCLUDE) SECTION                                               |
 + -------------------------------------------------------------------------*/

// math.h required for floating point operations for baud rate calculation
#include "mbed_assert.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "serial_api.h"
#include "cmsis.h"
#include "PeripheralPins.h"
#include "gpio_api.h"


#include "bsp_api.h"
#include "mbed_drv_cfg.h"
#include "mbed_critical.h"


/* -------------------------------------------------------------------------+
 |   EXTERNAL DATA (+ meaning)                                              |
 + -------------------------------------------------------------------------*/

/* none*/

/* -------------------------------------------------------------------------+
 |   MACROS                                                                 |
 + -------------------------------------------------------------------------*/


#define UART_NUM    10
#define IRQ_NUM     4
#define R_SCI_REG_SIZE                            (R_SCI1_BASE - R_SCI0_BASE)

/* Number of divisors in the data table used for baud rate calculation. */
#define SCI_UART_NUM_DIVISORS_ASYNC             (13U)

/* Valid range of values for the modulation duty register is 128 - 256 (256 = modulation disabled). */
#define SCI_UART_MDDR_MIN                       (128U)
#define SCI_UART_MDDR_MAX                       (256U)

/* The bit rate register is 8-bits, so the maximum value is 255. */
#define SCI_UART_BRR_MAX                        (255U)

/* -------------------------------------------------------------------------+
 |   TYPE DEFINITIONS                                                       |
 + -------------------------------------------------------------------------*/

typedef void (*IRQHandler)(void);

typedef struct st_baud_setting_const_t
{
    uint8_t bgdm  : 1;                 /**< BGDM value to get divisor */
    uint8_t abcs  : 1;                 /**< ABCS value to get divisor */
    uint8_t abcse : 1;                 /**< ABCSE value to get divisor */
    uint8_t cks   : 2;                 /**< CKS  value to get divisor (CKS = N) */
} baud_setting_const_t;

/** Register settings to acheive a desired baud rate and modulation duty. */
typedef struct st_baud_setting_t
{
    union
    {
        uint8_t semr_baudrate_bits;

        /* DEPRECATED: Anonymous structure. */
        struct
        {
            uint8_t       : 2;
            uint8_t brme  : 1;         ///< Bit Rate Modulation Enable
            uint8_t abcse : 1;         ///< Asynchronous Mode Extended Base Clock Select 1
            uint8_t abcs  : 1;         ///< Asynchronous Mode Base Clock Select
            uint8_t       : 1;
            uint8_t bgdm  : 1;         ///< Baud Rate Generator Double-Speed Mode Select
            uint8_t       : 1;
        };

        struct
        {
            uint8_t       : 2;
            uint8_t brme  : 1;         ///< Bit Rate Modulation Enable
            uint8_t abcse : 1;         ///< Asynchronous Mode Extended Base Clock Select 1
            uint8_t abcs  : 1;         ///< Asynchronous Mode Base Clock Select
            uint8_t       : 1;
            uint8_t bgdm  : 1;         ///< Baud Rate Generator Double-Speed Mode Select
            uint8_t       : 1;
        } semr_baudrate_bits_b;
    };
    uint8_t cks : 2;                   ///< CKS  value to get divisor (CKS = N)
    uint8_t brr;                       ///< Bit Rate Register setting
    uint8_t mddr;                      ///< Modulation Duty Register setting
} baud_setting_t;
 
/* -------------------------------------------------------------------------+
 |   GLOBAL CONSTANTS                                                       |
 + -------------------------------------------------------------------------*/

/* none*/

/* -------------------------------------------------------------------------+
 |   GLOBAL VARIABLES                                                       |
 + -------------------------------------------------------------------------*/

static uart_irq_handler irq_handler;

int stdio_uart_inited = 0;
serial_t stdio_uart;

struct serial_global_data_s {
    uint32_t serial_irq_id;
    gpio_t sw_rts, sw_cts;
    serial_t *tx_obj, *rx_obj;
    uint32_t async_tx_callback, async_rx_callback;
    int event, wanted_rx_events;
};

/* -------------------------------------------------------------------------+
 |   LOCAL FUNCTION PROTOTYPES                                              |
 + -------------------------------------------------------------------------*/

static void uart0_rxi_isr(void);
static void uart1_rxi_isr(void);
static void uart2_rxi_isr(void);
static void uart3_rxi_isr(void);
static void uart4_rxi_isr(void);
static void uart5_rxi_isr(void);
static void uart6_rxi_isr(void);
static void uart7_rxi_isr(void);
static void uart8_rxi_isr(void);
static void uart9_rxi_isr(void);
                               ;
static void uart0_txi_isr(void);
static void uart1_txi_isr(void);
static void uart2_txi_isr(void);
static void uart3_txi_isr(void);
static void uart4_txi_isr(void);
static void uart5_txi_isr(void);
static void uart6_txi_isr(void);
static void uart7_txi_isr(void);
static void uart8_txi_isr(void);
static void uart9_txi_isr(void);

static void uart0_tei_isr(void);
static void uart1_tei_isr(void);
static void uart2_tei_isr(void);
static void uart3_tei_isr(void);
static void uart4_tei_isr(void);
static void uart5_tei_isr(void);
static void uart6_tei_isr(void);
static void uart7_tei_isr(void);
static void uart8_tei_isr(void);
static void uart9_tei_isr(void);

static void uart0_eri_isr(void);
static void uart1_eri_isr(void);
static void uart2_eri_isr(void);
static void uart3_eri_isr(void);
static void uart4_eri_isr(void);
static void uart5_eri_isr(void);
static void uart6_eri_isr(void);
static void uart7_eri_isr(void);
static void uart8_eri_isr(void);
static void uart9_eri_isr(void);


static void serial_put_done(serial_t *obj);
static uint8_t serial_available_buffer(serial_t *obj);
static void serial_irq_err_set(serial_t *obj, uint32_t enable);

/* -------------------------------------------------------------------------+
 |   LOCAL  VARIABLES                                                       |
 + -------------------------------------------------------------------------*/

static struct serial_global_data_s uart_data[UART_NUM];

/* Baud rate divisor information (UART mode) */
static const baud_setting_const_t g_async_baud[SCI_UART_NUM_DIVISORS_ASYNC] =
{
    {0U, 0U, 1U, 0U},                  /* BGDM, ABCS, ABCSE, n */
    {1U, 1U, 0U, 0U},
    {1U, 0U, 0U, 0U},
    {0U, 0U, 1U, 1U},
    {0U, 0U, 0U, 0U},
    {1U, 0U, 0U, 1U},
    {0U, 0U, 1U, 2U},
    {0U, 0U, 0U, 1U},
    {1U, 0U, 0U, 2U},
    {0U, 0U, 1U, 3U},
    {0U, 0U, 0U, 2U},
    {1U, 0U, 0U, 3U},
    {0U, 0U, 0U, 3U}
};

static const uint16_t g_div_coefficient[SCI_UART_NUM_DIVISORS_ASYNC] =
{
    6U,
    8U,
    16U,
    24U,
    32U,
    64U,
    96U,
    128U,
    256U,
    384U,
    512U,
    1024U,
    2048U,
};


static const int32_t SCI_UART_100_PERCENT_X_1000 = 100000;
static const int32_t SCI_UART_MDDR_DIVISOR       = 256;

#if (SCI_UART_CFG_PARAM_CHECKING_ENABLE)
static const uint32_t SCI_UART_MAX_BAUD_RATE_ERROR_X_1000 = 15000;
#endif


static const elc_event_t irq_set_tbl[UART_NUM][IRQ_NUM] = {
    {ELC_EVENT_SCI0_RXI, ELC_EVENT_SCI0_TXI, ELC_EVENT_SCI0_TEI, ELC_EVENT_SCI0_ERI},
    {ELC_EVENT_SCI1_RXI, ELC_EVENT_SCI1_TXI, ELC_EVENT_SCI1_TEI, ELC_EVENT_SCI1_ERI},
    {ELC_EVENT_SCI2_RXI, ELC_EVENT_SCI2_TXI, ELC_EVENT_SCI2_TEI, ELC_EVENT_SCI2_ERI},
    {ELC_EVENT_SCI3_RXI, ELC_EVENT_SCI3_TXI, ELC_EVENT_SCI3_TEI, ELC_EVENT_SCI3_ERI},
    {ELC_EVENT_SCI4_RXI, ELC_EVENT_SCI4_TXI, ELC_EVENT_SCI4_TEI, ELC_EVENT_SCI4_ERI},
    {ELC_EVENT_SCI5_RXI, ELC_EVENT_SCI5_TXI, ELC_EVENT_SCI5_TEI, ELC_EVENT_SCI5_ERI},
    {ELC_EVENT_SCI6_RXI, ELC_EVENT_SCI6_TXI, ELC_EVENT_SCI6_TEI, ELC_EVENT_SCI6_ERI},
    {ELC_EVENT_SCI7_RXI, ELC_EVENT_SCI7_TXI, ELC_EVENT_SCI7_TEI, ELC_EVENT_SCI7_ERI},
    {ELC_EVENT_SCI8_RXI, ELC_EVENT_SCI8_TXI, ELC_EVENT_SCI8_TEI, ELC_EVENT_SCI8_ERI},
    {ELC_EVENT_SCI9_RXI, ELC_EVENT_SCI9_TXI, ELC_EVENT_SCI9_TEI, ELC_EVENT_SCI9_ERI}
};

static const IRQHandler hander_set_tbl[UART_NUM][IRQ_NUM] = {
    {uart0_rxi_isr, uart0_txi_isr, uart0_tei_isr, uart0_eri_isr},
    {uart1_rxi_isr, uart1_txi_isr, uart1_tei_isr, uart1_eri_isr},
    {uart2_rxi_isr, uart2_txi_isr, uart2_tei_isr, uart2_eri_isr},
    {uart3_rxi_isr, uart3_txi_isr, uart3_tei_isr, uart3_eri_isr},
    {uart4_rxi_isr, uart4_txi_isr, uart4_tei_isr, uart4_eri_isr},
    {uart5_rxi_isr, uart5_txi_isr, uart5_tei_isr, uart5_eri_isr},
    {uart6_rxi_isr, uart6_txi_isr, uart6_tei_isr, uart6_eri_isr},
    {uart7_rxi_isr, uart7_txi_isr, uart7_tei_isr, uart7_eri_isr},
    {uart8_rxi_isr, uart8_txi_isr, uart8_tei_isr, uart8_eri_isr},
    {uart9_rxi_isr, uart9_txi_isr, uart9_tei_isr, uart9_eri_isr}
};

/* -------------------------------------------------------------------------+
 |   LOCAL FUNCTIONS                                                        |                                         
 + -------------------------------------------------------------------------*/

/******************************************************************************
 * INTERRUPTS HANDLING
 ******************************************************************************/

static void uart_tx_irq(IRQn_Type irq_num, uint32_t index) {
    __IO uint16_t *dmy_rd_scscr;
    __IO uint16_t *dmy_rd_scfsr;
    serial_t *obj;
    int i;
    
//    dmy_rd_scscr = SCSCR_MATCH[index];
//    *dmy_rd_scscr &= 0x007B;                    // Clear TIE and Write to bit15~8,2 is always 0
//    dmy_rd_scfsr = SCFSR_MATCH[index];
//    *dmy_rd_scfsr = (*dmy_rd_scfsr & ~0x0020);  // Set TEND
//
    obj = uart_data[index].tx_obj;
    if (obj) {
        i = obj->tx_buff.length - obj->tx_buff.pos;
        if (0 < i) {
            if (serial_available_buffer(obj) < i) {
                i = serial_available_buffer(obj);
            }
            do {
                uint8_t c = *(uint8_t *)obj->tx_buff.buffer;
                obj->tx_buff.buffer = (uint8_t *)obj->tx_buff.buffer + 1;
                ++obj->tx_buff.pos;
                obj->serial.uart->TDR = c;
            } while (--i);
            serial_put_done(obj);
        } else {
            uart_data[index].tx_obj = NULL;
            uart_data[index].event = SERIAL_EVENT_TX_COMPLETE;
            ((void (*)())uart_data[index].async_tx_callback)();
        }
    }

    irq_handler(uart_data[index].serial_irq_id, TxIrq);
}

static void uart_rx_irq(IRQn_Type irq_num, uint32_t index) {
    __IO uint16_t *dmy_rd_scscr;
    __IO uint16_t *dmy_rd_scfsr;
    serial_t *obj;
    int c;
    
    dmy_rd_scscr = SCSCR_MATCH[index];
    *dmy_rd_scscr &= 0x00B3;                    // Clear RIE,REIE and Write to bit15~8,2 is always 0
    dmy_rd_scfsr = SCFSR_MATCH[index];
    *dmy_rd_scfsr = (*dmy_rd_scfsr & ~0x0003);  // Clear RDF,DR

    obj = uart_data[index].rx_obj;
    if (obj) {
        if (obj->serial.uart->SCLSR & 1) {
            if (uart_data[index].wanted_rx_events & SERIAL_EVENT_RX_OVERRUN_ERROR) {
                serial_rx_abort_asynch(obj);
                uart_data[index].event = SERIAL_EVENT_RX_OVERRUN_ERROR;
                ((void (*)())uart_data[index].async_rx_callback)();
            }
            return;
        }
        c = serial_getc(obj);
        if (c != -1) {
            ((uint8_t *)obj->rx_buff.buffer)[obj->rx_buff.pos] = c;
            ++obj->rx_buff.pos;
            if (c == obj->char_match && ! obj->char_found) {
                obj->char_found = 1;
                if (obj->rx_buff.pos == obj->rx_buff.length) {
                    if (uart_data[index].wanted_rx_events & SERIAL_EVENT_RX_COMPLETE) {
                        uart_data[index].event = SERIAL_EVENT_RX_COMPLETE;
                    }
                }
                if (uart_data[index].wanted_rx_events & SERIAL_EVENT_RX_CHARACTER_MATCH) {
                    uart_data[index].event |= SERIAL_EVENT_RX_CHARACTER_MATCH;
                }
                if (uart_data[index].event) {
                    uart_data[index].rx_obj = NULL;
                    ((void (*)())uart_data[index].async_rx_callback)();
                }
            } else if (obj->rx_buff.pos == obj->rx_buff.length) {
                uart_data[index].rx_obj = NULL;
                if (uart_data[index].wanted_rx_events & SERIAL_EVENT_RX_COMPLETE) {
                    uart_data[index].event = SERIAL_EVENT_RX_COMPLETE;
                    ((void (*)())uart_data[index].async_rx_callback)();
                }
            }
        } else {
            serial_rx_abort_asynch(obj);
            if (uart_data[index].wanted_rx_events & (SERIAL_EVENT_RX_PARITY_ERROR | SERIAL_EVENT_RX_FRAMING_ERROR)) {
                uart_data[index].event = SERIAL_EVENT_RX_PARITY_ERROR | SERIAL_EVENT_RX_FRAMING_ERROR;
                if (obj->serial.uart->SCFSR & 1 << 2) {
                    uart_data[index].event = SERIAL_EVENT_RX_PARITY_ERROR;
                } else if (obj->serial.uart->SCFSR & 1 << 3) {
                    uart_data[index].event = SERIAL_EVENT_RX_FRAMING_ERROR;
                }
                ((void (*)())uart_data[index].async_rx_callback)();
            }
            return;
        }
    }

    irq_handler(uart_data[index].serial_irq_id, RxIrq);
}

static void uart_err_irq(IRQn_Type irq_num, uint32_t index) {
    serial_t *obj = uart_data[index].receiving_obj;
    int err_read;
    
    if (obj) {
        serial_irq_err_set(obj, 0);
        if (uart_data[index].wanted_rx_events & (SERIAL_EVENT_RX_PARITY_ERROR | SERIAL_EVENT_RX_FRAMING_ERROR)) {
            uart_data[index].event = SERIAL_EVENT_RX_PARITY_ERROR | SERIAL_EVENT_RX_FRAMING_ERROR;
            if (obj->serial.uart->SCFSR & 1 << 2) {
                uart_data[index].event = SERIAL_EVENT_RX_PARITY_ERROR;
            } else if (obj->serial.uart->SCFSR & 1 << 3) {
                uart_data[index].event = SERIAL_EVENT_RX_FRAMING_ERROR;
            }
            ((void (*)())uart_data[index].async_rx_callback)();
        }
        serial_rx_abort_asynch(obj);

        core_util_critical_section_enter();
        if (obj->serial.uart->SCFSR & 0x93) {
            err_read = obj->serial.uart->SCFSR;
            obj->serial.uart->SCFSR = (err_read & ~0x93);
        }
        if (obj->serial.uart->SCLSR & 1) {
            obj->serial.uart->SCLSR = 0;
        }
        core_util_critical_section_exit();
    }
}


static void uart0_rxi_isr(void)  { uart_rx_irq(UART0); }
static void uart1_rxi_isr(void)  { uart_rx_irq(UART1); }
static void uart2_rxi_isr(void)  { uart_rx_irq(UART2); }
static void uart3_rxi_isr(void)  { uart_rx_irq(UART3); }
static void uart4_rxi_isr(void)  { uart_rx_irq(UART4); }
static void uart5_rxi_isr(void)  { uart_rx_irq(UART5); }
static void uart6_rxi_isr(void)  { uart_rx_irq(UART6); }
static void uart7_rxi_isr(void)  { uart_rx_irq(UART7); }
static void uart8_rxi_isr(void)  { uart_rx_irq(UART8); }
static void uart9_rxi_isr(void)  { uart_rx_irq(UART9); }

static void uart0_txi_isr(void)  { uart_tx_irq(UART0); }
static void uart1_txi_isr(void)  { uart_tx_irq(UART1); }
static void uart2_txi_isr(void)  { uart_tx_irq(UART2); }
static void uart3_txi_isr(void)  { uart_tx_irq(UART3); }
static void uart4_txi_isr(void)  { uart_tx_irq(UART4); }
static void uart5_txi_isr(void)  { uart_tx_irq(UART5); }
static void uart6_txi_isr(void)  { uart_tx_irq(UART6); }
static void uart7_txi_isr(void)  { uart_tx_irq(UART7); }
static void uart8_txi_isr(void)  { uart_tx_irq(UART8); }
static void uart9_txi_isr(void)  { uart_tx_irq(UART9); }

static void uart0_tei_isr(void)  { uart_tei_irq(UART0); }
static void uart1_tei_isr(void)  { uart_tei_irq(UART1); }
static void uart2_tei_isr(void)  { uart_tei_irq(UART2); }
static void uart3_tei_isr(void)  { uart_tei_irq(UART3); }
static void uart4_tei_isr(void)  { uart_tei_irq(UART4); }
static void uart5_tei_isr(void)  { uart_tei_irq(UART5); }
static void uart6_tei_isr(void)  { uart_tei_irq(UART6); }
static void uart7_tei_isr(void)  { uart_tei_irq(UART7); }
static void uart8_tei_isr(void)  { uart_tei_irq(UART8); }
static void uart9_tei_isr(void)  { uart_tei_irq(UART9); }

static void uart0_eri_isr(void)  { uart_eri_irq(UART0); }
static void uart1_eri_isr(void)  { uart_eri_irq(UART1); }
static void uart2_eri_isr(void)  { uart_eri_irq(UART2); }
static void uart3_eri_isr(void)  { uart_eri_irq(UART3); }
static void uart4_eri_isr(void)  { uart_eri_irq(UART4); }
static void uart5_eri_isr(void)  { uart_eri_irq(UART5); }
static void uart6_eri_isr(void)  { uart_eri_irq(UART6); }
static void uart7_eri_isr(void)  { uart_eri_irq(UART7); }
static void uart8_eri_isr(void)  { uart_eri_irq(UART8); }
static void uart9_eri_isr(void)  { uart_eri_irq(UART9); }


static uint8_t serial_available_buffer(serial_t *obj)
{
    return 1;
    /* Faster but unstable way */
    /*
     uint16_t ret = 16 - ((obj->serial.uart->SCFDR >> 8) & 0x1F);
     while (ret == 0) {
     ret = 16 - ((obj->serial.uart->SCFDR >> 8) & 0x1F);
     }
     MBED_ASSERT(0 < ret && ret <= 16);
     return ret;
     */
}

/* -------------------------------------------------------------------------+
 |   GLOBAL FUNCTIONS                                                       |
 + -------------------------------------------------------------------------*/

/*==========================================================================*/
void serial_init(serial_t *obj, PinName tx, PinName rx) 
/*==========================================================================*/
{
    volatile uint8_t dummy ;
    int is_stdio_uart = 0;
    // determine the UART to use
    uint32_t uart_tx = pinmap_peripheral(tx, PinMap_UART_TX);
    uint32_t uart_rx = pinmap_peripheral(rx, PinMap_UART_RX);
    uint32_t uart = pinmap_merge(uart_tx, uart_rx);

    MBED_ASSERT((int)uart != NC);

    obj->serial.uart = ((R_SCI0_Type *) (R_SCI0_BASE + (R_SCI_REG_SIZE * uart)));

    obj->serial.uart->SCMR = 0xF2;         // Transfer LSB-first,SMIF = 0

    /* ==== SCIF initial setting ==== */
    /* ---- Serial control register (SCSCR) setting ---- */
    /* B'00 : Internal CLK */
    obj->serial.uart->SCR = 0x00u;          /* SCIF transmitting and receiving operations stop */
//
    /* ---- FIFO control register (SCFCR) setting ---- */
    /* Transmit FIFO reset & Receive FIFO data register reset */
    obj->serial.uart->FCR = 0x0006u;

    /* ---- Serial status register (SCFSR) setting ---- */
    dummy = obj->serial.uart->SSR;
    obj->serial.uart->SSR = (dummy & 0x82u);         /* ER,BRK,DR bit clear */

    /* ---- Line status register (SCLSR) setting ---- */
    /* ORER bit clear */
    //obj->serial.uart->LSR = 0;

    /* ---- Serial extended mode register (SEMR) setting ----
    b6 BGDM - Baud rate generator double-speed mode  : Normal mode
    b4 ABCS - Base clock select in asynchronous mode : Base clock is 16 times the bit rate */
    obj->serial.uart->SEMR = 0x00u;

    /* ---- Bit rate register (SCBRR) setting ---- */
    serial_baud  (obj, 9600);
    serial_format(obj, 8, ParityNone, 1);

    /* ---- FIFO control register (SCFCR) setting ---- */
    obj->serial.uart->FCR = 0x0030u;

    /* ---- Serial port register (SCSPTR) setting ----
    b2 SPB2IO - Serial port break output : disabled
    b1 SPB2DT - Serial port break data   : High-level */
    obj->serial.uart->SPTR = 0x06u;    // SPB2IO = 1, SPB2DT = 1

    /* ---- Line status register (LSR) setting ----
    b0 ORER - Overrun error detect : clear */

    if (obj->serial.uart->SSR_b.ORER) {
        obj->serial.uart->SSR_b.ORER = 0;      // ORER clear
    }

    // pinout the chosen uart
    pinmap_pinout(tx, PinMap_UART_TX);
    pinmap_pinout(rx, PinMap_UART_RX);

    obj->serial.index = uart;

    uart_data[obj->serial.index].sw_rts.pin = NC;
    uart_data[obj->serial.index].sw_cts.pin = NC;

    /* ---- Serial control register (SCR) setting ---- */
    /* Setting the TE and RE bits enables the TxD and RxD pins to be used. */
    obj->serial.uart->SCR = 0x30;

    is_stdio_uart = (uart == STDIO_UART) ? (1) : (0);

    if (is_stdio_uart) {
        stdio_uart_inited = 1;
        memcpy(&stdio_uart, obj, sizeof(serial_t));
    }
}

/*==========================================================================*/
void serial_free(serial_t *obj) 
/*==========================================================================*/
{
    uart_data[obj->serial.index].serial_irq_id = 0;
}


/*==========================================================================*/
// serial_baud
// set the baud rate, taking in to account the current SystemFrequency
void serial_baud(serial_t *obj, int baudrate) 
/*==========================================================================*/
{
    baud_setting_t baud_setting;

    uint32_t baud_rate_error_x_1000 = (10*1000);

    baud_setting.brr = SCI_UART_BRR_MAX;
    baud_setting.semr_baudrate_bits_b.brme = 0U;
    baud_setting.mddr = SCI_UART_MDDR_MIN;

    /* Find the best BRR (bit rate register) value.
     *  In table g_async_baud, divisor values are stored for BGDM, ABCS, ABCSE and N values.  Each set of divisors
     *  is tried, and the settings with the lowest bit rate error are stored. The formula to calculate BRR is as
     *  follows and it must be 255 or less:
     *  BRR = (PCLK / (div_coefficient * baud)) - 1
     */

    int32_t  hit_bit_err = SCI_UART_100_PERCENT_X_1000;
    uint32_t hit_mddr    = 0U;
    uint32_t divisor     = 0U;

    uint32_t freq_hz = R_FSP_SystemClockHzGet(BSP_FEATURE_SCI_CLOCK);

    for (uint32_t select_16_base_clk_cycles = 0U;
         select_16_base_clk_cycles <= 1U && (hit_bit_err > ((int32_t) baud_rate_error_x_1000));
         select_16_base_clk_cycles++)
    {
        for (uint32_t i = 0U; i < SCI_UART_NUM_DIVISORS_ASYNC; i++)
        {
            /* if select_16_base_clk_cycles == true:  Skip this calculation for divisors that are not acheivable with 16 base clk cycles per bit.
             *  if select_16_base_clk_cycles == false: Skip this calculation for divisors that are only acheivable without 16 base clk cycles per bit.
             */
            if (((uint8_t) select_16_base_clk_cycles) ^ (g_async_baud[i].abcs | g_async_baud[i].abcse))
            {
                continue;
            }

            divisor = (uint32_t) g_div_coefficient[i] * baudrate;
            uint32_t temp_brr = freq_hz / divisor;

            if (temp_brr <= (SCI_UART_BRR_MAX + 1U))
            {
                while (temp_brr > 0U)
                {
                    temp_brr -= 1U;

                    /* Calculate the bit rate error. The formula is as follows:
                     *  bit rate error[%] = {(PCLK / (baud * div_coefficient * (BRR + 1)) - 1} x 100
                     *  calculates bit rate error[%] to three decimal places
                     */
                    int32_t err_divisor = (int32_t) (divisor * (temp_brr + 1U));

                    /* Promoting to 64 bits for calculation, but the final value can never be more than 32 bits, as
                     * described below, so this cast is safe.
                     *    1. (temp_brr + 1) can be off by an upper limit of 1 due to rounding from the calculation:
                     *       freq_hz / divisor, or:
                     *       freq_hz / divisor <= (temp_brr + 1) < (freq_hz / divisor) + 1
                     *    2. Solving for err_divisor:
                     *       freq_hz <= err_divisor < freq_hz + divisor
                     *    3. Solving for bit_err:
                     *       0 >= bit_err >= (freq_hz * 100000 / (freq_hz + divisor)) - 100000
                     *    4. freq_hz >= divisor (or temp_brr would be -1 and we would never enter this while loop), so:
                     *       0 >= bit_err >= 100000 / freq_hz - 100000
                     *    5. Larger frequencies yield larger bit errors (absolute value).  As the frequency grows,
                     *       the bit_err approaches -100000, so:
                     *       0 >= bit_err >= -100000
                     *    6. bit_err is between -100000 and 0.  This entire range fits in an int32_t type, so the cast
                     *       to (int32_t) is safe.
                     */
                    int32_t bit_err = (int32_t) (((((int64_t) freq_hz) * SCI_UART_100_PERCENT_X_1000) /
                                                  err_divisor) - SCI_UART_100_PERCENT_X_1000);

                    /* Take the absolute value of the bit rate error. */
                    if (bit_err < 0)
                    {
                        bit_err = -bit_err;
                    }

                    /* If the absolute value of the bit rate error is less than the previous lowest absolute value of
                     *  bit rate error, then store these settings as the best value.
                     */
                    if (bit_err < hit_bit_err)
                    {
                        baud_setting.semr_baudrate_bits_b.bgdm  = g_async_baud[i].bgdm;
                        baud_setting.semr_baudrate_bits_b.abcs  = g_async_baud[i].abcs;
                        baud_setting.semr_baudrate_bits_b.abcse = g_async_baud[i].abcse;
                        baud_setting.cks = g_async_baud[i].cks;
                        baud_setting.brr = (uint8_t) temp_brr;
                        hit_bit_err         = bit_err;
                    }

                    break;
                }
            }
        }
    }

    if ( hit_bit_err <= (int32_t) baud_rate_error_x_1000 )
    {
        obj->serial.uart->SEMR_b.BGDM = baud_setting.semr_baudrate_bits_b.bgdm;
        obj->serial.uart->SEMR_b.ABCS = baud_setting.semr_baudrate_bits_b.abcs;
        obj->serial.uart->SEMR_b.ABCSE = baud_setting.semr_baudrate_bits_b.abcse;
        obj->serial.uart->SMR_b.CKS = baud_setting.cks;
        obj->serial.uart->BRR = baud_setting.brr;
        obj->serial.uart->SEMR_b.BRME = 0;
    }
}

void serial_format(serial_t *obj, int data_bits, SerialParity parity, int stop_bits) {
    int parity_mode;
    int parity_select;

    MBED_ASSERT((stop_bits == 1) || (stop_bits == 2)); // 0: 1 stop bits, 1: 2 stop bits
    MBED_ASSERT((data_bits > 6) && (data_bits < 10)); // 5: 5 data bits ... 3: 8 data bits
    MBED_ASSERT((parity == ParityNone) || (parity == ParityOdd) || (parity == ParityEven) ||
                (parity == ParityForced1) || (parity == ParityForced0));

    stop_bits = (stop_bits == 1)? 0:
                (stop_bits == 2)? 1:
                0; // must not to be

    switch (data_bits) {
        case 7: data_bits = 0b11; break;
        case 9: data_bits = 0b00; break;
        case 8: 
        default:                // must not to be
            data_bits = 0b10;
            break;
    }

    switch (parity) {
    case ParityNone:
        parity_enable = 0;
        parity_mode = 0;
        break;
    case ParityOdd:
        parity_enable = 1;
        parity_mode = 1;
        break;
    case ParityEven:
        parity_enable = 1;
        parity_mode = 0;
        break;
    case ParityForced1:
    case ParityForced0:
    default:
        parity_enable = 0;
        parity_mode = 0;
        break;
    }

    obj->serial.uart->SMR_b.CHR = data_bits;    
    obj->serial.uart->SMR_b.PM = parity_mode;
    obj->serial.uart->SMR_b.PE = parity_enable;
    obj->serial.uart->SMR_b.STOP = stop_bits;
}




void serial_irq_handler(serial_t *obj, uart_irq_handler handler, uint32_t id) {
    irq_handler = handler;
    uart_data[obj->serial.index].serial_irq_id = id;
}

static void serial_irq_set_irq(IRQn_Type IRQn, IRQHandler handler, uint32_t enable)
{
    if (enable) {
        NVIC_SetVector(IRQn, handler);
        NVIC_EnableIRQ(IRQn);
    } else {
        NVIC_DisableIRQ(IRQn);
    }
}

void sci_uart_txi_isr (void)
{
}

void sci_uart_rxi_isr (void)
{
}

void sci_uart_tei_isr (void)
{
}

void sci_uart_eri_isr (void)
{
}

static void serial_irq_err_set(serial_t *obj, uint32_t enable)
{
//  serial_irq_set_irq(irq_set_tbl[obj->serial.index][2], hander_set_tbl[obj->serial.index][2], enable);
//  serial_irq_set_irq(irq_set_tbl[obj->serial.index][3], hander_set_tbl[obj->serial.index][3], enable);
}

void serial_irq_set(serial_t *obj, SerialIrq irq, uint32_t enable) 
{
    IRQn_Type IRQn;
    IRQHandler handler;
//
    IRQn = irq_set_tbl[obj->serial.index][irq];
    handler = hander_set_tbl[obj->serial.index][irq];
//
    if ((obj->serial.index >= 0) && (obj->serial.index <= 7)) {
        serial_irq_set_irq(IRQn, handler, enable);
    }
}
//
/******************************************************************************
 * READ/WRITE
 ******************************************************************************/
int serial_getc(serial_t *obj) {
    uint16_t err_read;
    int data;

    core_util_critical_section_enter();
    if (obj->serial.uart->SSR_FIFO & 0x93) {
        err_read = obj->serial.uart->SSR_FIFO;
        obj->serial.uart->SSR_FIFO = (err_read & ~0x93);
    }
    obj->serial.uart->SCR |= 0x0040;     // Set RIE
    core_util_critical_section_exit();

    if (obj->serial.uart->SSR_b.ORER) {
        obj->serial.uart->SSR_b.ORER = 0;      // ORER clear
    }

    while (!serial_readable(obj));
    data = obj->serial.uart->FRDRHL & 0xff;

    core_util_critical_section_enter();
    err_read = obj->serial.uart->SSR;
    obj->serial.uart->SSR = (err_read & 0xfffD);     // Clear RDF
    core_util_critical_section_exit();

    if (err_read & 0x80) {
        data = -1;  //err
    }
    return data;
}

void serial_putc(serial_t *obj, int c) {
    while (!serial_writable(obj));
    obj->serial.uart->TDR = c;
    serial_put_done(obj);
}

static void serial_put_done(serial_t *obj)
{
    volatile uint16_t dummy_read;

    core_util_critical_section_enter();
    dummy_read = obj->serial.uart->SSR;
    obj->serial.uart->SSR = (dummy_read & 0xff9f);  // Clear TEND/TDFE
    obj->serial.uart->SCR |= 0x0080;     // Set TIE
    core_util_critical_section_exit();
}

int serial_readable(serial_t *obj) {
    return ((obj->serial.uart->SSR_b.RDRF) != 0);  // RDRF
}

int serial_writable(serial_t *obj) {
    return ((obj->serial.uart->SSR & 0x80) != 0);  // TDRE
}

void serial_clear(serial_t *obj) {
    core_util_critical_section_enter();

    obj->serial.uart->FCR |=  0x0006u;       // TFRST = 1, RFRST = 1
    obj->serial.uart->FCR &= ~0x0006u;       // TFRST = 0, RFRST = 0
    obj->serial.uart->SSR_FIFO &= ~0x78u;       // ER, BRK, RDF, DR = 0

    core_util_critical_section_exit();
}

void serial_pinout_tx(PinName tx) {
    pinmap_pinout(tx, PinMap_UART_TX);
}

void serial_break_set(serial_t *obj) {
    core_util_critical_section_enter();
    // TxD Output(L)
    obj->serial.uart->SPTR &= ~0x02u;  // SPB2DT = 0
    //obj->serial.uart->SCR &= ~0x20u;   // TE = 0 (Output disable)
    obj->serial.uart->SCR_b.TE = 0;
    core_util_critical_section_exit();
}

void serial_break_clear(serial_t *obj) {
    core_util_critical_section_enter();
    obj->serial.uart->SCR |= 0x0020u; // TE = 1 (Output enable)
    obj->serial.uart->SPTR |= 0x0001u; // SPB2DT = 1
    core_util_critical_section_exit();
}

#if DEVICE_SERIAL_FC
void serial_set_flow_control(serial_t *obj, FlowControl type, PinName rxflow, PinName txflow) {
    // determine the UART to use

    if (type == FlowControlRTSCTS) {
        core_util_critical_section_enter();
        obj->serial.uart->SCFCR |= 0x0008u;   // CTS/RTS enable
        core_util_critical_section_exit();
        pinmap_pinout(rxflow, PinMap_UART_RTS);
        pinmap_pinout(txflow, PinMap_UART_CTS);
    } else {
        core_util_critical_section_enter();
        obj->serial.uart->SCFCR &= ~0x0008u; // CTS/RTS diable
        core_util_critical_section_exit();
    }
}
#endif



const PinMap *serial_tx_pinmap()
{
    return PinMap_UART_TX;
}

const PinMap *serial_rx_pinmap()
{
    return PinMap_UART_RX;
}

const PinMap *serial_cts_pinmap()
{
    return PinMap_UART_CTS;
}

const PinMap *serial_rts_pinmap()
{
    return PinMap_UART_RTS;
}

#if DEVICE_SERIAL_ASYNCH

/******************************************************************************
 * ASYNCHRONOUS HAL
 ******************************************************************************/

int serial_tx_asynch(serial_t *obj, const void *tx, size_t tx_length, uint8_t tx_width, uint32_t handler, uint32_t event, DMAUsage hint)
{
    int i;
    buffer_t *buf = &obj->tx_buff;
    struct serial_global_data_s *data = uart_data + obj->serial.index;
    
    if (tx_length == 0) {
        return 0;
    }
    
    buf->buffer = (void *)tx;
    buf->length = tx_length * tx_width / 8;
    buf->pos = 0;
    buf->width = tx_width;
    data->tranferring_obj = obj;
    data->async_tx_callback = handler;
    serial_irq_set(obj, TxIrq, 1);
    
    while (!serial_writable(obj));
    i = buf->length;
    if (serial_available_buffer(obj) < i) {
        i = serial_available_buffer(obj);
    }
    do {
        uint8_t c = *(uint8_t *)buf->buffer;
        obj->tx_buff.buffer = (uint8_t *)obj->tx_buff.buffer + 1;
        ++buf->pos;
        obj->serial.uart->TDR = c;
    } while (--i);
    serial_put_done(obj);
    
    return buf->length;
}

void serial_rx_asynch(serial_t *obj, void *rx, size_t rx_length, uint8_t rx_width, uint32_t handler, uint32_t event, uint8_t char_match, DMAUsage hint)
{
    buffer_t *buf = &obj->rx_buff;
    struct serial_global_data_s *data = uart_data + obj->serial.index;
    
    if (rx_length == 0) {
        return;
    }
    
    buf->buffer = rx;
    buf->length = rx_length * rx_width / 8;
    buf->pos = 0;
    buf->width = rx_width;
    obj->char_match = char_match;
    obj->char_found = 0;
    data->rx_obj = obj;
    data->async_rx_callback = handler;
    data->event = 0;
    data->wanted_rx_events = event;
    
    serial_irq_set(obj, RxIrq, 1);
    serial_irq_err_set(obj, 1);
}

uint8_t serial_tx_active(serial_t *obj)
{
    return uart_data[obj->serial.index].tx_obj != NULL;
}

uint8_t serial_rx_active(serial_t *obj)
{
    return uart_data[obj->serial.index].rx_obj != NULL;
}

int serial_irq_handler_asynch(serial_t *obj)
{
    return uart_data[obj->serial.index].event;
}

void serial_tx_abort_asynch(serial_t *obj)
{
    uart_data[obj->serial.index].tx_obj = NULL;
    obj->serial.uart->SCR |= 1 << 2;
    obj->serial.uart->SCR &= ~(1 << 2);
}

void serial_rx_abort_asynch(serial_t *obj)
{
    uart_data[obj->serial.index].rx_obj = NULL;
    obj->serial.uart->SCR |= 1 << 1;
    obj->serial.uart->SCR &= ~(1 << 1);
}

#endif
