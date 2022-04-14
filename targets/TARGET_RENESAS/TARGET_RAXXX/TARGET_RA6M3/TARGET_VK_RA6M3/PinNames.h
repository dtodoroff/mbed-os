
/* MBED TARGET LIST: VK-RA6M3 */

#ifndef MBED_PINNAMES_H
#define MBED_PINNAMES_H

#include "cmsis.h"
//#include "PinNamesTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PIN_INPUT,
    PIN_OUTPUT
} PinDirection;

#define PORT_SHIFT  4

typedef enum {
    // Not connected
    NC = (int)0xFFFFFFFF,
    P000 = 0,
          P001, P002, P003, P004, P005,_P006, P007, P008, P009, P010,                   P014=14, P015,
    P100, P101, P102, P103, P104, P105, P106, P107, P108, P109, P110, P111, P112, P113, P114, P115,
    P200, P201, P202, P203, P204, P205, P206, P207, P208, P209, P210, P211, P212, P213, P214, 
    P300=48, P301, P302, P303, P304, P305, P306, P307, P308, P309, P310, P311, P312, P313, P314, P315,
    P400, P401, P402, P403, P404, P405, P406, P407, P408, P409, P410, P411, P412, P413, P414, P415,
    P500, P501, P502, P503, P504, P505, P506, P507, P508,             P511=91, P512, P513,
    P600=96, P601, P602, P603, P604, P605, P606, P607, P608, P609, P610, P611, P612, P613, P614, P615,
    P700, P701, P702, P703, P704, P705, P706, P707, P708,
    P800=128, P801, P802, P803, P804, P805, P806, 
    P900=144, P901,                   P905=149, P906, P907, P908,
    PA00=160, PA01,                                         PA08=168, PA09, PA10,
    PB00=176, PB01,

//
#ifdef TARGET_FF_ARDUINO_UNO
    // Arduino Uno (Rev3) pins
    ARDUINO_UNO_A0  = P000,
    ARDUINO_UNO_A1  = P001,
    ARDUINO_UNO_A2  = P002,
    ARDUINO_UNO_A3  = P008,
    ARDUINO_UNO_A4  = P009,
    ARDUINO_UNO_A5  = P014,

    ARDUINO_UNO_D0  = P614,
    ARDUINO_UNO_D1  = P613,
    ARDUINO_UNO_D2  = P111,
    ARDUINO_UNO_D3  = P112,
    ARDUINO_UNO_D4  = P113,
    ARDUINO_UNO_D5  = P114,
    ARDUINO_UNO_D6  = P115,
    ARDUINO_UNO_D7  = P107,
    ARDUINO_UNO_D8  = P106,
    ARDUINO_UNO_D9  = P105,
    ARDUINO_UNO_D10 = P104,
    ARDUINO_UNO_D11 = P203,
    ARDUINO_UNO_D12 = P202,
    ARDUINO_UNO_D13 = P204,
    ARDUINO_UNO_D14 = P206,
    ARDUINO_UNO_D15 = P205,
#endif

    // mbed Pin Names
    LED1 = P603,
    LED2 = P604,
    LED3 = P605,
    LED4 = P608,

    LED_RED    = LED1,
    LED_GREEN  = LED2,
    LED_BLUE   = LED3,
    LED_YELLOW = LED4,

    BUTTON1 = P015,  // USER_Btn [B1]
    USER_BUTTON = BUTTON1,
//

    // STDIO for console print
#ifdef MBED_CONF_TARGET_STDIO_UART_TX
    CONSOLE_TX = MBED_CONF_TARGET_STDIO_UART_TX,
#else
    CONSOLE_TX = P109,
#endif
#ifdef MBED_CONF_TARGET_STDIO_UART_RX
    CONSOLE_RX = MBED_CONF_TARGET_STDIO_UART_RX,
#else
    CONSOLE_RX = P110,
#endif

    I2C_SCL = P205,
    I2C_SDA = P206,
 
    /**** USB FS pins ****/
    USB_FS_VBUS = P407,

    /**** USB HS pins ****/
    USB_HS_OC = P707,
    USB_HS_EN = PB00,
    USB_HS_VBUS = PB01,

    /**** ETHERNET pins ****/
    ETH_MDC = P401,
    ETH_MDIO = P402,
    ETH_TXD1 = P406,
    ETH_TX_EN = P405,
    ETH_TXD0 = P700,
    ETH_REF_CLK = P701,
    ETH_RXD0 = P702,
    ETH_RXD1 = P703,
    ETH_RX_ER = P704,
    ETH_CRS_DV = P705,
    ETH_IRQ = P706,

    /**** DEBUG pins ****/
    SYS_JTCK_SWCLK = P300,
    SYS_JTMS_SWDIO = P108,

//    /**** OSCILLATOR pins ****/
//    RCC_OSC32_IN = PC_14,
//    RCC_OSC32_OUT = PC_15,
//    RCC_OSC_IN = PH_0,
//    RCC_OSC_OUT = PH_1,
//

} PinName;

typedef enum {
    PullUp = 0,
    PullDown = 3,
    PullNone = 2,
    OpenDrain = 4,
    PullDefault = PullDown
} PinMode;

#define PINGROUP(pin) (((pin)>>PORT_SHIFT)&0x0f)
#define PINNO(pin) ((pin)&0x0f)



#ifdef __cplusplus
}
#endif

#endif
