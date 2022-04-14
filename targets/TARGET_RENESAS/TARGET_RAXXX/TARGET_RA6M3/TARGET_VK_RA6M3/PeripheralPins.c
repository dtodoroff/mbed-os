 
#include "PeripheralPins.h"

/************IRQ***************/
enum {
    IRQ0,IRQ1,
    IRQ2,IRQ3,
    IRQ4,IRQ5,
    IRQ6,IRQ7,
} IRQNo;

const PinMap PinMap_IRQ[] = {

    {NC,    NC,   0}
};

/************PINMAP***************/
const PinFunc PIPC_0_tbl[] = {
//   pin      func     pm
    {NC     , 0      , -1}
};

/************ADC***************/
const PinMap PinMap_ADC[] = {
    {P000, AN0, 1},
    {P001, AN1, 1},
    {P002, AN2, 1},
    {P008, AN3, 1},
    {P009, AN4, 1},
    {P014, AN5, 1},
//    {P508, AN20, 1},
    {NC   , NC , 0}
};

/************I2C***************/
const PinMap PinMap_I2C_SDA[] = {
    {P205, I2C_1, 1},
//
    {NC    , NC   , 0}
};

const PinMap PinMap_I2C_SCL[] = {
    {P206, I2C_1, 1},
    {NC    , NC,    0}
};

/************UART***************/
const PinMap PinMap_UART_TX[] = {
    {P101, UART0, 4},
    {P411, UART0, 4},
    {P213, UART1, 5},
    {P112, UART2, 4}, 
    {P302, UART2, 4}, 
    {P109, UART9, PFS_PSEL(0b00101)}, 
    {P203, UART9, 4}, 
    {P602, UART9, 4}, 
//        
    {NC    , NC   , 0}
};

const PinMap PinMap_UART_RX[] = {
    {P101, UART0, 4},
    {P411, UART0, 4},
    {P212, UART1, 5},
    {P113, UART2, 4}, 
    {P301, UART2, 4}, 
    {P110, UART9, PFS_PSEL(0b00101)}, 
    {P202, UART9, 4}, 
    {P601, UART9, 4},     
    {NC    , NC   , 0}
};

const PinMap PinMap_UART_CTS[] = {
    {P100 , UART0, 4},
    {P410 , UART0, 4}, 
    {NC    , NC   , 0}
};

const PinMap PinMap_UART_RTS[] = {
    {NC    , NC   , 0}
};

/************SPI***************/
const PinMap PinMap_SPI_SCLK[] = {
    {NC    , NC   , 0}
};

const PinMap PinMap_SPI_MOSI[] = {
    {NC    , NC   , 0}
};

const PinMap PinMap_SPI_MISO[] = {
    {NC    , NC   , 0}
};

const PinMap PinMap_SPI_SSEL[] = {
    {NC    , NC   , 0}
};

/************PWM***************/
const PinMap PinMap_PWM[] = {
    {NC    , NC        , 0}
};

/************CAN***************/
const PinMap PinMap_CAN_RD[] = {
    {P610 , CAN_1, 3},
    {NC    , NC   , 0}
};

const PinMap PinMap_CAN_TD[] = {
    {P609  , CAN_1, 3},
    {NC    , NC   , 0}
};

