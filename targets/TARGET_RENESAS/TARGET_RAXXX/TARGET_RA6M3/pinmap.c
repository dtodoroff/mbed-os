/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "PeripheralPins.h"
#include "mbed_error.h"
//#include "gpio_addrdefine.h"


PinName gpio_multi_guard = (PinName)NC; /* If set pin name here, setting of the "pin" is just one time */

void pin_function(PinName pin, int function) {
#if 0
    if (pin == (PinName)NC) return;
    
    int n = pin >> 4;
    int bitmask = 1<<(pin  & 0xf);

    const PinFunc * Pipc_0_func = PIPC_0_tbl;
    int pipc_data = 1;
    
    if (gpio_multi_guard != pin) {
        if (function == 0) {
            // means GPIO mode
            *PMC(n) &= ~bitmask;
        } else {
            // alt-function mode
            --function;

            if (function & (1 << 2)) { *PFCAE(n) |= bitmask;}else  { *PFCAE(n) &= ~bitmask;}
            if (function & (1 << 1)) { *PFCE(n) |= bitmask;}else  { *PFCE(n) &= ~bitmask;}
            if (function & (1 << 0)) { *PFC(n) |= bitmask;}else  { *PFC(n) &= ~bitmask;}

            while (Pipc_0_func->pin != NC) {
                if ((Pipc_0_func->pin == pin) && ((Pipc_0_func->function - 1) == function)) {
                    pipc_data = 0;
                    if (Pipc_0_func->pm == 0) {
                        *PMSR(n) = (bitmask << 16) | 0;
                    } else if (Pipc_0_func->pm == 1) {
                        *PMSR(n) = (bitmask << 16) | bitmask;
                    } else {
                        // Do Nothing
                    }
                    break;
                }
                Pipc_0_func++;
            }
            if (pipc_data == 1) {
            *PIPC(n) |= bitmask;
            } else {
                *PIPC(n) &= ~bitmask;
            }

            //if (P1_0 <= pin && pin <= P1_7 && function == 0) {
            //    *PBDC(n) |= bitmask;
            //}
            *PMC(n) |= bitmask;
        }
    } else {
        gpio_multi_guard = (PinName)NC;
    }
#else
    if (pin == (PinName)NC) 
        return;

    int port = PINGROUP(pin);

    __IOM R_PFS_PORT_PIN_Type *PmnnFS = &R_PFS->PORT[port].PIN[PINNO(pin)];

    if (gpio_multi_guard != pin) 
    {
        switch (function) {
        case PFS_GPIO: // means GPIO mode
            PmnnFS->PmnPFS_b.PMR = 0; // 0: Used as general I/O pin
            break;

        case PFS_ASEL: 
            PmnnFS->PmnPFS_b.ASEL = 1; // Used as analog pin
            break;

        case PFS_ISEL: 
            PmnnFS->PmnPFS_b.ISEL = 1; // Used as IRQn input pin
            break;

        case PFS_Reserved : 
            break;

        default: //PSEL
            if ( function <= PFS_PSEL_MAX ) {            
                function -= PFS_PSEL_0;
                PmnnFS->PmnPFS_b.PSEL = function; // select the peripheral function
                PmnnFS->PmnPFS_b.PMR = 1; // Used as I/O port for peripheral functions            
             }
            break;
        }
    }
    else 
    {
        gpio_multi_guard = (PinName)NC;
    }


#endif
}

void pin_mode(PinName pin, PinMode mode) {
//    if (pin == (PinName)NC) { return; }
}
