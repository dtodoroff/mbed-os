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
#include "gpio_api.h"
#include "pinmap.h"
#include "mbed_drv_cfg.h"


uint32_t gpio_set(PinName pin) {
    pin_function(pin, PFS_GPIO);
    return (1 << PINNO(pin));
}

void gpio_init(gpio_t *obj, PinName pin) {
    int group ;
    obj->pin = pin;
    if(pin == NC) return;
    
    obj->mask = gpio_set(pin);

    group = PINGROUP(pin);

    if (group > GPIO_GROUP_MAX) return;

    if (R_PMISC->PWPR_b.PFSWE == 0) {
        R_PMISC->PWPR_b.B0WI = 0;
        R_PMISC->PWPR_b.PFSWE = 1;
    }

    obj->reg_PmnnFS = &(R_PFS->PORT[group].PIN[PINNO(pin)]);

}

void gpio_mode(gpio_t *obj, PinMode mode) {
    switch( mode )
    {
    case PullUp:
        obj->reg_PmnnFS->PmnPFS_b.PCR = 1; // Enable input pull-up.
        break;

    case OpenDrain:
        obj->reg_PmnnFS->PmnPFS_b.NCODR = 1; // NMOS open-drain output.
        break;

    case PullNone:
    case PullDown:
        obj->reg_PmnnFS->PmnPFS_b.PCR = 0; // Disable input pull-up
        break;

    }

}

void gpio_dir(gpio_t *obj, PinDirection direction) {
    switch (direction) {
        case PIN_INPUT :
            obj->reg_PmnnFS->PmnPFS_b.PDR = 0;
            break;
        case PIN_OUTPUT:
            obj->reg_PmnnFS->PmnPFS_b.PDR = 1;
            break;
    }
}
