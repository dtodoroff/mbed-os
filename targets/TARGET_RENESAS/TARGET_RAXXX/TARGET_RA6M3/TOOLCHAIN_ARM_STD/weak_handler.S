;/*
; * Copyright (c) 2013-2018 Arm Limited. All rights reserved.
; *
; * SPDX-License-Identifier: Apache-2.0
; *
; * Licensed under the Apache License, Version 2.0 (the License); you may
; * not use this file except in compliance with the License.
; * You may obtain a copy of the License at
; *
; * www.apache.org/licenses/LICENSE-2.0
; *
; * Unless required by applicable law or agreed to in writing, software
; * distributed under the License is distributed on an AS IS BASIS, WITHOUT
; * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; * See the License for the specific language governing permissions and
; * limitations under the License.
; *
; * -----------------------------------------------------------------------------
; *
; * Project:     CMSIS-RTOS RTX
; *
; * -----------------------------------------------------------------------------
; */


; Dummy Exception Handlers (infinite loops which can be modified)

                AREA    |.text|, CODE, READONLY

NMI_Handler     PROC
                EXPORT  NMI_Handler                [WEAK]
                B       .
                ENDP                                    


Default_Handler PROC
                EXPORT  HardFault_Handler          [WEAK]
                EXPORT  MemManage_Handler          [WEAK]
                EXPORT  BusFault_Handler           [WEAK]
                EXPORT  UsageFault_Handler         [WEAK]
                EXPORT  SVC_Handler                [WEAK]
                EXPORT  DebugMon_Handler           [WEAK]   
                EXPORT  PendSV_Handler             [WEAK]
                EXPORT  SysTick_Handler            [WEAK]

HardFault_Handler   
MemManage_Handler   
BusFault_Handler    
UsageFault_Handler  
SVC_Handler         
DebugMon_Handler    
PendSV_Handler      
SysTick_Handler     

                B       .

                ENDP

                ALIGN

                END
