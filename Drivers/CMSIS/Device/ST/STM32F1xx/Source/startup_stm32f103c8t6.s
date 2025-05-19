; Copyright (c) 2017-2021 STMicroelectronics.
; Licensed under the Apache License, Version 2.0 (the "License");
; you may not use this file except in compliance with the License.
; You may obtain a copy of the License at
;
;      http://www.apache.org/licenses/LICENSE-2.0
;
; Unless required by applicable law or agreed to in writing, software
; distributed under the License is distributed on an "AS IS" BASIS,
; WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; See the License for the specific language governing permissions and
; limitations under the License.
;
; This software is licensed under terms that can be found in the LICENSE file in the root directory
; of this software component.
;
; If no LICENSE file comes with this software, it is provided AS-IS.

; Stack
                                AREA            STACK, NOINIT, READWRITE, ALIGN=3

                                IF              :DEF:__MICROLIB
                                EXPORT          __initial_sp
                                ENDIF

Stack_Size                      EQU             0x0800

Stack_Bottom                    SPACE           Stack_Size
Stack_Top
__initial_sp

; Heap
                                AREA            HEAP, NOINIT, READWRITE, ALIGN=3

                                IF              :DEF:__MICROLIB
                                EXPORT          __heap_base
                                EXPORT          __heap_limit
                                ENDIF

Heap_Size                       EQU             0x0800

__heap_base
Heap_Base                       SPACE           Heap_Size
Heap_Limit
__heap_limit

; Reset
                                AREA            RESET, DATA, READONLY, ALIGN=2

                                EXPORT          __Vectors
                                EXPORT          __Vectors_End
                                EXPORT          __Vectors_Size

__Vectors
                                DCD             Stack_Top
                                DCD             Reset_Handler
                                DCD             NMI_Handler
                                DCD             HardFault_Handler
                                DCD             MemManage_Handler
                                DCD             BusFault_Handler
                                DCD             UsageFault_Handler
                                DCD             0
                                DCD             0
                                DCD             0
                                DCD             0
                                DCD             SVC_Handler
                                DCD             DebugMon_Handler
                                DCD             0
                                DCD             PendSV_Handler
                                DCD             SysTick_Handler
                                DCD             WWDG_IRQHandler
                                DCD             PVD_IRQHandler
                                DCD             TAMPER_IRQHandler
                                DCD             RTC_IRQHandler
                                DCD             FLASH_IRQHandler
                                DCD             RCC_IRQHandler
                                DCD             EXTI0_IRQHandler
                                DCD             EXTI1_IRQHandler
                                DCD             EXTI2_IRQHandler
                                DCD             EXTI3_IRQHandler
                                DCD             EXTI4_IRQHandler
                                DCD             DMA1_Channel1_IRQHandler
                                DCD             DMA1_Channel2_IRQHandler
                                DCD             DMA1_Channel3_IRQHandler
                                DCD             DMA1_Channel4_IRQHandler
                                DCD             DMA1_Channel5_IRQHandler
                                DCD             DMA1_Channel6_IRQHandler
                                DCD             DMA1_Channel7_IRQHandler
                                DCD             ADC1_2_IRQHandler
                                DCD             USB_HP_CAN1_TX_IRQHandler
                                DCD             USB_LP_CAN1_RX0_IRQHandler
                                DCD             CAN1_RX1_IRQHandler
                                DCD             CAN1_SCE_IRQHandler
                                DCD             EXTI9_5_IRQHandler
                                DCD             TIM1_BRK_IRQHandler
                                DCD             TIM1_UP_IRQHandler
                                DCD             TIM1_TRG_COM_IRQHandler
                                DCD             TIM1_CC_IRQHandler
                                DCD             TIM2_IRQHandler
                                DCD             TIM3_IRQHandler
                                DCD             TIM4_IRQHandler
                                DCD             I2C1_EV_IRQHandler
                                DCD             I2C1_ER_IRQHandler
                                DCD             I2C2_EV_IRQHandler
                                DCD             I2C2_ER_IRQHandler
                                DCD             SPI1_IRQHandler
                                DCD             SPI2_IRQHandler
                                DCD             USART1_IRQHandler
                                DCD             USART2_IRQHandler
                                DCD             USART3_IRQHandler
                                DCD             EXTI15_10_IRQHandler
                                DCD             RTC_Alarm_IRQHandler
                                DCD             USBWakeUp_IRQHandler
__Vectors_End

__Vectors_Size                  EQU             __Vectors_End - __Vectors

;ISR
                                AREA            |.text|, CODE, READONLY

                                PRESERVE8
                                THUMB

                                IMPORT          SystemInit
                                IMPORT          __main

                                EXPORT          Reset_Handler                   [WEAK]
                                EXPORT          NMI_Handler                     [WEAK]
                                EXPORT          HardFault_Handler               [WEAK]
                                EXPORT          MemManage_Handler               [WEAK]
                                EXPORT          BusFault_Handler                [WEAK]
                                EXPORT          UsageFault_Handler              [WEAK]
                                EXPORT          SVC_Handler                     [WEAK]
                                EXPORT          DebugMon_Handler                [WEAK]
                                EXPORT          PendSV_Handler                  [WEAK]
                                EXPORT          SysTick_Handler                 [WEAK]
                                EXPORT          WWDG_IRQHandler                 [WEAK]
                                EXPORT          PVD_IRQHandler                  [WEAK]
                                EXPORT          TAMPER_IRQHandler               [WEAK]
                                EXPORT          RTC_IRQHandler                  [WEAK]
                                EXPORT          FLASH_IRQHandler                [WEAK]
                                EXPORT          RCC_IRQHandler                  [WEAK]
                                EXPORT          EXTI0_IRQHandler                [WEAK]
                                EXPORT          EXTI1_IRQHandler                [WEAK]
                                EXPORT          EXTI2_IRQHandler                [WEAK]
                                EXPORT          EXTI3_IRQHandler                [WEAK]
                                EXPORT          EXTI4_IRQHandler                [WEAK]
                                EXPORT          DMA1_Channel1_IRQHandler        [WEAK]
                                EXPORT          DMA1_Channel2_IRQHandler        [WEAK]
                                EXPORT          DMA1_Channel3_IRQHandler        [WEAK]
                                EXPORT          DMA1_Channel4_IRQHandler        [WEAK]
                                EXPORT          DMA1_Channel5_IRQHandler        [WEAK]
                                EXPORT          DMA1_Channel6_IRQHandler        [WEAK]
                                EXPORT          DMA1_Channel7_IRQHandler        [WEAK]
                                EXPORT          ADC1_2_IRQHandler               [WEAK]
                                EXPORT          USB_HP_CAN1_TX_IRQHandler       [WEAK]
                                EXPORT          USB_LP_CAN1_RX0_IRQHandler      [WEAK]
                                EXPORT          CAN1_RX1_IRQHandler             [WEAK]
                                EXPORT          CAN1_SCE_IRQHandler             [WEAK]
                                EXPORT          EXTI9_5_IRQHandler              [WEAK]
                                EXPORT          TIM1_BRK_IRQHandler             [WEAK]
                                EXPORT          TIM1_UP_IRQHandler              [WEAK]
                                EXPORT          TIM1_TRG_COM_IRQHandler         [WEAK]
                                EXPORT          TIM1_CC_IRQHandler              [WEAK]
                                EXPORT          TIM2_IRQHandler                 [WEAK]
                                EXPORT          TIM3_IRQHandler                 [WEAK]
                                EXPORT          TIM4_IRQHandler                 [WEAK]
                                EXPORT          I2C1_EV_IRQHandler              [WEAK]
                                EXPORT          I2C1_ER_IRQHandler              [WEAK]
                                EXPORT          I2C2_EV_IRQHandler              [WEAK]
                                EXPORT          I2C2_ER_IRQHandler              [WEAK]
                                EXPORT          SPI1_IRQHandler                 [WEAK]
                                EXPORT          SPI2_IRQHandler                 [WEAK]
                                EXPORT          USART1_IRQHandler               [WEAK]
                                EXPORT          USART2_IRQHandler               [WEAK]
                                EXPORT          USART3_IRQHandler               [WEAK]
                                EXPORT          EXTI15_10_IRQHandler            [WEAK]
                                EXPORT          RTC_Alarm_IRQHandler            [WEAK]
                                EXPORT          USBWakeUp_IRQHandler            [WEAK]

                                IF              :LNOT::DEF:__MICROLIB
                                IMPORT          __use_two_region_memory
                                EXPORT          __user_initial_stackheap
                                ENDIF

Reset_Handler                   PROC
                                LDR             R0, = SystemInit
                                BLX             R0
                                LDR             R0, = __main
                                BX              R0
                                ENDP

NMI_Handler                     PROC
                                B               .
                                ENDP

HardFault_Handler               PROC
                                B               .
                                ENDP

MemManage_Handler               PROC
                                B               .
                                ENDP

BusFault_Handler                PROC
                                B               .
                                ENDP

UsageFault_Handler              PROC
                                B               .
                                ENDP

SVC_Handler                     PROC
                                B               .
                                ENDP

DebugMon_Handler                PROC
                                B               .
                                ENDP

PendSV_Handler                  PROC
                                B               .
                                ENDP

SysTick_Handler                 PROC
                                B               .
                                ENDP

Default_Handler                 PROC
WWDG_IRQHandler
PVD_IRQHandler
TAMPER_IRQHandler
RTC_IRQHandler
FLASH_IRQHandler
RCC_IRQHandler
EXTI0_IRQHandler
EXTI1_IRQHandler
EXTI2_IRQHandler
EXTI3_IRQHandler
EXTI4_IRQHandler
DMA1_Channel1_IRQHandler
DMA1_Channel2_IRQHandler
DMA1_Channel3_IRQHandler
DMA1_Channel4_IRQHandler
DMA1_Channel5_IRQHandler
DMA1_Channel6_IRQHandler
DMA1_Channel7_IRQHandler
ADC1_2_IRQHandler
USB_HP_CAN1_TX_IRQHandler
USB_LP_CAN1_RX0_IRQHandler
CAN1_RX1_IRQHandler
CAN1_SCE_IRQHandler
EXTI9_5_IRQHandler
TIM1_BRK_IRQHandler
TIM1_UP_IRQHandler
TIM1_TRG_COM_IRQHandler
TIM1_CC_IRQHandler
TIM2_IRQHandler
TIM3_IRQHandler
TIM4_IRQHandler
I2C1_EV_IRQHandler
I2C1_ER_IRQHandler
I2C2_EV_IRQHandler
I2C2_ER_IRQHandler
SPI1_IRQHandler
SPI2_IRQHandler
USART1_IRQHandler
USART2_IRQHandler
USART3_IRQHandler
EXTI15_10_IRQHandler
RTC_Alarm_IRQHandler
USBWakeUp_IRQHandler
                                B               .
                                ENDP

                                ALIGN           4

                                IF              :LNOT::DEF:__MICROLIB
__user_initial_stackheap
                                LDR             R0, = Heap_Base
                                LDR             R1, = Stack_Top
                                LDR             R2, = Heap_Limit
                                LDR             R3, = Stack_Bottom
                                BX              LR

                                ALIGN           4
                                ENDIF

                                END
