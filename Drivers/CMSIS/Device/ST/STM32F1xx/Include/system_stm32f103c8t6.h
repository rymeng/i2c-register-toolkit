/*
 * Copyright (c) 2017-2021 STMicroelectronics.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at

 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * This software is licensed under terms that can be found in the LICENSE file in the root directory
 * of this software component.
 *
 * If no LICENSE file comes with this software, it is provided AS-IS.
 */

#ifndef I2C_REGISTER_TOOLKIT_DRIVERS_CMSIS_DEVICE_INCLUDE_SYSTEM_STM32F103C8T6_H_
#define I2C_REGISTER_TOOLKIT_DRIVERS_CMSIS_DEVICE_INCLUDE_SYSTEM_STM32F103C8T6_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx.h"

#ifndef HSE_VALUE
  #define HSE_VALUE (8000000U)
#endif

#ifndef HSI_VALUE
  #define HSI_VALUE (8000000U)
#endif

// #define USER_VECT_TAB_ADDRESS

#ifdef USER_VECT_TAB_ADDRESS
  // #define VECT_TAB_SRAM
  #ifdef VECT_TAB_SRAM
    #define VECT_TAB_BASE_ADDRESS SRAM_BASE
    #define VECT_TAB_OFFSET       (0x00000000U)
  #else
    #define VECT_TAB_BASE_ADDRESS FLASH_BASE
    #define VECT_TAB_OFFSET       (0x00000000U)
  #endif
#endif

extern uint32_t SystemCoreClock;
extern const uint8_t AHBPrescTable[16];
extern const uint8_t APBPrescTable[8];

void SystemInit(void);
void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif
