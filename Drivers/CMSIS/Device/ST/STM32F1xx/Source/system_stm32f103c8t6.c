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

/*
 * Modifications made by Rainer Meng on 2025:
 * Restructured function layout for improved readability
 */

#include "system_stm32f103c8t6.h"

uint32_t SystemCoreClock = HSI_VALUE;
const uint8_t AHBPrescTable[16] =
{
  0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
  0x01U, 0x02U, 0x03U, 0x04U, 0x06U, 0x07U, 0x08U, 0x09U
};
const uint8_t APBPrescTable[8] =
{
  0x00U, 0x00U, 0x00U, 0x00U, 0x01U, 0x02U, 0x03U, 0x04U
};

void SystemInit(void)
{
#ifdef USER_VECT_TAB_ADDRESS
  SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET;
#endif
}

void SystemCoreClockUpdate(void)
{
  const uint32_t sws_value = _FLD2VAL(RCC_CFGR_SWS, RCC->CFGR);
  switch (sws_value)
  {
    case 0x00000000U:
    {
      SystemCoreClock = HSI_VALUE;
      break;
    }
    case 0x00000001U:
    {
      SystemCoreClock = HSE_VALUE;
      break;
    }
    case 0x00000002U:
    {
      const uint32_t pllsrc_value = _FLD2VAL(RCC_CFGR_PLLSRC, RCC->CFGR);
      if (pllsrc_value == 0x00000000U)
      {
        SystemCoreClock = HSI_VALUE >> 1;
      }
      else
      {
        const uint32_t pllxtpre_value = _FLD2VAL(RCC_CFGR_PLLXTPRE, RCC->CFGR);
        if (pllxtpre_value == 0x00000000U)
        {
          SystemCoreClock = HSE_VALUE >> 0;
        }
        else
        {
          SystemCoreClock = HSE_VALUE >> 1;
        }
      }
      const uint32_t pllmull_value = _FLD2VAL(RCC_CFGR_PLLMULL, RCC->CFGR);
      const uint32_t pll_mult_factor = (pllmull_value < 0xFU) ? (pllmull_value + 0x2U) : 0x10U;
      SystemCoreClock *= pll_mult_factor;
      break;
    }
    default:
    {
      SystemCoreClock = HSI_VALUE;
      break;
    }
  }
  const uint32_t hpre_value = _FLD2VAL(RCC_CFGR_HPRE, RCC->CFGR);
  const uint32_t ahb_psc_shift = AHBPrescTable[hpre_value];
  SystemCoreClock >>= ahb_psc_shift;
}
