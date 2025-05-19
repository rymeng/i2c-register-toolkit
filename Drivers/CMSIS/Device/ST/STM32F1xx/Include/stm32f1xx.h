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

#ifndef I2C_REGISTER_TOOLKIT_DRIVERS_CMSIS_DEVICE_INCLUDE_STM32F1XX_H_
#define I2C_REGISTER_TOOLKIT_DRIVERS_CMSIS_DEVICE_INCLUDE_STM32F1XX_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef STM32F1
  #define STM32F1
#endif

#if !defined(STM32F100xB) && !defined(STM32F100xE) &&                                                   \
    !defined(STM32F101x6) && !defined(STM32F101xB) && !defined(STM32F101xE) && !defined(STM32F101xG) && \
    !defined(STM32F102x6) && !defined(STM32F102xB) &&                                                   \
    !defined(STM32F103x6) && !defined(STM32F103xB) && !defined(STM32F103xE) && !defined(STM32F103xG) && \
    !defined(STM32F105xC) &&                                                                            \
    !defined(STM32F107xC)
  // #define STM32F100xB
  // #define STM32F100xE
  // #define STM32F101x6
  // #define STM32F101xB
  // #define STM32F101xE
  // #define STM32F101xG
  // #define STM32F102x6
  // #define STM32F102xB
  // #define STM32F103x6
  #define STM32F103xB
  // #define STM32F103xE
  // #define STM32F103xG
  // #define STM32F105xC
  // #define STM32F107xC
#endif

#ifndef USE_HAL_DRIVER
  #define USE_HAL_DRIVER
#endif

#define __STM32F1_CMSIS_VERSION_MAIN  (0x04U)
#define __STM32F1_CMSIS_VERSION_SUB1  (0x03U)
#define __STM32F1_CMSIS_VERSION_SUB2  (0x05U)
#define __STM32F1_CMSIS_VERSION_RC    (0x00U)

#define __STM32F1_CMSIS_VERSION             \
  (                                         \
    (__STM32F1_CMSIS_VERSION_MAIN << 24) |  \
    (__STM32F1_CMSIS_VERSION_SUB1 << 16) |  \
    (__STM32F1_CMSIS_VERSION_SUB2 <<  8) |  \
    (__STM32F1_CMSIS_VERSION_RC   <<  0)    \
  )

#if defined(STM32F100xB)
  #include "stm32f100xb.h"
#elif defined(STM32F100xE)
  #include "stm32f100xe.h"
#elif defined(STM32F101x6)
  #include "stm32f101x6.h"
#elif defined(STM32F101xB)
  #include "stm32f101xb.h"
#elif defined(STM32F101xE)
  #include "stm32f101xe.h"
#elif defined(STM32F101xG)
  #include "stm32f101xg.h"
#elif defined(STM32F102x6)
  #include "stm32f102x6.h"
#elif defined(STM32F102xB)
  #include "stm32f102xb.h"
#elif defined(STM32F103x6)
  #include "stm32f103x6.h"
#elif defined(STM32F103xB)
  #include "stm32f103xb.h"
#elif defined(STM32F103xE)
  #include "stm32f103xe.h"
#elif defined(STM32F103xG)
  #include "stm32f103xg.h"
#elif defined(STM32F105xC)
  #include "stm32f105xc.h"
#elif defined(STM32F107xC)
  #include "stm32f107xc.h"
#else
  #error "Please select first the target STM32F1xx device used (In CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h file)."
#endif

typedef enum
{
  RESET = 0,
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum
{
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;

#define IS_FUNCTIONAL_STATE(STATE)              \
  (                                             \
    ((STATE) == DISABLE) || ((STATE) == ENABLE) \
  )

typedef enum
{
  SUCCESS = 0U,
  ERROR = !SUCCESS
} ErrorStatus;

#define READ_BIT(REG, MASK)                 ((REG) & (MASK))
#define SET_BIT(REG, SETMASK)               ((REG) |= (SETMASK))
#define CLEAR_BIT(REG, CLEARMASK)           ((REG) &= ~(CLEARMASK))

#define READ_REG(REG)                       ((REG))
#define CLEAR_REG(REG)                      ((REG) = (0x0))
#define WRITE_REG(REG, VAL)                 ((REG) = (VAL))

#define POSITION_VAL(VAL)                   (__CLZ(__RBIT(VAL)))

#define MODIFY_REG(REG, CLEARMASK, SETMASK) WRITE_REG((REG), (READ_REG(REG)) & (~(CLEARMASK)) | (SETMASK))

#define ATOMIC_SET_BIT(REG, SETMASK)                                        \
  do {                                                                      \
    uint32_t val;                                                           \
    do {                                                                    \
      val = __LDREXW((__IO uint32_t *)&(REG)) | (SETMASK);                  \
    } while (__STREXW(val, (__IO uint32_t *)&(REG)) != 0);                  \
  } while (0)

#define ATOMIC_CLEAR_BIT(REG, CLEARMASK)                                    \
  do {                                                                      \
    uint32_t val;                                                           \
    do {                                                                    \
      val = __LDREXW((__IO uint32_t *)&(REG)) & ~(CLEARMASK);               \
    } while (__STREXW(val, (__IO uint32_t *)&(REG)) != 0);                  \
  } while (0)

#define ATOMIC_MODIFY_REG(REG, CLEARMASK, SETMASK)                          \
  do {                                                                      \
    uint32_t val;                                                           \
    do {                                                                    \
      val = __LDREXW((__IO uint32_t *)&(REG)) & ~(CLEARMASK) | (SETMASK);   \
    } while (__STREXW(val, (__IO uint32_t *)&(REG)) != 0);                  \
  } while (0)

#define ATOMIC_SETH_BIT(REG, SETMASK)                                       \
  do {                                                                      \
    uint16_t val;                                                           \
      do {                                                                  \
        val = __LDREXW((__IO uint16_t *)&(REG)) | (SETMASK);                \
      } while (__STREXW(val, (__IO uint16_t *)&(REG)) != 0);                \
  } while (0)

#define ATOMIC_CLEARH_BIT(REG, CLEARMASK)                                   \
  do {                                                                      \
    uint16_t val;                                                           \
    do {                                                                    \
      val = __LDREXW((__IO uint16_t *)&(REG)) & ~(CLEARMASK);               \
    } while (__STREXW(val, (__IO uint16_t *)&(REG)) != 0);                  \
  } while (0)

#define ATOMIC_MODIFYH_REG(REG, CLEARMASK, SETMASK)                         \
  do {                                                                      \
    uint16_t val;                                                           \
    do {                                                                    \
      val = __LDREXW((__IO uint16_t *)&(REG)) & ~(CLEARMASK) | (SETMASK);   \
    } while (__STREXW(val, (__IO uint16_t *)&(REG)) != 0);                  \
  } while (0)

#ifdef USE_HAL_DRIVER
  #include "stm32f1xx_hal.h"
#endif

#ifdef __cplusplus
}
#endif

#endif
