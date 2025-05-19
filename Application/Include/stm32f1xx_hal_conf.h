/*
 * SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2017 STMicroelectronics.
 * Copyright (c) 2025 Rainer Meng.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and
 *    the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 *    and the following disclaimer in the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Full text: https://opensource.org/licenses/BSD-3-Clause
 *
 * This software is licensed under terms that can be found in the LICENSE file in the root directory
 * of this repository.
 *
 * If no LICENSE file comes with this software, it is provided AS-IS.
 */

#ifndef I2C_REGISTER_TOOLKIT_APPLICATION_INCLUDE_STM32F1XX_HAL_CONF_H_
#define I2C_REGISTER_TOOLKIT_APPLICATION_INCLUDE_STM32F1XX_HAL_CONF_H_

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_MODULE_ENABLED

// #define HAL_ADC_MODULE_ENABLED
// #define HAL_CRYP_MODULE_ENABLED
// #define HAL_CAN_MODULE_ENABLED
// #define HAL_CAN_LEGACY_MODULE_ENABLED
// #define HAL_CEC_MODULE_ENABLED
// #define HAL_CORTEX_MODULE_ENABLED
// #define HAL_CRC_MODULE_ENABLED
// #define HAL_DAC_MODULE_ENABLED
// #define HAL_I2C_MODULE_ENABLED
// #define HAL_I2S_MODULE_ENABLED
// #define HAL_IRDA_MODULE_ENABLED
// #define HAL_IWDG_MODULE_ENABLED
// #define HAL_NOR_MODULE_ENABLED
// #define HAL_NAND_MODULE_ENABLED
// #define HAL_PCCARD_MODULE_ENABLED
// #define HAL_PCD_MODULE_ENABLED
// #define HAL_HCD_MODULE_ENABLED
// #define HAL_RTC_MODULE_ENABLED
// #define HAL_SD_MODULE_ENABLED
// #define HAL_MMC_MODULE_ENABLED
// #define HAL_SDRAM_MODULE_ENABLED
// #define HAL_SMARTCARD_MODULE_ENABLED
// #define HAL_SPI_MODULE_ENABLED
// #define HAL_SRAM_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
// #define HAL_UART_MODULE_ENABLED
// #define HAL_USART_MODULE_ENABLED
// #define HAL_WWDG_MODULE_ENABLED

#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED

#ifndef HSE_VALUE
#define HSE_VALUE           (8000000U)
#endif

#ifndef HSE_STARTUP_TIMEOUT
#define HSE_STARTUP_TIMEOUT (100U)
#endif

#ifndef HSI_VALUE
#define HSI_VALUE           (8000000U)
#endif

#ifndef LSE_VALUE
#define LSE_VALUE           (32768U)
#endif

#ifndef LSE_STARTUP_TIMEOUT
#define LSE_STARTUP_TIMEOUT (5000U)
#endif

#ifndef LSI_VALUE
#define LSI_VALUE           (40000U)
#endif

#define VDD_VALUE           (3300U)
#define TICK_INT_PRIORITY   (0x0FU)
#define USE_RTOS            (0U)
#define PREFETCH_ENABLE     (1U)

#define USE_HAL_ADC_REGISTER_CALLBACKS          (0U)
#define USE_HAL_CAN_REGISTER_CALLBACKS          (0U)
#define USE_HAL_CEC_REGISTER_CALLBACKS          (0U)
#define USE_HAL_DAC_REGISTER_CALLBACKS          (0U)
#define USE_HAL_HCD_REGISTER_CALLBACKS          (0U)
#define USE_HAL_I2C_REGISTER_CALLBACKS          (0U)
#define USE_HAL_I2S_REGISTER_CALLBACKS          (0U)
#define USE_HAL_MMC_REGISTER_CALLBACKS          (0U)
#define USE_HAL_NAND_REGISTER_CALLBACKS         (0U)
#define USE_HAL_NOR_REGISTER_CALLBACKS          (0U)
#define USE_HAL_PCCARD_REGISTER_CALLBACKS       (0U)
#define USE_HAL_PCD_REGISTER_CALLBACKS          (0U)
#define USE_HAL_RTC_REGISTER_CALLBACKS          (0U)
#define USE_HAL_SD_REGISTER_CALLBACKS           (0U)
#define USE_HAL_SMARTCARD_REGISTER_CALLBACKS    (0U)
#define USE_HAL_IRDA_REGISTER_CALLBACKS         (0U)
#define USE_HAL_SRAM_REGISTER_CALLBACKS         (0U)
#define USE_HAL_SPI_REGISTER_CALLBACKS          (0U)
#define USE_HAL_TIM_REGISTER_CALLBACKS          (0U)
#define USE_HAL_UART_REGISTER_CALLBACKS         (0U)
#define USE_HAL_USART_REGISTER_CALLBACKS        (0U)
#define USE_HAL_WWDG_REGISTER_CALLBACKS         (0U)

#define USE_FULL_ASSERT                         (0U)

#define USE_SPI_CRC                             (1U)

#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32f1xx_hal_rcc.h"
#endif

#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32f1xx_hal_gpio.h"
#endif

#ifdef HAL_EXTI_MODULE_ENABLED
  #include "stm32f1xx_hal_exti.h"
#endif

#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32f1xx_hal_dma.h"
#endif

#ifdef HAL_CAN_MODULE_ENABLED
  #include "stm32f1xx_hal_can.h"
#endif

#ifdef HAL_CAN_LEGACY_MODULE_ENABLED
  #include "Legacy/stm32f1xx_hal_can_legacy.h"
#endif

#ifdef HAL_CEC_MODULE_ENABLED
  #include "stm32f1xx_hal_cec.h"
#endif

#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32f1xx_hal_cortex.h"
#endif

#ifdef HAL_ADC_MODULE_ENABLED
  #include "stm32f1xx_hal_adc.h"
#endif

#ifdef HAL_CRC_MODULE_ENABLED
  #include "stm32f1xx_hal_crc.h"
#endif

#ifdef HAL_DAC_MODULE_ENABLED
  #include "stm32f1xx_hal_dac.h"
#endif

#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32f1xx_hal_flash.h"
#endif

#ifdef HAL_SRAM_MODULE_ENABLED
  #include "stm32f1xx_hal_sram.h"
#endif

#ifdef HAL_NOR_MODULE_ENABLED
  #include "stm32f1xx_hal_nor.h"
#endif

#ifdef HAL_I2C_MODULE_ENABLED
  #include "stm32f1xx_hal_i2c.h"
#endif

#ifdef HAL_I2S_MODULE_ENABLED
  #include "stm32f1xx_hal_i2s.h"
#endif

#ifdef HAL_IWDG_MODULE_ENABLED
  #include "stm32f1xx_hal_iwdg.h"
#endif

#ifdef HAL_PWR_MODULE_ENABLED
  #include "stm32f1xx_hal_pwr.h"
#endif

#ifdef HAL_RTC_MODULE_ENABLED
  #include "stm32f1xx_hal_rtc.h"
#endif

#ifdef HAL_PCCARD_MODULE_ENABLED
  #include "stm32f1xx_hal_pccard.h"
#endif

#ifdef HAL_SD_MODULE_ENABLED
  #include "stm32f1xx_hal_sd.h"
#endif

#ifdef HAL_NAND_MODULE_ENABLED
  #include "stm32f1xx_hal_nand.h"
#endif

#ifdef HAL_SPI_MODULE_ENABLED
  #include "stm32f1xx_hal_spi.h"
#endif

#ifdef HAL_TIM_MODULE_ENABLED
  #include "stm32f1xx_hal_tim.h"
#endif

#ifdef HAL_UART_MODULE_ENABLED
  #include "stm32f1xx_hal_uart.h"
#endif

#ifdef HAL_USART_MODULE_ENABLED
  #include "stm32f1xx_hal_usart.h"
#endif

#ifdef HAL_IRDA_MODULE_ENABLED
  #include "stm32f1xx_hal_irda.h"
#endif

#ifdef HAL_SMARTCARD_MODULE_ENABLED
  #include "stm32f1xx_hal_smartcard.h"
#endif

#ifdef HAL_WWDG_MODULE_ENABLED
  #include "stm32f1xx_hal_wwdg.h"
#endif

#ifdef HAL_PCD_MODULE_ENABLED
  #include "stm32f1xx_hal_pcd.h"
#endif

#ifdef HAL_HCD_MODULE_ENABLED
  #include "stm32f1xx_hal_hcd.h"
#endif

#ifdef HAL_MMC_MODULE_ENABLED
  #include "stm32f1xx_hal_mmc.h"
#endif

#include "stm32f1xx_assert.h"

#ifdef __cplusplus
}
#endif

#endif
