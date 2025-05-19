/*
 * SPDX-License-Identifier: BSD-3-Clause
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

#ifndef I2C_REGISTER_TOOLKIT_APPLICATION_INCLUDE_SWI2C_H_
#define I2C_REGISTER_TOOLKIT_APPLICATION_INCLUDE_SWI2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"

#define SWI2C_SCL_PORT  GPIOA
#define SWI2C_SCL_PIN   GPIO_PIN_0
#define SWI2C_SDA_PORT  GPIOA
#define SWI2C_SDA_PIN   GPIO_PIN_1

#define SWI2C_DELAY()   Delay5us()

#ifndef SWI2C_IGNORE_SLAVE_ACK_MODE
  #define SWI2C_IGNORE_SLAVE_ACK_MODE (0U)
#endif

typedef enum {
  SWI2C_SUCCESS = 0,
  SWI2C_ERROR = 1
} SWI2C_StateTypeDef;

typedef enum {
  SWI2C_LL_LOW = 0,
  SWI2C_LL_HIGH = 1
} SWI2C_LLStateTypeDef;

typedef enum {
  SWI2C_ACK = 0,
  SWI2C_NACK = 1
} SWI2C_AckBitStateTypeDef;

__STATIC_FORCEINLINE void SWI2C_SetPin(GPIO_TypeDef *GPIO, uint16_t GPIO_Pin)
{
  GPIO->BSRR = GPIO_Pin;
}

__STATIC_FORCEINLINE void SWI2C_ResetPin(GPIO_TypeDef *GPIO, uint16_t GPIO_Pin)
{
  GPIO->BRR = GPIO_Pin;
}

__STATIC_FORCEINLINE SWI2C_LLStateTypeDef SWI2C_ReadPin(GPIO_TypeDef *GPIO, uint16_t GPIO_Pin)
{
  return (GPIO->IDR & GPIO_Pin) ? SWI2C_LL_HIGH : SWI2C_LL_LOW;
}

void SWI2C_Init(void);

void SWI2C_Start(void);
void SWI2C_Stop(void);

SWI2C_AckBitStateTypeDef SWI2C_GetAckBit(void);
void SWI2C_TransmitAckBit(void);
void SWI2C_TransmitNAckBit(void);

SWI2C_StateTypeDef SWI2C_TransmitBytes(const uint8_t *pdata, const uint32_t num);
void SWI2C_ReceiveBytes(uint8_t *pdata, const uint32_t num);

SWI2C_StateTypeDef SWI2C_Read(const uint8_t device_addr, const uint8_t data_start_addr, uint8_t *const pbuffer, const uint32_t num);
SWI2C_StateTypeDef SWI2C_Write(const uint8_t device_addr, const uint8_t data_start_addr, const uint8_t *const pbuffer, const uint32_t num);

#ifdef __cplusplus
}
#endif

#endif
