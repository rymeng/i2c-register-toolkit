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

#include "swi2c.h"

void SWI2C_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = SWI2C_SCL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SWI2C_SCL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SWI2C_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SWI2C_SDA_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(SWI2C_SCL_PORT, SWI2C_SCL_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SWI2C_SDA_PORT, SWI2C_SDA_PIN, GPIO_PIN_SET);
}

void SWI2C_Start(void)
{
  SWI2C_ResetPin(SWI2C_SCL_PORT, SWI2C_SCL_PIN);
  SWI2C_SetPin(SWI2C_SDA_PORT, SWI2C_SDA_PIN);
  SWI2C_DELAY();
  SWI2C_SetPin(SWI2C_SCL_PORT, SWI2C_SCL_PIN);
  SWI2C_ResetPin(SWI2C_SDA_PORT, SWI2C_SDA_PIN);
  SWI2C_DELAY();
  SWI2C_ResetPin(SWI2C_SCL_PORT, SWI2C_SCL_PIN);
}

void SWI2C_Stop(void)
{
  SWI2C_ResetPin(SWI2C_SCL_PORT, SWI2C_SCL_PIN);
  SWI2C_ResetPin(SWI2C_SDA_PORT, SWI2C_SDA_PIN);
  SWI2C_DELAY();
  SWI2C_SetPin(SWI2C_SCL_PORT, SWI2C_SCL_PIN);
  SWI2C_SetPin(SWI2C_SDA_PORT, SWI2C_SDA_PIN);
}

SWI2C_AckBitStateTypeDef SWI2C_GetAckBit(void)
{
  SWI2C_ResetPin(SWI2C_SCL_PORT, SWI2C_SCL_PIN);
  SWI2C_SetPin(SWI2C_SDA_PORT, SWI2C_SDA_PIN);
  SWI2C_DELAY();
  SWI2C_SetPin(SWI2C_SCL_PORT, SWI2C_SCL_PIN);
  SWI2C_LLStateTypeDef sda_ll_state = SWI2C_ReadPin(SWI2C_SDA_PORT, SWI2C_SDA_PIN);
  SWI2C_DELAY();
  SWI2C_ResetPin(SWI2C_SCL_PORT, SWI2C_SCL_PIN);

  if (sda_ll_state == SWI2C_LL_LOW)
  {
    return SWI2C_ACK;
  }
  else
  {
    return SWI2C_NACK;
  }
}

void SWI2C_TransmitAckBit(void)
{
  SWI2C_ResetPin(SWI2C_SCL_PORT, SWI2C_SCL_PIN);
  SWI2C_ResetPin(SWI2C_SDA_PORT, SWI2C_SDA_PIN);
  SWI2C_DELAY();
  SWI2C_SetPin(SWI2C_SCL_PORT, SWI2C_SCL_PIN);
  SWI2C_DELAY();
  SWI2C_ResetPin(SWI2C_SCL_PORT, SWI2C_SCL_PIN);
}

void SWI2C_TransmitNAckBit(void)
{
  SWI2C_ResetPin(SWI2C_SCL_PORT, SWI2C_SCL_PIN);
  SWI2C_SetPin(SWI2C_SDA_PORT, SWI2C_SDA_PIN);
  SWI2C_DELAY();
  SWI2C_SetPin(SWI2C_SCL_PORT, SWI2C_SCL_PIN);
  SWI2C_DELAY();
  SWI2C_ResetPin(SWI2C_SCL_PORT, SWI2C_SCL_PIN);
}

SWI2C_StateTypeDef SWI2C_TransmitBytes(const uint8_t *pdata, const uint32_t num)
{
  for (uint32_t i = 0U; i < num; ++i)
  {
    for (uint8_t j = 0x00U; j < 0x08U; ++j)
    {
      SWI2C_ResetPin(SWI2C_SCL_PORT, SWI2C_SCL_PIN);
      uint8_t data_mask = (uint8_t)0x80U >> j;
      if (*pdata & data_mask)
      {
        SWI2C_SetPin(SWI2C_SDA_PORT, SWI2C_SDA_PIN);
      }
      else
      {
        SWI2C_ResetPin(SWI2C_SDA_PORT, SWI2C_SDA_PIN);
      }
      SWI2C_DELAY();
      SWI2C_SetPin(SWI2C_SCL_PORT, SWI2C_SCL_PIN);
      SWI2C_DELAY();
    }
    SWI2C_AckBitStateTypeDef ack_bit = SWI2C_GetAckBit();
#if SWI2C_IGNORE_SLAVE_ACK_MODE == 0U
    if (ack_bit == SWI2C_NACK)
    {
      return SWI2C_ERROR;
    }
#else
    UNUSED(ack_bit);
#endif
    ++pdata;
  }
  return SWI2C_SUCCESS;
}

void SWI2C_ReceiveBytes(uint8_t *pdata, const uint32_t num)
{
  for (uint32_t i = 0U; i < num; ++i)
  {
    *pdata = 0x00U;
    for (uint8_t j = 0x00U; j < 0x08U; ++j)
    {
      SWI2C_ResetPin(SWI2C_SCL_PORT, SWI2C_SCL_PIN);
      SWI2C_SetPin(SWI2C_SDA_PORT, SWI2C_SDA_PIN);
      SWI2C_DELAY();
      SWI2C_SetPin(SWI2C_SCL_PORT, SWI2C_SCL_PIN);
      SWI2C_LLStateTypeDef sda_ll_state = SWI2C_ReadPin(SWI2C_SDA_PORT, SWI2C_SDA_PIN);
      SWI2C_DELAY();
      uint8_t data_mask = (uint8_t)0x80U >> j;
      if (sda_ll_state == SWI2C_LL_HIGH)
      {
        *pdata |= data_mask;
      }
    }
    if (i == num - 1)
    {
      SWI2C_TransmitNAckBit();
    }
    else
    {
      SWI2C_TransmitAckBit();
    }
    ++pdata;
  }
}

SWI2C_StateTypeDef SWI2C_Read(const uint8_t device_addr, const uint8_t data_start_addr, uint8_t *const pbuffer, const uint32_t num)
{
  SWI2C_StateTypeDef state = SWI2C_SUCCESS;
  uint8_t device_addr_framedata_read = (device_addr << 1) | 0x01U;
  uint8_t device_addr_framedata_write = (device_addr << 1) | 0x00U;

  SWI2C_Start();
  state = SWI2C_TransmitBytes(&device_addr_framedata_write, 1U);
  if (state == SWI2C_ERROR)
  {
    SWI2C_Stop();
    return state;
  }
  state = SWI2C_TransmitBytes(&data_start_addr, 1U);
  if (state == SWI2C_ERROR)
  {
    SWI2C_Stop();
    return state;
  }
  SWI2C_Start();
  state = SWI2C_TransmitBytes(&device_addr_framedata_read, 1U);
  if (state == SWI2C_ERROR)
  {
    SWI2C_Stop();
    return state;
  }
  SWI2C_ReceiveBytes(pbuffer, num);
  SWI2C_Stop();
  return state;
}

SWI2C_StateTypeDef SWI2C_Write(const uint8_t device_addr, const uint8_t data_start_addr, const uint8_t *const pbuffer, const uint32_t num)
{
  SWI2C_StateTypeDef state = SWI2C_SUCCESS;
  uint8_t device_addr_framedata_write = (device_addr << 1) | 0x00U;

  SWI2C_Start();
  state = SWI2C_TransmitBytes(&device_addr_framedata_write, 1U);
  if (state == SWI2C_ERROR)
  {
    SWI2C_Stop();
    return state;
  }
  state = SWI2C_TransmitBytes(&data_start_addr, 1U);
  if (state == SWI2C_ERROR)
  {
    SWI2C_Stop();
    return state;
  }
  state = SWI2C_TransmitBytes(pbuffer, num);
  SWI2C_Stop();
  return state;
}
