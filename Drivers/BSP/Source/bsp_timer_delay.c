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

#include "bsp_timer_delay.h"

TIM_HandleTypeDef htim2 = {0};
TIM_HandleTypeDef *htim_delay5us = &htim2;

void BSP_TIM_Delay5us_Init(void)
{
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0U;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0U;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(htim_delay5us);

  return;
}

void BSP_Delay5us(void)
{
  __HAL_TIM_SET_COUNTER(htim_delay5us, 0U);
  __HAL_TIM_SET_AUTORELOAD(htim_delay5us, 100U - 1U);

  HAL_TIM_Base_Start(htim_delay5us);
  while (!__HAL_TIM_GET_FLAG(htim_delay5us, TIM_FLAG_UPDATE)) {
    ;
  }
  __HAL_TIM_CLEAR_FLAG(htim_delay5us, TIM_FLAG_UPDATE);
  HAL_TIM_Base_Stop(htim_delay5us);

  return;
}
