/*
 * SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2016 STMicroelectronics.
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

#ifndef I2C_REGISTER_TOOLKIT_DRIVERS_BSP_INCLUDE_BSP_ASSERT_H_
#define I2C_REGISTER_TOOLKIT_DRIVERS_BSP_INCLUDE_BSP_ASSERT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "bsp_error.h"

#ifdef USE_FULL_ASSERT
  #define assert_param(expr)                                          \
    (                                                                 \
      (expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__) \
    )
#else
  #define assert_param(expr)                                          \
    (                                                                 \
      (void)0                                                         \
    )
#endif

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line);
#endif

#ifdef __cplusplus
}
#endif

#endif
