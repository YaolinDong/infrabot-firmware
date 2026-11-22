/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#pragma once
#include <stdint.h>
#include <stddef.h>
typedef struct {uint32_t CR, D2CCIP2R, AHB2ENR, AHB2RSTR;} MockRcc;
typedef struct {uint32_t CR, SR, DR;} MockRng;
extern MockRcc rcc;
extern MockRng rng;
#define RCC (&rcc)
#define RNG (&rng)
#define RCC_CR_HSI48RDY 0x2000U
#define RCC_AHB2ENR_RNGEN 64U
#define RCC_RNGCLKSOURCE_HSI48 0U
#define RNG_CR_RNGEN 4U
#define RNG_SR_DRDY 1U
#define RNG_SR_CECS 2U
#define RNG_SR_SECS 4U
#define RNG_SR_CEIS 32U
#define RNG_SR_SEIS 64U
void osc(void);
void barrier(void);
void reset_rng(void);
void HAL_Delay(unsigned);
uint32_t HAL_GetTick(void);
#define __DSB() barrier()
#define __HAL_RCC_HSI48_ENABLE() osc()
#define __HAL_RCC_RNG_CONFIG(v) (rcc.D2CCIP2R=(v))
#define __HAL_RCC_RNG_CLK_ENABLE() (rcc.AHB2ENR|=64)
#define __HAL_RCC_RNG_FORCE_RESET() reset_rng()
#define __HAL_RCC_RNG_RELEASE_RESET() (rcc.AHB2RSTR=0)
