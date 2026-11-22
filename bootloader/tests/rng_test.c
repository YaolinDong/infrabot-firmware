/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "main.h"
#include <assert.h>
#include <stdio.h>

MockRcc rcc;MockRng rng;
static uint32_t tick,sample,status;
static unsigned resets,lost_enable,stuck_clock,repeat;
void reset_rng(void){rcc.AHB2RSTR=64;rng=(MockRng){0};++resets;}
void osc(void){if(!stuck_clock)rcc.CR|=RCC_CR_HSI48RDY;}
void barrier(void){if(lost_enable && rng.CR){rng.CR=0;--lost_enable;}}
void HAL_Delay(unsigned n){tick+=n;}
uint32_t HAL_GetTick(void){
  if(rng.CR&RNG_CR_RNGEN){rng.SR=status;rng.DR=repeat?123:++sample;}
  return tick++;
}
uint64_t fw_challenge(void);
int main(void){
  status=RNG_SR_DRDY;
  uint64_t a=fw_challenge(),b=fw_challenge();
  assert(a && b && a!=b && !resets);  /* no reset in normal HELLO */
  rng.CR=0;lost_enable=1;assert(fw_challenge() && resets==1);
  lost_enable=2;rng.CR=0;assert(!fw_challenge());
  status=0;assert(!fw_challenge()); /* bounded when DRDY never asserts */
  status=RNG_SR_DRDY|RNG_SR_SECS;assert(!fw_challenge());
  status=RNG_SR_DRDY|RNG_SR_CEIS;assert(!fw_challenge());
  status=RNG_SR_DRDY;repeat=1;assert(!fw_challenge());
  stuck_clock=1;rcc.CR=0;assert(!fw_challenge());
  puts("RNG init: fresh samples, lost enable retry, timeouts and fail-closed health errors passed");
}
