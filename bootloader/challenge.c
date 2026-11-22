/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "main.h"
#include <stdio.h>

/* Both the application and the loader use this service. Do not pulse the
 * AHB2 peripheral reset immediately before writing RNGEN for every HELLO:
 * on the failing board the kernel clock was ready but RNGEN read back zero.
 * Keep the initialized generator running and consume fresh hardware words. */
static const char *next_word(uint32_t *out)
{
  const uint32_t start=HAL_GetTick();
  for(;;) {
    const uint32_t sr=RNG->SR;
    if(sr&(RNG_SR_CECS|RNG_SR_CEIS))return "clock-error";
    if(sr&(RNG_SR_SECS|RNG_SR_SEIS))return "seed-error";
    if(sr&RNG_SR_DRDY){*out=RNG->DR;return NULL;}
    if(HAL_GetTick()-start>=10U)return "no-data";
  }
}

uint64_t fw_challenge(void)
{
  const char *failure="HSI48-not-ready";
  __HAL_RCC_HSI48_ENABLE();
  const uint32_t start=HAL_GetTick();
  while(!(RCC->CR&RCC_CR_HSI48RDY))if(HAL_GetTick()-start>=10U)goto failed;
  __HAL_RCC_RNG_CONFIG(RCC_RNGCLKSOURCE_HSI48);
  __HAL_RCC_RNG_CLK_ENABLE();
  for(unsigned attempt=0;attempt<2;attempt++){
    uint32_t first,second,discard;
    if(attempt){
      __HAL_RCC_RNG_FORCE_RESET();
      (void)RCC->AHB2RSTR;__DSB();
      HAL_Delay(1);
      __HAL_RCC_RNG_RELEASE_RESET();
      (void)RCC->AHB2RSTR;__DSB();
      HAL_Delay(1);
    }
    if(!(RNG->CR&RNG_CR_RNGEN)){
      RNG->CR=RNG_CR_RNGEN; /* clock-error detection remains enabled */
      __DSB();
      if(!(RNG->CR&RNG_CR_RNGEN)){failure="enable-not-latched";continue;}
      failure=next_word(&discard);if(failure)continue;
    }
    failure=next_word(&first);if(failure)continue;
    failure=next_word(&second);if(failure)continue;
    if(first==second){failure="repeated-word";continue;}
    return ((uint64_t)first<<32)|second;
  }
failed:
  {
    static uint32_t logged;static uint8_t once;
    const uint32_t now=HAL_GetTick();
    if(!once || now-logged>=1000U){
      once=1;logged=now;
      printf("NETBOOT: RNG %s; RCC_CR=%08lx RNGSEL=%08lx ENR=%08lx RNG_CR=%08lx SR=%08lx\r\n",
        failure,(unsigned long)RCC->CR,(unsigned long)RCC->D2CCIP2R,(unsigned long)RCC->AHB2ENR,
        (unsigned long)((RCC->AHB2ENR&RCC_AHB2ENR_RNGEN)?RNG->CR:0),
        (unsigned long)((RCC->AHB2ENR&RCC_AHB2ENR_RNGEN)?RNG->SR:0));
    }
  }
  return 0; /* Never substitute a timer, fixed session or host-supplied nonce. */
}
