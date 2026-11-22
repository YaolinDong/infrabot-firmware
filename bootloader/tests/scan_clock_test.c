/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Native model of lock/switch acknowledgement with the actual H743 CMSIS bit definitions. */
#include "main.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

static RCC_TypeDef rcc;
static FLASH_TypeDef flash;
static PWR_TypeDef pwr;
#undef RCC
#undef FLASH
#undef PWR
#define RCC (&rcc)
#define FLASH (&flash)
#define PWR (&pwr)
uint32_t SystemCoreClock=HSI_VALUE;
static enum { OK,NO_LOCK,NO_SWITCH,STUCK_ON_PLL } fault;
static jmp_buf reset_target;
#undef NVIC_SystemReset
#define NVIC_SystemReset() longjmp(reset_target,1)
#define __DSB() ((void)0)
#define __ISB() ((void)0)

static void poll_clock(void)
{
    if ((RCC->CR & RCC_CR_PLL1ON) && fault!=NO_LOCK) RCC->CR |= RCC_CR_PLL1RDY;
    else RCC->CR &= ~RCC_CR_PLL1RDY;
    if ((RCC->CFGR & RCC_CFGR_SW)==RCC_CFGR_SW_PLL1) {
        assert((RCC->D1CFGR&RCC_D1CFGR_HPRE)==RCC_D1CFGR_HPRE_DIV4);
        assert(RCC->CR&RCC_CR_PLL1RDY);
        if(fault!=NO_SWITCH) MODIFY_REG(RCC->CFGR,RCC_CFGR_SWS,RCC_CFGR_SWS_PLL1);
    } else if(fault!=STUCK_ON_PLL) MODIFY_REG(RCC->CFGR,RCC_CFGR_SWS,0);
}
#define FW_CLOCK_POLL() poll_clock()
#include "../scan_clock.c"

int main(void)
{
    RCC->CR=RCC_CR_HSION|RCC_CR_HSIRDY; RCC->PLL1DIVR=0x1010280;
    RCC->PLLCKSELR=0x02020200; RCC->PLLCFGR=0x01ff0000;
    PWR->CR3=PWR_CR3_LDOEN; PWR->CSR1=PWR_CSR1_ACTVOSRDY|PWR_CSR1_ACTVOS_0;
    FLASH->ACR=FLASH_LATENCY_DEFAULT;
    RCC_TypeDef saved; memcpy(&saved,RCC,sizeof saved);
    struct fw_scan_clock st;
    for(fault=OK;fault<=NO_SWITCH;fault++){
        assert(fw_scan_clock_enter(&st)==(fault==OK?192000000U:HSI_VALUE));
        if(fault==OK){
            assert((RCC->PLLCKSELR & RCC_PLLCKSELR_PLLSRC)==RCC_PLLSOURCE_HSI);
            assert(((RCC->PLLCKSELR & RCC_PLLCKSELR_DIVM1)>>4U)==10U);
            assert((RCC->PLL1DIVR & RCC_PLL1DIVR_N1)==59U);
            assert((RCC->PLL1DIVR & RCC_PLL1DIVR_P1)==(1U<<9));
            assert((RCC->PLLCFGR & PLL_CFG_BITS)==(RCC_PLL1VCIRANGE_2|RCC_PLLCFGR_DIVP1EN));
        }
        fw_scan_clock_leave(&st);
        assert(!memcmp(&saved,RCC,sizeof saved));
    }
    RCC->CR|=RCC_CR_PLL2ON;
    assert(fw_scan_clock_enter(&st)==HSI_VALUE && !st.changed);
    fw_scan_clock_leave(&st);assert(RCC->CR==(saved.CR|RCC_CR_PLL2ON));
    RCC->CR=saved.CR;FLASH->ACR=0;
    assert(fw_scan_clock_enter(&st)==HSI_VALUE && !st.changed);
    assert(!memcmp(&saved,RCC,sizeof saved));FLASH->ACR=FLASH_LATENCY_DEFAULT;
    fault=STUCK_ON_PLL;
    assert(fw_scan_clock_enter(&st)==192000000U);
    if(!setjmp(reset_target)){fw_scan_clock_leave(&st);assert(0);}
    assert((RCC->D1CFGR&RCC_D1CFGR_HPRE)==RCC_D1CFGR_HPRE_DIV4);
    puts("Scan clock: normal, PLL lock/switch timeout, unexpected entry, HSI restore failure passed");
}
