/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "main.h"
#include "scan_clock.h"

/* H743 at its existing, stable LDO voltage (even VOS3 permits a 200 MHz
 * CPU). Internal HSI only: 64/10 * 60 / 2 = 192 MHz CPU, AHB /4 = 48 MHz.
 * No supply/VOS change, overdrive, HSE, D-cache, Flash ACR or interrupt.
 * The loader's clock globals stay at their reset values: no HAL tick or
 * peripheral that consults them may run inside this scope. */
_Static_assert(HSI_VALUE == 64000000U, "Scan PLL requires the H743 64 MHz HSI");
#define SCAN_HZ 192000000U
#define PLL_CFG_BITS (RCC_PLLCFGR_PLL1FRACEN | RCC_PLLCFGR_PLL1VCOSEL | RCC_PLLCFGR_PLL1RGE | \
                      RCC_PLLCFGR_DIVP1EN | RCC_PLLCFGR_DIVQ1EN | RCC_PLLCFGR_DIVR1EN)

#ifndef FW_CLOCK_POLL
#define FW_CLOCK_POLL() ((void)0)
#endif

static int wait_bits(volatile uint32_t *reg, uint32_t mask, uint32_t expected)
{
    /* Also terminates when a debugger locks CYCCNT or SysTick is off. */
    for (unsigned i=0; i<65536U; i++) {
        FW_CLOCK_POLL();
        if ((*reg & mask)==expected) return 1;
    }
    return 0;
}

void fw_scan_clock_leave(struct fw_scan_clock *s)
{
    if (!s->changed) return;
    /* Keep /4 until SWS confirms HSI, including a late PLL switch. */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI);
    if (!wait_bits(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_HSI)) NVIC_SystemReset();
    CLEAR_BIT(RCC->CR, RCC_CR_PLL1ON);
    if (!wait_bits(&RCC->CR, RCC_CR_PLL1RDY, 0)) NVIC_SystemReset();
    RCC->PLLCKSELR=s->pll_source;
    RCC->PLL1DIVR=s->pll_div;
    RCC->PLLCFGR=s->pll_cfg;
    RCC->D1CFGR=s->d1;
    __DSB(); __ISB();
    s->changed=0;
}

uint32_t fw_scan_clock_enter(struct fw_scan_clock *s)
{
    s->changed=0;
    const uint32_t plls=RCC_CR_PLL1ON | RCC_CR_PLL1RDY | RCC_CR_PLL2ON | RCC_CR_PLL2RDY |
                        RCC_CR_PLL3ON | RCC_CR_PLL3RDY;
    if (SystemCoreClock!=HSI_VALUE || (RCC->CR & (plls | RCC_CR_HSIDIV | RCC_CR_HSIRDY))!=RCC_CR_HSIRDY ||
        (RCC->CFGR & (RCC_CFGR_SW | RCC_CFGR_SWS))!=0 ||
        (RCC->D1CFGR & (RCC_D1CFGR_D1CPRE | RCC_D1CFGR_HPRE))!=0 ||
        (PWR->CR3 & (PWR_CR3_LDOEN | PWR_CR3_BYPASS))!=PWR_CR3_LDOEN ||
        !(PWR->CSR1 & PWR_CSR1_ACTVOSRDY) ||
        (FLASH->ACR & FLASH_ACR_LATENCY)!=FLASH_LATENCY_DEFAULT)
        return SystemCoreClock;

    s->pll_source=RCC->PLLCKSELR; s->pll_div=RCC->PLL1DIVR;
    s->pll_cfg=RCC->PLLCFGR; s->d1=RCC->D1CFGR;
    s->changed=1;
    MODIFY_REG(RCC->D1CFGR,RCC_D1CFGR_HPRE,RCC_D1CFGR_HPRE_DIV4);
    __DSB();
    if ((RCC->D1CFGR & RCC_D1CFGR_HPRE)!=RCC_D1CFGR_HPRE_DIV4) goto slow;
    __HAL_RCC_PLL_CONFIG(RCC_PLLSOURCE_HSI,10U,60U,2U,2U,2U);
    MODIFY_REG(RCC->PLLCFGR, PLL_CFG_BITS, RCC_PLL1VCIRANGE_2 | RCC_PLL1VCOWIDE | RCC_PLLCFGR_DIVP1EN);
    SET_BIT(RCC->CR, RCC_CR_PLL1ON);
    if (!wait_bits(&RCC->CR, RCC_CR_PLL1RDY, RCC_CR_PLL1RDY)) goto slow;
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL1);
    if (!wait_bits(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL1)) goto slow;
    __DSB(); __ISB();
    return SCAN_HZ;

slow:
    fw_scan_clock_leave(s);
    return SystemCoreClock;
}
