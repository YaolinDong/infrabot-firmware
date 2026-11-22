/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "main.h"
#include "slots.h"
#ifdef FW_BOOT_SCAN_PLL_192
#include "scan_clock.h"
#endif
#include "app_sdram.h"
#include "app_startup.h"
#include "app_persist.h"
#include "app_config.h"
#include "ethernetif.h"
#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include <string.h>
#include <stdio.h>
extern UART_HandleTypeDef huart1;
void Netboot_UartInit(void);
int __io_putchar(int ch){uint8_t c=(uint8_t)ch;HAL_UART_Transmit(&huart1,&c,1,10);return ch;}

void SysTick_Handler(void){HAL_IncTick();}
void ETH_IRQHandler(void){HAL_ETH_IRQHandler(&EthHandle);}
void AppEthernet_NotifyRxFromISR(void){} /* LwIP is polled */
static volatile unsigned checking_slot,skipped;
static unsigned watchdog;
static void watchdog_start(void)
{
  IWDG1->KR=0xcccc;IWDG1->KR=0x5555;
  IWDG1->PR=6;IWDG1->RLR=4095;
  while(IWDG1->SR){} /* ~32 s at nominal 32 kHz LSI */
  IWDG1->KR=0xaaaa;watchdog=1;
}
void fw_progress(void){if(watchdog)IWDG1->KR=0xaaaa;}
void HardFault_Handler(void)
{
  if(checking_slot){fw_mail(FW_FAULT_REQ,skipped|(1U<<(checking_slot-1)),0);NVIC_SystemReset();}
  for(;;){} /* recovery watchdog resets to confirmed app */
}
void fw_guard(unsigned slot){checking_slot=slot;}
static int rollback_keys(void)
{
  /* KEY2 alone must be held for three seconds across reset. SysTick polling
   * here is before HAL/scheduler and does not enable an interrupt. */
  if(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)||HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0))return 0;
  SysTick->LOAD=SystemCoreClock/1000U-1U;SysTick->VAL=0;
  SysTick->CTRL=SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_ENABLE_Msk;
  unsigned ms=0;
  while(ms<3000 && HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) && !HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0))
    if(SysTick->CTRL&SysTick_CTRL_COUNTFLAG_Msk)++ms;
  SysTick->CTRL=0;SysTick->LOAD=0;SysTick->VAL=0;
  return ms==3000;
}
void SystemClock_Config(void);
int Netboot_Run(void)
{
  uint32_t reason=0,slot=0,gen=0;
  const uint32_t reset_flags=RCC->RSR;
  if((reset_flags&RCC_RSR_SFTRSTF) && !(reset_flags&(RCC_RSR_PORRSTF|RCC_RSR_BORRSTF|RCC_RSR_IWDG1RSTF|RCC_RSR_WWDG1RSTF)) &&
      FW_MAILBOX[1]==~FW_MAILBOX[0]){
    reason=FW_MAILBOX[0];slot=FW_MAILBOX[2];gen=FW_MAILBOX[3];
  }
  /* Keep the ticket only for the permitted transition into maintenance or
   * the explicit post-install trial. A cold reset, expiry or physical SWD
   * install cannot manufacture an approved network transfer. */
  if(reason!=FW_REQUEST_MAGIC && reason!=FW_INSTALLED_REQ)FW_TICKET->cookie=0;
  FW_MAILBOX[0]=FW_MAILBOX[1]=0;
  __HAL_RCC_CLEAR_RESET_FLAGS();
  __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef input={.Mode=GPIO_MODE_INPUT,.Pull=GPIO_PULLDOWN,.Pin=GPIO_PIN_0};
  HAL_GPIO_Init(GPIOA,&input);input.Pin=GPIO_PIN_13;HAL_GPIO_Init(GPIOC,&input);
  const int key2=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13);
  const int keys=key2 && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
  /* Checking another slot after an ECC fault must not reread the torn one.
   * A real power cycle retries both; normal app data cannot select an arbitrary address. */
  skipped=reason==FW_FAULT_REQ?(slot&3U):0;
  /* Keep the recovery path independent of the optional scan PLL. Software
   * reset (including clock-restore failure), watchdog, update and key boots
   * retain the previous HSI-only path. Payload and markers stay uncached. */
  SCB_EnableICache();
  CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
  DWT->LAR=0xC5ACCE55U; /* CoreSight software unlock on a cold, debugger-free M7 */
  DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
  uint32_t scan_hz=SystemCoreClock; /* reset HSI at 64 MHz in the default build */
#ifdef FW_BOOT_SCAN_PLL_192
  struct fw_scan_clock clock={0};
  const uint32_t slow_reset=RCC_RSR_SFTRSTF | RCC_RSR_IWDG1RSTF | RCC_RSR_WWDG1RSTF;
  if(!reason && !key2 && !(reset_flags & slow_reset)) scan_hz=fw_scan_clock_enter(&clock);
#endif
  uint32_t scan_start=DWT->CYCCNT;
  unsigned images=fw_scan_slots(skipped,!reason && !key2);
  uint32_t scan_cycles=DWT->CYCCNT-scan_start;
#ifdef FW_BOOT_SCAN_PLL_192
  fw_scan_clock_leave(&clock);
#endif
  SCB_DisableICache(); /* restore reset state before app or flash maintenance */
  int known=fw_known();
  if(key2 && !keys && !reason && known>=0 && rollback_keys()){
    reason=FW_REVERT_REQ;slot=fw_slots[known].base;gen=fw_slots[known].gen;
  }
  int trial=0;
  const int selected=fw_select_start(reason,slot,gen,keys,&trial);
  if(selected>=0){
    FW_BOOT_STATS[0]=scan_cycles/(scan_hz/1000U);
    FW_BOOT_STATS[1]=images;
    FW_BOOT_STATS[2]=~FW_BOOT_STATS_MAGIC;
    FW_BOOT_STATS[3]=FW_BOOT_STATS_MAGIC;
    if(!trial)FW_TICKET->cookie=0;
    FwSlot *s=&fw_slots[selected];uint32_t vector=s->base+1024U;
    if(trial){fw_mail(FW_ATTEMPT,s->base,s->gen);watchdog_start();}
    const uint32_t *v=(const void*)(uintptr_t)vector;
    SCB->VTOR=vector;__DSB();__ISB();
    __asm volatile("msr msp,%0\n bx %1"::"r"(v[0]),"r"(v[1]):"memory");
    __builtin_unreachable();
  }
  /* Link/power loss or a loader hang cannot strand the old application.
   * Only progress in the locally approved upload refreshes the guard. */
  if(fw_active_slot>=0) watchdog_start();
  AppStartup_ConfigEthernetMemory();AppSdram_ConfigMpu();
  HAL_Init();SystemClock_Config();Netboot_UartInit();
  printf("NETBOOT: maintenance, confirmed slot=%d, upload base=0x%08lx\r\n",fw_active_slot,(unsigned long)fw_destination());
  if(!AppSdram_Init())Error_Handler();
  lwip_init();
  AppNetConfig cfg={{DM_ETH_IP_ADDR0,DM_ETH_IP_ADDR1,DM_ETH_IP_ADDR2,DM_ETH_IP_ADDR3},
    {DM_ETH_NETMASK0,DM_ETH_NETMASK1,DM_ETH_NETMASK2,DM_ETH_NETMASK3},
    {DM_ETH_GW_ADDR0,DM_ETH_GW_ADDR1,DM_ETH_GW_ADDR2,DM_ETH_GW_ADDR3}};
  (void)AppPersist_LoadNetwork(&cfg);
  ip4_addr_t ip,mask,gw;
  IP4_ADDR(&ip,cfg.ip[0],cfg.ip[1],cfg.ip[2],cfg.ip[3]);
  IP4_ADDR(&mask,cfg.netmask[0],cfg.netmask[1],cfg.netmask[2],cfg.netmask[3]);
  IP4_ADDR(&gw,cfg.gateway[0],cfg.gateway[1],cfg.gateway[2],cfg.gateway[3]);
  static struct netif net;
  if(!netif_add(&net,&ip,&mask,&gw,0,ethernetif_init,ethernet_input))Error_Handler();
  netif_set_default(&net);
  if(!fw_service_start())Error_Handler();
  uint32_t link=HAL_GetTick();
  for(;;){
    if(HAL_GetTick()-link>=100){link=HAL_GetTick();ethernet_link_check_state(&net);}
    ethernetif_input(&net);sys_check_timeouts();
  }
}
