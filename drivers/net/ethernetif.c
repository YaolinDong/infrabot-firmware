/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "stm32h7xx_hal.h"
#include "ethernetif.h"
#include "app_ethernet.h"

#include "lwip/init.h"
#include "lwip/memp.h"
#include "lwip/pbuf.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "lan8742.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define ETH_RX_BUFFER_SIZE 1536U
#define ETH_RX_BUFFER_CNT  8U
#define ETH_DMA_TIMEOUT    50U

/* LAN8720A is strapped to address 0 on the Wildfire H743 Challenger board.
 * Do not use LAN8742_Init()'s generic 0..31 scan here: while the PHY is still
 * powering up, an empty MDIO bus reads as 0xffff and address 31 then looks
 * like a valid SMR address.  That creates a permanent false link-up. */
#define ETH_PHY_BOARD_ADDR       0U
#define ETH_PHY_ID1_EXPECTED     0x0007U
#define ETH_PHY_ID2_MASK         0xFFF0U
#define ETH_PHY_ID2_EXPECTED     0xC0F0U

typedef struct
{
  struct pbuf_custom pbuf;
  uint8_t data[(ETH_RX_BUFFER_SIZE + 31U) & ~31U] __ALIGNED(32);
  uint8_t in_use;
} EthRxBuffer_t;

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]
  __attribute__((section(".RxDescripSection"), aligned(32)));
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]
  __attribute__((section(".TxDescripSection"), aligned(32)));
static EthRxBuffer_t RxPool[ETH_RX_BUFFER_CNT]
  __attribute__((section(".RxArraySection"), aligned(32)));

ETH_HandleTypeDef EthHandle;
volatile uint32_t ethernetif_rx_packets;
volatile uint32_t ethernetif_rx_alloc_errors;
volatile uint32_t ethernetif_tx_packets;
volatile uint32_t ethernetif_tx_errors;
static lan8742_Object_t Lan8742;
static lan8742_IOCtx_t Lan8742IO;
static uint8_t EthPhyConfigured;

/* Keep Ethernet MSP ownership beside this custom ethernetif.  CubeMX creates
 * a second MSP implementation in LWIP/Target/ethernetif.c, but that generated
 * driver is intentionally excluded from this project's CMake target. */
void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
  GPIO_InitTypeDef gpio = {0};

  if ((heth == NULL) || (heth->Instance != ETH)) return;

  __HAL_RCC_ETH1MAC_CLK_ENABLE();
  __HAL_RCC_ETH1TX_CLK_ENABLE();
  __HAL_RCC_ETH1RX_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF11_ETH;

  /* PA1 REF_CLK, PA2 MDIO, PA7 CRS_DV.  LAN8720A supplies REF_CLK. */
  gpio.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_11;                    /* PB11 TX_EN */
  HAL_GPIO_Init(GPIOB, &gpio);
  gpio.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5; /* PC1 MDC, PC4/5 RXD */
  HAL_GPIO_Init(GPIOC, &gpio);
  gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14;      /* PG13/14 TXD0/1 */
  HAL_GPIO_Init(GPIOG, &gpio);

  /* Priority 6 is below configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY in
   * urgency, so the RX callback may safely notify a FreeRTOS task. */
  HAL_NVIC_SetPriority(ETH_IRQn, 6U, 0U);
  HAL_NVIC_EnableIRQ(ETH_IRQn);
}

void HAL_ETH_MspDeInit(ETH_HandleTypeDef *heth)
{
  if ((heth == NULL) || (heth->Instance != ETH)) return;

  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7);
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);
  HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5);
  HAL_GPIO_DeInit(GPIOG, GPIO_PIN_13 | GPIO_PIN_14);
  HAL_NVIC_DisableIRQ(ETH_IRQn);
  __HAL_RCC_ETH1MAC_CLK_DISABLE();
  __HAL_RCC_ETH1TX_CLK_DISABLE();
  __HAL_RCC_ETH1RX_CLK_DISABLE();
}

static void EthCacheInvalidate(void *address, uint32_t length)
{
  uintptr_t start;
  uintptr_t end;

  if ((address == NULL) || (length == 0U)) return;
  start = (uintptr_t)address & ~(uintptr_t)31U;
  end = ((uintptr_t)address + (uintptr_t)length + 31U) & ~(uintptr_t)31U;
  SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
}

static void EthCacheClean(const void *address, uint32_t length)
{
  uintptr_t start;
  uintptr_t end;

  if ((address == NULL) || (length == 0U)) return;
  start = (uintptr_t)address & ~(uintptr_t)31U;
  end = ((uintptr_t)address + (uintptr_t)length + 31U) & ~(uintptr_t)31U;
  SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
}

static EthRxBuffer_t *EthRx_FromData(uint8_t *data)
{
  for (uint32_t i = 0U; i < ETH_RX_BUFFER_CNT; i++)
    if (RxPool[i].data == data) return &RxPool[i];
  return NULL;
}

static EthRxBuffer_t *EthRx_Alloc(void)
{
  for (uint32_t i = 0U; i < ETH_RX_BUFFER_CNT; i++)
  {
    if (RxPool[i].in_use == 0U)
    {
      RxPool[i].in_use = 1U;
      return &RxPool[i];
    }
  }
  return NULL;
}

static void EthRx_Free(struct pbuf *p)
{
  struct pbuf_custom *custom = (struct pbuf_custom *)p;
  EthRxBuffer_t *entry = (EthRxBuffer_t *)((uint8_t *)custom - offsetof(EthRxBuffer_t, pbuf));
  entry->in_use = 0U;
}

void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
  EthRxBuffer_t *entry = EthRx_Alloc();
  if (entry == NULL)
  {
    ethernetif_rx_alloc_errors++;
    *buff = NULL;
    return;
  }
  entry->pbuf.custom_free_function = EthRx_Free;
  pbuf_alloced_custom(PBUF_RAW, 0U, PBUF_REF, &entry->pbuf, entry->data,
                      ETH_RX_BUFFER_SIZE);
  *buff = entry->data;
}

void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t length)
{
  EthRxBuffer_t *entry = EthRx_FromData(buff);
  struct pbuf *p;
  if (entry == NULL) return;

  /* The DMA has just written this buffer.  Invalidate before LwIP parses the
   * Ethernet/ARP/IP headers so a cached line cannot hide the received frame. */
  EthCacheInvalidate(buff, length);

  p = (struct pbuf *)&entry->pbuf;
  p->next = NULL;
  p->len = length;
  p->tot_len = length;
  if (*pStart == NULL) *pStart = p;
  else ((struct pbuf *)*pEnd)->next = p;
  *pEnd = p;
}

void HAL_ETH_TxFreeCallback(uint32_t *buff)
{
  (void)buff;
}

static int32_t EthPhy_Init(void)
{
  HAL_ETH_SetMDIOClockRange(&EthHandle);
  return 0;
}
static int32_t EthPhy_DeInit(void) { return 0; }

static void EthPhy_AbortStuckMdio(void)
{
  uint32_t mdioar = EthHandle.Instance->MACMDIOAR;
  if ((mdioar & ETH_MACMDIOAR_MB) != 0U)
  {
    /* A timed-out MDIO transaction leaves MB set in some STM32H7 revisions;
     * the HAL then rejects every later access as BUSY.  Clearing MB before a
     * retry lets the 20-ms hot-plug monitor recover without a reset. */
    mdioar &= ~ETH_MACMDIOAR_MB;
    EthHandle.Instance->MACMDIOAR = mdioar;
  }
}

static int32_t EthPhy_Read(uint32_t dev, uint32_t reg, uint32_t *value)
{
  EthPhy_AbortStuckMdio();
  if (HAL_ETH_ReadPHYRegister(&EthHandle, dev, reg, value) == HAL_OK)
    return 0;
  EthPhy_AbortStuckMdio();
  return -1;
}
static int32_t EthPhy_Write(uint32_t dev, uint32_t reg, uint32_t value)
{
  EthPhy_AbortStuckMdio();
  if (HAL_ETH_WritePHYRegister(&EthHandle, dev, reg, value) == HAL_OK)
    return 0;
  EthPhy_AbortStuckMdio();
  return -1;
}
static int32_t EthPhy_Tick(void) { return (int32_t)HAL_GetTick(); }

static uint8_t EthPhy_IdentityValid(uint32_t *id1_out, uint32_t *id2_out)
{
  uint32_t id1 = 0xFFFFFFFFU;
  uint32_t id2 = 0xFFFFFFFFU;

  if ((EthPhy_Read(ETH_PHY_BOARD_ADDR, LAN8742_PHYI1R, &id1) < 0) ||
      (EthPhy_Read(ETH_PHY_BOARD_ADDR, LAN8742_PHYI2R, &id2) < 0))
  {
    return 0U;
  }
  if ((id1 != ETH_PHY_ID1_EXPECTED) ||
      ((id2 & ETH_PHY_ID2_MASK) != ETH_PHY_ID2_EXPECTED))
  {
    return 0U;
  }
  /* A cold/disconnected PHY can leave one transient MDIO timeout behind.
   * Once a later probe proves the PHY is responding, remove only the MDIO
   * access flags; preserve DMA/MAC faults for the health report. */
  if (EthHandle.gState != HAL_ETH_STATE_STARTED)
    EthHandle.ErrorCode &= ~(HAL_ETH_ERROR_TIMEOUT | HAL_ETH_ERROR_BUSY);
  if (id1_out != NULL) *id1_out = id1;
  if (id2_out != NULL) *id2_out = id2;
  return 1U;
}

/* LAN8742_GetLinkState() accepts 0xffff as a normal register value.  On an
 * unresponsive MDIO bus that means LINK_STATUS and AUTONEGO_DONE both appear
 * set, producing a false UP/10M-half result.  Validate each register before
 * interpreting it so a cold PHY remains DOWN and is retried by the task. */
static int32_t EthPhy_GetValidatedLinkState(void)
{
  uint32_t bsr = 0xFFFFFFFFU;
  uint32_t bcr = 0xFFFFFFFFU;
  uint32_t physcsr = 0xFFFFFFFFU;

  if ((EthPhy_Read(Lan8742.DevAddr, LAN8742_BSR, &bsr) < 0) ||
      (EthPhy_Read(Lan8742.DevAddr, LAN8742_BSR, &bsr) < 0) ||
      (bsr == 0U) || (bsr == 0xFFFFU))
  {
    return LAN8742_STATUS_READ_ERROR;
  }
  if ((bsr & LAN8742_BSR_LINK_STATUS) == 0U)
    return LAN8742_STATUS_LINK_DOWN;

  if ((EthPhy_Read(Lan8742.DevAddr, LAN8742_BCR, &bcr) < 0) ||
      (bcr == 0U) || (bcr == 0xFFFFU))
  {
    return LAN8742_STATUS_READ_ERROR;
  }
  if ((bcr & LAN8742_BCR_AUTONEGO_EN) == 0U)
  {
    if (((bcr & LAN8742_BCR_SPEED_SELECT) != 0U) &&
        ((bcr & LAN8742_BCR_DUPLEX_MODE) != 0U))
      return LAN8742_STATUS_100MBITS_FULLDUPLEX;
    if ((bcr & LAN8742_BCR_SPEED_SELECT) != 0U)
      return LAN8742_STATUS_100MBITS_HALFDUPLEX;
    if ((bcr & LAN8742_BCR_DUPLEX_MODE) != 0U)
      return LAN8742_STATUS_10MBITS_FULLDUPLEX;
    return LAN8742_STATUS_10MBITS_HALFDUPLEX;
  }

  if ((EthPhy_Read(Lan8742.DevAddr, LAN8742_PHYSCSR, &physcsr) < 0) ||
      (physcsr == 0xFFFFU))
  {
    return LAN8742_STATUS_READ_ERROR;
  }
  if ((physcsr & LAN8742_PHYSCSR_AUTONEGO_DONE) == 0U)
    return LAN8742_STATUS_AUTONEGO_NOTDONE;
  switch (physcsr & LAN8742_PHYSCSR_HCDSPEEDMASK)
  {
    case LAN8742_PHYSCSR_100BTX_FD:
      return LAN8742_STATUS_100MBITS_FULLDUPLEX;
    case LAN8742_PHYSCSR_100BTX_HD:
      return LAN8742_STATUS_100MBITS_HALFDUPLEX;
    case LAN8742_PHYSCSR_10BT_FD:
      return LAN8742_STATUS_10MBITS_FULLDUPLEX;
    default:
      return LAN8742_STATUS_10MBITS_HALFDUPLEX;
  }
}

uint8_t ethernetif_get_link_info(uint32_t *speed_mbps, uint8_t *full_duplex)
{
  int32_t state;

  if ((speed_mbps == NULL) || (full_duplex == NULL)) return 0U;
  state = EthPhy_GetValidatedLinkState();
  switch (state)
  {
    case LAN8742_STATUS_100MBITS_FULLDUPLEX:
      *speed_mbps = 100U;
      *full_duplex = 1U;
      return 1U;
    case LAN8742_STATUS_100MBITS_HALFDUPLEX:
      *speed_mbps = 100U;
      *full_duplex = 0U;
      return 1U;
    case LAN8742_STATUS_10MBITS_FULLDUPLEX:
      *speed_mbps = 10U;
      *full_duplex = 1U;
      return 1U;
    case LAN8742_STATUS_10MBITS_HALFDUPLEX:
      *speed_mbps = 10U;
      *full_duplex = 0U;
      return 1U;
    default:
      return 0U;
  }
}

/* The Wildfire board uses LAN8720A.  LAN8742's MDIO register map is
 * compatible, but its generic component driver only discovers the address;
 * the board example also performs a software reset and restarts auto-negotiation. */
static int32_t EthPhy_EnsureAutoNegotiation(void)
{
  uint32_t bcr = 0U;
  if (EthPhy_Read(Lan8742.DevAddr, LAN8742_BCR, &bcr) < 0)
    return -1;
  if (bcr == 0xFFFFU) return -1;

  /* LAN8720A starts auto-negotiation by itself after power-up.  Do not reset
   * or restart a PHY that already has a live/negotiating link: doing so drops
   * carrier for seconds after every MCU reset and makes the first pings fail.
   * Only repair a genuinely disabled/powered-down configuration. */
  bcr &= ~LAN8742_BCR_POWER_DOWN;
  if ((bcr & LAN8742_BCR_AUTONEGO_EN) == 0U)
    bcr |= LAN8742_BCR_AUTONEGO_EN | LAN8742_BCR_RESTART_AUTONEGO;
  return (EthPhy_Write(Lan8742.DevAddr, LAN8742_BCR, bcr) < 0) ? -1 : 0;
}

void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
  if (heth == &EthHandle) AppEthernet_NotifyRxFromISR();
}

void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth)
{
  /* Let the owner task drain/replenish descriptors after recoverable RBU/AIS
   * events.  Fatal errors remain visible through the existing health log. */
  if (heth == &EthHandle) AppEthernet_NotifyRxFromISR();
}

uint8_t ethernetif_get_phy_diagnostics(uint32_t *phy_addr,
                                       uint32_t *phy_id1,
                                       uint32_t *phy_id2,
                                       uint32_t *bcr,
                                       uint32_t *bsr,
                                       uint32_t *phyc_sr)
{
  if ((phy_addr == NULL) || (phy_id1 == NULL) || (phy_id2 == NULL) ||
      (bcr == NULL) || (bsr == NULL) || (phyc_sr == NULL))
    return 0U;

  *phy_addr = 0xFFFFFFFFU;
  *phy_id1 = 0U;
  *phy_id2 = 0U;
  *bcr = 0U;
  *bsr = 0U;
  *phyc_sr = 0U;

  if (EthPhy_IdentityValid(phy_id1, phy_id2) == 0U) return 0U;
  if ((EthPhy_Read(ETH_PHY_BOARD_ADDR, LAN8742_BCR, bcr) < 0) ||
      (EthPhy_Read(ETH_PHY_BOARD_ADDR, LAN8742_BSR, bsr) < 0) ||
      (EthPhy_Read(ETH_PHY_BOARD_ADDR, LAN8742_PHYSCSR, phyc_sr) < 0) ||
      (*bcr == 0xFFFFU) || (*bsr == 0U) || (*bsr == 0xFFFFU) ||
      (*phyc_sr == 0xFFFFU))
  {
    return 0U;
  }
  *phy_addr = ETH_PHY_BOARD_ADDR;
  return 1U;
}

static err_t EthLowLevelOutput(struct netif *netif, struct pbuf *p)
{
  ETH_BufferTypeDef buffers[ETH_TX_DESC_CNT] = {0};
  ETH_TxPacketConfigTypeDef config = {0};
  struct pbuf *q;
  uint32_t count = 0U;
  (void)netif;
  for (q = p; q != NULL; q = q->next)
  {
    if (count >= ETH_TX_DESC_CNT) return ERR_IF;
    /* Make CPU-written LwIP data visible to the Ethernet DMA. */
    EthCacheClean(q->payload, q->len);
    buffers[count].buffer = q->payload;
    buffers[count].len = q->len;
    if (count != 0U) buffers[count - 1U].next = &buffers[count];
    count++;
  }
  config.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  config.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  config.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  config.Length = p->tot_len;
  config.TxBuffer = buffers;
  config.pData = p;
  /* HAL_ETH_Transmit() uses the same descriptor bookkeeping as the
   * interrupt API, but unlike HAL_ETH_Transmit_IT() it does not copy pData
   * into CurrentPacketAddress itself.  Set it before the blocking call so
   * HAL_ETH_ReleaseTxPacket() can clear PacketAddress[] and decrement
   * BuffersInUse.  Without this, every packet leaves an apparently empty
   * descriptor in the release accounting; the O(n) walk eventually throttles
   * UDP output from 1 kHz to roughly 125 Hz. */
  EthHandle.TxDescList.CurrentPacketAddress = (uint32_t *)p;
  HAL_StatusTypeDef status = HAL_ETH_Transmit(&EthHandle, &config, ETH_DMA_TIMEOUT);
  /* HAL_ETH_Transmit() is polling/blocking, but it still accounts the packet
   * in TxDescList. Release it before the next UDP packet is queued. */
  (void)HAL_ETH_ReleaseTxPacket(&EthHandle);
  if (status == HAL_OK) ethernetif_tx_packets++;
  else ethernetif_tx_errors++;
  return (status == HAL_OK) ? ERR_OK : ERR_IF;
}

static void EthLowLevelInit(struct netif *netif)
{
  static uint8_t mac[6] = {0x02U, 0x00U, 0x00U, 0x00U, 0x00U, 0x08U};

  /* These objects deliberately live in NOLOAD D2 SRAM so the startup copy/
   * clear loops never touch DMA memory.  A debugger reset preserves that RAM,
   * therefore clear the software ownership flags and stale descriptors here. */
  memset(DMARxDscrTab, 0, sizeof(DMARxDscrTab));
  memset(DMATxDscrTab, 0, sizeof(DMATxDscrTab));
  memset(RxPool, 0, sizeof(RxPool));
  ethernetif_rx_packets = 0U;
  ethernetif_rx_alloc_errors = 0U;
  ethernetif_tx_packets = 0U;
  ethernetif_tx_errors = 0U;

  EthHandle.Instance = ETH;
  EthHandle.Init.MACAddr = mac;
  EthHandle.Init.MediaInterface = HAL_ETH_RMII_MODE;
  EthHandle.Init.RxDesc = DMARxDscrTab;
  EthHandle.Init.TxDesc = DMATxDscrTab;
  EthHandle.Init.RxBuffLen = ETH_RX_BUFFER_SIZE;
  if (HAL_ETH_Init(&EthHandle) != HAL_OK)
  {
    printf("ETH: HAL_ETH_Init FAILED err=0x%08lX\r\n",
           (unsigned long)EthHandle.ErrorCode);
    return;
  }

  netif->hwaddr_len = ETH_HWADDR_LEN;
  memcpy(netif->hwaddr, mac, sizeof(mac));
  netif->mtu = 1500U;
  netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET;

  Lan8742IO.Init = EthPhy_Init;
  Lan8742IO.DeInit = EthPhy_DeInit;
  Lan8742IO.ReadReg = EthPhy_Read;
  Lan8742IO.WriteReg = EthPhy_Write;
  Lan8742IO.GetTick = EthPhy_Tick;
  (void)LAN8742_RegisterBusIO(&Lan8742, &Lan8742IO);
  /* The board address is fixed.  A cold PHY may not answer yet, which is not
   * an initialization failure: the 20-ms link monitor will retry address 0
   * while motor bring-up proceeds independently. */
  Lan8742.DevAddr = ETH_PHY_BOARD_ADDR;
  Lan8742.Is_Initialized = 1U;
  EthPhyConfigured = 0U;
  if (EthPhy_IdentityValid(NULL, NULL) != 0U)
  {
    if (EthPhy_EnsureAutoNegotiation() < 0)
      printf("ETH: PHY addr=%lu autoneg setup FAILED\r\n",
             (unsigned long)Lan8742.DevAddr);
    else
      EthPhyConfigured = 1U;
  }
  else
  {
    printf("ETH: PHY addr=0 not ready; network start deferred\r\n");
  }
  ethernet_link_check_state(netif);
}

err_t ethernetif_init(struct netif *netif)
{
  netif->name[0] = 'e';
  netif->name[1] = 'n';
  netif->output = etharp_output;
  netif->linkoutput = EthLowLevelOutput;
  EthLowLevelInit(netif);
  return ERR_OK;
}

void ethernetif_input(struct netif *netif)
{
  struct pbuf *p;

  /* Do not touch the DMA ring until the MAC is started and the PHY reports
   * link-up.  This function is called every tick by the Ethernet task; the
   * guard keeps a disconnected cable from affecting CAN control. */
  if ((netif == NULL) || (netif_is_link_up(netif) == 0) ||
      (EthHandle.gState != HAL_ETH_STATE_STARTED))
  {
    return;
  }

  for (;;)
  {
    p = NULL;
    if (HAL_ETH_ReadData(&EthHandle, (void **)&p) != HAL_OK) p = NULL;
    if (p == NULL) break;
    ethernetif_rx_packets++;
    if (netif->input(p, netif) != ERR_OK) pbuf_free(p);
  }
}

void ethernet_link_check_state(struct netif *netif)
{
  ETH_MACConfigTypeDef mac = {0};
  int32_t state;
  uint32_t speed, duplex;

  /* If the PHY was still powering up during netif_add(), finish its setup on
   * the first valid retry.  This is deliberately non-blocking. */
  if (EthPhyConfigured == 0U)
  {
    if ((EthPhy_IdentityValid(NULL, NULL) == 0U) ||
        (EthPhy_EnsureAutoNegotiation() < 0))
      state = LAN8742_STATUS_READ_ERROR;
    else
    {
      EthPhyConfigured = 1U;
      state = EthPhy_GetValidatedLinkState();
    }
  }
  else
  {
    state = EthPhy_GetValidatedLinkState();
  }
  if ((state <= LAN8742_STATUS_LINK_DOWN) ||
      (state == LAN8742_STATUS_AUTONEGO_NOTDONE))
  {
    if (EthHandle.gState == HAL_ETH_STATE_STARTED)
      (void)HAL_ETH_Stop_IT(&EthHandle);
    netif_set_down(netif);
    netif_set_link_down(netif);
    return;
  }
  if ((netif_is_link_up(netif) != 0) &&
      (EthHandle.gState == HAL_ETH_STATE_STARTED)) return;

  /* Repair an inconsistent software/MAC state before a hot-plug restart. */
  if (EthHandle.gState == HAL_ETH_STATE_STARTED)
    (void)HAL_ETH_Stop_IT(&EthHandle);
  netif_set_down(netif);
  netif_set_link_down(netif);
  switch (state)
  {
    case LAN8742_STATUS_100MBITS_FULLDUPLEX: speed = ETH_SPEED_100M; duplex = ETH_FULLDUPLEX_MODE; break;
    case LAN8742_STATUS_100MBITS_HALFDUPLEX: speed = ETH_SPEED_100M; duplex = ETH_HALFDUPLEX_MODE; break;
    case LAN8742_STATUS_10MBITS_FULLDUPLEX: speed = ETH_SPEED_10M; duplex = ETH_FULLDUPLEX_MODE; break;
    default: speed = ETH_SPEED_10M; duplex = ETH_HALFDUPLEX_MODE; break;
  }
  (void)HAL_ETH_GetMACConfig(&EthHandle, &mac);
  mac.Speed = speed;
  mac.DuplexMode = duplex;
  (void)HAL_ETH_SetMACConfig(&EthHandle, &mac);
  if (HAL_ETH_Start_IT(&EthHandle) != HAL_OK)
  {
    ethernetif_tx_errors++;
    return;
  }
  netif_set_up(netif);
  netif_set_link_up(netif);
}

u32_t sys_now(void)
{
  return HAL_GetTick();
}
