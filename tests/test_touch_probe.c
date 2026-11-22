/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
/* Exercise actual startup touch probe without I2C hardware. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "app_touch.c"
GPIO_TypeDef touch_h,touch_i,touch_d;
static unsigned scheduler,hal_ms,rtos_ms,reads,writes,config_reads;
static uint8_t absent,inconsistent,cold,fail_first_ack;
static const char *id;
static uint16_t device_address,rx,ry,expected_reg;
BaseType_t xTaskGetSchedulerState(void) {return scheduler?taskSCHEDULER_RUNNING:0;}
void HAL_Delay(uint32_t n) {assert(!scheduler);hal_ms+=n;}
void vTaskDelay(TickType_t n) {assert(scheduler);rtos_ms+=n;}
void HAL_GPIO_WritePin(GPIO_TypeDef *p,uint16_t pin,unsigned v) {(void)p;(void)pin;(void)v;}
void HAL_GPIO_Init(GPIO_TypeDef *p,GPIO_InitTypeDef *g) {(void)p;(void)g;}
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *p) {(void)p;return HAL_OK;}
HAL_StatusTypeDef HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *p,uint32_t v) {(void)p;(void)v;return HAL_OK;}
HAL_StatusTypeDef HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *p,uint32_t v) {(void)p;(void)v;return HAL_OK;}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *p,uint16_t a,uint16_t r,uint16_t m,
                                 uint8_t *d,uint16_t n,uint32_t timeout) {
  (void)p;assert(m==I2C_MEMADD_SIZE_16BIT && timeout==10);++reads;
  if(absent || (cold && hal_ms+rtos_ms<80) || a!=device_address) return HAL_ERROR;
  assert(n==4);
  if(r==GT_REG_PRODUCT_ID) {memset(d,0,n);memcpy(d,id,strlen(id));}
  else {assert(r==expected_reg);d[0]=rx;d[1]=rx>>8;d[2]=ry;d[3]=ry>>8;
    if(inconsistent && config_reads%2) d[0]^=1;
    ++config_reads;}
  return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *p,uint16_t a,uint16_t r,uint16_t m,
                                   uint8_t *d,uint16_t n,uint32_t t) {
  (void)p;(void)m;(void)t;
  assert(a==device_address && r==GT_REG_STATUS && n==1 && *d==0);++writes;
  if(fail_first_ack && writes==1)return HAL_ERROR;
  return HAL_OK; /* Never writes sensor configuration. */
}
static void reset(const char *name,uint16_t w,uint16_t h,uint16_t reg) {
  app_touch_initialized=0; scheduler=hal_ms=rtos_ms=reads=writes=config_reads=0;
  app_touch_ready=app_touch_address=0;app_touch_range_x=app_touch_range_y=0;
  absent=inconsistent=cold=fail_first_ack=0;id=name;rx=w;ry=h;expected_reg=reg;device_address=0x28;
}
int main(void) {
  uint16_t x,y;
  reset("1158",480,272,0x8051);assert(AppTouch_Init());
  assert(hal_ms==0 && rtos_ms==0 && writes==1 && config_reads==2);
  assert(AppTouch_Range(&x,&y) && x==480 && y==272);
  unsigned previous=reads;scheduler=1;assert(AppTouch_Init() && reads==previous && rtos_ms==0);
  reset("1158",800,480,0x8051);assert(AppTouch_Init() && AppTouch_Range(&x,&y) && x==800);
  reset("5688",480,272,0x8051);scheduler=1;cold=1;assert(AppTouch_Init() && rtos_ms==80 && !hal_ms);
  reset("9157",800,480,0x8048);device_address=0xBA;assert(AppTouch_Init());
  reset("1158",480,272,0x8051);inconsistent=1;assert(AppTouch_Init());assert(!AppTouch_Range(&x,&y));
  reset("1158",1024,600,0x8051);assert(AppTouch_Init());assert(!AppTouch_Range(&x,&y));
  assert(hal_ms==80);
  reset("1158",480,272,0x8051);fail_first_ack=1;assert(AppTouch_Init() && hal_ms==80 && writes==2);
  reset("xxxx",480,272,0);assert(AppTouch_Init());assert(!AppTouch_Range(&x,&y) && !config_reads);
  reset("1158",480,272,0x8051);absent=1;assert(!AppTouch_Init());assert(!AppTouch_Range(&x,&y));
  puts("Touch: pre-scheduler/task delay, idempotence, both addresses, config maps, inconsistent/unknown/missing detection passed");
}
