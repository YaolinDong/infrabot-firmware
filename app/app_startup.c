/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "bus_state.h"
#include "control_stats.h"
#include "dm_runtime.h"
#include "app_startup.h"

#include "app_arm_power.h"
#include "app_config.h"
#include "app_fdcan.h"
#include "app_lcd.h"
#include "app_lcd_boot.h"
#include "app_rtos.h"
#include "app_persist.h"
#include "app_state.h"
#include "app_tasks.h"
#include "dm_motor.h"
#include "main.h"
#include "task.h"

#include <stdio.h>
#ifdef FW_NETBOOT_APP
#include "update.h"
#endif

static void AppStartup_RxTraceDrain(void)
{
  AppTasks_DrainTrace();
}

static void AppStartup_StepDelay(uint32_t ms)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
  {
    TickType_t ticks = pdMS_TO_TICKS(ms);
    vTaskDelay((ticks != 0U) ? ticks : 1U);
  }
  else
  {
    HAL_Delay(ms);
  }
  AppStartup_RxTraceDrain();
}

void AppStartup_DwtInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void AppStartup_ConfigEthernetMemory(void)
{
  MPU_Region_InitTypeDef region = {0};

  /* The generated MPU_Config() subsequently installs region 0.  MPU region
   * registers survive disable/enable, so regions 1 and 2 remain active even
   * after CubeMX regenerates main.c. */
  HAL_MPU_Disable();

  /* ETH descriptors plus the RX pool occupy 0x30000000..0x300032BF. */
  region.Enable = MPU_REGION_ENABLE;
  region.Number = MPU_REGION_NUMBER1;
  region.BaseAddress = 0x30000000U;
  region.Size = MPU_REGION_SIZE_16KB;
  region.SubRegionDisable = 0x00U;
  region.TypeExtField = MPU_TEX_LEVEL1;
  region.AccessPermission = MPU_REGION_FULL_ACCESS;
  region.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  region.IsShareable = MPU_ACCESS_SHAREABLE;
  region.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  region.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&region);

  /* LwIP's private 16-KB heap starts at the next aligned boundary. */
  region.Number = MPU_REGION_NUMBER2;
  region.BaseAddress = 0x30004000U;
  HAL_MPU_ConfigRegion(&region);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

static void AppStartup_InitMotorTable(void)
{
#if DM_SINGLE_4310_TEST
  printf("Single motor test: DM4310 CAN ID=0x%02X Master ID=0x%02X\r\n",
         (unsigned)DM_SINGLE_4310_ID, (unsigned)DM_SINGLE_4310_MASTER_ID);
  DM_Motor_Init(&dm_motors[0], DM_SINGLE_4310_ID, DM_SINGLE_4310_MASTER_ID,
                DM_4310_POSITION_MAX, DM_4310_VELOCITY_MAX, DM_4310_TORQUE_MAX);
#else
  printf("Motor map: ID 0x01..0x08, Master 0x11..0x18\r\n");
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
  {
    DM_Motor_Init(&dm_motors[i], (uint16_t)(i + 1U), (uint16_t)(0x11U + i),
                  dm_motor_position_max[i], dm_motor_velocity_max[i],
                  dm_motor_torque_max[i]);
  }
#endif
}

static void AppStartup_StartMode(const DM_Motor_t *motor, uint8_t control_mode)
{
#if DM_VERBOSE_STARTUP_TRACE
  printf("--- config motor id=0x%03X master=0x%03X ctrl_mode=%u ---\r\n",
         (unsigned)motor->id, (unsigned)motor->master_id, (unsigned)control_mode);
#endif
  if (DM_Motor_SetControlMode(&hfdcan2, motor, control_mode) != HAL_OK) Error_Handler();
}

static uint16_t AppStartup_MotorModeId(uint8_t index)
{
  return (index == DM_GRIPPER_INDEX) ? DM_MODE_POSITION_CURRENT : DM_MODE_MIT;
}

static void AppStartup_DisableAll(void)
{
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
  {
    /* Preflight and the first shutdown happen before CTRL_MODE is changed;
     * all drives are still addressed through the default MIT command ID. */
    if (DM_Motor_Disable(&hfdcan2, &dm_motors[i], DM_MODE_MIT) != HAL_OK) Error_Handler();
    AppStartup_StepDelay(2U);
  }
  AppStartup_StepDelay(DM_ALL_STATE_SETTLE_MS);
}

static void AppStartup_ClearAll(void)
{
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
  {
    if (DM_Motor_ClearError(&hfdcan2, &dm_motors[i], AppStartup_MotorModeId(i)) != HAL_OK) Error_Handler();
    AppStartup_StepDelay(2U);
  }
  AppStartup_StepDelay(DM_CLEAR_SETTLE_MS);
}

/* With the arm unpowered nobody ACKs: auto-retransmitted frames pile up in
 * the TX FIFO (a full FIFO makes AppStartup_DisableAll hit Error_Handler) and
 * the TX error counter reaches error passive, reported as bus FAULT and never
 * cleared. Re-initializing the controller drops pending frames and resets the
 * error counters. Only bring-up uses CAN at this point. */
static void AppStartup_ResetCan(void)
{
  AppFdcan_StartForDmMotor(DM_CAN_USE_FD);
}

static volatile uint8_t arm_waiting;
static volatile uint8_t arm_responding;

uint8_t AppStartup_ArmWaiting(uint8_t *responding)
{
  if (arm_waiting == 0U) return AppArmPower_Waiting(responding);
  if (responding != NULL) *responding = arm_responding;
  return 1U;
}

/* Bit i of *present_mask / *fd_mask: motor i answered / answered in CAN FD.
 * verbose=0 prints nothing (periodic re-probe while waiting for arm power). */
static uint8_t AppStartup_Preflight(uint32_t *present_mask, uint32_t *fd_mask, uint8_t verbose)
{
  uint32_t present = 0U;
  uint32_t fd = 0U;

  if ((present_mask == NULL) || (fd_mask == NULL)) return 0U;

  if (verbose != 0U) printf("--- CAN preflight: classic 1Mbps probe for ID 0x01..0x08 ---\r\n");
  DM_Motor_SetCanFdMode(0U);
  dm_bus_stats.feedback_enabled = 1U;

  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
  {
    DM_Motor_t *motor = &dm_motors[i];
    uint32_t feedback_before = motor->feedback_count;
    uint32_t fd_before = dm_bus_stats.feedback_fd_count;
    uint32_t classic_before = dm_bus_stats.feedback_classic_count;
    HAL_StatusTypeDef status = DM_Motor_Enable(&hfdcan2, motor, DM_MODE_MIT);

    AppStartup_StepDelay(DM_CAN_PREFLIGHT_GAP_MS);

    if ((status == HAL_OK) && (motor->feedback_count > feedback_before))
    {
      uint8_t got_fd = (dm_bus_stats.feedback_fd_count > fd_before) ? 1U : 0U;
      uint8_t got_classic = (dm_bus_stats.feedback_classic_count > classic_before) ? 1U : 0U;
      present |= 1UL << i;
      if (got_fd != 0U) fd |= 1UL << i;
      if (verbose != 0U)
        printf("PREFLIGHT M%u id=0x%02X master=0x%02X ONLINE format=%s\r\n",
             (unsigned)(i + 1U), (unsigned)motor->id,
             (unsigned)motor->master_id,
             (got_fd != 0U) ? "FD" : ((got_classic != 0U) ? "CLASSIC" : "UNKNOWN"));
    }
    else
    {
      if (verbose != 0U)
        printf("PREFLIGHT M%u id=0x%02X master=0x%02X NO FEEDBACK tx=%s\r\n",
             (unsigned)(i + 1U), (unsigned)motor->id,
             (unsigned)motor->master_id,
             (status == HAL_OK) ? "OK" : "ERR");
    }
  }

  dm_bus_stats.feedback_enabled = 0U;
  *present_mask = present;
  *fd_mask = fd;
  return (present == DM_ALL_MOTOR_MASK) ? 1U : 0U;
}

static unsigned AppStartup_Count(uint32_t mask)
{
  unsigned count = 0U;
  for (; mask != 0U; mask &= mask - 1U) count++;
  return count;
}

static void AppStartup_ResetRuntimeCounters(void)
{
  dm_bus_stats.feedback_enabled = 0U;
  dm_bus_stats.feedback_match_count = 0U;
  dm_bus_stats.feedback_fd_count = 0U;
  dm_bus_stats.feedback_classic_count = 0U;
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
    dm_motors[i].feedback_count = 0U;
  dm_bus_stats.rx_trace_tail = dm_bus_stats.rx_trace_head;
  dm_bus_stats.rx_trace_dropped = 0U;
}

/* Until each motor in mask (those on the bus) has reported once. */
static void AppStartup_WaitForFeedback(uint32_t mask)
{
  uint32_t wait_start = HAL_GetTick();
  for (;;)
  {
    uint8_t ready = 1U;
    AppStartup_RxTraceDrain();
    for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
    {
      if ((mask & (1UL << i)) != 0U && dm_motors[i].feedback_count == 0U) { ready = 0U; break; }
    }
    if ((ready != 0U) || ((HAL_GetTick() - wait_start) >= 500U)) break;
    AppStartup_StepDelay(1U);
  }
}

#if DM_SINGLE_4310_TEST
static void AppStartup_RunSingle4310(void)
{
  DM_Motor_t *motor = &dm_motors[0];
  uint32_t last_command_tick;
  uint32_t last_status_tick;
  uint32_t target_switch_tick;
  uint32_t tx_ok = 0U;
  uint32_t tx_err = 0U;
  uint8_t target_positive = 0U;
  uint8_t fd_probe_failed = 0U;
  float target_position = 0.0f;

  dm_bus_stats.startup_trace_enabled = 1U;
  dm_bus_stats.feedback_enabled = 0U;
  dm_bus_stats.rx_frame_count = 0U;
  dm_bus_stats.feedback_match_count = 0U;
  dm_bus_stats.feedback_fd_count = 0U;
  dm_bus_stats.feedback_classic_count = 0U;
  dm_bus_stats.motor_error_status = 0U;
  dm_bus_stats.motor_error_code = 0U;

  printf("--- DM4310 single CAN FD test ---\r\n");
  printf("CAN ID=0x%02X Master ID=0x%02X, nominal=1Mbps data=5Mbps, FD+BRS\r\n",
         (unsigned)motor->id, (unsigned)motor->master_id);
  printf("MIT test: position +/-0.5 rad, kp_x100=%ld kd_x100=%ld, period=%ums\r\n",
         (long)(DM_SINGLE_4310_KP * 100.0f),
         (long)(DM_SINGLE_4310_KD * 100.0f),
         (unsigned)DM_SINGLE_4310_PERIOD_MS);

  AppStartup_StepDelay(100U);
  if (DM_Motor_Disable(&hfdcan2, motor, DM_MODE_MIT) != HAL_OK) tx_err++; else tx_ok++;
  AppStartup_StepDelay(20U);
  if (DM_Motor_SetControlMode(&hfdcan2, motor, 1U) != HAL_OK) tx_err++; else tx_ok++;
  AppStartup_StepDelay(20U);
  if (DM_Motor_ReadRegister(&hfdcan2, motor, DM_RID_CAN_BR) != HAL_OK) tx_err++; else tx_ok++;
  AppStartup_StepDelay(30U);
  if (DM_Motor_ClearError(&hfdcan2, motor, DM_MODE_MIT) != HAL_OK) tx_err++; else tx_ok++;
  AppStartup_StepDelay(20U);
  dm_bus_stats.feedback_enabled = 1U;
  if (DM_Motor_Enable(&hfdcan2, motor, DM_MODE_MIT) != HAL_OK) tx_err++; else tx_ok++;
  AppStartup_StepDelay(100U);
  AppStartup_RxTraceDrain();
  dm_bus_stats.startup_trace_enabled = 0U;

  if (motor->feedback_count != 0U) target_position = motor->position;
  target_switch_tick = HAL_GetTick() + DM_SINGLE_4310_DWELL_MS;
  last_command_tick = HAL_GetTick();
  last_status_tick = last_command_tick;

  printf("startup tx_ok=%lu tx_err=%lu rx=%lu feedback=%lu fd_feedback=%lu classic_feedback=%lu\r\n",
         (unsigned long)tx_ok, (unsigned long)tx_err,
         (unsigned long)dm_bus_stats.rx_frame_count, (unsigned long)motor->feedback_count,
         (unsigned long)dm_bus_stats.feedback_fd_count, (unsigned long)dm_bus_stats.feedback_classic_count);

  while (1)
  {
    uint32_t now = HAL_GetTick();
    AppStartup_RxTraceDrain();
    if ((int32_t)(now - target_switch_tick) >= 0)
    {
      target_positive = (target_positive == 0U) ? 1U : 0U;
      target_switch_tick += DM_SINGLE_4310_DWELL_MS;
    }
    target_position = (target_positive != 0U) ? DM_SINGLE_4310_TARGET_MAX
                                             : DM_SINGLE_4310_TARGET_MIN;
    if ((fd_probe_failed == 0U) && ((now - last_command_tick) >= DM_SINGLE_4310_PERIOD_MS))
    {
      last_command_tick += DM_SINGLE_4310_PERIOD_MS;
      if (DM_Motor_MIT(&hfdcan2, motor, target_position, 0.0f,
                       DM_SINGLE_4310_KP, DM_SINGLE_4310_KD, 0.0f) == HAL_OK) tx_ok++;
      else tx_err++;
    }
    if ((now - last_status_tick) >= 1000U)
    {
      FDCAN_ProtocolStatusTypeDef protocol = {0};
      FDCAN_ErrorCountersTypeDef counters = {0};
      (void)HAL_FDCAN_GetProtocolStatus(&hfdcan2, &protocol);
      (void)HAL_FDCAN_GetErrorCounters(&hfdcan2, &counters);
      if ((fd_probe_failed == 0U) && (motor->feedback_count == 0U) &&
          (counters.TxErrorCnt >= 128U))
      {
        fd_probe_failed = 1U;
        printf("CAN FD probe failed: no feedback/ACK, TEC=%lu DLEC=0x%08lX. "
               "Stop MIT TX; check CAN_BR=9, FD transceiver and 1M/5M bus.\r\n",
               (unsigned long)counters.TxErrorCnt,
               (unsigned long)protocol.DataLastErrorCode);
      }
      printf("4310 target_mrad=%ld state=%u pos_mrad=%ld vel_mrad_s=%ld tq_mNm=%ld "
             "fb=%lu rx=%lu fd_fb=%lu classic_fb=%lu tx_ok=%lu tx_err=%lu probe=%s\r\n",
             (long)(target_position * 1000.0f), (unsigned)motor->state,
             (long)(motor->position * 1000.0f), (long)(motor->velocity * 1000.0f),
             (long)(motor->torque * 1000.0f), (unsigned long)motor->feedback_count,
             (unsigned long)dm_bus_stats.rx_frame_count, (unsigned long)dm_bus_stats.feedback_fd_count,
             (unsigned long)dm_bus_stats.feedback_classic_count, (unsigned long)tx_ok,
             (unsigned long)tx_err, (fd_probe_failed == 0U) ? "RUN" : "STOP");
      printf("FDCAN protocol lec=0x%08lX dlec=0x%08lX activity=0x%08lX "
             "ep=%lu ew=%lu bo=%lu rx_fdf=%lu rx_brs=%lu tdc=%lu "
             "TEC=%lu REC=%lu HALerr=0x%08lX\r\n",
             (unsigned long)protocol.LastErrorCode,
             (unsigned long)protocol.DataLastErrorCode,
             (unsigned long)protocol.Activity,
             (unsigned long)protocol.ErrorPassive,
             (unsigned long)protocol.Warning,
             (unsigned long)protocol.BusOff,
             (unsigned long)protocol.RxFDFflag,
             (unsigned long)protocol.RxBRSflag,
             (unsigned long)protocol.TDCvalue,
             (unsigned long)counters.TxErrorCnt,
             (unsigned long)counters.RxErrorCnt,
             (unsigned long)HAL_FDCAN_GetError(&hfdcan2));
      last_status_tick = now;
    }
    AppStartup_StepDelay(1U);
  }
}
#endif

static void AppStartup_MotorBringup(void)
{
#if DM_SINGLE_4310_TEST
  AppStartup_RunSingle4310();
#else
  AppStartup_StepDelay(DM_MOTOR_STARTUP_WAIT_MS);
  uint32_t preflight_present = 0U;
  uint32_t preflight_fd = 0U;
  /* Three quick probes; then, if J1-J7 are not all there (arm powered after
   * the MCU, or a drive off), keep probing every second until they answer,
   * instead of stopping for good. The gripper J8 may be absent (removed or
   * swapped for another tool): the arm then runs on its own, and the gripper
   * can be enabled once it is connected. */
  uint8_t settled = 0U;     /* arm just powered: let the drives finish booting, probe again */
  for (uint32_t attempt = 0U; ; attempt++)
  {
    const uint8_t verbose = ((attempt < 3U) || (settled != 0U)) ? 1U : 0U;
    const uint8_t all = AppStartup_Preflight(&preflight_present, &preflight_fd, verbose);
    const uint32_t arm = preflight_present & DM_ARM_MOTOR_MASK;
    arm_responding = (uint8_t)AppStartup_Count(arm);
    if ((arm_waiting != 0U) && (settled == 0U) && (arm == DM_ARM_MOTOR_MASK))
    {
      AppStartup_ResetCan();
      AppStartup_DisableAll();
      AppStartup_StepDelay(DM_MOTOR_STARTUP_WAIT_MS);
      settled = 1U;
      continue;
    }
    if (all != 0U) break;
    if ((attempt >= 2U) && (arm == DM_ARM_MOTOR_MASK)) break;
    settled = 0U;
    /* A single lost feedback frame during PHY/CAN startup must not leave one
     * drive powered while the rest of the arm is stopped. Reset first so the
     * disable frames always fit in the TX FIFO. */
    AppStartup_ResetCan();
    AppStartup_DisableAll();
    if (attempt < 2U)
    {
      printf("CAN preflight retry %u/2\r\n", (unsigned)(attempt + 1U));
      AppStartup_StepDelay(50U);
      continue;
    }
    if (arm_waiting == 0U || (attempt % 10U) == 0U)
      printf("CAN preflight: %u/%u arm motors respond; waiting for arm power, re-probing every 1 s\r\n",
             (unsigned)arm_responding, (unsigned)DM_ARM_MOTOR_COUNT);
    arm_waiting = 1U;
    AppStartup_RxTraceDrain();
    AppStartup_StepDelay(1000U);
  }
  arm_responding = (uint8_t)AppStartup_Count(preflight_present & DM_ARM_MOTOR_MASK);
  if (arm_waiting != 0U)
  {
    AppStartup_ResetCan();     /* start the run with clean error counters */
    arm_waiting = 0U;
    printf("CAN preflight: all %u arm motors respond; continuing bring-up\r\n",
           (unsigned)DM_ARM_MOTOR_COUNT);
  }
  if ((preflight_present & DM_GRIPPER_MOTOR_MASK) == 0U)
    printf("CAN preflight: gripper J8 not found; the arm runs without it (enable the arm alone)\r\n");

  /* CAN FD when every motor on the bus answered in FD. */
  const uint8_t use_fd_control =
      ((DM_8_MOTOR_FD_CONTROL != 0U) && (preflight_fd == preflight_present)) ? 1U : 0U;
  DM_Motor_SetCanFdMode(0U);
  printf("CAN preflight complete: %u/%u online, %u/%u FD feedback; control=%s\r\n",
         AppStartup_Count(preflight_present), (unsigned)DM_MOTOR_COUNT,
         AppStartup_Count(preflight_fd), AppStartup_Count(preflight_present),
         (use_fd_control != 0U) ? "FD+BRS" : "Classic CAN (FD feedback)");

  AppStartup_ResetRuntimeCounters();
  AppStartup_DisableAll();
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
  {
    AppStartup_StartMode(&dm_motors[i],
                         (i == DM_GRIPPER_INDEX) ? DM_GRIPPER_CONTROL_MODE : 1U);
    AppStartup_StepDelay(2U);
  }
  AppStartup_StepDelay(DM_CONFIG_SETTLE_MS);
  AppStartup_ClearAll();
  dm_bus_stats.feedback_enabled = 1U;

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) != 32U)
  {
    uint32_t wait_start = HAL_GetTick();
    while ((HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) != 32U) &&
           ((HAL_GetTick() - wait_start) < 100U))
    {
      AppStartup_RxTraceDrain();
      AppStartup_StepDelay(1U);
    }
  }
  DM_Motor_SetCanFdMode(use_fd_control);
  printf("--- setup complete: Classic setup frames drained, runtime control=%s ---\r\n",
         (use_fd_control != 0U) ? "FD+BRS" : "Classic CAN");
  printf("--- all motors DISABLED; control=IDLE; apply MIT/POS_VEL profile before ENABLE; "
         "then select RT/NRT JOINT or CARTESIAN (MCU IK); M8 always force-position ---\r\n");

  AppStartup_WaitForFeedback(preflight_present);
  AppTasks_InitControlProfile();
  (void)AppPersist_LoadZero();
  (void)AppPersist_LoadControlProfile();
  if (AppTasks_Create() != pdPASS) Error_Handler();
#endif
}

static void AppStartup_MotorTask(void *argument)
{
  (void)argument;
  AppStartup_MotorBringup();
  vTaskDelete(NULL);
}

BaseType_t AppStartup_CreateMotorTask(void)
{
  return xTaskCreate(AppStartup_MotorTask, "dm_start", DM_STARTUP_TASK_STACK,
                     NULL, DM_STARTUP_TASK_PRIORITY, NULL);
}

void AppStartup_Run(void)
{
  printf("\r\ninfrabot_firmware start\r\n");
#ifdef FW_NETBOOT_APP
  if(FW_BOOT_STATS[3]==FW_BOOT_STATS_MAGIC && FW_BOOT_STATS[2]==~FW_BOOT_STATS_MAGIC){
    printf("NETBOOT: slot_scan=%lums images_checked=%lu\r\n",
           (unsigned long)FW_BOOT_STATS[0],(unsigned long)FW_BOOT_STATS[1]);
    FW_BOOT_STATS[3]=0U;
  }
#endif
  printf("USART1 PA9/PA10: 115200 8-N-1\r\n");
#if DM_CAN_USE_FD
  printf("FDCAN2 PB5=RX PB6=TX: CAN FD+BRS nominal=1Mbps data=5Mbps, data SP=75%%\r\n");
  printf("FDCAN runtime: preconfigured CAN_BR=9, no baud migration, TDC=%u/%u, auto retransmission=ON\r\n",
         (unsigned)DM_CAN_TDC_OFFSET, (unsigned)DM_CAN_TDC_FILTER);
#else
  printf("FDCAN2 PB5=RX PB6=TX: Classic CAN 1Mbps\r\n");
  printf("FDCAN runtime: AutoRetransmission=OFF, 8-motor batched startup\r\n");
#endif
  printf("Control target: %u Hz PC command stream, motor command target: %u Hz each, "
         "M1..M7 MIT + M8 position/current, SysTick 1 ms control, "
         "DWT jitter timer, UART TX ring\r\n",
         (unsigned)DM_CONTROL_HZ, (unsigned)DM_MOTOR_COMMAND_HZ);
  AppStartup_InitMotorTable();
  AppFdcan_StartForDmMotor(DM_CAN_USE_FD);
#if DM_LCD_ENABLE
  (void)AppLcd_Init(AppLcdBoot_Draw);
#endif

  /* Start scheduling before motor bring-up.  Ethernet has the higher startup
   * priority, so it performs the initial cable probe first; motor delays then
   * yield normally while networking and hot-plug monitoring remain alive. */
  AppRtos_Start();
}

void DM_Motor_CanTxTrace(uint16_t can_id,
                         const uint8_t *data,
                         uint8_t length,
                         uint8_t fd_format,
                         uint8_t brs,
                         HAL_StatusTypeDef status)
{
  if ((dm_motor_runtime.control_enabled != 0U) && (status == HAL_OK))
  {
    if (fd_format != 0U) dm_control_stats.fd_tx++;
    else dm_control_stats.classic_tx++;
  }
  if (dm_bus_stats.startup_trace_enabled == 0U) return;
  printf("CAN TX id=0x%03X dlc=%u fmt=%s brs=%u status=%s data=", (unsigned)can_id,
         (unsigned)length, (fd_format != 0U) ? "FD" : "CLASSIC", (unsigned)brs,
         (status == HAL_OK) ? "OK" : "ERR");
  for (uint8_t i = 0U; i < length && i < 8U; i++) printf("%02X ", (unsigned)data[i]);
  if (status != HAL_OK) printf("halerr=0x%08lX", (unsigned long)HAL_FDCAN_GetError(&hfdcan2));
  printf("\r\n");
}
