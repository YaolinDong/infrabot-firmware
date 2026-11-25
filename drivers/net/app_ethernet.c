/* SPDX-License-Identifier: Apache-2.0 */
/* Copyright (c) 2026 Yaolin Dong */
#include "dm_runtime.h"
#include "eth_state.h"
#include "app_runtime.h"
#include "app_ethernet.h"
#include "app_info.h"
#include "app_param.h"
#ifdef FW_NETBOOT_APP
#include "update.h"
#endif

#include "app_config.h"
#if defined(FW_NETBOOT_APP) && DM_ETH_USE_DHCP
#error "Network firmware update currently requires static addressing across reboots"
#endif
#include "app_state.h"
#include "app_motion.h"
#include "app_control_mode.h"
#include "robot_controller.h"
#include "app_tasks.h"
#include "app_safety.h"
#include "app_damiao.h"
#include "app_persist.h"
#include "app_log_task.h"
#include "ethernetif.h"
#include "lwip/init.h"
#include "lwip/ip4_addr.h"
#include "lwip/pbuf.h"
#include "lwip/timeouts.h"
#include "lwip/udp.h"
#include "lwip/etharp.h"
#if DM_ETH_USE_DHCP
#include "lwip/dhcp.h"
#endif
#include "netif/ethernet.h"

#include <stdio.h>
#include <string.h>

/* Every datagram is defined in infrabot/protocol.h (via app_config.h). */

static struct netif eth_netif;
static struct udp_pcb *telemetry_pcb;
static struct udp_pcb *command_pcb;
static struct udp_pcb *log_pcb;
static ip_addr_t telemetry_client_ip;
static uint16_t telemetry_client_port;
static uint8_t telemetry_client_valid;
static uint32_t telemetry_sequence;
/* Address change requested by the LCD; applied by this task (LwIP is NO_SYS
 * and owned by it). */
static AppNetConfig eth_pending_address;
static volatile uint8_t eth_pending_address_valid;
static volatile uint8_t eth_address_from_flash;
/* The MCU's view of the command link over the previous second (status task). */
static InfrabotLinkStats link_stats;
/* UDP is connectionless, so motion ownership is explicitly leased to one
 * source endpoint. A DISABLE/STOP packet remains an unconditional safety
 * operation. */
static uint8_t control_owner_valid;
static uint32_t control_owner_ip;
static uint16_t control_owner_port;
static uint32_t control_owner_last_tick;
#define MAINTENANCE_RESPONSE_DEPTH 8U
static volatile uint32_t maintenance_response_head, maintenance_response_tail;
static InfrabotMaintenanceReply maintenance_responses[MAINTENANCE_RESPONSE_DEPTH];

static uint8_t AppEthernet_ControlOwnerAllowed(const ip_addr_t *addr,
                                               uint16_t port,
                                               uint8_t command)
{
  const uint32_t ip = ip4_addr_get_u32(ip_2_ip4(addr));
  const uint32_t now = HAL_GetTick();
  /* Any client may remove power, from both groups or from one. */
  const uint8_t safety_command = (command == DM_ETH_CMD_DISABLE ||
                                   command == DM_ETH_CMD_STOP ||
                                   command == DM_ETH_CMD_DISABLE_ARM ||
                                   command == DM_ETH_CMD_DISABLE_GRIPPER);
  const uint8_t arm_idle = (dm_motor_runtime.control_enabled == 0U) &&
                           !AppMotion_Owned() &&
                           eth_udp_command.control_mode == DM_ETH_CONTROL_IDLE;
  if (safety_command)
  {
    if (command == DM_ETH_CMD_DISABLE) control_owner_valid = 0U;
    return 1U;
  }
  if (!control_owner_valid || arm_idle ||
      (now - control_owner_last_tick) > (DM_ETH_COMMAND_TIMEOUT_MS * 5U))
  {
    control_owner_valid = 1U;
    control_owner_ip = ip;
    control_owner_port = port;
  }
  if (control_owner_ip != ip || control_owner_port != port) return 0U;
  control_owner_last_tick = now;
  return 1U;
}

static uint8_t AppEthernet_TelemetryEndpointAllowed(const ip_addr_t *addr)
{
  const uint32_t ip = ip4_addr_get_u32(ip_2_ip4(addr));
  const uint8_t arm_idle = (dm_motor_runtime.control_enabled == 0U) &&
                           !AppMotion_Owned() &&
                           eth_udp_command.control_mode == DM_ETH_CONTROL_IDLE;
  /* The controlling PC may register again (a restarted GUI, which must get
   * telemetry to resume the heartbeat of a controlled stop); another PC only
   * when no one controls the arm or its owner has gone quiet, as for
   * commands (AppEthernet_ControlOwnerAllowed). */
  const uint8_t owner_quiet =
      (HAL_GetTick() - control_owner_last_tick) > (DM_ETH_COMMAND_TIMEOUT_MS * 5U);
  if (!control_owner_valid || arm_idle || control_owner_ip == ip || owner_quiet) return 1U;
  return 0U;
}

static void AppEthernet_OwnerReject(uint32_t sequence, uint8_t command)
{
  AppEthernet_QueueMaintenanceResult(sequence, command, DM_ETH_MOTOR_ALL,
                                     5U, DM_ETH_REGISTER_U32, 0U, 0U);
}

/* Counts a rejected command datagram and keeps why (DM_ETH_INVALID_*). */
static void AppEthernet_Reject(uint8_t reason, uint16_t detail)
{
  eth_udp_command.invalid_count++;
  eth_udp_command.invalid_reason = reason;
  eth_udp_command.invalid_length = detail;
}

static uint8_t AppEthernet_TryInfo(struct pbuf *p, const ip_addr_t *addr, uint16_t port)
{
  InfrabotInfoRequest request;
  if (p->tot_len != sizeof(request) ||
      pbuf_copy_partial(p, &request, sizeof(request), 0U) != sizeof(request)) return 0U;
  InfrabotInfoReplyV2 reply;
  const uint16_t length = AppInfo_Reply(&request, &reply);
  if (length == 0U) return 0U;
  if (addr != NULL && telemetry_pcb != NULL) {
    struct pbuf *out = pbuf_alloc(PBUF_TRANSPORT, length, PBUF_RAM);
    if (out != NULL) {
      memcpy(out->payload, &reply, length);
      (void)udp_sendto(telemetry_pcb, out, addr, port);
      pbuf_free(out);
    }
  }
  return 1U;
}

/* True when this sender may change parameters: it controls the arm, or no one does.
 * Unlike AppEthernet_ControlOwnerAllowed it never takes ownership. */
static uint8_t AppEthernet_ParamWriteAllowed(const ip_addr_t *addr, uint16_t port)
{
  const uint32_t ip = ip4_addr_get_u32(ip_2_ip4(addr));
  const uint8_t arm_idle = (dm_motor_runtime.control_enabled == 0U) &&
                           !AppMotion_Owned() &&
                           eth_udp_command.control_mode == DM_ETH_CONTROL_IDLE;
  const uint8_t owner_quiet =
      (HAL_GetTick() - control_owner_last_tick) > (DM_ETH_COMMAND_TIMEOUT_MS * 5U);
  if (!control_owner_valid || arm_idle || owner_quiet) return 1U;
  return (control_owner_ip == ip && control_owner_port == port) ? 1U : 0U;
}

/* Parameter service: one DMPR in, one DMPA out from the command port. */
static uint8_t AppEthernet_TryParam(struct pbuf *p, const ip_addr_t *addr, uint16_t port)
{
  InfrabotParamRequest request;
  if (p->tot_len != sizeof(request) ||
      pbuf_copy_partial(p, &request, sizeof(request), 0U) != sizeof(request)) return 0U;
  InfrabotParamReply reply;
  const uint8_t may_write = (addr != NULL && AppEthernet_ParamWriteAllowed(addr, port)) ? 1U : 0U;
  const uint16_t length = AppParam_Handle(&request, may_write, &reply);
  if (length == 0U) return 0U;
  if (addr != NULL && command_pcb != NULL) {
    struct pbuf *out = pbuf_alloc(PBUF_TRANSPORT, length, PBUF_RAM);
    if (out != NULL) {
      memcpy(out->payload, &reply, length);
      (void)udp_sendto(command_pcb, out, addr, port);
      pbuf_free(out);
    }
  }
  return 1U;
}

uint64_t AppEthernet_GetTimeUs(void)
{
  /* DWT->CYCCNT wraps in roughly 10 seconds on a 400 MHz H7.  Extend it
   * under a short RTOS critical section; both the LwIP callback and the 1 kHz
   * control task use this function, so no caller can observe a torn wrap. */
  static uint32_t previous_cycles;
  static uint64_t high_cycles;
  uint32_t now;
  uint64_t extended;
  taskENTER_CRITICAL();
  now = DWT->CYCCNT;
  if (now < previous_cycles) high_cycles += (1ULL << 32);
  previous_cycles = now;
  extended = high_cycles + now;
  taskEXIT_CRITICAL();
  return (SystemCoreClock != 0U) ?
         ((extended * 1000000ULL) / SystemCoreClock) : 0ULL;
}


void AppEthernet_QueueMaintenanceResult(uint32_t sequence,
                                        uint8_t command,
                                        uint8_t motor_index,
                                        uint8_t status,
                                        uint8_t value_type,
                                        uint8_t register_id,
                                        uint32_t value)
{
  InfrabotMaintenanceReply maintenance_response = {0};
  maintenance_response.magic = INFRABOT_MAGIC_MAINTENANCE;
  maintenance_response.version = INFRABOT_MAINTENANCE_V1;
  maintenance_response.command = command;
  maintenance_response.motor_index = motor_index;
  maintenance_response.status = status;
  maintenance_response.value_type = value_type;
  maintenance_response.register_id = register_id;
  maintenance_response.reserved = 0U;
  maintenance_response.sequence = sequence;
  maintenance_response.value = value;
  maintenance_response.crc32 = infrabot_crc32(
      (const uint8_t *)&maintenance_response,
      sizeof(maintenance_response) - sizeof(maintenance_response.crc32));
  taskENTER_CRITICAL();
  if (maintenance_response_head - maintenance_response_tail < MAINTENANCE_RESPONSE_DEPTH)
    maintenance_responses[maintenance_response_head++ % MAINTENANCE_RESPONSE_DEPTH] = maintenance_response;
  else
    eth_udp_tx_stats.errors++;
  taskEXIT_CRITICAL();
}

static uint32_t AppEthernet_CyclesToUs(uint32_t cycles)
{
  if (SystemCoreClock == 0U) return 0U;
  return (uint32_t)(((uint64_t)cycles * 1000000ULL) / SystemCoreClock);
}

static float AppEthernet_CommandMin(uint8_t index)
{
  return (index == DM_GRIPPER_INDEX) ? DM_GRIPPER_POSITION_MIN :
                                       -dm_motors[index].position_max;
}

static float AppEthernet_CommandMax(uint8_t index)
{
  return (index == DM_GRIPPER_INDEX) ? DM_GRIPPER_POSITION_MAX :
                                       dm_motors[index].position_max;
}

static float AppEthernet_ClampAbs(float value, float limit)
{
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

static void AppEthernet_RecordTiming(volatile uint32_t *count,
                                     volatile uint32_t *interval_count,
                                     volatile uint32_t *interval_min_us,
                                     volatile uint32_t *interval_max_us,
                                     volatile uint32_t *interval_sum_us,
                                     volatile uint32_t *last_interval_us,
                                     volatile uint32_t *jitter_sum_us,
                                     volatile uint32_t *jitter_max_us,
                                     volatile uint32_t *last_cycle,
                                     uint32_t nominal_us)
{
  uint32_t now = DWT->CYCCNT;
  uint32_t previous = *last_cycle;
  *last_cycle = now;
  (*count)++;
  if (previous != 0U)
  {
    uint32_t interval_us = AppEthernet_CyclesToUs(now - previous);
    uint32_t jitter_us = (interval_us >= nominal_us) ?
                         (interval_us - nominal_us) : (nominal_us - interval_us);
    if ((*interval_count == 0U) || (interval_us < *interval_min_us))
      *interval_min_us = interval_us;
    if ((*interval_count == 0U) || (interval_us > *interval_max_us))
      *interval_max_us = interval_us;
    if (jitter_us > *jitter_max_us) *jitter_max_us = jitter_us;
    *last_interval_us = interval_us;
    *interval_sum_us += interval_us;
    *jitter_sum_us += jitter_us;
    (*interval_count)++;
  }
}

void AppEthernet_NotifyRxFromISR(void)
{
  BaseType_t higher_priority_task_woken = pdFALSE;

  if (dm_ethernet_task_handle != NULL)
  {
    vTaskNotifyGiveFromISR(dm_ethernet_task_handle, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
}

static void AppEthernet_PrintAddress(const char *label, const ip4_addr_t *address)
{
  printf("ETH: %s=%u.%u.%u.%u\r\n", label,
         (unsigned)ip4_addr1(address), (unsigned)ip4_addr2(address),
         (unsigned)ip4_addr3(address), (unsigned)ip4_addr4(address));
}

static void AppEthernet_Receive(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                                const ip_addr_t *addr, u16_t port)
{
  (void)arg;
  if (p != NULL)
  {
    InfrabotSyncRequest request;
    if ((p->tot_len == sizeof(request)) &&
        (pbuf_copy_partial(p, &request, sizeof(request), 0U) == sizeof(request)) &&
        request.magic == INFRABOT_MAGIC_SYNC_REQUEST && request.version == INFRABOT_SYNC_V1 &&
        infrabot_crc32((const uint8_t *)&request, sizeof(request) - sizeof(request.crc32)) == request.crc32)
    {
      InfrabotSyncReply response;
      memset(&response, 0, sizeof(response));
      response.magic = INFRABOT_MAGIC_SYNC_REPLY;
      response.version = INFRABOT_SYNC_V1;
      response.sequence = request.sequence;
      response.pc_timestamp_us = request.pc_timestamp_us;
      response.mcu_rx_timestamp_us = AppEthernet_GetTimeUs();
      struct pbuf *out = pbuf_alloc(PBUF_TRANSPORT, sizeof(response), PBUF_RAM);
      if (out != NULL)
      {
        response.mcu_tx_timestamp_us = AppEthernet_GetTimeUs();
        response.crc32 = infrabot_crc32((const uint8_t *)&response,
                                           sizeof(response) - sizeof(response.crc32));
        memcpy(out->payload, &response, sizeof(response));
        (void)udp_sendto(telemetry_pcb, out, addr, port);
        pbuf_free(out);
      }
    }
    else
    {
      /* A second PC may observe the port, but it must not steal the sole
       * telemetry destination while another source owns motion control. */
      if (AppEthernet_TelemetryEndpointAllowed(addr) != 0U)
      {
        telemetry_client_ip = *addr;
        telemetry_client_port = port;
        telemetry_client_valid = 1U;
      }
    }
    pbuf_free(p);
    (void)pcb;
  }
}

static void AppEthernet_AcceptCommand(uint32_t sequence, uint8_t command,
                                      uint64_t execute_mcu_us,
                                      const int32_t *target_position_mrad)
{
  uint8_t sequence_is_new = 1U;
  const uint64_t rx_mcu_us = AppEthernet_GetTimeUs();

  if (command == DM_ETH_CMD_HOLD_HEARTBEAT) {
    /* No target fields are read. This heartbeat cannot keep a lost RT
     * stream or an active DMOV session alive. */
    if ((eth_udp_command.control_mode == DM_ETH_CONTROL_IDLE ||
         (eth_udp_command.control_mode == DM_ETH_CONTROL_NRT_JOINT && !AppMotion_Owned())) &&
        eth_udp_command.control_request != DM_ETH_CMD_DISABLE) {
      eth_udp_command.last_tick = HAL_GetTick();
    }
    return;
  }
  if (command == DM_ETH_CMD_STOP) {
    /* Controlled stop (any client, like DISABLE): brake, hold in IDLE.  The
     * PC repeats it; a repeat is a no-op. */
    AppSafety_RequestStop(INFRABOT_STOP_REQUESTED);
    return;
  }
  if (command >= DM_ETH_CMD_ENABLE_ARM && command <= DM_ETH_CMD_DISABLE_GRIPPER) {
    /* One drive group; the control task stages or disables it.  Like
     * ENABLE, an enable needs IDLE and starts the command watchdog. */
    const uint8_t enable = command == DM_ETH_CMD_ENABLE_ARM || command == DM_ETH_CMD_ENABLE_GRIPPER;
    const uint8_t mask = (command == DM_ETH_CMD_ENABLE_ARM || command == DM_ETH_CMD_DISABLE_ARM) ?
                         DM_ARM_MOTOR_MASK : DM_GRIPPER_MOTOR_MASK;
    if (enable && (eth_udp_command.control_mode != DM_ETH_CONTROL_IDLE || !AppControlMode_ProfileReady())) {
      AppEthernet_Reject(DM_ETH_INVALID_STATE, command);
      return;
    }
    taskENTER_CRITICAL();
    if (enable) eth_udp_command.enable_request |= mask;
    else eth_udp_command.disable_request |= mask;
    /* A move owns the arm: it ends, the gripper keeps running.  Together
     * with the request, so the control task never sees one alone. */
    if (command == DM_ETH_CMD_DISABLE_ARM) AppMotion_RequestRelease();
    eth_udp_command.last_tick = HAL_GetTick();
    eth_udp_command.active = 1U;
    taskEXIT_CRITICAL();
    return;
  }
  if (command == DM_ETH_CMD_MIT &&
      (eth_udp_command.control_mode != DM_ETH_CONTROL_RT_JOINT || !dm_motor_runtime.control_enabled ||
       AppTasks_Enabling() || eth_udp_command.control_request || !AppControlMode_ProfileReady())) {
    AppEthernet_Reject(DM_ETH_INVALID_JOINT_STREAM, command);
    return;
  }
  if ((command == DM_ETH_CMD_ENABLE || command == DM_ETH_CMD_SET_ZERO_ENABLE) &&
      (eth_udp_command.control_mode != DM_ETH_CONTROL_IDLE || !AppControlMode_ProfileReady())) {
    AppEthernet_Reject(DM_ETH_INVALID_STATE, command);
    return;
  }
  if (command == DM_ETH_CMD_SET_ZERO &&
      (eth_udp_command.control_mode != DM_ETH_CONTROL_IDLE || dm_motor_runtime.control_enabled)) {
    AppEthernet_Reject(DM_ETH_INVALID_STATE, command);
    return;
  }

  if (command == DM_ETH_CMD_DISABLE) AppMotion_RequestCancel();
  else if (AppMotion_Owned()) {
    /* Never let a concurrent 1 kHz GUI stream overwrite a blocking move. */
    AppEthernet_Reject(DM_ETH_INVALID_MOTION_OWNED, command);
    return;
  }

  if ((command != DM_ETH_CMD_MIT) &&
      (command != DM_ETH_CMD_ENABLE) &&
      (command != DM_ETH_CMD_DISABLE) &&
      (command != DM_ETH_CMD_SET_ZERO) &&
      (command != DM_ETH_CMD_SET_ZERO_ENABLE))
  {
    AppEthernet_Reject(DM_ETH_INVALID_COMMAND, command);
    return;
  }

  taskENTER_CRITICAL();
  if ((command == DM_ETH_CMD_MIT) &&
      (eth_udp_command.sequence_valid != 0U) &&
      (((sequence - eth_udp_command.last_sequence) == 0U) ||
       ((sequence - eth_udp_command.last_sequence) >= 0x80000000U)))
  {
    sequence_is_new = 0U;
  }
  eth_udp_command.last_tick = HAL_GetTick();
  eth_udp_command.last_rx_mcu_us = rx_mcu_us;
  if (sequence_is_new != 0U)
  {
    eth_udp_command.last_sequence = sequence;
    for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
    {
      float target = (float)target_position_mrad[i] / 1000.0f;
      const float min_target = AppEthernet_CommandMin(i);
      const float max_target = AppEthernet_CommandMax(i);
      if (target > max_target) target = max_target;
      if (target < min_target) target = min_target;
      eth_udp_command.pending_position[i] = target;
    }
    eth_udp_command.pending_sequence = sequence;
    eth_udp_command.pending_execute_us = execute_mcu_us;
    if (command != DM_ETH_CMD_MIT)
    {
      eth_udp_command.control_request = command;
      eth_udp_command.queue_head = 0U;
      eth_udp_command.queue_tail = 0U;
      eth_udp_command.queue_count = 0U;
      eth_udp_command.applied_sequence = sequence;
      eth_udp_command.applied_mcu_us = rx_mcu_us;
      eth_udp_command.pending_execute_us = 0U;
      /* ENABLE target latching is deliberately deferred to the control task:
       * AppTasks_StartEnableSequence() samples each motor's measured raw
       * angle immediately before the staged CAN enable.  This prevents a
       * stale PC target from producing motion during enable. */
      /* ENABLE/DISABLE/ZERO starts a new PC command epoch.  This allows a
       * freshly launched PC process whose sequence begins at zero to control
       * an MCU that was previously driven by another process. */
      eth_udp_command.sequence_valid = 0U;
    }
    else if (execute_mcu_us == 0U)
    {
      eth_udp_command.queue_head = 0U;
      eth_udp_command.queue_tail = 0U;
      eth_udp_command.queue_count = 0U;
      for (uint8_t i = 0U; i < DM_ARM_MOTOR_COUNT; i++) /* J8: DM_ETH_CMD_GRIPPER */
        if (!dm_motor_runtime.override[i]) eth_udp_command.target_position[i] = eth_udp_command.pending_position[i];
      eth_udp_command.applied_sequence = sequence;
      eth_udp_command.applied_mcu_us = rx_mcu_us;
      eth_udp_command.pending_execute_us = 0U;
    }
    else
    {
      uint8_t slot;
      if (eth_udp_command.queue_count >= DM_ETH_CMD_QUEUE_DEPTH)
      {
        eth_udp_command.queue_tail =
            (uint8_t)((eth_udp_command.queue_tail + 1U) % DM_ETH_CMD_QUEUE_DEPTH);
        eth_udp_command.queue_count--;
      }
      slot = eth_udp_command.queue_head;
      for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
        eth_udp_command.queue_position[slot][i] = eth_udp_command.pending_position[i];
      eth_udp_command.queue_sequence[slot] = sequence;
      eth_udp_command.queue_execute_us[slot] = execute_mcu_us;
      eth_udp_command.queue_head =
          (uint8_t)((eth_udp_command.queue_head + 1U) % DM_ETH_CMD_QUEUE_DEPTH);
      eth_udp_command.queue_count++;
    }
    eth_udp_command.sequence_valid = 1U;
  }
  eth_udp_command.active = 1U;
  taskEXIT_CRITICAL();

  AppEthernet_RecordTiming(&eth_udp_command.rx_count,
                           &eth_udp_command.interval_count,
                           &eth_udp_command.interval_min_us,
                           &eth_udp_command.interval_max_us,
                           &eth_udp_command.interval_sum_us,
                           &eth_udp_command.last_interval_us,
                           &eth_udp_command.jitter_sum_us,
                           &eth_udp_command.jitter_max_us,
                           &eth_udp_command.last_cycle,
                           (DM_ETH_COMMAND_HZ != 0U) ?
                           (1000000U / DM_ETH_COMMAND_HZ) : 0U);
}

static void AppEthernet_AcceptArmCommand(const InfrabotJointStream *packet)
{
  static uint32_t last_sequence;
  const uint64_t rx_mcu_us = AppEthernet_GetTimeUs();
  if (packet == NULL || packet->execute_mcu_us != 0U)
  {
    AppEthernet_Reject(DM_ETH_INVALID_TIMED, packet ? packet->command : 0U);
    return;
  }

  /* A v5 packet with command 19 is a low-rate, all-joint control-profile
   * transaction.  It is not part of the 1 kHz stream: values are copied into
   * the mailbox here and AppDamiao_Process validates and writes them to the
   * reserved MCU Flash sector when the arm is disabled. */
  if (packet->command == DM_ETH_CMD_STORE_CONTROL_PROFILE)
  {
    if (AppMotion_Owned() || AppTasks_Enabling() || dm_motor_runtime.control_enabled ||
        eth_udp_command.control_mode != DM_ETH_CONTROL_IDLE || eth_udp_command.control_request)
    {
      AppEthernet_QueueMaintenanceResult(packet->sequence,
                                          DM_ETH_CMD_STORE_CONTROL_PROFILE,
                                          DM_ETH_MOTOR_ALL, 4U, 0U, 0U, 0U);
      return;
    }
    taskENTER_CRITICAL();
    if (eth_maintenance.profile_pending != 0U || eth_maintenance.pending != 0U ||
        eth_maintenance.waiting != 0U)
    {
      taskEXIT_CRITICAL();
      AppEthernet_QueueMaintenanceResult(packet->sequence,
                                          DM_ETH_CMD_STORE_CONTROL_PROFILE,
                                          DM_ETH_MOTOR_ALL, 4U, 0U, 0U, 0U);
      return;
    }
    for (uint8_t i = 0U; i < DM_MOTOR_COUNT; ++i)
    {
      eth_udp_command.target_velocity[i] = (float)packet->target_velocity_mrad_s[i] / 1000.0f;
      eth_udp_command.target_kp[i] = (float)packet->kp_milli[i] / 1000.0f;
      eth_udp_command.target_kd[i] = (float)packet->kd_milli[i] / 1000.0f;
      eth_udp_command.target_torque[i] = (float)packet->torque_mNm[i] / 1000.0f;
      eth_udp_command.velocity_limit[i] = (float)packet->velocity_limit_mrad_s[i] / 1000.0f;
      eth_udp_command.torque_limit_ratio[i] = (float)packet->torque_ratio_milli[i] / 1000.0f;
    }
    eth_maintenance.profile_sequence = packet->sequence;
    eth_maintenance.profile_pending = 1U;
    taskEXIT_CRITICAL();
    return;
  }

  /* v5 is the joint RT stream. Cartesian RT uses the separate v7 pose packet
   * below; keeping the formats distinct prevents a pose payload from being
   * mistaken for joint milliradian targets. */
  if (AppMotion_Owned() || packet->command != DM_ETH_CMD_MIT ||
      eth_udp_command.control_mode != DM_ETH_CONTROL_RT_JOINT || !dm_motor_runtime.control_enabled ||
      AppTasks_Enabling() || eth_udp_command.control_request || !AppControlMode_ProfileReady())
  {
    AppEthernet_Reject(AppMotion_Owned() ? DM_ETH_INVALID_MOTION_OWNED : DM_ETH_INVALID_JOINT_STREAM,
                       packet->command);
    return;
  }

  taskENTER_CRITICAL();
  if ((eth_udp_command.sequence_valid != 0U) &&
      (((packet->sequence - last_sequence) == 0U) ||
       ((packet->sequence - last_sequence) >= 0x80000000U)))
  {
    taskEXIT_CRITICAL();
    return;
  }
  last_sequence = packet->sequence;
  eth_udp_command.last_sequence = packet->sequence;
  eth_udp_command.last_tick = HAL_GetTick();
  eth_udp_command.last_rx_mcu_us = rx_mcu_us;
  /* The arm: the gripper keeps its own command (DM_ETH_CMD_GRIPPER), whatever
   * the stream's J8 fields say. */
  for (uint8_t i = 0U; i < DM_ARM_MOTOR_COUNT; ++i)
  {
    float position = (float)packet->target_position_mrad[i] / 1000.0f;
    const float pmin = AppEthernet_CommandMin(i);
    const float pmax = AppEthernet_CommandMax(i);
    if (position < pmin) position = pmin;
    if (position > pmax) position = pmax;
    eth_udp_command.target_position[i] = position;
    eth_udp_command.target_velocity[i] = AppEthernet_ClampAbs(
        (float)packet->target_velocity_mrad_s[i] / 1000.0f,
        fminf(dm_motors[i].velocity_max,DM_ARM_MAX_SPEED_RAD_S));
    eth_udp_command.target_kp[i] = ((float)packet->kp_milli[i] / 1000.0f);
    if (eth_udp_command.target_kp[i] < 0.0f) eth_udp_command.target_kp[i] = 0.0f;
    if (eth_udp_command.target_kp[i] > 500.0f) eth_udp_command.target_kp[i] = 500.0f;
    eth_udp_command.target_kd[i] = ((float)packet->kd_milli[i] / 1000.0f);
    if (eth_udp_command.target_kd[i] < 0.0f) eth_udp_command.target_kd[i] = 0.0f;
    if (eth_udp_command.target_kd[i] > 5.0f) eth_udp_command.target_kd[i] = 5.0f;
    eth_udp_command.target_torque[i] = AppEthernet_ClampAbs(
        (float)packet->torque_mNm[i] / 1000.0f, dm_motors[i].torque_max);
    eth_udp_command.velocity_limit[i] = (float)packet->velocity_limit_mrad_s[i] / 1000.0f;
    if (eth_udp_command.velocity_limit[i] < 0.0f) eth_udp_command.velocity_limit[i] = 0.0f;
    if (eth_udp_command.velocity_limit[i] > fminf(dm_motors[i].velocity_max,DM_ARM_MAX_SPEED_RAD_S))
      eth_udp_command.velocity_limit[i] = fminf(dm_motors[i].velocity_max,DM_ARM_MAX_SPEED_RAD_S);
    eth_udp_command.torque_limit_ratio[i] = (float)packet->torque_ratio_milli[i] / 1000.0f;
    if (eth_udp_command.torque_limit_ratio[i] < 0.0f) eth_udp_command.torque_limit_ratio[i] = 0.0f;
    if (eth_udp_command.torque_limit_ratio[i] > 1.0f) eth_udp_command.torque_limit_ratio[i] = 1.0f;
    /* The whole-arm stream takes ownership from any previous one-shot v4
     * command.  The selected mode itself is changed only by SET_ARM_MODE. */
    dm_motor_runtime.override[i] = 0U;
    dm_motor_runtime.stopped[i] = 0U;
  }
  eth_udp_command.applied_sequence = packet->sequence;
  eth_udp_command.applied_mcu_us = rx_mcu_us;
  eth_udp_command.active = 1U;
  eth_udp_command.sequence_valid = 1U;
  taskEXIT_CRITICAL();

  AppEthernet_RecordTiming(&eth_udp_command.rx_count,
                           &eth_udp_command.interval_count,
                           &eth_udp_command.interval_min_us,
                           &eth_udp_command.interval_max_us,
                           &eth_udp_command.interval_sum_us,
                           &eth_udp_command.last_interval_us,
                           &eth_udp_command.jitter_sum_us,
                           &eth_udp_command.jitter_max_us,
                           &eth_udp_command.last_cycle,
                           (DM_ETH_COMMAND_HZ != 0U) ?
                           (1000000U / DM_ETH_COMMAND_HZ) : 0U);
}

static void AppEthernet_AcceptCartesianCommand(const InfrabotCartesianStream *packet)
{
  static uint32_t last_sequence;
  if (packet == NULL || packet->execute_mcu_us != 0U ||
      packet->command != DM_ETH_CMD_MIT ||
      eth_udp_command.control_mode != DM_ETH_CONTROL_RT_CARTESIAN ||
      !dm_motor_runtime.control_enabled || AppTasks_Enabling() || eth_udp_command.control_request ||
      AppMotion_Owned() || !AppControlMode_ProfileReady())
  {
    AppEthernet_Reject(DM_ETH_INVALID_CARTESIAN, packet ? packet->command : 0U);
    return;
  }
  /* Copied out: the packed member may be unaligned. */
  uint16_t wire_limits[4];
  memcpy(wire_limits, packet->cartesian_limits, sizeof(wire_limits));
  ArmCartesianLimits limits;
  if (!ArmMotion_CartesianLimitsFromWire(wire_limits, &limits))
  {
    AppEthernet_Reject(DM_ETH_INVALID_LIMITS, packet->command);
    return;
  }
  taskENTER_CRITICAL();
  if ((eth_udp_command.sequence_valid != 0U) &&
      (((packet->sequence - last_sequence) == 0U) ||
       ((packet->sequence - last_sequence) >= 0x80000000U)))
  {
    taskEXIT_CRITICAL();
    return;
  }
  last_sequence = packet->sequence;
  for (uint8_t i = 0U; i < 12U; ++i)
    eth_udp_command.cartesian_pose[i] = packet->pose[i];
  for (uint8_t i = 0U; i < 4U; ++i)
    eth_udp_command.cartesian_limits[i] = wire_limits[i];
  for (uint8_t i = 0U; i < DM_ARM_MOTOR_COUNT; ++i) /* J8: DM_ETH_CMD_GRIPPER */
  {
    eth_udp_command.target_velocity[i] = AppEthernet_ClampAbs(
        (float)packet->target_velocity_mrad_s[i] / 1000.0f,
        fminf(dm_motors[i].velocity_max,DM_ARM_MAX_SPEED_RAD_S));
    eth_udp_command.target_kp[i] = AppEthernet_ClampAbs(
        (float)packet->kp_milli[i] / 1000.0f, 500.0f);
    eth_udp_command.target_kd[i] = AppEthernet_ClampAbs(
        (float)packet->kd_milli[i] / 1000.0f, 5.0f);
    eth_udp_command.target_torque[i] = AppEthernet_ClampAbs(
        (float)packet->torque_mNm[i] / 1000.0f, dm_motors[i].torque_max);
    eth_udp_command.velocity_limit[i] = AppEthernet_ClampAbs(
        (float)packet->velocity_limit_mrad_s[i] / 1000.0f,
        fminf(dm_motors[i].velocity_max,DM_ARM_MAX_SPEED_RAD_S));
    eth_udp_command.torque_limit_ratio[i] = AppEthernet_ClampAbs(
        (float)packet->torque_ratio_milli[i] / 1000.0f, 1.0f);
    dm_motor_runtime.override[i] = 0U;
    dm_motor_runtime.stopped[i] = 0U;
  }
  eth_udp_command.last_sequence = packet->sequence;
  eth_udp_command.last_tick = HAL_GetTick();
  eth_udp_command.last_rx_mcu_us = AppEthernet_GetTimeUs();
  eth_udp_command.applied_sequence = packet->sequence;
  eth_udp_command.applied_mcu_us = eth_udp_command.last_rx_mcu_us;
  eth_udp_command.cartesian_target_valid = 1U;
  eth_udp_command.active = 1U;
  eth_udp_command.sequence_valid = 1U;
  taskEXIT_CRITICAL();

  AppEthernet_RecordTiming(&eth_udp_command.rx_count,
                           &eth_udp_command.interval_count,
                           &eth_udp_command.interval_min_us,
                           &eth_udp_command.interval_max_us,
                           &eth_udp_command.interval_sum_us,
                           &eth_udp_command.last_interval_us,
                           &eth_udp_command.jitter_sum_us,
                           &eth_udp_command.jitter_max_us,
                           &eth_udp_command.last_cycle,
                           (DM_ETH_COMMAND_HZ != 0U) ?
                           (1000000U / DM_ETH_COMMAND_HZ) : 0U);
}

static void AppEthernet_AcceptMaintenance(const InfrabotMaintenance *packet)
{
  static uint64_t last_timestamp;
  static uint32_t last_sequence;
  if (packet == NULL) return;
  /* A move owns the arm only: the gripper stays commandable. */
  if (AppMotion_Owned() && packet->command != DM_ETH_CMD_DISABLE &&
      packet->command != DM_ETH_CMD_STOP && packet->command != DM_ETH_CMD_GET_STATE &&
      packet->command != DM_ETH_CMD_SET_CONTROL_MODE &&
      packet->command != DM_ETH_CMD_GET_HOLD_POSITION &&
      packet->command != DM_ETH_CMD_GRIPPER) {
    AppEthernet_QueueMaintenanceResult(packet->sequence, packet->command, packet->motor_index,
                                      4U, packet->value_type, packet->register_id, 0U);
    return;
  }
  if ((packet->command < DM_ETH_CMD_ENABLE) || packet->command == DM_ETH_CMD_SET_ZERO_ENABLE ||
      packet->execute_mcu_us != 0U || packet->value_type > 1U ||
      (packet->command > DM_ETH_CMD_SET_CONTROL_MODE &&
       packet->command != DM_ETH_CMD_GET_HOLD_POSITION &&
       packet->command != DM_ETH_CMD_GRIPPER) ||
      (packet->command == DM_ETH_CMD_GET_HOLD_POSITION &&
       packet->motor_index >= DM_MOTOR_COUNT) ||
      (packet->command == DM_ETH_CMD_GRIPPER && packet->motor_index != DM_GRIPPER_INDEX) ||
      packet->command == DM_ETH_CMD_STORE_CONTROL_PROFILE ||
      (packet->command == DM_ETH_CMD_SET_ARM_MODE &&
       (packet->motor_index != DM_ETH_MOTOR_ALL ||
        (packet->mode != DM_ETH_MODE_MIT && packet->mode != DM_ETH_MODE_POS_VEL))) ||
      (packet->command == DM_ETH_CMD_SET_CONTROL_MODE &&
       (packet->motor_index != DM_ETH_MOTOR_ALL ||
        packet->mode > DM_ETH_CONTROL_CARTESIAN_ADMITTANCE)) ||
      ((packet->motor_index != DM_ETH_MOTOR_ALL) &&
       (packet->motor_index >= DM_MOTOR_COUNT)))
  {
    AppEthernet_Reject(DM_ETH_INVALID_MAINTENANCE, packet->command);
    AppEthernet_QueueMaintenanceResult(packet->sequence, packet->command,
                                        packet->motor_index, 3U,
                                        packet->value_type, packet->register_id, 0U);
    return;
  }

  taskENTER_CRITICAL();
  if (((eth_maintenance.pending || eth_maintenance.waiting) &&
       packet->command != DM_ETH_CMD_DISABLE) ||
      (last_timestamp == packet->pc_timestamp_us && last_sequence == packet->sequence))
  {
    taskEXIT_CRITICAL();
    AppEthernet_QueueMaintenanceResult(packet->sequence, packet->command, packet->motor_index,
                                       4U, packet->value_type, packet->register_id, 0U);
    return;
  }
  /* DISABLE ends a move with the drives; STOP holds the motor and makes a
   * move owning the arm a controlled stop. */
  if (packet->command == DM_ETH_CMD_DISABLE)
    AppMotion_RequestCancel();
  else if (packet->command == DM_ETH_CMD_STOP && AppMotion_Owned())
    AppSafety_RequestStop(INFRABOT_STOP_REQUESTED);
  last_timestamp = packet->pc_timestamp_us;
  last_sequence = packet->sequence;
  eth_maintenance.pending = 1U;
  eth_maintenance.command = packet->command;
  eth_maintenance.motor_index = packet->motor_index;
  eth_maintenance.mode = packet->mode;
  eth_maintenance.value_type = packet->value_type;
  eth_maintenance.register_id = packet->register_id;
  eth_maintenance.value = packet->register_value;
  eth_maintenance.position_mrad = packet->target_position_mrad[
      (packet->motor_index < DM_MOTOR_COUNT) ? packet->motor_index : 0U];
  eth_maintenance.velocity_mrad_s = packet->target_velocity_mrad_s;
  eth_maintenance.kp_milli = packet->kp_milli;
  eth_maintenance.kd_milli = packet->kd_milli;
  eth_maintenance.torque_mNm = packet->torque_mNm;
  eth_maintenance.velocity_limit_mrad_s = packet->velocity_limit_mrad_s;
  eth_maintenance.torque_ratio_milli = packet->torque_ratio_milli;
  eth_maintenance.sequence = packet->sequence;
  eth_maintenance.pending_tick = HAL_GetTick();
  taskEXIT_CRITICAL();
}

static void AppEthernet_CommandReceive(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                                       const ip_addr_t *addr, u16_t port)
{
  (void)arg;
  (void)pcb;
  /* Commands are unicast.  A LAN router (192.168.0.1 here) broadcasts
   * 380-byte datagrams to this port every ~10 s; drop broadcasts and
   * multicasts unseen instead of counting them as invalid commands. */
  if (p != NULL && (ip_addr_isbroadcast(ip_current_dest_addr(), ip_current_netif()) ||
                    ip_addr_ismulticast(ip_current_dest_addr())))
  {
    pbuf_free(p);
    return;
  }
  if (p != NULL)
  {
    if (AppEthernet_TryInfo(p, addr, port) || AppEthernet_TryParam(p, addr, port))
    {
      pbuf_free(p);
      return;
    }
    AppMotionRequest motion_request;
    if (p->tot_len == sizeof(motion_request) &&
        pbuf_copy_partial(p, &motion_request, sizeof(motion_request), 0U) == sizeof(motion_request) &&
        motion_request.magic == INFRABOT_MAGIC_MOVE_REQUEST && motion_request.version == INFRABOT_MOVE_V1 &&
        motion_request.reserved == 0U &&
        infrabot_crc32((const uint8_t *)&motion_request, sizeof(motion_request) - 4U) == motion_request.crc32) {
      /* A full mailbox drops this datagram. The PC retries the SAME id at
       * 100 Hz; no unbounded queue of future robot movements is accumulated. */
      if (AppEthernet_ControlOwnerAllowed(addr, port, DM_ETH_CMD_MIT))
        (void)AppMotion_Publish(&motion_request, ip4_addr_get_u32(ip_2_ip4(addr)), port);
      else
        AppEthernet_OwnerReject(motion_request.motion_id, DM_ETH_CMD_MIT);
      pbuf_free(p);
      return;
    }
    InfrabotCommand packet;
    InfrabotCommandV2 packet_v2;
    InfrabotJointStream packet_arm;
    InfrabotCartesianStream packet_cartesian;
    InfrabotMaintenance packet_v4;
    uint8_t valid = 0U;
    uint8_t command = DM_ETH_CMD_MIT;
    uint32_t sequence = 0U;
    uint64_t execute_mcu_us = 0U;
    int32_t targets[DM_MOTOR_COUNT] = {0};
    if ((p->tot_len == sizeof(packet_cartesian)) &&
        (pbuf_copy_partial(p, &packet_cartesian, sizeof(packet_cartesian), 0U) == sizeof(packet_cartesian)) &&
        (packet_cartesian.magic == INFRABOT_MAGIC_COMMAND) && (packet_cartesian.version == INFRABOT_COMMAND_V7) &&
        (packet_cartesian.motor_count == DM_MOTOR_COUNT) &&
        (infrabot_crc32((const uint8_t *)&packet_cartesian,
                           sizeof(packet_cartesian) - sizeof(packet_cartesian.crc32)) == packet_cartesian.crc32))
    {
      if (AppEthernet_ControlOwnerAllowed(addr, port, packet_cartesian.command))
        AppEthernet_AcceptCartesianCommand(&packet_cartesian);
      else
        AppEthernet_OwnerReject(packet_cartesian.sequence, packet_cartesian.command);
      pbuf_free(p);
      return;
    }
    else if ((p->tot_len == sizeof(packet_arm)) &&
        (pbuf_copy_partial(p, &packet_arm, sizeof(packet_arm), 0U) == sizeof(packet_arm)) &&
        (packet_arm.magic == INFRABOT_MAGIC_COMMAND) && (packet_arm.version == INFRABOT_COMMAND_V5) &&
        (packet_arm.motor_count == DM_MOTOR_COUNT) &&
        (infrabot_crc32((const uint8_t *)&packet_arm,
                           sizeof(packet_arm) - sizeof(packet_arm.crc32)) == packet_arm.crc32))
    {
      if (AppEthernet_ControlOwnerAllowed(addr, port, packet_arm.command))
        AppEthernet_AcceptArmCommand(&packet_arm);
      else
        AppEthernet_OwnerReject(packet_arm.sequence, packet_arm.command);
      pbuf_free(p);
      return;
    }
    else if ((p->tot_len == sizeof(packet_v4)) &&
        (pbuf_copy_partial(p, &packet_v4, sizeof(packet_v4), 0U) == sizeof(packet_v4)) &&
        (packet_v4.magic == INFRABOT_MAGIC_COMMAND) && (packet_v4.version == INFRABOT_COMMAND_V4) &&
        (packet_v4.motor_count == DM_MOTOR_COUNT) &&
        (infrabot_crc32((const uint8_t *)&packet_v4,
                           sizeof(packet_v4) - sizeof(packet_v4.crc32)) == packet_v4.crc32))
    {
      const uint8_t read_only = (packet_v4.command == DM_ETH_CMD_GET_REGISTER) ||
                                (packet_v4.command == DM_ETH_CMD_GET_STATE) ||
                                (packet_v4.command == DM_ETH_CMD_REQUEST_FEEDBACK) ||
                                (packet_v4.command == DM_ETH_CMD_GET_HOLD_POSITION);
      if (read_only || AppEthernet_ControlOwnerAllowed(addr, port, packet_v4.command))
        AppEthernet_AcceptMaintenance(&packet_v4);
      else
        AppEthernet_OwnerReject(packet_v4.sequence, packet_v4.command);
      pbuf_free(p);
      return;
    }
    else if ((p->tot_len == sizeof(packet)) &&
        (pbuf_copy_partial(p, &packet, sizeof(packet), 0U) == sizeof(packet)) &&
        (packet.magic == INFRABOT_MAGIC_COMMAND) && (packet.version == INFRABOT_COMMAND_V3) &&
        (packet.motor_count == DM_MOTOR_COUNT) &&
        (infrabot_crc32((const uint8_t *)&packet, sizeof(packet) - sizeof(packet.crc32)) == packet.crc32))
    {
      valid = 1U;
      command = packet.command;
      sequence = packet.sequence;
      execute_mcu_us = packet.execute_mcu_us;
      for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++) targets[i] = packet.target_position_mrad[i];
    }
    else if ((p->tot_len == sizeof(packet_v2)) &&
             (pbuf_copy_partial(p, &packet_v2, sizeof(packet_v2), 0U) == sizeof(packet_v2)) &&
             (packet_v2.magic == INFRABOT_MAGIC_COMMAND) && (packet_v2.version == INFRABOT_COMMAND_V2) &&
             (packet_v2.motor_count == DM_MOTOR_COUNT) &&
             (infrabot_crc32((const uint8_t *)&packet_v2,
                                sizeof(packet_v2) - sizeof(packet_v2.crc32)) == packet_v2.crc32))
    {
      valid = 1U;
      sequence = packet_v2.sequence;
      execute_mcu_us = packet_v2.execute_mcu_us;
      for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++) targets[i] = packet_v2.target_position_mrad[i];
    }
    if (valid != 0U)
    {
      if (AppEthernet_ControlOwnerAllowed(addr, port, command))
        AppEthernet_AcceptCommand(sequence, command, execute_mcu_us, targets);
      else
        AppEthernet_OwnerReject(sequence, command);
    }
    else
    {
      AppEthernet_Reject(DM_ETH_INVALID_FORMAT, p->tot_len);
    }
    pbuf_free(p);
  }
}

void AppEthernet_SetLinkStats(const InfrabotLinkStats *stats)
{
  taskENTER_CRITICAL();
  link_stats = *stats;
  taskEXIT_CRITICAL();
}

/* DMTL v3: the measured state, what the MCU commands each drive, the link
 * statistics of the previous second and the safety status.  Also the run
 * log's 1 kHz record (app_log_task.c, from another task: every shared read is
 * inside the critical sections; sequence is the caller's). */
static void AppEthernet_FillPacket(InfrabotTelemetryV3 *packet)
{
  InfrabotTelemetry *head = &packet->head;
  memset(packet, 0, sizeof(*packet));
  head->magic = INFRABOT_MAGIC_TELEMETRY;
  head->version = INFRABOT_TELEMETRY_V3;
  head->motor_count = DM_MOTOR_COUNT;
  head->timestamp_ms = HAL_GetTick();
  head->mcu_timestamp_us = AppEthernet_GetTimeUs();
  const uint32_t now = head->timestamp_ms;
  taskENTER_CRITICAL();
  const uint8_t torque_limit = AppSafety_TorqueLimitMask();
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
  {
    const float direction = DM_MOTOR_DIRECTION(i);
    InfrabotTelemetryMotor *motor = &head->motor[i];
    motor->position_mrad = (int32_t)((dm_motors[i].position - dm_motor_runtime.zero_offset[i]) *
                                     direction * 1000.0f);
    motor->velocity_mrad_s = (int32_t)(dm_motors[i].velocity * direction * 1000.0f);
    motor->torque_mNm = (int32_t)(dm_motors[i].torque * direction * 1000.0f);
    motor->state = dm_motors[i].state;
    motor->temperature_mos = dm_motors[i].temperature_mos;
    motor->feedback_count = (uint16_t)dm_motors[i].feedback_count;

    /* The planned command (position-velocity: before ArmDriveLead), its
     * velocity term (MIT feed-forward, else the drive's v_des) and the MIT
     * torque feed-forward, all joint frame. */
    InfrabotTelemetryCommand *command = &packet->command[i];
    const uint8_t mode = eth_udp_command.motor_mode[i];
    const uint8_t stopped = dm_motor_runtime.stopped[i];
    const float velocity = (mode == DM_ETH_MODE_POS_VEL || mode == DM_ETH_MODE_FORCE_POS) ?
                           eth_udp_command.velocity_limit[i] :
                           (stopped ? 0.0f : eth_udp_command.target_velocity[i]);
    command->position_mrad = (int32_t)((dm_motor_runtime.command_position[i] - dm_motor_runtime.zero_offset[i]) *
                                       direction * 1000.0f);
    command->velocity_mrad_s = (int32_t)(velocity * 1000.0f);
    command->torque_mNm = (mode == DM_ETH_MODE_MIT && !stopped) ?
                          (int32_t)(eth_udp_command.target_torque[i] * 1000.0f) : 0;
    command->temperature_coil = dm_motors[i].temperature_coil;
    command->drive_mode = mode;
    uint8_t flags = 0U;
    if (dm_motor_runtime.enabled[i] != 0U)
    {
      flags |= INFRABOT_MOTOR_ENABLED;
      if (dm_motors[i].feedback_count == 0U ||
          (int32_t)(now - dm_motors[i].feedback_tick) > (int32_t)ARM_MOVE_FEEDBACK_MS)
        flags |= INFRABOT_MOTOR_FEEDBACK_STALE;
    }
    if (dm_motor_runtime.override[i] != 0U) flags |= INFRABOT_MOTOR_OVERRIDE;
    if (stopped != 0U) flags |= INFRABOT_MOTOR_STOPPED;
    if ((torque_limit & (1U << i)) != 0U) flags |= INFRABOT_MOTOR_TORQUE_LIMIT;
    command->flags = flags;
  }
  taskEXIT_CRITICAL();
  taskENTER_CRITICAL();
  head->command_last_sequence = eth_udp_command.last_sequence;
  head->command_applied_sequence = eth_udp_command.applied_sequence;
  head->command_active = eth_udp_command.active;
  head->status = (uint8_t)((dm_motor_runtime.control_enabled != 0U) ? INFRABOT_STATUS_FLAG_ENABLED : 0U);
  head->status |= (uint8_t)((dm_motor_runtime.zero_valid != 0U) ? INFRABOT_STATUS_FLAG_ZERO_VALID : 0U);
  /* The controller-level operational state in bits 2..7. */
  head->status |= (uint8_t)((RobotController_OperationalStatus() & 0x3FU) << 2U);
  head->enabled_mask = 0U;
  for (uint8_t i = 0U; i < DM_MOTOR_COUNT; i++)
    if (dm_motor_runtime.enabled[i] != 0U) head->enabled_mask |= (uint8_t)(1U << i);
  /* Capability flag, configured motor profile, and authoritative mode. */
  head->mode = AppControlMode_Telemetry();
  head->command_last_rx_mcu_us = eth_udp_command.last_rx_mcu_us;
  head->command_applied_mcu_us = eth_udp_command.applied_mcu_us;
  packet->link = link_stats;
  AppSafety_Report(&packet->safety);
  AppDamiao_ReportSelfCheck(&packet->safety);
  taskEXIT_CRITICAL();
}

void AppEthernet_FillTelemetry(InfrabotTelemetryV3 *packet)
{
  AppEthernet_FillPacket(packet);
}

static void AppEthernet_LogReceive(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                                   const ip_addr_t *addr, u16_t port)
{
  (void)arg;
  if (p == NULL) return;
  /* One request in, at most one reply out, in this (the Ethernet) task.
   * Static: 1.5 KB is too much for the task stack, and only this task uses it. */
  static uint8_t request[sizeof(InfrabotLogRequest)];
  static uint8_t reply[sizeof(InfrabotLogReplyHeader) + INFRABOT_LOG_DATA_MAX];
  size_t length = 0U;
  if (p->tot_len == sizeof(request) &&
      pbuf_copy_partial(p, request, sizeof(request), 0U) == sizeof(request))
    length = AppLogTask_Serve(request, sizeof(request), reply);
  pbuf_free(p);
  if (length == 0U || !netif_is_link_up(&eth_netif)) return;
  struct pbuf *out = pbuf_alloc(PBUF_TRANSPORT, (u16_t)length, PBUF_RAM);
  if (out == NULL) return;
  memcpy(out->payload, reply, length);
  (void)udp_sendto(pcb, out, addr, port);
  pbuf_free(out);
}

static void AppEthernet_SendTelemetry(void)
{
  InfrabotTelemetryV3 packet;
  struct pbuf *p;
  if ((telemetry_client_valid == 0U) || !netif_is_up(&eth_netif) || !netif_is_link_up(&eth_netif))
    return;
  AppEthernet_FillPacket(&packet);
  packet.head.sequence = telemetry_sequence++;
  p = pbuf_alloc(PBUF_TRANSPORT, sizeof(packet), PBUF_RAM);
  if (p == NULL)
  {
    eth_udp_tx_stats.errors++;
    return;
  }
  memcpy(p->payload, &packet, sizeof(packet));
  err_t result = udp_sendto(telemetry_pcb, p, &telemetry_client_ip, telemetry_client_port);
  if (result == ERR_OK)
  {
    AppEthernet_RecordTiming(&eth_udp_tx_stats.count,
                             &eth_udp_tx_stats.interval_count,
                             &eth_udp_tx_stats.interval_min_us,
                             &eth_udp_tx_stats.interval_max_us,
                             &eth_udp_tx_stats.interval_sum_us,
                             &eth_udp_tx_stats.last_interval_us,
                             &eth_udp_tx_stats.jitter_sum_us,
                             &eth_udp_tx_stats.jitter_max_us,
                             &eth_udp_tx_stats.last_cycle,
                             (DM_ETH_TELEMETRY_HZ != 0U) ?
                             (1000000U / DM_ETH_TELEMETRY_HZ) : 0U);
  }
  else
  {
    eth_udp_tx_stats.errors++;
  }
  pbuf_free(p);
}

static void AppEthernet_SendMaintenanceResult(void)
{
  InfrabotMaintenanceReply response;
  struct pbuf *p;
  if ((maintenance_response_head == maintenance_response_tail) ||
      (telemetry_client_valid == 0U) ||
      !netif_is_up(&eth_netif) || !netif_is_link_up(&eth_netif))
    return;
  taskENTER_CRITICAL();
  response = maintenance_responses[maintenance_response_tail++ % MAINTENANCE_RESPONSE_DEPTH];
  taskEXIT_CRITICAL();
  p = pbuf_alloc(PBUF_TRANSPORT, sizeof(response), PBUF_RAM);
  if (p == NULL)
  {
    eth_udp_tx_stats.errors++;
    return;
  }
  memcpy(p->payload, &response, sizeof(response));
  if (udp_sendto(telemetry_pcb, p, &telemetry_client_ip, telemetry_client_port) != ERR_OK)
    eth_udp_tx_stats.errors++;
  pbuf_free(p);
}

static void AppEthernet_SendMotionReply(void)
{
  AppMotionReply reply;
  uint32_t ip;
  uint16_t port;
  if (!AppMotion_TakeReply(&reply, &ip, &port)) return;
  reply.crc32 = infrabot_crc32((const uint8_t *)&reply, sizeof(reply) - 4U);
  struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(reply), PBUF_RAM);
  if (p == NULL) { eth_udp_tx_stats.errors++; return; }
  ip_addr_t destination;
  ip_addr_set_ip4_u32(&destination, ip);
  memcpy(p->payload, &reply, sizeof(reply));
  /* Reply to the requesting socket, not to the GUI's shared telemetry client. */
  if (udp_sendto(command_pcb, p, &destination, port) != ERR_OK) eth_udp_tx_stats.errors++;
  pbuf_free(p);
}

static void AppEthernet_Task(void *argument)
{
  (void)argument;
  ip4_addr_t ip, netmask, gateway;
  TickType_t wake = xTaskGetTickCount();
  TickType_t link_tick = wake;
  TickType_t health_tick = wake;
  TickType_t telemetry_tick = wake;
  uint8_t last_link_state = 0xFFU;
  uint8_t phy_reported = 0U;
#if DM_ETH_USE_DHCP
  uint8_t dhcp_running = 0U;
  uint8_t dhcp_reported = 0U;
#endif
  const TickType_t period = (DM_ETH_TELEMETRY_HZ >= configTICK_RATE_HZ) ? 1U :
                            pdMS_TO_TICKS(1000U / DM_ETH_TELEMETRY_HZ);
  const TickType_t link_monitor_period = pdMS_TO_TICKS(20U);
  /* Health is a 60 s heartbeat; a rising error counter prints at once. */
  const TickType_t health_period = pdMS_TO_TICKS(60000U);
  uint32_t reported_errors = 0U;

  lwip_init();
  /* A static address saved from the LCD overrides the compiled default. */
  AppNetConfig stored;
  if (AppPersist_LoadNetwork(&stored) != 0U)
  {
    eth_address_from_flash = 1U;
    IP4_ADDR(&ip, stored.ip[0], stored.ip[1], stored.ip[2], stored.ip[3]);
    IP4_ADDR(&netmask, stored.netmask[0], stored.netmask[1], stored.netmask[2], stored.netmask[3]);
    IP4_ADDR(&gateway, stored.gateway[0], stored.gateway[1], stored.gateway[2], stored.gateway[3]);
  }
  else
  {
    IP4_ADDR(&ip, DM_ETH_IP_ADDR0, DM_ETH_IP_ADDR1, DM_ETH_IP_ADDR2, DM_ETH_IP_ADDR3);
    IP4_ADDR(&netmask, DM_ETH_NETMASK0, DM_ETH_NETMASK1, DM_ETH_NETMASK2, DM_ETH_NETMASK3);
    IP4_ADDR(&gateway, DM_ETH_GW_ADDR0, DM_ETH_GW_ADDR1, DM_ETH_GW_ADDR2, DM_ETH_GW_ADDR3);
  }
#if DM_ETH_USE_DHCP
  ip_addr_set_zero_ip4(&ip);
  ip_addr_set_zero_ip4(&netmask);
  ip_addr_set_zero_ip4(&gateway);
#endif
  if (netif_add(&eth_netif, &ip, &netmask, &gateway, NULL,
                ethernetif_init, ethernet_input) == NULL)
  {
    /* A malformed/undersized LwIP pool must not take down the scheduler.
     * Leave CAN and UART tasks running and retry network setup only after a
     * future reset. */
    printf("ETH: netif_add FAILED; Ethernet disabled, CAN control continues\r\n");
    for (;;)
      vTaskDelay(pdMS_TO_TICKS(1000U));
  }
  netif_set_default(&eth_netif);
#ifdef FW_NETBOOT_APP
  if(!fw_service_start())printf("ETH: firmware update service unavailable\r\n");
#endif
#if DM_ETH_USE_DHCP
  if (netif_is_link_up(&eth_netif) != 0)
  {
    netif_set_up(&eth_netif);
    if (dhcp_start(&eth_netif) == ERR_OK) dhcp_running = 1U;
  }
  else
  {
    netif_set_down(&eth_netif);
  }
#else
  /* Keep both the netif and MAC down when no cable was detected at boot.
   * ethernet_link_check_state() starts them together after a hot-plug. */
  if (netif_is_link_up(&eth_netif) != 0) netif_set_up(&eth_netif);
  else netif_set_down(&eth_netif);
#endif
#ifdef FW_NETBOOT_APP
  telemetry_pcb = fw_is_trial()?NULL:udp_new();
#else
  telemetry_pcb = udp_new();
#endif
  if (telemetry_pcb != NULL)
  {
    (void)udp_bind(telemetry_pcb, IP_ADDR_ANY, DM_ETH_UDP_PORT);
    udp_recv(telemetry_pcb, AppEthernet_Receive, NULL);
  }
#ifdef FW_NETBOOT_APP
  command_pcb = fw_is_trial()?NULL:udp_new();
#else
  command_pcb = udp_new();
#endif
  if (command_pcb != NULL)
  {
    (void)udp_bind(command_pcb, IP_ADDR_ANY, DM_ETH_COMMAND_UDP_PORT);
    udp_recv(command_pcb, AppEthernet_CommandReceive, NULL);
  }
#ifdef FW_NETBOOT_APP
  log_pcb = fw_is_trial()?NULL:udp_new();
#else
  log_pcb = udp_new();
#endif
  /* The 4th of MEMP_NUM_UDP_PCB (update service, telemetry, command, log). */
  if (log_pcb != NULL)
  {
    (void)udp_bind(log_pcb, IP_ADDR_ANY, INFRABOT_LOG_PORT);
    udp_recv(log_pcb, AppEthernet_LogReceive, NULL);
  }
#ifdef FW_NETBOOT_APP
  else if (!fw_is_trial())
#else
  else
#endif
    printf("ETH: run log port %u unavailable (no UDP PCB)\r\n", (unsigned)INFRABOT_LOG_PORT);
  printf("ETH: RMII LAN8720A UDP telemetry=%uHz port=%u command_port=%u mode=%s\r\n",
         DM_ETH_TELEMETRY_HZ, DM_ETH_UDP_PORT, DM_ETH_COMMAND_UDP_PORT,
#if DM_ETH_USE_DHCP
         "DHCP"
#else
         "STATIC"
#endif
         );
  printf("ETH: address source=%s\r\n",
         (eth_address_from_flash != 0U) ? "MCU Flash (set on LCD)" : "compiled default");
  AppEthernet_PrintAddress("IP", ip_2_ip4(&eth_netif.ip_addr));
  AppEthernet_PrintAddress("MASK", ip_2_ip4(&eth_netif.netmask));
  AppEthernet_PrintAddress("GW", ip_2_ip4(&eth_netif.gw));

  /* This task has a higher startup priority than the motor bring-up task,
   * therefore this initial result is known before motor probing begins. */
  ethernet_link_check_state(&eth_netif);
  last_link_state = (netif_is_link_up(&eth_netif) != 0) ? 1U : 0U;
  printf("ETH: boot probe link=%s network=%s hotplug=ON\r\n",
         (last_link_state != 0U) ? "UP" : "DOWN",
         (last_link_state != 0U) ? "STARTED" : "DEFERRED");
  if (last_link_state != 0U)
  {
    etharp_gratuitous(&eth_netif);
    printf("ETH: ARP announce rx=%lu tx=%lu tx_err=%lu\r\n",
           (unsigned long)ethernetif_rx_packets,
           (unsigned long)ethernetif_tx_packets,
           (unsigned long)ethernetif_tx_errors);
  }

  for (;;)
  {
#ifdef FW_NETBOOT_APP
    fw_swd_poll();
#endif
    if (eth_pending_address_valid != 0U)
    {
      AppNetConfig address;
      ip4_addr_t new_ip, new_mask, new_gw;
      taskENTER_CRITICAL();
      address = eth_pending_address;
      eth_pending_address_valid = 0U;
      taskEXIT_CRITICAL();
      IP4_ADDR(&new_ip, address.ip[0], address.ip[1], address.ip[2], address.ip[3]);
      IP4_ADDR(&new_mask, address.netmask[0], address.netmask[1], address.netmask[2],
               address.netmask[3]);
      IP4_ADDR(&new_gw, address.gateway[0], address.gateway[1], address.gateway[2],
               address.gateway[3]);
      netif_set_addr(&eth_netif, &new_ip, &new_mask, &new_gw);
      /* The PC talks to the old address: it has to register again. */
      telemetry_client_valid = 0U;
      eth_address_from_flash = 1U;
      if (netif_is_link_up(&eth_netif) != 0) etharp_gratuitous(&eth_netif);
      printf("ETH: address changed from LCD\r\n");
      AppEthernet_PrintAddress("IP", &new_ip);
      AppEthernet_PrintAddress("MASK", &new_mask);
      AppEthernet_PrintAddress("GW", &new_gw);
    }
    if ((xTaskGetTickCount() - link_tick) >= link_monitor_period)
    {
      ethernet_link_check_state(&eth_netif);
      link_tick = xTaskGetTickCount();
      uint8_t link_state = (netif_is_link_up(&eth_netif) != 0) ? 1U : 0U;
      if (link_state != last_link_state)
      {
        last_link_state = link_state;
        printf("ETH: link=%s netif=%s\r\n",
               (link_state != 0U) ? "UP" : "DOWN",
               (netif_is_up(&eth_netif) != 0) ? "UP" : "DOWN");
        if (link_state != 0U)
        {
          uint32_t speed_mbps = 0U;
          uint8_t full_duplex = 0U;
          if (ethernetif_get_link_info(&speed_mbps, &full_duplex) != 0U)
          {
            printf("ETH: negotiated speed=%luMbps duplex=%s\r\n",
                   (unsigned long)speed_mbps,
                   (full_duplex != 0U) ? "FULL" : "HALF");
          }
          else
          {
            printf("ETH: negotiated speed=UNKNOWN\r\n");
          }
#if DM_ETH_USE_DHCP
          if (dhcp_running == 0U)
          {
            if (dhcp_start(&eth_netif) == ERR_OK) dhcp_running = 1U;
            dhcp_reported = 0U;
          }
#endif
          /* Announce the static address immediately.  Besides reducing ARP
           * latency on a direct cable, this proves the MAC TX path before a
           * host sends its first ping. */
          etharp_gratuitous(&eth_netif);
          printf("ETH: ARP announce rx=%lu tx=%lu tx_err=%lu\r\n",
                 (unsigned long)ethernetif_rx_packets,
                 (unsigned long)ethernetif_tx_packets,
                 (unsigned long)ethernetif_tx_errors);
        }
#if DM_ETH_USE_DHCP
        else if (dhcp_running != 0U)
        {
          dhcp_stop(&eth_netif);
          dhcp_running = 0U;
          dhcp_reported = 0U;
        }
#endif
      }
      if ((link_state == 0U) && (phy_reported == 0U))
      {
        uint32_t phy_addr, phy_id1, phy_id2, bcr, bsr, phyc_sr;
        if (ethernetif_get_phy_diagnostics(&phy_addr, &phy_id1, &phy_id2,
                                           &bcr, &bsr, &phyc_sr) != 0U)
        {
          printf("ETH: PHY addr=%lu id1=0x%04lX id2=0x%04lX "
                 "BCR=0x%04lX BSR=0x%04lX PHYSCSR=0x%04lX HALerr=0x%08lX\r\n",
                 (unsigned long)phy_addr, (unsigned long)phy_id1,
                 (unsigned long)phy_id2, (unsigned long)bcr,
                 (unsigned long)bsr, (unsigned long)phyc_sr,
                 (unsigned long)EthHandle.ErrorCode);
        }
        else
        {
          printf("ETH: PHY diagnostics unavailable HALerr=0x%08lX\r\n",
                 (unsigned long)EthHandle.ErrorCode);
        }
        phy_reported = 1U;
      }
      if (link_state != 0U) phy_reported = 0U;
#if DM_ETH_USE_DHCP
      if (dhcp_supplied_address(&eth_netif) != 0U)
      {
        if (dhcp_reported == 0U)
        {
          dhcp_reported = 1U;
          AppEthernet_PrintAddress("DHCP IP", ip_2_ip4(&eth_netif.ip_addr));
          AppEthernet_PrintAddress("DHCP MASK", ip_2_ip4(&eth_netif.netmask));
          AppEthernet_PrintAddress("DHCP GW", ip_2_ip4(&eth_netif.gw));
        }
      }
#endif
    }

    const uint32_t errors = ethernetif_rx_alloc_errors + ethernetif_tx_errors +
                            (EthHandle.ErrorCode != 0U) + (EthHandle.DMAErrorCode != 0U);
    const TickType_t health_age = xTaskGetTickCount() - health_tick;
    if (health_age >= health_period ||
        (errors != reported_errors && health_age >= pdMS_TO_TICKS(1000U)))
    {
      reported_errors = errors;
      uint32_t phy_addr, phy_id1, phy_id2, bcr, bsr, phyc_sr;
      uint8_t phy_ok = ethernetif_get_phy_diagnostics(
        &phy_addr, &phy_id1, &phy_id2, &bcr, &bsr, &phyc_sr);
      printf("ETH: health link=%s mac=0x%02lX phy=%s BSR=0x%04lX "
             "rx=%lu rx_nomem=%lu tx=%lu tx_err=%lu HALerr=0x%08lX "
             "DMAerr=0x%08lX DMACSR=0x%08lX\r\n",
             (netif_is_link_up(&eth_netif) != 0) ? "UP" : "DOWN",
             (unsigned long)EthHandle.gState,
             (phy_ok != 0U) ? "OK" : "ERR",
             (unsigned long)bsr,
             (unsigned long)ethernetif_rx_packets,
             (unsigned long)ethernetif_rx_alloc_errors,
             (unsigned long)ethernetif_tx_packets,
             (unsigned long)ethernetif_tx_errors,
             (unsigned long)EthHandle.ErrorCode,
             (unsigned long)EthHandle.DMAErrorCode,
             (unsigned long)ETH->DMACSR);
      health_tick = xTaskGetTickCount();
    }

    /* Only poll RX/timeouts after the link checker has started the MAC.
     * While the cable/PHY is down this task is just a lightweight link
     * monitor and cannot consume DMA or control-loop time. */
    if (netif_is_link_up(&eth_netif) != 0)
    {
      ethernetif_input(&eth_netif);
      sys_check_timeouts();
      AppEthernet_SendMaintenanceResult();
      AppEthernet_SendMotionReply();
      TickType_t now = xTaskGetTickCount();
      if ((now - telemetry_tick) >= period)
      {
        /* Keep telemetry periodic even if RX interrupts wake this task more
         * than once per millisecond. */
        telemetry_tick = now;
        AppEthernet_SendTelemetry();
      }

      /* RX interrupts now wake this task immediately.  The one-tick timeout
       * remains as a safety poll and services LwIP timers/UDP telemetry when
       * no inbound traffic is present. */
      (void)ulTaskNotifyTake(pdTRUE, period);
    }
    else
    {
      /* Poll PHY negotiation at 20 ms while carrier is down. */
      (void)ulTaskNotifyTake(pdTRUE, link_monitor_period);
      wake = xTaskGetTickCount();
      telemetry_tick = wake;
    }
  }
}

uint8_t AppEthernet_LinkUp(void)
{
  return (netif_is_link_up(&eth_netif) != 0) ? 1U : 0U;
}

void AppEthernet_RequestAddress(const AppNetConfig *config)
{
  taskENTER_CRITICAL();
  eth_pending_address = *config;
  eth_pending_address_valid = 1U;
  taskEXIT_CRITICAL();
  if (dm_ethernet_task_handle != NULL) xTaskNotifyGive(dm_ethernet_task_handle);
}

void AppEthernet_GetInfo(AppEthernetInfo *info)
{
  const ip4_addr_t *ip = ip_2_ip4(&eth_netif.ip_addr);
  const ip4_addr_t *mask = ip_2_ip4(&eth_netif.netmask);
  const ip4_addr_t *gw = ip_2_ip4(&eth_netif.gw);

  memset(info, 0, sizeof(*info));
  info->address.ip[0] = ip4_addr1(ip); info->address.ip[1] = ip4_addr2(ip);
  info->address.ip[2] = ip4_addr3(ip); info->address.ip[3] = ip4_addr4(ip);
  info->address.netmask[0] = ip4_addr1(mask); info->address.netmask[1] = ip4_addr2(mask);
  info->address.netmask[2] = ip4_addr3(mask); info->address.netmask[3] = ip4_addr4(mask);
  info->address.gateway[0] = ip4_addr1(gw); info->address.gateway[1] = ip4_addr2(gw);
  info->address.gateway[2] = ip4_addr3(gw); info->address.gateway[3] = ip4_addr4(gw);
  memcpy(info->mac, eth_netif.hwaddr, sizeof(info->mac));
  info->link_up = (netif_is_link_up(&eth_netif) != 0) ? 1U : 0U;
  info->from_flash = eth_address_from_flash;
  info->pending = eth_pending_address_valid;
  info->client_valid = telemetry_client_valid;
  if (info->client_valid != 0U)
  {
    const ip4_addr_t *client = ip_2_ip4(&telemetry_client_ip);
    info->client_ip[0] = ip4_addr1(client); info->client_ip[1] = ip4_addr2(client);
    info->client_ip[2] = ip4_addr3(client); info->client_ip[3] = ip4_addr4(client);
    info->client_port = telemetry_client_port;
  }
}

BaseType_t AppEthernet_CreateTask(void)
{
  return xTaskCreate(AppEthernet_Task, "dm_eth", DM_ETH_TASK_STACK, NULL,
                     DM_ETH_TASK_PRIORITY, &dm_ethernet_task_handle);
}
