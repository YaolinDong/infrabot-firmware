# 100 Hz 非实时运动：movej / movep

## 执行语义

`movej()` 是 **PC 调用者阻塞、MCU 状态机非阻塞** 的接口：

1. PC 每 10 ms 发送同一个 motion ID、同一组不可变参数，直到 MCU 返回完成。
2. MCU 只执行一条运动。七轴共用时长的五次多项式轨迹，每个 1 ms 控制 tick 更新位置和速度，再走原有 CAN 发送路径。
3. 轨迹结束后仍检查实际反馈。J1–J7 全部位置误差和速度达标，连续保持指定时间，才置 `DONE`。
4. PC 收到当前 session、当前 motion ID 的 `DONE` 后，`movej()` 才返回；调用者这时才能发下一条。

100 Hz 是命令重发/保活/状态获取频率，**不是每秒启动 100 条独立运动**。执行中的不同 motion ID 返回 `BUSY`，不会覆盖目标，也不积压隐式动作队列。重复当前命令只续保活，不重启轨迹；同 ID 修改参数会被拒绝。丢失完成应答时，重复同 ID 仍可读取锁存的完成状态。

`movep()` 现在复用相同的串行执行语义：MCU 在每个 1 kHz tick 调用固定尺寸、无动态分配的
Infrabot IK step，以当前七轴反馈作为 seed，并把最后一个有效近似解送入原有 CAN 路径。
超过 500 μs 预算时保留本周期之前的有效解；输入非法或 QP 数值失败才失败，不做碰撞包络检测。

## 模式与参数

先阅读[分层模式协议](control-modes.md)：使能与运动分离，DMOV 关节目标在 NRT_JOINT 执行，
笛卡尔目标在 NRT_CARTESIAN 执行。

- J1–J7 必须已显式使能、静止、有新鲜反馈，且全部处于 MIT 或全部处于位置速度模式。接口不隐式使能、切换模式、校零或保存 Flash。
- MIT 使用规划位置/速度及逐轴 Kp、Kd、前馈力矩；位置速度模式使用规划位置及逐轴速度上限。两种模式都做 MCU 到位判断。
- J8 保留已有的力位混合目标，不参与七轴轨迹和到位判断。本版本在 move session 占用期间拒绝会改变控制状态的维护操作（STOP/DISABLE 除外）。
- 七轴采用 `s(u)=10u³−15u⁴+6u⁵`，每段从静止到静止，不做段间混合。时长至少满足每轴 `1.875*abs(delta_q)/v_max`、`sqrt(5.774*abs(delta_q)/a_max)`，向上取整到毫秒。
- move 的规划位置不再经过旧实时流固定 0.5 rad/s 斜坡，避免双重插值。DMCP 实时流仅在 RT_JOINT 接受，不会覆盖 IDLE 或 NRT 的目标。
- 速度/加速度上限是**规划参考轨迹约束**，不保证实际轴速度绝不超限；实际跟踪还取决于负载、增益及电机设置。
- 当前位置检查使用电机协议量程及现有零点。IK 不做整臂碰撞包络检测；MCU 的额定力矩/驱动故障保护仍是独立安全链路。

PC 默认到位参数（每次 move 可配置）：

| 参数 | 默认值 | MCU 允许范围 |
|---|---:|---:|
| 位置误差 | 0.01 rad | 0.001–0.05 rad |
| 实际速度绝对值 | 0.05 rad/s | 0.01–0.1 rad/s |
| 连续稳定时间 | 100 ms | 20–5000 ms |
| 运动总超时 | 30 s | 必须大于规划时长+稳定时间，最多 600 s |
| 每轴规划速度上限 | 0.1 rad/s | >0 且不超过该电机量程 |
| 每轴规划加速度上限 | 0.2 rad/s² | >0 且 ≤10 rad/s² |

这些是可调的初始工程参数，不是经过实机负载验证的安全参数。位置/速度/加速度单位为 rad、rad/s、rad/s²。Kp、Kd、力矩单位为 Nm/rad、Nm/(rad/s)、Nm。Kp/Kd 要求显式传入，不自动套用高刚度模板。

## 异常与控制权

同一 session 绑定来源 IP/UDP 端口。运行、等待到位及完成后的保持阶段都须 100 Hz 保活；超过 **100 ms** 未收到有效保活即失败。七轴反馈持续过期、异常状态、失能、CAN 提交失败或整段超时，同样失败。

反馈空窗采用两级判定：单轴反馈短暂超过约 50 ms 时，MCU 暂停推进本次轨迹并保持最后一个已发布目标；只有连续约 100 ms 没有完整七轴反馈才报告 `FEEDBACK_STALE` 并受控停止。这样可容忍 CAN 仲裁/中断造成的偶发缺帧，同时仍不会在电机反馈真正中断时持续驱动。收到新鲜反馈后，暂停时长从轨迹时钟中扣除，继续时不会补发跳点。

失败停止规划输出，MCU 做受控停止（刹停后在 IDLE 保持使能，没有 PC 也一直保持，见 control-modes.md“受控停止”），不失能；失败在 DMOS 中锁存，直到 PC 取消、切换模式或失能；不自动执行下一段、不自动复位/重启。完成后保持最终关节目标；PC 后台线程继续保活，因此业务代码短暂计算不会中断保持。保活中断（客户端退出、网线断开）、显式取消运动会话或断开连接，或 STOP/DISABLE 会终止会话：除 DISABLE 立即失能外都是受控停止，move 结束后机械臂保持使能。外部 DISABLE 始终允许；其他控制流不能覆盖 move 目标。

**失能不是安全制动，承重机械臂可能下坠。实机调试需机械支撑、急停及确认过的限位。** 软件状态应答不能替代功率级/制动器安全确认。UDP CRC 与 session 只用于完整性和归属，不提供认证。

## PC 接入

配套 SDK 的 C++ 和 Python 都提供 `movej`、`movep`、异步提交和取消接口。Qt 使用原生 DMOV 会话（SDK 的 `src/motion_session.hpp`），保持 UI 线程响应。RT/NRT 必须在静止时经过 IDLE 切换；电机 MIT/位置速度配置必须失能才能改变。`movep` 的 208 字节 DMOV v1 载荷使用 xyz（米）+ 行主序 3x3 旋转矩阵，末尾原保留的 8 字节现为 `cartesian_limits`（笛卡尔速度上限，0 = 默认，见 control-modes.md）；RT Cartesian 使用 DMCP v7 的 284 字节 pose 流，同样每包带速度上限。

Qt movej 操作流程：

1. 全部失能，选择并应用七轴统一的 MIT / 位置速度模式，然后全部使能；MCU 在 IDLE 保持，不执行滑条目标。
2. 检查逐轴增益等参数，在执行模式选择 NRT Joint，点击“应用控制模式”。准备期只发无目标心跳，不发 RT 目标；确认反馈新鲜且静止后，以 100 Hz 提交当前位置保持动作。收到保持动作的 `DONE` 才显示“已就绪”。
3. 下方七轴滑条/数值框只编辑目标，点击“执行 movej”并确认后提交。运动中禁止提交下一条；收到 MCU 实际到位的 `DONE` 后才允许下一条。Qt 窗口始终可响应取消，不阻塞事件循环。
4. 可配置每轴速度/加速度上限、Kp/Kd、前馈力矩，以及整段位置/速度到位阈值、连续稳定时间和总超时。J8 保持使能准备时的力位目标，不参与这条七轴 move；已独立使能的夹爪可通过 SDK 的夹爪接口操作。
5. “停止 move（受控停止并保持）”只发 DMOV CANCEL：MCU 刹停机械臂并在 IDLE 保持使能，不失能。应答中断或 MCU 报失败时，会话锁存失败并同样只发 CANCEL，不再附带 DISABLE；不自动执行下一段，也不恢复实时流。遥测过期时取消按钮仍可用。需要隔离动力时先支撑机械臂、显式失能，再切断驱动电源。

move 参数不会自动写入电机 Flash；启动采用现有 PC 控制配置的低增益默认值，导入控制模板后可同步其 Kp/Kd。不要同时运行多个运动控制器；重连或关闭 GUI 可能中断当前会话。首次测试需核对固件支持 DMOV，并做好机械支撑。

已处于 NRT Joint 的 SDK 客户端可调用 `robot.movej(q, options)`；等待返回后核对结果和停止原因，再决定是否提交下一目标。示例从当前保持指令起步。[SDK movej 示例](https://github.com/YaolinDong/infrabot-sdk/blob/main/examples/python/03_movej.py)

## UDP 协议

目的端口为 5001，独立于 DMCP/DMTL。所有整数小端，浮点为 IEEE754 binary32，CRC32/IEEE（与 DMCP 相同）覆盖除最后 4 字节外的完整报文。

请求 `AppMotionRequest`：208 字节。依次为 `u32 magic(0x444d4f56), u16 version(1), u8 op, u8 reserved(0), u64 session, u32 motion_id`；关节 MOVEJ 参数为 `u32 timeout_ms, u32 dwell_ms, float position_tolerance, float velocity_tolerance` 后接各 7 个 float 的 `target, velocity_max, acceleration_max, kp, kd, torque`。笛卡尔 MOVEP 保持同一长度，参数区改为 `timeout_ms, dwell_ms, position_tolerance, velocity_tolerance, pose[12]`（xyz 米 + 行主序 R），以及各 7 个 `velocity_max, acceleration_max, kp, kd` 和两个保留 u32；最后 `u32 crc32`。

op：1=MOVEJ，2=MOVEP，3=KEEPALIVE，4=CANCEL，5=QUERY。QUERY 只读，不喂看门狗。motion ID 从 1 单调递增，不循环复用；session 使用非零随机 64 位数。取消的会话不可继续运动。

响应 `AppMotionReply`：56 字节，回复到**请求来源 socket**，不抢占 Qt 的遥测订阅。依次为 `u32 magic(0x444d4f53), u16 version(1), u8 result, u8 state, u64 session`；7 个 u32：`request_id, active_id, elapsed_ms, duration_ms, arrived_mask, enabled_mask, fresh_mask`；`float max_error_rad, u32 reason, u32 crc32`。

state：0=IDLE、1=RUNNING、2=SETTLING、3=DONE、4=FAILED、5=CANCELLED。

result/reason：0=OK、1=BUSY、2=NOT_READY、3=INVALID、4=UNSUPPORTED、5=BAD_SESSION、6=STALE_ID、7=COMM_TIMEOUT、8=MOTION_TIMEOUT、9=FEEDBACK_STALE、10=MOTOR_FAULT、11=CANCELLED、12=TX_ERROR。`result=OK` **不是到位**；必须核对 session、motion ID、state=DONE。bit0–6 对应 J1–J7，全部到位为 `0x7f`。

串口增加低优先级 `MOVE id=... state=... reason=... arrived=... fresh=... enabled=... healthy=... error_mrad=... heartbeat_age_ms=...`。原 `udp_cmd_rx` 仍只统计旧实时流，新协议查看 MOVE 的 `rx`。`fresh` 的 bit0–6 可直接定位是哪一轴反馈没有在 50 ms 窗口内到达。

## 离线验证

在 MCU 仓库执行（不烧录、不访问机械臂）：

```sh
tests/run_host_tests.sh
cmake --build build/Debug -j
```

PC 端运动会话与客户端回环测试随 SDK 的 CTest 运行。

覆盖轨迹限速/限加速度、七轴到位、持续稳定、重复包、串行放行、超时、反馈过期、故障、来源校验、取消抢占、时钟回绕、PC 应答 ID/CRC 校验。离线通过不代表实机的到位精度、跟踪稳定性或 1 kHz 最坏执行时间已验证。
