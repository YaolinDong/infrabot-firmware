# Firmware architecture

The board initializes clocks, SDRAM, the display and FDCAN. The control task runs every millisecond. CAN reception updates joint feedback in the interrupt handler. Lower-priority tasks service Ethernet, render the display and drain diagnostic output.

The control modules use the declarations in `app/platform.h` and `app/platform_drive.h`. CMake selects inline implementations from the board directory. Those implementations include HAL and FreeRTOS in firmware builds. The layer check preprocesses each portable control translation unit with the real interface headers and no board backend or header mocks.

`drivers/damiao/dm_protocol.c` packs commands and decodes feedback. The CAN transport lives in `dm_motor.c`. Shared control statistics, bus statistics, motor state, UDP state and UART state are defined in their owning modules. The canonical wire definitions live in `protocol/infrabot/protocol.h`.

## H7C（无实体法兰） 机械臂模型和 MCU 本地重力补偿

MCU 和 InfraBot SDK 统一使用 H7C（无实体法兰） 的七个活动关节，TCP 为 **base_link → Link_flange**。
虚拟法兰通过 `Joint_flange` 固定在 Link_7 上，不占用 J8。原点位于安装面中心，相对旧 TCP 沿原法兰 −X 移动 5 mm，轴向不变。七轴全零时法兰 XYZ 约为
**(53.000, 0.253, 220.675) mm**。旧结构的笛卡尔目标需重新核对；更新固件时同步更新 PC 模型。

本地 `GRAVITY_FREE`、`GRAVITY_LOCK` 和阻抗前馈使用 [H7C 裸臂辨识 v1 URDF](../models/h7c/infrabot-H7C_v1.urdf) 及[配套关节参数](../models/h7c/infrabot-H7C_v1_friction.yaml)，LCD 模型名为 `H7C bare v1`。源包是 `infrabot-H7C_urdf_v1`；辨识更新了质量、质心和关节偏置/摩擦项，质心处的转动惯量锁定为 CAD。重力前馈包括 `offset + spring*q`；速度相关摩擦补偿比例保持 0，与 PC 手拖验证一致。夹爪和实体法兰均未计入模型。
[辨识前的 CAD 模型](../models/h7c/infrabot-H7C.urdf) 仅作对照；`models/gravity_v1/` 是更早的另一结构，不用于当前固件。运动学、TCP、电机方向、编码器零位和力矩保护未变。
原包说明指出 J5 的定义与 **H7C 原始导出**相差约 90°，上机先核对机械轴、驱动正方向与模型显示，再验收笛卡尔和重力模式。

开机默认进入屏幕 `HOME` 页，点 `ENABLE ARM` 使能 J1–J7。尚未完成模式自检时，这个按钮会先
异步配置 MIT 并执行与 SDK 相同的寄存器自检，再进行分步使能；已验证的电机模式保持原样。
零位必须已标定，按钮不会自动设零。MCU 可以先于机械臂上电：开机探测不到 J1–J7 时每秒重新探测一次，首页显示
`Waiting for arm power: n/7`，机械臂上电后等 1 s 再确认一次，然后继续初始化，之后即可使能。等待期间状态显示 PRECHECK（总线上没人应答，CAN 错误计数升高属于预期）；每次探测失败后重新初始化 CAN 控制器，清掉未应答的帧和错误计数，机械臂上电后不会残留 FAULT。运行中机械臂断电（先于 MCU 下电、急停、掉线，使能中也一样）：CAN 故障且 J1–J7 全部 200 ms 没有反馈时，判定机械臂断电，把电机标记为失能（电机本身已经断电），然后每秒重新初始化 CAN 控制器并探测 J1–J7，状态显示 PRECHECK、首页显示 `Waiting for arm power`；机械臂重新上电、连续两次探测全部应答后恢复，串口打印 `CAN: all 7 arm motors respond again`。电机重启后控制模式恢复为其自身保存的值，所以恢复后的第一次 ENABLE 会重新下发控制模式并做自检。J8 不属于首页使能组，夹爪未连接也可操作机械臂。
使能 MIT 后，在 `GRAVITY` 页点 FREE 或 LOCK。
切换 free/lock 前先 STOP。STOP 在 MCU 内以 0.5 s 渐变恢复进入补偿前的保持增益，保持使能；
J8 继续自己的控制。进入补偿渐变时间 1 s。未使能、未标零、反馈过期、模式不对或温度过高时拒绝启动。
进入 FREE/LOCK（包括 STOP 后 RESUME）保留当前保持位姿和前馈力矩，按完整 MIT 输出平滑交接：
原来的 PD 承重与前馈一起过渡到重力补偿，避免先清零前馈或同时缩小刚度和位置误差造成承重缺口。
交接结束后 FREE 的 Kp 为 0，最终补偿由 H7C 重力模型决定；力矩限幅保持不变。
运行中监控 CAN、反馈、关节范围和现有力矩保护；超过限速一半开始增加制动力矩，接近关节限位时
提前制动，明显超速才退出补偿并停止。

这是显式的本地控制模式：进入后不需要 PC 指令流或心跳，PC 断开不会结束补偿。
屏幕 STOP 或原 SDK `Robot.stop()` 会退出；`disconnect()` 本身仍只断开连接。
串口 `GRAV ... us=当前/最大 ticks=...` 和页面 `calc` 显示模型计算耗时，预算 250 us。
需上板确认 1 kHz `miss=0` 和 LCD 30 Hz；离线数值测试不是机械臂稳定性验收。

`GRAVITY` 页两个滑条对 J1–J7 使用统一设置：

| 滑条 | 范围 | 默认 | 含义 |
|---|---|---|---|
| Damping Kd | 0.1–3 N·m·s/rad | 0.8 | 自由拖动和锁定模式的阻尼（最小 0.1，避免几乎无阻尼） |
| Lock Kp | 0.5–30 N·m/rad | 8 | LOCK 的位置刚度；FREE 的 Kp 始终为 0 |

滑条实时更新 RAM，重启恢复默认值；不改写辨识模型或控制增益 Flash。
控制任务分别以每秒 2、20 的最大参数变化率平滑生效，最终力矩仍受原有保护限幅。
STOP 恢复进入补偿前的保持增益，不采用滑条上的 Lock Kp 作为停止刚度。

**限速**：FREE/LOCK 与 PC 运动指令使用同一个 3.14 rad/s 每轴上限（`DM_GRAVITY_MAX_SPEED_RAD_S`）。

| 关节 | 开始制动（50%） | 满制动（100%） | 异常超速停止（125%） |
|---|---:|---:|---:|
| J1–J7 | 1.57 rad/s | 3.14 rad/s | 3.925 rad/s |

- 从 50% 开始平滑增加反向制动力矩，100% 时达到每轴力矩保护值的 25%，按 50 ms 上升到最大值；
  它始终反对当前实测速度，不会在反向或松手后继续沿旧方向推动。
- 达到或略超过 3.14 rad/s 时继续运行 FREE/LOCK；超过 125% 才受控停止（并不自动失能）。

**关节限位制动**：朝某个限位运动、且速度大于 `sqrt(2·5 rad/s²·剩余角度)`（按 5 rad/s² 来得及停下的速度）
时施加反向制动力矩，超出 0.5 rad/s 时达到每轴力矩保护值的 30%。只反对朝限位的运动，离开限位方向不施力。
例如 3.14 rad/s 时约在限位前 56° 开始制动，1 rad/s 时约 6°。手持续用力顶向限位时仍可能到达
限位外 3° 的停止边界并受控停止。

- 页面显示 `Braking (speed/joint limit): Jn`，串口 `speed_mask` 标识正在制动的关节；这不是故障。
  显式 STOP 后附加制动项随保持增益恢复而退出，避免把临时制动力矩留作静态保持偏置。

这些是固件的调节/保护设定，不是建议拖动速度。手拖速度由外力产生，有限的制动力矩不能保证
实测速度瞬时严格不超过限值；效果必须上板验证。进入/恢复重力模式仍要求先静止。

串口 `GRAV` 会附带生效参数 `kd_milli` 和 `speed_mask`。自动保护停止时，
下一次状态输出增加 `GRAV_STOP`：`check=SPEED/LOWER_LIMIT/UPPER_LIMIT/NONFINITE/MODEL_INVALID/CALC_BUDGET/...`、
触发关节（1–7，0 表示全臂）、触发值、限值、当时 q/dq 和参数。速度单位为 mrad/s，位置为 mrad，
运算预算为 us；非有限/无法表示的 q/dq 用 INT32_MIN 标记。该记录在恢复到 IDLE 后仍保留，
重力页显示 `Last stop ...`；它是最近一次事件，不代表当前仍有同一故障。
`STOP: invalid target Jn detail=1..6` 在重力模式下分别对应非有限反馈、超速、低于下限、超过上限、
模型输入/输出无效、计算超预算。

SDK 可通过 `Robot.switch_mode(Mode.GravityFree/GravityLock)` 进入；固件能力位为
`INFRABOT_CAP_LOCAL_GRAVITY`，旧固件会被 SDK 拒绝。模式值 5/6，已有协议字段和旧模式值保持兼容。
生成文件和 LCD 显示模型名称与发布标识。H7C bare v1 沿用首次发布时 URDF 与摩擦 YAML 拼接的 SHA256。清理来源注释后的输入通过完整文件摘要与这份发布标识对应；其他输入使用实际文件的 SHA256。更新模型需重新生成、构建、验证并烧录。

```bash
python3 tools/export_gravity_model.py \
  --urdf models/h7c/infrabot-H7C_v1.urdf \
  --friction models/h7c/infrabot-H7C_v1_friction.yaml --name "H7C bare v1" \
  --out control/arm_gravity_model.inc
python3 tools/export_kinematics.py models/h7c/infrabot-H7C_v1.urdf control/arm_kinematics_model.inc
tests/run_host_tests.sh
python3 tests/test_h7c_kinematics.py --sdk ../infrabot-sdk
python3 tests/test_gravity_model.py
cmake --build --preset Debug
```
