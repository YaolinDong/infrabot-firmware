# Ethernet and CAN interfaces

## 连接时的版本协商

SDK 通过命令端口发送 16 字节的 DMIF 请求，固件从遥测端口向该请求的 IP 与源端口回复 DMIO。每个回复回显 sequence。V1 回复保留原有 154 字节布局、能力位和 CRC。V2 回复共 218 字节，前 154 字节使用版本号为 2 的 info 布局并保留前缀 CRC，之后是两个 16 位协议版本号、24 字节固件版本、16 字节板型、16 字节机械臂型号和整包 CRC。文本以 NUL 结尾，超长版本字符串截短。

整体协议当前为 1.1。主版本不同，SDK 断开并报告两端版本；次版本允许新增能力，由 capabilities 判断功能。仅支持 DMIO V1 的固件按 1.0 处理，固件版本显示 unknown。新版 SDK 先尝试 V2，再回退查询 V1，收到有效回复后才启动心跳。旧 SDK 仍可直接请求 V1。回复 V1 表示兼容协议 1.0；未来不兼容的主版本不得用 V1 应答旧客户端。

## 当前配置

```text
USART1：PA9/PA10，115200 8-N-1
FDCAN2：PB5=RX，PB6=TX（外接 CAN 收发器模块）
CAN FD：仲裁段 1 Mbps，数据段 5 Mbps，FD+BRS
电机控制：8 台，每台目标 1000 Hz
电机 ID：0x01 ... 0x08
Master ID：0x11 ... 0x18
```

电机需要预先保存为 `CAN_BR=9`。PB5/PB6 需要连接支持 CAN FD 5 Mbps 的外部
收发器，CAN 总线两端安装 120 Ω 终端电阻。

PB8/PB9（FDCAN1 和板载 CAN 收发器）留给 RGB 屏的 LTDC_B6/B7，不再用于 CAN；底板跳帽
J40 的 1-2 不要插，让板载收发器保持断电。外接野火 CAN 收发器模块（SIT1042）的接法：

```text
VIO -> 3V3    5V -> 5V    GND -> GND    TXD -> PB6    RXD -> PB5
```

PB5 在底板上也接摄像头接口，PB6 也是 I2C1 SCL（EEPROM、WM8978、摄像头 SCCB）；
使用 CAN 时不要插摄像头，也不能再用 I2C1。

## 参数服务（DMPR / DMPA）

协议 1.1 在命令端口新增一对报文，应答发回请求方。DMPR（24 字节）带操作、请求 ID 和参数 ID；DMPA（80 字节）回显请求 ID，并描述被访问的参数：名称、单位、最小值、最大值、默认值、变化速率，以及请求值和已生效值。参数 ID 的高字节是分组（1 重力、2 阻抗、3 导纳），低字节是组内序号。INFO 用列表位置代替参数 ID，同时返回参数总数，PC 不需要自带参数表就能列出全部参数，固件新增参数时协议不用改。

| 操作 | 含义 |
|---|---|
| 0 INFO | 返回列表中某个位置的参数描述 |
| 1 GET | 按 ID 读取 |
| 2 SET | 写入请求值，应答为写入后的状态 |
| 3 SAVE | 保留，应答 UNSUPPORTED |

写入不会让增益阶跃：控制任务按该参数的速率把已生效值滑向请求值，范围和速率与触摸屏滑条相同。数值只在 RAM 中，重启后恢复默认。INFO 和 GET 任何 PC 都可以发；SET 只接受控制机械臂的 PC（或当前没有控制方），否则应答 BUSY。值不是有限数或超出范围时应答 RANGE，不改变任何值。参数表在 `drivers/net/app_param.c`，主机测试是 `tests/test_app_param.c`。能力位为 `INFRABOT_CAP_PARAMETERS`（0x4000）。

## 以太网关节角度遥测

当前工程已加入野火挑战者板载 LAN8720A 的 RMII 网卡和 LwIP UDP 遥测，和 FDCAN
共存，不占用 PB5/PB6：

```text
PA1  REF_CLK       PA7  CRS_DV
PC4  RXD0          PC5  RXD1
PB11 TX_EN         PG13 TXD0          PG14 TXD1
PC1  MDC           PA2  MDIO
```

默认静态地址为 `192.168.0.50/24`，UDP 遥测端口为 `5000`，发送频率为 1000 Hz；
UDP 指令端口为 `5001`，遥测/时钟同步端口为 `5000`。PC 端先配置到同一网段（例如 `192.168.0.10/24`），
向板端口 5000 发送任意一个 UDP 数据报完成遥测注册，之后板端会把二进制关节状态包
回发到该源地址和源端口。参数在
`app/app_config.h` 中修改。

向 UDP 5001 发送 DMCP v3 数据报即可使能、设零、失能或改变外部电机目标。每帧包含
命令、序号、8 个目标位置、
PC 发送时间、MCU 执行时间和 CRC32；MCU 先缓存数据，在 `execute_mcu_us` 到达后由
1 kHz 控制任务统一应用。连续指令（IDLE 时为 100 Hz 心跳）中断超过 100 ms 会受控停止：
刹停后在 IDLE 保持使能，没有 PC 也一直保持，MCU 不会自行失能；PC 可一键恢复到被打断的模式（见 [受控停止](control-modes.md#受控停止)）。每秒串口
状态行中的 `udp_cmd_rx`、`interval_us`、`last_us`、
`jitter_avg_us` 和 `jitter_max_us` 分别表示接收数量、相邻接收间隔的平均/最小/最大值、
最近一次间隔、平均绝对抖动和最大绝对抖动；`udp_tel_tx` 是遥测 `udp_sendto()` 成功
次数，后面的字段含义相同，`udp_tx_err` 为 pbuf 或发送错误次数。

遥测为 DMTL v3（348 字节），头部为 `<IHHIIQ`：`magic=0x444D544C`、版本、关节数量、序号、
时间戳毫秒、MCU 单调时间微秒；随后每个电机 16 字节，格式为 `<iiiBBH`：位置 mrad、速度
mrad/s、力矩 mNm、状态、MOS 温度、反馈计数低 16 位。前 180 字节与 v2 相同；之后是每个电机本拍的
指令（位置、速度项、力矩前馈、线圈温度、标志）、MCU 侧链路统计和受控停止/参数自检状态，见
[遥测 v3](control-modes.md#遥测-v3)。所有报文定义在 `protocol/infrabot/protocol.h`；PC 端
InfraBot SDK 保存一份相同的拷贝，两个仓库互不依赖。

需要估计时钟偏移的客户端可以向 UDP 5000 发送 DMCS v1 同步请求。MCU 返回 DMCR v1，包含接收和发送时刻。

构建和主机测试只用本仓库的文件。主机测试：`tests/run_host_tests.sh`。

**USB 转串口下载（STM32H743 ROM，Linux）**：不用修改应用或预烧串口 bootloader。先离线执行 `python3 tools/uart_flash.py prepare`，由本次构建的 ELF 生成 `build/StandaloneDebug/infrabot_firmware-uart.bin`。勿使用目录中旧的同名构建产物混烧。

确认机械臂停止、驱动动力断开，控制板正常供电。用 3.3 V UART：适配器 TX 接 PA10、RX 接 PA9，GND 共地。BOOT0 是复位时采样的启动选择输入；在默认启动选项下，将 BOOT0 置高、复位进入 ST 系统下载程序。请按实际底板原理图确认跳帽/BOOT 键，KEY1/KEY2 不替代 BOOT0。关闭串口终端后，先运行 `python3 tools/uart_flash.py probe /dev/ttyUSB0`（端口是示例），核对连接的确实是预期 H743。串口握手和读 Flash 成功后才运行 `python3 tools/uart_flash.py flash /dev/ttyUSB0`。脚本会先读 Loader 标记和两槽头，识别到 A/B 布局或读取失败时不会擦除。这一串口入口仅用于非 A/B 板。程序使用 CubeProgrammer 的 115200 8-E-1、只擦除镜像覆盖的 Bank 1 扇区并写后校验，不擦 Bank 2 的配置扇区。连接失败时先检查接线/端口权限及启动设置，再复位重试。成功后恢复 BOOT0 低电平、按 RESET；串口监视器改回 115200 8-N-1 确认 `infrabot_firmware start`。可用 `STM32_PROGRAMMER_CLI` 指定 CubeProgrammer 可执行文件路径。

验证网口连接时，先用配套 SDK 的只读连接示例检查固件版本和关节遥测。需要进入运动模式时，按 SDK 文档完成驱动配置、自检、显式使能和模式切换。[InfraBot SDK](https://github.com/YaolinDong/infrabot-sdk)

最小 Python 接收示例：

```python
import socket, struct

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("0.0.0.0", 6000))
s.sendto(b"subscribe", ("192.168.0.50", 5000))
while True:
    data, _ = s.recvfrom(512)
    magic, version, count, sequence, timestamp_ms, mcu_timestamp_us = struct.unpack_from("<IHHIIQ", data)
    if magic != 0x444D544C:
        continue
    for i in range(count):
        off = 24 + i * 16
        pos, vel, torque, state, temp, feedback = struct.unpack_from("<iiiBBH", data, off)
        print(i + 1, pos, vel, torque, state, temp, feedback)
```

网卡初始化、DMA 描述符和 D2 SRAM 非缓存区分别位于 `drivers/net/ethernetif.c`、
`drivers/net/app_ethernet.c` 和链接脚本中；以太网处理在独立的低优先级 FreeRTOS
任务执行，不进入 1 kHz FDCAN 控制任务。
