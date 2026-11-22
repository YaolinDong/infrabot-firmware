# Touch panels and local controls

## RGB 屏幕

支持野火 4.3 寸 **480×272** 和 5 寸 **800×480** RGB 电容屏，启动时读取触摸控制器
出厂配置中的坐标范围选择屏幕。GT115x 可用于两种屏，不能只凭产品 ID 判断尺寸。

- 4.3 寸默认时序来自 `4.3寸/版本_EBF410261V1R0/屏幕/GL043056B0-40-N 7-LED300.pdf` 第 11 页：
  9 MHz、总周期 531×292（约 58.05 Hz），同步脉宽 4/4，含同步的后消隐 43/12，前消隐 8/8。
  HAL 的 HBP/VBP 不含同步，分别填 39/8。老版可用兼容例程时序，见下面的宏。
- 5 寸保留 27 MHz、总周期 887×519（约 58.65 Hz）和原引脚、极性。
- 两种屏的接线相同，LTDC 层 0 使用 RGB565，物理显存位于核心板 SDRAM 的 `0xD0000000`。
- 4.3 寸使用专用 **480×272 原生布局**（`app_lcd_ui_small.inc`），8×16 点阵字体逐像素绘制，不缩小、不插值；切换成简体中文后汉字是 16×16 的 4 bit 抗锯齿字形（占两个字符宽）。5 寸保持原来的 800×480 英文布局（标签式导航，下文 HOME/IMPED/INFO 等说明指 5 寸）。
- **主菜单**：开机进入。10 个磁贴写全名，下行是实时状态：Arm & Joints、3D View、Gravity Compensation、Network、Joint Impedance、Cartesian Impedance、Joint Admittance、Cartesian Admittance、Diagnostics、Settings。正在运行的模式磁贴变绿（如 `FREE active`），出问题的变橙（如 `STOP: STREAM_LOST`、`link DOWN`、CAN 错误）。
- **顶栏每页相同**：子页面左上角 `<` 返回主菜单（只有两层），然后是页面全名；右侧是控制器状态色块、一个健康词和 **STOP**。STOP 每页都在最右上角，离返回键最远。健康词正常为绿色 `OK`，否则写出最严重的问题（`NO ARM`、`BUS-OFF`、`DRV ERR`、`PC LOST`、`MISSED`、`CAN ERR`、`FB LOST`），前面的圆点兼作 1 Hz 心跳。
- **页内切换条**代替“再点一次标签翻页”：3D（MODEL/RENDER）、Gravity（CONTROL/DETAILS）、Network（EDIT/STATUS）、Cartesian Impedance（PARAMS/LIVE）、Joint Admittance（PARAMS/SHARED/LIVE）、Cartesian Admittance（PARAMS/SHARED/ELBOW/LIVE）、Settings（LANGUAGE/ABOUT/MOTORS/PD）。
- **Arm & Joints**：ENABLE ARM / DISABLE ARM / RESUME，一行状态（等待上电、拒绝原因、受控停止原因和阶段），8 关节表（使能点、q、cmd、力矩、MOS 温度、状态：正常 `OK`，驱动故障红色、力矩到限橙色、无反馈灰色），TCP XYZ（mm）和 RPY（deg）。
- **四个柔顺模式各一页**：ENTER 按钮按 MCU 实际状态变色：未进入时**蓝色**（点它开启该模式），已请求等待确认或过渡中**琥珀色**（WAITING / RAMP），运行中**绿色**（ACTIVE）；重力补偿页的 FREE / LOCK 同样。拒绝时状态行橙色给出原因。左侧是本模式自己的参数列表：点一行选中，下方滑条调值（拖动期间锁定，不会误触返回或 STOP），显示生效值和范围；右侧是 J1–J7 力矩（阻抗：FF/Drive，导纳：Ext/PD）。导纳的静摩擦比、摩擦 beta、驱动增益和偏置学习是两种导纳**共用**的同一组 RAM 参数，两页的 SHARED 都能改，并标注 `shared`；阻尼比 zeta 同样共用。
- **Diagnostics**：控制（频率、丢拍、最大周期、模型计算耗时、停止原因）、CAN（收发、txerr、错误事件、TEC/REC、被动/总线关闭）、网络（链路、PC 指令/遥测、非法包、PC 地址）、显示（触摸芯片、坐标、I2C 错误、帧耗时）。原 HOME 2/2 的内容都在这里。
- **Settings（设置）**：第一个视图 LANGUAGE 选择 4.3 寸屏的语言：`English` / `简体中文`（按钮各用本语言书写，任何一种语言下都认得出；中文时标题和视图名也带英文 `设置 Settings`、`语言 Language`）。点一下立即切换整个 4.3 寸界面：标题、按钮、表头、状态行、拒绝原因、诊断文字和其他模块的提示（停止原因、驱动状态、本地控制消息）都翻译；数字、单位和 Kp、Kd、TCP、CAN、IP 等缩写保持原样。**保存到 MCU Flash 需要所有电机都失能**（和保存 IP 一样，擦扇区会让 LCD 任务停约 1 s）：机械臂使能时只临时切换，页面橙色提示“未保存，重启后恢复已保存的语言”，失能后再点一次即保存；保存成功显示绿色，串口打印 `UI: language N saved to MCU Flash seq=...`。Flash 忙或写入失败时如实提示，不会显示成已保存。开机时读取保存的语言，没有记录时为英文。5 寸屏和 PC 工具仍为英文。其余三个视图是原 INFO 三页（ABOUT 固件版本/作者、MOTORS 电机型号和编码范围、PD 参数），只读。
- 3D 直接渲染到小屏 320×237 原生视窗；RENDER 视图暂停模型绘制并显示最后一帧的分项耗时。按钮命中、滑条、拖动和每页 STOP 都有主机回归测试（`test_lcd_ui_43`）。
- 显示在低优先级 LCD 任务内完成，未增加 1 kHz 控制任务的工作；实际 UI 帧率和控制周期仍需上板验收。

`app/app_config.h`：

```c
#define DM_LCD_PANEL              0U   // 0=AUTO, 43=强制4.3寸, 5=强制5寸
#define DM_LCD_FALLBACK_PANEL     43U   // AUTO无法识别时的回退，不等于识别成功
#define DM_LCD_43_LEGACY_TIMING    0U   // 0=新版GL043056; 1=老例程41/8/4、10/2/4时序
```

触摸先于 LTDC 初始化，只读配置、不下载或覆盖触摸参数。先尝试直接读取已就绪的控制器；仅当 ID、两次配置范围及清除旧触摸报告均正常时跳过复位。否则沿用原硬复位的 10+10+60 ms 时序后重试。串口的 `probe=...ms` 单独统计探测时间。GT115x/GT5688/GT917S 读取 `0x8051`，
GT911/GT9157/GT615 读取 `0x8048`；连续两次读数一致且范围是 480×272 或 800×480 才自动选择。
纵横交换的对应范围也支持，触摸坐标同时交换。此方法依赖屏幕出厂配置，不能识别被改写成错误坐标范围的屏幕，
也不能区分相同分辨率的其他尺寸/不同液晶批次。这里只提供已知 4.3/5 寸配置，不支持热插拔。

串口示例：`LCD: touch GT1158 range=480x272; panel=4.3in source=touch-config`，随后显示
`LCD: 4.3in 480x272 ... pclk=9MHz`。探测失败会打印 `FALLBACK (not detected)`；这时可修改强制型号。
未知/不匹配的触摸范围会禁止触摸操作，避免错误坐标触发运动按钮，板载按键与 PC 功能照常运行。
5 寸顶部导航新增 `INFO`：点击进入，继续点击同一标签在三页间循环（4.3 寸在主菜单的 Settings 页，用页内切换条）。`INFO 1/3` 显示 MCU 固件版本、编译时间、作者 YaolinDong、邮箱 dong1505lin@gmail.com、屏幕/控制频率及配置就绪状态；`INFO 2/3` 显示八个电机的型号、CAN ID / Master ID 和 PMAX/VMAX/TMAX 协议编码范围（不是软件安全限值）；`INFO 3/3` 显示 MCU 当前 MIT Kp/Kd、驱动模式、缓存/默认的 KP_APR，以及重力模式已应用的参数。电机启动未完成时 PD 数值显示 `--`。INFO 为只读页面，不发起 CAN 参数查询或写入；位置速度模式的电机内部 PD 与表中的 MCU MIT 参数不是同一组。

固件版本由每次构建运行的 `cmake/GenerateBuildInfo.cmake` 从 Git 标签/提交号生成，存在已跟踪源码修改时带 `-dirty`；无 Git 元数据时显示 `unknown`。

LCD 初始化时先在帧缓冲画好字标和署名，再打开背光。串口的 `app_first_frame=...ms` 是从**应用 HAL 开始计时**到首帧写完并开启背光的时间，不含此前 A/B Loader 的镜像验签，也不是面板实际发光的测量值。随后显示约 3 秒的开机动画（从开启背光计时）：黑底青蓝色 `infrabot` 从可见亮度渐亮，绕屏幕竖直 Y 轴左右翻转 360°（透视投影、Nunito 圆润粗体、圆角立体边缘、动态明暗和柔和投影；字标约占屏宽三分之二），停留后进入 HOME。底部两行保留中英文作者（董耀琳 / Dong Yaolin）及邮箱 `dong1505lin@gmail.com`，最下方第三行居中显示 `https://github.com/YaolinDong`。作者/邮箱和链接保持两块屏各自的原生字号；首帧在调度器启动前绘制，后续旋转在 LCD 低优先级任务内运行，与启动自检并行，不发送电机控制命令。字模由 `tools/gen_lcd_boot.py` 离线生成。两个面板共用一套 640×280、16 色、71 帧的旋转字标：5 寸按原像素绘制，4.3 寸做 5:3 面积滤波到 384×168。生成器逐行裁掉黑边、按 4 bit 打包，再用 LZ4 HC 压缩；每帧至多引用一张独立关键帧，所以低优先级任务错过某帧时可以直接跳到目标帧。解码在已有 SDRAM scratch 内，不新增帧缓冲；目标 30 Hz，串口输出上机实际帧数和最大绘制耗时。应用镜像缩小会减少 Loader 每次哈希该槽的字节数，不会跳过签名校验。重新生成帧需要主机 C 编译器及 liblz4。

SDRAM 或 LTDC 初始化失败只打印日志，机械臂照常运行。

界面目标 30 Hz，1 kHz RTOS tick 下按 33/33/34 ms 调度；每帧按实际唤醒时刻重设节拍，超时后不追赶旧周期。
文字只重画变化的字符；3D 页的模型每帧检查，
右侧 TCP 和性能文字最多 10 Hz 更新。

顶部导航栏在每个页面都有：`HOME` / `NETWORK` / `3D` / `GRAVITY` 四个标签（点击切换页面，当前页高亮），右边是控制器状态
（RT_JOINT 等，按状态着色），右上角绿色方块 1 Hz 闪烁。

**HOME 页**（默认首页，刷新调度仍为 30 Hz）：

- `ENABLE ARM`：自检后分步使能 J1–J7，锁存当前位姿，不自动回零。
- `DISABLE ARM`：明确撤去 J1–J7 力矩，J8 保留自己的控制；按钮下方明确显示失能含义。
- `STOP`：中止启动或运动，制动并保持已使能的关节；重力模式以 0.5 s 恢复保持增益。
- `RESUME`：等待刹停和增益恢复结束后，检查反馈、故障和静止条件；重力模式重新以 1 s 渐变进入。
  原来是 PC 的 RT/NRT 任务时，恢复到 IDLE，等待 PC 下发新的运动，不自动重放旧轨迹。
  本地保持和恢复不要求 PC 心跳；进入外部 RT/NRT 模式后重新使用原有通信看门狗。
- 按钮下方显示设置、自检、使能进度或拒绝原因；故障/力矩保护停止不能用 RESUME 静默清除。

- 第 1 行：控制频率、丢拍、最大周期，CAN 每秒收发帧数、发送错误、TEC、开机以来 CAN 错误事件数 `ev`、
  错误被动/总线关闭。
- 第 2 行：网口链路、PC 指令和遥测每秒包数、非法包数，锁存的受控停止原因和阶段。
- 关节表：J1–J8 的使能、关节角（度，两位小数）、指令角度 `cmd`（度，控制任务跟随的关节指令，经 MCU 限速、未加位置速度模式的超前补偿，只在使能时显示；和 `q` 相减就是跟踪误差）、力矩（Nm）、MOS 温度、驱动状态；
  驱动故障红色，力矩到限橙色，没有反馈灰色。
- TCP 行：J1–J7 实测角度正运动学算出的末端位置（mm）和 ZYX 欧拉角 RPY（度），坐标系和串口 RTIK 行的 `meas` 相同。
- 底行：触摸状态、坐标和点数，I2C 错误数，本帧/最大绘制耗时（µs）。

**3D 页**（数字孪生，只读）：

- 实体模型：H7C 的 8 个 STL 网格（基座、Link_1–7）从 825910 个三角形离线减面到 4414 个三角形、1398 个顶点
  （`tools/gen_lcd_mesh.py` 默认 `30hz` 配置，顶点聚类，格子约 11–23 mm，每格选取二次误差最小且靠近均值的点，生成
  `ui/app_lcd_mesh.c`），按实测关节角挂到各连杆坐标系（`ArmIk_LinkFrames()`，和 IK/TCP 同一条运动学链），
  按深度从远到近画（计数排序）、平面光照。减面后三角形朝向不可靠，所以不做背面剔除（剔除会打出缺口），
  两面都画。面法线离线算好（生成器写进 `app_lcd_mesh.c`），光照方向每个连杆转一次到连杆坐标系，每个面只做
  点积、查 32 级明暗表；深度在相机目标 ±1.2 m 的固定范围内分 4096 档两遍计数排序。几何数组约 58 KiB 放在
  AXI SRAM（链接脚本 `.lcd_ram` 段），面直接按从远到近的顺序存放。真机连杆之间是薄侧板，侧看会露出
  背景，模型本来如此。
- 默认保持 `30hz` 网格预算，虚拟法兰只定义末端坐标轴，不绘制实体。
  简化会减少细小孔洞、棱边和曲面细节，整体耗时仍需上板测量；运动学和控制使用完整 H7C 关节参数，不受网格简化影响。
  重新生成：`python3 tools/gen_lcd_mesh.py /path/to/infrabot-H7C_urdf_v1/urdf/infrabot-H7C_v1.urdf --profile 30hz`。
  保留原精度生成选项 `--profile detail`；加 `--output-root /tmp/lcd-detail` 可在独立目录生成原网格作对照。
- 末端画工具坐标轴（X/Y/Z = 红/绿/蓝）；地面 0.1 m 网格和基座 X/Y 轴。
- 相机和 PC 端 Qt GUI（InfraBot SDK 的 URDF 视图）一致：方位角从 +X 量起、仰角从水平面量起、Z 轴朝上，初始视角
  `HOME` = 方位 -45°、仰角 25°、对准 (0.05, 0, 0.22) m；手指拖动每像素转 0.5°（和 GUI 鼠标拖动一样）。
  LCD 用正交投影，大小按 GUI 1.05 m 距离、45° 视场对应到这块视窗，固定不缩放；对准点画在视窗中心偏下。
  `FRONT` / `SIDE` / `TOP` 一键切换视角，`HOME` 恢复初始视角。右边显示 TCP 实测位置（mm）和最近一次
  真正绘制的耗时（`render 本次/最大 us`）。
- 更新矩形先画到 128 KiB AXI SRAM 连续缓冲，每块不超过 65536 像素，分别用 DMA2D 拷贝到屏幕。
  CPU 以成对 RGB565 写入清空并绘制，结束后对齐到 32 字节做 cache clean；DMA2D 只读这个缓冲，因此不需要
  invalidate。缓冲位于 `.lcd_ram`，不会误放进 DMA2D 无法访问的 DTCM。`buf SRAM` 表示使用这条路径。
  较大区域先画模型包围矩形，再画四周互不重叠的背景矩形；每个矩形按容量分块，背景块无需遍历模型三角形。
  几何与排序每帧只计算一次，分块边界保持扫描线取整一致，每个更新像素只提交一次。
- 关闭 `APP_LCD_FAST_BUFFER` 时使用 SDRAM 离屏缓冲（`0xD0100000`，`buf SDRAM`）。MPU region 4 只把该处
  1 MB 设为可缓存（write-back / write-allocate），显示显存保持不可缓存。3D 渲染器独占整个 scratch 区域：
  DMA2D 清屏前后 invalidate，CPU 绘制后 clean，再交给 DMA2D 拷贝；缓存操作范围覆盖矩形的跨行间隙，
  起止地址按 32 字节向外对齐。CPU 绘制限制在该矩形内，每帧结束都回写，保证边界缓存行上的相邻像素可从 SDRAM 恢复。
  三角形拆成上下半段，扫描线交点以斜率累加，边界行保留钳位与取整；较长跨度对齐后一次写两个 RGB565 像素。
- 固定视角下，用本帧和上一帧的模型/工具轴包围矩形的并集做局部清屏、绘制和拷贝，旧位置恢复地面网格。
  拖动/切换视角时更新前后模型、工具轴、地面网格和基座的联合范围，范围之外保留背景。
  首帧、切换页面或改变视窗时完整刷新。`area` 显示所有块累计更新的像素数与占视窗比例，
  完整刷新是 100%；`tiles`（屏幕 `t`）是本帧提交块数，拖动时也应保持 `buf=SRAM`。
- 右边 `geo` / `px` 是几何和像素总耗时；`clr` 是 CPU 清屏或 DMA 清屏（含 invalidate），`ras` 是网格、三角形和工具轴绘制，
  `cln` 是绘制后的 cache clean，`cpy` 是 DMA2D 拷贝，单位均为 us。计时包含被高优先级任务抢占的时间，
  分项时间累计所有块，`px` 还包含分块调度开销。`render 本次/最大 us` 的最大值自进入页面起累计。
- `UI` 是实测显示任务完成频率，`draw` 是实际绘制 3D 的频率，静止时允许为 0。`frame 本次/最大 us`、
  `gap 最小/最大 us` 和 `over` 每约一秒统计一次：分别是含触摸/文字/日志的完整 UI 工作耗时、任务开始间隔、
  工作耗时超过 33.33 ms 的次数。`over` 不统计调度迟到，需同时看 `gap` 和实测频率。
  进入页面清零，首帧和超过 100 ms 的慢帧仍计入耗时统计；3D 页串口每约一秒输出一行 `LCD3D`，
  末尾 `mesh=4414` 标识编入固件的显示模型三角形总数。
- `APP_LCD_FAST_BUFFER` 默认 `ON`；用 `-DAPP_LCD_FAST_BUFFER=OFF` 对比原 SDRAM 绘制路径。
  SDRAM 缓存默认开启。可用 `cmake --preset Debug -DAPP_LCD_FAST_BUFFER=OFF -DAPP_LCD_SCRATCH_CACHE=OFF`
  后重新编译，在同一关节姿态、视角和控制负载下对比纯 SDRAM 无缓存路径；然后把两个选项都设为 `ON` 恢复。
  单独关闭 `APP_LCD_SCRATCH_CACHE` 不影响 AXI SRAM 的 cache clean。主机覆盖/裁剪和缓存交接测试：
  `tests/run_host_tests.sh test_lcd_3d test_lcd_3d_uncached test_lcd_3d_full test_lcd_3d_sdram test_lcd_3d_sdram_uncached`。
  `-DAPP_LCD_PARTIAL_REFRESH=OFF` 可单独关闭局部刷新以对比，默认 `ON`。测试包含连续运动下局部刷新与完整
  重画的逐像素对照、连续旋转、128 KiB 容量边界、分块接缝与不重复提交。
  `tests/run_host_tests.sh test_lcd_schedule` 模拟持续超时后松手，验证无补帧突发及 tick 回绕。
  实际耗时与 DMA/显示一致性仍需上板确认。
- 关节角变化都不超过 1 mrad 且视角没变时不重画，机械臂静止时 3D 页几乎不占 CPU。

**NETWORK 页**：

- 左上显示当前 IP（后面标注来源：`saved` 屏幕上保存过 / `default` 编译默认值 / `applying` 正在切换）、
  子网掩码、网关、MAC、链路状态和已注册遥测的 PC 地址。
- 下面是新地址编辑区：点 IP / MASK / GATEWAY 选中一行（黄色，带光标），用右边键盘输入数字和 `.`。
  `DEL` 删一个字符，`CLR` 清空这一行，`REVERT` 恢复成当前在用的地址，`DEFAULT` 填入编译默认值
  （`app_config.h` 的 `DM_ETH_IP_ADDR*` 等）。
- `SAVE & APPLY`：检查格式（四段 0–255）和合理性（掩码连续、主机位不全 0/全 1、网关在同一子网或为
  0.0.0.0）；**必须所有电机都失能**，否则拒绝（换 IP 会断开 PC 连接）。通过后写入 MCU Flash（约 1 s），
  再由网口任务立即切换到新地址并发一次 ARP 广播，串口打印 `NET: saved ...` 和 `ETH: address changed`。
  PC 端要改成连接新 IP 重新连接。
- 开机时 Flash 里有有效的网络记录就用它（串口 `ETH: address source=MCU Flash`），否则用编译默认值。
  网络记录、4.3 寸界面语言和零点/控制参数在同一个 Flash 扇区的不同位置（0x081E0000 起：控制 256 B、
  网络 32 B、界面 32 B，各带 CRC），保存任一项都会原样保留其余各项（`tests/test_persist.c`）。

每秒统计来自状态任务发布的快照（`AppTasks_GetStatusSnapshot()`），和串口 `STAT` 行是同一组数。

触摸芯片是 Goodix GT9xx 系列（实测这块屏是 GT1158，地址 0x28）：I2C2（SCL=PH4，SDA=PH5，50 kHz），
RST=PI8，INT=PD13。复位时 INT 拉低，先试 0xBA 再试 0x28。显示任务每帧轮询一次，不用中断；按下时触发
按钮动作，按钮高亮到手指抬起。
**触摸不会控制机械臂运动**，只能切换页面和修改网络设置（修改要求电机全部失能）。找不到触摸芯片时打印
`LCD: touch controller not found`，界面照常显示。

- 时序参数来自野火例程 A-4-1（`screen.c`），在 `drivers/display/app_lcd.c`。
- 显示任务优先级 1（最低），目标 30 Hz，只用控制、网口、状态任务剩下的空闲时间；
  `DM_LCD_ENABLE=0` 可关闭。
- ASCII 字库由 `tools/gen_lcd_ascii_font.py` 从 Terminus Font 的 8x16 / 12x24 点阵转换而来（`ui/app_lcd_font.c`，SIL OFL 1.1）。
- 4.3 寸中文：译文表 `ui/app_lcd_ui_zh.inc`（以英文原句为键，格式串的 `%` 转换顺序必须一致，`test_lcd_ui_43`
  检查）；汉字字形 `ui/app_lcd_zh_font.inc` 由 `/usr/bin/python3 tools/gen_lcd_zh_font.py` 从译文表生成，只含用到的
  字（现 352 个，约 45 KB Flash；Noto Sans CJK SC，SIL OFL）。生成器会拒绝没有译文的 `TR("...")` 字符串和其他模块的
  提示字符串，改了界面文字要重新运行。主机测试把每一页、每个视图在中文下走一遍，检查不截断、不重叠、没有缺字。
- MPU region 3 把 SDRAM 设为不可缓存，LTDC/DMA2D 和 CPU 看到的数据始终一致。
- LTDC/SDRAM 的 HAL 模块在顶层 `CMakeLists.txt` 里用编译宏打开，没有改 CubeMX 生成的
  `stm32h7xx_hal_conf.h`；屏幕和 SDRAM 的引脚不在 `.ioc` 里，用 CubeMX 分配新外设时
  不要占用这些引脚（RGB 数据线、PG7/PI9/PI10/PF10、PD4/PD7、触摸 PH4/PH5/PI8/PD13 和 FMC SDRAM 引脚）。

当前电机映射：

```text
M1/M2：DM8009P
M3/M4：DM4340P
M5/M6/M7/M8：DM4310（M8 为夹爪）
```

## 板载状态灯和按键

开发板上的 RGB 灯和 KEY1/KEY2 在 `app/app_panel.c`，独立的低优先级任务（优先级 3，每 10 ms 一次），
屏幕没起来也照常工作。引脚不在 `.ioc` 里，用 CubeMX 分配外设时不要占用：

```text
KEY1 -> PA0（按下为高，底板上还经 1K 接 WiFi 模块 INT）   KEY2 -> PC13（按下为高）
LED  -> PH10 红 / PH11 绿 / PH12 蓝（共阳，低电平亮；这三个脚也接摄像头接口，不要插摄像头）
```

按键和屏幕 HOME 页的 STOP / RESUME 按钮走同一条路（`AppLocalControl_Request()`），检查条件完全相同：

- **KEY1 按下 = STOP**：消抖 30 ms 后立刻生效，制动并保持使能，和屏幕 STOP 一样。
- **KEY2 按住 1 s = RESUME**：长按防误触；能否恢复和屏幕 RESUME 的条件一样（故障类停止不能恢复）。
- 使能和失能不放在按键上，只在屏幕和 PC 上。开机时已经按着的键要先松开再按才算；电机启动流程
  还没完成时按键只打印 `PANEL: ... ignored`。按键生效时串口打印 `PANEL: KEY1 STOP` / `PANEL: KEY2 RESUME`。

状态灯同时只显示最严重的一个状态（慢闪约 0.6 Hz，快闪 2 Hz）：

| 灯 | 状态 |
|---|---|
| 红慢闪 | 机械臂断电（J1–J7 都不应答），每秒重新探测 |
| 白慢闪 | 上电，电机启动流程进行中 |
| 红常亮 | CAN 总线故障、已使能的驱动器报故障，或 RESUME 不能清除的停止（掉线、力矩超限、驱动器故障、跟随/目标/轨迹失败），需要检查后重新使能 |
| 黄慢闪 | 已停止并保持使能，长按 KEY2 可恢复（操作员 STOP、PC 指令流或轨迹心跳丢失） |
| 蓝快闪 | 分步使能、下发模式参数和自检、读写电机寄存器 |
| 白常亮 | 所有电机失能 |
| 蓝慢闪 | 重力补偿 FREE（手拖） |
| 绿常亮 | PC 的 RT/NRT 运动模式 |
| 蓝常亮 | 已使能保持（IDLE 保持、重力 LOCK） |

红/黄的划分和 RESUME 的判定是同一个函数（`AppSafety_Resumable()`）。状态灯只是提示，不是安全功能。
颜色和按键消抖的逻辑在 `app/app_panel_logic.h`，由 `tests/test_panel.c` 在电脑上测试。

## ASCII 字形

两块屏共用 8×16 和 12×24 两套等宽 ASCII 字形，取自 Terminus Font。数字 0 有斜杠。Debian/Ubuntu 的 `console-setup-linux` 提供 PSF 原始点阵，生成器按 Unicode 映射提取 95 个可打印字符，转成 LCD 使用的列优先布局。安装相同字体后，运行 `python3 tools/gen_lcd_ascii_font.py --check` 可以逐字节核对固件中的两张表。4.3 寸简体中文字形单独存放。
