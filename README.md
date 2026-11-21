# lhy1105 双臂遥操作

本包读取 FE-URT-2 上的 14 个飞特 STS3215 舵机（TTL 总线，1 Mbps），把两条主手臂的 6 个关节和夹爪分别映射到本机的两条 YAM follower 臂。**URT-2 虽然曾被称为“CAN 接口”，实际在本机呈现为 USB 串口 `/dev/ttyACM0`；follower 臂才使用 CAN。**

本机已实测 ID 1–14 均能 PING，型号寄存器均返回 777，位置寄存器可读。默认假设 ID 1–7 是左主手、8–14 是右主手，各组最后一个 ID 是夹爪。左右 follower 接口与本机 `Dream-1/station.yaml` 一致，分别为 `can_follower_l` 和 `can_follower_r`，夹爪类型 `linear_4310`。

## 运行

### 用另外两条 YAM 臂作六关节从臂

本机的 `can_leader_l` 和 `can_leader_r` 各有 6 个 YAM 关节电机，末端是被动教学手柄，
没有第 7 个电动夹爪。`config_can_leader.json` 单独指向这两路 CAN，且设置
`follower_gripper: none`；默认 `config.json` 仍指向 `can_follower_l/r`。
替代配置的 `mapping_confirmed` 保持 `false`，逐关节试运行可用 `--trial`。

先在现场支撑两条臂、确认左右物理对应及急停可用，再逐关节小幅检查：

```bash
cd /home/noematrix/yanwen/lhy1105
PYTHONPATH=/home/noematrix/ForceGello/i2rt \
  /home/noematrix/miniconda3/envs/lhy/bin/python -m lhy1105 \
  --config config_can_leader.json multi --ids 1 8
```

确认 12 个关节方向后，可用原来的两条飞特主臂控制这两条 YAM 臂的六个关节：

```bash
PYTHONPATH=/home/noematrix/ForceGello/i2rt \
  /home/noematrix/miniconda3/envs/lhy/bin/python -m lhy1105 \
  --config config_can_leader.json multi --ids 1 2 3 4 5 6 8 9 10 11 12 13
```

此路径不读取飞特夹爪 ID 7/14 的运动，也不连接原来的 `can_follower_l/r`。
`multi` 会要求输入 `CONNECT` 和 `ARM`；正常退出时将连接的关节缓慢移到 0 rad，
然后释放力矩。要以 `live` 路径试运行全部 12 个关节，执行：

```bash
PYTHONPATH=/home/noematrix/ForceGello/i2rt \
  /home/noematrix/miniconda3/envs/lhy/bin/python -m lhy1105 \
  --config config_can_leader.json teleop --live --trial
```

此命令的启动检查与控制循环都跳过 ID 7/14，且保留关节速度限制。它同样要求输入
`CONNECT` 和 `ARM`。确认全部关节方向后，可将替代配置的 `mapping_confirmed`
改为 `true`，并去掉 `--trial`。

### 单关节实时角度和速度图

选择一个主臂舵机（1–6 或 8–13），窗口上方实时绘制主臂映射后的目标角度和从臂实际
反馈角度；下方绘制这两者的速度曲线（rad/s）。映射目标以 `ARM` 时的从臂姿态为基准，
已按配置方向转换，但尚未经过关节限位和速度限制；从臂角度来自电机反馈。速度由角度
数据按实际时间戳计算，使用约 0.1 秒的差分窗口平滑。绘图与控制分开运行。

```bash
cd /home/noematrix/yanwen/lhy1105
PYTHONPATH=/home/noematrix/ForceGello/i2rt \
  /home/noematrix/miniconda3/envs/lhy/bin/python scripts/plot_single_joint.py --id 2
```

省略 `--id 2` 可在终端输入要分析的 ID；也可加 `--window 30` 看最近 30 秒。
按终端提示输入 `CONNECT`、`ARM` 后再移动主臂；按 `Ctrl+C`、`x`、`q` 或关闭绘图窗口停止。
单关节程序退出时释放从臂力矩，不会自动回零，请支撑机械臂。每次运行的完整数据与
一张上下两块的 PNG 图片分别保存为 `1120/idN/idN_频率Hz_限速radps.csv` 和
`1120/idN/idN_频率Hz_限速radps.png`，例如 `1120/id2/id2_60Hz_0.8radps.png`。
频率和速度取本次配置的目标循环频率与该关节的 `max_velocity`；相同 ID、频率、速度
再次运行时会覆盖之前的 CSV 和图片。CSV 只保留采样时间、主臂映射目标角度、从臂反馈角度
和配置频率；在开始采样前退出则不生成图片。
实时窗口显示配置的循环目标频率、最近 3 秒的实际采样频率和采样间隔的第 95 百分位；
PNG 则显示本次运行的整体实际频率。实际值由 CSV 中相邻采样时间戳计算，反映此程序
完成主臂读取和从臂反馈的循环速度，不代表电机内部控制器的更新频率。

### 整体 live 的 12 关节角度图

左右从臂各 6 个关节同时运行时，用下面的脚本打开 12 格实时图。每格蓝线是主臂映射
后的目标角度（限位和限速前），绿线是对应从臂的实际反馈角度；不绘制速度或夹爪。
此脚本使用 `config_can_leader.json`，若映射尚未确认会沿用 `--trial` 模式。

```bash
cd /home/noematrix/yanwen/lhy1105
PYTHONPATH=/home/noematrix/ForceGello/i2rt \
  /home/noematrix/miniconda3/envs/lhy/bin/python scripts/plot_all_joints.py
```

在终端输入 `CONNECT`、`ARM` 后开始采集。按 `x`、`q`、`Ctrl+C` 或关闭图窗停止；
控制程序按原有逻辑让两条从臂回零后保存完整图片。CSV 和一张 12 格 PNG 保存在
`1120/all/all_频率Hz_限速radps.csv` 和同名 `.png`，例如当前配置会写入
`1120/all/all_200Hz_2radps.png`。若六个关节限速不同，文件名逐项列出六个限速值；
相同配置再次运行会覆盖对应文件。图上同时显示配置频率和实际采样频率。

### 逐关节 live 检查

需要同时检查多个关节时，用 `multi --ids` 列出主臂 ID。可以选同一侧或左右两侧的关节；只连接选中的从臂侧，未选关节保持启动目标。方向默认取 `config.json`，可用 `--reverse` 临时反转指定 ID 的方向。终端实时显示每个所选关节的主臂 tick、相对角度、从臂反馈、命令和目标，CSV 记录保存在 `calibration_runs/`。

```bash
cd /home/noematrix/yanwen/lhy1105
/home/noematrix/miniconda3/envs/lhy/bin/python -m lhy1105 multi --ids 1 2 3
# 左右两侧一起检查，并临时反转 ID 2：
/home/noematrix/miniconda3/envs/lhy/bin/python -m lhy1105 multi --ids 1 2 8 9 --reverse 2
# 连接并保持左右两条完整从臂，未选关节钉在 ARM 时的位置：
/home/noematrix/miniconda3/envs/lhy/bin/python -m lhy1105 multi --ids 1 2 8 9 --lock-others
```

程序要求依次输入 `CONNECT`、`ARM`。按下 `ARM` 时把所有所选主从关节的当前位置对齐，随后移动主臂。`--lock-others` 会连接左右两条完整从臂，把所有未选关节持续保持在 `ARM` 时的位置。按 `x`、`q` 或 `Ctrl+C` 正常退出，或到达 `--duration` 时间后，所有已连接从臂缓慢回到配置的六个关节 `0 rad`；有电动夹爪时还会移到 `0.99`，确认反馈到位再释放力矩。通信或反馈故障时不会盲目执行自动复位。主臂位移不设软件上限；从臂目标限制在 YAM 模型的标称关节范围内，超出时显示 `LIMIT`。I2RT 初始化时提供的关节范围额外向两侧扩展了 0.15 rad，标定模式不会把这部分当成可用运动空间。

选择 `--lock-others` 时，两侧夹爪即使没有被选中，也可能在 `CONNECT` 阶段由 I2RT 自动移动来检测行程；`ARM` 之后才会保持所有未选关节。运行和自动回零时都要支撑双臂。

先用单关节模式验证主从对应关系，再考虑双臂 `teleop --live`。每次只选一个主臂 ID：1–6 对应左从臂关节 1–6，7 对应左夹爪；8–13 对应右从臂关节 1–6，14 对应右夹爪。此模式**按编号直接对应**，不会采用 `config.json` 中可能交换顺序的 `joint_sources`；方向默认取对应关节的 `joint_signs`，可用 `--sign -1` 或 `--sign 1` 临时覆盖。检查结果不会自动改写配置。

```bash
cd /home/noematrix/yanwen/lhy1105
/home/noematrix/miniconda3/envs/lhy/bin/python -m lhy1105 calibrate --id 1
# 检查下一个关节时重新运行，并把 1 换成 2..14
```

命令先检查全部主臂舵机，再要求输入 `CONNECT` 和 `ARM`。连接后把主臂选中的关节放到舒适位置；输入 `ARM` 时记录双方起点。随后移动**选中的主臂关节**，从臂对应关节会按相对位移跟随，其他关节保持启动目标。主臂没有软件位移上限，默认持续运行；按 `x`、`q` 或 `Ctrl+C` 结束，也可用 `--duration 120` 设置最长运行时间。终端每次采样实时显示主臂 tick、相对角度、从臂反馈、命令和目标。

从臂的目标会停在 YAM 模型标称关节范围或夹爪 0–1 范围内；超出时终端显示 `LIMIT`，主臂可继续移动，程序不会因越界退出。命令仍按配置速度上限逐步跟随。若其他从臂关节偏离启动位置超过 0.08 rad，或通讯失效，程序会停止。

每次运行的 CSV 保存在 `calibration_runs/`，含主臂原始 tick、连续相对角度、原始目标、限位后的目标、命令、反馈和其他关节的最大漂移，退出时打印观测到的主从范围。检查夹爪 ID 7 或 14 时，I2RT 可能先自动移动**选中的夹爪**来检测行程；请确保夹爪周围没有障碍物。普通关节模式不会连接夹爪。此脚本不写飞特舵机零位，也不自动回 rest；结束后控制器关闭、力矩释放，请始终支撑从臂。未确认方向、行程和从臂零位前，不要启用双臂 `teleop --live`。

### 常规命令

`lhy` 环境已含 `numpy`、`pyserial`、`i2rt`。从本目录运行：

```bash
cd /home/noematrix/yanwen/lhy1105
/home/noematrix/miniconda3/envs/lhy/bin/python -m lhy1105 scan
/home/noematrix/miniconda3/envs/lhy/bin/python -m lhy1105 monitor
/home/noematrix/miniconda3/envs/lhy/bin/python -m lhy1105 teleop --duration 10
```

以上三个命令都不会控制 follower。最后一条只计算和显示左右关节目标的变化。

要运行真实遥操作，先确认主手 ID 分组、每个关节的方向与顺序，以及夹爪开合方向。编辑 `config.json` 中的 `joint_sources`、`joint_signs`、`gripper_sign`、`gripper_span_ticks`，完成现场小幅检查后把 `mapping_confirmed` 改为 `true`。两条机械臂周围保持空旷，支撑 follower，确认急停可用，并确保 Dream station 或其他程序没有同时连接这两条 CAN。然后：

```bash
/home/noematrix/miniconda3/envs/lhy/bin/python -m lhy1105 teleop --live
```

`teleop --live` 复用 `multi` 的双臂控制路径，并将主从位置记录到 `calibration_runs/`。默认配置使用全部 14 个 ID；无电动夹爪的替代配置只使用 12 个关节 ID。程序要求输入 `CONNECT`、`ARM`；I2RT 的 `linear_4310` 初始化可能执行夹爪标定。输入 `ARM` 时主手和 follower 当前位置对齐，运行中显示各关节主从位置。主手位移没有 0.5 rad 或 0.25 夹爪行程的启动姿态限制，单次读数变化和跟踪误差也不触发停机。超出 YAM 模型标称关节范围的目标停在边界并显示 `LIMIT`，夹爪目标限制在 0–1。默认配置的 `teleop --live` 直接发送限位后的目标；无夹爪配置、`multi` 校准和退出回零仍按 `max_velocity` 限速。

运行时按 `x`、`q` 或一次 `Ctrl+C` 会将两条 follower 缓慢移向 rest（六个关节零位，夹爪 0.99），检查反馈到位后再关闭控制器。**rest 指令不保证机械臂已经接触台面；全过程都要保持支撑，关闭后力矩会释放。**通讯或反馈故障时不会尝试盲目回 rest，而是直接停机并报告错误。

## 通讯和退出行为

- 实时控制前检查全部 14 个 ID 和型号；任何读取超时、校验失败或舵机报错立即停止。
- 主手采样或控制循环停滞、follower CAN 反馈失效时停止。
- 从臂目标饱和在 YAM 模型标称关节范围内；输出按配置的每关节速度上限平滑。
- 正常退出时参考 15:00 的双臂脚本缓慢回 rest，并检查最终反馈；故障时跳过自动运动。
- 默认只读演练；真实控制需显式 `--live`、确认映射并在终端二次输入。
- 不修改舵机 ID、零点、工作模式或电机超时设置；异常退出会关闭 I2RT 控制器。

软件控制不能替代硬件急停。主手可继续移动到从臂范围以外，但从臂不会越过其模型范围。

## 来源

关节增量映射和 follower API 参考本机 `~/uarm/i2rt/scripts/uarm2i2rt_right_follower.py`，以及 [LeRobot-Anything-U-Arm](https://github.com/MINT-SJTU/LeRobot-Anything-U-Arm) 的主从臂读取与夹爪控制思路。URT-2 读取使用[飞特 FT-SCS 协议](https://ftservo.github.io/ftservo-wiki/reference/protocol/ft-scs/)的 PING 和 READ 指令。
