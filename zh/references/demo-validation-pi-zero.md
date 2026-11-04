## 树莓派 Pi Zero W 上的演示开发与验证

> **注意：本参考文档为 beta 版本，仅供参考。如有疑问或遇到问题，请联系 Seeed 获取支持。**

> **文件布局说明。** 下文引用的示例后来合并为一条流水线，见 [`examples/README.md`](../../examples/README.md)：`examples/host/plan_with_pinocchio.py` 在 Docker 容器中规划（Seeed 的 IK 加 Pinocchio 逐路点重力前馈），`examples/edge/play_trajectory.py` 在板子上回放 JSON。`host/plan_cartesian.py`、`host/fit_gravity.py`、`host/plan_gripper.py`、`edge/gravity_comp_demo.py`、`edge/test_mit_frames.py`、`host/trim_trajectory.py` 等路径属于早期布局，已不存在；下文的测量结果与经验仍然成立。

本文记录纯 Python 运动演示（[`examples/edge/gradual_motion_demo.py`](../../examples/edge/gradual_motion_demo.py)）是如何在 32 位树莓派上构建并验证的，以及更有价值的部分——那些在动作到达机械臂之前就把真实缺陷抓出来的检查。在这类硬件上开发任何新的运动脚本，都建议按同样顺序进行。

**验证基线**

| 项目 | 值 |
|---|---|
| 板卡 | Raspberry Pi Zero W Rev 1.1（BCM2835，512 MB） |
| 架构 | `armv6l`（32 位，无 NEON） |
| 系统 / 内核 | Raspbian GNU/Linux 12 (bookworm) / `6.6.20+rpt-rpi-v6` |
| Python | 3.11.2（系统自带） |
| CAN | RS485 CAN HAT，SPI0 CE0 上的 MCP2515，12 MHz 晶振，`INT` = GPIO25 |
| 电机 | 7 颗 RobStride RS 系列：**1–3 号 RS-06，4–7 号 RS-00** |
| 库 | Seeed `RobStride_Control` → `robstride_dynamics`（纯 Python） |
| 软件包 | `python3-can` 4.1.0、`python3-numpy` 1.24.2、`python3-tqdm` 4.64.1（均由 apt 安装） |

## 关卡 1：这套栈到底能不能跑？

在安装任何东西之前先探测（见 `AGENTS.md` 第零步）：

```bash
python3 -m venv ~/mb-probe
~/mb-probe/bin/pip download motorbridge --no-deps --only-binary=:all: -d /tmp/mb-probe
# ERROR: No matching distribution found for motorbridge
```

`motorbridge` 没有 `armv6l`/`armv7l` 的 wheel——PyPI 上只有 `linux_x86_64`、`linux_aarch64`、`macos_arm64` 和 `win_amd64`——而它的 sdist 只是一个薄薄的 Python 封装，真正的实现在预编译的 `libmotor_abi.so` 与 `ws_gateway` 二进制里（`abi.py` 会依次查找 `MOTORBRIDGE_LIB`、`motorbridge/lib/`、`target/release/`）。在 `armv6l` 上没有任何可加载的东西，所以源码安装得到的模块能 import，但一运行就失败。Miniforge 同样帮不上忙——没有 `armv6l`/`armv7l` 构建。

**结论**：motorbridge 以及建立在它之上的一切（Motorbridge Studio、零点校准界面、`reBotArm_control_py`、ROS 2 工作空间）都无法在这块板上运行。保留 CAN 通路，另选一个客户端库。

## 关卡 2：把 `can0` 拉起来，并让它熬过重启

HAT 装上了但完全没工作：SPI 被注释掉、没有 overlay、`can0` 不存在。

```bash
grep -n "spi=on\|mcp2515" /boot/firmware/config.txt   # bookworm；更早的系统是 /boot/config.txt
# dtparam=spi=on
# dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000
sudo reboot
sudo dmesg | grep mcp251          # mcp251x spi0.0 can0: MCP2515 successfully initialized.
sudo ip link set can0 type can bitrate 1000000 restart-ms 100 && sudo ip link set can0 up
ip -details link show can0        # can state ERROR-ACTIVE, bitrate 1000000, clock 6000000
```

有两件事会浪费你的时间，值得提前知道：

1. **`clock 6000000` 是对的。** 驱动上报的是晶振频率的一半（`mcp251x.c` 中 `can.clock.freq = freq / 2`），12 MHz 的 HAT 这里永远是 6 MHz。不要因此去"修" overlay。
2. **让 `can0` 持久化是开机时序问题，不是 `ip link` 的问题。** 最初的 systemd 单元以 `Cannot find device "can0"` 失败：在 Zero W 上，`mcp251x` 驱动要到开机后约 45–50 秒才创建接口，那时单元早就跑完了。可用的单元会等待 netdev 出现（`.device` 依赖加 `ExecStartPre` 中有界循环）——见 [CAN HAT 配置](setup-can-hat.md#第-6-步配置持久化推荐)。要用真实的 `reboot` 验证，而不是 `systemctl start`。

完整流程与故障排查表见 [CAN HAT 配置](setup-can-hat.md)。

## 关卡 3：在写运动代码之前，先证明总线与电机是好的

三个只读检查，从最省事的开始。

```bash
# 1. MCP2515 回环：证明 SPI + 驱动 + 波特率，不涉及电机
sudo ip link set can0 type can bitrate 1000000 loopback on
# ... 用 CAN_RAW socket 发一帧再读回来 ...

# 2. 总线上是否有节点：普通模式下向电机不处理的 ID 发一帧
#    被 ACK -> 保持 ERROR-ACTIVE、tx_errors=0；没有节点 -> BUS-OFF

# 3. 电机是否存在：逐个 ping（只读，不上力矩）
RobstrideBus.scan_channel("can0", 1, 9)     # -> 电机 1..7 及 UUID
bus.read(name, ParameterType.MECHANICAL_POSITION)   # -> 关节角度（rad）
```

回环测试不需要额外软件包——只要开启 `CAN_RAW_RECV_OWN_MSGS` 并让接口处于 loopback 模式，stdlib 的 `socket(AF_CAN, SOCK_RAW, CAN_RAW)` 就够了。这一点很重要：它让你在什么都不装的情况下，把"HAT 坏了"和"电机没上电"区分开。

## 关卡 4：让脚本以"拒绝"为核心设计，而不是以"功能"

| 设计决定 | 原因 |
|---|---|
| 默认 dry run，必须加 `--execute` 才动 | 随手敲的 `python3 demo.py` 不应该让一台 6 自由度机械臂动起来 |
| 起始位姿取自 **ENABLE 状态帧**，绝不假设 | 机械臂的零点在两次会话之间已经变过一次；假设零点是机械臂被错误驱动的典型来源 |
| `\|delta\| ≤ 0.35 rad`、增益限幅、目标保持在 ±4π MIT 范围内侧 0.2 rad | 命令行参数只能让运动更小，不能更大 |
| 故障帧（通信类型 21）抛出异常而不是被丢弃 | 否则循环会继续给已经报故障的电机下发指令 |
| 每一段都从**上一次下发的**位置开始插值 | 见缺陷 1 |
| 退出时回位、保持，然后断力矩（或 `--stay-enabled`） | 崩溃安全，并把重力下垂的取舍显式化 |

## 关卡 5：用测量来验证，而不是用挂钟时间

验证过程中发现了三个缺陷，它们最初都产生了"看起来很合理"的输出。

| # | 现象 | 根因 | 修复 |
|---|---|---|---|
| 1 | 回程"用了 3.03 秒"，看起来是渐进的 | 插值总是从初始化位姿开始，于是"目标就是起始位姿"的那一段在第一帧就发出了该位姿——一次跳变，然后干等 3 秒 | 改为从最近一次下发的位置插值（`self.last`） |
| 2 | `--track` 输出中出现 `WARNING: Invalid device ID, got 7, expected 1` | 多颗电机同时使能后，状态帧到达顺序不固定；"读取下一帧"把别的关节位置算到了这个关节上 | `read_status()` 按设备 ID 过滤，超时后明确失败 |
| 3 | 3、4 号关节稳定在目标以下 0.05–0.14 rad | 机械臂此时要承担自身重量；单一的 `--kp` 无法同时适配空载的基座关节和带载的腕部关节 | 分关节增益（`--kp 4:45,5:35`） |

缺陷 1 是最需要内化的：**循环跑满了预定时间，完全不能证明运动是连续的。** 真正把它变成证据的是 `--track`（在每段 25/50/76/100 % 处读取实测位置）。

缺陷 2 的影响不止于日志：同一个调用在初始化时决定了起始位姿，而错误的读数会成为第一条指令的目标。

## 测量结果

读这些数字时要注意一点：这些运行把所有电机都按 rs-00 的参数表发送。1–3 号是 RS-06，kp 范围是 rs-00 的十倍，所以它们的实际刚度是命令行 `--kp` 的十倍（kp 15 → 150 Nm/rad）；rs-00 的腕部关节与命令一致。现在的脚本默认使用正确的每关节型号，要复现这些具体数字需要加 `--model-spec ""`——本页末尾的命令行里已经加上。

单关节，δ = 0.0524 rad（3°），kp 15 / kd 0.6，30 Hz——实测位置：

| 段 | 24 % | 50 % | 76 % | 100 % |
|---|---|---|---|---|
| 去程 | +0.0284 | +0.0456 | +0.0629 | +0.0736 |
| 回程 | +0.0717 | +0.0548 | +0.0376 | +0.0268 |

两段各约 3.03 秒（目标 3 秒，30 Hz 保持住），最终回到起点 2 mrad 以内。

全部 7 个关节，δ = 0.1048 rad（6°），`--track`，远点处的跟随误差（实测 − 指令）：

| 关节 | kp 15 | kp `4:45 5:35` |
|---|---|---|
| 1 | −0.002 | −0.003 |
| 2 | −0.008 | −0.005 |
| 3 | −0.050 | −0.037 |
| 4 | −0.141 | −0.045 |
| 5 | −0.008 | −0.004 |
| 6 | −0.007 | −0.003 |
| 7 | −0.001 | +0.002 |

3、4 号关节始终偏短——4 号即使给到 45 Nm/rad、3 号仍是 20 的基准值——而机械臂静止时两者都能跟到约 2 mrad，这说明是负载而非调参问题。正确的解法是重力补偿：`reBotArm_control_py` 里有，但依赖 motorbridge；现在这条纯 Python 通路上也有了[重力补偿示例](../../examples/edge/gravity_comp_demo.py)，只是要按实测下垂标定。时序：20 Hz 下 3 秒的段实际跑了 3.35–3.40 秒，也就是**7 关节下约 18 Hz 有效速率**——差不多就是 Zero W 加这块 HAT 的上限。

## 下一次 Pi Zero W 会话的检查清单

1. `dpkg -s python3-can python3-numpy python3-tqdm`——用 apt 装，不要用 pip（避免在 512 MB 的 armv6 CPU 上编译 numpy）
2. `ip -details link show can0`——`ERROR-ACTIVE`、`bitrate 1000000`
3. 任何动作之前先扫描电机，确认 ID 1–7
4. **先 dry run**——计划应该与你期待看到的现象一致
5. 先验证单个关节，再验证七个；其他关节松垮会掩盖重力负载
6. 使用 `--track`；把每段的 `done in +X.XXs` 与 `--duration` 对比，判断循环是否超时
7. 7 关节时 `--hz ≤ 20`，1–2 关节时 `≤ 30`
8. 断力矩后机械臂会下垂——下一次运行前重新读取位置，不要沿用上一次的起始位姿

## 复现这些运行

```bash
# 一次性准备（用 apt，不用 pip：避免在 512 MB 的 armv6 CPU 上编译 numpy）
curl -sL -o /tmp/rs.tar.gz https://codeload.github.com/Seeed-Projects/RobStride_Control/tar.gz/refs/heads/master
tar xzf /tmp/rs.tar.gz && mv RobStride_Control-master ~/RobStride_Control
sudo apt-get install -y python3-can python3-numpy python3-tqdm

# 单关节，3 度，带实测
# （--model-spec "" = 上表实测时所用的 rs-00 参数表）
python3 edge/gradual_motion_demo.py --sdk-path ~/RobStride_Control/python \
    --model-spec "" --execute --joints 1 --delta 0.0524 --duration 3 --kp 15 \
    --hz 30 --track

# 全部关节，6 度，腕部更硬
python3 edge/gradual_motion_demo.py --sdk-path ~/RobStride_Control/python \
    --model-spec "" --execute --joints all --delta 0.1048 --kp 4:45,5:35 \
    --hz 20 --track

# 用每关节型号（现在的默认）时 --kp 就是真实刚度，下面这条在 1–3 号上比上面软十倍
python3 edge/gradual_motion_demo.py --sdk-path ~/RobStride_Control/python \
    --execute --joints all --delta 0.1048 --kp 20 --hz 20 --track
```

## 离线笛卡尔规划

树莓派无法在控制回路里做 IK：在 Zero W 上一次阻尼最小二乘求解要花约 1.3 秒，20 Hz 完全没有希望。可行的分工——这也是示例代码按 `edge/`、`host/`、`trajectories/` 组织的原因：

| 阶段 | 在哪 | 文件 | 做什么 |
|---|---|---|---|
| 1. 快照 | edge | `edge/dump_joints.py` | 读取 1–6 号电机的 `0x7019`，输出 JSON。只读，不上力矩。 |
| 2. 规划 | host | `host/plan_cartesian.py` + `host/rebot_ik.py` | 在当前位姿做 FK，按笛卡尔增量逐点求解 IK，写出关节空间轨迹 |
| 3. 回放 | edge | `edge/play_trajectory.py` | 按 `control_hz` 逐帧下发。不做 IK，不需要 numpy。 |

```bash
# 在边缘设备上
python3 edge/dump_joints.py --sdk-path ~/RobStride_Control/python > current.json
# 在主机上
python3 host/plan_cartesian.py --joints current.json --delta 0,0,0.10 \
    --duration 5 --pause 1.0 --hz 20 --out trajectories/raise10cm.json
# 回到边缘设备
python3 edge/play_trajectory.py trajectories/raise10cm.json            # dry run
python3 edge/play_trajectory.py trajectories/raise10cm.json --execute
```

运动学来自 Seeed 的 URDF（`ReBot_Arm_RS.urdf`），转录进 `host/rebot_ik.py`。六个旋转关节，**腕部不是球形腕**——最后三根轴相距 104 mm 和 60 mm——因此没有闭式解，只能用数值方法。

### 本板的控制回路预算

把 7 号电机加进六个手臂关节后实测：

| 项目 | 值 |
|---|---|
| 一帧 7 关节 MIT 指令 | **50 ms**（收掉 7 条回包再加 3.8 ms；电机使能时约 72 ms/帧） |
| 20 Hz 预算 | 50 ms——**超出**，25 秒的轨迹会跑成 35 秒 |
| 15 Hz 预算 | 66.7 ms——勉强，22.1 秒的轨迹跑成 23.9 秒 |
| 一帧 6 关节 MIT 指令 | 约 43 ms——能满足 20 Hz，这也是 6 关节运行时序没问题的原因 |

开销在 `write_operation_frame` 内部：每颗电机约 6 次 numpy 标量运算，在 1 GHz 的 ARMv6 上每次约 1 ms。两个结论：7 关节轨迹按 15 Hz 跑（想留余量就用 12 Hz）；不要试图通过优化读取侧来解决时序问题——在实测写入路径之前，这一点被误判过两次。

**真正的解法是不再调用 numpy。** `edge/mit_frames.MitSender` 用普通算术和 `struct` 构造完全相同的 MIT 帧：7 关节一帧 8.2 ms，而不是 50.6 ms（6.1 倍），足够支撑舒适的 30–40 Hz。`edge/test_mit_frames.py` 会在位置、增益、力矩（含越界值）上与 `write_operation_frame` 逐字节比对——488 帧、0 处不一致——通过之后才允许它驱动电机。

用手工构造的帧跑 30 Hz 时，同一段序列 **22.00 秒的计划跑出 22.00 秒，帧间隔均值 33.3 ms、最大 34.8 ms，660 帧中 0 帧迟到**。之前的 15 Hz 运行慢了 8–9 %，并且有明显的台阶感，因为 66 ms 的指令间隔配上 kp 30–45，会让电机一格格执行位置指令、然后在两格之间停住。

### 10 cm 抬升的结果（已验证）

计划：221 个路点，上升 5 秒、保持 1 秒、下降 5 秒，20 Hz，峰值关节速度 0.137 rad/s，最差 IK 残差 0.49 mm。

| 项目 | 值 |
|---|---|
| 末端计划抬升 | 99.5 mm |
| **实测**抬升（对实测关节做 FK） | **78.8 mm** |
| 上升过程中的横向漂移 | 0.9 mm |
| 回到起点（实测） | −1.7 mm |
| 回放时间 / 计划 | 11.00 s / 11.00 s |
| 顶点处关节误差 | 2 号 −3 mrad、3 号 −18 mrad、4 号 −43 mrad |

所以*路径*是对的——垂直、笔直、也回到了起点——但末端低了 21 mm。这是重力下的关节柔度，不是 IK 误差：规划器自身残差只有 0.5 mm。4 号关节承担腕部，就是它让掉了（43 mrad ≈ 2.5°）。

第二次跑到 **20 cm**（同样的曲线，实测 180.9 mm / 计划 199.5 mm）短了 18.6 mm。第三次是复合动作——先上 25 cm，再前移 10 cm，转角停 0.5 秒——垂直方向 225.2 mm / 250.2 mm、水平方向 102.7 mm / 99.8 mm，**z 方向差 25 mm、x 方向差 3 mm**。

也就是说，10–25 cm 的动作上，差值稳定在 18–25 mm 区间，并且随着机械臂伸展只缓慢变大，说明它由腕部柔度主导，而不是比例误差。因此幅度越大，百分比精度越好：10 cm 时短 21 %、20 cm 时短 9 %、25 cm 时短 10 %（伸展增加抵消了一部分收益）。

重力前馈做过建模（`host/rebot_ik.gravity_torques`，与 `dU/dq` 的数值梯度比对误差 3.5e-9 Nm）。把模型与机械臂在抬升顶点实际表现出的下垂对比，kp 取**实际生效值**：

| 关节 | 实际 kp | 下垂 | 推算负载 | URDF 模型 | 比值 |
|---|---|---|---|---|---|
| 2 号 | 200 Nm/rad | 0.005 | 1.0 Nm | 1.75 Nm | 0.57 |
| 3 号 | 200 Nm/rad | 0.037 | 7.4 Nm | 5.81 Nm | 1.27 |
| 4 号 | 45 Nm/rad | 0.045 | 2.0 Nm | 1.67 Nm | 1.20 |

这些 kp 不是命令行上的值。1–3 号是 RS-06，MIT 的 kp 范围是 5000 Nm/rad，而 rs-00 是 500；用 rs-00 表（这些运行正是如此）时，kp 20 在基座三个关节上其实是 200 Nm/rad；腕部是 rs-00，kp 45 就是 45。模型误差在 30–40 % 以内，说明 78 mm 的差值来自负载下的关节柔度，把模型力矩灌进去就是正确的修法——下面这个示例正是这么做的。

### 真正用上它：带标定的示例

[`examples/edge/gravity_comp_demo.py`](../../examples/edge/gravity_comp_demo.py) 把最后这一步做完了，而不只是写一句建议，并且已经在这台机械臂上跑过。`--mode hold` 先用位置控制保持一个位姿，把下垂量读成 `kp × 下垂`，据此对模型求每关节的比例系数，再用由实测位姿逐周期重算的前馈保持一次。

在上面那次 10 cm 抬升的顶点、用上机械臂需要的每关节型号（`--model-spec "1:rs-06,2:rs-06,3:rs-06"`）、kp 30、10 Hz 下实测：

| 关节 | 模型 Nm | 实测负载 Nm | 系数 | 补偿前下垂 | 补偿后下垂 |
|---|---|---|---|---|---|
| 2 号 | 1.40 | 0.86 | 0.613 | 0.0287 | 0.0095 |
| 3 号 | 6.22 | 7.46 | 1.199 | 0.2485 | 0.0909 |
| 4 号 | 1.65 | 1.85 | 1.120 | 0.0616 | 0.0183 |

平均 |下垂| 从 0.057 rad 降到 0.020 rad（消除 65 %），各受力关节 63–70 %；系数落在 0.6–1.2，说明 URDF 模型大体是对的、标定只是在修小差。

这次运行把三件事坐实了：

- **前馈只能抵消它超过的那部分负载。** 3 号施加 7.46 Nm，收回它 0.25 rad 下垂中的 63 %；余下的是摩擦与静摩擦，它决定小力矩关节能跟到多准。
- **顶在限位上的位姿测到的是支撑力，不是重力。** 同样的实验在静止位姿（2、3 号正好压在 0 rad 限位上）得到 0.08 和 0.63 的系数、只消除 4 %：那时机械臂靠在支撑物上。标定前先把机械臂抬离支撑。
- **这个模型在这块板子上每次调用约 80 ms**，所以示例默认 10 Hz、周期正好 100 ms。按 20 Hz 跑会出现 90 ms 的周期、一半超时——瓶颈是 numpy，不是总线。

`examples/host/fit_gravity.py` 会用多个位姿拟合每个关节的系数，并报告它是否可迁移。安装与用法见[示例说明](../../examples/README.md#gravity-compensation)。

### 夹爪（7 号电机）

7 号电机不在手臂链上，所以没有 IK 可言：`plan_gripper.py` 写一条单电机轨迹，`edge/play_trajectory.py` 像回放其他轨迹一样回放它，期间手臂其余关节保持断力矩。

在参考机上有两个实测结论：

- **闭合止点大约在 −0.015 rad，而不是 0。** 夹爪合拢时编码器读到 −0.015，再往下的指令没有任何作用：回放器的对齐段请求 −0.0277，整段实测都停在 −0.015。规划时把 `--start-rad`/`--close-rad` 设在实测止点上，别让电机一直顶着它（kp 10 下那是稳定约 0.13 Nm 的白白消耗）。
- **跟踪到 4.12 rad 是干净的**（实测 4.083–4.111，没有卡死），说明行程超过 4.12 rad，Seeed 所说的 0–5 rad 闭合到全开是可信的。

rad 到厘米的换算是小齿轮几何，URDF 并没有建模（它把手指建成移动副，0.05 m + 0.0715 m）。默认的 0.412 rad/cm 来自"5 rad 对应那约 12 cm 行程"的假设，而且它是以闭合止点为基准的线性映射，所以用尺子量一次就能定标。

### 用手测量抬升量

末端是纯平移（姿态保持不变），所以**夹爪上的任意一点**上升量都相同——把这一个点对着固定的外部参照测量即可。**不要**在两个关节之间量：不同连杆上的两个点描述的是弦长，不是位移。对 99.4 mm 的末端抬升，2 号到 5 号的中心距会长 88.6 mm，而同一测量若取边沿到边沿又会得到别的数——足以让 78 mm 的结果看起来像"大约 100 mm"。

### 开发中踩到的两个 IK 细节

1. **关节限位余量不能顶走本来就位于余量带内的关节。** 2 号关节静止在 0.005 rad，而限位下界是 0；按 0.02 rad 余量做约束会悄悄把初值推到 0.02，于是每次求解都差约 1 mm 收不回去。现在的做法是 `min(lo+margin, seed) .. max(hi−margin, seed)`。
2. **余量带还必须包含计划的起始位姿**（`anchor=`）。否则回程会在最后 4 % 卡住——它回不到最初出发的位姿，因为那个位姿落在余量带内部。

## 已知限制

- **笛卡尔运动是离线规划的。** `host/rebot_ik.py` + `host/plan_cartesian.py` 在工作站上运行；树莓派只回放关节路点。末端的精度受关节柔度限制（kp 30–45 时 100 mm 的抬升差 21 mm），而不是受规划器限制。
- **厂商／motorbridge 通路上没有重力补偿。** 模型存在且已数值验证（`host/rebot_ik.gravity_torques`），[`examples/edge/gravity_comp_demo.py`](../../examples/edge/gravity_comp_demo.py) 在纯 Python 通路上用上了它——前提是用对每关节的电机型号（1–3 号 RS-06、4–6 号 RS-00），并按实测下垂拟合该位姿的系数。在位姿顶点它能消除约 65 % 的静态下垂，余下的是关节摩擦。厂商的 `reBotArm_control_py` 通路不受这个示例影响，仍然需要 motorbridge。
- **随库附带的 `write_id()` 是坏的**：它调用了不存在的 `CommunicationType.SET_CAN_ID`（正确常量是 `SET_DEVICE_ID = 7`）。ID 设好之后无关紧要，但不要相信那个方法。
