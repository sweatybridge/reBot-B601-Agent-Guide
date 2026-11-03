# B601-RS 机械臂使用指南

> 本文档仅适用于 **B601-RS 机械臂**。
> 除了需要用户给予权限的命令外，都由你在得到用户同意后执行，除了异常状况外，不要让用户自己转到终端处理。
> 不要使用文档中没有提及的 motorbridge-cli 指令，因为那可能会启动电机导致意外发生。

## 环境状态记录

执行流程期间，检查并更新 `memory/local-machine-env.md` 记录当前机器的环境信息，避免重复初始化：

- **初始化前**：先读取该文件，若已有控制主机、conda 路径、motorbridge 信息、CAN 接口或电机配置等，可直接使用
- **每步完成后**：按参考文档中的提示，将新信息追加到该文件
- **文件不存在时**：跳过读取步骤，正常执行即可

至少记录以下信息：控制主机类型（本地／远程 SSH）、SSH 目标、CPU 架构、CAN 接口名称与速率、motorbridge CLI 路径、已配置的电机 ID。

---

## 第零步：确认控制主机与 CAN 通路

`motorbridge` 通过**本地** CAN 接口（`can0`）与机械臂通信，它没有网络／远程传输方式——`socketcan`、`socketcanfd`、`dm-serial`、`dm-device`、`mcu-serial` 都是本地的。因此运行 motorbridge 的机器**必须就是物理连接在机械臂 CAN 总线上的那台机器**。动手之前先确认属于哪种情况：

| 情况 | motorbridge 运行在 | 你如何访问 |
|------|-------------------|-----------|
| **A. 本地电脑** | 用户的电脑（Windows / macOS / Linux），接 USB-CAN 转接板 | 直接访问 |
| **B. 远程单板机（SSH）** | 树莓派／Jetson 等，板上有 CAN HAT 或 USB-CAN | `ssh <用户名>@<主机>` |

**情况 B** 下，后续步骤都要注意：

- 所有命令都通过 SSH 执行，例如 `ssh <用户名>@<主机> "motorbridge-cli scan --vendor robstride --channel can0 ..."`
- 与本地相同的原则：不要把命令丢给用户，让用户自己在板子的终端上执行
- 板子通常没有显示器，因此需要浏览器的步骤（Motorbridge Studio）要在**用户自己的电脑**上完成，并需要端口转发——见[第三步](#第三步校准零点)
- 长时间运行的进程（gateway）必须能在 SSH 断开后存活：使用 `setsid nohup ... >/tmp/gw.log 2>&1 &` 或 `tmux`，并记录 PID

### 主机支持检查（先做这一步）

`motorbridge` 的 wheel 中带有预编译的 `libmotor_abi.so` 与 `ws_gateway` 二进制，没有可用的源码安装回退方案——直接源码安装只会得到 Python 封装层，随后在加载 ABI 时失败。

| `uname -m` | 是否有 wheel | 结论 |
|------------|-------------|------|
| `x86_64` / `amd64` | 有 | 继续 |
| `aarch64` / `arm64`（64 位系统） | 有 | 继续 |
| `armv6l` / `armv7l`（32 位树莓派系统） | **没有** | **停止**——见下文 |

不安装任何东西就能探测：

```bash
python3 -m venv ~/mb-probe
~/mb-probe/bin/pip download motorbridge --no-deps --only-binary=:all: -d /tmp/mb-probe
```

若输出 `No matching distribution found for motorbridge`，说明该平台不受支持。**不要**改用 `--no-binary` 重试，也不要继续执行第一步。向用户说明以下选项并让用户选择：

1. 换用 64 位主机（Pi Zero 2 W / Pi 3 / Pi 4 / Pi 5 配 64 位系统，或任意 x86_64 Linux 机器）——同一块 CAN HAT，`pip install motorbridge` 直接可用
2. 为该架构从源码编译 `motor_abi` + `ws_gateway`（在 Pi Zero W 上需要数小时，超出本文档范围）
3. 把 CAN 转接板换到受支持的主机上
4. 继续使用这块 32 位板，改用纯 Python 的 `robstride_dynamics` 驱动——已在 Pi Zero W 上验证可用。板上只做关节空间，但配合工作站上的离线笛卡尔规划器（`examples/host/plan_cartesian.py` + `examples/host/rebot_ik.py`）可以把笛卡尔运动转成关节轨迹交给板子回放。仍然没有 Studio 校准界面，也没有重力补偿。详见 [Pi Zero W 演示开发与验证](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/zh/references/demo-validation-pi-zero.md)

> CAN HAT 本身仍然在第一步配置，32 位板同样可以配置；只有 Python／ABI 这一侧需要受支持的架构。

---

## 快速开始

请确认你的机械臂属于以下哪种形式：

| 类型 | 说明 | 起始步骤 |
|------|------|----------|
| 散件版本 | 需要用户自行组装 | 第零步 → 第一步 |
| 套件版本 | 已组装完成 | 第零步 → 第三步 |

---

## 第一步：环境初始化

运行 [环境初始化参考](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/zh/references/setup-environment.md) 完成以下安装：

- [ ] Python 环境准备（Miniforge 或已有 Python）
- [ ] motorbridge 安装
- [ ] 按实际硬件配置 CAN 通路

CAN 通路的配置取决于转接方式，而不只是操作系统：

| 转接方式 | 参考文档 |
|---------|---------|
| PCAN-USB（Linux / Windows / macOS） | [环境初始化](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/zh/references/setup-environment.md) |
| CANable / candleLight（SocketCAN） | [环境初始化](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/zh/references/setup-environment.md) |
| **CAN HAT，SPI 接口的 MCP2515**（树莓派常见） | [CAN HAT 配置](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/zh/references/setup-can-hat.md) |
| Damiao USB 串口 | [环境初始化](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/zh/references/setup-environment.md) |

完成后确保 motorbridge-cli 可用：

```bash
motorbridge-cli --help
```

> **Windows 用户**：使用 Miniforge 创建 conda 环境，不要使用系统 Python 直接 `pip install motorbridge`。如果已安装 `uv`，使用独立 uv venv 也是认可的替代方案（见参考文档中的 uv 小节）——无论哪种方式，都不要装进系统 Python。不需要配置 can0。详见 [环境初始化参考](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/zh/references/setup-environment.md)。
>
> **32 位树莓派系统用户**：Miniforge 没有 `armv6l`/`armv7l` 构建，motorbridge 也没有对应架构的 wheel。必须先解决第零步。

---

## 第二步：写入电机 ID

运行 [写入电机ID参考](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/zh/references/write-motor-id.md)

该参考文档会引导你完成：

1. **确认 CAN 接口**（`can0`，bitrate `1000000`；使用 CAN HAT 时接口必须已 up **且重启后依然有效**——见 [CAN HAT 配置](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/zh/references/setup-can-hat.md)）
2. **逐颗电机设置 ID**（共 7 颗电机，ID 范围 1–7）
   - 每次仅连接一颗电机
   - 扫描确认当前电机 ID（默认 127，部分批次已预设为 1~7）
   - 修改为用户指定的目标 ID
   - > **注意**：robstride 电机 `id-set` 命令可能报 `store_parameters failed` 超时错误，实际写入已生效，扫描确认即可
3. **最终检查** — 扫描确认所有电机 ID（1–7）均已正确写入

---

## 第三步：校准零点

参考 [零点校准文档](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/zh/references/write-motor-zero.md)。

该文档会引导你完成：

1. **启动 motorbridge-gateway**（绑定 `127.0.0.1:9002`，使用 `can0`）
2. **引导用户打开 Motorbridge Studio 网页** 完成零点校准

> **远程主机（情况 B）**：gateway 监听的是板子自己的回环地址，而 Studio 运行在用户电脑的浏览器里，因此必须做端口转发：
>
> ```bash
> ssh -L 9002:127.0.0.1:9002 <用户名>@<主机>
> ```
>
> 先在板子上以后台方式启动 gateway（脱离 SSH 会话也能存活），保持这条转发用的 SSH 会话不关，再让用户打开 <https://motorbridge.github.io/motorbridge-studio/> 并连接到 `127.0.0.1:9002`。完整流程见 [零点校准参考](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/zh/references/write-motor-zero.md)。

---

## 第四步（可选）：ROS 2

ROS 2 是支持的，但它**不是**第零步到第三步的替代方案——它建立在同一套底层之上，需要先完成电机 ID 与零点校准。

参考 Seeed 教程：[reBot Arm B601-RS ROS 2 集成](https://wiki.seeedstudio.com/rebot_arm_b601_rs_ros2_integration/)

它对环境的要求，也正是它会改变平台选择的原因：

| 要求 | 说明 |
|------|------|
| 主机系统 | Ubuntu 22.04（ROS 2 Humble）或 Ubuntu 24.04（ROS 2 Jazzy） |
| ROS 软件包 | `ros-${ROS_DISTRO}-pinocchio`、`control-msgs`、`trajectory-msgs`、`rviz2`，可选 MoveIt 2 |
| 工作空间 | `Seeed-Projects/reBotArmController_ROS2` + `Seeed-Projects/reBotArm_control_py` |
| CAN 访问 | ROS 2 节点在**本地**打开 `can0`，因此工作空间必须运行在连接机械臂的那台机器上 |
| motorbridge | 必需——第零步中的 wheel／架构限制同样适用 |

实际结论：在这台机械臂上用 ROS 2，需要一台 x86_64（或 64 位 ARM Ubuntu）主机加 CAN 转接板，而不是 32 位树莓派系统板。Raspbian 12 / `armv6l` 上跑 ROS 2 不在支持范围内。

---

## 参考文档状态

| 参考文档 | 文件 | 状态 |
|----------|------|------|
| 环境初始化 | `references/setup-environment.md` | 已完成 |
| CAN HAT 配置（SPI 接口 MCP2515） | `references/setup-can-hat.md` | 已完成 |
| Pi Zero W 演示开发与验证 | `references/demo-validation-pi-zero.md` | 已完成 |
| 写入电机 ID | `references/write-motor-id.md` | 已完成 |
| 零点校准 | `references/write-motor-zero.md` | 已完成 |
