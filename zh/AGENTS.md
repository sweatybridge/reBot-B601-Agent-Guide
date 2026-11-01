# B601-RS 机械臂使用指南

> 本文档仅适用于 **B601-RS 机械臂**。
> 除了需要用户给予权限的命令外，都由你在得到用户同意后执行，除了异常状况外，不要让用户自己转到终端处理。
> 不要使用文档中没有提及的 motorbridge-cli 指令，因为那可能会启动电机导致意外发生。

## 环境状态记录

执行流程期间，检查并更新 `memory/local-machine-env.md` 记录当前机器的环境信息，避免重复初始化：

- **初始化前**：先读取该文件，若已有 conda 路径、motorbridge 信息、电机配置等，可直接使用
- **每步完成后**：按参考文档中的提示，将新信息追加到该文件
- **文件不存在时**：跳过读取步骤，正常执行即可

## 快速开始

请确认你的机械臂属于以下哪种形式：

| 类型 | 说明 | 起始步骤 |
|------|------|----------|
| 散件版本 | 需要用户自行组装 | 从第一步开始 |
| 套件版本 | 已组装完成 | 直接跳转至第三步 |

---

## 第一步：环境初始化

运行 [环境初始化参考](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/zh/references/setup-environment.md) 完成以下安装：

- [ ] Python 环境准备（Miniforge 或已有 Python）
- [ ] motorbridge 安装
- [ ] 平台特定配置（Linux 配置 can0，Windows 安装 PCAN-USB 驱动）

完成后确保 motorbridge-cli 可用：

```bash
motorbridge-cli --help
```

> **Windows 用户**：使用 Miniforge 创建 conda 环境，不要使用系统 Python 直接 `pip install motorbridge`。如果已安装 `uv`，使用独立 uv venv 也是认可的替代方案（见参考文档中的 uv 小节）——无论哪种方式，都不要装进系统 Python。不需要配置 can0。详见 [环境初始化参考](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/zh/references/setup-environment.md)。

---

## 第二步：写入电机 ID

运行 [写入电机ID参考](https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/zh/references/write-motor-id.md)

该参考文档会引导你完成：

1. **识别 USB-CAN 转接板**（PCAN-USB；Linux 配置 `can0` bitrate `1000000`，Windows 驱动自动识别）
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

---

## 参考文档状态

| 参考文档 | 文件 | 状态 |
|----------|------|------|
| 环境初始化 | `references/setup-environment.md` | 已完成 |
| 写入电机 ID | `references/write-motor-id.md` | 已完成 |
| 零点校准 | `references/write-motor-zero.md` | 已完成 |
