# reBot B601-RS & B601-DM Workflow Guide

> **Note: All reference documents are beta versions and for reference only. If you have any questions or encounter issues, please contact Seeed for support.**

To let the agent help you initialize the robotic arm, copy the prompt below and send it to your agent:

## Quick Start

### English

```
Please follow the workflow in AGENTS.md (https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/en/AGENTS.md) to help me initialize the robotic arm.
```

- [AGENTS Workflow](en/AGENTS.md) — Complete robotic arm initialization workflow
- [Environment Setup](en/references/setup-environment.md) — Python, motorbridge, platform config
- [CAN HAT Setup](en/references/setup-can-hat.md) — Raspberry Pi + MCP2515 CAN HAT (SPI)
- [Demo Development & Validation](en/references/demo-validation-pi-zero.md) — Building and validating a motion demo on a Pi Zero W
- [Write Motor ID](en/references/write-motor-id.md) — Set motor IDs 1–7
- [Zero Calibration](en/references/write-motor-zero.md) — Calibrate motor zero positions
- [Examples](examples/README.md) — `edge/` (Pi), `host/` (workstation) and `trajectories/` code

### 中文

```
请参考 AGENTS.md（https://github.com/Welt-liu/reBot-B601-Agent-Guide/blob/main/zh/AGENTS.md）中的流程，帮助用户完成新机械臂的初始化。
```

- [AGENTS 工作流](zh/AGENTS.md) — 机械臂初始化完整流程
- [环境初始化](zh/references/setup-environment.md) — Python、motorbridge、平台配置
- [CAN HAT 配置](zh/references/setup-can-hat.md) — 树莓派 + SPI 接口 MCP2515 CAN HAT
- [演示开发与验证](zh/references/demo-validation-pi-zero.md) — 在 Pi Zero W 上构建并验证运动演示
- [写入电机 ID](zh/references/write-motor-id.md) — 写入电机 ID 1–7
- [零点校准](zh/references/write-motor-zero.md) — 校准电机零点
