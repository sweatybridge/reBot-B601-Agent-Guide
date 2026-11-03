# 本地机器环境配置

> **使用说明**：此文件用于记录当前机器的环境信息，避免重复初始化和配置。
> Agent 在执行流程时会自动读取和更新此文件。
>
> **文件位置**：请将此文件放在当前项目或固定路径下，执行 Skill 前确认 `memory/local-machine-env.md` 可被访问。

---

## 控制主机

<!-- 确认控制主机后由 Agent 自动写入 -->

- 主机类型（本地 / 远程 SSH）：
- SSH 目标（`用户名@主机`）：
- 操作系统 / 版本：
- CPU 架构（`uname -m`）：
- motorbridge wheel 是否受支持（是 / 否）：

## CAN 接口

<!-- CAN 通路配置完成后由 Agent 自动写入 -->

- 转接方式（PCAN-USB / CANable / CAN HAT MCP2515 / damiao 串口）：
- 接口名称与速率（`can0` @ 1000000）：
- 是否重启后自动生效（是 / 否，以及实现方式）：
- CAN HAT overlay 配置行 / 配置文件：

## conda

<!-- 安装 Miniforge 后由 Agent 自动写入 -->

- 安装路径：
- conda.sh：

## motorbridge

<!-- motorbridge 安装成功后由 Agent 自动写入 -->

- CLI 路径：
- 安装方式：

## PCAN-USB

<!-- 驱动安装完成后由 Agent 自动写入 -->

- 驱动状态：

## 达妙串口

<!-- 确认 COM 端口后由 Agent 自动写入 -->

- 串口：

## 电机配置

<!-- 收集设备信息后由 Agent 自动写入 -->

- 电机类型（robstride / damiao）：
- 设备版本（散件版 / 套件版）：
- 目标 ID 范围：

### 已设置的电机

<!-- 每颗电机 ID 设置成功后由 Agent 追加 -->

- 已设置 ID：
- 最终扫描时间：
