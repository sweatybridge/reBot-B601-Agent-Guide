## 零点校准

> **注意：当前参考文档为测试版本，仅供参考。如有任何疑问或遇到问题，请联系 Seeed 获取支持。**

## 使用前提

- 已完成电机 ID 写入，电机 ID 已正确写入
- `can0` 已配置且处于 UP 状态（Linux 需在环境初始化中配置；树莓派 CAN HAT 需要先启用 SPI overlay 并重启，见 [CAN HAT 配置](setup-can-hat.md)；Windows/macOS 无需配置）
- 使用 CAN HAT 时，`can0` 必须在重启后依然生效：否则 Gateway 会报 `SocketCAN write failed: Network is down`

## 步骤

### 1. 激活环境并启动 Gateway

**Linux：**

```bash
conda activate rebot
motorbridge-gateway -- --bind 127.0.0.1:9002 --transport socketcan --channel can0
```

**macOS（推荐路由模式）：**

```bash
conda activate rebot
motorbridge-gateway -- --bind 127.0.0.1:9002
```

macOS 使用 PCAN-USB，无需配置 can0 接口。路由模式下仅需 `--bind`，web UI 会自动选择 vendor/channel/ID。如需指定通道，使用：

```bash
motorbridge-gateway -- --bind 127.0.0.1:9002 --channel can0@1000000
```

**Windows（推荐路由模式）：**

```bash
conda activate rebot
motorbridge-gateway -- --bind 127.0.0.1:9002
```

路由模式下仅需 `--bind`，web UI 会自动选择 vendor/channel/ID。如需指定通道，使用：

```bash
motorbridge-gateway -- --bind 127.0.0.1:9002 --transport socketcan --channel can0
# 或带比特率
motorbridge-gateway -- --bind 127.0.0.1:9002 --transport socketcan --channel can0@1000000
```

> **Windows `conda activate` 提示**：如果报错 "CommandNotFoundError"，说明 conda 未初始化当前 shell。
> 使用 Miniforge Prompt 执行，或使用完整路径：`<miniforge安装路径>\Scripts\activate.bat rebot`。
> Git Bash 用户参见 [环境初始化参考](setup-environment.md) 中的初始化说明。

Gateway 启动后保持运行，不要关闭该终端。

> **重要：Gateway 会独占 CAN 总线**。在 Gateway 运行期间，`motorbridge-cli scan` 和其他 CLI 命令将无法与电机通信，会返回 `0 motor(s) found`。
> 如需在 Gateway 运行期间扫描电机 ID，必须先停止 Gateway：
>
> ```bash
> # Linux
> pkill -f ws_gateway
> ```
>
> ```cmd
> :: Windows cmd
> taskkill /F /IM ws_gateway.exe
> ```
>
> ```powershell
> # PowerShell
> Stop-Process -Name "ws_gateway" -Force -ErrorAction SilentlyContinue
> ```
>
> **Windows 说明**：`motorbridge-gateway.exe` 只是一个启动器，真正占用总线和端口的是它的子进程 `ws_gateway.exe`。只结束启动器会留下 `ws_gateway.exe` 继续运行，端口仍被占用，下次启动会报 `Os { code: 10048, kind: AddrInUse }`。请结束 `ws_gateway.exe`，启动器会随之退出。
>
> 扫描完成后再重新启动 Gateway。建议在启动 Gateway 前完成所有 CLI 扫描和电机 ID 设置工作。

### 1b. 远程主机（SSH）

当机械臂接在一块板子上（树莓派／Jetson），而用户的浏览器在另一台电脑上时，Gateway 仍绑定在板子的回环地址，通过端口转发让用户电脑访问。**不要**绑定 `0.0.0.0`——那需要 `MOTORBRIDGE_WS_TOKEN`，并且会把电机控制暴露到网络上。

1. 在板子上以后台方式启动 Gateway，使其脱离 SSH 会话也能存活：

```bash
ssh <用户名>@<主机> "source ~/miniforge3/etc/profile.d/conda.sh && conda activate rebot && \
  setsid nohup motorbridge-gateway -- --bind 127.0.0.1:9002 --transport socketcan --channel can0 \
  >/tmp/motorbridge-gateway.log 2>&1 &"
```

2. 确认端口已监听（若没有，查看日志）：

```bash
ssh <用户名>@<主机> "ss -ltnp | grep 9002; tail -5 /tmp/motorbridge-gateway.log"
```

3. 在用户电脑上保持一条转发会话：

```bash
ssh -L 9002:127.0.0.1:9002 <用户名>@<主机>
```

4. 然后让用户打开 Motorbridge Studio 并连接到 `127.0.0.1:9002`——这个地址现在通过隧道指向板子上的 Gateway。

5. 校准完成后，在板子上停止 Gateway：

```bash
ssh <用户名>@<主机> "pkill -f ws_gateway"
```

> 整个校准期间隧道必须保持打开。如果浏览器报连接失败，先检查 SSH 会话，再看板子上的 `ss -ltnp`——而不是去改网页里填的地址。

### 2. 打开 Motorbridge Studio

提示用户访问：https://motorbridge.github.io/motorbridge-studio/

通过网页界面完成零点校准操作。
