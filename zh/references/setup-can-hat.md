## 树莓派 CAN HAT 配置（SPI 接口的 MCP2515）

> **注意：本参考文档为 beta 版本，仅供参考。如有疑问或遇到问题，请联系 Seeed 获取支持。**

当机械臂的 CAN 总线通过 **CAN HAT**（接在 40 pin 排针 SPI 上的 MCP2515 控制器）连接到控制主机，而不是通过 USB-CAN 转接板时，请使用本文档。对于这类硬件，它替代[环境初始化](setup-environment.md)中 `peak_usb` / `can0` 相关步骤。

**验证环境**：Raspberry Pi Zero W Rev 1.1、Raspbian GNU/Linux 12 (bookworm)、内核 6.6.20、微雪 RS485 CAN HAT（12 MHz 晶振）。同样的步骤适用于任何带 40 pin 排针的树莓派，以及其他 SPI 接口的 MCP2515 板卡。

## 开始之前

- [ ] HAT 已完全插紧在 40 pin 排针上（没插到底通常会在第 3 步失败）
- [ ] HAT 的 `CAN_H` / `CAN_L` 接到机械臂的 `H` / `L`（H-H、L-L，不能交叉）
- [ ] 机械臂使用独立电源供电。**不要用树莓派的 5V 给机械臂供电。**
- [ ] 总线终端电阻：HAT 上有板载 120 Ω 电阻（跳线或焊盘）。仅当 HAT 位于总线末端时才启用；机械臂一端已经自带终端。1 Mbit/s 下两端各 120 Ω 并联成 60 Ω 在台面上通常也能通，但规范做法是每端一个终端。

## 第 1 步：确认 HAT 的晶振频率

overlay 必须写入真实的晶振频率，否则控制器的实际波特率不对，电机永远不会应答。

- **12 MHz**（现行版本，晶振丝印为 `12.000`）→ 使用 `oscillator=12000000`
- **8 MHz**（2019 年 8 月之前购买的版本）→ 使用 `oscillator=8000000`

## 第 2 步：启用 SPI 与 overlay

根据系统版本编辑对应的启动配置文件：

| 系统版本 | 路径 |
|---|---|
| Bookworm (12) 及更新 | `/boot/firmware/config.txt` |
| Bullseye (11) 及更早 | `/boot/config.txt` |

追加以下内容（只加一次，先用 `grep mcp2515 <文件>` 确认是否已存在）：

```bash
sudo tee -a /boot/firmware/config.txt >/dev/null <<'EOF'

# B601-RS: RS485 CAN HAT (MCP2515, 12MHz crystal, INT=GPIO25)
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000
EOF
```

- `interrupt=25` 是 BCM GPIO25，即 HAT 的 `INT` 引脚。如果该引脚没接或编号写错，控制器仍能初始化，但会**丢失接收到的帧**，表现为"电机偶尔不应答"。
- `spimaxfrequency=2000000` 是微雪默认值，1 Mbit/s 下足够安全。若在高负载下看到 `rx_over_errors` 持续增长，可以提高（MCP2515 最高支持 10 MHz）。

## 第 3 步：重启并确认控制器

```bash
sudo reboot
```

重启后执行：

```bash
sudo dmesg | grep -i mcp251
```

预期输出：

```
mcp251x spi0.0 can0: MCP2515 successfully initialized.
```

如果看到的是 `Cannot initialize MCP2515. Wrong wiring?`，说明 HAT 没插紧、SPI 没启用或 overlay 缺失。

## 第 4 步：以 1 Mbit/s 启动 `can0`

```bash
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 65536
```

确认：

```bash
ip -details link show can0
```

关键输出：

```
can state ERROR-ACTIVE restart-ms 100
	  bitrate 1000000 sample-point 0.666
	  clock 6000000
```

> **`clock 6000000` 是正确的，不是配置错误。** `mcp251x` 驱动上报的是晶振频率的一半（驱动中 `can.clock.freq = freq / 2`），所以 12 MHz 的 HAT 这里始终显示 6 MHz。真正要看的是 `bitrate 1000000`。

## 第 5 步：不接机械臂，先自测 HAT

这一步在怀疑电机之前，先证明 SPI、驱动和波特率都正常。它不需要额外安装任何软件包——回环发生在 MCP2515 芯片内部，因此**不会**测试收发器和接线。

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 loopback on
sudo ip link set can0 up
sudo python3 - <<'PY'
import socket, struct
SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, CAN_RAW_LOOPBACK = 101, 4, 3
s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, 1)
s.bind(("can0",))
s.setsockopt(SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, 1)
s.setsockopt(SOL_CAN_RAW, CAN_RAW_LOOPBACK, 1)
s.settimeout(1.0)
s.send(struct.pack("=IB3x8s", 0x123, 2, b"\xde\xad" + b"\x00" * 6))
cid, dlc = struct.unpack("=IB3x", s.recv(16)[:8])
print("loopback ok: id=0x%X dlc=%d" % (cid, dlc))
PY
```

预期输出：`loopback ok: id=0x123 dlc=2`。之后恢复普通模式：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100 loopback off
sudo ip link set can0 up
```

> **总线上是否有节点（可选）**：在普通模式下，向一个电机不处理的 ID（例如 `0x7FF`）发送一帧。如果总线上有已上电的节点，它会 ACK 这一帧，`ip -details link show can0` 保持 `ERROR-ACTIVE` 且 `tx_errors=0`。若总线上没有其他节点，控制器重传后会进入 `ERROR-PASSIVE`/`BUS-OFF`。这可以区分"机械臂没上电／没接上"和"机械臂不应答"。

## 第 6 步：配置持久化（推荐）

除非有程序去配置，否则每次重启后 `can0` 都会回到 DOWN。用 oneshot 的 systemd 单元最通用——不依赖 `ifupdown`、`dhcpcd` 或 NetworkManager：

```bash
sudo tee /etc/systemd/system/can0-setup.service >/dev/null <<'EOF'
[Unit]
Description=Configure can0 (MCP2515 CAN HAT) at 1 Mbit/s
After=sys-subsystem-net-devices-can0.device
Wants=sys-subsystem-net-devices-can0.device

[Service]
Type=oneshot
RemainAfterExit=yes
# mcp251x loads late on slow boards; wait for the netdev instead of racing it
ExecStartPre=/bin/sh -c "for i in $(seq 1 150); do [ -e /sys/class/net/can0 ] && exit 0; sleep 0.2; done; exit 1"
ExecStart=/sbin/ip link set can0 down
ExecStart=/sbin/ip link set can0 type can bitrate 1000000 restart-ms 100
ExecStart=/sbin/ip link set can0 up
ExecStart=/sbin/ip link set can0 txqueuelen 65536
ExecStartPost=/bin/sh -c "/sbin/ip -details link show can0 | /bin/grep -q 'state UP'"

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable --now can0-setup.service
```

要用真实重启来验证，而不是只运行 `systemctl start`：

```bash
sudo systemctl reboot
# 重启后
systemctl is-active can0-setup.service     # -> active
ip -details link show can0 | grep -E "can state|bitrate"
```

> **时序说明**：在 Pi Zero W 上，`mcp251x` 驱动大约在开机 45–50 秒时才创建 `can0`。如果没有 `ExecStartPre` 的等待（或 `.device` 依赖），单元会先启动，报 `Cannot find device "can0"` 失败，并且悄悄让总线保持 down 状态。

## 故障排查

| 现象 | 可能原因 | 处理 |
|---|---|---|
| `dmesg` 中没有 `mcp251x` 相关行 | SPI 未启用、overlay 缺失、HAT 没插好 | 重新检查第 2 步，`dtparam=spi=on` 必须取消注释 |
| `Cannot initialize MCP2515. Wrong wiring?` | HAT 没插到底／SPI 被禁用 | 重新插紧 HAT 后重启 |
| `can0` 存在但电机始终不应答 | `oscillator=` 写错 → 实际波特率不对 | 确认晶振（第 1 步）并重新写 overlay |
| `up` 之后立刻 `bus-off` | 总线上没有其他节点，或 H/L 接反 | 检查机械臂供电，对调 H/L |
| `rx_over_errors` 持续增长 | SPI 速率跟不上流量 | 提高 `spimaxfrequency`（例如 10000000） |
| `ip link set can0` 当时可用，重启后又 down | 持久化单元缺失或失败 | 见第 6 步，`journalctl -u can0-setup.service -b` |
| `RTNETLINK answers: Operation not permitted` | 缺少 `sudo` | 命令前加 `sudo` |
