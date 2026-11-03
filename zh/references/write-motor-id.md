## 写入电机 ID

> **注意：当前参考文档为测试版本，仅供参考。如有任何疑问或遇到问题，请联系 Seeed 获取支持。**
> **提醒：执行本流程前请确保电机已正确连接且上电，操作过程中注意用电安全。**

## 使用前提

已完成环境初始化步骤或 motorbridge 已安装。

执行前先验证 motorbridge-cli 可用：

```bash
motorbridge-cli --help 2>/dev/null || echo "motorbridge-cli not found"
```

如果未找到命令：
- Linux/macOS：确认 conda 环境已激活（`conda activate rebot`）
- Windows：使用完整路径，如 `<miniforge_path>\envs\rebot\Scripts\motorbridge-cli.exe`

> **重要：扫描电机时电机必须上电（RS 为 48V，DM 为 24V），否则不会有任何响应。** 扫描失败时首先提示用户确认是否已上电。

## 执行前信息收集

在开始扫描之前，**必须先向用户确认以下信息**：

| 信息 | 说明 | 示例 |
| --- | --- | --- |
| 电机类型 | robstride（灵足）或 damiao（达妙） | robstride |
| 设备版本 | 散件版（需逐个写 ID）或套件版（可能已有 ID） | 散件版 |
| 目标 ID 范围 | 用户想把电机设成哪些 ID | 1~7 |
| 硬件连接 | PCAN-USB/串口是否插入、CAN 线是否连接、电机是否上电 | 已连接已上电 |
| 当前连接数 | 修改 ID 时只能连接一颗电机 | 仅 1 颗 |

> **写入 memory**：收集完上述信息后，追加到 `../memory/local-machine-env.md`（如文件中尚无电机相关信息）：
>
> ```markdown
> ## 电机配置
> - 类型：robstride / damiao
> - 设备版本：散件版 / 套件版
> - 目标 ID 范围：1~7
> - 串口（damiao）：COMx（如适用）
> ```

## 确认设备类型

开始前先确认电机使用的协议类型：

| 类型 | 连接方式 | 说明 |
|------|----------|------|
| robstride（灵足） | USB-CAN（PCAN-USB / CANable） | Linux 通过 socketcan `can0` 通信；Windows 驱动自动识别 |
| robstride（灵足） | CAN HAT（SPI 接口 MCP2515） | 树莓派：先用 [CAN HAT 配置](setup-can-hat.md) 配置一次，之后 `can0` 以 1 Mbit/s 工作 |
| damiao（达妙） | USB 串口 | Linux 通过 `/dev/ttyACM0` 通信；Windows 通过 `COMx` |

> **远程主机**：如果这些电机接在你通过 SSH 访问的板子上，请给每条命令加上 `ssh <用户名>@<主机> "…"` 前缀。板子上有 conda 环境时，把激活也串进同一条命令（`"source ~/miniforge3/etc/profile.d/conda.sh && conda activate rebot && motorbridge-cli ..."`）。

## CLI 命令路径约定

下文中所有命令使用 `motorbridge-cli` 作为命令名。实际操作时：

- **Linux/macOS**（已激活 conda 环境）：直接运行 `motorbridge-cli`

- **Windows**（conda 环境未加入 PATH）：使用完整路径，例如：

  ```
  <miniforge_path>\envs\rebot\Scripts\motorbridge-cli.exe
  ```

  或在 conda 环境中先执行 `conda activate rebot` 后直接运行 `motorbridge-cli`

---

## 达妙 vs 灵足 ID 写入区别

| | 达妙（Damiao） | 灵足（RobStride） |
|---|---|---|
| 设备 ID 寄存器 | `ESC_ID`（rid=8） | `device_id` |
| 反馈 ID 寄存器 | `MST_ID`（rid=7） | `host_id`（不修改） |
| 默认设备 ID | `0x01` | `127`（0x7F），但部分批次可能已预设为 1~7 |
| 默认反馈 ID | `0x11`（= motor_id + 0x10） | `0xFD`（固定值） |
| 改 ID 时是否改反馈 ID | **是**，`feedback_id = motor_id + 0x10` | **否**，仅改 device_id |
| 参数 | `--motor-id` / `--new-motor-id` + `--feedback-id` / `--new-feedback-id` | `--motor-id` / `--new-motor-id` |

> 达妙电机的 `feedback_id` 与 `motor_id` 存在固定偏移：`feedback_id = motor_id + 0x10`。
> 例如 motor_id=0x01 → feedback_id=0x11，motor_id=0x05 → feedback_id=0x15。

---

## 第一步：设备初始化

根据操作系统和设备类型完成初始化。

### Linux — robstride（USB-CAN）

```bash
# 加载 PCAN-USB 驱动（CANable/candleLight 用 gs_usb，跳过这步）
sudo modprobe peak_usb
ip -br link

# 配置 can0
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up

# 确认状态
ip -br link show can0
```

### Linux — robstride（CAN HAT，SPI 接口 MCP2515）

HAT 不使用 `peak_usb`；只有启用 SPI overlay 并重启之后 `can0` 才会出现。先按 [CAN HAT 配置](setup-can-hat.md) 做一次，然后在扫描前确认接口状态：

```bash
ip -details link show can0 | grep -E "can state|bitrate"
# can state ERROR-ACTIVE / bitrate 1000000
```

### Windows — robstride（USB-CAN）

PCAN-USB 驱动安装后自动可用，**不需要配置 can0**。确认 PCAN-USB 已插入即可。

### Linux — damiao（USB 串口）

1. 确认串口设备存在：`ls -l /dev/ttyACM0`
2. 确认用户在 `dialout` 组：`groups`
3. 如不在组中：`sudo usermod -aG dialout $USER`（需重新登录生效）

### Windows — damiao（USB 串口）

1. 在设备管理器中确认 COM 端口号
2. 后续命令中 `--serial-port` 参数使用 `COMx`（如 `COM3`）

---

## 第二步：检查现有电机

根据设备类型执行对应命令：

**robstride（Linux）：**
```bash
motorbridge-cli scan --vendor robstride --channel can0 --start-id 1 --end-id 8
```

**robstride（Windows）：** 命令格式与 Linux 相同（仍需 `--channel can0`，PCAN 驱动在底层自动映射）：
```bash
motorbridge-cli scan --vendor robstride --channel can0 --start-id 1 --end-id 8
```

**damiao（Linux）：**
```bash
motorbridge-cli scan \
    --vendor damiao \
    --transport dm-serial \
    --serial-port /dev/ttyACM0 \
    --serial-baud 921600 \
    --start-id 1 \
    --end-id 32
```

**damiao（Windows）：** 将 `--serial-port /dev/ttyACM0` 替换为 `--serial-port COMx`

判断逻辑：
- **robstride** 如果命中 1～7 号电机，提示用户当前机械臂可能为成品版本，不需要修改 ID，本流程无需执行
- **damiao** 根据命中的 ID 判断是否需要修改

---

## 第三步：修改电机 ID

### 连接新电机

1. 要求用户**断电后**拔掉当前电机上的线
2. 将线连接到下一颗电机
3. **开启电源**（RS 为 48V，DM 为 24V；电机必须上电才能通信）
4. 等待用户确认上电后，执行**查找当前电机 ID**

### 查找当前电机 ID

执行前要求用户**只能连接一颗电机**。

> **扫描策略：同时扫描出厂默认范围和 1~7 范围。** 部分批次电机出厂时 ID 可能已预设为 1~7，而非默认值 127。

**robstride（Linux / Windows 命令相同）：**

```bash
# 同时扫描两个范围，根据结果判断电机状态
motorbridge-cli scan --vendor robstride --channel can0 --start-id 1 --end-id 7 --timeout-ms 300
motorbridge-cli scan --vendor robstride --channel can0 --start-id 126 --end-id 127 --timeout-ms 300
```

> Windows 下 `--channel can0` 仍然需要，PCAN 驱动在底层自动映射，不需要提前配置 can0 接口。

**damiao：**
```bash
motorbridge-cli scan \
    --vendor damiao \
    --transport dm-serial \
    --serial-port /dev/ttyACM0 \
    --serial-baud 921600 \
    --start-id 1 \
    --end-id 1
```

- **robstride** 命中 **127**：说明电机未设置过 ID，进入**修改当前电机 ID**步骤
- **robstride** 命中 **1~7 中某个值**：说明已预设 ID。如果该值正好是目标 ID，这颗电机已完成；否则询问用户是否需要重新修改
- **damiao** 命中 **0x01**：说明电机未设置过 ID，进入**修改当前电机 ID**步骤
- **未命中默认 ID**：扩大扫描范围（1~32 或 1~50），若命中某个 ID，提示用户并询问是否需要重新修改
- **未命中任何电机**：提示用户检查：① 电源是否已开启（RS 48V / DM 24V）② CAN 线/串口是否连接牢固 ③ 电机是否故障

### 修改当前电机 ID

确认电机当前 ID 后，执行修改命令：

**robstride（Linux / Windows 命令相同）：**
```bash
# 示例：将 ID 127 改为 5
motorbridge-cli id-set --vendor robstride --channel can0 --motor-id 127 --new-motor-id 5
```

**damiao：**
```bash
# 示例：将 ESC_ID 0x05 改为 1，MST_ID 自动从 0x15 改为 0x11
motorbridge-cli id-set \
    --vendor damiao \
    --transport dm-serial \
    --serial-port /dev/ttyACM0 \
    --serial-baud 921600 \
    --motor-id 5 \
    --feedback-id 0x15 \
    --new-motor-id 1 \
    --new-feedback-id 0x11
```

> **robstride id-set 超时说明**：执行 `id-set` 后可能出现 `store_parameters failed: response ack timeout` 错误（exit code 2）。这是因为电机写入后重启导致确认超时，**ID 实际已写入成功**。出现此错误后不要重试，直接执行**扫描确认**验证 ID 是否已变更。

写入成功后，达妙电机会输出类似以下内容：

```
write rid=7 (MST_ID) <= 0x11
write rid=8 (ESC_ID) <= 0x1
store_parameters sent
verify rid=8 (ESC_ID): 0x1
verify rid=7 (MST_ID): 0x11
verify ok
```

> 达妙电机修改 ID 后会重启，命令末尾可能出现 `get_register_u32 failed` 超时提示。这表示验证失败，需要立即执行**扫描**确认 ID 是否实际写入成功，如未成功则重新修改。

### 扫描确认

修改完成后立即扫描确认：

```bash
# robstride：扫描旧 ID 和新 ID
motorbridge-cli scan --vendor robstride --channel can0 --start-id <新ID> --end-id <新ID> --timeout-ms 300

# damiao：同上，加上传输参数
```

- 如果新 ID 命中、旧 ID 无响应 → 修改成功
- 如果旧 ID 仍然命中 → 修改失败，重新执行 `id-set`

> **写入 memory**：每颗电机 ID 设置成功后，追加到 `../memory/local-machine-env.md`：
>
> ```markdown
> ### 已设置的电机
> - ID 1：已确认 ✅
> ```

---

## 第四步：最后确认

所有 7 颗电机设置完成后，将所有电机同时连接到 CAN 总线/串口，执行最终扫描：

**robstride（Linux / Windows 命令相同）：**
```bash
motorbridge-cli scan --vendor robstride --channel can0 --start-id 1 --end-id 7 --timeout-ms 500
```

**damiao：**
```bash
motorbridge-cli scan \
    --vendor damiao \
    --transport dm-serial \
    --serial-port /dev/ttyACM0 \
    --serial-baud 921600 \
    --start-id 1 \
    --end-id 32
```

扫描成功后 CLI 会输出类似以下内容：

```
[hit] probe=0x01 vendor=robstride via=ping feedback_id=0xFD device_id=1 responder_id=254
[hit] probe=0x02 vendor=robstride via=ping feedback_id=0xFD device_id=2 responder_id=254
...
scan done: 7 motor(s) found
```

告知用户命中了哪些电机。用户确认 ID 1~7 全部正确后，本流程执行完成；如有缺失则返回对应电机重新修改。

> **写入 memory**：最终扫描完成后，更新 `../memory/local-machine-env.md`：
>
> ```markdown
> ### 电机 ID 设置完成
> - 已设置 ID：1, 2, 3, 4, 5, 6, 7
> - 最终扫描时间：YYYY-MM-DD
> ```
