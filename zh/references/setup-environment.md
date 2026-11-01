## 环境初始化

> **注意：当前参考文档为测试版本，仅供参考。如有任何疑问或遇到问题，请联系 Seeed 获取支持。**

## 执行前必读

开始前，先检查 `../memory/local-machine-env.md` 是否已有当前机器的环境配置。如果已有 conda 路径和 motorbridge 信息，可直接使用；如果没有或信息不完整，按以下步骤重新初始化。

## 第零步：检查是否已安装

在开始之前，先检查 motorbridge 是否已安装：

```bash
# Linux / macOS / Git Bash：
motorbridge-cli --help 2>/dev/null || python -m motorbridge --help 2>/dev/null || echo "motorbridge not installed"

# Windows PowerShell（无 2>/dev/null）：
motorbridge-cli --help *>&1 | Out-Null; if ($LASTEXITCODE -ne 0) { python -m motorbridge *>&1 | Out-Null; if ($LASTEXITCODE -ne 0) { echo "motorbridge not installed" } }
```

- 如果输出帮助信息，说明 **motorbridge 已安装**，可跳过第一、二步，直接进入后续步骤
- 如果输出 `motorbridge not installed`，继续执行以下步骤

---

## 第一步：准备 Python 环境

### Linux / Jetson / 树莓派

> 以下命令在 **bash/zsh** 中执行。

安装 Miniforge：

```bash
wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
```

如果系统没有 `wget`，改用 `curl`：

```bash
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
```

创建并激活虚拟环境：

```bash
conda create -y -n rebot python=3.12
conda activate rebot
```

### macOS

> 以下命令在 **bash/zsh** 中执行。
>
> **注意**：macOS 下 `$(uname)` 返回 `Darwin`，但 Miniforge 安装包使用 `MacOSX`，因此 URL 需显式指定。

安装 Miniforge：

```bash
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-MacOSX-$(uname -m).sh"
bash Miniforge3-MacOSX-$(uname -m).sh
```

创建并激活虚拟环境：

```bash
conda create -y -n rebot python=3.12
conda activate rebot
```

### Windows

> 以下命令在 **PowerShell** 中执行。如果使用 Git Bash，可将 PowerShell 的 `Invoke-WebRequest` 替换为 `curl` 命令。

#### 使用 Miniforge（推荐）

> **注意：Windows 用户请自行下载 Miniforge，不要由 Agent 代为下载。** Agent 环境（Git Bash）使用 `curl` 下载 GitHub 文件速度极慢，即使使用代理也不理想。

1. **下载 Miniforge**（任选一种方式）：

   **方式一：PowerShell 下载（推荐）**

   ```powershell
   Invoke-WebRequest -Uri "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Windows-x86_64.exe" -OutFile "$env:USERPROFILE\Downloads\Miniforge3-Windows-x86_64.exe"
   ```

   **方式二：浏览器下载**

   在浏览器中打开 Miniforge 的 Release 页面，找到最新版本的 `Miniforge3-Windows-x86_64.exe` 点击下载：

   ```text
   https://github.com/conda-forge/miniforge/releases
   ```

   > 不要使用系统 Python（如 `C:\Python314\python`）直接 `pip install motorbridge`，可能导致安装失败。必须使用 Miniforge 创建的 conda 虚拟环境。

2. 双击运行安装器完成安装，然后打开 **Anaconda Prompt**（或 Miniforge Prompt），创建并激活虚拟环境：

> **写入 memory**：安装完成后，记录 Miniforge 实际安装路径到 `../memory/local-machine-env.md`：
>
> ```markdown
> ## conda
> - 安装路径：`F:\miniforge`（Git Bash：`/f/miniforge`）
> - conda.sh：`/f/miniforge/etc/profile.d/conda.sh`
> ```

   ```bash
   conda create -y -n rebot python=3.12 --no-shortcuts
   conda activate rebot
   ```

   > 如果 `conda activate` 报错 "CommandNotFoundError: Your shell has not been properly configured"，
   > 先执行 `conda init cmd.exe`（或 `conda init powershell`），重启终端后再 activate。

   **Git Bash 用户**：如果 `conda` 命令找不到，说明 Git Bash 未加载 conda 环境。需要先初始化：

   ```bash
   # 临时生效（当前终端），<安装路径> 替换为实际路径
   source <安装路径>/etc/profile.d/conda.sh

   # 永久生效（写入 bashrc，执行一次即可）
   echo 'source <安装路径>/etc/profile.d/conda.sh' >> ~/.bashrc
   source ~/.bashrc
   ```

   > 默认安装路径示例：`~/miniforge3`（即 `C:\Users\<用户名>\miniforge3`）。

   **PowerShell 用户**：如果 `conda` 命令找不到，需要先初始化 PowerShell：

   ```powershell
   # 初始化 PowerShell（执行一次即可）
   conda init powershell
   ```

   关闭并重新打开 PowerShell 后，`conda` 命令即可使用。或者使用 Miniforge 自带的 **Anaconda Prompt**（安装时自动创建），无需额外配置。

#### 备选方案：uv

> 仅在用户已安装 `uv` 且希望用它替代 Miniforge 时使用。本节要满足的是**环境隔离**，而非 conda 本身：uv venv 拥有独立的解释器和独立的 `Scripts\` 目录，不会污染系统 Python。

```powershell
uv venv .rebot-venv --python 3.12
uv pip install --python .rebot-venv\Scripts\python.exe motorbridge
.\.rebot-venv\Scripts\motorbridge-cli.exe --help
```

- 保持 Python 3.12 与本文档其余部分一致；`motorbridge` 提供 cp310–cp314 的 Windows wheel，其他版本通常也可用。
- 将其他参考文档中的 `conda activate rebot` 替换为 `.\.rebot-venv\Scripts\Activate.ps1`；也可以完全不激活，直接用完整路径调用 `.\.rebot-venv\Scripts\motorbridge-cli.exe` 与 `.\.rebot-venv\Scripts\motorbridge-gateway.exe`。若 `Activate.ps1` 被执行策略拦截，请使用完整路径方式。
- `uv tool install motorbridge` 是另一种方式：它会把 `motorbridge-cli` 和 `motorbridge-gateway` 加入 PATH（`%USERPROFILE%\.local\bin`），其他参考文档中的命令可直接执行。
- 如果 `uv` 无法写入默认缓存目录（受限或被沙箱限制的环境），请将 `$env:UV_CACHE_DIR` 与 `$env:UV_PYTHON_INSTALL_DIR` 指向可写目录。
- `uv` 不读取 `pip.ini`，因此第二步中的 `global.target` 检查不适用。

> **写入 memory**：记录 uv venv 路径，代替 conda 相关字段：
>
> ```markdown
> ## Python Environment (uv)
> - venv path: `D:\work\reBot-B601-Agent-Guide\.rebot-venv`
> - Activate: `.rebot-venv\Scripts\Activate.ps1`
> ```

---

## 第二步：安装 motorbridge

> **Windows 用户注意**：如果之前配置过 pip 的全局 `target` 路径，`pip install` 可能会装到错误的位置。
> 安装前先检查：
>
> ```bash
> python -m pip config list
> ```
>
> 如果输出包含 `global.target=...`，需要编辑 `C:\Users\<你的用户名>\AppData\Roaming\pip\pip.ini`，
> 在 `target` 行前加 `#` 注释掉，然后继续。

```bash
pip install --force-reinstall --no-cache-dir motorbridge
```

> 不锁定版本号，使用最新稳定版。安装完成后验证：

```bash
motorbridge-cli --help
```

如果提示命令未找到，检查 Python Scripts 目录是否在 PATH 中。Windows 用户使用完整路径执行。

> **写入 memory**：安装成功后，记录 motorbridge-cli 实际路径到 `../memory/local-machine-env.md`：
>
> ```markdown
> ## motorbridge
> - CLI：`/f/miniforge/envs/rebot/Scripts/motorbridge-cli`
> - 安装方式：conda rebot 环境中 pip install
> ```

---

## 第三步：平台特定配置

根据操作系统和设备类型，完成以下配置：

### Linux — robstride（USB-CAN）

```bash
# 加载 PCAN-USB 驱动
sudo modprobe peak_usb
ip -br link

# 配置 can0
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up

# 确认状态
ip -br link show can0
```

### macOS — robstride（USB-CAN）

macOS 使用 PCAN-USB **不需要配置 can0 接口**，但需要安装 PCBUSB 运行时库（`libPCBUSB.dylib`）。

1. 安装 PCBUSB 库：

```bash
curl -L -o macOS_Library_for_PCANUSB_v0.13.tar.gz \
  https://raw.githubusercontent.com/tianrking/motorbridge/main/third_party/pcan/macos/macOS_Library_for_PCANUSB_v0.13.tar.gz
tar -xzf macOS_Library_for_PCANUSB_v0.13.tar.gz
cd PCBUSB
sudo ./install.sh
```

2. 配置 `DYLD_LIBRARY_PATH`，确保 motorbridge-gateway 运行时能找到 PCBUSB 库。在 conda 环境中创建激活脚本，每次 `conda activate rebot` 自动生效：

```bash
mkdir -p "$CONDA_PREFIX/etc/conda/activate.d"
cat > "$CONDA_PREFIX/etc/conda/activate.d/env_vars.sh" << 'EOF'
export DYLD_LIBRARY_PATH="/usr/local/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
EOF
```

3. 验证安装是否就绪：

```bash
# 检查 Python 包和 CLI 是否就绪
python -c "import motorbridge; print('motorbridge OK')"
motorbridge-cli --help

# 检查 PCBUSB 运行时是否可加载
python -c "import ctypes; ctypes.CDLL('libPCBUSB.dylib'); print('PCBUSB load OK')"

# 确认环境变量已生效
echo $DYLD_LIBRARY_PATH
```

> **写入 memory**：安装完成后，更新 `../memory/local-machine-env.md`：
>
> ```markdown
> - PCBUSB 库：已安装（macOS）
> - DYLD_LIBRARY_PATH：通过 `$CONDA_PREFIX/etc/conda/activate.d/env_vars.sh` 自动设置 `/usr/local/lib`
> ```

### Windows — robstride（USB-CAN）

Windows 使用 PCAN-USB **不需要配置 can0 接口**。

1. 首次使用需前往 [PCAN-USB 官方页面](https://www.peak-system.com/products/hardware/external-pc-interfaces/pcan-usb/) 下载并安装驱动
2. 插入 PCAN-USB 转接板
3. 无需执行任何额外配置命令，直接使用 `motorbridge-cli` 即可

> **写入 memory**：PCAN-USB 驱动安装完成后，更新 `../memory/local-machine-env.md`：
>
> ```markdown
> - PCAN-USB 驱动：已安装
> ```

### Linux — damiao（USB 串口）

1. 确认串口设备存在：

```bash
ls -l /dev/ttyACM0
```

2. 检查权限。输出应为 `crw-rw----` 且组为 `dialout`，确认当前用户在该组：

```bash
groups
```

如果输出中不包含 `dialout`，执行：

```bash
sudo usermod -aG dialout $USER
```

> 加入组后需要**重新登录**（或重启）才生效。重新登录后再次确认 `groups` 输出包含 `dialout`，否则无法访问串口。

### Windows — damiao（USB 串口）

1. 插入达妙电机 USB 串口线
2. 在设备管理器中确认 COM 端口号
3. 使用 `--serial-port COMx` 指定对应端口（如 `COM3`）

> **写入 memory**：确认 COM 端口后，更新 `../memory/local-machine-env.md`：
>
> ```markdown
> - 达妙串口：COMx
> ```
