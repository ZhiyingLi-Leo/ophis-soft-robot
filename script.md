# OPHIS LAB · 启动与操作指南

## 1. 启动工作台

使用 D405 时，双击 `control code/local_dashboard/start_d405.command`。它会复用或启动相机桥接和工作台，再打开相机页。[D405 接入说明](control%20code/local_dashboard/CAMERA.md#macos-usb-采集会话)

在 macOS 终端中逐行执行：

```sh
cd "/实际路径/ophis-soft-robot"
"control code/local_dashboard/.venv/bin/python" "control code/local_dashboard/server.py"
```

在 Chrome 打开 [http://127.0.0.1:8765/](http://127.0.0.1:8765/)，保持服务终端运行。将目录替换为本机项目路径（现有工作区目录名为 `00_system_build`），路径用双引号包住；两条命令之间直接换行，不在行尾添加 `\`。

项目已安装独立 Python 和 RealSense SDK，启动时使用上面的 `.venv/bin/python`，无需激活 Conda base。启动不连接硬件，也不执行自动回零。

网页已经能打开时，无需重复启动服务。确需使用其他端口时：

```sh
"control code/local_dashboard/.venv/bin/python" "control code/local_dashboard/server.py" --port 8766
```

此时访问 `http://127.0.0.1:8766/`。不要让两个服务接入同一串口设备。结束操作时先点击网页“全部停止”，再在服务终端按 Ctrl-C。

## 2. 接入设备

1. 接好需要使用的 USB 设备，在侧栏“串口设备”旁点击扫描按钮。
2. 进入“液压驱动”或“滚轮推进器”，点击对应设备的接入按钮。设备可独立接入，无需接齐其他模块。
3. 自动识别只有一个匹配设备时直接使用；存在多个匹配串口时，在设备列表选择实际端口。
4. 接入后核对 ID、位置与反馈状态，再进行启用和运动操作。

扫描只查询状态，不发运动指令；打开 Nano 串口可能触发开发板复位。已启用运动时禁止扫描，应先停止。USB 更换接口后重新扫描即可，不必把串口名称写死。

## 3. 电缸控制

工作站视图从上到下对应 ID3、ID2、ID1。

- **单轴**：接入 → 启用 → 输入目标毫米位置 → 移动。目标为绝对位置；±0.1 mm 按钮用于相对点动。
- **三轴**：三个电缸分别接入后，点击“一键启用三轴”，填写三个目标，再点击“执行三轴目标”。串口按顺序发送，不是硬件同步运动。
- **停止**：单轴停止会撤销该轴启用；再次运动前需重新启用。Esc 或顶部“全部停止”作用于全部设备。

配置文件为 `control code/system_control/cylinders.conf`：三个电缸均为 16 mm 行程、0–16 mm 目标限位、0.5 mm/s 速度上限。修改配置后需停止设备并重启服务。导管连接后的实验范围应单独确认，不默认允许走完整行程。

网页不自动回零。本机已有配置中的 `auto_zero=true` 仍适用于命令行运动模式；仓库模板默认关闭回零，不能把网页的启动行为等同于命令行行为。

## 4. 滚轮推进器

进入“滚轮推进器”并接入设备：

- ESP32-S3 PCB：每张 M1–M4 卡片可单独设速度、前进、后退和停止。
- “四电机统一速度”与一起前进/后退按钮用于四路共同控制。
- Nano：仅支持统一控制，不提供四电机独立速度或四路遥测。

网页速度范围为 0–1600 step/s。前进或后退会持续运行，点击停止结束。正负方向和 M1–M4 实物对应关系需现场核对。电流显示为板端设定值，非实测值。

## 5. 蓝牙手柄

1. 在 macOS 蓝牙设置中配对手柄。
2. 回到 Chrome 工作台，按一下手柄按钮，让浏览器识别标准映射手柄。
3. 接入需要控制的设备，将两个摇杆回中，点击手柄栏“连接”。该操作停止原有运动，并启用当前在线且正常的电缸。
4. 点击“退出”结束手柄控制并停止本次会话的设备。

| 摇杆 | 正向 | 反向 |
|---|---|---|
| 左摇杆左右 | 右：推进器前进 | 左：推进器后退 |
| 左摇杆上下 | 上：ID1 伸出 | 下：ID1 回退 |
| 右摇杆左右 | 右：ID2 伸出 | 左：ID2 回退 |
| 右摇杆上下 | 上：ID3 伸出 | 下：ID3 回退 |

未接入设备忽略输入，回中停止对应通道。电缸使用配置限位和速度上限；推进器最大速度取推进器页面的“四电机统一速度”，默认 400 step/s。

B / ○、Esc 或“全部停止”可停止。手柄断开或输入超时后需回中重新连接；新增设备也需重新连接手柄控制才能纳入。切换窗口不主动停机，Chrome 若暂停后台采样，0.6 秒输入超时保护仍会停止手柄控制。

## 6. D405 预览与记录

D405 可独立使用，不需要连接电缸或推进器。插入相机后进入“深度视觉”，点击“接入 D405”；多台相机时填写序列号。

将白色透明柔性段置于绿色背景前，保持悬空，橙色近端纳入画面。通过 ROI、最低亮度和最高饱和度调整轮廓识别，观察整段路径、根部与尖端。当前三维结果为未标定的可见表面角，不直接作为真实弯曲角控制依据。

填写实验名称，点击“开始记录”，结束时点击“结束记录”。记录保存到 `control code/local_dashboard/data/`，包含原始颜色、深度、时间戳、识别结果及电缸反馈；以界面显示的实际目录为准。

在项目根目录的终端检查 SDK 与只读设备枚举：

```sh
"control code/local_dashboard/.venv/bin/python" "control code/local_dashboard/scripts/camera_check.py" --probe
```

未插相机时返回 0 台设备正常。完整环境与测量说明见 [CAMERA.md](control%20code/local_dashboard/CAMERA.md)。

## 7. 网页实验控制台

这是运行项目 Python 脚本的入口，不是完整终端。支持：

| 指令 | 用途 |
|---|---|
| `help` | 查看帮助 |
| `scripts` | 列出脚本 |
| `clear` | 清空输出 |
| `stop` | 中止实验并请求停止设备 |
| `python scripts/name.py [参数]` | 运行指定脚本 |

以下输入到**网页控制台**：

```text
python scripts/status.py
python scripts/camera_check.py --probe
python scripts/camera_record.py --seconds 30 --label reference
python scripts/sample_positions.py --help
```

相机记录命令要求先在页面接入相机。所有脚本都使用服务的 Python 解释器，以 `local_dashboard/` 为工作目录；不能在网页控制台使用 `cd`、`pip` 或 shell 管道。

脚本启动和结束都会请求停止设备，即使脚本只是读取状态。运行期间手动运动受限，手柄会退出；不要在需要保持运动时启动检查脚本。脚本是有本机权限的 Python 程序，只运行可信代码，通过 `experiment_api.Robot` 控制已接入设备，不自行打开串口。

位置采样的 CSV 列为 `q1_mm,q2_mm,q3_mm,hold_s`，需指定三轴各自上下限和总位移上限。`sample_positions.py` 默认仅校验计划；只有显式添加 `--execute` 才会启用并移动电缸。具体参数见 `--help` 和 [采集说明](control%20code/local_dashboard/CAMERA.md)，实验限位需现场确定。

导出数据可在**本机终端、项目根目录**执行，将目录名替换为实际记录目录：

```sh
"control code/local_dashboard/.venv/bin/python" "control code/local_dashboard/scripts/export_session.py" --session "control code/local_dashboard/data/实际记录目录"
```

该命令读取记录文件并生成 `summary.csv`，不访问硬件。控制台输出仅为有限内存缓冲，不能代替实验数据文件。

## 8. 排查与只读诊断

| 现象 | 检查方法 |
|---|---|
| 网页打不开 | 查看服务终端是否运行，核对端口；本机代理应绕过 `127.0.0.1` 和 `localhost` |
| 地址已被占用 | 先访问已有服务，避免重复启动；确需另一实例时用 `--port` |
| 手柄一直等待 | 检查系统蓝牙状态，在 Chrome 按手柄按钮；仅接受浏览器标准映射 |
| 提示其他页面正在控制 | 关闭重复控制页面，等待原会话释放后接管 |
| 电缸未收到有效回复 | 停止并检查电源、USB 转接器、总线和 ID，重新扫描接入；不自动重试运动 |
| 相机依赖缺失 | 确认服务使用项目 `.venv/bin/python`，运行相机环境检查 |
| 提示控制心跳或手柄输入超时 | 恢复页面/输入更新，确认设备状态后主动重新启用 |

电缸命令行只读诊断前，先在网页断开对应设备，释放串口。在项目根目录执行：

```sh
make -C "control code/system_control"
"control code/system_control/build/robot_control" --mode inspect --config "control code/system_control/cylinders.conf"
```

`inspect` 不执行回零、不写运动寄存器。其他命令行模式见 [system_control 文档](control%20code/system_control/README.md)。

## 9. 软件测试与固件

在项目根目录执行以下无硬件测试：

```sh
PYTHONDONTWRITEBYTECODE=1 "control code/local_dashboard/.venv/bin/python" -m unittest discover -s "control code/local_dashboard" -p 'test_*.py' -q
make -C "control code/nano_firmware" test
make -C "control code/system_control" test
```

Nano 固件仅编译、不烧录：

```sh
(cd "control code/nano_firmware" && ~/.platformio/penv/bin/pio run --environment nano_168)
```

Nano 板型为 ATmega168。烧录会复位开发板，操作前按 [Nano 固件说明](control%20code/nano_firmware/README.md) 核对板型和实际端口。ESP32-S3 的编译与烧录见 [PCB 文档](control%20code/pcb_esp32s3/README.md) 和 [烧录说明](control%20code/pcb_esp32s3/releases/烧录说明.md)。两种固件不可互换。

Nano 没有板端通信超时停车，需独立停机手段。停止脉冲不等于断电；测试结束关闭驱动电源。装配与接线见 [推进器装配说明](docs/推进器装配与接线.md)。
