# D405 采集与实验脚本

## 当前功能

工作台的“深度视觉”页面支持独立接入 D405、预览轮廓识别结果、设置 ROI 与白色阈值、开始和结束文件记录。三个电缸或推进器未接入时，相机仍可独立使用。

测量对象为长 30 mm、外径 6 mm 的白色透明柔性段，橙色近端，悬空于绿色背景上方。当前算法用颜色分割、细化、根部追踪和等距离重采样提取 21 点图像路径。实际表面透明程度、边界和阴影需要在相机到位后调整阈值。

界面分别显示二维投影角和三维可见表面角。三维点来自对齐深度反投影，坐标单位为 mm，原点在相机。基座配准、截面半径补偿、真实三维中心线精度和控制模型尚未标定；三维表面角不作为角度控制真值。

## 环境与启动

从项目根目录执行：

```sh
"control code/local_dashboard/.venv/bin/python" "control code/local_dashboard/server.py"
```

访问 http://127.0.0.1:8765/ 。服务已运行时不要再启动第二份。相机模块延迟加载 SDK，依赖缺失不影响原有电缸及推进器功能。

独立环境中的图像处理依赖：

```sh
"control code/local_dashboard/.venv/bin/python" -m pip install -r "control code/local_dashboard/requirements-camera.txt"
"control code/local_dashboard/.venv/bin/python" "control code/local_dashboard/scripts/camera_check.py"
```

### 本机环境

工作台使用项目内 `.venv/` 独立 Conda 环境，原生 Apple Silicon arm64：

| 组件 | 已安装版本 |
|---|---|
| Python | 3.11.18 |
| RealSense SDK / pyrealsense2 | 2.58.3 |
| libusb | 1.0.30 |
| NumPy | 2.4.6 |
| OpenCV | 4.14.0 |

RealSense 和 libusb 来自 [conda-forge 原生包](https://anaconda.org/conda-forge/pyrealsense2/files)，OpenCV 来自 PyPI。环境定义保存在 `environment-camera.yml`。重建新环境时，在项目根目录运行：

```sh
conda env create --prefix "control code/local_dashboard/.venv" --file "control code/local_dashboard/environment-camera.yml"
```

当前环境已安装，不需要重复创建。服务和实验脚本均使用 `.venv/bin/python`，无需修改系统 Python 或激活 base 环境。

2026-11-17 验证结果：依赖导入、依赖一致性检查、SDK 投影/反投影、JPEG 编解码及 39 项工作台测试通过。普通用户权限下 SDK 上下文初始化与相机枚举正常，当前相机数量为 0。

只读环境检查：

```sh
"control code/local_dashboard/.venv/bin/python" "control code/local_dashboard/scripts/camera_check.py" --probe
```

检查脚本显示版本、平台、各依赖状态和相机列表。未连接相机但枚举正常时退出码为 0；依赖缺失或无法导入时为 2；设备枚举异常时为 3。它不打开电缸串口、不启动相机图像流。

D405 实际插入后的 USB 访问、640×480 图像流及白色透明导管的测量质量需要现场验证。macOS 的 USB 权限处理依实际设备结果确定；不以 root 运行整个网页服务及实验脚本。官方说明见 [macOS SDK 文档](https://github.com/realsenseai/librealsense/blob/master/doc/installation_osx.md)。

## 明天下午的接入顺序

1. 完成 SDK 导入检查。在系统终端运行 `camera_check.py --probe`，确认枚举到 D405；该选项只读枚举，不启动图像流。
2. 启动工作台，进入“深度视觉”，点击“接入 D405”。单台自动匹配，多台时输入序列号。只匹配 D405，不自动替换为其他型号。
3. 当前采集请求为深度 Z16 与颜色 BGR8、640×480、30 fps。SDK 不支持该组合或 USB 访问失败时，页面显示具体错误，不影响电缸模块。
4. 调整画面，使橙白交界和整个柔性段处于 ROI 内。结合原图检查紫色路径、蓝色根部与红色尖端是否对应正确。
5. 先采集静止数据，再检查背景改变时表面深度是否跟随背景。有效深度比例高不等于深度准确。
6. 完成安全位移范围与测量噪声确认后，再执行带运动的位置表。

## 控制台指令

以下短命令输入网页的“实验控制台”；系统终端执行时使用工作台 `.venv/bin/python` 和相应完整路径。

```sh
python scripts/camera_check.py
python scripts/camera_record.py --seconds 30 --label reference
```

`camera_record.py` 使用已接入相机，检查新鲜图像并记录指定时间，不启用或移动电缸。运行任何工作台实验脚本前，现有实验运行器会停止当前运动；脚本结束也请求停止设备。要在手动操作期间持续录制，使用相机页面的“开始记录”和“结束记录”。

记录目标为 5 帧/秒，预览与处理经过节流。设备请求帧率、实际处理数量、原始帧号和每帧时间分别保留；不宣称保存了所有 30 fps 帧。

每次记录在 `local_dashboard/data/` 下新建目录：

```text
metadata.json          装置尺寸、ROI、识别参数、相机内外参、深度尺度和固件信息
frame_000000.npz        color_bgr、native_depth、aligned_depth
frames.jsonl           每帧时间、形状估计、电缸反馈、样本注记及结束原因
summary.csv            由导出脚本生成的逐帧汇总
```

原生与对齐深度均为原始 uint16 数值，换算 mm 使用 `metadata.calibration.depth_scale_m × 1000`。不做深度填洞。每轴 `last_seen` 是实际反馈时间；串口锁忙时记录上一份完成的反馈快照并标注 `snapshot_reused`。主机接收时间与设备曝光时间分别保存，尚未补偿两者的偏移。

导出记录：

```sh
python scripts/export_session.py --session data/实际实验目录
```

该脚本只读原始帧索引，并写入该目录的 `summary.csv`；再次执行会更新这个派生文件，原始数据不变。

## 位置表采样

CSV 文件由操作者填写，表头为：

```csv
q1_mm,q2_mm,q3_mm,hold_s
```

每行填写三个绝对目标位置和稳态保持秒数。列名对应实际硬件 ID，不对应工作站的上下排列；模型与工作站从上到下是 ID 3、2、1。

查看参数：

```sh
python scripts/sample_positions.py --help
```

运行时必须提供 `--plan`、`--limits LO1 HI1 LO2 HI2 LO3 HI3` 和 `--max-total-mm`。这些值由当前装配的安全实验范围确定，不能直接照搬 0–16 mm。`max-total-mm` 是三轴绝对位置之和上限；不同注射器面积下，它不能替代液压体积或压力约束。

默认只校验 CSV；只有显式加入 `--execute` 才开始真实采样。执行时要求相机和三轴已接入、反馈有效、起点在实验限制内；脚本会启用三轴。

- 目标拆为小步，默认每轴单步不超过 0.1 mm。
- 减少位移的动作先执行，逐轴等待到位后再执行下一轴；不宣称三轴硬件同步。
- 同时检查实际电缸位置、视觉表面角和整段表面曲线变化。稳定窗口、角度容差、曲线位移容差需要按实测噪声调整。
- 相机失联、轮廓或完整深度曲线缺失、记录失败、电缸故障或等待超时会终止脚本并请求停止。
- 普通相机页面的掉线不主动停止独立手动电缸；要求相机参与的采样脚本会检查依赖并停止。
- 记录每个目标与稳态片段的起止注记，保留移动过程。脚本结束不自动回零。

该脚本用于受控小范围的数据采样。尚未实现根据目标三维角度自动求解位置；需要完成基座配准、视觉精度验证和位置—形状模型后加入。

## 设备插件接口

`device_plugins/` 注册以下设备对象：

| 名称 | 操作 |
|---|---|
| `cylinder.1`、`cylinder.2`、`cylinder.3` | connect、disconnect、arm、move、jog、stop |
| `propulsion` | connect、disconnect、motor、all、stop；motor 参数为 1–4 |
| `camera.d405` | connect、disconnect、configure、record_start、record_stop、annotate |

设备统一提供 `snapshot()` 和 `command()`，状态通过 `/api/state` 的 `modules` 字段返回。新设备在注册处加入实例，即可复用命令入口；页面仍按设备功能提供专用工作区。

实验脚本使用：

```python
from experiment_api import Robot

with Robot() as robot:
    print(robot.state()['modules'])
    robot.record_start('reference')
    robot.annotate(kind='reference_start')
    robot.wait(2)
    robot.record_stop()
```

单轴指令使用 `robot.device('cylinder.2', 'move', mm=目标位置)`，仍需先启用并满足限位。脚本禁止调用 connect、disconnect、configure；连接与参数调整由操作者在页面完成。

相机驱动拥有独立采集线程。电缸与推进器插件目前是现有控制引擎的适配层，串口驱动、共享总线调度和安全保护仍由 `server.py` 管理；不是每个电缸各开一个串口，也不是独立进程沙箱。四路推进电机有独立操作，但共享一块控制器及其故障状态。

## 验证范围

自动测试覆盖插件独立接入、SDK 缺失、识别参数校验、直管与背景识别、深度缺失、原始帧保存、磁盘失败、位置表范围校验以及原有通信和停止保护。输入为合成图像、临时文件和伪串口。D405 真实 USB 采集、白色透明导管识别、精度与负载下实时性等待实机验证。
