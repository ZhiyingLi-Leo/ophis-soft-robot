# OPHIS · Soft Robot Workstation

三通道液压软体导管的本地控制与视觉标定工作台。支持电缸独立位移控制、四电机滚轮推进、蓝牙手柄、RealSense D405 采集和 Python 实验脚本联调。

## 快速开始

本机已有环境时，在仓库根目录启动：

```sh
"control code/local_dashboard/.venv/bin/python" "control code/local_dashboard/server.py"
```

Chrome 打开 http://127.0.0.1:8765/ 。设备独立接入，服务启动不连接硬件、不自动回零。

新检出工作区先准备环境和本机配置：

```sh
conda env create --prefix "control code/local_dashboard/.venv" --file "control code/local_dashboard/environment-camera.yml"
cp -n "control code/system_control/cylinders.conf.example" "control code/system_control/cylinders.conf"
```

环境已在 Apple Silicon macOS 上验证。电缸配置模板对应 ID1/2/3、16 mm 行程、0.5 mm/s 速度上限；接真机前按装配和液压实验范围核对限位。`cp -n` 不覆盖已有配置。Windows/Linux 的工作台串口后端尚未适配验证。

## 使用与实验

- [功能、参数与目录](intro.md)
- [启动、设备接入、手柄、相机和脚本操作](script.md)
- [工作台与统一设备接口](control%20code/local_dashboard/README.md)
- [D405 环境与数据采集](control%20code/local_dashboard/CAMERA.md)
- [视觉标定与位置控制实验计划](docs/柔性导管视觉标定与位置控制实验计划.md)

电缸通信已有实机检查。相机采集、轮廓识别及角度估计需进行实物标定，当前三维表面角不是已验证的弯曲角真值。Nano 固件没有断线停车保证，实物需独立停机手段。

## 无硬件测试

```sh
PYTHONDONTWRITEBYTECODE=1 "control code/local_dashboard/.venv/bin/python" -m unittest discover -s "control code/local_dashboard" -p 'test_*.py' -q
make -C "control code/nano_firmware" test
make -C "control code/system_control" test
```

## 版本与数据

较大功能改动完成后，运行相关测试，检查 `git diff`，提交明确的变更说明，再推送到 GitHub。论文实验应记录对应 Git 提交；确定论文使用的版本后可增加标签。当前未设置定时或自动推送。

仓库保留源代码、配置模板、固件烧录包、模型和文档；不提交 Python 环境、构建缓存、本机电缸配置和原始采集数据。实验数据保存在 `control code/local_dashboard/data/`，需另行备份；本仓库不替代数据存储。

模型包含网页所需 GLB 和 STEP 源文件。第三方许可见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。项目尚未指定开源许可证；仓库公开可见不代表授予对项目自有代码、模型和数据的再分发许可。
