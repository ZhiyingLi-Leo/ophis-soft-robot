# 推进器集成PCB控制代码

适用硬件：PCB1 / Y2，ESP32-S3-WROOM-1-N8，四路DRV8434PWPR，四个28HS28电机。电缸与压力传感器不在此代码范围。

Nano 推进器固件及烧录配置位于相邻的 `nano_firmware/`。本目录的 ESP32-S3 固件与 Nano 固件分别维护，烧录时必须核对板型。

## 目录

| 路径 | 内容 |
|---|---|
| `firmware/` | ESP32-S3固件，USB CDC、BLE、四路独立STEP/DIR和电流PWM |
| `releases/` | 已编译的首次烧录整包及各分区二进制 |
| `host/` | Python跨平台USB/BLE、终端、按键松开即停的键盘/手柄控制、CSV状态采集 |
| `host_macos/` | 沿用GameController/IOKit读取方式的macOS原生上位机，适配新板USB |
| `ros2_ws/` | 独立ROS 2包 `propulsion_pcb_control`，USB串口；不与旧包重名 |
| `docs/` | 协议、引脚、首板调试和验证范围 |
| `tests/` | 控制核心、命令边界、引脚及串口伪终端联调测试 |

## 先运行不接硬件的示例

```sh
cd "control code/pcb_esp32s3"
python3 host/control.py --dry-run status
python3 host/control.py --dry-run jog --speed 400 --duration 0.5
make -C host_macos demo
```

`--dry-run`只演示上位机指令，不仿真电机、电流、USB供电或BLE无线性能。

## 编译与烧录

安装PlatformIO后：

```sh
cd firmware
pio run
# 仅在样板焊接和供电检查完成后执行：
pio run -t upload --upload-port /dev/cu.usbmodem实际端口
```

已编译文件位于 `releases/`。`propulsion_pcb_full.bin`是从地址0x0写入的整包；`firmware.bin`只是应用程序，不能在首片空白芯片上单独替代整包。优先使用上面的PlatformIO上传命令。USB是ESP32-S3原生CDC，不再是CH340的 `cu.usbserial`。

首片/恢复下载：只接合适的USB-C数据线即可给主控供电；按住BOOT，按一下RESET，松开BOOT，再选枚举端口。正常运行后可再次经同一Type-C烧录。烧录时无需接24V；电机运行仍需外部24V及电源开关接通。供电能力、枚举和自动复位须首板实测。

## 上位机使用

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r host/requirements.txt
# 只查询，不解锁、不转电机：
python host/control.py --port /dev/cu.usbmodem实际端口 status
# Windows用COMx；Linux通常为/dev/ttyACM0。不要照抄端口占位符。
python host/control.py --port /dev/cu.usbmodem实际端口 terminal
```

终端中先输入 `A1`解锁，再输入 `V400`同步运行，`S`停止并保持电流，`A0`释放驱动，`quit`退出。主控启动不自动解锁。SDK/上位机每200ms发送心跳；用普通串口终端时须自行每秒内持续发送 `H`或有效速度指令，否则停车并要求重新 `A1`。

首件单轴短时点动：断电接好全部四个电机，均脱离导管负载，再上电；不能让其他三路电机悬空，以免触发驱动开路故障。方向确认后再联动：

```sh
python host/control.py --port /dev/cu.usbmodem实际端口 jog --motor 1 --speed 400 --duration 0.5
python host/control.py --port /dev/cu.usbmodem实际端口 jog --speed 400 --duration 0.5
```

点动命令会解锁，退出时停止并释放驱动；它与持续终端里的 `S`保持电流有区别。

键盘/手柄与记录：

```sh
python host/control.py --port /dev/cu.usbmodem实际端口 --log run.csv keyboard
python host/control.py --port /dev/cu.usbmodem实际端口 --log run.csv gamepad --axis-index 3 --stop-button 6
python host/control.py --scan
python host/control.py --ble status
python host/control.py --ble --log ble_run.csv gamepad --axis-index 3 --stop-button 6
```

Python键盘界面需保持焦点：W/上箭头前进，S/下箭头回退，松键停止，空格停止，Esc退出；丢失窗口焦点立即停止。默认速度上限3200步/秒，允许显式调到16000步/秒，默认加速度3200步/秒²。手柄先dry-run核对轴及急停键；Windows/Linux/macOS可能映射不同，反向可加 `--axis-sign -1`。连接与映射故障不自动恢复运动。

macOS原生手柄程序保留原读取方式：

```sh
make -C host_macos
host_macos/bin/pcb_mac_control --mode gamepad --serial /dev/cu.usbmodem实际端口 --diagnostic
```

原生程序的 `--mode simulate`保留原键盘逐档调速方式：a/d改变档位，s/空格回中，x停止，q退出；不是松键即停。原生适配器只走USB；BLE和CSV采集用Python入口。原生程序必须收到新板INFO身份与A1应答，持续发送心跳，退出时停止并释放。

## Linux ROS 2

```sh
cd ros2_ws
colcon build --packages-select propulsion_pcb_control
source install/setup.bash
# 默认dry_run=true，不开串口：
ros2 launch propulsion_pcb_control hardware.launch.py
# 完成真机检查后，才显式允许串口：
ros2 launch propulsion_pcb_control hardware.launch.py dry_run:=false serial_port:=/dev/ttyACM0
```

需要ROS 2、joy、sensor_msgs、std_msgs和python3-serial。节点仍使用原死区、加速度及0.5秒手柄超时；新传输层增加身份检查、解锁、命令应答和200ms心跳。ROS 2运行时与真实手柄映射尚需Linux环境验证，未在这台Mac上冒充已运行ROS链路。

## 功能及使用边界

- `V`让四路同速；上组M1/M3与下组M2/M4方向相反。`M`或`W`可单独控制四路。
- 正常 `S`/`V0`仅停止STEP，保留使能与设定电流；通信超时/断线也停脉冲、保留电气保持并释放控制权。重新运动必须 `A1`，不会自动重连后续跑。
- `A0`、`E`、掉外部电源或驱动故障会关输出；此时不能承诺保持位置。`E`及驱动故障需 `CLEAR`再 `A1`，不会自行恢复。
- 四路硬件RMT发脉冲，BLE/USB不靠忙等计步；同速不是闭环同步，未提供编码器反馈，也不能保证四轴绝对位置或完全同相启动。修改速度会短暂停脉冲以满足DIR时序。
- 默认每路满量程电流设定约0.50A，可调100～600mA；是名义满量程/峰值设定，不是实测电流或RMS。先测VREF和温升，再决定是否增加。板上理论满PWM约0.716A。
- 新旧均1/64细分，1.8°电机每转12800脉冲：400步/秒约1.875rpm、3200约15rpm、16000约75rpm。导管mm/s还需滚轮有效直径及打滑标定，不能直接当实测位移。
- BLE采用NUS风格GATT服务，支持单个BLE连接；USB和BLE均可查询，一次只允许一个控制源持有运动权限。BLE是实验室原型连接，未实现身份鉴权，不作为临床或不受控场所的无线控制方案。
- CSV记录主机时间、板端运行时间、命令步频、供电选择和故障状态；没有新增力、压力、温度、实际位置/速度传感器。

验证结果及首板步骤见 `docs/验证与首板测试.md`。暂未烧录新板、未实测运动、未改动硬件工程，也未重新启用嘉立创MCP。
