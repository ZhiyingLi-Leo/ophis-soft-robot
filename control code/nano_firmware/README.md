# Nano 推进器固件

适用 ATmega168 + CH340C Nano 兼容板，四电机共用脉冲、上下分组方向。源码位于 `arduino/propulsion_controller/propulsion_controller.ino`，PlatformIO 环境为 `nano_168`。此固件不能实现四电机独立速度；独立控制使用 ESP32-S3 PCB 固件。

## 测试与编译

在本目录执行：

```sh
make test
~/.platformio/penv/bin/pio run --environment nano_168
```

`make test` 在本机验证固件引脚逻辑，不打开串口；PlatformIO 命令编译板端固件。

## 烧录

操作者确认开发板与实际端口后执行，烧录会复位开发板：

```sh
~/.platformio/penv/bin/pio device list
~/.platformio/penv/bin/pio run --environment nano_168 --target upload --upload-port /dev/cu.usbserial-实际端口
```

配置文件保留原板型与参数，其中端口为历史值 `/dev/cu.usbserial-1120`，烧录时用实际端口覆盖。板型是 ATmega168，不是 ATmega328P。

## 协议与停止

115200 baud、8N1。`V<带符号步频>` 设置四电机统一速度，`V0` 或 `S` 停止，`?` 查询。D8 为公共脉冲负端，D9 为公共脉冲正端，D10 为公共方向负端，D11/D12 分别控制下组/上组方向。

固件没有通信超时停车功能；主机崩溃、强制退出或物理断线后不能保证停止，实物必须保留独立停机手段。停止脉冲不切断驱动板保持电流，测试结束需关闭 24 V。
