# 第三方组件

## Three.js

网页随附 Three.js 0.170.0（r170）及 GLTFLoader、OrbitControls、BufferGeometryUtils、RoomEnvironment，采用 MIT 许可证。完整版权和许可文本保留在 [LICENSE.three](control%20code/local_dashboard/static/vendor/LICENSE.three)。来源和本地导入路径调整见 [vendor/README.md](control%20code/local_dashboard/static/vendor/README.md)。

## 环境与固件依赖

Python、NumPy、OpenCV、RealSense SDK、libusb、Arduino 框架及 PlatformIO 平台由环境或构建工具安装，其源码和安装目录不随本仓库上传，使用遵循各组件许可证。环境定义见 `control code/local_dashboard/environment-camera.yml`，固件平台见各自 `platformio.ini`。

`control code/pcb_esp32s3/releases/` 保留已有 ESP32-S3 烧录包，包括固件、引导程序和分区文件。二进制中的第三方组件遵循其上游许可；公开分发前应结合具体构建版本核对完整许可要求。
