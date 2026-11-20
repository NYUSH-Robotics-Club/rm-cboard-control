# Quest 双手柄经 VT13 / VT03 控制 Infantry 底盘

本目录保存 Windows 工作台、Quest WebXR 页面、MQTT 发送端、STM32F407 固件工程、实板核对过的 ELF 和离线测试。使用 2026 官方裁判端与选手端。Quest 使用浏览器，无需安装 FrankaBot。

## 数据怎样到车上

1. Quest 浏览器执行 [vr.js](studio/www/vr.js)，读取左右手柄的 WebXR 位姿、扳机与右手指向。USB 调试转发将头显的 localhost:18780 接到电脑的同一端口。
2. [server.py](studio/server.py) 接收带会话和递增序号的帧。[校准程序](studio/repository_calibration.py) 采右向、前向及原点，[vehicle.py](studio/vehicle.py) 将水平位移投影到标定轴。右扳机加右手位移控制平移。左扳机加左手向左移动控制顺时针，向右控制逆时针。松开一侧扳机，该侧回零，监听继续。
3. [发布器](studio/mqtt_vehicle.py) 每 50 ms 将三轴封装成 30 字节 VRD3，经 Protobuf CustomControl 向本机官方 MQTT 服务 127.0.0.1:3333 发布。主题 CustomControl，客户端 ID 为 1，QoS 0，retain 关闭。包内依次为标记、16 位序号、启用和诊断标志、三个小端有符号 16 位量（前、右、旋转，范围 ±100 RC 刻度），最后 16 字节保留为零。
4. 官方系统通过电脑侧 VT13 上行至车侧 VT03。VT03 串口输出裁判协议 0x0311，经 C 板 UART1 接头（MCU USART6，RX PG9，921600 baud）进入 [接收器](firmware/modules/remote/remote_control.c)。校验帧头、长度、CRC、载荷和序号后，前、右、旋转分别映射 RC ch3、ch2、ch4。物理遥控优先，保留原有超时和底盘控制。Jetson 用于构建或经 ST-Link 读取板上状态。
5. 视频走反方向。官方选手端在电脑上输出 UDP 127.0.0.1:3334 的 HEVC 分片。[视频接收器](studio/video.py) 重组、解码并生成最大 960×540 的 JPEG，浏览器拉取并显示在头显。图传画面正常与车上收到控制包需要分别验证。

## Windows 和 Quest 启动

先连接 VT03、VT13 并对频，在官方裁判端和选手端完成登录。此前使用的官方版本分别为 Engine 12.0.0.176 和 Client 12.0.0.213。官方软件和 Android platform-tools 需另行安装。先停止其他使用同一机器人 MQTT 身份的发布器；不同仓库副本拥有不同的本地锁文件。

在本目录下执行以下 PowerShell 命令。首次使用需安装 Python 3.13 并创建虚拟环境。

~~~powershell
cd studio
py -3.13 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\launch.ps1
adb devices
adb reverse tcp:18780 tcp:18780
.\start-vehicle.ps1
.\.venv\Scripts\pythonw.exe hand_monitor.py
~~~

在 Quest 浏览器打开 http://127.0.0.1:18780/vr 并点击“进入沉浸式 VR”。电脑编辑页为同一地址的根路径，校准页为 /calibration-ui。布局和校准存在电脑。点“重新校准”，每次松开再扣右扳机，依次记录右向起止、前向起止、中立原点和保存。瞄准“启用控车”按下右扳机并松开，等待界面确认。重建 XR 会话后重新校准。先退出控车，再依次运行 stop-vehicle.ps1 和 stop.ps1。

## C 板构建和验证

在 firmware 目录安装 CMake、Ninja 和 arm-none-eabi-gcc 后执行：

~~~sh
cmake -S . -B build/vt03 -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DROBOT_TYPE=infantry_standard -DVR_VTM_UART_BAUD=921600 -DCMAKE_BUILD_TYPE=Debug
cmake --build build/vt03
python tools/vr/tests/test_c_receiver.py
python tools/vr/tests/test_dual_receiver.py
~~~

后两项使用主机 C 编译器。历史实板 ELF 在 [board-dual-final.elf](studio/evidence/board-dual-final.elf)，其 SHA256 及 Flash 可加载段的读回比对在 [核对记录](studio/evidence/board-final-check.json)。部署到另一块板时须独立核对固件、串口与接线。板端可通过 Jetson 上的 tools/vr/scripts/read_board_status.py 指定匹配 ELF，经 ST-Link 读取接收序号、错误和旋转值。烧录入口和工具要求见 [固件手册](firmware/docs/quickstart.md)。

## 调试与局限

在 studio 目录执行 python -m unittest discover -s tests -q 可离线检查校准、输入和视频重组。浏览器回归需安装 Chrome、Node 和 Playwright（npm install --no-save playwright），先运行 python tests/browser_server.py，再在另一终端运行 node tests/browser.mjs。隔离服务使用 18781，不连接车辆。

实时查看 /api/status 中的手柄追踪、calibration、control、publisher、video。MQTT 发布成功仅表示本地发送成功；检验无线下发应同时观察板端序号和轴值。运动方向还需按实际轮组验证。视频解码错误为零也不能排除上游马赛克。此快照保留当前视频实现，本次发布没有重新验证画质或实车运动。
