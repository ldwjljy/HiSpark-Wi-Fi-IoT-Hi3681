# HiSpark Wi-Fi IoT 智能小车 Demo

一个面向展示和教学的 HiSpark Wi-Fi IoT 智能小车示例工程，基于 OpenHarmony LiteOS-M / Hi3861。项目把小车常用能力收敛成一个干净的正式 Demo：手机网页遥控、模式切换、黑线循迹、超声波测距/避障、OLED 状态显示和 Wi-Fi 热点访问。

## 功能亮点

- **手机网页控制**：连接小车热点后，浏览器打开控制台即可使用方向按键遥控。
- **三种运行模式**：停止、循迹、避障，可在网页或 OLED 按键中切换。
- **黑线循迹**：适配宽黑线闭环赛道，默认按“遇白亮灯、遇黑熄灭”的循迹模块状态识别黑线。
- **超声波测距**：实时显示前方距离，并保留五方向距离数据展示。
- **超声波避障**：低于阈值时停车、后退、左右扫描并选择方向绕行。
- **OLED 信息页**：显示当前模式、距离、速度、电量、Wi-Fi SSID、Password、访问地址和保护状态。
- **正式展示网页**：页面只保留演示需要的核心内容：按键控制、模式切换、方向距离和连接信息。

## 连接信息

小车启动后会开启 Wi-Fi 热点：

| 项目 | 内容 |
| --- | --- |
| SSID | `HMZXYYDS` |
| Password | `HMZXYYDS` |
| 控制台地址 | `http://192.168.5.1/` |

手机连接热点后，使用浏览器打开 `http://192.168.5.1/` 即可进入控制台。

## 目录结构

```text
.
├── firmware/
│   ├── OHOS_Image.bin                  # 推荐烧录镜像
│   └── Hi3861_wifiiot_app_burn.bin     # 应用烧录镜像
├── src/
│   ├── BUILD.gn                        # wifi-iot app 入口构建配置
│   └── oled_demo/
│       ├── BUILD.gn                    # Demo 模块构建配置
│       └── oled_demo.c                 # Demo 主程序
└── docs/
    └── build_demo.log                  # 本次构建日志
```

## 快速烧录

优先使用：

```text
firmware/OHOS_Image.bin
```

也可以根据你的烧录工具流程选择：

```text
firmware/Hi3861_wifiiot_app_burn.bin
```

烧录完成后重启开发板，等待 OLED 显示启动信息，然后手机连接 `HMZXYYDS` 热点。

## 集成到 OpenHarmony 源码

如果需要重新编译，把源码复制到 OpenHarmony 工程：

```bash
cp src/BUILD.gn ~/openharmony/applications/sample/wifi-iot/app/BUILD.gn
cp src/oled_demo/BUILD.gn ~/openharmony/applications/sample/wifi-iot/app/oled_demo/BUILD.gn
cp src/oled_demo/oled_demo.c ~/openharmony/applications/sample/wifi-iot/app/oled_demo/oled_demo.c
```

然后编译：

```bash
cd ~/openharmony
./build.sh --product-name wifiiot_hispark_pegasus --ccache --no-prebuilt-sdk
```

编译产物通常位于：

```text
out/hispark_pegasus/wifiiot_hispark_pegasus/OHOS_Image.bin
out/hispark_pegasus/wifiiot_hispark_pegasus/Hi3861_wifiiot_app_burn.bin
```

## 操作说明

### 网页端

1. 手机连接热点 `HMZXYYDS`。
2. 浏览器打开 `http://192.168.5.1/`。
3. 使用方向按键进行遥控。
4. 点击“停止 / 循迹 / 避障”切换运行模式。
5. 查看前方距离、方向距离、当前模式和连接状态。

### OLED 按键

- `S1`：切换小车运行模式。
- `S2`：切换参数页。
- 参数页会显示当前模式、Wi-Fi 状态、SSID、Password 和网站地址。

## 主要硬件引脚

| 功能 | GPIO |
| --- | --- |
| 左电机 A/B | GPIO0 / GPIO1 |
| 舵机 | GPIO2 |
| 超声波 TRIG / ECHO | GPIO7 / GPIO8 |
| 右电机 A/B | GPIO9 / GPIO10 |
| 左/右循迹 | GPIO11 / GPIO12 |
| OLED SDA / SCL | GPIO13 / GPIO14 |

## HTTP 接口

| 接口 | 说明 |
| --- | --- |
| `/` | 手机控制台页面 |
| `/api/status` | 状态 JSON：模式、距离、速度、Wi-Fi、方向距离等 |
| `/cmd?move=forward` | 前进 |
| `/cmd?move=backward` | 后退 |
| `/cmd?move=left` | 左转 |
| `/cmd?move=right` | 右转 |
| `/cmd?move=stop` | 停止 |
| `/mode?set=stop` | 停止模式 |
| `/mode?set=trace` | 循迹模式 |
| `/mode?set=obstacle` | 避障模式 |

## 展示建议

- 准备一条宽黑线闭环赛道。
- 小车放在线上后，先进入“停止模式”，确认网页能看到距离和连接状态。
- 切换到“循迹模式”展示自动巡线。
- 在前方放置障碍物，切换到“避障模式”展示超声波避障。
- 需要人工演示时，回到“停止模式”或使用网页方向按键遥控。

## 版本说明

这是一个正式展示用 Demo 版本，目标是稳定、直观、容易复现。网页不展示调参和校准等开发功能，只保留面向观众的核心体验。
