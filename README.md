# VibeFire 电动软弹枪智能火控系统 (AI 全栈开发)

[![Platform](https://img.shields.io/badge/platform-Arduino%20Nano-blue)]()
[![Language](https://img.shields.io/badge/language-C%2B%2B-orange)]()
[![License](https://img.shields.io/badge/license-MIT-green)]()
[![Build](https://img.shields.io/badge/build-PlatformIO-passing-brightgreen)]()
[![AI](https://img.shields.io/badge/code-100%25%20AI%20generated-red)]()

> 从火控到视觉瞄准——纯 AI 开发的玩具枪电子火控与自动瞄准系统

---

## 项目简介

VibeFire 是一套完整的 DIY 电动软弹枪 (AEG) 电子火控系统。它用 Arduino Nano 替代传统机械开关，实现闭环齿轮定位、多种射击模式切换、NFC 弹匣检测、OLED 信息显示等专业功能。整套系统包含两个子系统：运行在 Arduino Nano 上的火控单元 (FCU) 和运行在 Radxa Zero 3E / RK3566 上的计算机视觉自动瞄准模块 (AutoTrigger)。

火控单元负责电机控制的核心逻辑——检测扳机动作、驱动齿轮箱、在正确位置刹停、检测弹匣状态、管理供电安全。AutoTrigger 子系统则通过摄像头实时捕捉目标，结合 YOLOv5n 目标检测、卡尔曼滤波轨迹预测、ToF 激光测距和弹道补偿算法，在瞄准点对准时自动发送击发信号。

**本项目所有代码——包括固件、C++ 工程、单元测试、文档、装配手册——均由 AI 生成。** 从需求分析到代码审查到安全审计，开发全流程由 AI 驱动。经 Oracle 安全审计多轮迭代后，代码已达到生产可用标准。

---

## 系统架构

```
+------------------------------------------------------------------+
|                        VibeFire 系统架构                           |
|                                                                   |
|  +---------------------------+    +---------------------------+   |
|  |    FCU 火控单元           |    |  AutoTrigger CV 智能瞄具  |   |
|  |    (Arduino Nano)         |    |  (Radxa Zero 3E / RK3566) |   |
|  |                           |    |                           |   |
|  |  霍尔传感器 ──┐          |    |  USB 摄像头 ──┐          |   |
|  |  按键 ────────┤          |    |               │          |   |
|  |  NFC 读卡 ────┤ 状态机   |    |  YOLOv5n ─────┤ 推理     |   |
|  |  电压采样 ────┤   →     |    |  (NPU 30FPS)  │   →      |   |
|  |               │          |    |               │          |   |
|  |  MOSFET ×2 ───┘          |    |  卡尔曼滤波 ──┤ 预测     |   |
|  |  OLED 0.96" ── 显示      |    |  ToF 激光 ────┤ 测距     |   |
|  |  EEPROM ────── 持久化    |    |  弹道模型 ────┤ 补偿     |   |
|  |  看门狗 ────── 安全      |    |               │          |   |
|  |                           |    |  圆形瞄具 ← OLED ← 瞄准    |   |
|  +------------+--------------+    +-------+------+-----------+   |
|               |                           |                      |
|               |   触发信号 (GPIO)          |                      |
|               +<--------------------------+                      |
|                                                                   |
|  数据流:  摄像头 → 检测 → 跟踪 → 判准 → 击发 → 电机 → 齿轮归位    |
+------------------------------------------------------------------+
```

---

## 功能特性

### FCU 火控单元

| 功能 | 说明 |
|------|------|
| **5 种射击模式** | SAFE / SEMI / BINARY / BURST(2-10 可调) / AUTO |
| **闭环齿轮定位** | 霍尔传感器 + ISR 中断，精确检测齿轮归位信号 |
| **主动刹车** | 双 MOSFET 驱动，带死区互锁保护，防止 H 桥短路 |
| **OLED 信息显示** | SSD1306 0.96" I2C OLED，三按键菜单系统显示模式/弹药/电压/爆弹计数 |
| **NFC 弹匣检测** | MFRC522 读卡模块，换弹匣自动重置弹药计数 |
| **预压延迟** | 单发模式击发前延迟 (0-100ms 可调)，模拟真铁连动机构 |
| **电池电压监测** | 支持 6S LiPo (22.2V)，低电压自动切断保护 |
| **堵转保护** | 2.5 秒超时自动停机，防止齿轮箱损坏 |
| **看门狗定时器** | AVR 硬件看门狗，程序跑飞自动复位 |
| **EEPROM 持久化** | 所有设置掉电保存，含 CRC 校验 |
| **Wokwi 在线仿真** | 完整电路仿真，无需硬件即可测试 |
| **浏览器模拟器** | HTML 交互式火控模拟器，可视化操作逻辑 |

### AutoTrigger CV 智能瞄具

| 功能 | 说明 |
|------|------|
| **AI 目标检测** | YOLOv5n 在 RK3566 NPU 上推理，30 FPS 实时检测 |
| **轨迹预测** | 卡尔曼滤波器跟踪目标运动轨迹，预估未来位置 |
| **激光测距** | JRT TOF01 ToF 传感器，最远测距 40 米 |
| **弹道补偿** | 基于阻力模型的弹道下坠补偿计算 |
| **瞄准反馈** | 圆形 OLED 瞄具，准心对准目标时自动触发器 |
| **安全机制** | 多级安全校验，防误触发 |

---

## 快速开始

### 环境要求

- [PlatformIO](https://platformio.org/) (推荐通过 `pip install platformio` 安装)
- Arduino Nano (ATmega328P) 开发板
- USB 转串口下载器 (CH340 / CP2102)

### 编译与烧录

```bash
# 1. 克隆仓库
git clone https://github.com/OnlyAnIronFW/VibeFire-Airsoft-AEG-ETU-made-with-only-AI.git
cd VibeFire-Airsoft-AEG-ETU-made-with-only-AI

# 2. 编译
pio run

# 3. 烧录到 Arduino Nano
pio run -t upload

# 4. 打开串口监视器 (115200 波特率)
pio device monitor -b 115200
```

### 在线体验

- **Wokwi 仿真**: 将 `wokwi/` 目录下的文件导入 [Wokwi.com](https://wokwi.com)，无需硬件即可运行完整仿真。
- **浏览器模拟器**: 在任意浏览器中打开 `sim/fcu_simulator.html`，体验火控操作逻辑。

---

## 项目结构

```
VibeFire/
├── src/                          # FCU 固件 (PlatformIO 构建目标)
│   └── fire_control.ino          # 主固件 (~960 行)
├── firmware/                     # 固件副本
│   └── fire_control.ino
├── wokwi/                        # Wokwi 在线仿真
│   ├── fire_control.ino          # 仿真版固件 (无 NFC)
│   ├── diagram.json              # Wokwi 电路图配置
│   └── libraries.txt             # Wokwi 依赖库列表
├── sim/                          # 浏览器火控模拟器
│   └── fcu_simulator.html        # 交互式火控模拟器
├── docs/                         # 项目文档
│   ├── FCU装配手册.md             # 完整装配手册 (911 行)
│   ├── 火控设计文档.md             # 硬件电路设计文档
│   └── 接线图与装配指南.md         # 接线图与分步装配指南
├── AutoTrigger/                  # CV 自动瞄准子系统 (RK3566)
│   ├── include/autotrigger/      # C++ 头文件 (10+ 模块)
│   │   ├── ballistics.h          # 弹道模型
│   │   ├── kalman_filter.h       # 卡尔曼滤波器
│   │   ├── pipeline.h            # 图像处理流水线
│   │   ├── yolo_infer.h          # YOLOv5n 推理封装
│   │   ├── ranging.h             # 激光测距驱动
│   │   ├── trigger.h             # 触发判定逻辑
│   │   ├── display.h             # OLED 瞄具显示
│   │   ├── safety.h              # 安全控制
│   │   ├── calibrate.h           # 瞄准校准
│   │   └── hal/                  # 硬件抽象层
│   ├── src/                      # C++ 源文件 (10 模块)
│   │   ├── main.cpp
│   │   ├── ballistics.cpp
│   │   ├── kalman_filter.cpp
│   │   ├── pipeline.cpp
│   │   ├── yolo_infer.cpp
│   │   ├── ranging.cpp
│   │   ├── trigger.cpp
│   │   ├── display.cpp
│   │   ├── safety.cpp
│   │   └── calibrate.cpp
│   ├── tests/                    # 单元测试 (Catch2, 65+ 用例)
│   │   ├── test_sanity.cpp
│   │   ├── test_ballistics.cpp
│   │   ├── test_kalman.cpp
│   │   ├── test_pipeline.cpp
│   │   ├── test_yolo.cpp
│   │   ├── test_ranging.cpp
│   │   ├── test_trigger.cpp
│   │   ├── test_display.cpp
│   │   ├── test_safety.cpp
│   │   └── CMakeLists.txt
│   └── docs/                     # AutoTrigger 文档
│       └── ASSEMBLY_GUIDE.md     # 装配指南
├── platformio.ini                # PlatformIO 项目配置
├── LICENSE                       # MIT 许可证
└── README.md                     # 本文件
```

---

## 硬件需求

### FCU 火控单元

| 组件 | 规格 | 数量 | 预估价格 |
|------|------|------|---------|
| 主控 | Arduino Nano (ATmega328P) | 1 | ¥12 |
| OLED 屏幕 | SSD1306 0.96" 128×64 I2C | 1 | ¥8 |
| 驱动 MOS | IRLB3034 N-MOSFET | 2 | ¥6 |
| 降压模块 | Mini-360 DC-DC (23V → 5V) | 1 | ¥3 |
| 霍尔传感器 | 897 霍尔开关模块 | 3 | ¥6 |
| NFC 模块 | MFRC522 + S50 标签 | 1 | ¥8 |
| 按钮 | 6×6mm 微动开关 | 3 | ¥2 |
| 电阻 | 10kΩ, 100kΩ, 1MΩ | 若干 | ¥2 |
| 电容 | 100μF 25V, 0.1μF | 若干 | ¥3 |
| 二极管 | 1N5819 肖特基 | 2 | ¥2 |
| 排针/杜邦线 | - | 若干 | ¥10 |
| 万用板 | 5×7cm | 1 | ¥5 |
| 电池 | 6S LiPo (22.2V) | 1 | ¥45 |

> **总成本**: 约 ¥112 / $16（不含电池约 ¥67）

完整 BOM 清单与采购链接请参阅 [FCU 装配手册](docs/FCU装配手册.md)。

---

## 火控模式说明

| 模式 | 英文 | 行为 | 适用场景 |
|------|------|------|----------|
| 保险 | SAFE | 扳机锁定，电机不响应 | 日常安全、携带收纳 |
| 单发 | SEMI | 扣一次扳机击发一发，松手后才可再次击发 | 竞技对抗、精准射击 |
| 双行程 | BINARY | 扣下击发第 1 发，松开击发第 2 发 | 快速双发、CQB 近战 |
| x 连发 | BURST | 扣一次扳机击发 N 发 (N 可调 2-10) | 战术点射、节省弹药 |
| 连发 | AUTO | 扣住扳机连续击发，松手或弹尽停止 | 火力压制、娱乐 |

模式切换通过快慢机旋钮上安装的霍尔传感器实现。快慢机在不同角度触发不同的霍尔编码组合，Arduino 轮询读取后切换状态机。

---

## 📖 文档

| 文档 | 说明 |
|------|------|
| [完整装配手册](docs/FCU装配手册.md) | 含完整 BOM、焊接指南、校准步骤 |
| [火控设计文档](docs/火控设计文档.md) | 电路设计、架构设计、引脚分配 |
| [接线图与装配指南](docs/接线图与装配指南.md) | 分步接线图与安装指引 |
| [AutoTrigger 装配指南](AutoTrigger/docs/ASSEMBLY_GUIDE.md) | CV 子系统硬件装配说明 |

---

## 在线体验

### Wokwi 在线仿真

Wokwi 仿真是无需硬件的快速测试方式。仿真中：
- 按钮替代扳机/菜单操作
- 开关模拟霍尔传感器归位/快慢机信号
- LED 指示电机/刹车状态
- 电位器模拟电池电压
- NFC 不可用（手动设弹量替代）

操作步骤见 [wokwi/README.md](wokwi/README.md)。

### 浏览器模拟器

打开 `sim/fcu_simulator.html` 即可在浏览器中交互体验火控操作逻辑，包含完整的 OLED 显示、按键操作和状态切换。

---

## 🤖 AI 开发说明

本项目是 **100% AI 生成** 的完整嵌入式系统工程，代码、文档、测试均由 AI 完成。

### 开发流程

```
需求分析 → 架构设计 → 模块实现 → 单元测试
    ↓           ↓           ↓           ↓
 AI 探索     AI 规划     AI 编码     AI 测试
    ↓           ↓           ↓           ↓
 代码审查 ← 安全审计 ← QA 验证 ← 集成测试
    ↓
 发布就绪
```

### 关键里程碑

- **硬件抽象与状态机**：从空的 `fire_control.ino` 开始，AI 逐模块实现引脚定义、中断处理、状态机、电机控制、显示驱动、NFC 通信
- **安全审计修复**：Oracle 安全代理发现并修复了看门狗定时器遗漏、ISR 原子性保护不足、BURST 模式低电压保护缺失、LED 引脚与 SPI 冲突等关键缺陷
- **自动化测试**：AutoTrigger 子系统实现 65+ Catch2 单元测试用例，覆盖弹道计算、卡尔曼滤波、触发逻辑、安全控制等核心模块
- **文档生成**：三份中文装配/设计文档从零生成，内容详实，直接可用
- **双平台模拟**：Wokwi 硬件仿真 + 浏览器 HTML 模拟器，两条验证路径

开发过程中的详细思路、prompt 策略和技术细节，请参阅各子目录下的 `CLAUDE.md` / `AGENTS.md` 和对话记录。

---

## 安全声明

> ⚠️ **重要安全警告**

- **锂电池安全**：6S LiPo 电池能量密度极高。必须使用平衡充电器，过充、过放、短路均可能引发火灾。充电时须有人值守，存放于防火安全袋中。
- **眼部保护**：操作测试时务必佩戴护目镜。软弹即使速度不高，击中眼部仍可能造成严重伤害。
- **绝对禁止指向人或动物**：无论是否上膛、是否通电，任何时候不得将发射器指向人或动物。
- **法律合规**：请查阅当地法律。部分地区对仿真枪外观、动能、使用场所有严格限制。使用者须自行承担法律责任。
- **MOSFET 死区保护**：本系统使用双 MOSFET 驱动方案。改动固件时务必确保死区互锁逻辑不被破坏，否则可能直接短路电池造成严重事故。
- **堵转风险**：机械卡死时电机堵转电流极大。固件内置 2.5 秒堵转保护，请勿禁用该功能。

本项目仅供学习研究与合法竞技使用。作者不对因使用本项目造成的任何人身伤害、财产损失或法律后果承担责任。

---

## License

[MIT](LICENSE)

Copyright (c) 2025 VibeFire Contributors
